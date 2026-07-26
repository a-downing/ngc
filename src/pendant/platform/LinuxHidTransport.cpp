#include "pendant/HidTransport.h"

#include "platform/HidReportDescriptor.h"

#include <hidapi/hidapi.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <mutex>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace ngc::pendant {
    namespace {
        constexpr std::chrono::milliseconds CANCELLATION_POLL_PERIOD { 25 };

        HidErrorCode classifyError(const int code) noexcept {
            if (code == EACCES || code == EPERM) {
                return HidErrorCode::AccessDenied;
            }
            if (code == ECANCELED || code == EINTR) {
                return HidErrorCode::Cancelled;
            }
            if (code == ENOENT || code == ENODEV || code == ENXIO || code == EPIPE || code == EBADF
                || code == ESHUTDOWN || code == EIO) {
                return HidErrorCode::Disconnected;
            }
            return HidErrorCode::IoFailure;
        }

        std::string narrowError(const wchar_t *message) {
            if (!message) {
                return {};
            }
            std::string result;
            while (*message != L'\0') {
                const auto value = static_cast<std::uint32_t>(*message++);
                result.push_back(value <= 0x7f ? static_cast<char>(value) : '?');
            }
            return result;
        }

        HidError linuxError(const std::string_view operation, const int code,
                            const wchar_t *hidMessage) {
            auto detail = narrowError(hidMessage);
            if (detail.empty() && code != 0) {
                detail = std::system_category().message(code);
            }
            auto message = std::string(operation);
            if (!detail.empty()) {
                message += ": " + detail;
            }
            return {
                classifyError(code), static_cast<std::uint32_t>(std::max(0, code)),
                std::move(message),
            };
        }

        HidError cancelledError() {
            return {
                HidErrorCode::Cancelled, static_cast<std::uint32_t>(ECANCELED),
                "HID operation was cancelled",
            };
        }

        class Enumeration {
        public:
            explicit Enumeration(hid_device_info *devices) : m_devices(devices) { }
            ~Enumeration() { hid_free_enumeration(m_devices); }
            Enumeration(const Enumeration &) = delete;
            Enumeration &operator=(const Enumeration &) = delete;

            hid_device_info *get() const noexcept { return m_devices; }

        private:
            hid_device_info *m_devices = nullptr;
        };

        class Device {
        public:
            explicit Device(hid_device *device) : m_device(device) { }
            ~Device() {
                if (m_device) {
                    hid_close(m_device);
                }
            }
            Device(const Device &) = delete;
            Device &operator=(const Device &) = delete;
            Device(Device &&other) noexcept : m_device(std::exchange(other.m_device, nullptr)) { }
            Device &operator=(Device &&other) noexcept {
                if (this != &other) {
                    if (m_device) {
                        hid_close(m_device);
                    }
                    m_device = std::exchange(other.m_device, nullptr);
                }
                return *this;
            }

            hid_device *get() const noexcept { return m_device; }

        private:
            hid_device *m_device = nullptr;
        };

        class LinuxHidTransport final : public HidTransport {
        public:
            LinuxHidTransport(Device device, const hid_detail::HidApiReportLengths lengths)
                : m_device(std::move(device)), m_inputLength(lengths.input),
                  m_outputLength(lengths.output) { }

            ~LinuxHidTransport() override { cancel(); }

            std::size_t inputReportLength() const noexcept override { return m_inputLength; }
            std::size_t outputReportLength() const noexcept override { return m_outputLength; }

            std::expected<std::size_t, HidError>
            readInputReport(const std::span<std::uint8_t> report,
                            const std::chrono::milliseconds timeout) override {
                if (report.size() != m_inputLength) {
                    return std::unexpected(HidError {
                        HidErrorCode::InvalidReport, 0,
                        "input buffer does not match the HID report length",
                    });
                }

                using clock = std::chrono::steady_clock;
                const auto finite = timeout != std::chrono::milliseconds::max();
                const auto duration = std::max(timeout, std::chrono::milliseconds::zero());
                const auto now = clock::now();
                const auto maximumDuration = std::chrono::duration_cast<std::chrono::milliseconds>(
                    clock::time_point::max() - now);
                const auto deadline = !finite || duration >= maximumDuration
                    ? clock::time_point::max() : now + duration;
                for (;;) {
                    if (m_cancelled.load(std::memory_order_acquire)) {
                        return std::unexpected(cancelledError());
                    }
                    while (m_outputPending.load(std::memory_order_acquire)) {
                        m_outputPending.wait(true, std::memory_order_acquire);
                    }

                    auto wait = CANCELLATION_POLL_PERIOD;
                    if (finite) {
                        const auto remaining = deadline - clock::now();
                        if (remaining <= clock::duration::zero()) {
                            wait = std::chrono::milliseconds::zero();
                        } else {
                            wait = std::min(wait, std::chrono::ceil<std::chrono::milliseconds>(remaining));
                        }
                    }

                    int count = 0;
                    HidError failure;
                    {
                        std::scoped_lock lock(m_ioMutex);
                        if (m_cancelled.load(std::memory_order_acquire)) {
                            return std::unexpected(cancelledError());
                        }
                        errno = 0;
                        count = hid_read_timeout(
                            m_device.get(), report.data(), report.size(),
                            static_cast<int>(wait.count()));
                        const auto code = errno;
                        if (count < 0) {
                            failure = linuxError("HID input report failed", code,
                                                 hid_error(m_device.get()));
                        }
                    }
                    if (count > 0) {
                        return static_cast<std::size_t>(count);
                    }
                    if (count < 0) {
                        if (m_cancelled.load(std::memory_order_acquire)) {
                            return std::unexpected(cancelledError());
                        }
                        return std::unexpected(std::move(failure));
                    }
                    if (m_cancelled.load(std::memory_order_acquire)) {
                        return std::unexpected(cancelledError());
                    }
                    if (finite && clock::now() >= deadline) {
                        return std::unexpected(HidError {
                            HidErrorCode::TimedOut, static_cast<std::uint32_t>(ETIMEDOUT),
                            "timed out waiting for a HID input report",
                        });
                    }
                }
            }

            std::expected<void, HidError>
            writeOutputReport(const std::span<const std::uint8_t> report) override {
                if (report.size() != m_outputLength) {
                    return std::unexpected(HidError {
                        HidErrorCode::InvalidReport, 0,
                        "output buffer does not match the HID report length",
                    });
                }
                if (m_cancelled.load(std::memory_order_acquire)) {
                    return std::unexpected(cancelledError());
                }

                m_outputPending.store(true, std::memory_order_release);
                auto result = [&]() -> std::expected<void, HidError> {
                    std::scoped_lock lock(m_ioMutex);
                    if (m_cancelled.load(std::memory_order_acquire)) {
                        return std::unexpected(cancelledError());
                    }
                    errno = 0;
                    const auto streamCount =
                        hid_write(m_device.get(), report.data(), report.size());
                    const auto streamCode = errno;
                    if (streamCount < 0) {
                        return std::unexpected(linuxError(
                            "HID output stream write failed", streamCode,
                            hid_error(m_device.get())));
                    }
                    if (static_cast<std::size_t>(streamCount) != report.size()) {
                        return std::unexpected(HidError {
                            HidErrorCode::IoFailure, 0,
                            "HID output report was only partially written",
                        });
                    }

                    // Match the device's working Windows sequence: its LCD applies
                    // the control output report, while the stream write alone is ignored.
                    errno = 0;
                    const auto outputCount =
                        hid_send_output_report(m_device.get(), report.data(), report.size());
                    const auto outputCode = errno;
                    if (outputCount < 0) {
                        return std::unexpected(linuxError(
                            "HID output report failed", outputCode, hid_error(m_device.get())));
                    }
                    if (static_cast<std::size_t>(outputCount) != report.size()) {
                        return std::unexpected(HidError {
                            HidErrorCode::IoFailure, 0,
                            "HID output report was only partially sent",
                        });
                    }

                    return {};
                }();
                m_outputPending.store(false, std::memory_order_release);
                m_outputPending.notify_all();
                return result;
            }

            void cancel() noexcept override {
                m_cancelled.store(true, std::memory_order_release);
            }

        private:
            Device m_device;
            std::size_t m_inputLength = 0;
            std::size_t m_outputLength = 0;
            std::atomic<bool> m_cancelled { false };
            std::atomic<bool> m_outputPending { false };
            std::mutex m_ioMutex;
        };

        std::expected<std::vector<std::string>, HidError>
        matchingDevicePaths(const HidDeviceSelector &selector) {
            errno = 0;
            Enumeration devices(hid_enumerate(selector.vendorId, selector.productId));
            const auto code = errno;
            if (!devices.get() && code != 0) {
                return std::unexpected(linuxError(
                    "could not enumerate HID devices", code, hid_error(nullptr)));
            }

            std::vector<std::string> matches;
            for (auto *device = devices.get(); device; device = device->next) {
                if (device->path) {
                    matches.emplace_back(device->path);
                }
            }
            return matches;
        }
    }

    std::expected<std::unique_ptr<HidTransport>, HidError>
    openHidTransport(const HidDeviceSelector &selector) {
        errno = 0;
        if (hid_init() != 0) {
            return std::unexpected(linuxError(
                "could not initialize HIDAPI", errno, hid_error(nullptr)));
        }

        auto matches = matchingDevicePaths(selector);
        if (!matches) {
            return std::unexpected(std::move(matches.error()));
        }
        if (matches->empty()) {
            return std::unexpected(HidError {
                HidErrorCode::NotFound, 0, "matching HID device was not found",
            });
        }
        if (matches->size() != 1) {
            return std::unexpected(HidError {
                HidErrorCode::MultipleMatches, 0,
                "more than one matching HID device was found",
            });
        }

        errno = 0;
        Device device(hid_open_path(matches->front().c_str()));
        const auto openCode = errno;
        if (!device.get()) {
            return std::unexpected(linuxError(
                "could not open matching HID device", openCode, hid_error(nullptr)));
        }

        std::array<std::uint8_t, HID_API_MAX_REPORT_DESCRIPTOR_SIZE> descriptor{};
        errno = 0;
        const auto descriptorLength = hid_get_report_descriptor(
            device.get(), descriptor.data(), descriptor.size());
        const auto descriptorCode = errno;
        if (descriptorLength < 0) {
            return std::unexpected(linuxError(
                "could not read HID report descriptor", descriptorCode, hid_error(device.get())));
        }
        auto lengths = hid_detail::hidApiReportLengths(
            std::span<const std::uint8_t>(
                descriptor.data(), static_cast<std::size_t>(descriptorLength)));
        if (!lengths) {
            return std::unexpected(HidError {
                HidErrorCode::InvalidReport, 0,
                "could not decode HID report descriptor: " + std::move(lengths.error()),
            });
        }

        return std::unique_ptr<HidTransport>(
            new LinuxHidTransport(std::move(device), *lengths));
    }
}
