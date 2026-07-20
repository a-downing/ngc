#define NOMINMAX
#include <windows.h>
#include <hidsdi.h>
#include <setupapi.h>

#include "pendant/HidTransport.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <limits>
#include <system_error>
#include <utility>
#include <vector>

namespace ngc::pendant {
    namespace {
        HidErrorCode classifyError(const DWORD code) noexcept {
            if(code == ERROR_ACCESS_DENIED) return HidErrorCode::AccessDenied;
            if(code == ERROR_OPERATION_ABORTED) return HidErrorCode::Cancelled;
            if(code == ERROR_DEVICE_NOT_CONNECTED || code == ERROR_INVALID_HANDLE
               || code == ERROR_GEN_FAILURE)
                return HidErrorCode::Disconnected;
            return HidErrorCode::IoFailure;
        }

        HidError windowsError(const std::string_view operation, const DWORD code = GetLastError()) {
            return {
                classifyError(code), code,
                std::string(operation) + ": " + std::system_category().message(static_cast<int>(code)),
            };
        }

        DWORD windowsTimeout(const std::chrono::milliseconds timeout) noexcept {
            if(timeout == std::chrono::milliseconds::max()) return INFINITE;
            if(timeout.count() <= 0) return 0;
            constexpr auto MAXIMUM_FINITE_TIMEOUT = static_cast<std::uint64_t>(INFINITE) - 1;
            return static_cast<DWORD>(std::min(
                static_cast<std::uint64_t>(timeout.count()), MAXIMUM_FINITE_TIMEOUT));
        }

        class Handle {
        public:
            Handle() = default;
            explicit Handle(HANDLE value) : m_value(value) { }
            ~Handle() { reset(); }
            Handle(const Handle &) = delete;
            Handle &operator=(const Handle &) = delete;
            Handle(Handle &&other) noexcept : m_value(std::exchange(other.m_value, INVALID_HANDLE_VALUE)) { }
            Handle &operator=(Handle &&other) noexcept {
                if(this != &other) {
                    reset();
                    m_value = std::exchange(other.m_value, INVALID_HANDLE_VALUE);
                }
                return *this;
            }
            HANDLE get() const noexcept { return m_value; }
            bool valid() const noexcept { return m_value != nullptr && m_value != INVALID_HANDLE_VALUE; }
            HANDLE release() noexcept { return std::exchange(m_value, INVALID_HANDLE_VALUE); }
            void reset(HANDLE value = INVALID_HANDLE_VALUE) noexcept {
                if(valid()) CloseHandle(m_value);
                m_value = value;
            }

        private:
            HANDLE m_value = INVALID_HANDLE_VALUE;
        };

        class DeviceInfoSet {
        public:
            explicit DeviceInfoSet(const HDEVINFO value) : m_value(value) { }
            ~DeviceInfoSet() {
                if(m_value != INVALID_HANDLE_VALUE) SetupDiDestroyDeviceInfoList(m_value);
            }
            DeviceInfoSet(const DeviceInfoSet &) = delete;
            DeviceInfoSet &operator=(const DeviceInfoSet &) = delete;
            HDEVINFO get() const noexcept { return m_value; }

        private:
            HDEVINFO m_value = INVALID_HANDLE_VALUE;
        };

        class WindowsHidTransport final : public HidTransport {
        public:
            WindowsHidTransport(Handle inputDevice, Handle outputDevice, Handle cancellation,
                                const std::size_t inputLength, const std::size_t outputLength)
                : m_inputDevice(std::move(inputDevice)), m_outputDevice(std::move(outputDevice)),
                  m_cancellation(std::move(cancellation)),
                  m_inputLength(inputLength), m_outputLength(outputLength) { }

            ~WindowsHidTransport() override { cancel(); }

            std::size_t inputReportLength() const noexcept override { return m_inputLength; }
            std::size_t outputReportLength() const noexcept override { return m_outputLength; }

            std::expected<std::size_t, HidError>
            readInputReport(const std::span<std::uint8_t> report,
                            const std::chrono::milliseconds timeout) override {
                if(report.size() != m_inputLength)
                    return std::unexpected(HidError { HidErrorCode::InvalidReport, 0,
                                                      "input buffer does not match the HID report length" });
                auto result = transfer(false, report.data(), report.size(), windowsTimeout(timeout));
                if(!result) return std::unexpected(std::move(result.error()));
                return *result;
            }

            std::expected<void, HidError>
            writeOutputReport(const std::span<const std::uint8_t> report) override {
                if(report.size() != m_outputLength)
                    return std::unexpected(HidError { HidErrorCode::InvalidReport, 0,
                                                      "output buffer does not match the HID report length" });
                auto streamWrite = transfer(
                    true, const_cast<std::uint8_t *>(report.data()), report.size(), INFINITE);
                if(!streamWrite) return std::unexpected(std::move(streamWrite.error()));
                if(*streamWrite != report.size())
                    return std::unexpected(HidError { HidErrorCode::IoFailure, 0,
                                                      "HID output report was only partially written" });
                // Retain the control transfer used by the observed working
                // Windows sequence. The P2-S driver filters the device's output
                // echoes before any bytes reach control decoding.
                if(HidD_SetOutputReport(
                    m_outputDevice.get(), const_cast<std::uint8_t *>(report.data()),
                    static_cast<ULONG>(report.size())) != FALSE) return {};
                return std::unexpected(windowsError("HID output report failed"));
            }

            void cancel() noexcept override {
                if(m_cancellation.valid()) SetEvent(m_cancellation.get());
                if(m_inputDevice.valid()) CancelIoEx(m_inputDevice.get(), nullptr);
                if(m_outputDevice.valid()) CancelIoEx(m_outputDevice.get(), nullptr);
            }

        private:
            std::expected<std::size_t, HidError>
            transfer(const bool write, std::uint8_t *data, const std::size_t size,
                     const DWORD timeout) {
                if(WaitForSingleObject(m_cancellation.get(), 0) == WAIT_OBJECT_0)
                    return std::unexpected(HidError { HidErrorCode::Cancelled, ERROR_OPERATION_ABORTED,
                                                      "HID operation was cancelled" });
                Handle completed(CreateEventW(nullptr, TRUE, FALSE, nullptr));
                if(!completed.valid()) return std::unexpected(windowsError("could not create HID I/O event"));
                OVERLAPPED operation{};
                operation.hEvent = completed.get();
                const auto device = write ? m_outputDevice.get() : m_inputDevice.get();
                DWORD count = 0;
                const auto started = write
                    ? WriteFile(device, data, static_cast<DWORD>(size), &count, &operation)
                    : ReadFile(device, data, static_cast<DWORD>(size), &count, &operation);
                if(started) return static_cast<std::size_t>(count);
                const auto startError = GetLastError();
                if(startError != ERROR_IO_PENDING)
                    return std::unexpected(windowsError(write ? "HID write failed" : "HID read failed", startError));

                const std::array events { completed.get(), m_cancellation.get() };
                const auto wait = WaitForMultipleObjects(
                    static_cast<DWORD>(events.size()), events.data(), FALSE, timeout);
                if(wait == WAIT_TIMEOUT) {
                    CancelIoEx(device, &operation);
                    GetOverlappedResult(device, &operation, &count, TRUE);
                    return std::unexpected(HidError {
                        HidErrorCode::TimedOut, WAIT_TIMEOUT,
                        "timed out waiting for a HID input report",
                    });
                }
                if(wait == WAIT_OBJECT_0 + 1) {
                    CancelIoEx(device, &operation);
                    GetOverlappedResult(device, &operation, &count, TRUE);
                    return std::unexpected(HidError { HidErrorCode::Cancelled, ERROR_OPERATION_ABORTED,
                                                      "HID operation was cancelled" });
                }
                if(wait != WAIT_OBJECT_0) {
                    const auto waitError = GetLastError();
                    CancelIoEx(device, &operation);
                    GetOverlappedResult(device, &operation, &count, TRUE);
                    return std::unexpected(windowsError("waiting for HID I/O failed", waitError));
                }
                if(!GetOverlappedResult(device, &operation, &count, FALSE))
                    return std::unexpected(windowsError(write ? "HID write completion failed"
                                                              : "HID read completion failed"));
                return static_cast<std::size_t>(count);
            }

            Handle m_inputDevice;
            Handle m_outputDevice;
            Handle m_cancellation;
            std::size_t m_inputLength = 0;
            std::size_t m_outputLength = 0;
        };

        std::expected<std::vector<std::wstring>, HidError>
        matchingDevicePaths(const HidDeviceSelector &selector) {
            GUID hidGuid{};
            HidD_GetHidGuid(&hidGuid);
            DeviceInfoSet devices(SetupDiGetClassDevsW(
                &hidGuid, nullptr, nullptr, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE));
            if(devices.get() == INVALID_HANDLE_VALUE)
                return std::unexpected(windowsError("could not enumerate HID devices"));

            std::vector<std::wstring> matches;
            for(DWORD index = 0;; ++index) {
                SP_DEVICE_INTERFACE_DATA interfaceData{};
                interfaceData.cbSize = sizeof(interfaceData);
                if(!SetupDiEnumDeviceInterfaces(devices.get(), nullptr, &hidGuid, index, &interfaceData)) {
                    const auto error = GetLastError();
                    if(error == ERROR_NO_MORE_ITEMS) break;
                    return std::unexpected(windowsError("could not enumerate a HID interface", error));
                }
                DWORD detailSize = 0;
                SetupDiGetDeviceInterfaceDetailW(
                    devices.get(), &interfaceData, nullptr, 0, &detailSize, nullptr);
                if(GetLastError() != ERROR_INSUFFICIENT_BUFFER || detailSize == 0) continue;
                std::vector<std::byte> storage(detailSize);
                auto *detail = reinterpret_cast<SP_DEVICE_INTERFACE_DETAIL_DATA_W *>(storage.data());
                detail->cbSize = sizeof(*detail);
                if(!SetupDiGetDeviceInterfaceDetailW(
                       devices.get(), &interfaceData, detail, detailSize, nullptr, nullptr)) continue;
                Handle inspection(CreateFileW(detail->DevicePath, 0,
                    FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr));
                if(!inspection.valid()) continue;
                HIDD_ATTRIBUTES attributes{};
                attributes.Size = sizeof(attributes);
                if(HidD_GetAttributes(inspection.get(), &attributes)
                   && attributes.VendorID == selector.vendorId
                   && attributes.ProductID == selector.productId)
                    matches.emplace_back(detail->DevicePath);
            }
            return matches;
        }
    }

    std::expected<std::unique_ptr<HidTransport>, HidError>
    openHidTransport(const HidDeviceSelector &selector) {
        auto matches = matchingDevicePaths(selector);
        if(!matches) return std::unexpected(std::move(matches.error()));
        if(matches->empty())
            return std::unexpected(HidError { HidErrorCode::NotFound, 0, "matching HID device was not found" });
        if(matches->size() != 1)
            return std::unexpected(HidError { HidErrorCode::MultipleMatches, 0,
                                              "more than one matching HID device was found" });

        Handle inputDevice(CreateFileW(matches->front().c_str(), GENERIC_READ | GENERIC_WRITE,
            FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, nullptr));
        if(!inputDevice.valid()) return std::unexpected(windowsError("could not open matching HID input"));
        Handle outputDevice(CreateFileW(matches->front().c_str(), GENERIC_READ | GENERIC_WRITE,
            FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, nullptr));
        if(!outputDevice.valid()) return std::unexpected(windowsError("could not open matching HID output"));

        PHIDP_PREPARSED_DATA preparsed = nullptr;
        if(!HidD_GetPreparsedData(inputDevice.get(), &preparsed))
            return std::unexpected(windowsError("could not read HID capabilities"));
        HIDP_CAPS capabilities{};
        const auto status = HidP_GetCaps(preparsed, &capabilities);
        HidD_FreePreparsedData(preparsed);
        if(status != HIDP_STATUS_SUCCESS)
            return std::unexpected(HidError { HidErrorCode::IoFailure, static_cast<std::uint32_t>(status),
                                              "could not decode HID capabilities" });

        Handle cancellation(CreateEventW(nullptr, TRUE, FALSE, nullptr));
        if(!cancellation.valid()) return std::unexpected(windowsError("could not create HID cancellation event"));
        return std::unique_ptr<HidTransport>(new WindowsHidTransport(
            std::move(inputDevice), std::move(outputDevice), std::move(cancellation),
            capabilities.InputReportByteLength, capabilities.OutputReportByteLength));
    }
}
