#pragma once

#include <cstddef>
#include <chrono>
#include <cstdint>
#include <expected>
#include <memory>
#include <span>
#include <string>

namespace ngc::pendant {
    enum class HidErrorCode {
        NotFound,
        MultipleMatches,
        AccessDenied,
        Disconnected,
        TimedOut,
        Cancelled,
        InvalidReport,
        IoFailure,
        Unsupported,
    };

    struct HidError {
        HidErrorCode code = HidErrorCode::IoFailure;
        std::uint32_t systemCode = 0;
        std::string message;
    };

    struct HidDeviceSelector {
        std::uint16_t vendorId = 0;
        std::uint16_t productId = 0;
    };

    class HidTransport {
    public:
        virtual ~HidTransport() = default;

        virtual std::size_t inputReportLength() const noexcept = 0;
        virtual std::size_t outputReportLength() const noexcept = 0;
        virtual std::expected<std::size_t, HidError>
        readInputReport(std::span<std::uint8_t> report,
                        std::chrono::milliseconds timeout) = 0;
        virtual std::expected<void, HidError>
        writeOutputReport(std::span<const std::uint8_t> report) = 0;

        // May be called from another NRT thread to release a blocking read.
        virtual void cancel() noexcept = 0;
    };

    std::expected<std::unique_ptr<HidTransport>, HidError>
    openHidTransport(const HidDeviceSelector &selector);
}
