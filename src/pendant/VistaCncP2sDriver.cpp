#include "pendant/VistaCncP2sDriver.h"

#include <algorithm>
#include <array>
#include <cstring>

namespace ngc::pendant::vista_cnc_p2s {
    namespace {
        HidError invalidReport(const std::string_view message) {
            return { HidErrorCode::InvalidReport, 0, std::string(message) };
        }

        bool isDisplayEcho(const InputReport &input, const InputReport &displayPrefix) {
            if(input == displayPrefix) return true;
            constexpr std::size_t MINIMUM_ECHO_PREFIX = 4;
            std::size_t matched = 0;
            while(matched < input.size() && input[matched] == displayPrefix[matched]) ++matched;
            return matched >= MINIMUM_ECHO_PREFIX
                && std::all_of(input.begin() + matched, input.end(), [](const auto byte) {
                    return byte == 0;
                });
        }
    }

    Driver::Driver(std::unique_ptr<HidTransport> transport) : m_transport(std::move(transport)) { }

    std::expected<std::unique_ptr<Driver>, HidError> Driver::open() {
        auto transport = openHidTransport({ VENDOR_ID, PRODUCT_ID });
        if(!transport) return std::unexpected(std::move(transport.error()));
        if((*transport)->inputReportLength() != INPUT_PAYLOAD_SIZE
           && (*transport)->inputReportLength() != INPUT_PAYLOAD_SIZE + 1)
            return std::unexpected(invalidReport("VistaCNC P2-S has an unexpected HID input-report length"));
        if((*transport)->outputReportLength() != 20 && (*transport)->outputReportLength() != 21)
            return std::unexpected(invalidReport("VistaCNC P2-S has an unexpected HID output-report length"));
        return std::make_unique<Driver>(std::move(*transport));
    }

    std::expected<InputState, HidError> Driver::read(const std::chrono::milliseconds timeout) {
        while(true) {
            std::array<std::uint8_t, INPUT_PAYLOAD_SIZE + 1> framed{};
            const auto reportLength = m_transport->inputReportLength();
            if(reportLength != INPUT_PAYLOAD_SIZE && reportLength != framed.size())
                return std::unexpected(invalidReport("VistaCNC P2-S input-report length changed after opening"));
            auto count = m_transport->readInputReport(
                std::span<std::uint8_t>(framed.data(), reportLength), timeout);
            if(!count) return std::unexpected(std::move(count.error()));
            if(*count != reportLength)
                return std::unexpected(invalidReport("VistaCNC P2-S returned a partial input report"));

            const auto payloadOffset = reportLength == INPUT_PAYLOAD_SIZE + 1 ? std::size_t { 1 } : 0;
            if(payloadOffset != 0 && framed[0] != 0)
                return std::unexpected(invalidReport("VistaCNC P2-S returned an unexpected HID report ID"));
            InputReport payload{};
            std::copy_n(framed.begin() + payloadOffset, payload.size(), payload.begin());
            {
                std::scoped_lock lock(m_displayStateMutex);
                bool echo = false;
                for(std::size_t index = 0; index < m_outputEchoCount; ++index)
                    if(isDisplayEcho(payload, m_outputEchoPrefixes[index])) {
                        echo = true;
                        break;
                    }
                if(echo) continue;
            }
            return m_decoder.decode(payload);
        }
    }

    std::expected<void, HidError> Driver::writeDisplay(const std::string_view text) {
        std::array<std::uint8_t, 21> framed{};
        const auto reportLength = m_transport->outputReportLength();
        if(reportLength != 20 && reportLength != framed.size())
            return std::unexpected(invalidReport("VistaCNC P2-S output-report length changed after opening"));
        const auto payloadOffset = reportLength == framed.size() ? std::size_t { 1 } : 0;
        auto *payload = framed.data() + payloadOffset;
        std::fill_n(payload, 17, static_cast<std::uint8_t>(' '));
        if(!text.empty())
            std::memcpy(payload, text.data(), std::min(text.size(), std::size_t { 16 }));
        payload[17] = m_displaySequence++;
        payload[18] = 0;
        payload[19] = 0;
        {
            std::scoped_lock lock(m_displayStateMutex);
            auto &prefix = m_outputEchoPrefixes[m_nextOutputEcho];
            std::copy_n(payload, prefix.size(), prefix.begin());
            m_nextOutputEcho = (m_nextOutputEcho + 1) % m_outputEchoPrefixes.size();
            m_outputEchoCount = std::min(m_outputEchoCount + 1, m_outputEchoPrefixes.size());
        }
        return m_transport->writeOutputReport(
            std::span<const std::uint8_t>(framed.data(), reportLength));
    }

    void Driver::cancel() noexcept { m_transport->cancel(); }
}
