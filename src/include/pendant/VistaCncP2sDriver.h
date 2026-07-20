#pragma once

#include <chrono>
#include <array>
#include <cstdint>
#include <expected>
#include <memory>
#include <mutex>
#include <string_view>

#include "pendant/HidTransport.h"
#include "pendant/VistaCncP2sProtocol.h"

namespace ngc::pendant::vista_cnc_p2s {
    inline constexpr std::chrono::milliseconds REPORT_TIMEOUT { 2000 };

    class Driver {
    public:
        explicit Driver(std::unique_ptr<HidTransport> transport);

        Driver(const Driver &) = delete;
        Driver &operator=(const Driver &) = delete;

        static std::expected<std::unique_ptr<Driver>, HidError> open();

        std::expected<InputState, HidError> read(
            std::chrono::milliseconds timeout = std::chrono::milliseconds::max());
        std::expected<void, HidError> writeDisplay(std::string_view text);
        void cancel() noexcept;

    private:
        std::unique_ptr<HidTransport> m_transport;
        InputDecoder m_decoder;
        std::uint8_t m_displaySequence = 0;
        std::mutex m_displayStateMutex;
        static constexpr std::size_t OUTPUT_ECHO_HISTORY = 8;
        std::array<InputReport, OUTPUT_ECHO_HISTORY> m_outputEchoPrefixes{};
        std::size_t m_outputEchoCount = 0;
        std::size_t m_nextOutputEcho = 0;
    };
}
