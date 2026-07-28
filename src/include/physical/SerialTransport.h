#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <memory>
#include <span>
#include <string>

#include "physical/PhysicalBackendConfiguration.h"

namespace ngc::physical {
    class SerialTransport {
    public:
        virtual ~SerialTransport() = default;

        [[nodiscard]] virtual bool exchange(
            std::span<const std::uint8_t> request,
            std::span<std::uint8_t> response,
            std::size_t &responseSize) noexcept = 0;
    };

    [[nodiscard]] std::expected<
        std::unique_ptr<SerialTransport>, std::string>
    openSerialTransport(
        const HuanyangSpindleConfiguration &configuration,
        std::chrono::milliseconds timeout =
            std::chrono::milliseconds(500));
}
