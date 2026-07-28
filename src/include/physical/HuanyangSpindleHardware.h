#pragma once

#include <cstddef>
#include <cstdint>
#include <expected>
#include <memory>
#include <span>
#include <string>

#include "machine/SpindleHardware.h"
#include "physical/PhysicalBackendConfiguration.h"
#include "physical/SerialTransport.h"

namespace ngc::physical {
    [[nodiscard]] std::uint16_t huanyangCrc16(
        std::span<const std::uint8_t> bytes) noexcept;

    class HuanyangSpindleHardware final : public SpindleHardware {
    public:
        [[nodiscard]] static std::expected<
            std::unique_ptr<HuanyangSpindleHardware>, std::string>
        create(
            const HuanyangSpindleConfiguration &configuration,
            std::unique_ptr<SerialTransport> transport);

        [[nodiscard]] bool applyDesired(
            const SpindleEvent &desired) noexcept override;
        [[nodiscard]] bool pollStatus(
            SpindleHardwareStatus &status) noexcept override;
        void safeStop() noexcept override;

    private:
        HuanyangSpindleHardware(
            HuanyangSpindleConfiguration configuration,
            std::unique_ptr<SerialTransport> transport);

        [[nodiscard]] bool initialize() noexcept;
        [[nodiscard]] bool readParameter(
            std::uint8_t parameter,
            std::uint16_t &value) noexcept;
        [[nodiscard]] bool writeFrequency(
            std::uint16_t frequency) noexcept;
        [[nodiscard]] bool writeControl(
            std::uint8_t control) noexcept;
        [[nodiscard]] bool readStatus(
            std::uint8_t selector,
            std::uint16_t &value) noexcept;
        [[nodiscard]] bool transaction(
            std::uint8_t function,
            std::span<const std::uint8_t> data,
            std::span<std::uint8_t> response,
            std::size_t &responseSize) noexcept;

        HuanyangSpindleConfiguration m_configuration;
        std::unique_ptr<SerialTransport> m_transport;
        SpindleEvent m_desired{};
        double m_maximumFrequency = 0.0;
        double m_minimumFrequency = 0.0;
        double m_ratedMaximumSpeed = 0.0;
    };

    [[nodiscard]] std::expected<
        std::unique_ptr<SpindleHardware>, std::string>
    createHuanyangSpindleHardware(
        const HuanyangSpindleConfiguration &configuration);
}
