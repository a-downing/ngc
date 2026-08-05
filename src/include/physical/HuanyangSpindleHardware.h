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

    struct HuanyangSpindleSetup {
        double baseFrequency = 0.0;
        double maximumFrequency = 0.0;
        double minimumFrequency = 0.0;
        double ratedVoltage = 0.0;
        double ratedCurrent = 0.0;
        std::uint16_t motorPoles = 0;
        double ratedSpeedAt50Hz = 0.0;
        double ratedMaximumSpeed = 0.0;
    };

    class HuanyangSpindleHardware final : public SpindleHardware {
    public:
        [[nodiscard]] static std::expected<
            std::unique_ptr<HuanyangSpindleHardware>, std::string>
        create(
            const HuanyangSpindleConfiguration &configuration,
            std::unique_ptr<SerialTransport> transport);
        ~HuanyangSpindleHardware() override;

        [[nodiscard]] bool applyDesired(
            const SpindleEvent &desired,
            const std::atomic<SpindleSafetyState> &safety) noexcept override;
        [[nodiscard]] bool pollStatus(
            SpindleHardwareStatus &status) noexcept override;
        void safeStop() noexcept override;
        [[nodiscard]] const HuanyangSpindleSetup &setup()
            const noexcept;

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
        HuanyangSpindleSetup m_setup;
        SpindleEvent m_desired{};
    };

    [[nodiscard]] std::expected<
        std::unique_ptr<SpindleHardware>, std::string>
    createHuanyangSpindleHardware(
        const HuanyangSpindleConfiguration &configuration);
}
