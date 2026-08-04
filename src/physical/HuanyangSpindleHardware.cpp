#include "physical/HuanyangSpindleHardware.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace ngc::physical {
    namespace {
        constexpr std::uint8_t FUNCTION_READ_PARAMETER = 0x01;
        constexpr std::uint8_t FUNCTION_WRITE_CONTROL = 0x03;
        constexpr std::uint8_t FUNCTION_READ_STATUS = 0x04;
        constexpr std::uint8_t FUNCTION_WRITE_FREQUENCY = 0x05;

        constexpr std::uint8_t CONTROL_RUN_FORWARD = 0x01;
        constexpr std::uint8_t CONTROL_STOP = 0x08;
        constexpr std::uint8_t CONTROL_RUN_REVERSE = 0x11;

        constexpr std::uint8_t STATUS_OUTPUT_FREQUENCY = 0x01;
        constexpr std::uint8_t STATUS_OUTPUT_CURRENT = 0x02;

        constexpr std::uint8_t PARAMETER_BASE_FREQUENCY = 4;
        constexpr std::uint8_t PARAMETER_MAXIMUM_FREQUENCY = 5;
        constexpr std::uint8_t PARAMETER_MINIMUM_FREQUENCY = 11;
        constexpr std::uint8_t PARAMETER_RATED_VOLTAGE = 141;
        constexpr std::uint8_t PARAMETER_RATED_CURRENT = 142;
        constexpr std::uint8_t PARAMETER_MOTOR_POLES = 143;
        constexpr std::uint8_t PARAMETER_RATED_SPEED = 144;

        constexpr std::size_t MAXIMUM_PACKET_SIZE = 8;

        bool finitePositive(const double value) noexcept {
            return std::isfinite(value) && value > 0.0;
        }
    }

    std::uint16_t huanyangCrc16(
        const std::span<const std::uint8_t> bytes) noexcept {
        auto crc = std::uint16_t{0xFFFF};
        for (const auto byte : bytes) {
            crc ^= byte;
            for (auto bit = 0; bit < 8; ++bit) {
                if ((crc & 1) != 0) {
                    crc = static_cast<std::uint16_t>(
                        (crc >> 1) ^ 0xA001);
                } else {
                    crc = static_cast<std::uint16_t>(crc >> 1);
                }
            }
        }

        return crc;
    }

    std::expected<
        std::unique_ptr<HuanyangSpindleHardware>, std::string>
    HuanyangSpindleHardware::create(
        const HuanyangSpindleConfiguration &configuration,
        std::unique_ptr<SerialTransport> transport) {
        if (!transport) {
            return std::unexpected(
                "Huanyang spindle requires a serial transport");
        }
        auto result = std::unique_ptr<HuanyangSpindleHardware>(
            new HuanyangSpindleHardware(
                configuration, std::move(transport)));
        if (!result->initialize()) {
            result->safeStop();

            return std::unexpected(
                "Huanyang spindle initialization failed");
        }

        return result;
    }

    HuanyangSpindleHardware::HuanyangSpindleHardware(
        HuanyangSpindleConfiguration configuration,
        std::unique_ptr<SerialTransport> transport)
        : m_configuration(std::move(configuration)),
          m_transport(std::move(transport)) { }

    HuanyangSpindleHardware::~HuanyangSpindleHardware() {
        safeStop();
    }

    bool HuanyangSpindleHardware::initialize() noexcept {
        if (!writeControl(CONTROL_STOP)) {
            return false;
        }

        auto baseFrequency = std::uint16_t{0};
        auto maximumFrequency = std::uint16_t{0};
        auto minimumFrequency = std::uint16_t{0};
        auto ratedVoltage = std::uint16_t{0};
        auto ratedCurrent = std::uint16_t{0};
        auto motorPoles = std::uint16_t{0};
        auto ratedSpeedAt50Hz = std::uint16_t{0};
        if (!readParameter(
                PARAMETER_BASE_FREQUENCY, baseFrequency)
            || !readParameter(
                PARAMETER_MAXIMUM_FREQUENCY, maximumFrequency)
            || !readParameter(
                PARAMETER_MINIMUM_FREQUENCY, minimumFrequency)
            || !readParameter(
                PARAMETER_RATED_VOLTAGE, ratedVoltage)
            || !readParameter(
                PARAMETER_RATED_CURRENT, ratedCurrent)
            || !readParameter(
                PARAMETER_MOTOR_POLES, motorPoles)
            || !readParameter(
                PARAMETER_RATED_SPEED, ratedSpeedAt50Hz)) {
            return false;
        }

        m_setup = {
            .baseFrequency = baseFrequency * 0.01,
            .maximumFrequency = maximumFrequency * 0.01,
            .minimumFrequency = minimumFrequency * 0.01,
            .ratedVoltage = ratedVoltage * 0.1,
            .ratedCurrent = ratedCurrent * 0.1,
            .motorPoles = motorPoles,
            .ratedSpeedAt50Hz =
                static_cast<double>(ratedSpeedAt50Hz),
            .ratedMaximumSpeed =
                ratedSpeedAt50Hz / 50.0
                * (maximumFrequency * 0.01),
        };
        if (!finitePositive(m_setup.baseFrequency)
            || !finitePositive(m_setup.maximumFrequency)
            || !std::isfinite(m_setup.minimumFrequency)
            || m_setup.minimumFrequency < 0.0
            || m_setup.minimumFrequency
                > m_setup.maximumFrequency
            || ratedVoltage == 0
            || ratedCurrent == 0
            || motorPoles == 0
            || !finitePositive(m_setup.ratedMaximumSpeed)) {
            return false;
        }

        return true;
    }

    bool HuanyangSpindleHardware::applyDesired(
        const SpindleEvent &desired) noexcept {
        if (!desired.enabled) {
            if (!writeControl(CONTROL_STOP)) {
                return false;
            }
            m_desired = {};

            return true;
        }
        if (!finitePositive(desired.speed)
            || desired.speed > m_configuration.maximumSpeed) {
            return false;
        }

        const auto requestedFrequency =
            desired.speed / m_setup.ratedMaximumSpeed
            * m_setup.maximumFrequency;
        const auto limitedFrequency = std::clamp(
            requestedFrequency, m_setup.minimumFrequency,
            m_setup.maximumFrequency);
        const auto hundredths = std::llround(
            limitedFrequency * 100.0);
        if (hundredths <= 0
            || hundredths
                > std::numeric_limits<std::uint16_t>::max()) {
            return false;
        }
        if (!writeFrequency(
                static_cast<std::uint16_t>(hundredths))) {
            return false;
        }
        const auto control = desired.direction == Direction::CW
            ? CONTROL_RUN_FORWARD
            : CONTROL_RUN_REVERSE;
        if (!writeControl(control)) {
            return false;
        }

        m_desired = desired;

        return true;
    }

    bool HuanyangSpindleHardware::pollStatus(
        SpindleHardwareStatus &status) noexcept {
        auto frequency = std::uint16_t{0};
        auto current = std::uint16_t{0};
        if (!readStatus(STATUS_OUTPUT_FREQUENCY, frequency)
            || !readStatus(STATUS_OUTPUT_CURRENT, current)) {
            return false;
        }

        const auto outputFrequency = frequency * 0.01;
        const auto speed =
            outputFrequency / m_setup.maximumFrequency
            * m_setup.ratedMaximumSpeed;
        auto atSpeed = false;
        if (m_desired.enabled
            && finitePositive(m_desired.speed)) {
            atSpeed = std::abs(
                speed / m_desired.speed - 1.0)
                <= m_configuration.atSpeedTolerance;
        }
        status = {
            .communicationHealthy = true,
            .atSpeed = atSpeed,
            .speed = speed,
            .current = current * 0.1,
            .faultCode = 0,
        };

        return true;
    }

    void HuanyangSpindleHardware::safeStop() noexcept {
        static_cast<void>(writeControl(CONTROL_STOP));
        m_desired = {};
    }

    const HuanyangSpindleSetup &
    HuanyangSpindleHardware::setup() const noexcept {
        return m_setup;
    }

    bool HuanyangSpindleHardware::readParameter(
        const std::uint8_t parameter,
        std::uint16_t &value) noexcept {
        const std::array data{
            std::uint8_t{0x03}, parameter,
            std::uint8_t{0x00}, std::uint8_t{0x00},
        };
        auto response = std::array<std::uint8_t, MAXIMUM_PACKET_SIZE>{};
        auto responseSize = std::size_t{0};
        if (!transaction(
                FUNCTION_READ_PARAMETER, data,
                response, responseSize)
            || response[3] != parameter
            || (response[2] != 2 && response[2] != 3)) {
            return false;
        }
        value = response[2] == 2
            ? response[4]
            : static_cast<std::uint16_t>(
                response[4] << 8 | response[5]);

        return true;
    }

    bool HuanyangSpindleHardware::writeFrequency(
        const std::uint16_t frequency) noexcept {
        const std::array data{
            std::uint8_t{0x02},
            static_cast<std::uint8_t>(frequency >> 8),
            static_cast<std::uint8_t>(frequency),
        };
        auto response = std::array<std::uint8_t, MAXIMUM_PACKET_SIZE>{};
        auto responseSize = std::size_t{0};
        return transaction(
                FUNCTION_WRITE_FREQUENCY, data,
                response, responseSize)
            && response[2] == 2
            && response[3] == data[1]
            && response[4] == data[2];
    }

    bool HuanyangSpindleHardware::writeControl(
        const std::uint8_t control) noexcept {
        const std::array data{
            std::uint8_t{0x01}, control,
        };
        auto response = std::array<std::uint8_t, MAXIMUM_PACKET_SIZE>{};
        auto responseSize = std::size_t{0};
        return transaction(
                FUNCTION_WRITE_CONTROL, data,
                response, responseSize)
            && response[2] == 1
            && response[3] == 0;
    }

    bool HuanyangSpindleHardware::readStatus(
        const std::uint8_t selector,
        std::uint16_t &value) noexcept {
        const std::array data{
            std::uint8_t{0x01}, selector,
        };
        auto response = std::array<std::uint8_t, MAXIMUM_PACKET_SIZE>{};
        auto responseSize = std::size_t{0};
        if (!transaction(
                FUNCTION_READ_STATUS, data,
                response, responseSize)
            || response[2] != 3
            || response[3] != selector) {
            return false;
        }
        value = static_cast<std::uint16_t>(
            response[4] << 8 | response[5]);

        return true;
    }

    bool HuanyangSpindleHardware::transaction(
        const std::uint8_t function,
        const std::span<const std::uint8_t> data,
        const std::span<std::uint8_t> response,
        std::size_t &responseSize) noexcept {
        auto request = std::array<std::uint8_t, MAXIMUM_PACKET_SIZE>{};
        const auto requestSize = data.size() + 2;
        if (requestSize + 2 > request.size()
            || response.size() < 5) {
            return false;
        }
        request[0] = m_configuration.slaveAddress;
        request[1] = function;
        std::ranges::copy(data, request.begin() + 2);
        const auto crc = huanyangCrc16(
            std::span(request).first(requestSize));
        request[requestSize] =
            static_cast<std::uint8_t>(crc);
        request[requestSize + 1] =
            static_cast<std::uint8_t>(crc >> 8);
        if (!m_transport->exchange(
                std::span(request).first(requestSize + 2),
                response, responseSize)
            || responseSize < 5
            || responseSize !=
                static_cast<std::size_t>(response[2]) + 5
            || response[0] != m_configuration.slaveAddress
            || response[1] != function) {
            return false;
        }
        const auto receivedCrc = static_cast<std::uint16_t>(
            response[responseSize - 2]
            | response[responseSize - 1] << 8);
        const auto calculatedCrc = huanyangCrc16(
            response.first(responseSize - 2));

        return receivedCrc == calculatedCrc;
    }

    std::expected<std::unique_ptr<SpindleHardware>, std::string>
    createHuanyangSpindleHardware(
        const HuanyangSpindleConfiguration &configuration) {
        auto transport = openSerialTransport(configuration);
        if (!transport) {
            return std::unexpected(transport.error());
        }
        auto hardware = HuanyangSpindleHardware::create(
            configuration, std::move(*transport));
        if (!hardware) {
            return std::unexpected(hardware.error());
        }

        return std::unique_ptr<SpindleHardware>(
            std::move(*hardware));
    }
}
