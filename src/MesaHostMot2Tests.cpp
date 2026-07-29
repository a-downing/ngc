#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <filesystem>
#include <format>
#include <iostream>
#include <limits>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

#include "machine/DigitalIoProgram.h"
#include "mesa/HostMot2CyclicIo.h"
#include "mesa/HostMot2Discovery.h"
#include "mesa/HostMot2Latency.h"
#include "mesa/Lbp16CyclicTransaction.h"
#include "mesa/MesaBackendConfiguration.h"
#include "mesa/MesaProductionExecutorIo.h"
#include "mesa/SevenI96Capabilities.h"
#include "mesa/SevenI96CyclicLayout.h"

namespace {
    constexpr std::uint32_t COOKIE_ADDRESS = 0x0100;
    constexpr std::uint32_t CONFIGURATION_NAME_ADDRESS = 0x0104;
    constexpr std::uint32_t IDROM_ADDRESS_ADDRESS = 0x010C;
    constexpr std::uint32_t IDROM_ADDRESS = 0x0400;
    constexpr std::uint32_t MODULE_ADDRESS = IDROM_ADDRESS + 64;
    constexpr std::uint32_t PIN_ADDRESS = IDROM_ADDRESS + 512;

    void require(const bool condition, const std::string_view message) {
        if (!condition) {
            throw std::runtime_error(std::string(message));
        }
    }

    void putLittleEndian16(
        const std::span<std::byte> destination,
        const std::uint16_t value) {
        require(destination.size() >= 2,
                "little-endian 16-bit destination is too small");
        destination[0] = static_cast<std::byte>(value);
        destination[1] = static_cast<std::byte>(value >> 8);
    }

    void putLittleEndian32(
        const std::span<std::byte> destination,
        const std::uint32_t value) {
        require(destination.size() >= 4,
                "little-endian 32-bit destination is too small");
        destination[0] = static_cast<std::byte>(value);
        destination[1] = static_cast<std::byte>(value >> 8);
        destination[2] = static_cast<std::byte>(value >> 16);
        destination[3] = static_cast<std::byte>(value >> 24);
    }

    std::uint16_t littleEndian16(
        const std::span<const std::byte> source) {
        require(source.size() >= 2,
                "little-endian 16-bit source is too small");

        return static_cast<std::uint16_t>(
            std::to_integer<std::uint16_t>(source[0])
            | (std::to_integer<std::uint16_t>(source[1]) << 8));
    }

    std::uint32_t littleEndian32(
        const std::span<const std::byte> source) {
        require(source.size() >= 4,
                "little-endian 32-bit source is too small");

        return std::to_integer<std::uint32_t>(source[0])
            | (std::to_integer<std::uint32_t>(source[1]) << 8)
            | (std::to_integer<std::uint32_t>(source[2]) << 16)
            | (std::to_integer<std::uint32_t>(source[3]) << 24);
    }

    std::uint8_t byteValue(
        const std::span<const std::byte> bytes,
        const std::size_t offset) {
        require(offset < bytes.size(), "byte fixture offset is out of range");

        return std::to_integer<std::uint8_t>(bytes[offset]);
    }

    class FixtureReader final : public ngc::mesa::HostMot2RegisterReader {
    public:
        std::expected<void, std::string> read(
            const std::uint32_t address,
            const std::span<std::byte> destination) override {
            if (address >= m_memory.size()
                || destination.size() > m_memory.size() - address) {
                return std::unexpected("fixture read is out of range");
            }
            std::ranges::copy(
                std::span(m_memory).subspan(address, destination.size()),
                destination.begin());

            return {};
        }

        void putByte(
            const std::uint32_t address,
            const std::uint8_t value) {
            m_memory[address] = static_cast<std::byte>(value);
        }

        void putLittleEndian16(
            const std::uint32_t address,
            const std::uint16_t value) {
            putByte(address, static_cast<std::uint8_t>(value));
            putByte(address + 1, static_cast<std::uint8_t>(value >> 8));
        }

        void putLittleEndian32(
            const std::uint32_t address,
            const std::uint32_t value) {
            putByte(address, static_cast<std::uint8_t>(value));
            putByte(address + 1, static_cast<std::uint8_t>(value >> 8));
            putByte(address + 2, static_cast<std::uint8_t>(value >> 16));
            putByte(address + 3, static_cast<std::uint8_t>(value >> 24));
        }

        void putString(
            const std::uint32_t address,
            const std::string_view value,
            const std::size_t size) {
            require(value.size() <= size, "fixture string is too large");
            for (std::size_t index = 0; index < value.size(); ++index) {
                putByte(
                    address + static_cast<std::uint32_t>(index),
                    static_cast<std::uint8_t>(value[index]));
            }
        }

    private:
        std::array<std::byte, 0x1'0000> m_memory{};
    };

    class LatencyFixtureReader final : public ngc::mesa::HostMot2RegisterReader {
    public:
        std::expected<void, std::string> read(
            const std::uint32_t address,
            const std::span<std::byte> destination) override {
            require(address == COOKIE_ADDRESS,
                    "latency measurement read the wrong register");
            require(destination.size() == 4,
                    "latency measurement used the wrong read size");

            ++m_reads;
            if (m_reads == 2) {
                return std::unexpected("synthetic read failure");
            }

            const auto cookie = m_reads == 3
                ? std::uint32_t{0x1234'5678}
                : std::uint32_t{0x55AA'CAFE};
            for (std::size_t index = 0; index < destination.size(); ++index) {
                destination[index] = static_cast<std::byte>(
                    cookie >> (index * 8));
            }

            return {};
        }

    private:
        std::size_t m_reads = 0;
    };

    class DatagramFixtureTransport final
        : public ngc::mesa::Lbp16DatagramTransport {
    public:
        ngc::mesa::Lbp16DatagramResult exchange(
            const std::span<const std::byte> request,
            const std::span<std::byte> response) noexcept override {
            m_requestSize = request.size();
            std::ranges::copy(request, m_request.begin());
            if (m_status != ngc::mesa::Lbp16DatagramStatus::Complete) {
                return {
                    .status = m_status,
                    .systemError = m_systemError,
                    .sentBytes = request.size(),
                    .receivedBytes = 0,
                };
            }
            const auto queued = m_nextQueuedResponse < m_queuedResponseCount;
            const auto responseSize = queued
                ? m_queuedResponseSizes[m_nextQueuedResponse]
                : m_responseSize;
            if (response.size() != responseSize) {
                return {
                    .status =
                        ngc::mesa::Lbp16DatagramStatus::UnexpectedResponseSize,
                    .systemError = 0,
                    .sentBytes = request.size(),
                    .receivedBytes = responseSize,
                };
            }

            if (queued) {
                std::ranges::copy(
                    std::span(
                        m_queuedResponses[m_nextQueuedResponse])
                        .first(responseSize),
                    response.begin());
                ++m_nextQueuedResponse;
            } else {
                std::ranges::copy(
                    std::span(m_response).first(m_responseSize),
                    response.begin());
            }

            return {
                .status = ngc::mesa::Lbp16DatagramStatus::Complete,
                .systemError = 0,
                .sentBytes = request.size(),
                .receivedBytes = response.size(),
            };
        }

        std::span<std::byte> response(const std::size_t size) {
            require(size <= m_response.size(),
                    "datagram fixture response is too large");
            m_response.fill(std::byte{});
            m_responseSize = size;

            return std::span(m_response).first(size);
        }

        std::span<std::byte> queueResponse(const std::size_t size) {
            require(
                m_queuedResponseCount < m_queuedResponses.size(),
                "too many queued datagram fixture responses");
            require(size <= m_queuedResponses.front().size(),
                    "queued datagram fixture response is too large");
            const auto index = m_queuedResponseCount++;
            m_queuedResponses[index].fill(std::byte{});
            m_queuedResponseSizes[index] = size;

            return std::span(m_queuedResponses[index]).first(size);
        }

        std::span<const std::byte> request() const {
            return std::span(m_request).first(m_requestSize);
        }

        void fail(
            const ngc::mesa::Lbp16DatagramStatus status,
            const int systemError) {
            m_status = status;
            m_systemError = systemError;
        }

    private:
        std::array<
            std::byte,
            ngc::mesa::LBP16_MAX_DATAGRAM_SIZE> m_request{};
        std::array<
            std::byte,
            ngc::mesa::LBP16_MAX_DATAGRAM_SIZE> m_response{};
        std::array<
            std::array<
                std::byte,
                ngc::mesa::LBP16_MAX_DATAGRAM_SIZE>,
            4> m_queuedResponses{};
        std::array<std::size_t, 4> m_queuedResponseSizes{};
        std::size_t m_requestSize = 0;
        std::size_t m_responseSize = 0;
        std::size_t m_queuedResponseCount = 0;
        std::size_t m_nextQueuedResponse = 0;
        ngc::mesa::Lbp16DatagramStatus m_status =
            ngc::mesa::Lbp16DatagramStatus::Complete;
        int m_systemError = 0;
    };

    struct RequestWrite {
        std::size_t commandOffset = 0;
        std::span<const std::byte> data;
    };

    RequestWrite findRequestWrite(
        const std::span<const std::byte> request,
        const std::uint16_t address) {
        auto offset = std::size_t{0};
        auto found = RequestWrite{};
        auto hasMatch = false;
        while (offset + 4 <= request.size()) {
            const auto command = littleEndian16(request.subspan(offset));
            const auto commandAddress =
                littleEndian16(request.subspan(offset + 2));
            const auto argumentCount = command & 0x7F;
            const auto argumentSize = (command & 0x0200) != 0
                ? std::size_t{4}
                : (command & 0x0100) != 0
                    ? std::size_t{2}
                    : std::size_t{1};
            const auto write = (command & 0x8000) != 0;
            const auto dataSize = write
                ? argumentCount * argumentSize
                : std::size_t{0};
            require(offset + 4 + dataSize <= request.size(),
                    "LBP16 fixture request is truncated");
            if (write && commandAddress == address) {
                found = {
                    .commandOffset = offset,
                    .data = request.subspan(offset + 4, dataSize),
                };
                hasMatch = true;
            }
            offset += 4 + dataSize;
        }
        if (hasMatch) {
            return found;
        }

        throw std::runtime_error(
            "LBP16 fixture request does not contain the expected write");
    }

    void putCyclicConfirmation(
        const std::span<std::byte> response,
        const std::size_t offset,
        const std::uint32_t readSequence,
        const std::uint32_t writeSequence,
        const std::uint16_t boardError = 0) {
        require(response.size() >= offset + 10,
                "cyclic confirmation fixture is too small");
        putLittleEndian32(
            response.subspan(offset), readSequence);
        putLittleEndian32(
            response.subspan(offset + 4), writeSequence);
        putLittleEndian16(
            response.subspan(offset + 8), boardError);
    }

    void putModule(
        FixtureReader &reader, const std::size_t index,
        const std::uint8_t tag, const std::uint8_t version,
        const std::uint8_t clockTag, const std::uint8_t instances,
        const std::uint16_t baseAddress, const std::uint8_t registers,
        const std::uint8_t strides, const std::uint32_t bitmap) {
        const auto address =
            MODULE_ADDRESS + static_cast<std::uint32_t>(index * 12);
        reader.putByte(address, tag);
        reader.putByte(address + 1, version);
        reader.putByte(address + 2, clockTag);
        reader.putByte(address + 3, instances);
        reader.putLittleEndian16(address + 4, baseAddress);
        reader.putByte(address + 6, registers);
        reader.putByte(address + 7, strides);
        reader.putLittleEndian32(address + 8, bitmap);
    }

    FixtureReader sevenI96Fixture() {
        FixtureReader reader;
        reader.putLittleEndian32(COOKIE_ADDRESS, 0x55AACAFE);
        reader.putString(
            CONFIGURATION_NAME_ADDRESS, "HOSTMOT2", 8);
        reader.putLittleEndian32(
            IDROM_ADDRESS_ADDRESS, IDROM_ADDRESS);

        reader.putLittleEndian32(IDROM_ADDRESS, 3);
        reader.putLittleEndian32(IDROM_ADDRESS + 4, 64);
        reader.putLittleEndian32(IDROM_ADDRESS + 8, 512);
        reader.putString(IDROM_ADDRESS + 12, "MESA7I96", 8);
        reader.putLittleEndian32(IDROM_ADDRESS + 20, 9);
        reader.putLittleEndian32(IDROM_ADDRESS + 24, 144);
        reader.putLittleEndian32(IDROM_ADDRESS + 28, 3);
        reader.putLittleEndian32(IDROM_ADDRESS + 32, 51);
        reader.putLittleEndian32(IDROM_ADDRESS + 36, 17);
        reader.putLittleEndian32(IDROM_ADDRESS + 40, 100'000'000);
        reader.putLittleEndian32(IDROM_ADDRESS + 44, 200'000'000);
        reader.putLittleEndian32(IDROM_ADDRESS + 48, 4);
        reader.putLittleEndian32(IDROM_ADDRESS + 52, 64);
        reader.putLittleEndian32(IDROM_ADDRESS + 56, 256);
        reader.putLittleEndian32(IDROM_ADDRESS + 60, 256);

        putModule(reader, 0, 0x1A, 0, 1, 1, 0x7000, 7, 0, 0);
        putModule(reader, 1, 0x02, 0, 1, 1, 0x0C00, 3, 0, 0);
        putModule(
            reader, 2, 0x03, 0, 1, 3, 0x1000, 5, 0, 0x1F);
        putModule(
            reader, 3, 0x05, 2, 1, 5, 0x2000, 10, 0, 0x1FF);
        putModule(
            reader, 4, 0x04, 2, 1, 1, 0x3000, 5, 0, 0x03);
        putModule(
            reader, 5, 0xC1, 0, 1, 1, 0x5B00, 6, 0x10, 0x3C);
        putModule(
            reader, 6, 0xC3, 0, 1, 1, 0x7D00, 2, 0, 0x03);
        putModule(
            reader, 7, 0x80, 0, 1, 1, 0x0200, 1, 0, 0);

        for (std::uint32_t index = 0; index < 51; ++index) {
            const auto address = PIN_ADDRESS + index * 4;
            reader.putByte(address + 3, 0x03);
        }
        for (std::uint32_t output = 0; output < 6; ++output) {
            const auto address = PIN_ADDRESS + (11 + output) * 4;
            reader.putByte(
                address, static_cast<std::uint8_t>(0x81 + output));
            reader.putByte(address + 1, 0xC3);
            reader.putByte(address + 2, 0);
        }
        for (std::uint32_t channel = 0; channel < 5; ++channel) {
            const auto address = PIN_ADDRESS + (17 + channel * 2) * 4;
            reader.putByte(address, 0x81);
            reader.putByte(address + 1, 0x05);
            reader.putByte(
                address + 2, static_cast<std::uint8_t>(channel));
            reader.putByte(address + 4, 0x82);
            reader.putByte(address + 5, 0x05);
            reader.putByte(
                address + 6, static_cast<std::uint8_t>(channel));
        }
        for (std::uint32_t pin = 0; pin < 3; ++pin) {
            const auto address = PIN_ADDRESS + (27 + pin) * 4;
            reader.putByte(
                address, static_cast<std::uint8_t>(pin + 1));
            reader.putByte(address + 1, 0x04);
            reader.putByte(address + 2, 0);
        }

        return reader;
    }

    void testDiscoversTypedSevenI96Inventory() {
        auto reader = sevenI96Fixture();

        const auto inventory = ngc::mesa::discoverHostMot2(reader);

        require(inventory.has_value(), "valid HostMot2 fixture was rejected");
        require(inventory->configurationName == "HOSTMOT2",
                "configuration name was not decoded");
        require(inventory->idrom.boardName == "MESA7I96",
                "board name was not decoded");
        require(inventory->idrom.ioPorts == 3
                && inventory->idrom.ioWidth == 51
                && inventory->idrom.portWidth == 17,
                "I/O topology was not decoded");
        require(inventory->modules.size() == 8,
                "module sentinel was not honored");
        require(inventory->modules[3].tag == 0x05
                && inventory->modules[3].instances == 5
                && inventory->modules[3].baseAddress == 0x2000,
                "StepGen descriptor was not decoded");
        require(
            ngc::mesa::hostMot2ModuleClockHz(
                *inventory, inventory->modules[3]) == 100'000'000,
            "module clock selection was not decoded");
        require(inventory->pins.size() == 51,
                "pin descriptor count did not use IDROM I/O width");
        require(inventory->pins[17].secondaryOutput()
                && inventory->pins[17].secondaryPinNumber() == 1
                && inventory->pins[17].secondaryTag == 0x05,
                "secondary pin descriptor was not decoded");
    }

    void testRejectsWrongCookie() {
        auto reader = sevenI96Fixture();
        reader.putLittleEndian32(COOKIE_ADDRESS, 0x12345678);

        const auto inventory = ngc::mesa::discoverHostMot2(reader);

        require(!inventory.has_value()
                && inventory.error().find("cookie mismatch")
                    != std::string::npos,
                "wrong HostMot2 cookie was not rejected");
    }

    void testRejectsInconsistentPinTopology() {
        auto reader = sevenI96Fixture();
        reader.putLittleEndian32(IDROM_ADDRESS + 32, 50);

        const auto inventory = ngc::mesa::discoverHostMot2(reader);

        require(!inventory.has_value()
                && inventory.error().find("does not equal")
                    != std::string::npos,
                "inconsistent HostMot2 I/O width was not rejected");
    }

    void testRejectsUnterminatedModuleDescriptors() {
        auto reader = sevenI96Fixture();
        for (std::size_t index = 8; index < 32; ++index) {
            putModule(reader, index, 0x03, 0, 1, 1, 0x1000, 5, 0, 0x1F);
        }

        const auto inventory = ngc::mesa::discoverHostMot2(reader);

        require(!inventory.has_value()
                && inventory.error().find("no terminating null tag")
                    != std::string::npos,
                "unterminated HostMot2 modules were not rejected");
    }

    void testRejectsOverflowingDescriptorOffset() {
        auto reader = sevenI96Fixture();
        reader.putLittleEndian32(IDROM_ADDRESS + 4, 0xFFFF'FFFF);

        const auto inventory = ngc::mesa::discoverHostMot2(reader);

        require(!inventory.has_value()
                && inventory.error().find("offset")
                    != std::string::npos,
                "overflowing HostMot2 descriptor offset was not rejected");
    }

    ngc::mesa::HostMot2Inventory sevenI96Inventory() {
        auto reader = sevenI96Fixture();
        auto inventory = ngc::mesa::discoverHostMot2(reader);
        require(inventory.has_value(),
                "valid 7I96 capability fixture discovery failed");

        return std::move(*inventory);
    }

    void testValidatesSevenI96CapabilitiesAndSelections() {
        const auto inventory = sevenI96Inventory();

        const auto capabilities =
            ngc::mesa::validateSevenI96Capabilities(
                inventory, {
                    .stepGeneratorChannels = {0, 1, 2, 3},
                    .isolatedInputPins = {0, 1, 2},
                    .isolatedOutputPins = {0},
                    .encoderChannel = 0,
                });

        require(capabilities.has_value(),
                "valid 7I96 capabilities were rejected");
        require(capabilities->stepGenerator.descriptor.instances == 5
                && capabilities->stepGenerators[4].channel == 4
                && capabilities->stepGenerators[4].stepPin == 25
                && capabilities->stepGenerators[4].directionPin == 26,
                "7I96 step-generator capabilities are incorrect");
        require(capabilities->isolatedInputPins.front() == 0
                && capabilities->isolatedInputPins.back() == 10
                && capabilities->isolatedOutputPins.front() == 11
                && capabilities->isolatedOutputPins.back() == 16,
                "7I96 isolated I/O capabilities are incorrect");
        require(capabilities->encoderPins.pinA == 27
                && capabilities->encoderPins.pinB == 28
                && capabilities->encoderPins.indexPin == 29,
                "7I96 encoder capabilities are incorrect");
    }

    void testLoadsProposedMesaBackendConfiguration() {
        const auto configuration =
            ngc::mesa::loadMesaBackendConfiguration(
                std::filesystem::path(NGC_SOURCE_DIR)
                    / "physical_backend.toml");

        require(configuration.has_value()
                && configuration->address == "10.10.10.10"
                && configuration->linearUnit
                    == ngc::mesa::MesaLinearUnit::Millimeter
                && configuration->watchdogTimeoutNanoseconds
                    == 5'000'000
                && configuration->dpll.stepGeneratorTimer == 1
                && configuration->dpll
                    .stepGeneratorSampleOffsetNanoseconds == -500'000
                && configuration->dpll
                    .maximumPhaseErrorNanoseconds == 400'000
                && configuration->safety.has_value()
                && configuration->safety->enableInput
                    == "external_enable"
                && configuration->safety->polarity
                    == ngc::mesa::MesaSafetyPolarity::ActiveHigh
                && configuration->fieldInputs.size() == 5
                && configuration->stepGenerators.size() == 4,
                "proposed Mesa backend configuration did not load");
        const std::array expectedChannels{
            std::uint8_t{1}, std::uint8_t{2},
            std::uint8_t{3}, std::uint8_t{0},
        };
        for (std::size_t index = 0;
             index < expectedChannels.size(); ++index) {
            const auto &stepGenerator =
                configuration->stepGenerators[index];
            require(stepGenerator.joint == index
                    && stepGenerator.channel
                        == expectedChannels[index]
                    && stepGenerator.stepsPerUnit == 160.0
                    && stepGenerator.maximumCorrectionVelocity == 5.0
                    && stepGenerator.maximumGeneratedStepError == 2.0,
                    "Mesa StepGen configuration was decoded incorrectly");
        }
    }

    void testRejectsIncompatibleSevenI96IdentityAndModules() {
        auto wrongBoard = sevenI96Inventory();
        wrongBoard.idrom.boardName = "MESA7I95";
        auto wrongStepGenerator = sevenI96Inventory();
        const auto stepGenerator = std::ranges::find(
            wrongStepGenerator.modules, std::uint8_t{0x05},
            &ngc::mesa::HostMot2Module::tag);
        require(stepGenerator != wrongStepGenerator.modules.end(),
                "fixture has no StepGen module");
        stepGenerator->version = 1;

        const auto boardCapabilities =
            ngc::mesa::validateSevenI96Capabilities(wrongBoard);
        const auto moduleCapabilities =
            ngc::mesa::validateSevenI96Capabilities(
                wrongStepGenerator);

        require(!boardCapabilities.has_value()
                && boardCapabilities.error().find("MESA7I96")
                    != std::string::npos,
                "wrong 7I96 board identity was not rejected");
        require(!moduleCapabilities.has_value()
                && moduleCapabilities.error().find("version")
                    != std::string::npos,
                "incompatible StepGen version was not rejected");
    }

    void testRejectsIncompatibleSevenI96PinsAndSelections() {
        auto wrongPin = sevenI96Inventory();
        wrongPin.pins[18].secondaryPin = 0x83;

        const auto pinCapabilities =
            ngc::mesa::validateSevenI96Capabilities(wrongPin);
        const auto duplicateSelection =
            ngc::mesa::validateSevenI96Capabilities(
                sevenI96Inventory(), {
                    .stepGeneratorChannels = {0, 0},
                    .isolatedInputPins = {},
                    .isolatedOutputPins = {},
                    .encoderChannel = std::nullopt,
                });
        const auto outOfRangeSelection =
            ngc::mesa::validateSevenI96Capabilities(
                sevenI96Inventory(), {
                    .stepGeneratorChannels = {},
                    .isolatedInputPins = {11},
                    .isolatedOutputPins = {},
                    .encoderChannel = std::nullopt,
                });

        require(!pinCapabilities.has_value()
                && pinCapabilities.error().find("direction")
                    != std::string::npos,
                "incompatible step/direction pin was not rejected");
        require(!duplicateSelection.has_value()
                && duplicateSelection.error().find("duplicated")
                    != std::string::npos,
                "duplicate StepGen selection was not rejected");
        require(!outOfRangeSelection.has_value()
                && outOfRangeSelection.error().find("range")
                    != std::string::npos,
                "out-of-range isolated input selection was not rejected");
    }

    void testMeasuresReadOnlyCyclicLatencyAndAnomalies() {
        LatencyFixtureReader reader;

        const auto result = ngc::mesa::measureHostMot2ReadLatency(
            reader, {
                .sampleCount = 4,
                .period = std::chrono::microseconds(100),
            });

        require(result.has_value(),
                "valid latency measurement was rejected");
        require(result->attemptedSamples == 4
                && result->receivedResponses == 3
                && result->readFailures == 1
                && result->contentMismatches == 1,
                "latency anomaly counts are incorrect");
        require(result->roundTrip.sampleCount == 3,
                "round-trip distribution included a failed read");
        require(result->wakeLateness.sampleCount == 4,
                "wake distribution omitted an attempted cycle");
        require(result->roundTrip.minimum <= result->roundTrip.percentile50
                && result->roundTrip.percentile50
                    <= result->roundTrip.percentile95
                && result->roundTrip.percentile95
                    <= result->roundTrip.percentile99
                && result->roundTrip.percentile99
                    <= result->roundTrip.maximum,
                "round-trip percentiles are not ordered");
        require(result->lastReadError == "synthetic read failure",
                "last latency read failure was not retained");
    }

    void testRejectsInvalidLatencyConfiguration() {
        LatencyFixtureReader reader;

        const auto noSamples = ngc::mesa::measureHostMot2ReadLatency(
            reader, {
                .sampleCount = 0,
                .period = std::chrono::milliseconds(1),
            });
        const auto noPeriod = ngc::mesa::measureHostMot2ReadLatency(
            reader, {
                .sampleCount = 1,
                .period = std::chrono::nanoseconds::zero(),
            });

        require(!noSamples.has_value()
                && noSamples.error().find("sample count")
                    != std::string::npos,
                "zero latency sample count was not rejected");
        require(!noPeriod.has_value()
                && noPeriod.error().find("period")
                    != std::string::npos,
                "zero latency period was not rejected");
    }

    void testBuildsAndValidatesCyclicLbp16Transaction() {
        DatagramFixtureTransport transport;
        ngc::mesa::Lbp16CyclicTransaction transaction(transport);
        const auto write =
            transaction.addHostMot2Write(0x2000, 8);
        const auto read =
            transaction.addHostMot2Read(0x1000, 8);
        require(write.has_value() && read.has_value(),
                "valid cyclic operations were rejected");
        const std::array<std::byte, 8> output{
            std::byte{0x10}, std::byte{0x20},
            std::byte{0x30}, std::byte{0x40},
            std::byte{0x50}, std::byte{0x60},
            std::byte{0x70}, std::byte{0x80},
        };
        std::ranges::copy(
            output, transaction.writeData(*write).begin());
        require(transaction.finalize().has_value(),
                "valid cyclic transaction could not be finalized");

        const auto readSequence = std::uint32_t{0x1122'3344};
        const auto writeSequence = std::uint32_t{0x5566'7788};
        auto response = transport.response(transaction.responseSize());
        const std::array<std::byte, 8> input{
            std::byte{0x81}, std::byte{0x72},
            std::byte{0x63}, std::byte{0x54},
            std::byte{0x45}, std::byte{0x36},
            std::byte{0x27}, std::byte{0x18},
        };
        std::ranges::copy(input, response.begin());
        putLittleEndian32(response.subspan(8), readSequence);
        putLittleEndian32(response.subspan(12), writeSequence);
        putLittleEndian16(response.subspan(16), 0);

        const auto result =
            transaction.exchange(readSequence, writeSequence);

        require(result.fault == ngc::mesa::Lbp16CyclicFault::None
                && transaction.hasValidInputs(),
                "valid cyclic response was rejected");
        require(std::ranges::equal(
                    transaction.readData(*read), input),
                "validated cyclic input image is incorrect");
        const auto request = transport.request();
        require(request.size() == 36,
                "cyclic request has the wrong size");
        require(byteValue(request, 0) == 0x82
                && byteValue(request, 1) == 0xC2
                && byteValue(request, 2) == 0x00
                && byteValue(request, 3) == 0x20,
                "cyclic HostMot2 write command is incorrect");
        require(std::ranges::equal(
                    request.subspan(4, output.size()), output),
                "cyclic HostMot2 output payload is incorrect");
        require(byteValue(request, 12) == 0x82
                && byteValue(request, 13) == 0x42
                && byteValue(request, 14) == 0x00
                && byteValue(request, 15) == 0x10,
                "cyclic HostMot2 read command is incorrect");
        require(byteValue(request, 16) == 0x84
                && byteValue(request, 17) == 0xD1
                && byteValue(request, 18) == 0x10
                && byteValue(request, 19) == 0x00,
                "cyclic sequence write command is incorrect");
        require(byteValue(request, 28) == 0x84
                && byteValue(request, 29) == 0x51
                && byteValue(request, 32) == 0x01
                && byteValue(request, 33) == 0x59,
                "cyclic validation read commands are incorrect");
    }

    void testCyclicFailuresInvalidateInputs() {
        DatagramFixtureTransport transport;
        ngc::mesa::Lbp16CyclicTransaction transaction(transport);
        const auto read =
            transaction.addHostMot2Read(0x1000, 4);
        require(read.has_value() && transaction.finalize().has_value(),
                "cyclic failure fixture could not be built");
        auto response = transport.response(transaction.responseSize());
        putLittleEndian32(response, 0x1234'5678);
        putLittleEndian32(response.subspan(4), 1);
        putLittleEndian32(response.subspan(8), 1);

        require(transaction.exchange(1, 1).fault
                    == ngc::mesa::Lbp16CyclicFault::None
                && !transaction.readData(*read).empty(),
                "cyclic failure fixture did not establish valid input");

        putLittleEndian32(response.subspan(4), 0);
        const auto stale = transaction.exchange(2, 2);
        require(stale.fault
                    == ngc::mesa::Lbp16CyclicFault::ReadSequenceMismatch
                && stale.receivedReadSequence == 0
                && transaction.readData(*read).empty(),
                "stale cyclic response did not invalidate input");

        putLittleEndian32(response.subspan(4), 2);
        putLittleEndian32(response.subspan(8), 1);
        const auto unconfirmedWrite = transaction.exchange(2, 2);
        require(unconfirmedWrite.fault
                    == ngc::mesa::Lbp16CyclicFault::WriteSequenceMismatch
                && unconfirmedWrite.receivedWriteSequence == 1
                && transaction.readData(*read).empty(),
                "unconfirmed cyclic write did not invalidate input");

        putLittleEndian32(response.subspan(4), 3);
        putLittleEndian32(response.subspan(8), 3);
        putLittleEndian16(response.subspan(12), 0x0020);
        const auto boardError = transaction.exchange(3, 3);
        require(boardError.fault
                    == ngc::mesa::Lbp16CyclicFault::BoardProtocolError
                && boardError.boardError == 0x0020
                && transaction.readData(*read).empty(),
                "board protocol error did not invalidate input");

        transport.fail(
            ngc::mesa::Lbp16DatagramStatus::ReceiveFailed, 11);
        const auto transportError = transaction.exchange(4, 4);
        require(transportError.fault
                    == ngc::mesa::Lbp16CyclicFault::Transport
                && transportError.transport.systemError == 11
                && transaction.readData(*read).empty(),
                "transport failure did not invalidate input");
    }

    void testRejectsInvalidCyclicTransactionConstruction() {
        DatagramFixtureTransport transport;
        ngc::mesa::Lbp16CyclicTransaction transaction(transport);

        const auto unaligned =
            transaction.addHostMot2Read(0x1002, 4);
        const auto tooLarge =
            transaction.addHostMot2Write(0x2000, 512);

        require(!unaligned.has_value()
                && unaligned.error().find("aligned")
                    != std::string::npos,
                "unaligned cyclic read was not rejected");
        require(!tooLarge.has_value()
                && tooLarge.error().find("word command limit")
                    != std::string::npos,
                "oversized cyclic write was not rejected");
        require(transaction.finalize().has_value(),
                "empty validation-only cyclic transaction was rejected");
        require(!transaction.finalize().has_value(),
                "cyclic transaction accepted duplicate finalization");
        require(!transaction.addHostMot2Read(0x1000, 4).has_value(),
                "cyclic transaction accepted an operation after finalization");
    }

    std::unique_ptr<ngc::mesa::HostMot2CyclicIo>
    sevenI96CyclicIo(
        DatagramFixtureTransport &transport,
        const bool enableDpll = false) {
        const auto capabilities =
            ngc::mesa::validateSevenI96Capabilities(
                sevenI96Inventory());
        require(capabilities.has_value(),
                "cyclic fixture capability validation failed");
        const auto layout = ngc::mesa::sevenI96CyclicLayout(
            *capabilities, {
                .stepLengthNanoseconds = 1'000,
                .stepSpaceNanoseconds = 2'000,
                .directionSetupNanoseconds = 3'000,
                .directionHoldNanoseconds = 4'000,
            });
        auto configuration = ngc::mesa::HostMot2CyclicConfiguration{
            .watchdogTimeoutNanoseconds = 5'000'000,
            .dpll = enableDpll
                ? ngc::mesa::HostMot2DpllConfiguration{
                    .enabled = true,
                    .stepGeneratorTimer = 1,
                    .stepGeneratorSampleOffsetNanoseconds = -50'000,
                    .servoPeriodNanoseconds = 1'000'000,
                    .maximumPhaseErrorNanoseconds = 25'000,
                    .convergenceCycles = 1,
                }
                : ngc::mesa::HostMot2DpllConfiguration{},
        };
        configuration.isolatedOutputFrequencyHz[0] = 1'000'000;
        auto io = ngc::mesa::HostMot2CyclicIo::create(
            transport, layout, configuration);
        require(io.has_value(),
                "valid HostMot2 cyclic fixture was rejected");

        return std::move(*io);
    }

    void testInitializesHostMot2CyclicIoWithSafeOutputs() {
        DatagramFixtureTransport transport;
        auto io = sevenI96CyclicIo(transport);
        require(io->stepGeneratorCount() == 5
                && io->digitalInputCount() == 11
                && io->digitalOutputCount() == 6,
                "7I96 cyclic adapter exposed incorrect binding counts");
        auto response = transport.response(10);
        putCyclicConfirmation(response, 0, 1, 1);

        const auto initialized = io->initializeSafe();

        require(
            initialized.fault == ngc::mesa::HostMot2CyclicIoFault::None
                && io->initialized(),
            "safe HostMot2 initialization failed");
        const auto request = transport.request();
        const auto stepRates = findRequestWrite(request, 0x2000);
        const auto stepModes = findRequestWrite(request, 0x2200);
        const auto stepLength = findRequestWrite(request, 0x2500);
        const auto stepSpace = findRequestWrite(request, 0x2600);
        const auto directionSetup = findRequestWrite(request, 0x2300);
        const auto directionHold = findRequestWrite(request, 0x2400);
        const auto masterDds = findRequestWrite(request, 0x2900);
        const auto alternateSource = findRequestWrite(request, 0x1200);
        const auto ssrData = findRequestWrite(request, 0x7D00);
        const auto ssrRate = findRequestWrite(request, 0x7E00);
        const auto watchdogTimer = findRequestWrite(request, 0x0C00);
        const auto watchdogStatus = findRequestWrite(request, 0x0D00);
        const auto watchdogReset = findRequestWrite(request, 0x0E00);
        const auto dataDirection = findRequestWrite(request, 0x1100);

        require(std::ranges::all_of(
                    stepRates.data,
                    [](const std::byte value) {
                        return value == std::byte{};
                    })
                && std::ranges::all_of(
                    stepModes.data,
                    [](const std::byte value) {
                        return value == std::byte{};
                    }),
                "safe initialization commanded a nonzero StepGen rate or mode");
        for (std::size_t channel = 0; channel < 5; ++channel) {
            require(littleEndian32(
                        stepLength.data.subspan(channel * 4)) == 100
                    && littleEndian32(
                        stepSpace.data.subspan(channel * 4)) == 200
                    && littleEndian32(
                        directionSetup.data.subspan(channel * 4)) == 300
                    && littleEndian32(
                        directionHold.data.subspan(channel * 4)) == 400,
                    "safe initialization encoded incorrect StepGen timing");
        }
        require(littleEndian32(masterDds.data) == 0xFFFF'FFFF,
                "safe initialization did not set the StepGen master DDS");
        require(littleEndian32(alternateSource.data) == 0x0001'F800
                && littleEndian32(
                    alternateSource.data.subspan(4)) == 0x0000'03FF
                && littleEndian32(
                    alternateSource.data.subspan(8)) == 0,
                "safe initialization routed incorrect HostMot2 pins");
        require(std::ranges::equal(
                    alternateSource.data, dataDirection.data)
                && dataDirection.commandOffset
                    > alternateSource.commandOffset
                && dataDirection.commandOffset > ssrRate.commandOffset,
                "safe initialization did not write matching DDR last");
        require(littleEndian32(ssrData.data) == 0
                && littleEndian32(ssrRate.data) == 0,
                "safe initialization enabled an SSR output");
        require(littleEndian32(watchdogTimer.data) == 0x8000'0000
                && littleEndian32(watchdogStatus.data) == 0
                && littleEndian32(watchdogReset.data) == 0x5A00'0000,
                "safe initialization encoded incorrect watchdog state");
    }

    void testConfiguresAndGatesHostMot2Dpll() {
        const auto capabilities =
            ngc::mesa::validateSevenI96Capabilities(
                sevenI96Inventory());
        require(capabilities.has_value(),
                "DPLL fixture capability validation failed");
        const auto layout = ngc::mesa::sevenI96CyclicLayout(
            *capabilities, {
                .stepLengthNanoseconds = 1'000,
                .stepSpaceNanoseconds = 2'000,
                .directionSetupNanoseconds = 3'000,
                .directionHoldNanoseconds = 4'000,
            });
        auto configuration = ngc::mesa::HostMot2CyclicConfiguration{
            .watchdogTimeoutNanoseconds = 5'000'000,
            .dpll = {
                .enabled = true,
                .stepGeneratorTimer = 1,
                .stepGeneratorSampleOffsetNanoseconds = -50'000,
                .servoPeriodNanoseconds = 1'000'000,
                .maximumPhaseErrorNanoseconds = 25'000,
                .convergenceCycles = 2,
            },
        };
        configuration.isolatedOutputFrequencyHz[0] = 1'000'000;
        DatagramFixtureTransport transport;
        auto created = ngc::mesa::HostMot2CyclicIo::create(
            transport, layout, configuration);
        require(created.has_value(),
                "valid HostMot2 DPLL configuration was rejected");
        auto io = std::move(*created);
        auto response = transport.queueResponse(14);
        putLittleEndian32(response, 48);
        putCyclicConfirmation(response, 4, 1, 1);
        response = transport.queueResponse(10);
        putCyclicConfirmation(response, 0, 2, 2);

        const auto initialized = io->initializeSafe();

        require(
            initialized.fault == ngc::mesa::HostMot2CyclicIoFault::None,
            std::format(
                "HostMot2 DPLL initialization failed with fault {} "
                "and transaction fault {}",
                static_cast<int>(initialized.fault),
                static_cast<int>(initialized.transaction.fault)));
        const auto setupRequest = transport.request();
        require(
            littleEndian32(
                findRequestWrite(setupRequest, 0x7000).data)
                    == 2'814'749'767
                && littleEndian32(
                    findRequestWrite(setupRequest, 0x7200).data)
                    == 0x0140'0000
                && littleEndian32(
                    findRequestWrite(setupRequest, 0x7300).data)
                    == 0x07D0'0000,
            "HostMot2 DPLL base or control registers were encoded incorrectly");
        require(
            littleEndian32(
                findRequestWrite(setupRequest, 0x7400).data) == 0x0000'0CCC
                && littleEndian32(
                    findRequestWrite(setupRequest, 0x2A00).data)
                    == 0x0000'9000,
            "HostMot2 DPLL timer or StepGen latch selection was incorrect");

        auto outputs = ngc::mesa::HostMot2CyclicOutputImage{};
        outputs.stepGeneratorsEnabled = true;
        outputs.stepGenerators[0] = {
            .stepsPerSecond = 1'000.0,
            .enabled = true,
        };
        outputs.watchdogEnabled = true;
        for (std::uint32_t sequence = 3; sequence <= 5; ++sequence) {
            response = transport.response(50);
            putCyclicConfirmation(response, 40, sequence, sequence);

            const auto result = io->cycle(outputs);

            require(
                result.fault == ngc::mesa::HostMot2CyclicIoFault::None,
                "HostMot2 DPLL convergence cycle failed");
            const auto rate = littleEndian32(
                findRequestWrite(transport.request(), 0x2000).data);
            if (sequence < 5) {
                require(rate == 0,
                        "StepGen motion escaped before DPLL convergence");
            } else {
                require(
                    rate == static_cast<std::uint32_t>(
                        static_cast<std::int32_t>(42'949.67296)),
                    "StepGen motion remained gated after DPLL convergence");
            }
        }
        require(io->inputImage().dpll.enabled
                && io->inputImage().dpll.ready
                && io->inputImage().dpll.phaseErrorNanoseconds == 0,
                "DPLL readiness was not published in the input image");

        response = transport.response(50);
        putLittleEndian32(response.subspan(36), 128'849'019);
        putCyclicConfirmation(response, 40, 6, 6);

        const auto phaseFault = io->cycle(outputs);

        require(
            phaseFault.fault
                == ngc::mesa::HostMot2CyclicIoFault::DpllPhaseError
                && phaseFault.dpllPhaseErrorValid
                && phaseFault.dpllPhaseErrorNanoseconds == 30'000
                && io->fault()
                    == ngc::mesa::HostMot2CyclicIoFault::DpllPhaseError,
            "out-of-window DPLL phase error was not latched");
    }

    void testAcceptsBoardIndependentHostMot2CyclicLayout() {
        const auto capabilities =
            ngc::mesa::validateSevenI96Capabilities(
                sevenI96Inventory());
        require(capabilities.has_value(),
                "generic cyclic layout fixture validation failed");
        auto layout = ngc::mesa::sevenI96CyclicLayout(
            *capabilities, {
                .stepLengthNanoseconds = 1'000,
                .stepSpaceNanoseconds = 1'000,
                .directionSetupNanoseconds = 1'000,
                .directionHoldNanoseconds = 1'000,
            });
        layout.ioPort.descriptor.instances = 2;
        layout.ioPortWidth = 8;
        layout.stepGenerator.descriptor.instances = 2;
        layout.stepGeneratorCount = 1;
        layout.stepGenerators[0] = {
            .channel = 1,
            .stepPin = 8,
            .directionPin = 9,
            .invertDirection = true,
            .timing = layout.stepGenerators[0].timing,
        };
        layout.digitalInputCount = 1;
        layout.digitalInputs[0] = {
            .pin = 0,
        };
        layout.digitalOutputCount = 1;
        layout.digitalOutputs[0] = {
            .instance = 0,
            .output = 7,
            .pin = 1,
            .activeLow = false,
            .safeState = false,
        };
        auto configuration = ngc::mesa::HostMot2CyclicConfiguration{
            .watchdogTimeoutNanoseconds = 10'000'000,
            .dpll = {},
        };
        configuration.isolatedOutputFrequencyHz[0] = 500'000;
        DatagramFixtureTransport transport;

        auto io = ngc::mesa::HostMot2CyclicIo::create(
            transport, layout, configuration);

        require(io.has_value()
                && (*io)->stepGeneratorCount() == 1
                && (*io)->digitalInputCount() == 1
                && (*io)->digitalOutputCount() == 1,
                "HostMot2 cyclic I/O retained a 7I96-specific topology");
        auto response = transport.response(10);
        putCyclicConfirmation(response, 0, 1, 1);
        require(
            (*io)->initializeSafe().fault
                == ngc::mesa::HostMot2CyclicIoFault::None,
            "generic HostMot2 layout failed safe initialization");
        response = transport.response(30);
        putLittleEndian32(response.subspan(12), 0x0001'0000);
        putCyclicConfirmation(response, 20, 2, 2);
        auto outputs = ngc::mesa::HostMot2CyclicOutputImage{};
        outputs.stepGeneratorsEnabled = true;
        outputs.stepGenerators[0] = {
            .stepsPerSecond = 100.0,
            .enabled = true,
        };
        outputs.digitalOutputsEnabled = true;
        outputs.digitalOutputs[0] = true;
        outputs.watchdogEnabled = true;

        const auto result = (*io)->cycle(outputs);

        require(
            result.fault == ngc::mesa::HostMot2CyclicIoFault::None
                && result.inputsValid
                && !(*io)->inputImage().fieldDigitalInputs[0]
                && (*io)->inputImage().stepAccumulatorSubcounts[0]
                    == 65'536,
            "generic HostMot2 bindings were not applied to cyclic inputs");
        const auto request = transport.request();
        const auto stepRates = findRequestWrite(request, 0x2000);
        require(littleEndian32(stepRates.data) == 0
                && littleEndian32(stepRates.data.subspan(4))
                    == static_cast<std::uint32_t>(
                        static_cast<std::int32_t>(-4'294.967296)),
                "generic sparse/inverted StepGen binding was not applied");
        require(littleEndian32(
                    findRequestWrite(request, 0x7D00).data) == 0x80,
                "generic SSR output binding was not applied");

        auto noSsrLayout = layout;
        noSsrLayout.ssr = {};
        noSsrLayout.digitalOutputCount = 0;
        DatagramFixtureTransport noSsrTransport;
        const auto noSsrIo = ngc::mesa::HostMot2CyclicIo::create(
            noSsrTransport, noSsrLayout, configuration);
        require(noSsrIo.has_value()
                && (*noSsrIo)->digitalOutputCount() == 0,
                "HostMot2 cyclic I/O required a board-specific SSR module");
    }

    void testExchangesTypedHostMot2CyclicImages() {
        DatagramFixtureTransport transport;
        auto io = sevenI96CyclicIo(transport);
        auto response = transport.response(10);
        putCyclicConfirmation(response, 0, 1, 1);
        require(
            io->initializeSafe().fault
                == ngc::mesa::HostMot2CyclicIoFault::None,
            "cyclic image fixture initialization failed");

        response = transport.response(46);
        putLittleEndian32(response, 0x0000'0401);
        putLittleEndian32(response.subspan(12), 0x0001'0000);
        putLittleEndian32(response.subspan(16), 0xFFFF'0000);
        putLittleEndian32(response.subspan(20), 0);
        putLittleEndian32(response.subspan(24), 0x7FFF'FFFF);
        putLittleEndian32(response.subspan(28), 0x8000'0000);
        putLittleEndian32(response.subspan(32), 0);
        putCyclicConfirmation(response, 36, 2, 2);
        auto outputs = ngc::mesa::HostMot2CyclicOutputImage{};
        outputs.stepGeneratorsEnabled = true;
        outputs.stepGenerators[0] = {
            .stepsPerSecond = 1'000.0,
            .enabled = true,
        };
        outputs.stepGenerators[1] = {
            .stepsPerSecond = -500.0,
            .enabled = true,
        };
        outputs.digitalOutputsEnabled = true;
        outputs.digitalOutputs[0] = true;
        outputs.digitalOutputs[5] = true;
        outputs.watchdogEnabled = true;

        const auto result = io->cycle(outputs);

        require(
            result.fault == ngc::mesa::HostMot2CyclicIoFault::None
                && result.inputsValid,
            "valid HostMot2 cyclic image was rejected");
        const auto request = transport.request();
        const auto stepRates = findRequestWrite(request, 0x2000);
        const auto ssrData = findRequestWrite(request, 0x7D00);
        const auto ssrRate = findRequestWrite(request, 0x7E00);
        const auto watchdogTimer = findRequestWrite(request, 0x0C00);
        require(
            littleEndian32(stepRates.data)
                == static_cast<std::uint32_t>(
                    static_cast<std::int32_t>(42'949.67296))
                && littleEndian32(stepRates.data.subspan(4))
                    == static_cast<std::uint32_t>(
                        static_cast<std::int32_t>(-21'474.83648)),
            "cyclic StepGen rates use the wrong DDS conversion");
        require(littleEndian32(ssrData.data) == 0x21
                && littleEndian32(ssrRate.data) == 0x1030,
                "cyclic digital outputs use the wrong SSR image");
        require(littleEndian32(watchdogTimer.data) == 499'999,
                "cyclic watchdog timeout uses the wrong clock conversion");
        const auto &inputs = io->inputImage();
        require(inputs.fieldDigitalInputs[0]
                && inputs.fieldDigitalInputs[10]
                && !inputs.fieldDigitalInputs[1],
                "cyclic field digital inputs were decoded incorrectly");
        require(inputs.rawStepAccumulator[0] == 0x0001'0000
                && inputs.stepAccumulatorSubcounts[0] == 65'536
                && inputs.stepAccumulatorSubcounts[1] == -65'536,
                "cyclic StepGen accumulators were decoded incorrectly");
    }

    void testLatchesHostMot2WatchdogAndInvalidOutputFaults() {
        DatagramFixtureTransport watchdogTransport;
        auto watchdogIo = sevenI96CyclicIo(watchdogTransport);
        auto response = watchdogTransport.response(10);
        putCyclicConfirmation(response, 0, 1, 1);
        require(
            watchdogIo->initializeSafe().fault
                == ngc::mesa::HostMot2CyclicIoFault::None,
            "watchdog fixture initialization failed");
        response = watchdogTransport.response(46);
        putLittleEndian32(response.subspan(32), 1);
        putCyclicConfirmation(response, 36, 2, 2);

        const auto disabledResult = watchdogIo->cycle({});
        response = watchdogTransport.response(46);
        putLittleEndian32(response.subspan(32), 1);
        putCyclicConfirmation(response, 36, 3, 3);
        auto enabled = ngc::mesa::HostMot2CyclicOutputImage{};
        enabled.watchdogEnabled = true;
        const auto watchdogFault = watchdogIo->cycle(enabled);
        const auto repeatedWatchdogFault = watchdogIo->cycle({});

        require(
            disabledResult.fault
                    == ngc::mesa::HostMot2CyclicIoFault::None
                && disabledResult.inputsValid
                && watchdogFault.fault
                == ngc::mesa::HostMot2CyclicIoFault::WatchdogTripped
                && repeatedWatchdogFault.fault
                    == ngc::mesa::HostMot2CyclicIoFault::WatchdogTripped
                && !watchdogFault.inputsValid,
            "HostMot2 watchdog state was not gated and latched correctly");

        DatagramFixtureTransport invalidTransport;
        auto invalidIo = sevenI96CyclicIo(invalidTransport);
        response = invalidTransport.response(10);
        putCyclicConfirmation(response, 0, 1, 1);
        require(
            invalidIo->initializeSafe().fault
                == ngc::mesa::HostMot2CyclicIoFault::None,
            "invalid-output fixture initialization failed");
        auto invalid = ngc::mesa::HostMot2CyclicOutputImage{};
        invalid.stepGeneratorsEnabled = true;
        invalid.watchdogEnabled = true;
        invalid.stepGenerators[0] = {
            .stepsPerSecond =
                std::numeric_limits<double>::infinity(),
            .enabled = true,
        };

        const auto invalidFault = invalidIo->cycle(invalid);

        require(
            invalidFault.fault
                == ngc::mesa::HostMot2CyclicIoFault::InvalidOutput
                && invalidIo->fault()
                    == ngc::mesa::HostMot2CyclicIoFault::InvalidOutput,
            "invalid HostMot2 output was not rejected and latched");
    }

    void testExecutesBoundedDigitalIoProgram() {
        constexpr std::array<ngc::DigitalInputId, 3> logicalInputs{
            0, 1, 2,
        };
        constexpr std::array<ngc::DigitalOutputId, 2> declaredOutputs{
            3, 4,
        };
        constexpr std::array symbols{
            ngc::DigitalIoSymbol{
                .name = "probe_field",
                .kind = ngc::DigitalIoSymbolKind::FieldInput,
                .id = 0,
            },
            ngc::DigitalIoSymbol{
                .name = "probe",
                .kind = ngc::DigitalIoSymbolKind::LogicalInput,
                .id = 0,
            },
            ngc::DigitalIoSymbol{
                .name = "shared_home",
                .kind = ngc::DigitalIoSymbolKind::LogicalInput,
                .id = 1,
            },
            ngc::DigitalIoSymbol{
                .name = "y2_home",
                .kind = ngc::DigitalIoSymbolKind::LogicalInput,
                .id = 2,
            },
            ngc::DigitalIoSymbol{
                .name = "spindle_enable",
                .kind = ngc::DigitalIoSymbolKind::LogicalOutput,
                .id = 3,
            },
            ngc::DigitalIoSymbol{
                .name = "coolant",
                .kind = ngc::DigitalIoSymbolKind::LogicalOutput,
                .id = 4,
            },
        };
        auto program = ngc::DigitalIoProgram::compile(
            R"PROGRAM(
                not r0, probe_field
                mov r1, fieldin1
                and r2, r0, r1
                debounce probe, r2, 3ms
                or shared_home, fieldin0, fieldin1
                xor y2_home, fieldin0, fieldin1
                not fieldout0, spindle_enable
                mov fieldout1, coolant
            )PROGRAM",
            2, logicalInputs, 2, declaredOutputs, 0.001,
            symbols);
        require(program.has_value()
                && program->instructionCount() == 8
                && program->fieldInputCount() == 2
                && program->logicalInputCount() == 3
                && program->fieldOutputCount() == 2
                && program->logicalOutputCount() == 2,
                "valid digital I/O program was rejected");

        auto fieldInputs = ngc::FieldDigitalInputImage{};
        fieldInputs[1] = true;
        auto logicalOutputs = ngc::LogicalDigitalOutputImage{};
        logicalOutputs[4] = true;
        auto logical = ngc::LogicalDigitalInputImage{};
        program->executeInputs(
            fieldInputs, logicalOutputs, logical);
        require(logical[0] && logical[1] && logical[2],
                "digital-input program produced incorrect Boolean outputs");
        auto fieldOutputs = ngc::FieldDigitalOutputImage{};
        program->executeOutputs(
            fieldInputs, logicalOutputs, fieldOutputs);
        require(fieldOutputs[0] && fieldOutputs[1],
                "digital-output program produced incorrect Boolean outputs");

        fieldInputs[1] = false;
        logicalOutputs[3] = true;
        logicalOutputs[4] = false;
        program->executeInputs(
            fieldInputs, logicalOutputs, logical);
        require(logical[0] && !logical[1] && !logical[2],
                "debounce changed before its stable-time threshold");
        program->executeOutputs(
            fieldInputs, logicalOutputs, fieldOutputs);
        require(fieldOutputs.none(),
                "digital-output program did not observe logical outputs");
        program->executeInputs(
            fieldInputs, logicalOutputs, logical);
        require(logical[0],
                "debounce changed one tick before its threshold");
        program->executeInputs(
            fieldInputs, logicalOutputs, logical);
        require(!logical[0],
                "debounce did not change at its stable-time threshold");

        program->reset();
        program->executeInputs(
            fieldInputs, logicalOutputs, logical);
        require(!logical[0],
                "digital I/O program reset retained debounce state");
    }

    void testRejectsInvalidDigitalIoPrograms() {
        constexpr std::array<ngc::DigitalInputId, 1> logicalInput{0};
        constexpr std::array<ngc::DigitalOutputId, 1> logicalOutput{0};
        constexpr std::array duplicateSymbols{
            ngc::DigitalIoSymbol{
                .name = "duplicate",
                .kind = ngc::DigitalIoSymbolKind::LogicalInput,
                .id = 0,
            },
            ngc::DigitalIoSymbol{
                .name = "duplicate",
                .kind = ngc::DigitalIoSymbolKind::LogicalOutput,
                .id = 0,
            },
        };
        constexpr std::array reservedSymbol{
            ngc::DigitalIoSymbol{
                .name = "in0",
                .kind = ngc::DigitalIoSymbolKind::LogicalInput,
                .id = 0,
            },
        };
        const auto duplicateSymbol =
            ngc::DigitalIoProgram::compile(
                "mov in0, fieldin0",
                1, logicalInput, 0, {}, 0.001,
                duplicateSymbols);
        const auto reserved =
            ngc::DigitalIoProgram::compile(
                "mov in0, fieldin0",
                1, logicalInput, 0, {}, 0.001,
                reservedSymbol);
        const auto uninitialized =
            ngc::DigitalIoProgram::compile(
                "and r0, r1, fieldin0\nmov in0, r0",
                1, logicalInput, 0, {}, 0.001);
        const auto invalidFieldInput =
            ngc::DigitalIoProgram::compile(
                "mov in0, fieldin1",
                1, logicalInput, 0, {}, 0.001);
        const auto duplicateOutput =
            ngc::DigitalIoProgram::compile(
                "mov in0, fieldin0\nmov in0, 1",
                1, logicalInput, 0, {}, 0.001);
        const auto directDebounceOutput =
            ngc::DigitalIoProgram::compile(
                "debounce in0, fieldin0, 10ms",
                1, logicalInput, 0, {}, 0.001);
        const auto invalidDuration =
            ngc::DigitalIoProgram::compile(
                "debounce r0, fieldin0, forever\nmov in0, r0",
                1, logicalInput, 0, {}, 0.001);
        const auto readFieldOutput =
            ngc::DigitalIoProgram::compile(
                "mov in0, fieldout0\nmov fieldout0, out0",
                0, logicalInput, 1, logicalOutput, 0.001);
        const auto writeLogicalOutput =
            ngc::DigitalIoProgram::compile(
                "mov in0, 0\nmov out0, 1\nmov fieldout0, out0",
                0, logicalInput, 1, logicalOutput, 0.001);
        const auto missingFieldOutput =
            ngc::DigitalIoProgram::compile(
                "mov in0, 0",
                0, logicalInput, 1, logicalOutput, 0.001);
        const auto undeclaredLogicalOutput =
            ngc::DigitalIoProgram::compile(
                "mov in0, 0\nmov fieldout0, out1",
                0, logicalInput, 1, logicalOutput, 0.001);

        require(!uninitialized.has_value()
                && uninitialized.error().find("uninitialized")
                    != std::string::npos,
                "input program accepted an uninitialized register");
        require(!duplicateSymbol.has_value()
                && duplicateSymbol.error().find(
                    "declared more than once")
                    != std::string::npos,
                "digital I/O program accepted duplicate global names");
        require(!reserved.has_value()
                && reserved.error().find("reserved")
                    != std::string::npos,
                "digital I/O program accepted a reserved logical name");
        require(!invalidFieldInput.has_value()
                && invalidFieldInput.error().find("out of range")
                    != std::string::npos,
                "input program accepted an unavailable field input");
        require(!duplicateOutput.has_value()
                && duplicateOutput.error().find("more than once")
                    != std::string::npos,
                "input program accepted duplicate logical output");
        require(directDebounceOutput.has_value(),
                "digital I/O program rejected a direct debounce destination");
        require(!invalidDuration.has_value()
                && invalidDuration.error().find("invalid duration")
                    != std::string::npos,
                "input program accepted an invalid duration");
        require(!readFieldOutput.has_value()
                && readFieldOutput.error().find("write-only")
                    != std::string::npos,
                "digital I/O program read a field output");
        require(!writeLogicalOutput.has_value()
                && writeLogicalOutput.error().find("invalid destination")
                    != std::string::npos,
                "digital I/O program wrote a logical output");
        require(!missingFieldOutput.has_value()
                && missingFieldOutput.error().find(
                    "does not write field output")
                    != std::string::npos,
                "digital I/O program accepted an unassigned field output");
        require(!undeclaredLogicalOutput.has_value()
                && undeclaredLogicalOutput.error().find(
                    "undeclared logical output")
                    != std::string::npos,
                "digital I/O program read an undeclared logical output");
    }

    void testBridgesMesaInputsAndStepGeneratorsToExecutorIo() {
        DatagramFixtureTransport transport;
        auto io = sevenI96CyclicIo(transport, true);
        auto response = transport.queueResponse(14);
        putLittleEndian32(response, 48);
        putCyclicConfirmation(response, 4, 1, 1);
        response = transport.queueResponse(10);
        putCyclicConfirmation(response, 0, 2, 2);

        constexpr std::array<ngc::DigitalInputId, 1> logicalInput{0};
        constexpr std::array<ngc::DigitalOutputId, 1> logicalOutput{0};
        auto program = ngc::DigitalIoProgram::compile(
            R"PROGRAM(
                not r0, fieldin0
                mov in0, r0
                mov fieldout0, out0
                mov fieldout1, 0
                mov fieldout2, 0
                mov fieldout3, 0
                mov fieldout4, 0
                mov fieldout5, 0
            )PROGRAM",
            11, logicalInput, 6, logicalOutput, 0.001);
        require(program.has_value(),
                "Mesa executor digital I/O program did not compile");
        constexpr std::array mappings{
            ngc::mesa::MesaStepGeneratorMapping{
                .joint = 0,
                .stepGenerator = 0,
                .stepsPerMachineUnit = 4'000.0,
                .positionGainPerSecond = 1'000.0,
                .maximumCorrectionVelocity = 0.1,
                .maximumGeneratedStepError = 2.0,
            },
        };
        auto adapter = ngc::mesa::MesaProductionExecutorIo::create(
            std::move(io), std::move(*program), mappings,
            ngc::mesa::MesaExecutorSafetyInput{
                .input = 0,
                .requiredLevel = true,
            });
        require(adapter.has_value(),
                "valid Mesa executor I/O adapter was rejected");

        auto outputs = ngc::ProductionExecutorOutputState{};
        (*adapter)->applyOutputs(outputs);
        require(
            !(*adapter)->pendingOutputs().watchdogEnabled
                && !(*adapter)->pendingOutputs().stepGeneratorsEnabled
                && !(*adapter)->pendingOutputs().digitalOutputsEnabled,
            "disabled Mesa executor I/O did not retain safe outputs");

        outputs.executorEnabled = true;
        outputs.digitalOutputs[0] = true;
        (*adapter)->applyOutputs(outputs);
        require(
            (*adapter)->pendingOutputs().digitalOutputsEnabled
                && (*adapter)->pendingOutputs().digitalOutputs[0],
            "Mesa executor I/O did not stage its field output");
        response = transport.response(50);
        putCyclicConfirmation(response, 40, 3, 3);
        auto inputs = ngc::ProductionExecutorDigitalInputs{};

        (*adapter)->sampleDigitalInputs(inputs);

        require(inputs[0] && (*adapter)->faultCode() == 0,
                "Mesa field input program did not publish logical input zero");
        outputs.commandedJoints.position[0] = 0.000'012'5;
        outputs.commandedJoints.velocity[0] = 0.25;
        (*adapter)->applyOutputs(outputs);
        response = transport.response(50);
        putLittleEndian32(response.subspan(12), 3 * 65'536);
        putCyclicConfirmation(response, 40, 4, 4);

        (*adapter)->sampleDigitalInputs(inputs);

        const auto stepRates = findRequestWrite(
            transport.request(), 0x2000);
        require(littleEndian32(stepRates.data)
                    == static_cast<std::uint32_t>(
                        static_cast<std::int32_t>(42'949.67296)),
                "Mesa executor I/O mapped the wrong joint StepGen rate");
        require(littleEndian32(
                    findRequestWrite(
                        transport.request(), 0x7D00).data) == 1,
                "Mesa executor I/O did not map the field output to SSR data");

        outputs.commandedJoints.position[0] = 0.001'012'5;
        (*adapter)->applyOutputs(outputs);
        response = transport.response(50);
        putLittleEndian32(response.subspan(12), 3 * 65'536);
        putCyclicConfirmation(response, 40, 5, 5);

        (*adapter)->sampleDigitalInputs(inputs);

        require(
            littleEndian32(
                findRequestWrite(
                    transport.request(), 0x2000).data)
                == static_cast<std::uint32_t>(
                    static_cast<std::int32_t>(60'129.542144)),
            "Mesa StepGen position error did not produce bounded correction");

        response = transport.response(50);
        putLittleEndian32(response.subspan(32), 1);
        putCyclicConfirmation(response, 40, 6, 6);
        inputs.set();
        (*adapter)->sampleDigitalInputs(inputs);
        (*adapter)->applyOutputs(outputs);

        require((*adapter)->faultCode()
                    == (ngc::mesa::MESA_PRODUCTION_EXECUTOR_IO_FAULT_BASE
                        | static_cast<std::uint32_t>(
                            ngc::mesa::HostMot2CyclicIoFault::WatchdogTripped))
                && inputs.none()
                && !(*adapter)->pendingOutputs().watchdogEnabled
                && !(*adapter)->pendingOutputs().stepGeneratorsEnabled
                && !(*adapter)->pendingOutputs().digitalOutputsEnabled,
                "Mesa executor I/O did not latch its fault and stage safe outputs");
    }

    void testExternalEnableLossFaultsMesaExecutorIo() {
        DatagramFixtureTransport transport;
        auto io = sevenI96CyclicIo(transport, false);
        auto response = transport.queueResponse(10);
        putCyclicConfirmation(response, 0, 1, 1);

        constexpr std::array<ngc::DigitalInputId, 1> logicalInput{3};
        constexpr std::array<ngc::DigitalOutputId, 0> logicalOutputs{};
        auto program = ngc::DigitalIoProgram::compile(
            "mov in3, fieldin2",
            11, logicalInput, 0, logicalOutputs, 0.001);
        require(program.has_value(),
                "external-enable digital I/O program did not compile");
        auto adapter = ngc::mesa::MesaProductionExecutorIo::create(
            std::move(io), std::move(*program), {},
            ngc::mesa::MesaExecutorSafetyInput{
                .input = 3,
                .requiredLevel = true,
            });
        require(adapter.has_value(),
                "external-enable Mesa executor adapter was rejected");

        response = transport.response(46);
        putCyclicConfirmation(response, 36, 2, 2);
        auto inputs = ngc::ProductionExecutorDigitalInputs{};
        inputs.set();

        (*adapter)->sampleDigitalInputs(inputs);
        auto outputs = ngc::ProductionExecutorOutputState{
            .digitalOutputs = {},
            .executorEnabled = true,
        };
        (*adapter)->applyOutputs(outputs);

        require(
            (*adapter)->faultCode()
                    == ngc::mesa::MESA_EXTERNAL_ENABLE_FAULT
                && inputs.none()
                && !(*adapter)->pendingOutputs().watchdogEnabled
                && !(*adapter)->pendingOutputs().stepGeneratorsEnabled
                && !(*adapter)->pendingOutputs().digitalOutputsEnabled,
            "inactive external enable did not latch safe Mesa outputs");
    }

    void testRebasesStationaryMesaCoordinatesAndFaultsFollowingError() {
        DatagramFixtureTransport transport;
        auto io = sevenI96CyclicIo(transport, true);
        auto response = transport.queueResponse(14);
        putLittleEndian32(response, 48);
        putCyclicConfirmation(response, 4, 1, 1);
        response = transport.queueResponse(10);
        putCyclicConfirmation(response, 0, 2, 2);
        constexpr std::array<ngc::DigitalInputId, 0> logicalInputs{};
        constexpr std::array<ngc::DigitalOutputId, 0> logicalOutputs{};
        auto program = ngc::DigitalIoProgram::compile(
            "", 0, logicalInputs, 0, logicalOutputs, 0.001);
        require(program.has_value(),
                "empty Mesa digital I/O program did not compile");
        constexpr std::array mappings{
            ngc::mesa::MesaStepGeneratorMapping{
                .joint = 0,
                .stepGenerator = 0,
                .stepsPerMachineUnit = 4'000.0,
                .positionGainPerSecond = 100.0,
                .maximumCorrectionVelocity = 0.1,
                .maximumGeneratedStepError = 2.0,
            },
        };
        auto adapter = ngc::mesa::MesaProductionExecutorIo::create(
            std::move(io), std::move(*program), mappings);
        require(adapter.has_value(),
                "following-error Mesa adapter fixture was rejected");
        auto outputs = ngc::ProductionExecutorOutputState{};
        outputs.executorEnabled = true;
        (*adapter)->applyOutputs(outputs);
        response = transport.response(50);
        putCyclicConfirmation(response, 40, 3, 3);
        auto inputs = ngc::ProductionExecutorDigitalInputs{};
        (*adapter)->sampleDigitalInputs(inputs);

        outputs.commandedJoints.position[0] = 0.2;
        (*adapter)->applyOutputs(outputs);

        require((*adapter)->faultCode() == 0
                && (*adapter)->pendingOutputs()
                    .stepGenerators[0].stepsPerSecond == 0.0,
                "stationary joint-coordinate assignment did not rebase "
                "Mesa accumulator feedback");

        outputs.commandedJoints.position[0] = 0.4;
        outputs.commandedJoints.velocity[0] = 0.1;
        (*adapter)->applyOutputs(outputs);

        require(
            (*adapter)->faultCode()
                == ngc::mesa::MESA_STEPGEN_FOLLOWING_ERROR_FAULT
                && !(*adapter)->pendingOutputs().watchdogEnabled
                && !(*adapter)->pendingOutputs()
                    .stepGeneratorsEnabled,
            "Mesa following error did not latch a safe-output fault");
    }

    void testMesaAccumulatorFeedbackPreventsCumulativeJitterDrift() {
        DatagramFixtureTransport transport;
        auto io = sevenI96CyclicIo(transport, true);
        auto response = transport.queueResponse(14);
        putLittleEndian32(response, 48);
        putCyclicConfirmation(response, 4, 1, 1);
        response = transport.queueResponse(10);
        putCyclicConfirmation(response, 0, 2, 2);
        constexpr std::array<ngc::DigitalInputId, 0> logicalInputs{};
        constexpr std::array<ngc::DigitalOutputId, 0> logicalOutputs{};
        auto program = ngc::DigitalIoProgram::compile(
            "", 0, logicalInputs, 0, logicalOutputs, 0.001);
        require(program.has_value(),
                "jitter-feedback digital I/O program did not compile");
        constexpr auto stepsPerUnit = 4'000.0;
        constexpr std::array mappings{
            ngc::mesa::MesaStepGeneratorMapping{
                .joint = 0,
                .stepGenerator = 0,
                .stepsPerMachineUnit = stepsPerUnit,
                .positionGainPerSecond = 100.0,
                .maximumCorrectionVelocity = 0.1,
                .maximumGeneratedStepError = 2.0,
            },
        };
        auto adapter = ngc::mesa::MesaProductionExecutorIo::create(
            std::move(io), std::move(*program), mappings);
        require(adapter.has_value(),
                "jitter-feedback Mesa adapter fixture was rejected");
        auto outputs = ngc::ProductionExecutorOutputState{};
        outputs.executorEnabled = true;
        (*adapter)->applyOutputs(outputs);
        response = transport.response(50);
        putCyclicConfirmation(response, 40, 3, 3);
        auto inputs = ngc::ProductionExecutorDigitalInputs{};
        (*adapter)->sampleDigitalInputs(inputs);

        constexpr auto servoPeriod = 0.001;
        constexpr auto velocity = 0.25;
        constexpr auto iterationCount = 200;
        auto accumulatorSubcounts = 0.0;
        auto midpointTrackingOffset = 0.0;
        auto lastStepsPerSecond = 0.0;
        for (auto iteration = 1;
             iteration <= iterationCount; ++iteration) {
            outputs.commandedJoints.position[0] =
                velocity * servoPeriod * iteration;
            outputs.commandedJoints.velocity[0] = velocity;
            (*adapter)->applyOutputs(outputs);
            response = transport.response(50);
            putLittleEndian32(
                response.subspan(12),
                static_cast<std::uint32_t>(
                    std::llround(accumulatorSubcounts)));
            const auto phaseErrorNanoseconds =
                iteration % 3 == 0
                ? 10'000
                : iteration % 3 == 1
                    ? -10'000
                    : 0;
            const auto rawPhase = static_cast<std::int32_t>(
                static_cast<double>(phaseErrorNanoseconds)
                * 4'294'967'296.0 / 1'000'000.0);
            putLittleEndian32(
                response.subspan(36),
                std::bit_cast<std::uint32_t>(rawPhase));
            const auto sequence =
                static_cast<std::uint32_t>(iteration + 3);
            putCyclicConfirmation(
                response, 40, sequence, sequence);

            (*adapter)->sampleDigitalInputs(inputs);

            require((*adapter)->faultCode() == 0,
                    "bounded DPLL phase variation faulted accumulator feedback");
            const auto rateWord = littleEndian32(
                findRequestWrite(
                    transport.request(), 0x2000).data);
            const auto rate = std::bit_cast<std::int32_t>(
                rateWord);
            const auto stepsPerSecond =
                static_cast<double>(rate)
                * 100'000'000.0 / 4'294'967'296.0;
            lastStepsPerSecond = stepsPerSecond;
            accumulatorSubcounts +=
                stepsPerSecond * 65'536.0 * servoPeriod;
            if (iteration == iterationCount / 2) {
                midpointTrackingOffset =
                    accumulatorSubcounts
                        / (stepsPerUnit * 65'536.0)
                    - outputs.commandedJoints.position[0];
            }
        }

        const auto actualPosition =
            accumulatorSubcounts
            / (stepsPerUnit * 65'536.0);
        const auto finalTrackingOffset =
            actualPosition
            - outputs.commandedJoints.position[0];
        require(
            std::abs(
                finalTrackingOffset - midpointTrackingOffset)
                < 2.0 / (stepsPerUnit * 65'536.0),
            std::format(
                "DPLL-latched accumulator feedback accumulated position "
                "drift: midpoint offset {} final offset {}",
                midpointTrackingOffset, finalTrackingOffset));
        require(std::abs(finalTrackingOffset)
                    < velocity * servoPeriod * 2.1
                && std::abs(lastStepsPerSecond - 1'000.0) < 0.1,
                "Mesa accumulator feedback did not settle to bounded "
                "nominal-rate tracking");
    }
}

int main() {
    try {
        testDiscoversTypedSevenI96Inventory();
        testRejectsWrongCookie();
        testRejectsInconsistentPinTopology();
        testRejectsUnterminatedModuleDescriptors();
        testRejectsOverflowingDescriptorOffset();
        testValidatesSevenI96CapabilitiesAndSelections();
        testLoadsProposedMesaBackendConfiguration();
        testRejectsIncompatibleSevenI96IdentityAndModules();
        testRejectsIncompatibleSevenI96PinsAndSelections();
        testMeasuresReadOnlyCyclicLatencyAndAnomalies();
        testRejectsInvalidLatencyConfiguration();
        testBuildsAndValidatesCyclicLbp16Transaction();
        testCyclicFailuresInvalidateInputs();
        testRejectsInvalidCyclicTransactionConstruction();
        testInitializesHostMot2CyclicIoWithSafeOutputs();
        testConfiguresAndGatesHostMot2Dpll();
        testAcceptsBoardIndependentHostMot2CyclicLayout();
        testExchangesTypedHostMot2CyclicImages();
        testLatchesHostMot2WatchdogAndInvalidOutputFaults();
        testExecutesBoundedDigitalIoProgram();
        testRejectsInvalidDigitalIoPrograms();
        testBridgesMesaInputsAndStepGeneratorsToExecutorIo();
        testExternalEnableLossFaultsMesaExecutorIo();
        testRebasesStationaryMesaCoordinatesAndFaultsFollowingError();
        testMesaAccumulatorFeedbackPreventsCumulativeJitterDrift();
    } catch (const std::exception &error) {
        std::cerr << "Mesa HostMot2 test failure: "
                  << error.what() << '\n';

        return 1;
    }

    std::cout << "Mesa HostMot2 tests passed\n";

    return 0;
}
