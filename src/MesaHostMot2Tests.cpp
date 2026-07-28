#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

#include "mesa/HostMot2Discovery.h"
#include "mesa/HostMot2Latency.h"
#include "mesa/Lbp16CyclicTransaction.h"
#include "mesa/SevenI96Capabilities.h"

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
            if (response.size() != m_responseSize) {
                return {
                    .status =
                        ngc::mesa::Lbp16DatagramStatus::UnexpectedResponseSize,
                    .systemError = 0,
                    .sentBytes = request.size(),
                    .receivedBytes = m_responseSize,
                };
            }

            std::ranges::copy(
                std::span(m_response).first(m_responseSize),
                response.begin());

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
        std::size_t m_requestSize = 0;
        std::size_t m_responseSize = 0;
        ngc::mesa::Lbp16DatagramStatus m_status =
            ngc::mesa::Lbp16DatagramStatus::Complete;
        int m_systemError = 0;
    };

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
}

int main() {
    try {
        testDiscoversTypedSevenI96Inventory();
        testRejectsWrongCookie();
        testRejectsInconsistentPinTopology();
        testRejectsUnterminatedModuleDescriptors();
        testRejectsOverflowingDescriptorOffset();
        testValidatesSevenI96CapabilitiesAndSelections();
        testRejectsIncompatibleSevenI96IdentityAndModules();
        testRejectsIncompatibleSevenI96PinsAndSelections();
        testMeasuresReadOnlyCyclicLatencyAndAnomalies();
        testRejectsInvalidLatencyConfiguration();
        testBuildsAndValidatesCyclicLbp16Transaction();
        testCyclicFailuresInvalidateInputs();
        testRejectsInvalidCyclicTransactionConstruction();
    } catch (const std::exception &error) {
        std::cerr << "Mesa HostMot2 test failure: "
                  << error.what() << '\n';

        return 1;
    }

    std::cout << "Mesa HostMot2 tests passed\n";

    return 0;
}
