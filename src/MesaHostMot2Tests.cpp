#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

#include "mesa/HostMot2Discovery.h"

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
            reader, 2, 0x05, 2, 1, 5, 0x2000, 10, 0, 0x1FF);

        for (std::uint32_t index = 0; index < 51; ++index) {
            const auto address = PIN_ADDRESS + index * 4;
            reader.putByte(address + 3, 0x03);
        }
        reader.putByte(PIN_ADDRESS + 17 * 4, 0x81);
        reader.putByte(PIN_ADDRESS + 17 * 4 + 1, 0x05);
        reader.putByte(PIN_ADDRESS + 17 * 4 + 2, 0);

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
        require(inventory->modules.size() == 3,
                "module sentinel was not honored");
        require(inventory->modules[2].tag == 0x05
                && inventory->modules[2].instances == 5
                && inventory->modules[2].baseAddress == 0x2000,
                "StepGen descriptor was not decoded");
        require(
            ngc::mesa::hostMot2ModuleClockHz(
                *inventory, inventory->modules[2]) == 100'000'000,
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
        for (std::size_t index = 3; index < 32; ++index) {
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
}

int main() {
    try {
        testDiscoversTypedSevenI96Inventory();
        testRejectsWrongCookie();
        testRejectsInconsistentPinTopology();
        testRejectsUnterminatedModuleDescriptors();
        testRejectsOverflowingDescriptorOffset();
    } catch (const std::exception &error) {
        std::cerr << "Mesa HostMot2 test failure: "
                  << error.what() << '\n';

        return 1;
    }

    std::cout << "Mesa HostMot2 tests passed\n";

    return 0;
}
