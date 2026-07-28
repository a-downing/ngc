#include "mesa/HostMot2Discovery.h"

#include <array>
#include <format>
#include <limits>

namespace ngc::mesa {
    namespace {
        constexpr std::uint32_t HOSTMOT2_COOKIE_ADDRESS = 0x0100;
        constexpr std::uint32_t HOSTMOT2_CONFIGURATION_NAME_ADDRESS = 0x0104;
        constexpr std::uint32_t HOSTMOT2_IDROM_ADDRESS_ADDRESS = 0x010C;
        constexpr std::uint32_t HOSTMOT2_COOKIE = 0x55AACAFE;
        constexpr std::size_t CONFIGURATION_NAME_SIZE = 8;
        constexpr std::size_t IDROM_SIZE = 64;
        constexpr std::size_t MODULE_SIZE = 12;
        constexpr std::size_t PIN_SIZE = 4;
        constexpr std::size_t MAX_MODULES = 32;
        constexpr std::size_t MAX_PINS = 144;
        constexpr std::uint32_t HOSTMOT2_ADDRESS_SPACE_SIZE = 0x1'0000;

        std::uint16_t littleEndian16(
            const std::span<const std::byte> bytes,
            const std::size_t offset) noexcept {
            return static_cast<std::uint16_t>(
                std::to_integer<std::uint8_t>(bytes[offset]))
                | static_cast<std::uint16_t>(
                    std::to_integer<std::uint8_t>(bytes[offset + 1]))
                    << 8;
        }

        std::uint32_t littleEndian32(
            const std::span<const std::byte> bytes,
            const std::size_t offset) noexcept {
            return static_cast<std::uint32_t>(
                std::to_integer<std::uint8_t>(bytes[offset]))
                | static_cast<std::uint32_t>(
                    std::to_integer<std::uint8_t>(bytes[offset + 1]))
                    << 8
                | static_cast<std::uint32_t>(
                    std::to_integer<std::uint8_t>(bytes[offset + 2]))
                    << 16
                | static_cast<std::uint32_t>(
                    std::to_integer<std::uint8_t>(bytes[offset + 3]))
                    << 24;
        }

        std::string fixedString(const std::span<const std::byte> bytes) {
            std::string result;
            result.reserve(bytes.size());
            for (const auto value : bytes) {
                const auto character =
                    static_cast<char>(std::to_integer<unsigned char>(value));
                if (character == '\0') {
                    break;
                }
                result.push_back(character);
            }
            while (!result.empty() && result.back() == ' ') {
                result.pop_back();
            }

            return result;
        }

        std::expected<void, std::string> validateRegion(
            const std::uint32_t address, const std::size_t size,
            const std::string_view description) {
            if (address >= HOSTMOT2_ADDRESS_SPACE_SIZE
                || size > HOSTMOT2_ADDRESS_SPACE_SIZE - address) {
                return std::unexpected(std::format(
                    "{} at 0x{:X} with size {} exceeds the HostMot2 address space",
                    description, address, size));
            }

            return {};
        }

        std::expected<std::uint32_t, std::string> relativeAddress(
            const std::uint32_t base, const std::uint32_t offset,
            const std::string_view description) {
            if (base >= HOSTMOT2_ADDRESS_SPACE_SIZE
                || offset > HOSTMOT2_ADDRESS_SPACE_SIZE - base) {
                return std::unexpected(std::format(
                    "{} offset 0x{:X} from 0x{:X} exceeds the HostMot2 address space",
                    description, offset, base));
            }

            return base + offset;
        }

        std::expected<void, std::string> readRegion(
            HostMot2RegisterReader &reader, const std::uint32_t address,
            const std::span<std::byte> destination,
            const std::string_view description) {
            const auto valid =
                validateRegion(address, destination.size(), description);
            if (!valid) {
                return std::unexpected(valid.error());
            }
            const auto read = reader.read(address, destination);
            if (!read) {
                return std::unexpected(std::format(
                    "failed to read {} at 0x{:04X}: {}",
                    description, address, read.error()));
            }

            return {};
        }

        HostMot2Idrom parseIdrom(
            const std::span<const std::byte> bytes) {
            HostMot2Idrom result{
                .type = littleEndian32(bytes, 0),
                .moduleOffset = littleEndian32(bytes, 4),
                .pinOffset = littleEndian32(bytes, 8),
                .boardName = fixedString(bytes.subspan(12, 8)),
                .fpgaSize = littleEndian32(bytes, 20),
                .fpgaPins = littleEndian32(bytes, 24),
                .ioPorts = littleEndian32(bytes, 28),
                .ioWidth = littleEndian32(bytes, 32),
                .portWidth = littleEndian32(bytes, 36),
                .clockLowHz = littleEndian32(bytes, 40),
                .clockHighHz = littleEndian32(bytes, 44),
                .instanceStride0 = littleEndian32(bytes, 48),
                .instanceStride1 = littleEndian32(bytes, 52),
                .registerStride0 = littleEndian32(bytes, 56),
                .registerStride1 = littleEndian32(bytes, 60),
            };

            return result;
        }

        HostMot2Module parseModule(
            const std::span<const std::byte> bytes) noexcept {
            return {
                .tag = std::to_integer<std::uint8_t>(bytes[0]),
                .version = std::to_integer<std::uint8_t>(bytes[1]),
                .clockTag = std::to_integer<std::uint8_t>(bytes[2]),
                .instances = std::to_integer<std::uint8_t>(bytes[3]),
                .baseAddress = littleEndian16(bytes, 4),
                .registers = std::to_integer<std::uint8_t>(bytes[6]),
                .strides = std::to_integer<std::uint8_t>(bytes[7]),
                .multipleRegisterBitmap = littleEndian32(bytes, 8),
            };
        }

        HostMot2Pin parsePin(
            const std::span<const std::byte> bytes) noexcept {
            return {
                .secondaryPin = std::to_integer<std::uint8_t>(bytes[0]),
                .secondaryTag = std::to_integer<std::uint8_t>(bytes[1]),
                .secondaryChannel = std::to_integer<std::uint8_t>(bytes[2]),
                .primaryTag = std::to_integer<std::uint8_t>(bytes[3]),
            };
        }

        std::expected<void, std::string> validateIdrom(
            const std::uint32_t address, const HostMot2Idrom &idrom) {
            if (idrom.type == 0) {
                return std::unexpected("HostMot2 IDROM type must not be zero");
            }
            if (idrom.boardName.empty()) {
                return std::unexpected("HostMot2 IDROM board name is empty");
            }
            if (idrom.ioPorts == 0 || idrom.portWidth == 0) {
                return std::unexpected(
                    "HostMot2 IDROM must describe at least one non-empty I/O port");
            }
            const auto describedWidth =
                static_cast<std::uint64_t>(idrom.ioPorts)
                * idrom.portWidth;
            if (idrom.ioWidth != describedWidth) {
                return std::unexpected(std::format(
                    "HostMot2 IDROM I/O width {} does not equal {} ports times width {}",
                    idrom.ioWidth, idrom.ioPorts, idrom.portWidth));
            }
            if (idrom.ioWidth > MAX_PINS) {
                return std::unexpected(std::format(
                    "HostMot2 IDROM describes {} pins, exceeding the supported maximum {}",
                    idrom.ioWidth, MAX_PINS));
            }
            if (idrom.clockLowHz == 0 || idrom.clockHighHz == 0) {
                return std::unexpected(
                    "HostMot2 IDROM clock frequencies must be positive");
            }
            if (idrom.instanceStride0 == 0 || idrom.instanceStride1 == 0
                || idrom.registerStride0 == 0
                || idrom.registerStride1 == 0) {
                return std::unexpected(
                    "HostMot2 IDROM strides must be positive");
            }

            const auto moduleAddress = relativeAddress(
                address, idrom.moduleOffset,
                "HostMot2 module descriptors");
            if (!moduleAddress) {
                return std::unexpected(moduleAddress.error());
            }
            const auto modules = validateRegion(
                *moduleAddress,
                MAX_MODULES * MODULE_SIZE, "HostMot2 module descriptors");
            if (!modules) {
                return std::unexpected(modules.error());
            }
            const auto pinAddress = relativeAddress(
                address, idrom.pinOffset,
                "HostMot2 pin descriptors");
            if (!pinAddress) {
                return std::unexpected(pinAddress.error());
            }
            const auto pins = validateRegion(
                *pinAddress,
                idrom.ioWidth * PIN_SIZE, "HostMot2 pin descriptors");
            if (!pins) {
                return std::unexpected(pins.error());
            }

            return {};
        }
    }

    bool HostMot2Pin::secondaryOutput() const noexcept {
        return (secondaryPin & 0x80) != 0;
    }

    std::uint8_t HostMot2Pin::secondaryPinNumber() const noexcept {
        return secondaryPin & 0x7F;
    }

    bool HostMot2Pin::globalSecondaryChannel() const noexcept {
        return (secondaryChannel & 0x80) != 0;
    }

    std::uint8_t HostMot2Pin::secondaryChannelNumber() const noexcept {
        return secondaryChannel & 0x7F;
    }

    std::expected<HostMot2Inventory, std::string>
    discoverHostMot2(HostMot2RegisterReader &reader) {
        std::array<std::byte, 4> cookieBytes{};
        if (const auto read = readRegion(
                reader, HOSTMOT2_COOKIE_ADDRESS, cookieBytes,
                "HostMot2 cookie"); !read) {
            return std::unexpected(read.error());
        }
        const auto cookie = littleEndian32(cookieBytes, 0);
        if (cookie != HOSTMOT2_COOKIE) {
            return std::unexpected(std::format(
                "HostMot2 cookie mismatch: expected 0x{:08X}, received 0x{:08X}",
                HOSTMOT2_COOKIE, cookie));
        }

        std::array<std::byte, CONFIGURATION_NAME_SIZE> configurationBytes{};
        if (const auto read = readRegion(
                reader, HOSTMOT2_CONFIGURATION_NAME_ADDRESS,
                configurationBytes, "HostMot2 configuration name"); !read) {
            return std::unexpected(read.error());
        }
        const auto configurationName = fixedString(configurationBytes);
        if (configurationName != "HOSTMOT2") {
            return std::unexpected(std::format(
                "unsupported HostMot2 configuration name '{}'",
                configurationName));
        }

        std::array<std::byte, 4> addressBytes{};
        if (const auto read = readRegion(
                reader, HOSTMOT2_IDROM_ADDRESS_ADDRESS, addressBytes,
                "HostMot2 IDROM address"); !read) {
            return std::unexpected(read.error());
        }
        const auto idromAddress = littleEndian32(addressBytes, 0);
        if ((idromAddress & 0x3) != 0) {
            return std::unexpected(std::format(
                "HostMot2 IDROM address 0x{:X} is not 32-bit aligned",
                idromAddress));
        }

        std::array<std::byte, IDROM_SIZE> idromBytes{};
        if (const auto read = readRegion(
                reader, idromAddress, idromBytes,
                "HostMot2 IDROM"); !read) {
            return std::unexpected(read.error());
        }
        auto inventory = HostMot2Inventory{
            .configurationName = configurationName,
            .idromAddress = idromAddress,
            .idrom = parseIdrom(idromBytes),
            .modules = {},
            .pins = {},
        };
        if (const auto valid =
                validateIdrom(idromAddress, inventory.idrom); !valid) {
            return std::unexpected(valid.error());
        }

        std::array<std::byte, MAX_MODULES * MODULE_SIZE> moduleBytes{};
        const auto moduleAddress =
            idromAddress + inventory.idrom.moduleOffset;
        if (const auto read = readRegion(
                reader, moduleAddress, moduleBytes,
                "HostMot2 module descriptors"); !read) {
            return std::unexpected(read.error());
        }
        for (std::size_t index = 0; index < MAX_MODULES; ++index) {
            const auto module = parseModule(std::span(moduleBytes).subspan(
                index * MODULE_SIZE, MODULE_SIZE));
            if (module.tag == 0) {
                break;
            }
            if (module.instances == 0 || module.registers == 0) {
                return std::unexpected(std::format(
                    "HostMot2 module descriptor {} has a zero instance or register count",
                    index));
            }
            inventory.modules.push_back(module);
        }
        if (inventory.modules.size() == MAX_MODULES) {
            return std::unexpected(
                "HostMot2 module descriptors have no terminating null tag");
        }

        std::vector<std::byte> pinBytes(
            inventory.idrom.ioWidth * PIN_SIZE);
        const auto pinAddress = idromAddress + inventory.idrom.pinOffset;
        if (const auto read = readRegion(
                reader, pinAddress, pinBytes,
                "HostMot2 pin descriptors"); !read) {
            return std::unexpected(read.error());
        }
        inventory.pins.reserve(inventory.idrom.ioWidth);
        for (std::size_t index = 0;
             index < inventory.idrom.ioWidth; ++index) {
            inventory.pins.push_back(parsePin(std::span(pinBytes).subspan(
                index * PIN_SIZE, PIN_SIZE)));
        }

        return inventory;
    }

    std::string_view hostMot2ModuleName(const std::uint8_t tag) noexcept {
        switch (tag) {
            case 0x00: return "None";
            case 0x01: return "IRQ";
            case 0x02: return "Watchdog";
            case 0x03: return "IOPort";
            case 0x04: return "Encoder";
            case 0x05: return "StepGen";
            case 0x06: return "PWMGen";
            case 0x07: return "SPI";
            case 0x08: return "SSI";
            case 0x09: return "UARTTx";
            case 0x0A: return "UARTRx";
            case 0x0B: return "TranslationRAM";
            case 0x0C: return "MuxedEncoder";
            case 0x0D: return "MuxedEncoderSelect";
            case 0x0E: return "BufferedSPI";
            case 0x0F: return "DBSPI";
            case 0x10: return "DPLL";
            case 0x13: return "ThreePhasePWM";
            case 0x14: return "WaveGen";
            case 0x15: return "DAQFIFO";
            case 0x18: return "BiSS";
            case 0x19: return "FanucAbs";
            case 0x1A: return "DPLL";
            case 0x1B: return "PacketUARTTx";
            case 0x1C: return "PacketUARTRx";
            case 0x1E: return "InputMux";
            case 0x1F: return "Sigma5";
            case 0x23: return "InM";
            case 0x2C: return "RCPWMGen";
            case 0x2D: return "OutM";
            case 0x40: return "LocalIOPort";
            case 0x80: return "LED";
            case 0xC0: return "Resolver";
            case 0xC1: return "SSerial";
            case 0xC2: return "Twiddler";
            case 0xC3: return "SSR";
            case 0xC4: return "CPDrive";
            case 0xC5: return "DSAD";
            case 0xC6: return "SSerialB";
            case 0xC7: return "OneShot";
            case 0xC8: return "Period";
            case 0xC9: return "BufferedSPID";
            default: return "Unknown";
        }
    }

    std::uint32_t hostMot2ModuleClockHz(
        const HostMot2Inventory &inventory,
        const HostMot2Module &module) noexcept {
        return module.clockTag == 1
            ? inventory.idrom.clockLowHz
            : inventory.idrom.clockHighHz;
    }

    std::uint32_t hostMot2ModuleInstanceStride(
        const HostMot2Inventory &inventory,
        const HostMot2Module &module) noexcept {
        return (module.strides & 0xF0) == 0
            ? inventory.idrom.instanceStride0
            : inventory.idrom.instanceStride1;
    }

    std::uint32_t hostMot2ModuleRegisterStride(
        const HostMot2Inventory &inventory,
        const HostMot2Module &module) noexcept {
        return (module.strides & 0x0F) == 0
            ? inventory.idrom.registerStride0
            : inventory.idrom.registerStride1;
    }
}
