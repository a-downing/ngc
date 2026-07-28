#pragma once

#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace ngc::mesa {
    class HostMot2RegisterReader {
    public:
        virtual ~HostMot2RegisterReader() = default;

        [[nodiscard]] virtual std::expected<void, std::string> read(
            std::uint32_t address, std::span<std::byte> destination) = 0;
    };

    struct HostMot2Idrom {
        std::uint32_t type = 0;
        std::uint32_t moduleOffset = 0;
        std::uint32_t pinOffset = 0;
        std::string boardName;
        std::uint32_t fpgaSize = 0;
        std::uint32_t fpgaPins = 0;
        std::uint32_t ioPorts = 0;
        std::uint32_t ioWidth = 0;
        std::uint32_t portWidth = 0;
        std::uint32_t clockLowHz = 0;
        std::uint32_t clockHighHz = 0;
        std::uint32_t instanceStride0 = 0;
        std::uint32_t instanceStride1 = 0;
        std::uint32_t registerStride0 = 0;
        std::uint32_t registerStride1 = 0;
    };

    struct HostMot2Module {
        std::uint8_t tag = 0;
        std::uint8_t version = 0;
        std::uint8_t clockTag = 0;
        std::uint8_t instances = 0;
        std::uint16_t baseAddress = 0;
        std::uint8_t registers = 0;
        std::uint8_t strides = 0;
        std::uint32_t multipleRegisterBitmap = 0;
    };

    struct HostMot2Pin {
        std::uint8_t secondaryPin = 0;
        std::uint8_t secondaryTag = 0;
        std::uint8_t secondaryChannel = 0;
        std::uint8_t primaryTag = 0;

        [[nodiscard]] bool secondaryOutput() const noexcept;
        [[nodiscard]] std::uint8_t secondaryPinNumber() const noexcept;
        [[nodiscard]] bool globalSecondaryChannel() const noexcept;
        [[nodiscard]] std::uint8_t secondaryChannelNumber() const noexcept;
    };

    struct HostMot2Inventory {
        std::string configurationName;
        std::uint32_t idromAddress = 0;
        HostMot2Idrom idrom;
        std::vector<HostMot2Module> modules;
        std::vector<HostMot2Pin> pins;
    };

    [[nodiscard]] std::expected<HostMot2Inventory, std::string>
    discoverHostMot2(HostMot2RegisterReader &reader);

    [[nodiscard]] std::string_view hostMot2ModuleName(
        std::uint8_t tag) noexcept;
    [[nodiscard]] std::uint32_t hostMot2ModuleClockHz(
        const HostMot2Inventory &inventory,
        const HostMot2Module &module) noexcept;
    [[nodiscard]] std::uint32_t hostMot2ModuleInstanceStride(
        const HostMot2Inventory &inventory,
        const HostMot2Module &module) noexcept;
    [[nodiscard]] std::uint32_t hostMot2ModuleRegisterStride(
        const HostMot2Inventory &inventory,
        const HostMot2Module &module) noexcept;
}
