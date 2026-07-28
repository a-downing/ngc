#include <charconv>
#include <chrono>
#include <cstdint>
#include <format>
#include <iostream>
#include <print>
#include <stdexcept>
#include <string>
#include <string_view>

#include "mesa/HostMot2Discovery.h"
#include "mesa/Lbp16UdpTransport.h"
#include "mesa/SevenI96Capabilities.h"

namespace {
    struct Options {
        std::string address;
        std::chrono::milliseconds timeout{250};
        bool validateSevenI96 = false;
    };

    std::uint32_t parseUnsigned(
        const std::string_view value,
        const std::string_view description) {
        std::uint32_t result = 0;
        const auto parsed = std::from_chars(
            value.data(), value.data() + value.size(), result);
        if (parsed.ec != std::errc{}
            || parsed.ptr != value.data() + value.size()) {
            throw std::runtime_error(std::format(
                "{} must be an unsigned integer", description));
        }

        return result;
    }

    Options parseOptions(const int argc, char **argv) {
        Options result;
        for (auto index = 1; index < argc; ++index) {
            const auto option = std::string_view(argv[index]);
            const auto value = [&]() -> std::string_view {
                if (++index == argc) {
                    throw std::runtime_error(std::format(
                        "{} requires a value", option));
                }

                return argv[index];
            };

            if (option == "--address") {
                result.address = value();
            } else if (option == "--validate-7i96") {
                result.validateSevenI96 = true;
            } else if (option == "--timeout-ms") {
                const auto milliseconds =
                    parseUnsigned(value(), "--timeout-ms");
                if (milliseconds == 0) {
                    throw std::runtime_error(
                        "--timeout-ms must be positive");
                }
                result.timeout = std::chrono::milliseconds(milliseconds);
            } else {
                throw std::runtime_error(std::format(
                    "unknown option '{}'", option));
            }
        }
        if (result.address.empty()) {
            throw std::runtime_error("--address is required");
        }

        return result;
    }

    void printInventory(const ngc::mesa::HostMot2Inventory &inventory) {
        const auto &idrom = inventory.idrom;
        std::println("Configuration: {}", inventory.configurationName);
        std::println("Board: {}", idrom.boardName);
        std::println("IDROM: type={} address=0x{:04X}",
                     idrom.type, inventory.idromAddress);
        std::println("FPGA: {} kgates, {} pins",
                     idrom.fpgaSize, idrom.fpgaPins);
        std::println("I/O: {} ports, {} pins, {} pins per port",
                     idrom.ioPorts, idrom.ioWidth, idrom.portWidth);
        std::println("Clocks: low={} Hz high={} Hz",
                     idrom.clockLowHz, idrom.clockHighHz);
        std::println("Strides: instance={}/{} register={}/{}",
                     idrom.instanceStride0, idrom.instanceStride1,
                     idrom.registerStride0, idrom.registerStride1);

        std::println("\nModules ({}):", inventory.modules.size());
        for (const auto &module : inventory.modules) {
            std::println(
                "  {:<20} tag=0x{:02X} version={} instances={} "
                "registers={} base=0x{:04X} clock={} Hz "
                "instance_stride={} register_stride={} multiple=0x{:08X}",
                ngc::mesa::hostMot2ModuleName(module.tag),
                module.tag, module.version, module.instances,
                module.registers, module.baseAddress,
                ngc::mesa::hostMot2ModuleClockHz(inventory, module),
                ngc::mesa::hostMot2ModuleInstanceStride(inventory, module),
                ngc::mesa::hostMot2ModuleRegisterStride(inventory, module),
                module.multipleRegisterBitmap);
        }

        std::println("\nPins ({}):", inventory.pins.size());
        for (std::size_t index = 0;
             index < inventory.pins.size(); ++index) {
            const auto &pin = inventory.pins[index];
            if (pin.secondaryTag == 0) {
                std::println(
                    "  {:3} primary={}",
                    index,
                    ngc::mesa::hostMot2ModuleName(pin.primaryTag));
                continue;
            }
            std::println(
                "  {:3} primary={} secondary={} channel={}{} pin={} {}",
                index,
                ngc::mesa::hostMot2ModuleName(pin.primaryTag),
                ngc::mesa::hostMot2ModuleName(pin.secondaryTag),
                pin.secondaryChannelNumber(),
                pin.globalSecondaryChannel() ? " (global)" : "",
                pin.secondaryPinNumber(),
                pin.secondaryOutput() ? "output" : "input");
        }
    }
}

int main(const int argc, char **argv) {
    try {
        const auto options = parseOptions(argc, argv);
        const auto transport = ngc::mesa::Lbp16UdpTransport::open({
            .address = options.address,
            .timeout = options.timeout,
        });
        if (!transport) {
            throw std::runtime_error(transport.error());
        }
        const auto inventory =
            ngc::mesa::discoverHostMot2(**transport);
        if (!inventory) {
            throw std::runtime_error(inventory.error());
        }

        printInventory(*inventory);
        if (options.validateSevenI96) {
            const auto capabilities =
                ngc::mesa::validateSevenI96Capabilities(*inventory);
            if (!capabilities) {
                throw std::runtime_error(capabilities.error());
            }
            std::println(
                "\nValidated 7I96 capabilities: {} StepGen channels, "
                "{} isolated inputs, {} isolated outputs, "
                "{} encoder channel, watchdog and DPLL",
                capabilities->stepGenerators.size(),
                capabilities->isolatedInputPins.size(),
                capabilities->isolatedOutputPins.size(),
                capabilities->encoder.descriptor.instances);
        }
    } catch (const std::exception &error) {
        std::cerr << "Mesa discovery failed: "
                  << error.what() << '\n';

        return 1;
    }

    return 0;
}
