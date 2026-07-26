#include "platform/HidReportDescriptor.h"

#include <algorithm>
#include <array>
#include <limits>
#include <vector>

namespace ngc::pendant::hid_detail {
    namespace {
        struct GlobalState {
            std::uint64_t reportSize = 0;
            std::uint64_t reportCount = 0;
            std::uint8_t reportId = 0;
        };

        std::uint64_t unsignedValue(const std::span<const std::uint8_t> bytes) {
            std::uint64_t result = 0;
            for (std::size_t index = 0; index < bytes.size(); ++index) {
                result |= static_cast<std::uint64_t>(bytes[index]) << (index * 8);
            }
            return result;
        }

        std::expected<void, std::string>
        addReportBits(std::array<std::uint64_t, 256> &bits, const GlobalState &state) {
            if (state.reportSize != 0
                && state.reportCount > std::numeric_limits<std::uint64_t>::max() / state.reportSize) {
                return std::unexpected("HID report size overflows");
            }
            const auto added = state.reportSize * state.reportCount;
            auto &total = bits[state.reportId];
            if (added > std::numeric_limits<std::uint64_t>::max() - total) {
                return std::unexpected("HID report length overflows");
            }
            total += added;
            return {};
        }

        std::expected<std::size_t, std::string>
        byteLength(const std::array<std::uint64_t, 256> &bits, const bool output) {
            std::uint64_t maximum = 0;
            for (std::size_t reportId = 0; reportId < bits.size(); ++reportId) {
                if (bits[reportId] == 0) {
                    continue;
                }
                auto length = bits[reportId] / 8 + (bits[reportId] % 8 != 0 ? 1 : 0);
                if (output || reportId != 0) {
                    ++length;
                }
                maximum = std::max(maximum, length);
            }
            if (maximum > std::numeric_limits<std::size_t>::max()) {
                return std::unexpected("HID report length is not representable");
            }
            return static_cast<std::size_t>(maximum);
        }
    }

    std::expected<HidApiReportLengths, std::string>
    hidApiReportLengths(const std::span<const std::uint8_t> descriptor) {
        std::array<std::uint64_t, 256> inputBits{};
        std::array<std::uint64_t, 256> outputBits{};
        GlobalState global;
        std::vector<GlobalState> globalStack;

        std::size_t offset = 0;
        while (offset < descriptor.size()) {
            const auto prefix = descriptor[offset++];
            if (prefix == 0xfe) {
                if (descriptor.size() - offset < 2) {
                    return std::unexpected("truncated long HID descriptor item");
                }
                const auto length = descriptor[offset++];
                ++offset;
                if (descriptor.size() - offset < length) {
                    return std::unexpected("truncated long HID descriptor payload");
                }
                offset += length;
                continue;
            }

            const auto encodedSize = prefix & 0x03;
            const std::size_t size = encodedSize == 3 ? 4 : encodedSize;
            if (descriptor.size() - offset < size) {
                return std::unexpected("truncated HID descriptor item");
            }
            const auto data = descriptor.subspan(offset, size);
            offset += size;

            const auto type = (prefix >> 2) & 0x03;
            const auto tag = (prefix >> 4) & 0x0f;
            if (type == 0) {
                if (tag == 8) {
                    auto added = addReportBits(inputBits, global);
                    if (!added) {
                        return std::unexpected(std::move(added.error()));
                    }
                } else if (tag == 9) {
                    auto added = addReportBits(outputBits, global);
                    if (!added) {
                        return std::unexpected(std::move(added.error()));
                    }
                }
                continue;
            }
            if (type != 1) {
                continue;
            }

            const auto value = unsignedValue(data);
            if (tag == 7) {
                global.reportSize = value;
            } else if (tag == 8) {
                if (value == 0 || value > 255) {
                    return std::unexpected("HID report ID is outside the valid range");
                }
                global.reportId = static_cast<std::uint8_t>(value);
            } else if (tag == 9) {
                global.reportCount = value;
            } else if (tag == 10) {
                globalStack.push_back(global);
            } else if (tag == 11) {
                if (globalStack.empty()) {
                    return std::unexpected("HID descriptor pops an empty global stack");
                }
                global = globalStack.back();
                globalStack.pop_back();
            }
        }

        if (!globalStack.empty()) {
            return std::unexpected("HID descriptor leaves global state pushed");
        }
        auto input = byteLength(inputBits, false);
        if (!input) {
            return std::unexpected(std::move(input.error()));
        }
        auto output = byteLength(outputBits, true);
        if (!output) {
            return std::unexpected(std::move(output.error()));
        }
        if (*input == 0 || *output == 0) {
            return std::unexpected("HID descriptor does not define input and output reports");
        }
        return HidApiReportLengths { *input, *output };
    }
}
