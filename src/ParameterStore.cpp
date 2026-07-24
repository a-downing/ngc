#include "memory/ParameterStore.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <format>
#include <limits>
#include <set>
#include <stdexcept>

#include "utils.h"

namespace ngc {
    namespace {
        std::string_view trim(std::string_view text) {
            constexpr std::string_view whitespace = " \t\r";
            const auto first = text.find_first_not_of(whitespace);
            if (first == std::string_view::npos) {
                return {};
            }
            const auto last = text.find_last_not_of(whitespace);

            return text.substr(first, last - first + 1);
        }

        std::vector<std::string_view> tokens(const std::string_view line) {
            std::vector<std::string_view> result;
            auto remaining = line;
            while (!remaining.empty()) {
                const auto first = remaining.find_first_not_of(" \t\r");
                if (first == std::string_view::npos) {
                    break;
                }
                remaining.remove_prefix(first);
                const auto end = remaining.find_first_of(" \t\r");
                result.push_back(remaining.substr(0, end));
                if (end == std::string_view::npos) {
                    break;
                }
                remaining.remove_prefix(end);
            }

            return result;
        }

        std::string_view unitName(const Machine::Unit unit) {
            return unit == Machine::Unit::Inch ? "inch" : "mm";
        }

        template<typename Value>
        std::expected<Value, std::string> parseNumber(
            const std::string_view text, const std::size_t line, const std::string_view description) {
            Value result{};
            const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), result);
            if (error != std::errc{} || end != text.data() + text.size()) {
                return std::unexpected(std::format(
                    "parameter file line {} has an invalid {} '{}'", line, description, text));
            }

            return result;
        }

        std::string formatDouble(const double value) {
            std::array<char, 64> buffer{};
            const auto [end, error] = std::to_chars(
                buffer.data(), buffer.data() + buffer.size(), value,
                std::chars_format::general, std::numeric_limits<double>::max_digits10);
            if (error != std::errc{}) {
                throw std::runtime_error("failed to serialize a persistent parameter");
            }

            return std::string(buffer.data(), end);
        }
    }

    std::expected<std::vector<Memory::PersistentParameter>, std::string>
    parsePersistentParameters(const std::string_view text, const Machine::Unit unit, const Memory &memory) {
        enum class HeaderState { Signature, Unit, Data };
        HeaderState state = HeaderState::Signature;
        std::vector<Memory::PersistentParameter> result;
        std::set<std::uint32_t> addresses;
        std::size_t lineNumber = 0;
        std::size_t start = 0;

        while (start <= text.size()) {
            ++lineNumber;
            auto end = text.find('\n', start);
            if (end == std::string_view::npos) {
                end = text.size();
            }
            auto line = text.substr(start, end - start);
            if (const auto comment = line.find('#'); comment != std::string_view::npos) {
                line = line.substr(0, comment);
            }
            line = trim(line);
            if (!line.empty()) {
                const auto fields = tokens(line);
                if (state == HeaderState::Signature) {
                    if (fields.size() != 2 || fields[0] != "NGC_PARAMETERS" || fields[1] != "1") {
                        return std::unexpected(std::format(
                            "parameter file line {} must be 'NGC_PARAMETERS 1'", lineNumber));
                    }
                    state = HeaderState::Unit;
                } else if (state == HeaderState::Unit) {
                    if (fields.size() != 2 || fields[0] != "unit") {
                        return std::unexpected(std::format(
                            "parameter file line {} must declare the machine unit", lineNumber));
                    }
                    if (fields[1] != unitName(unit)) {
                        return std::unexpected(std::format(
                            "parameter file unit '{}' does not match configured unit '{}'",
                            fields[1], unitName(unit)));
                    }
                    state = HeaderState::Data;
                } else {
                    if (fields.size() != 2) {
                        return std::unexpected(std::format(
                            "parameter file line {} must contain an address and value", lineNumber));
                    }
                    auto address = parseNumber<std::uint32_t>(fields[0], lineNumber, "address");
                    auto value = parseNumber<double>(fields[1], lineNumber, "value");
                    if (!address) {
                        return std::unexpected(address.error());
                    }
                    if (!value) {
                        return std::unexpected(value.error());
                    }
                    if (!std::isfinite(*value)) {
                        return std::unexpected(std::format(
                            "parameter file line {} has a non-finite value", lineNumber));
                    }
                    if (!memory.isPersistentParameter(*address)) {
                        return std::unexpected(std::format(
                            "parameter file line {} references ineligible or unknown address {}",
                            lineNumber, *address));
                    }
                    if (*address == memory.deref(Var::COORDSYS)
                        && (*value != std::trunc(*value) || *value < 1.0 || *value > 9.0)) {
                        return std::unexpected(std::format(
                            "parameter file line {} has invalid active coordinate system {}",
                            lineNumber, *value));
                    }
                    if (!addresses.insert(*address).second) {
                        return std::unexpected(std::format(
                            "parameter file line {} duplicates address {}", lineNumber, *address));
                    }
                    result.push_back({*address, *value});
                }
            }
            if (end == text.size()) {
                break;
            }
            start = end + 1;
        }

        if (state != HeaderState::Data) {
            return std::unexpected("parameter file is missing its signature or unit declaration");
        }
        std::ranges::sort(result, {}, &Memory::PersistentParameter::address);

        return result;
    }

    std::string serializePersistentParameters(
        const Machine::Unit unit, const std::span<const Memory::PersistentParameter> parameters) {
        auto sorted = std::vector(parameters.begin(), parameters.end());
        std::ranges::sort(sorted, {}, &Memory::PersistentParameter::address);
        std::string result = std::format("NGC_PARAMETERS 1\nunit {}\n\n", unitName(unit));
        for (const auto &[address, value] : sorted) {
            result += std::format("{} {}\n", address, formatDouble(value));
        }

        return result;
    }

    std::expected<void, std::string>
    loadPersistentParameters(const std::filesystem::path &path, const Machine::Unit unit, Memory &memory) {
        const auto file = readFile(path);
        if (!file) {
            return std::unexpected(file.error().what());
        }
        auto parameters = parsePersistentParameters(*file, unit, memory);
        if (!parameters) {
            return std::unexpected(std::format("{}: {}", path.string(), parameters.error()));
        }
        if (!memory.applyPersistentParameters(*parameters)) {
            return std::unexpected(std::format(
                "{}: validated persistent parameters could not be applied", path.string()));
        }

        return {};
    }

    std::expected<void, std::string>
    savePersistentParameters(const std::filesystem::path &path, const Machine::Unit unit, const Memory &memory) {
        const auto parameters = memory.persistentParameters();
        for (const auto &[address, value] : parameters) {
            if (!std::isfinite(value)) {
                return std::unexpected(std::format(
                    "cannot save non-finite persistent parameter {}", address));
            }
            if (address == memory.deref(Var::COORDSYS)
                && (value != std::trunc(value) || value < 1.0 || value > 9.0)) {
                return std::unexpected(std::format(
                    "cannot save invalid active coordinate system {}", value));
            }
        }

        const auto text = serializePersistentParameters(unit, parameters);
        const auto saved = writeFile(path, text);
        if (!saved) {
            return std::unexpected(saved.error().what());
        }

        return {};
    }
}
