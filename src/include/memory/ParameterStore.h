#pragma once

#include <expected>
#include <filesystem>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "machine/Machine.h"
#include "memory/Memory.h"

namespace ngc {
    struct ParameterStorePaths {
        std::filesystem::path real = "real_parameters.var";
        std::filesystem::path simulation = "simulation_parameters.var";
    };

    std::expected<std::vector<Memory::PersistentParameter>, std::string>
    parsePersistentParameters(std::string_view text, Machine::Unit unit, const Memory &memory);

    std::string serializePersistentParameters(
        Machine::Unit unit, std::span<const Memory::PersistentParameter> parameters);

    std::expected<void, std::string>
    loadPersistentParameters(const std::filesystem::path &path, Machine::Unit unit, Memory &memory);

    std::expected<void, std::string>
    savePersistentParameters(const std::filesystem::path &path, Machine::Unit unit, const Memory &memory);
}
