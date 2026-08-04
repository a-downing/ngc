#include <array>
#include <chrono>
#include <expected>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <string_view>

#include "machine/MachineConfiguration.h"

namespace {
    void require(const bool condition, const std::string_view message) {
        if (!condition) {
            throw std::runtime_error(std::string(message));
        }
    }

    std::string readFile(const std::filesystem::path &path) {
        std::ifstream file(path);
        require(static_cast<bool>(file),
                std::format("failed to open {}", path.string()));

        return {
            std::istreambuf_iterator<char>(file),
            std::istreambuf_iterator<char>(),
        };
    }

    void replaceFirst(std::string &source, const std::string_view from,
                      const std::string_view to) {
        const auto position = source.find(from);
        require(position != std::string::npos,
                "machine configuration mutation anchor was not found");
        source.replace(position, from.size(), to);
    }

    std::expected<ngc::MachineConfiguration, std::string> loadSource(
        const std::string_view source) {
        const auto stamp = std::chrono::steady_clock::now()
            .time_since_epoch().count();
        const auto path = std::filesystem::temp_directory_path()
            / std::format("ngc-machine-configuration-test-{}.toml", stamp);
        {
            std::ofstream file(path, std::ios::binary | std::ios::trunc);
            require(static_cast<bool>(file),
                    "failed to create temporary machine configuration");
            file.write(source.data(), static_cast<std::streamsize>(source.size()));
            require(static_cast<bool>(file),
                    "failed to write temporary machine configuration");
        }

        auto result = ngc::loadMachineConfiguration(path);
        std::filesystem::remove(path);

        return result;
    }

    void requireRejected(const std::string_view source,
                         const std::string_view expected) {
        const auto configuration = loadSource(source);
        require(!configuration,
                "unsupported homing configuration should be rejected");
        require(configuration.error().find(expected) != std::string::npos,
                std::format(
                    "unsupported homing configuration should identify '{}': {}",
                    expected, configuration.error()));
    }

    void testRepositoryConfigurationLoads() {
        const auto path = std::filesystem::path {NGC_SOURCE_DIR} / "machine.toml";
        const auto configuration = ngc::loadMachineConfiguration(path);
        require(configuration.has_value(),
                configuration ? "" : configuration.error());
    }

    void testJointCoordinateRangeAppliesAndOrdersScale() {
        auto joint = ngc::JointConfiguration{};
        joint.minimum = -2.0;
        joint.maximum = 5.0;

        joint.coordinateScale = 0.5;
        auto range = ngc::jointCoordinateRange(joint);
        require(range.minimum == -1.0 && range.maximum == 2.5,
                "positive joint scale was not applied to its coordinate range");

        joint.coordinateScale = -2.0;
        range = ngc::jointCoordinateRange(joint);
        require(range.minimum == -10.0 && range.maximum == 4.0,
                "negative joint scale did not reorder its coordinate range");
    }

    void testUnsupportedHomingModesAreRejected() {
        const auto path = std::filesystem::path {NGC_SOURCE_DIR} / "machine.toml";
        const auto source = readFile(path);

        auto mutation = source;
        replaceFirst(mutation, "final_velocity = 0.0\n",
                     "final_velocity = 0.0\nuse_index = true\n");
        requireRejected(mutation, "joints.homing.use_index");

        mutation = source;
        replaceFirst(mutation,
                     "name = \"z\"\nsequence = 0\njoints = [3]\n",
                     "name = \"z\"\nsequence = 0\njoints = [3]\n"
                     "start_together = true\n");
        requireRejected(mutation,
                        "must not be specified for a single-joint group");

        constexpr std::array options {
            "start_together",
            "stop_each_joint_on_trigger",
            "final_move_together",
        };
        for (const auto *option : options) {
            mutation = source;
            replaceFirst(mutation, std::format("{} = true", option),
                         std::format("{} = false", option));
            requireRejected(mutation, option);
        }

        mutation = source;
        replaceFirst(mutation, "start_together = true\n", "");
        requireRejected(mutation, "start_together");
    }
}

int main() {
    try {
        testRepositoryConfigurationLoads();
        testJointCoordinateRangeAppliesAndOrdersScale();
        testUnsupportedHomingModesAreRejected();
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';

        return 1;
    }

    return 0;
}
