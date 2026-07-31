#include "mesa/MesaBackendConfiguration.h"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <string_view>
#include <utility>

#include <toml++/toml.hpp>

#include "config/TomlConfiguration.h"
#include "machine/MotionBackend.h"
#include "mesa/SevenI96Capabilities.h"

namespace ngc::mesa {
    namespace {
        std::expected<std::uint32_t, std::string> nanoseconds(
            const toml::table &table,
            const std::string_view name,
            const std::filesystem::path &path,
            const bool requirePositive) {
            const auto seconds = requirePositive
                ? toml_configuration::positiveNumber(
                    table, name, path)
                : toml_configuration::number(
                    table, name, path);
            if (!seconds) {
                return std::unexpected(seconds.error());
            }
            const auto scaled = *seconds * 1'000'000'000.0;
            if (scaled < 0.0
                || scaled > std::numeric_limits<std::uint32_t>::max()) {
                return std::unexpected(toml_configuration::error(
                    path, name,
                    "is outside the supported nanosecond range",
                    table.get(name)));
            }
            const auto rounded =
                static_cast<std::uint32_t>(std::llround(scaled));
            if (requirePositive && rounded == 0) {
                return std::unexpected(toml_configuration::error(
                    path, name,
                    "rounds to zero nanoseconds",
                    table.get(name)));
            }

            return rounded;
        }

        std::expected<std::int32_t, std::string> signedNanoseconds(
            const toml::table &table,
            const std::string_view name,
            const std::filesystem::path &path) {
            const auto seconds = toml_configuration::number(
                table, name, path);
            if (!seconds) {
                return std::unexpected(seconds.error());
            }
            const auto scaled = *seconds * 1'000'000'000.0;
            if (scaled < std::numeric_limits<std::int32_t>::min()
                || scaled > std::numeric_limits<std::int32_t>::max()) {
                return std::unexpected(toml_configuration::error(
                    path, name,
                    "is outside the supported nanosecond range",
                    table.get(name)));
            }

            return static_cast<std::int32_t>(std::llround(scaled));
        }

        const toml::table *requiredTable(
            const toml::table &parent,
            const std::string_view name,
            const std::filesystem::path &path,
            std::string &error) {
            const auto *node = parent.get(name);
            const auto *table = node != nullptr
                ? node->as_table() : nullptr;
            if (table == nullptr) {
                error = toml_configuration::error(
                    path, name, "must be a table", node);
            }

            return table;
        }
    }

    std::expected<MesaBackendConfiguration, std::string>
    loadMesaBackendConfiguration(
        const std::filesystem::path &path) {
        const auto document =
            toml_configuration::loadDocument(path);
        if (!document) {
            return std::unexpected(document.error());
        }

        return loadMesaBackendConfiguration(
            document->table, path);
    }

    std::expected<MesaBackendConfiguration, std::string>
    loadMesaBackendConfiguration(
        const toml::table &document,
        const std::filesystem::path &path) {
        try {
            const auto *motion = document["motion"].as_table();
            if (motion == nullptr) {
                return std::unexpected(toml_configuration::error(
                    path, "motion", "missing table"));
            }

            std::string tableError;
            const auto *units = requiredTable(
                *motion, "units", path, tableError);
            if (units == nullptr) {
                return std::unexpected(tableError);
            }
            const auto *stepTiming = requiredTable(
                *motion, "step_timing", path, tableError);
            if (stepTiming == nullptr) {
                return std::unexpected(tableError);
            }
            const auto *dpll = requiredTable(
                *motion, "dpll", path, tableError);
            if (dpll == nullptr) {
                return std::unexpected(tableError);
            }
            const auto *fieldInputs = requiredTable(
                *motion, "field_inputs", path, tableError);
            if (fieldInputs == nullptr) {
                return std::unexpected(tableError);
            }
            if (fieldInputs->empty()) {
                return std::unexpected(
                    toml_configuration::error(
                        path, "motion.field_inputs",
                        "must not be empty",
                        motion->get("field_inputs")));
            }
            const auto *stepGeneratorsNode =
                motion->get("stepgens");
            const auto *stepGenerators =
                stepGeneratorsNode != nullptr
                ? stepGeneratorsNode->as_array() : nullptr;
            if (stepGenerators == nullptr
                || stepGenerators->empty()) {
                return std::unexpected(toml_configuration::error(
                    path, "motion.stepgens",
                    "must be a non-empty array of tables",
                    stepGeneratorsNode));
            }

            const auto driver = toml_configuration::requiredString(
                *motion, "driver", path);
            const auto address = toml_configuration::requiredString(
                *motion, "address", path);
            const auto expectedBoard =
                toml_configuration::requiredString(
                    *motion, "expected_board", path);
            const auto ioProgram = toml_configuration::requiredString(
                *motion, "io_program", path);
            const auto watchdog = nanoseconds(
                *motion, "watchdog_timeout", path, true);
            const auto linear = toml_configuration::requiredString(
                *units, "linear", path);
            const auto angular = toml_configuration::requiredString(
                *units, "angular", path);
            const auto stepLength = nanoseconds(
                *stepTiming, "step_length", path, true);
            const auto stepSpace = nanoseconds(
                *stepTiming, "step_space", path, true);
            const auto directionSetup = nanoseconds(
                *stepTiming, "direction_setup", path, true);
            const auto directionHold = nanoseconds(
                *stepTiming, "direction_hold", path, true);
            const auto dpllEnabled =
                toml_configuration::requiredBool(
                    *dpll, "enabled", path);
            const auto dpllTimer = toml_configuration::integer(
                *dpll, "stepgen_timer", path);
            const auto dpllOffset = signedNanoseconds(
                *dpll, "stepgen_sample_offset", path);
            const auto dpllMaximumPhase = nanoseconds(
                *dpll, "maximum_phase_error", path, true);
            const auto dpllConvergence =
                toml_configuration::integer(
                    *dpll, "convergence_cycles", path);
            const std::array required{
                driver.has_value(), address.has_value(),
                expectedBoard.has_value(), ioProgram.has_value(),
                watchdog.has_value(), linear.has_value(),
                angular.has_value(),
                stepLength.has_value(), stepSpace.has_value(),
                directionSetup.has_value(), directionHold.has_value(),
                dpllEnabled.has_value(), dpllTimer.has_value(),
                dpllOffset.has_value(), dpllMaximumPhase.has_value(),
                dpllConvergence.has_value(),
            };
            if (!std::ranges::all_of(
                    required, [](const bool value) {
                        return value;
                    })) {
                const std::array errors{
                    driver.error_or({}), address.error_or({}),
                    expectedBoard.error_or({}), ioProgram.error_or({}),
                    watchdog.error_or({}), linear.error_or({}),
                    angular.error_or({}),
                    stepLength.error_or({}), stepSpace.error_or({}),
                    directionSetup.error_or({}), directionHold.error_or({}),
                    dpllEnabled.error_or({}), dpllTimer.error_or({}),
                    dpllOffset.error_or({}), dpllMaximumPhase.error_or({}),
                    dpllConvergence.error_or({}),
                };
                const auto found = std::ranges::find_if(
                    errors, [](const std::string &value) {
                        return !value.empty();
                    });

                return std::unexpected(*found);
            }
            if (*driver != "mesa_hostmot2") {
                return std::unexpected(toml_configuration::error(
                    path, "motion.driver",
                    "must be mesa_hostmot2", motion->get("driver")));
            }
            if (*expectedBoard != "7i96") {
                return std::unexpected(toml_configuration::error(
                    path, "motion.expected_board",
                    "must be 7i96", motion->get("expected_board")));
            }
            if (*linear != "mm" && *linear != "inch") {
                return std::unexpected(toml_configuration::error(
                    path, "motion.units.linear",
                    "must be mm or inch", units->get("linear")));
            }
            if (*angular != "degree") {
                return std::unexpected(toml_configuration::error(
                    path, "motion.units.angular",
                    "must be degree", units->get("angular")));
            }
            if (*dpllTimer < 1 || *dpllTimer > 4
                || *dpllOffset >= 0
                || *dpllConvergence <= 0
                || *dpllConvergence
                    > std::numeric_limits<std::uint32_t>::max()) {
                return std::unexpected(toml_configuration::error(
                    path, "motion.dpll",
                    "contains an invalid timer, offset, or convergence count",
                    motion->get("dpll")));
            }

            auto safetyConfiguration =
                std::optional<MesaSafetyConfiguration>{};
            if (const auto *safetyNode = motion->get("safety");
                safetyNode != nullptr) {
                const auto *safety = safetyNode->as_table();
                if (safety == nullptr) {
                    return std::unexpected(
                        toml_configuration::error(
                            path, "motion.safety",
                            "must be a table", safetyNode));
                }
                const auto enableInput =
                    toml_configuration::requiredString(
                        *safety, "enable_input", path);
                const auto active =
                    toml_configuration::requiredString(
                        *safety, "active", path);
                if (!enableInput) {
                    return std::unexpected(enableInput.error());
                }
                if (!active) {
                    return std::unexpected(active.error());
                }
                if (*active != "high" && *active != "low") {
                    return std::unexpected(
                        toml_configuration::error(
                            path, "motion.safety.active",
                            "must be high or low",
                            safety->get("active")));
                }
                safetyConfiguration = {
                    .enableInput = *enableInput,
                    .polarity = *active == "high"
                        ? MesaSafetyPolarity::ActiveHigh
                        : MesaSafetyPolarity::ActiveLow,
                };
            }

            MesaBackendConfiguration result{
                .address = *address,
                .expectedBoard = *expectedBoard,
                .ioProgram = *ioProgram,
                .linearUnit = *linear == "mm"
                    ? MesaLinearUnit::Millimeter
                    : MesaLinearUnit::Inch,
                .stepTiming = {
                    .stepLengthNanoseconds = *stepLength,
                    .stepSpaceNanoseconds = *stepSpace,
                    .directionSetupNanoseconds = *directionSetup,
                    .directionHoldNanoseconds = *directionHold,
                },
                .watchdogTimeoutNanoseconds = *watchdog,
                .dpll = {
                    .enabled = *dpllEnabled,
                    .stepGeneratorTimer =
                        static_cast<std::uint8_t>(*dpllTimer),
                    .stepGeneratorSampleOffsetNanoseconds =
                        *dpllOffset,
                    .maximumPhaseErrorNanoseconds =
                        *dpllMaximumPhase,
                    .convergenceCycles =
                        static_cast<std::uint32_t>(
                            *dpllConvergence),
                },
                .safety = std::move(safetyConfiguration),
                .fieldInputs = {},
                .stepGenerators = {},
            };
            std::array<bool, SEVEN_I96_ISOLATED_INPUT_COUNT>
                selectedFieldInputs{};
            for (const auto &[key, node] : *fieldInputs) {
                const auto index = node.value<std::int64_t>();
                const auto name = std::string(key.str());
                const auto field = std::format(
                    "motion.field_inputs.{}", name);
                if (!index.has_value()
                    || *index < 0
                    || *index >= static_cast<std::int64_t>(
                        SEVEN_I96_ISOLATED_INPUT_COUNT)) {
                    return std::unexpected(
                        toml_configuration::error(
                            path, field,
                            "must be an available 7I96 input index",
                            &node));
                }
                if (selectedFieldInputs[*index]) {
                    return std::unexpected(
                        toml_configuration::error(
                            path, field,
                            "duplicates another physical input index",
                            &node));
                }
                selectedFieldInputs[*index] = true;
                result.fieldInputs.push_back({
                    .name = name,
                    .index =
                        static_cast<std::uint16_t>(*index),
                });
            }
            std::array<bool, MAX_JOINTS> selectedJoints{};
            std::array<bool, SEVEN_I96_STEP_GENERATOR_COUNT>
                selectedChannels{};
            for (const auto &entry : *stepGenerators) {
                const auto *table = entry.as_table();
                if (table == nullptr) {
                    return std::unexpected(
                        toml_configuration::error(
                            path, "motion.stepgens",
                            "entries must be tables", &entry));
                }
                const auto joint = toml_configuration::integer(
                    *table, "joint", path);
                const auto channel = toml_configuration::integer(
                    *table, "channel", path);
                const auto scale =
                    toml_configuration::positiveNumber(
                        *table, "steps_per_unit", path);
                const auto invert =
                    toml_configuration::requiredBool(
                        *table, "invert_direction", path);
                const auto gain =
                    toml_configuration::positiveNumber(
                        *table, "position_gain", path);
                const auto correction =
                    toml_configuration::positiveNumber(
                        *table, "maximum_correction_velocity", path);
                const auto maximumError =
                    toml_configuration::positiveNumber(
                        *table, "maximum_generated_step_error", path);
                if (!joint || !channel || !scale || !invert
                    || !gain || !correction || !maximumError) {
                    const std::array errors{
                        joint.error_or({}), channel.error_or({}),
                        scale.error_or({}), invert.error_or({}),
                        gain.error_or({}), correction.error_or({}),
                        maximumError.error_or({}),
                    };
                    const auto found = std::ranges::find_if(
                        errors, [](const std::string &value) {
                            return !value.empty();
                        });

                    return std::unexpected(*found);
                }
                if (*joint < 0
                    || *joint >= static_cast<std::int64_t>(MAX_JOINTS)
                    || *channel < 0
                    || *channel
                        >= static_cast<std::int64_t>(
                            SEVEN_I96_STEP_GENERATOR_COUNT)) {
                    return std::unexpected(
                        toml_configuration::error(
                            path, "motion.stepgens",
                            "joint or channel is outside its supported range",
                            &entry));
                }
                if (selectedJoints[*joint]
                    || selectedChannels[*channel]) {
                    return std::unexpected(
                        toml_configuration::error(
                            path, "motion.stepgens",
                            "contains a duplicate joint or channel",
                            &entry));
                }
                selectedJoints[*joint] = true;
                selectedChannels[*channel] = true;
                result.stepGenerators.push_back({
                    .joint = static_cast<std::uint8_t>(*joint),
                    .channel = static_cast<std::uint8_t>(*channel),
                    .stepsPerUnit = *scale,
                    .invertDirection = *invert,
                    .positionGainPerSecond = *gain,
                    .maximumCorrectionVelocity = *correction,
                    .maximumGeneratedStepError = *maximumError,
                });
            }

            return result;
        } catch (const toml::parse_error &error) {
            return std::unexpected(std::format(
                "{}: {}", path.string(), error.description()));
        }
    }
}
