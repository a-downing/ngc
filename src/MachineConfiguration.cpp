#include "machine/MachineConfiguration.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <format>
#include <limits>
#include <string_view>
#include <unordered_set>

#include <toml++/toml.hpp>

namespace ngc {
    namespace {
        std::string configurationError(const std::filesystem::path &path, const std::string_view field,
                                       const std::string_view message, const toml::node *node = nullptr) {
            if(node) {
                const auto source = node->source();
                return std::format("{}:{}:{}: {}: {}", path.string(), source.begin.line,
                                   source.begin.column, field, message);
            }
            return std::format("{}: {}: {}", path.string(), field, message);
        }

        std::expected<double, std::string> number(const toml::table &table, const std::string_view name,
                                                  const std::filesystem::path &path) {
            const auto *node = table.get(name);
            const auto value = node ? node->value<double>() : std::optional<double>{};
            if(!value || !std::isfinite(*value))
                return std::unexpected(configurationError(path, name, "must be a finite number", node));
            return *value;
        }

        std::expected<double, std::string> positiveNumber(
            const toml::table &table, const std::string_view name, const std::filesystem::path &path) {
            auto value = number(table, name, path);
            if(!value) return value;
            if(*value <= 0.0)
                return std::unexpected(configurationError(path, name, "must be greater than zero",
                                                          table.get(name)));
            return value;
        }

        std::expected<std::string, std::string> requiredString(
            const toml::table &table, const std::string_view name, const std::filesystem::path &path) {
            const auto *node = table.get(name);
            const auto value = node ? node->value<std::string>() : std::optional<std::string>{};
            if(!value || value->empty())
                return std::unexpected(configurationError(path, name, "must be a non-empty string", node));
            return *value;
        }

        std::expected<bool, std::string> requiredBool(
            const toml::table &table, const std::string_view name, const std::filesystem::path &path) {
            const auto *node = table.get(name);
            const auto value = node ? node->value<bool>() : std::optional<bool>{};
            if(!value) return std::unexpected(configurationError(path, name, "must be a boolean", node));
            return *value;
        }

        std::expected<std::int64_t, std::string> integer(
            const toml::table &table, const std::string_view name, const std::filesystem::path &path) {
            const auto *node = table.get(name);
            const auto value = node ? node->value<std::int64_t>() : std::optional<std::int64_t>{};
            if(!value) return std::unexpected(configurationError(path, name, "must be an integer", node));
            return *value;
        }

        std::expected<Machine::Axis, std::string> parseAxis(
            const std::string_view value, const std::filesystem::path &path, const std::string_view field,
            const toml::node *node = nullptr) {
            if(value == "x") return Machine::Axis::X;
            if(value == "y") return Machine::Axis::Y;
            if(value == "z") return Machine::Axis::Z;
            if(value == "a") return Machine::Axis::A;
            if(value == "b") return Machine::Axis::B;
            if(value == "c") return Machine::Axis::C;
            return std::unexpected(configurationError(path, field, "must be one of x, y, z, a, b, or c", node));
        }

        std::string_view axisName(const Machine::Axis axis) {
            switch(axis) {
                case Machine::Axis::X: return "x";
                case Machine::Axis::Y: return "y";
                case Machine::Axis::Z: return "z";
                case Machine::Axis::A: return "a";
                case Machine::Axis::B: return "b";
                case Machine::Axis::C: return "c";
            }
            return "?";
        }

        std::expected<InputCondition, std::string> parseCondition(
            const std::string_view value, const std::filesystem::path &path, const std::string_view field,
            const toml::node *node = nullptr) {
            if(value == "active") return InputCondition::Active;
            if(value == "inactive") return InputCondition::Inactive;
            if(value == "rising") return InputCondition::RisingEdge;
            if(value == "falling") return InputCondition::FallingEdge;
            return std::unexpected(configurationError(
                path, field, "must be active, inactive, rising, or falling", node));
        }

        const DigitalInputConfiguration *findInput(
            const std::vector<DigitalInputConfiguration> &inputs, const std::string_view name) {
            const auto found = std::ranges::find(inputs, name, &DigitalInputConfiguration::name);
            return found == inputs.end() ? nullptr : &*found;
        }

        const JointConfiguration *findJoint(const std::vector<JointConfiguration> &joints, const JointId id) {
            const auto found = std::ranges::find(joints, id, &JointConfiguration::id);
            return found == joints.end() ? nullptr : &*found;
        }

        std::expected<std::vector<JointId>, std::string> parseJointIds(
            const toml::table &table, const std::string_view name, const std::filesystem::path &path,
            const std::string_view field) {
            const auto *node = table.get(name);
            const auto *array = node ? node->as_array() : nullptr;
            if(!array || array->empty())
                return std::unexpected(configurationError(path, field, "must be a non-empty integer array", node));

            std::vector<JointId> result;
            std::unordered_set<JointId> seen;
            for(const auto &entry : *array) {
                const auto id = entry.value<std::int64_t>();
                if(!id || *id < 0 || *id >= static_cast<std::int64_t>(MAX_JOINTS))
                    return std::unexpected(configurationError(
                        path, field, std::format("joint IDs must be between 0 and {}", MAX_JOINTS - 1), &entry));
                const auto converted = static_cast<JointId>(*id);
                if(!seen.insert(converted).second)
                    return std::unexpected(configurationError(path, field, "contains a duplicate joint ID", &entry));
                result.push_back(converted);
            }
            return result;
        }
    }

    std::expected<MachineConfiguration, std::string>
    loadMachineConfiguration(const std::filesystem::path &path) {
        try {
            const auto document = toml::parse_file(path.string());
            const auto *machine = document["machine"].as_table();
            const auto *trajectory = document["trajectory"].as_table();
            const auto *simulation = document["simulation"].as_table();
            const auto *jogging = document["jogging"].as_table();
            const auto *axes = document["axes"].as_table();
            const auto *digitalInputs = document["digital_inputs"].as_table();
            const auto *probing = document["probing"].as_table();
            const auto *joints = document["joints"].as_array();
            const auto *homing = document["homing"].as_table();
            if(!machine) return std::unexpected(configurationError(path, "machine", "missing table"));
            if(!trajectory) return std::unexpected(configurationError(path, "trajectory", "missing table"));
            if(!simulation) return std::unexpected(configurationError(path, "simulation", "missing table"));
            if(!jogging) return std::unexpected(configurationError(path, "jogging", "missing table"));
            if(!axes) return std::unexpected(configurationError(path, "axes", "missing table"));
            if(!digitalInputs)
                return std::unexpected(configurationError(path, "digital_inputs", "missing table"));
            if(!probing) return std::unexpected(configurationError(path, "probing", "missing table"));
            if(!joints || joints->empty())
                return std::unexpected(configurationError(path, "joints", "must be a non-empty array of tables"));
            if(!homing) return std::unexpected(configurationError(path, "homing", "missing table"));

            MachineConfiguration result;
            const auto units = requiredString(*machine, "units", path);
            if(!units) return std::unexpected(units.error());
            if(*units != "inch" && *units != "mm")
                return std::unexpected(configurationError(path, "machine.units", "must be 'inch' or 'mm'",
                                                          machine->get("units")));
            result.unit = *units == "inch" ? Machine::Unit::Inch : Machine::Unit::Millimeter;

            const auto *coordinateNode = machine->get("coordinates");
            const auto *coordinates = coordinateNode ? coordinateNode->as_array() : nullptr;
            if(!coordinates || coordinates->empty())
                return std::unexpected(configurationError(
                    path, "machine.coordinates", "must be a non-empty axis-name array", coordinateNode));
            for(const auto &entry : *coordinates) {
                const auto name = entry.value<std::string>();
                if(!name)
                    return std::unexpected(configurationError(
                        path, "machine.coordinates", "entries must be axis names", &entry));
                const auto axis = parseAxis(*name, path, "machine.coordinates", &entry);
                if(!axis) return std::unexpected(axis.error());
                if(std::ranges::find(result.coordinates, *axis) != result.coordinates.end())
                    return std::unexpected(configurationError(
                        path, "machine.coordinates", "contains a duplicate axis", &entry));
                result.coordinates.push_back(*axis);
            }

            const auto acceleration = positiveNumber(*trajectory, "path_acceleration", path);
            const auto jerk = positiveNumber(*trajectory, "path_jerk", path);
            const auto rapidVelocity = positiveNumber(*trajectory, "rapid_velocity", path);
            const auto chordTolerance = positiveNumber(*trajectory, "arc_chord_tolerance", path);
            const auto servoPeriod = positiveNumber(*simulation, "servo_period", path);
            const auto schedulerPeriod = positiveNumber(*simulation, "scheduler_period", path);
            const auto jogAcceleration = positiveNumber(*jogging, "acceleration", path);
            const auto jogJerk = positiveNumber(*jogging, "jerk", path);
            if(!acceleration) return std::unexpected(acceleration.error());
            if(!jerk) return std::unexpected(jerk.error());
            if(!rapidVelocity) return std::unexpected(rapidVelocity.error());
            if(!chordTolerance) return std::unexpected(chordTolerance.error());
            if(!servoPeriod) return std::unexpected(servoPeriod.error());
            if(!schedulerPeriod) return std::unexpected(schedulerPeriod.error());
            if(!jogAcceleration) return std::unexpected(jogAcceleration.error());
            if(!jogJerk) return std::unexpected(jogJerk.error());
            const auto schedulerTicks = *schedulerPeriod / *servoPeriod;
            if(schedulerTicks < 1.0 || std::abs(schedulerTicks - std::round(schedulerTicks)) > 1e-9)
                return std::unexpected(configurationError(
                    path, "simulation.scheduler_period", "must be an integer multiple of simulation.servo_period",
                    simulation->get("scheduler_period")));
            result.trajectory.pathAcceleration = *acceleration;
            result.trajectory.pathJerk = *jerk;
            result.trajectory.rapidSpeed = *rapidVelocity * 60.0;
            result.trajectory.arcChordTolerance = *chordTolerance;
            result.simulation = { *servoPeriod, *schedulerPeriod };
            result.jogging = { *jogAcceleration, *jogJerk };

            std::unordered_set<JointId> axisJointIds;
            for(const auto axis : result.coordinates) {
                const auto name = axisName(axis);
                const auto *axisTable = axes->get(name) ? axes->get(name)->as_table() : nullptr;
                const auto prefix = std::format("axes.{}", name);
                if(!axisTable)
                    return std::unexpected(configurationError(path, prefix, "missing table", axes->get(name)));
                auto jointIds = parseJointIds(*axisTable, "joints", path, prefix + ".joints");
                auto minimum = number(*axisTable, "minimum", path);
                auto maximum = number(*axisTable, "maximum", path);
                auto maxVelocity = positiveNumber(*axisTable, "max_velocity", path);
                auto maxAcceleration = positiveNumber(*axisTable, "max_acceleration", path);
                if(!jointIds) return std::unexpected(jointIds.error());
                if(!minimum) return std::unexpected(minimum.error());
                if(!maximum) return std::unexpected(maximum.error());
                if(!maxVelocity) return std::unexpected(maxVelocity.error());
                if(!maxAcceleration) return std::unexpected(maxAcceleration.error());
                if(*minimum >= *maximum)
                    return std::unexpected(configurationError(path, prefix, "minimum must be less than maximum",
                                                              axisTable));
                for(const auto id : *jointIds)
                    if(!axisJointIds.insert(id).second)
                        return std::unexpected(configurationError(
                            path, prefix + ".joints", "a joint may belong to only one logical axis", axisTable));
                result.axes.push_back({ axis, std::move(*jointIds), *minimum, *maximum,
                                        *maxVelocity, *maxAcceleration });
            }

            std::unordered_set<DigitalInputId> inputIds;
            for(const auto &[key, node] : *digitalInputs) {
                const auto value = node.value<std::int64_t>();
                if(!value || *value < 0 || *value > std::numeric_limits<DigitalInputId>::max())
                    return std::unexpected(configurationError(
                        path, std::format("digital_inputs.{}", key.str()), "must be a valid non-negative input ID",
                        &node));
                const auto id = static_cast<DigitalInputId>(*value);
                if(!inputIds.insert(id).second)
                    return std::unexpected(configurationError(
                        path, std::format("digital_inputs.{}", key.str()), "duplicates another input ID", &node));
                result.digitalInputs.push_back({ std::string(key.str()), id });
            }
            if(result.digitalInputs.empty())
                return std::unexpected(configurationError(path, "digital_inputs", "must not be empty", digitalInputs));

            const auto probeInputName = requiredString(*probing, "input", path);
            const auto probeConditionName = requiredString(*probing, "condition", path);
            const auto probeDebounce = number(*probing, "debounce", path);
            if(!probeInputName) return std::unexpected(probeInputName.error());
            if(!probeConditionName) return std::unexpected(probeConditionName.error());
            if(!probeDebounce) return std::unexpected(probeDebounce.error());
            const auto *probeInput = findInput(result.digitalInputs, *probeInputName);
            if(!probeInput)
                return std::unexpected(configurationError(path, "probing.input", "references an unknown input",
                                                          probing->get("input")));
            const auto probeCondition = parseCondition(*probeConditionName, path, "probing.condition",
                                                       probing->get("condition"));
            if(!probeCondition) return std::unexpected(probeCondition.error());
            if(*probeDebounce < 0.0)
                return std::unexpected(configurationError(path, "probing.debounce", "must not be negative",
                                                          probing->get("debounce")));
            result.probing = { probeInput->id, *probeCondition, *probeDebounce };

            std::unordered_set<JointId> configuredJointIds;
            std::unordered_set<std::string> jointNames;
            for(const auto &entry : *joints) {
                const auto *table = entry.as_table();
                if(!table)
                    return std::unexpected(configurationError(path, "joints", "entries must be tables", &entry));
                const auto idValue = integer(*table, "id", path);
                const auto name = requiredString(*table, "name", path);
                const auto axisText = requiredString(*table, "axis", path);
                if(!idValue) return std::unexpected(idValue.error());
                if(!name) return std::unexpected(name.error());
                if(!axisText) return std::unexpected(axisText.error());
                if(*idValue < 0 || *idValue >= static_cast<std::int64_t>(MAX_JOINTS))
                    return std::unexpected(configurationError(path, "joints.id", "is outside the backend joint range",
                                                              table->get("id")));
                const auto id = static_cast<JointId>(*idValue);
                if(!configuredJointIds.insert(id).second)
                    return std::unexpected(configurationError(path, "joints.id", "is duplicated", table->get("id")));
                if(!jointNames.insert(*name).second)
                    return std::unexpected(configurationError(path, "joints.name", "is duplicated", table->get("name")));
                const auto axis = parseAxis(*axisText, path, "joints.axis", table->get("axis"));
                if(!axis) return std::unexpected(axis.error());
                if(std::ranges::find(result.coordinates, *axis) == result.coordinates.end())
                    return std::unexpected(configurationError(path, "joints.axis", "is not a configured coordinate",
                                                              table->get("axis")));

                auto coordinateScale = number(*table, "coordinate_scale", path);
                auto minimum = number(*table, "minimum", path);
                auto maximum = number(*table, "maximum", path);
                auto maxVelocity = positiveNumber(*table, "max_velocity", path);
                auto maxAcceleration = positiveNumber(*table, "max_acceleration", path);
                auto maxJerk = positiveNumber(*table, "max_jerk", path);
                if(!coordinateScale) return std::unexpected(coordinateScale.error());
                if(!minimum) return std::unexpected(minimum.error());
                if(!maximum) return std::unexpected(maximum.error());
                if(!maxVelocity) return std::unexpected(maxVelocity.error());
                if(!maxAcceleration) return std::unexpected(maxAcceleration.error());
                if(!maxJerk) return std::unexpected(maxJerk.error());
                if(*coordinateScale == 0.0)
                    return std::unexpected(configurationError(path, "joints.coordinate_scale", "must not be zero",
                                                              table->get("coordinate_scale")));
                if(*minimum >= *maximum)
                    return std::unexpected(configurationError(path, "joints", "minimum must be less than maximum", table));

                const auto *home = table->get("homing") ? table->get("homing")->as_table() : nullptr;
                if(!home)
                    return std::unexpected(configurationError(path, "joints.homing", "missing table", table));
                const auto inputName = requiredString(*home, "input", path);
                const auto conditionName = requiredString(*home, "condition", path);
                auto homePosition = number(*home, "home_position", path);
                auto switchPosition = number(*home, "switch_position", path);
                auto searchVelocity = number(*home, "search_velocity", path);
                auto latchVelocity = number(*home, "latch_velocity", path);
                auto backoffDistance = positiveNumber(*home, "backoff_distance", path);
                auto debounce = number(*home, "debounce", path);
                auto finalVelocity = number(*home, "final_velocity", path);
                auto useIndex = requiredBool(*home, "use_index", path);
                if(!inputName) return std::unexpected(inputName.error());
                if(!conditionName) return std::unexpected(conditionName.error());
                if(!homePosition) return std::unexpected(homePosition.error());
                if(!switchPosition) return std::unexpected(switchPosition.error());
                if(!searchVelocity) return std::unexpected(searchVelocity.error());
                if(!latchVelocity) return std::unexpected(latchVelocity.error());
                if(!backoffDistance) return std::unexpected(backoffDistance.error());
                if(!debounce) return std::unexpected(debounce.error());
                if(!finalVelocity) return std::unexpected(finalVelocity.error());
                if(!useIndex) return std::unexpected(useIndex.error());
                if(*searchVelocity == 0.0 || *latchVelocity == 0.0)
                    return std::unexpected(configurationError(
                        path, "joints.homing", "search_velocity and latch_velocity must be nonzero", home));
                if(*debounce < 0.0)
                    return std::unexpected(configurationError(path, "joints.homing.debounce",
                                                              "must not be negative", home->get("debounce")));
                const auto *input = findInput(result.digitalInputs, *inputName);
                if(!input)
                    return std::unexpected(configurationError(path, "joints.homing.input",
                                                              "references an unknown input", home->get("input")));
                const auto condition = parseCondition(*conditionName, path, "joints.homing.condition",
                                                      home->get("condition"));
                if(!condition) return std::unexpected(condition.error());

                result.joints.push_back({ id, *name, *axis, *coordinateScale, *minimum, *maximum,
                    *maxVelocity, *maxAcceleration, *maxJerk,
                    { input->id, *condition, *homePosition, *switchPosition, *searchVelocity,
                      *latchVelocity, *backoffDistance, *debounce, *finalVelocity, *useIndex } });
            }

            if(configuredJointIds != axisJointIds)
                return std::unexpected(configurationError(
                    path, "axes", "axis joint lists must reference every configured joint exactly once", axes));
            for(const auto &axis : result.axes)
                for(const auto id : axis.joints) {
                    const auto *joint = findJoint(result.joints, id);
                    if(!joint || joint->axis != axis.axis)
                        return std::unexpected(configurationError(
                            path, std::format("axes.{}.joints", axisName(axis.axis)),
                            "joint axis does not match its logical-axis table", axes));
                }

            const auto requireBeforeMotion = requiredBool(*homing, "require_before_motion", path);
            if(!requireBeforeMotion) return std::unexpected(requireBeforeMotion.error());
            result.homing.requireBeforeMotion = *requireBeforeMotion;
            const auto *groupNode = homing->get("groups");
            const auto *groups = groupNode ? groupNode->as_array() : nullptr;
            if(!groups || groups->empty())
                return std::unexpected(configurationError(path, "homing.groups", "must be a non-empty array", groupNode));
            std::unordered_set<std::string> groupNames;
            std::unordered_set<std::uint32_t> sequences;
            std::unordered_set<JointId> groupedJoints;
            for(const auto &entry : *groups) {
                const auto *table = entry.as_table();
                if(!table)
                    return std::unexpected(configurationError(path, "homing.groups", "entries must be tables", &entry));
                const auto name = requiredString(*table, "name", path);
                const auto sequence = integer(*table, "sequence", path);
                auto ids = parseJointIds(*table, "joints", path, "homing.groups.joints");
                if(!name) return std::unexpected(name.error());
                if(!sequence) return std::unexpected(sequence.error());
                if(!ids) return std::unexpected(ids.error());
                if(*sequence < 0 || *sequence > std::numeric_limits<std::uint32_t>::max())
                    return std::unexpected(configurationError(path, "homing.groups.sequence",
                                                              "must be a non-negative 32-bit integer",
                                                              table->get("sequence")));
                const auto convertedSequence = static_cast<std::uint32_t>(*sequence);
                if(!groupNames.insert(*name).second)
                    return std::unexpected(configurationError(path, "homing.groups.name", "is duplicated",
                                                              table->get("name")));
                if(!sequences.insert(convertedSequence).second)
                    return std::unexpected(configurationError(path, "homing.groups.sequence", "is duplicated",
                                                              table->get("sequence")));
                for(const auto id : *ids) {
                    if(!findJoint(result.joints, id))
                        return std::unexpected(configurationError(path, "homing.groups.joints",
                                                                  "references an unknown joint", table->get("joints")));
                    if(!groupedJoints.insert(id).second)
                        return std::unexpected(configurationError(path, "homing.groups.joints",
                                                                  "a joint may belong to only one homing group",
                                                                  table->get("joints")));
                }
                const auto optionalBool = [&](const std::string_view key) -> std::expected<bool, std::string> {
                    if(!table->contains(key)) return false;
                    return requiredBool(*table, key, path);
                };
                auto startTogether = optionalBool("start_together");
                auto stopSeparately = optionalBool("stop_each_joint_on_trigger");
                auto finalTogether = optionalBool("final_move_together");
                if(!startTogether) return std::unexpected(startTogether.error());
                if(!stopSeparately) return std::unexpected(stopSeparately.error());
                if(!finalTogether) return std::unexpected(finalTogether.error());
                result.homing.groups.push_back({ *name, convertedSequence, std::move(*ids),
                    *startTogether, *stopSeparately, *finalTogether });
            }
            if(groupedJoints != configuredJointIds)
                return std::unexpected(configurationError(
                    path, "homing.groups", "must include every configured joint exactly once", groupNode));
            std::ranges::sort(result.homing.groups, {}, &HomingGroupConfiguration::sequence);

            return result;
        } catch(const toml::parse_error &error) {
            return std::unexpected(std::format("{}:{}:{}: {}", path.string(), error.source().begin.line,
                                               error.source().begin.column, error.description()));
        }
    }
}
