#pragma once

#include <functional>
#include <optional>
#include <ranges>
#include <type_traits>
#include <utility>
#include <vector>

#include "machine/MachineCommand.h"

namespace ngc {
    class ToolpathRecorder {
        std::vector<MachineCommand> m_commands;
        std::vector<WorkCoordinateSystem> m_workCoordinateSystems;
        std::uint64_t m_revision = 0;

    public:
        void clear() { m_commands.clear(); m_workCoordinateSystems.clear(); ++m_revision; }

        void consume(MachineCommand command, const position_t &toolOffset = {},
                     std::optional<WorkCoordinateSystem> workCoordinateSystem = std::nullopt) {
            if(workCoordinateSystem) {
                const auto match = std::ranges::find_if(m_workCoordinateSystems, [&](const WorkCoordinateSystem &value) {
                    return value.name == workCoordinateSystem->name;
                });
                if(match == m_workCoordinateSystems.end()) m_workCoordinateSystems.push_back(*workCoordinateSystem);
                else *match = *workCoordinateSystem;
            }
            m_commands.emplace_back(std::visit([&](auto &&value) -> MachineCommand {
                using T = std::decay_t<decltype(value)>;

                if constexpr(std::same_as<T, MoveLine>) {
                    return MoveLine {
                        value.from() - toolOffset,
                        value.to() - toolOffset,
                        value.speed(),
                        value.machineCoordinates(),
                    };
                } else if constexpr(std::same_as<T, MoveArc>) {
                    const auto &center = value.center();
                    return MoveArc {
                        value.from() - toolOffset,
                        value.to() - toolOffset,
                        { center.x - toolOffset.x, center.y - toolOffset.y, center.z - toolOffset.z },
                        value.axis(),
                        value.speed(),
                    };
                } else if constexpr(std::same_as<T, ProbeMove>) {
                    return ProbeMove {
                        value.id(),
                        value.from() - toolOffset,
                        value.target() - toolOffset,
                        value.feed(),
                        value.stopOnContact(),
                        value.errorIfNotFound(),
                    };
                } else {
                    return std::forward<decltype(value)>(value);
                }
            }, std::move(command)));
            ++m_revision;
        }

        const std::vector<MachineCommand> &commands() const { return m_commands; }
        const std::vector<WorkCoordinateSystem> &workCoordinateSystems() const { return m_workCoordinateSystems; }
        std::uint64_t revision() const { return m_revision; }

        void foreachCommand(const std::function<void(const MachineCommand &)> &callback) const {
            for(const auto &command : m_commands) {
                callback(command);
            }
        }
    };
}
