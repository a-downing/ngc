#include "machine/SimulationExecutor.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <numbers>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>
#include <unordered_map>

#include "machine/MachineCommand.h"
#include "evaluator/InterpreterStatus.h"

namespace ngc {
#if 0 // Public value types and geometry declarations live in SimulationExecutor.h.
    enum class SimulationStatus {
        Stopped,
        Running,
        Paused,
        Completed,
        Error,
    };

    struct SimulatedCommand {
        std::optional<MachineCommand> command;
        position_t toolOffset{};
        ToolGeometry tool{};
        std::optional<WorkCoordinateSystem> workCoordinateSystem;
        std::vector<std::string> modalGCodes;
        std::optional<BlockExecution> block;
        std::optional<InterpreterBlockLifecycle> lifecycle;

        SimulatedCommand(MachineCommand commandValue, position_t activeToolOffset = {}, ToolGeometry toolGeometry = {},
                         std::optional<WorkCoordinateSystem> activeWorkCoordinateSystem = std::nullopt,
                         std::vector<std::string> activeModalGCodes = {},
                         std::optional<BlockExecution> blockExecution = std::nullopt)
            : command(std::move(commandValue)), toolOffset(activeToolOffset), tool(std::move(toolGeometry)),
              workCoordinateSystem(std::move(activeWorkCoordinateSystem)), modalGCodes(std::move(activeModalGCodes)),
              block(std::move(blockExecution)) { }

        static SimulatedCommand lifecycleMarker(InterpreterBlockLifecycle blockLifecycle) {
            SimulatedCommand marker { SpindleStop {} };
            marker.command.reset();
            marker.lifecycle = std::move(blockLifecycle);
            return marker;
        }
    };

    struct SimulationSnapshot {
        SimulationStatus status = SimulationStatus::Stopped;
        position_t machinePosition{};
        position_t toolPosition{};
        ToolPose toolPose{};
        double commandProgress = 0.0;
        bool hasActiveMotion = false;
        bool spindleRunning = false;
        double spindleSpeed = 0.0;
        Direction spindleDirection = Direction::CW;
        std::string error;
        std::vector<InterpreterStatusMessage> statusMessages;
        std::optional<WorkCoordinateSystem> activeWorkCoordinateSystem;
        std::vector<std::string> activeModalGCodes;
        std::vector<WorkCoordinateSystem> usedWorkCoordinateSystems;
        std::vector<BlockExecution> activeBlocks;
        std::vector<BlockExecution> completedBlocks;
        std::unordered_map<std::string, std::vector<std::uint8_t>> completedLineFlags;
    };

    namespace simulation_detail {
        inline position_t mix(const position_t &from, const position_t &to, const double t) {
            return {
                std::lerp(from.x, to.x, t), std::lerp(from.y, to.y, t), std::lerp(from.z, to.z, t),
                std::lerp(from.a, to.a, t), std::lerp(from.b, to.b, t), std::lerp(from.c, to.c, t),
            };
        }

        inline double linearDistance(const position_t &from, const position_t &to) {
            return vec3_t { to.x - from.x, to.y - from.y, to.z - from.z }.length();
        }

        inline double dot(const vec3_t &a, const vec3_t &b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
        inline vec3_t cross(const vec3_t &a, const vec3_t &b) {
            return { a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x };
        }
        inline vec3_t scale(const vec3_t &v, const double s) { return { v.x*s, v.y*s, v.z*s }; }
        inline vec3_t normalize(const vec3_t &v) { return scale(v, 1.0 / v.length()); }
        inline vec3_t rotate(const vec3_t &v, const double angle, const vec3_t &axisUnit) {
            return scale(v, std::cos(angle))
                + scale(cross(axisUnit, v), std::sin(angle))
                + scale(axisUnit, dot(axisUnit, v) * (1.0 - std::cos(angle)));
        }

        struct ArcGeometry {
            vec3_t center;
            vec3_t axisUnit;
            vec3_t startArm;
            vec3_t endArm;
            vec3_t axial;
            double sweep;
        };

        inline std::optional<ArcGeometry> arcGeometry(const MoveArc &arc) {
            const vec3_t start { arc.from().x, arc.from().y, arc.from().z };
            const vec3_t end { arc.to().x, arc.to().y, arc.to().z };
            const auto axisLength = arc.axis().length();
            if(axisLength == 0.0) return std::nullopt;

            const auto axisUnit = normalize(arc.axis());
            const auto startDelta = start - arc.center();
            const auto endDelta = end - arc.center();
            const auto startArm = startDelta - scale(axisUnit, dot(startDelta, axisUnit));
            const auto endArm = endDelta - scale(axisUnit, dot(endDelta, axisUnit));
            if(startArm.length() == 0.0 || endArm.length() == 0.0) return std::nullopt;

            auto sweep = std::atan2(dot(axisUnit, cross(normalize(startArm), normalize(endArm))),
                                    dot(normalize(startArm), normalize(endArm)));
            if(sweep < 0.0) sweep += 2.0 * std::numbers::pi;
            if((startArm - endArm).length() < 1e-9) sweep = 2.0 * std::numbers::pi;

            return ArcGeometry {
                .center = arc.center(),
                .axisUnit = axisUnit,
                .startArm = startArm,
                .endArm = endArm,
                .axial = scale(axisUnit, dot(end - start, axisUnit)),
                .sweep = sweep,
            };
        }

        inline position_t interpolate(const MoveArc &arc, const double t) {
            const auto geometry = arcGeometry(arc);
            if(!geometry) return mix(arc.from(), arc.to(), t);

            const auto fromStart = rotate(geometry->startArm, geometry->sweep * t, geometry->axisUnit);
            const auto fromEnd = rotate(geometry->endArm, -(geometry->sweep * (1.0 - t)), geometry->axisUnit);
            const auto radial = scale(fromStart, 1.0 - t) + scale(fromEnd, t);
            const auto xyz = geometry->center + radial + scale(geometry->axial, t);
            auto result = mix(arc.from(), arc.to(), t);
            result.x = xyz.x;
            result.y = xyz.y;
            result.z = xyz.z;
            return result;
        }

        inline double pathLength(const MoveArc &arc) {
            const auto geometry = arcGeometry(arc);
            if(!geometry) return (arc.to() - arc.from()).length();
            const auto radius = 0.5 * (geometry->startArm.length() + geometry->endArm.length());
            return std::hypot(radius * geometry->sweep, geometry->axial.length());
        }
    }

#endif
    namespace simulation_detail {
        position_t mix(const position_t &from, const position_t &to, const double t) {
            return { std::lerp(from.x, to.x, t), std::lerp(from.y, to.y, t), std::lerp(from.z, to.z, t),
                     std::lerp(from.a, to.a, t), std::lerp(from.b, to.b, t), std::lerp(from.c, to.c, t) };
        }
        double linearDistance(const position_t &from, const position_t &to) {
            return vec3_t { to.x - from.x, to.y - from.y, to.z - from.z }.length();
        }
        static double dot(const vec3_t &a, const vec3_t &b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
        static vec3_t cross(const vec3_t &a, const vec3_t &b) {
            return { a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x };
        }
        static vec3_t scale(const vec3_t &v, const double s) { return { v.x*s, v.y*s, v.z*s }; }
        static vec3_t normalize(const vec3_t &v) { return scale(v, 1.0 / v.length()); }
        static vec3_t rotate(const vec3_t &v, const double angle, const vec3_t &axisUnit) {
            return scale(v, std::cos(angle)) + scale(cross(axisUnit, v), std::sin(angle))
                + scale(axisUnit, dot(axisUnit, v) * (1.0 - std::cos(angle)));
        }
        std::optional<ArcGeometry> arcGeometry(const MoveArc &arc) {
            const vec3_t start { arc.from().x, arc.from().y, arc.from().z };
            const vec3_t end { arc.to().x, arc.to().y, arc.to().z };
            if(arc.axis().length() == 0.0) return std::nullopt;
            const auto axisUnit = normalize(arc.axis());
            const auto startDelta = start - arc.center();
            const auto endDelta = end - arc.center();
            const auto startArm = startDelta - scale(axisUnit, dot(startDelta, axisUnit));
            const auto endArm = endDelta - scale(axisUnit, dot(endDelta, axisUnit));
            if(startArm.length() == 0.0 || endArm.length() == 0.0) return std::nullopt;
            auto sweep = std::atan2(dot(axisUnit, cross(normalize(startArm), normalize(endArm))),
                                    dot(normalize(startArm), normalize(endArm)));
            if(sweep < 0.0) sweep += 2.0 * std::numbers::pi;
            if((startArm - endArm).length() < 1e-9) sweep = 2.0 * std::numbers::pi;
            return ArcGeometry { arc.center(), axisUnit, startArm, endArm,
                                 scale(axisUnit, dot(end - start, axisUnit)), sweep };
        }
        position_t interpolate(const MoveArc &arc, const double t) {
            const auto geometry = arcGeometry(arc);
            if(!geometry) return mix(arc.from(), arc.to(), t);
            const auto fromStart = rotate(geometry->startArm, geometry->sweep * t, geometry->axisUnit);
            const auto fromEnd = rotate(geometry->endArm, -(geometry->sweep * (1.0 - t)), geometry->axisUnit);
            const auto radial = scale(fromStart, 1.0 - t) + scale(fromEnd, t);
            const auto xyz = geometry->center + radial + scale(geometry->axial, t);
            auto result = mix(arc.from(), arc.to(), t);
            result.x = xyz.x; result.y = xyz.y; result.z = xyz.z;
            return result;
        }
        double pathLength(const MoveArc &arc) {
            const auto geometry = arcGeometry(arc);
            if(!geometry) return (arc.to() - arc.from()).length();
            const auto radius = 0.5 * (geometry->startArm.length() + geometry->endArm.length());
            return std::hypot(radius * geometry->sweep, geometry->axial.length());
        }
    }

    class SimulationExecutor::Impl {
        static constexpr std::size_t QUEUE_CAPACITY = 32;

        std::deque<SimulatedCommand> m_queue;
        std::optional<SimulatedCommand> m_active;
        SimulationSnapshot m_snapshot;
        position_t m_lastToolOffset{};
        position_t m_motionStart{};
        double m_elapsed = 0.0;
        double m_duration = 0.0;
        double m_rapidSpeed = 100.0;
        std::optional<ProbeResult> m_probeResult;

    public:
        void reset() {
            m_queue.clear();
            m_active.reset();
            m_snapshot = {};
            m_lastToolOffset = {};
            m_motionStart = {};
            m_elapsed = 0.0;
            m_duration = 0.0;
            m_probeResult.reset();
        }

        void prepareContinuation() {
            m_queue.clear();
            m_active.reset();
            m_elapsed = 0.0;
            m_duration = 0.0;
            m_motionStart = m_snapshot.machinePosition;
            m_probeResult.reset();
            m_snapshot.status = SimulationStatus::Stopped;
            m_snapshot.error.clear();
            m_snapshot.commandProgress = 0.0;
            m_snapshot.hasActiveMotion = false;
            m_snapshot.activeBlocks.clear();
        }

        bool canAccept() const { return m_queue.size() + (m_active ? 1 : 0) < QUEUE_CAPACITY; }
        bool empty() const { return m_queue.empty() && !m_active; }
        void setRapidSpeed(const double speed) { m_rapidSpeed = std::max(speed, 1e-9); }
        void setStatus(const SimulationStatus status) { m_snapshot.status = status; }
        void setError(std::string error) {
            m_queue.clear();
            m_active.reset();
            m_probeResult.reset();
            m_snapshot.status = SimulationStatus::Error;
            m_snapshot.error = std::move(error);
        }

        void consume(SimulatedCommand command) { m_queue.emplace_back(std::move(command)); }

        void advance(double seconds) {
            seconds = std::max(seconds, 0.0);
            while(seconds > 0.0 || (!m_active && !m_queue.empty())) {
                if(!m_active) {
                    if(m_queue.empty()) {
                        break;
                    }
                    m_active.emplace(std::move(m_queue.front()));
                    m_queue.pop_front();
                    if(m_active->workCoordinateSystem) {
                        m_snapshot.activeWorkCoordinateSystem = m_active->workCoordinateSystem;
                        const auto match = std::ranges::find_if(m_snapshot.usedWorkCoordinateSystems,
                            [&](const WorkCoordinateSystem &value) {
                                return value.name == m_active->workCoordinateSystem->name;
                            });
                        if(match == m_snapshot.usedWorkCoordinateSystems.end()) {
                            m_snapshot.usedWorkCoordinateSystems.push_back(*m_active->workCoordinateSystem);
                        } else {
                            *match = *m_active->workCoordinateSystem;
                        }
                    }
                    if(!m_active->modalGCodes.empty()) {
                        m_snapshot.activeModalGCodes = m_active->modalGCodes;
                    }
                    m_elapsed = 0.0;
                    m_motionStart = m_snapshot.machinePosition;
                    m_duration = commandDuration(*m_active);
                    if(!m_active->command || !isMotion(*m_active->command)) {
                        completeActive();
                        continue;
                    }
                }

                const auto remaining = std::max(m_duration - m_elapsed, 0.0);
                const auto consumed = std::min(seconds, remaining);
                m_elapsed += consumed;
                seconds -= consumed;
                updateActive();

                if(m_elapsed + 1e-12 >= m_duration) {
                    completeActive();
                } else {
                    break;
                }
            }
        }

        void completeQueued() {
            while(!empty() && !m_probeResult) {
                if(!m_active) {
                    advance(0.0);
                } else {
                    advance(std::max(m_duration - m_elapsed, 0.0));
                }
            }
        }

        SimulationSnapshot snapshot() const { return m_snapshot; }

        SimulationSnapshot lightweightSnapshot() const {
            SimulationSnapshot result;
            result.status = m_snapshot.status;
            result.machinePosition = m_snapshot.machinePosition;
            result.toolPosition = m_snapshot.toolPosition;
            result.toolPose = m_snapshot.toolPose;
            result.commandProgress = m_snapshot.commandProgress;
            result.hasActiveMotion = m_snapshot.hasActiveMotion;
            result.spindleRunning = m_snapshot.spindleRunning;
            result.spindleSpeed = m_snapshot.spindleSpeed;
            result.spindleDirection = m_snapshot.spindleDirection;
            result.error = m_snapshot.error;
            result.activeWorkCoordinateSystem = m_snapshot.activeWorkCoordinateSystem;
            result.activeModalGCodes = m_snapshot.activeModalGCodes;
            result.usedWorkCoordinateSystems = m_snapshot.usedWorkCoordinateSystems;
            result.activeBlocks = m_snapshot.activeBlocks;
            result.completedLineFlags = m_snapshot.completedLineFlags;
            return result;
        }

        std::optional<ProbeResult> takeProbeResult() {
            return std::exchange(m_probeResult, std::nullopt);
        }

    private:
        static bool isMotion(const MachineCommand &command) {
            return std::holds_alternative<MoveLine>(command)
                || std::holds_alternative<MoveArc>(command)
                || std::holds_alternative<ProbeMove>(command);
        }

        static position_t probeContactPosition(const SimulatedCommand &command, const ProbeMove &probe) {
            return probe.target() + command.tool.offset - command.toolOffset;
        }

        double commandDuration(const SimulatedCommand &simulated) const {
            if(!simulated.command) return 0.0;
            return std::visit([&](const auto &value) {
                using T = std::decay_t<decltype(value)>;
                if constexpr(std::same_as<T, MoveLine>) {
                    const auto speed = value.speed() < 0.0 ? m_rapidSpeed : value.speed();
                    return 60.0 * simulation_detail::linearDistance(m_motionStart, value.to()) / std::max(speed, 1e-9);
                } else if constexpr(std::same_as<T, MoveArc>) {
                    return 60.0 * simulation_detail::pathLength(value) / std::max(value.speed(), 1e-9);
                } else if constexpr(std::same_as<T, ProbeMove>) {
                    return 60.0 * simulation_detail::linearDistance(m_motionStart, probeContactPosition(simulated, value)) / std::max(value.feed(), 1e-9);
                } else {
                    return 0.0;
                }
            }, *simulated.command);
        }

        void updateActive() {
            const auto progress = m_duration > 0.0 ? std::clamp(m_elapsed / m_duration, 0.0, 1.0) : 1.0;
            m_snapshot.commandProgress = progress;
            m_snapshot.hasActiveMotion = true;
            m_snapshot.machinePosition = std::visit([&](const auto &value) -> position_t {
                using T = std::decay_t<decltype(value)>;
                if constexpr(std::same_as<T, MoveLine>) return simulation_detail::mix(m_motionStart, value.to(), progress);
                if constexpr(std::same_as<T, MoveArc>) return simulation_detail::interpolate(value, progress);
                if constexpr(std::same_as<T, ProbeMove>) return simulation_detail::mix(m_motionStart, probeContactPosition(*m_active, value), progress);
                return m_snapshot.machinePosition;
            }, *m_active->command);
            m_lastToolOffset = m_active->tool.offset;
            m_snapshot.toolPosition = m_snapshot.machinePosition - m_lastToolOffset;
            m_snapshot.toolPose = {
                .geometry = m_active->tool,
                .spindlePosition = m_snapshot.machinePosition,
                .tipPosition = m_snapshot.toolPosition,
            };
        }

        void completeActive() {
            if(!m_active->command) {
                if(m_active->lifecycle) {
                    const auto &lifecycle = *m_active->lifecycle;
                    if(lifecycle.phase == BlockLifecyclePhase::Entered) {
                        m_snapshot.activeBlocks.emplace_back(lifecycle.block);
                    } else {
                        const auto match = std::ranges::find_if(m_snapshot.activeBlocks, [&](const BlockExecution &block) {
                            return block.id == lifecycle.block.id;
                        });
                        if(match != m_snapshot.activeBlocks.end()) m_snapshot.activeBlocks.erase(match);
                        m_snapshot.completedBlocks.emplace_back(lifecycle.block);
                        auto &flags = m_snapshot.completedLineFlags[lifecycle.block.source];
                        if(lifecycle.block.line >= static_cast<int>(flags.size())) {
                            flags.resize(static_cast<std::size_t>(lifecycle.block.line) + 1);
                        }
                        flags[lifecycle.block.line] = 1;
                    }
                }
                m_snapshot.hasActiveMotion = false;
                m_snapshot.commandProgress = 1.0;
                m_active.reset();
                return;
            }
            std::visit([&](const auto &value) {
                using T = std::decay_t<decltype(value)>;
                if constexpr(std::same_as<T, SpindleStart>) {
                    m_snapshot.spindleRunning = true;
                    m_snapshot.spindleSpeed = value.speed();
                    m_snapshot.spindleDirection = value.direction();
                } else if constexpr(std::same_as<T, SpindleStop>) {
                    m_snapshot.spindleRunning = false;
                    m_snapshot.spindleSpeed = 0.0;
                } else if constexpr(std::same_as<T, ProbeMove>) {
                    updateActive();
                    const auto contact = probeContactPosition(*m_active, value);
                    m_probeResult = ProbeResult {
                        .id = value.id(),
                        .status = ProbeStatus::Triggered,
                        .triggerPosition = contact,
                        .stoppedPosition = contact,
                    };
                } else {
                    updateActive();
                }
            }, *m_active->command);
            m_snapshot.hasActiveMotion = false;
            m_snapshot.commandProgress = 1.0;
            m_active.reset();
        }
    };

    SimulationExecutor::SimulationExecutor() : m_impl(std::make_unique<Impl>()) { }
    SimulationExecutor::~SimulationExecutor() = default;
    SimulationExecutor::SimulationExecutor(SimulationExecutor &&) noexcept = default;
    SimulationExecutor &SimulationExecutor::operator=(SimulationExecutor &&) noexcept = default;
    void SimulationExecutor::reset() { m_impl->reset(); }
    void SimulationExecutor::prepareContinuation() { m_impl->prepareContinuation(); }
    bool SimulationExecutor::canAccept() const { return m_impl->canAccept(); }
    bool SimulationExecutor::empty() const { return m_impl->empty(); }
    void SimulationExecutor::setRapidSpeed(const double speed) { m_impl->setRapidSpeed(speed); }
    void SimulationExecutor::setStatus(const SimulationStatus status) { m_impl->setStatus(status); }
    void SimulationExecutor::setError(std::string error) { m_impl->setError(std::move(error)); }
    void SimulationExecutor::consume(SimulatedCommand command) { m_impl->consume(std::move(command)); }
    void SimulationExecutor::advance(const double seconds) { m_impl->advance(seconds); }
    void SimulationExecutor::completeQueued() { m_impl->completeQueued(); }
    SimulationSnapshot SimulationExecutor::snapshot() const { return m_impl->snapshot(); }
    SimulationSnapshot SimulationExecutor::lightweightSnapshot() const { return m_impl->lightweightSnapshot(); }
    std::optional<ProbeResult> SimulationExecutor::takeProbeResult() { return m_impl->takeProbeResult(); }
}
