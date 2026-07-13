#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <expected>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>

#include "machine/ExactStopTrajectoryPlanner.h"

namespace ngc {
    enum class ExecutablePathMode { ExactStop, Continuous };

    struct TrajectoryPlanningMetadata {
        ExecutablePathMode pathMode = ExecutablePathMode::ExactStop;
        std::optional<double> pathTolerance;
    };

    struct TrajectoryPlannerInput {
        MachineCommand command;
        TrajectoryPlanningMetadata metadata;
    };

    struct PlannedExecution {
        MachineCommand command;
        TrajectoryPlanningMetadata metadata;
        ExecutionItem item;
    };

    struct TrajectoryPlanningDiagnostics {
        std::uint64_t commandsPlanned = 0;
        std::uint64_t continuousModeInputs = 0;
        std::uint64_t exactStopFallbacks = 0;
        std::size_t maximumWindowCommands = 0;
        std::uint32_t maximumNormalSpans = 0;
        std::uint32_t maximumStopSpans = 0;
        double plannedDuration = 0.0;
        double lastPlanningSeconds = 0.0;
        double maximumPlanningSeconds = 0.0;
    };

    // NRT-only bounded command window. The first implementation deliberately
    // commits every entry through the exact-stop planner. Future executable G64
    // may retain eligible entries in this window, but every emitted PlanChunk
    // must continue to pass the stop-branch gate below before publication.
    class BoundedLookaheadTrajectoryPlanner {
    public:
        static constexpr std::size_t MAX_LOOKAHEAD_COMMANDS = 32;

    private:
        ExactStopTrajectoryPlanner m_exactStop;
        std::deque<TrajectoryPlannerInput> m_window;
        TrajectoryPlanningDiagnostics m_diagnostics;

        static constexpr std::array AXIS_COMPONENTS {
            &position_t::x, &position_t::y, &position_t::z,
            &position_t::a, &position_t::b, &position_t::c,
        };

        static position_t scalePosition(const position_t &value, const double amount) {
            return { value.x*amount, value.y*amount, value.z*amount,
                     value.a*amount, value.b*amount, value.c*amount };
        }

        static bool continuousMotion(const TrajectoryPlannerInput &input) {
            if(input.metadata.pathMode != ExecutablePathMode::Continuous) return false;
            return std::visit([](const auto &command) {
                using T = std::decay_t<decltype(command)>;
                return std::same_as<T, MoveLine> || std::same_as<T, MoveArc>;
            }, input.command);
        }

        static double maximumAxisVelocity(const AxisPolynomialSpan &span,
                                          const double position_t::*component) {
            const auto at = [&](const double u) {
                return std::abs((3.0*span.a.*component*u*u + 2.0*span.b.*component*u
                    + span.c.*component) * span.inverseDuration);
            };
            auto result = std::max(at(0.0), at(1.0));
            if(std::abs(span.a.*component) > 1e-15) {
                const auto stationary = -(span.b.*component) / (3.0*(span.a.*component));
                if(stationary > 0.0 && stationary < 1.0) result = std::max(result, at(stationary));
            }
            return result;
        }

        static double maximumAxisAcceleration(const AxisPolynomialSpan &span,
                                              const double position_t::*component) {
            const auto at = [&](const double u) {
                return std::abs((6.0*span.a.*component*u + 2.0*span.b.*component)
                    * span.inverseDurationSquared);
            };
            return std::max(at(0.0), at(1.0));
        }

        static double maximumAxisJerk(const AxisPolynomialSpan &span,
                                      const double position_t::*component) {
            return std::abs(6.0*span.a.*component * span.inverseDurationCubed);
        }

        static bool exceeds(const double value, const double limit) {
            return value > limit*(1.0 + 1e-9) + 1e-9;
        }

        std::expected<void, std::string> verifyStopBranch(const PlanChunk &chunk) const {
            if(chunk.normalMotion.size == 0) return std::unexpected("planned chunk has no normal motion");
            if(chunk.stopTail.size == 0) return std::unexpected("planned chunk has no bounded stop branch");
            const auto discontinuity = [](const position_t &a, const position_t &b) {
                return (a-b).length() > 1e-8;
            };
            auto previous = chunk.branchState;
            const auto &limits = m_exactStop.limits();
            for(const auto &span : chunk.stopTail) {
                if(!std::isfinite(span.duration) || span.duration <= 0.0)
                    return std::unexpected("planned stop branch contains an invalid span duration");
                const MotionState start {
                    span.d,
                    scalePosition(span.c, span.inverseDuration),
                    scalePosition(span.b, 2.0*span.inverseDurationSquared),
                };
                if(discontinuity(start.position, previous.position)
                   || discontinuity(start.velocity, previous.velocity)
                   || discontinuity(start.acceleration, previous.acceleration))
                    return std::unexpected("planned stop branch is discontinuous at a span boundary");
                for(const auto component : AXIS_COMPONENTS) {
                    if(exceeds(maximumAxisVelocity(span, component), limits.axisVelocity.*component)
                       || exceeds(maximumAxisAcceleration(span, component), limits.axisAcceleration.*component)
                       || exceeds(maximumAxisJerk(span, component), limits.axisJerk.*component))
                        return std::unexpected("planned stop branch exceeds a configured axis limit");
                }
                const auto acceleration0 = start.acceleration.length();
                const auto acceleration1 = span.end.acceleration.length();
                const auto jerk = scalePosition(span.a, 6.0*span.inverseDurationCubed).length();
                if(exceeds(std::max(acceleration0, acceleration1), limits.pathAcceleration)
                   || exceeds(jerk, limits.pathJerk))
                    return std::unexpected("planned stop branch exceeds a configured path limit");
                previous = span.end;
            }
            if(discontinuity(previous.position, chunk.stopState.position)
               || discontinuity(previous.velocity, chunk.stopState.velocity)
               || discontinuity(previous.acceleration, chunk.stopState.acceleration))
                return std::unexpected("planned stop branch terminal state does not match its declared stop state");
            if(chunk.stopState.velocity.length() > 1e-8 || chunk.stopState.acceleration.length() > 1e-8)
                return std::unexpected("planned stop branch does not terminate at rest");
            return {};
        }

        void record(const ExecutionItem &item, const double planningSeconds) {
            ++m_diagnostics.commandsPlanned;
            m_diagnostics.lastPlanningSeconds = planningSeconds;
            m_diagnostics.maximumPlanningSeconds = std::max(
                m_diagnostics.maximumPlanningSeconds, planningSeconds);
            if(const auto *chunk = std::get_if<PlanChunk>(&item)) {
                m_diagnostics.maximumNormalSpans = std::max(
                    m_diagnostics.maximumNormalSpans, chunk->normalMotion.size);
                m_diagnostics.maximumStopSpans = std::max(
                    m_diagnostics.maximumStopSpans, chunk->stopTail.size);
                for(const auto &span : chunk->normalMotion)
                    m_diagnostics.plannedDuration += span.duration;
            }
        }

    public:
        explicit BoundedLookaheadTrajectoryPlanner(const TrajectoryLimits limits = {})
            : m_exactStop(limits) { }

        void reset(const EpochId epoch, const position_t &position = {}) {
            m_window.clear();
            m_exactStop.reset(epoch, position);
        }

        void clearDiagnostics() { m_diagnostics = {}; }
        void setLimits(const TrajectoryLimits &limits) { m_exactStop.setLimits(limits); }
        const TrajectoryLimits &limits() const { return m_exactStop.limits(); }
        const TrajectoryPlanningDiagnostics &diagnostics() const { return m_diagnostics; }
        std::size_t windowSize() const { return m_window.size(); }
        bool full() const { return m_window.size() == MAX_LOOKAHEAD_COMMANDS; }

        bool enqueue(TrajectoryPlannerInput input) {
            if(full()) return false;
            if(continuousMotion(input)) {
                ++m_diagnostics.continuousModeInputs;
                ++m_diagnostics.exactStopFallbacks;
            }
            m_window.push_back(std::move(input));
            m_diagnostics.maximumWindowCommands = std::max(
                m_diagnostics.maximumWindowCommands, m_window.size());
            return true;
        }

        std::expected<std::unique_ptr<PlannedExecution>, std::string> planOne() {
            if(m_window.empty()) return std::unique_ptr<PlannedExecution>{};
            const auto started = std::chrono::steady_clock::now();
            auto input = std::move(m_window.front());
            m_window.pop_front();
            auto item = std::visit([&](const auto &command) -> std::expected<ExecutionItem, std::string> {
                using T = std::decay_t<decltype(command)>;
                if constexpr(std::same_as<T, ProbeMove>) {
                    auto move = m_exactStop.compileTriggeredMove(command);
                    if(!move) return std::unexpected(move.error());
                    return ExecutionItem { *move };
                } else {
                    auto chunk = m_exactStop.compile(input.command);
                    if(!chunk) return std::unexpected(chunk.error());
                    if(auto verified = verifyStopBranch(*chunk); !verified)
                        return std::unexpected(verified.error());
                    return ExecutionItem { *chunk };
                }
            }, input.command);
            if(!item) return std::unexpected(item.error());
            const auto planningSeconds = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - started).count();
            record(*item, planningSeconds);
            return std::make_unique<PlannedExecution>(PlannedExecution {
                std::move(input.command), std::move(input.metadata), std::move(*item) });
        }
    };
}
