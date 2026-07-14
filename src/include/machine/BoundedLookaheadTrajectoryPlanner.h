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
#include <vector>

#include "machine/ExactStopTrajectoryPlanner.h"
#include "evaluator/InterpreterSession.h"

namespace ngc {
    enum class ExecutablePathMode { ExactStop, Continuous };

    struct TrajectoryPlanningMetadata {
        ExecutablePathMode pathMode = ExecutablePathMode::ExactStop;
        std::optional<double> pathTolerance;
    };

    struct TrajectoryCommandPresentation {
        ToolGeometry tool{};
        position_t activeToolOffset{};
        std::optional<WorkCoordinateSystem> workCoordinateSystem;
        std::vector<std::string> modalGCodes;
        std::vector<BlockExecution> activeBlocks;
    };

    struct TrajectoryPlannerInput {
        MachineCommand command;
        TrajectoryPlanningMetadata metadata;
        TrajectoryCommandPresentation presentation;
    };

    struct PlannedExecution {
        MachineCommand command;
        TrajectoryPlanningMetadata metadata;
        ExecutionItem item;
        std::vector<TrajectoryPlannerInput> inputs;
        std::vector<SpanId> activationSpans;

        template<typename Item>
        PlannedExecution(MachineCommand commandValue, TrajectoryPlanningMetadata metadataValue,
                         Item &&itemValue, std::vector<TrajectoryPlannerInput> inputValues,
                         std::vector<SpanId> activationSpanValues = {})
            : command(std::move(commandValue)), metadata(std::move(metadataValue)),
              item(std::forward<Item>(itemValue)), inputs(std::move(inputValues)),
              activationSpans(std::move(activationSpanValues)) { }
    };

    struct TrajectoryPlanningDiagnostics {
        std::uint64_t commandsPlanned = 0;
        std::uint64_t continuousModeInputs = 0;
        std::uint64_t exactStopFallbacks = 0;
        std::uint64_t blendedWindows = 0;
        std::uint64_t blendedCommands = 0;
        std::size_t maximumWindowCommands = 0;
        std::uint32_t maximumNormalSpans = 0;
        std::uint32_t maximumStopSpans = 0;
        double plannedDuration = 0.0;
        double lastPlanningSeconds = 0.0;
        double maximumPlanningSeconds = 0.0;
    };

    // NRT-only bounded command window. Compatible G64 lines and arcs are planned
    // as exact retained primitives joined by local degree-three B-spline blends.
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
                if constexpr(std::same_as<T,MoveLine>)
                    return command.speed()>0.0&&!command.machineCoordinates();
                else if constexpr(std::same_as<T,MoveArc>) return command.speed()>0.0;
                else return false;
            }, input.command);
        }

        static bool samePosition(const position_t &left,const position_t &right) {
            return (left-right).length()<=1e-12;
        }

        static std::optional<position_t> motionStart(const MachineCommand &command) {
            return std::visit([](const auto &value) -> std::optional<position_t> {
                using T=std::decay_t<decltype(value)>;
                if constexpr(std::same_as<T,MoveLine>||std::same_as<T,MoveArc>) return value.from();
                else return std::nullopt;
            },command);
        }

        static std::optional<position_t> motionEnd(const MachineCommand &command) {
            return std::visit([](const auto &value) -> std::optional<position_t> {
                using T=std::decay_t<decltype(value)>;
                if constexpr(std::same_as<T,MoveLine>||std::same_as<T,MoveArc>) return value.to();
                else return std::nullopt;
            },command);
        }

        static bool sameProtectedPresentation(const TrajectoryCommandPresentation &left,
                                               const TrajectoryCommandPresentation &right) {
            if(!samePosition(left.activeToolOffset,right.activeToolOffset)) return false;
            if(left.tool.number!=right.tool.number||left.tool.diameter!=right.tool.diameter
               ||!samePosition(left.tool.offset,right.tool.offset)) return false;
            if(left.workCoordinateSystem.has_value()!=right.workCoordinateSystem.has_value()) return false;
            if(left.workCoordinateSystem&&
               (left.workCoordinateSystem->name!=right.workCoordinateSystem->name
                ||!samePosition(left.workCoordinateSystem->offset,right.workCoordinateSystem->offset))) return false;
            const auto protectedModes=[](const std::vector<std::string> &modes) {
                std::vector<std::string_view> result;
                for(const auto &mode:modes) {
                    if(mode=="G0"||mode=="G1"||mode=="G2"||mode=="G3"
                       ||mode=="G38.2"||mode=="G38.3"||mode=="G38.4"||mode=="G38.5") continue;
                    result.push_back(mode);
                }
                return result;
            };
            if(protectedModes(left.modalGCodes)!=protectedModes(right.modalGCodes)) return false;
            return true;
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

        std::expected<void,std::string> verifyContinuousNormalMotion(const PlanChunk &chunk) const {
            if(chunk.normalMotion.size==0)
                return std::unexpected("continuous plan has no normal motion");
            const auto discontinuity=[](const position_t &left,const position_t &right,const double tolerance) {
                return (left-right).length()>tolerance;
            };
            const auto &limits=m_exactStop.limits();
            std::optional<MotionState> previous;
            for(const auto &span:chunk.normalMotion) {
                if(!std::isfinite(span.duration)||span.duration<=0.0)
                    return std::unexpected("continuous plan has an invalid span duration");
                const MotionState start {
                    span.d,
                    scalePosition(span.c,span.inverseDuration),
                    scalePosition(span.b,2.0*span.inverseDurationSquared),
                };
                if(previous&&(discontinuity(previous->position,start.position,1e-8)
                    ||discontinuity(previous->velocity,start.velocity,1e-7)
                    ||discontinuity(previous->acceleration,start.acceleration,1e-7)))
                    return std::unexpected("continuous plan is not C2 at a polynomial boundary");
                for(const auto component:AXIS_COMPONENTS) {
                    if(exceeds(maximumAxisVelocity(span,component),limits.axisVelocity.*component)
                       ||exceeds(maximumAxisAcceleration(span,component),limits.axisAcceleration.*component)
                       ||exceeds(maximumAxisJerk(span,component),limits.axisJerk.*component))
                        return std::unexpected("continuous plan exceeds a configured axis limit");
                }
                const auto acceleration0=start.acceleration.length();
                const auto acceleration1=span.end.acceleration.length();
                const auto jerk=scalePosition(span.a,6.0*span.inverseDurationCubed).length();
                if(exceeds(std::max(acceleration0,acceleration1),limits.pathAcceleration)
                   ||exceeds(jerk,limits.pathJerk))
                    return std::unexpected("continuous plan exceeds a configured path limit");
                previous=span.end;
            }
            const MotionState first {
                chunk.normalMotion[0].d,
                scalePosition(chunk.normalMotion[0].c,chunk.normalMotion[0].inverseDuration),
                scalePosition(chunk.normalMotion[0].b,
                              2.0*chunk.normalMotion[0].inverseDurationSquared),
            };
            if(first.velocity.length()>1e-8||first.acceleration.length()>1e-8
               ||chunk.normalMotion[chunk.normalMotion.size-1].end.velocity.length()>1e-8
               ||chunk.normalMotion[chunk.normalMotion.size-1].end.acceleration.length()>1e-8)
                return std::unexpected("continuous plan is not rest-to-rest at its protected boundaries");
            return {};
        }

        void record(const ExecutionItem &item, const double planningSeconds,
                    const std::size_t commandCount=1) {
            m_diagnostics.commandsPlanned+=commandCount;
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
        static bool eligibleForLookahead(const TrajectoryPlannerInput &input) {
            return continuousMotion(input);
        }

        bool canAppend(const TrajectoryPlannerInput &input) const {
            if(full()) return false;
            if(m_window.empty()) return true;
            const auto previousEnd=motionEnd(m_window.back().command);
            const auto nextStart=motionStart(input.command);
            return continuousMotion(input) && continuousMotion(m_window.front())
                && input.metadata.pathTolerance == m_window.front().metadata.pathTolerance
                &&sameProtectedPresentation(input.presentation,m_window.back().presentation)
                &&previousEnd&&nextStart&&samePosition(*previousEnd,*nextStart);
        }

        bool enqueue(TrajectoryPlannerInput input) {
            if(full()) return false;
            if(continuousMotion(input)) {
                ++m_diagnostics.continuousModeInputs;
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
            if(continuousMotion(input)) ++m_diagnostics.exactStopFallbacks;
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
            const auto activation=std::visit([](const auto &value) -> SpanId {
                using T=std::decay_t<decltype(value)>;
                if constexpr(std::same_as<T,PlanChunk>) return value.normalMotion[0].id;
                else return 0;
            },*item);
            return std::make_unique<PlannedExecution>(
                input.command, input.metadata, std::move(*item),
                std::vector<TrajectoryPlannerInput>{std::move(input)},
                std::vector<SpanId>{activation});
        }

        std::expected<std::unique_ptr<PlannedExecution>, std::string> planWindow() {
            if(m_window.size()<2||!std::ranges::all_of(m_window,continuousMotion)) return planOne();
            const auto started=std::chrono::steady_clock::now();
            std::vector<MachineCommand> commands;
            commands.reserve(m_window.size());
            for(const auto &input:m_window) commands.push_back(input.command);
            constexpr double DEFAULT_BLEND_SCALE=0.001;
            const auto blendScale=std::max(
                m_window.front().metadata.pathTolerance.value_or(DEFAULT_BLEND_SCALE),1e-9);
            auto continuous=m_exactStop.compileContinuous(commands,blendScale);
            if(!continuous) return planOne();
            if(auto verified=verifyContinuousNormalMotion((*continuous)->chunk);!verified)
                return std::unexpected(verified.error());
            if(auto verified=verifyStopBranch((*continuous)->chunk);!verified)
                return std::unexpected(verified.error());

            std::vector<TrajectoryPlannerInput> inputs;
            inputs.reserve(m_window.size());
            while(!m_window.empty()) {
                inputs.push_back(std::move(m_window.front()));
                m_window.pop_front();
            }
            const auto planningSeconds=std::chrono::duration<double>(
                std::chrono::steady_clock::now()-started).count();
            ++m_diagnostics.blendedWindows;
            m_diagnostics.blendedCommands+=inputs.size();
            auto representativeCommand=inputs.front().command;
            auto representativeMetadata=inputs.front().metadata;
            auto planned=std::make_unique<PlannedExecution>(
                std::move(representativeCommand),std::move(representativeMetadata),
                std::move((*continuous)->chunk),std::move(inputs),
                std::move((*continuous)->activationSpans));
            record(planned->item,planningSeconds,planned->inputs.size());
            return planned;
        }
    };
}
