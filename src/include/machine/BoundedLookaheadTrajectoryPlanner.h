#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <expected>
#include <format>
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
        std::vector<ExecutionItem> items;
        std::vector<TrajectoryPlannerInput> inputs;
        std::vector<SpanId> activationSpans;
        std::vector<std::size_t> activationItems;

        template<typename Item>
        PlannedExecution(MachineCommand commandValue, TrajectoryPlanningMetadata metadataValue,
                         Item &&itemValue, std::vector<TrajectoryPlannerInput> inputValues,
                         std::vector<SpanId> activationSpanValues = {})
            : command(std::move(commandValue)), metadata(std::move(metadataValue)),
              items{ExecutionItem{std::forward<Item>(itemValue)}}, inputs(std::move(inputValues)),
              activationSpans(std::move(activationSpanValues)),activationItems(inputs.size(),0) { }

        PlannedExecution(MachineCommand commandValue,TrajectoryPlanningMetadata metadataValue,
                         std::vector<ExecutionItem> itemValues,
                         std::vector<TrajectoryPlannerInput> inputValues,
                         std::vector<SpanId> activationSpanValues,
                         std::vector<std::size_t> activationItemValues)
            : command(std::move(commandValue)),metadata(std::move(metadataValue)),
              items(std::move(itemValues)),inputs(std::move(inputValues)),
              activationSpans(std::move(activationSpanValues)),
              activationItems(std::move(activationItemValues)) { }

        const ExecutionItem &primaryItem() const { return items.front(); }
    };

    struct TrajectoryPlanningDiagnostics {
        std::uint64_t commandsPlanned = 0;
        std::uint64_t planChunks = 0;
        std::uint64_t continuousModeInputs = 0;
        std::uint64_t continuousExactStops = 0;
        std::uint64_t blendedWindows = 0;
        std::uint64_t blendedCommands = 0;
        std::size_t maximumWindowCommands = 0;
        std::uint32_t maximumNormalSpans = 0;
        std::uint32_t maximumStopSpans = 0;
        double plannedDuration = 0.0;
        double lastPlanningSeconds = 0.0;
        double maximumPlanningSeconds = 0.0;
    };

    // NRT-only compatible command horizon. RT capacity is imposed later while
    // the verified polynomial stream is packetized into PlanChunk values; it is
    // deliberately not expressed as an arbitrary G-code command count.
    class BoundedLookaheadTrajectoryPlanner {
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

        static std::string formatPosition(const position_t &value) {
            return std::format("[X={} Y={} Z={} A={} B={} C={}]",
                value.x,value.y,value.z,value.a,value.b,value.c);
        }

        static std::string inputLocation(const TrajectoryPlannerInput &input) {
            if(input.presentation.activeBlocks.empty()) return "<no active source block>";
            const auto &block=input.presentation.activeBlocks.back();
            return std::format("{}:{} block {} '{}'",block.source,block.line,block.id,block.text);
        }

        std::string continuousWindowContext() const {
            if(m_window.empty()) return "empty G64 window";
            const auto start=motionStart(m_window.front().command);
            const auto end=motionEnd(m_window.back().command);
            return std::format(
                "G64 window commands={} P={} first={} last={} start={} end={}",
                m_window.size(),m_window.front().metadata.pathTolerance
                    ? std::format("{}",*m_window.front().metadata.pathTolerance) : std::string("<default>"),
                inputLocation(m_window.front()),inputLocation(m_window.back()),
                start?formatPosition(*start):std::string("<none>"),
                end?formatPosition(*end):std::string("<none>"));
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
            const auto &normalEnd=chunk.normalMotion[chunk.normalMotion.size-1].end;
            if(discontinuity(normalEnd.position,chunk.branchState.position)
               ||discontinuity(normalEnd.velocity,chunk.branchState.velocity)
               ||discontinuity(normalEnd.acceleration,chunk.branchState.acceleration))
                return std::unexpected("planned chunk branch state does not match normal-motion terminal state");
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
            const auto &limits=m_exactStop.limits();
            std::optional<MotionState> previous;
            SpanId previousSpan=0;
            std::size_t spanIndex=0;
            for(const auto &span:chunk.normalMotion) {
                if(!std::isfinite(span.duration)||span.duration<=0.0)
                    return std::unexpected("continuous plan has an invalid span duration");
                const MotionState start {
                    span.d,
                    scalePosition(span.c,span.inverseDuration),
                    scalePosition(span.b,2.0*span.inverseDurationSquared),
                };
                if(previous) {
                    const auto positionJump=(previous->position-start.position).length();
                    const auto velocityJump=(previous->velocity-start.velocity).length();
                    const auto accelerationJump=(previous->acceleration-start.acceleration).length();
                    if(positionJump>1e-8||velocityJump>1e-7||accelerationJump>1e-7) {
                        const auto boundaryType=spanIndex%3==0
                            ? "between emitted three-span chains" : "inside one emitted three-span chain";
                        return std::unexpected(std::format(
                            "continuous plan C2 verification failed {} at normal span index {} "
                            "(span {} -> {}): position jump={} tolerance=1e-8; velocity jump={} "
                            "tolerance=1e-7; acceleration jump={} tolerance=1e-7; previous end "
                            "position={} velocity={} acceleration={}; current start position={} "
                            "velocity={} acceleration={}",boundaryType,spanIndex,previousSpan,span.id,
                            positionJump,velocityJump,accelerationJump,
                            formatPosition(previous->position),formatPosition(previous->velocity),
                            formatPosition(previous->acceleration),formatPosition(start.position),
                            formatPosition(start.velocity),formatPosition(start.acceleration)));
                    }
                }
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
                previousSpan=span.id;
                ++spanIndex;
            }
            return {};
        }

        std::expected<void,std::string> verifyContinuousPlan(
                const std::vector<PlanChunk> &chunks) const {
            if(chunks.empty()) return std::unexpected("continuous plan produced no RT chunks");
            std::optional<MotionState> previous;
            BranchSequence predecessor=chunks.front().predecessorBranch;
            for(std::size_t chunkIndex=0;chunkIndex<chunks.size();++chunkIndex) {
                const auto &chunk=chunks[chunkIndex];
                if(chunk.predecessorBranch!=predecessor)
                    return std::unexpected(std::format(
                        "continuous chunk {} predecessor branch {} does not match expected {}",
                        chunkIndex,chunk.predecessorBranch,predecessor));
                if(auto verified=verifyContinuousNormalMotion(chunk);!verified)
                    return std::unexpected(std::format("chunk {}: {}",chunkIndex,verified.error()));
                const MotionState start {
                    chunk.normalMotion[0].d,
                    scalePosition(chunk.normalMotion[0].c,chunk.normalMotion[0].inverseDuration),
                    scalePosition(chunk.normalMotion[0].b,
                        2.0*chunk.normalMotion[0].inverseDurationSquared),
                };
                if(previous) {
                    const auto positionJump=(previous->position-start.position).length();
                    const auto velocityJump=(previous->velocity-start.velocity).length();
                    const auto accelerationJump=(previous->acceleration-start.acceleration).length();
                    if(positionJump>1e-8||velocityJump>1e-7||accelerationJump>1e-7)
                        return std::unexpected(std::format(
                            "continuous packet boundary {} -> {} is not C2: position jump={} "
                            "velocity jump={} acceleration jump={}",chunkIndex-1,chunkIndex,
                            positionJump,velocityJump,accelerationJump));
                } else if(start.velocity.length()>1e-8||start.acceleration.length()>1e-8) {
                    return std::unexpected(std::format(
                        "continuous plan does not start at rest: velocity={} acceleration={}",
                        formatPosition(start.velocity),formatPosition(start.acceleration)));
                }
                previous=chunk.normalMotion[chunk.normalMotion.size-1].end;
                predecessor=chunk.branch;
            }
            if(previous->velocity.length()>1e-8||previous->acceleration.length()>1e-8)
                return std::unexpected(std::format(
                    "continuous plan does not end at rest: velocity={} acceleration={}",
                    formatPosition(previous->velocity),formatPosition(previous->acceleration)));
            return {};
        }

        void record(const ExecutionItem &item, const double planningSeconds,
                    const std::size_t commandCount=1) {
            m_diagnostics.commandsPlanned+=commandCount;
            m_diagnostics.lastPlanningSeconds = planningSeconds;
            m_diagnostics.maximumPlanningSeconds = std::max(
                m_diagnostics.maximumPlanningSeconds, planningSeconds);
            if(const auto *chunk = std::get_if<PlanChunk>(&item)) {
                ++m_diagnostics.planChunks;
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

        void rebase(const EpochId epoch, const position_t &position) {
            m_exactStop.reset(epoch,position);
        }

        void clearDiagnostics() { m_diagnostics = {}; }
        void setLimits(const TrajectoryLimits &limits) { m_exactStop.setLimits(limits); }
        const TrajectoryLimits &limits() const { return m_exactStop.limits(); }
        const TrajectoryPlanningDiagnostics &diagnostics() const { return m_diagnostics; }
        std::size_t windowSize() const { return m_window.size(); }
        static bool eligibleForLookahead(const TrajectoryPlannerInput &input) {
            return continuousMotion(input);
        }

        bool canAppend(const TrajectoryPlannerInput &input) const {
            if(m_window.empty()) return true;
            const auto previousEnd=motionEnd(m_window.back().command);
            const auto nextStart=motionStart(input.command);
            return continuousMotion(input) && continuousMotion(m_window.front())
                && input.metadata.pathTolerance == m_window.front().metadata.pathTolerance
                &&sameProtectedPresentation(input.presentation,m_window.back().presentation)
                &&previousEnd&&nextStart&&samePosition(*previousEnd,*nextStart);
        }

        bool enqueue(TrajectoryPlannerInput input) {
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
            if(continuousMotion(input)) ++m_diagnostics.continuousExactStops;
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
            const auto context=continuousWindowContext();
            auto continuous=m_exactStop.compileContinuous(commands,blendScale);
            if(!continuous) return std::unexpected(std::format(
                "fatal continuous-motion compilation failure: {}; cause: {}",context,continuous.error()));
            if(auto verified=verifyContinuousPlan((*continuous)->chunks);!verified)
                return std::unexpected(std::format(
                    "fatal continuous-motion verification failure: {}; packets={}; cause: {}",
                    context,(*continuous)->chunks.size(),verified.error()));
            for(std::size_t chunk=0;chunk<(*continuous)->chunks.size();++chunk)
                if(auto verified=verifyStopBranch((*continuous)->chunks[chunk]);!verified)
                    return std::unexpected(std::format(
                        "fatal continuous-motion stop-branch verification failure: {}; packet={} chunk={} "
                        "normal spans={} stop spans={}; cause: {}",context,chunk,
                        (*continuous)->chunks[chunk].id,
                        (*continuous)->chunks[chunk].normalMotion.size,
                        (*continuous)->chunks[chunk].stopTail.size,verified.error()));

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
            std::vector<ExecutionItem> items;
            items.reserve((*continuous)->chunks.size());
            for(auto &chunk:(*continuous)->chunks) items.emplace_back(std::move(chunk));
            std::vector<std::size_t> activationItems((*continuous)->activationSpans.size(),0);
            for(std::size_t input=0;input<(*continuous)->activationSpans.size();++input) {
                const auto activation=(*continuous)->activationSpans[input];
                const auto owner=std::ranges::find_if(items,[&](const auto &item) {
                    const auto &chunk=std::get<PlanChunk>(item);
                    return std::ranges::any_of(chunk.normalMotion,[&](const auto &span) {
                        return span.id==activation;
                    });
                });
                if(owner==items.end()) return std::unexpected(std::format(
                    "fatal continuous-motion metadata failure: {}; activation span {} for input {} "
                    "does not belong to any packet",context,activation,input));
                activationItems[input]=static_cast<std::size_t>(std::distance(items.begin(),owner));
            }
            auto planned=std::make_unique<PlannedExecution>(
                std::move(representativeCommand),std::move(representativeMetadata),
                std::move(items),std::move(inputs),std::move((*continuous)->activationSpans),
                std::move(activationItems));
            for(std::size_t item=0;item<planned->items.size();++item)
                record(planned->items[item],item==0?planningSeconds:0.0,
                    item==0?planned->inputs.size():0);
            m_diagnostics.lastPlanningSeconds=planningSeconds;
            m_diagnostics.maximumPlanningSeconds=std::max(
                m_diagnostics.maximumPlanningSeconds,planningSeconds);
            return planned;
        }
    };
}
