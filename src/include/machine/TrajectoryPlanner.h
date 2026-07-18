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
#include <tuple>
#include <utility>
#include <vector>

#include "machine/TrajectoryCompiler.h"
#include "machine/ArcInterpolation.h"
#include "machine/PreparedGeometry.h"
#include "evaluator/InterpreterSession.h"

namespace ngc {
    // Compatibility input retained for the current command-window planner.
    // PreparedCommandRecord is the transport representation; conversion into
    // this value is deliberately owned by the planning side.
    struct TrajectoryPlannerInput {
        MachineCommand command;
        TrajectoryPlanningMetadata metadata;
        TrajectoryCommandPresentation presentation;
        bool presentationActivation = true;
        double continuousScaleOverride = 0.0;
        std::optional<PreparedPathPiece> preparedPiece;

        TrajectoryPlannerInput(MachineCommand commandValue,
                               TrajectoryPlanningMetadata metadataValue = {},
                               TrajectoryCommandPresentation presentationValue = {},
                               const bool presentationActivationValue = true,
                               const double continuousScaleOverrideValue = 0.0,
                               std::optional<PreparedPathPiece> preparedPieceValue = std::nullopt)
            : command(std::move(commandValue)), metadata(std::move(metadataValue)),
              presentation(std::move(presentationValue)),
              presentationActivation(presentationActivationValue),
              continuousScaleOverride(continuousScaleOverrideValue),
              preparedPiece(std::move(preparedPieceValue)) { }
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
        double totalPlanningSeconds = 0.0;
        std::uint64_t continuousHorizons = 0;
        double firstContinuousHorizonSeconds = 0.0;
        double lastContinuousHorizonSeconds = 0.0;
        double maximumContinuousHorizonSeconds = 0.0;
        double totalContinuousHorizonSeconds = 0.0;
        std::uint64_t rollingBoundaryCandidates = 0;
        std::uint64_t rollingSuffixProbeFailures = 0;
        std::uint64_t rollingPrefixProbeFailures = 0;
        double rollingSearchSeconds = 0.0;
        // All attempted scalar Ruckig solves, including failed rolling probes.
        TimeLawDiagnostics timeLaw;
        TimeLawDiagnostics publishedTimeLaw;
        TimeLawDiagnostics rollingPrefixProbeTimeLaw;
        TimeLawDiagnostics rollingSuffixProbeTimeLaw;
        SplineInverseDiagnostics publishedSplineInverse;
        simulation_detail::ArcInverseDiagnostics publishedArcInverse;
    };

    // NRT-only compatible command horizon. RT capacity is imposed later while
    // the verified polynomial stream is packetized into PlanChunk values; it is
    // deliberately not expressed as an arbitrary G-code command count.
    class TrajectoryPlanner {
    private:
        TrajectoryCompiler m_compiler;
        std::deque<TrajectoryPlannerInput> m_window;
        std::optional<PreparedContinuousGeometry> m_preparedWindow;
        TrajectoryPlanningDiagnostics m_diagnostics;
        MotionState m_continuousBoundary{};
        std::string m_lastRollingCandidateError;
        std::size_t m_nextRollingAttemptWindowSize = 2;
        std::optional<double> m_lastRollingVelocityFraction;
        std::function<void(const ContinuousTrajectoryPlan &,
            std::span<const TrajectoryPlannerInput>)> m_continuousDiagnosticCallback;
        static constexpr unsigned ROLLING_VELOCITY_SEARCH_ITERATIONS = 6;

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

        static void accumulateSplineInverse(SplineInverseDiagnostics &total,
                                             const SplineInverseDiagnostics &value) {
            total.constructionIntegralEvaluations+=value.constructionIntegralEvaluations;
            total.queries+=value.queries;
            total.endpointQueries+=value.endpointQueries;
            total.exactCacheHits+=value.exactCacheHits;
            total.inverseIntegralEvaluations+=value.inverseIntegralEvaluations;
            total.newtonIterations+=value.newtonIterations;
            total.seedConvergences+=value.seedConvergences;
            total.safeguardedBisections+=value.safeguardedBisections;
            total.iterationLimitHits+=value.iterationLimitHits;
            total.maximumNewtonIterations=std::max(
                total.maximumNewtonIterations,value.maximumNewtonIterations);
        }

        static void accumulateArcInverse(simulation_detail::ArcInverseDiagnostics &total,
                                         const simulation_detail::ArcInverseDiagnostics &value) {
            total.constructionIntegralEvaluations+=value.constructionIntegralEvaluations;
            total.queries+=value.queries;
            total.endpointQueries+=value.endpointQueries;
            total.exactCacheHits+=value.exactCacheHits;
            total.inverseIntegralEvaluations+=value.inverseIntegralEvaluations;
            total.newtonIterations+=value.newtonIterations;
            total.seedConvergences+=value.seedConvergences;
            total.safeguardedBisections+=value.safeguardedBisections;
            total.iterationLimitHits+=value.iterationLimitHits;
            total.maximumNewtonIterations=std::max(
                total.maximumNewtonIterations,value.maximumNewtonIterations);
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

        static double positionDot(const position_t &left,const position_t &right) {
            return left.x*right.x+left.y*right.y+left.z*right.z
                +left.a*right.a+left.b*right.b+left.c*right.c;
        }

        struct RollingSplit {
            MachineCommand prefix;
            MachineCommand suffix;
            double prefixLength = 0.0;
            double suffixLength = 0.0;
            position_t position{};
            position_t tangent{};
            position_t curvature{};
            double velocityLimit = 0.0;
        };

        static std::optional<double> continuousMotionLength(const MachineCommand &command) {
            return std::visit([](const auto &value) -> std::optional<double> {
                using T=std::decay_t<decltype(value)>;
                if constexpr(std::same_as<T,MoveLine>) return (value.to()-value.from()).length();
                else if constexpr(std::same_as<T,MoveArc>) {
                    simulation_detail::ArcReference reference(value);
                    if(reference.valid()) return reference.length();
                    return std::nullopt;
                } else return std::nullopt;
            },command);
        }

        static double continuousFeed(const MachineCommand &command) {
            return std::visit([](const auto &value) {
                using T=std::decay_t<decltype(value)>;
                if constexpr(std::same_as<T,MoveLine>||std::same_as<T,MoveArc>)
                    return value.speed()/60.0;
                else return 0.0;
            },command);
        }

        std::optional<RollingSplit> rollingSplit(const MachineCommand &command,
                                                  const double programmedScale) const {
            const auto length=continuousMotionLength(command);
            // A rolling anchor is valid only inside the exact retained middle
            // of a line that is genuinely longer than the complete 6P blend
            // allocation. Using the entity's automatic min(P, length / 6)
            // scale makes every short line satisfy this test by equality and
            // lets a rolling boundary cut through a spline cluster.
            if(!length||*length<=6.0*programmedScale*(1.0+1e-10))
                return std::nullopt;
            const auto splitDistance=*length/2.0;
            return std::visit([&](const auto &value) -> std::optional<RollingSplit> {
                using T=std::decay_t<decltype(value)>;
                position_t position;
                position_t tangent;
                position_t curvature{};
                MachineCommand prefix=value;
                MachineCommand suffix=value;
                if constexpr(std::same_as<T,MoveLine>) {
                    const auto delta=value.to()-value.from();
                    tangent=scalePosition(delta,1.0 / *length);
                    position=value.from()+scalePosition(tangent,splitDistance);
                    prefix=MoveLine{value.from(),position,value.speed(),value.machineCoordinates()};
                    suffix=MoveLine{position,value.to(),value.speed(),value.machineCoordinates()};
                } else if constexpr(std::same_as<T,MoveArc>) {
                    simulation_detail::ArcReference reference(value);
                    if(!reference.valid()) return std::nullopt;
                    position=reference.positionAtDistance(splitDistance);
                    tangent=reference.tangentAtDistance(splitDistance);
                    const auto step=std::clamp(*length*1e-5,1e-8,
                        std::max(*length*1e-3,1e-8));
                    const auto from=std::max(0.0,splitDistance-step);
                    const auto to=std::min(*length,splitDistance+step);
                    if(to-from<=1e-15) return std::nullopt;
                    curvature=scalePosition(reference.tangentAtDistance(to)
                        -reference.tangentAtDistance(from),1.0/(to-from));
                    curvature=curvature-scalePosition(tangent,
                        positionDot(curvature,tangent));
                    prefix=MoveArc{value.from(),position,value.center(),value.axis(),value.speed()};
                    suffix=MoveArc{position,value.to(),value.center(),value.axis(),value.speed()};
                } else return std::nullopt;

                auto velocityLimit=continuousFeed(command);
                const auto &limits=m_compiler.limits();
                for(const auto component:AXIS_COMPONENTS) {
                    const auto tangentComponent=std::abs(tangent.*component);
                    if(tangentComponent>1e-15)
                        velocityLimit=std::min(velocityLimit,
                            limits.axisVelocity.*component/tangentComponent);
                    const auto curvatureComponent=std::abs(curvature.*component);
                    if(curvatureComponent>1e-15)
                        velocityLimit=std::min(velocityLimit,std::sqrt(
                            limits.axisAcceleration.*component/curvatureComponent));
                }
                if(const auto magnitude=curvature.length();magnitude>1e-15)
                    velocityLimit=std::min(velocityLimit,
                        std::sqrt(limits.pathAcceleration/magnitude));
                if(!std::isfinite(velocityLimit)||velocityLimit<=0.0) return std::nullopt;
                return RollingSplit{
                    .prefix=std::move(prefix),.suffix=std::move(suffix),
                    .prefixLength=splitDistance,.suffixLength=*length-splitDistance,
                    .position=position,.tangent=tangent,.curvature=curvature,
                    .velocityLimit=velocityLimit,
                };
            },command);
        }

        static MotionState splitBoundaryState(const RollingSplit &split,const double velocity) {
            return {
                split.position,
                scalePosition(split.tangent,velocity),
                scalePosition(split.curvature,velocity*velocity),
            };
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
                return std::unexpected(std::format(
                    "planned chunk branch state does not match normal-motion terminal state: "
                    "position={} velocity={} acceleration={}",
                    (normalEnd.position-chunk.branchState.position).length(),
                    (normalEnd.velocity-chunk.branchState.velocity).length(),
                    (normalEnd.acceleration-chunk.branchState.acceleration).length()));
            auto previous = chunk.branchState;
            const auto &limits = m_compiler.limits();
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
            const auto &limits=m_compiler.limits();
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
                }
                previous=chunk.normalMotion[chunk.normalMotion.size-1].end;
                predecessor=chunk.branch;
            }
            return {};
        }

        void record(const ExecutionItem &item,const std::size_t commandCount=1) {
            m_diagnostics.commandsPlanned+=commandCount;
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

        void recordPlanningTime(const double planningSeconds,const bool continuous) {
            m_diagnostics.lastPlanningSeconds=planningSeconds;
            m_diagnostics.maximumPlanningSeconds=std::max(
                m_diagnostics.maximumPlanningSeconds,planningSeconds);
            m_diagnostics.totalPlanningSeconds+=planningSeconds;
            if(continuous) {
                if(m_diagnostics.continuousHorizons==0)
                    m_diagnostics.firstContinuousHorizonSeconds=planningSeconds;
                ++m_diagnostics.continuousHorizons;
                m_diagnostics.lastContinuousHorizonSeconds=planningSeconds;
                m_diagnostics.maximumContinuousHorizonSeconds=std::max(
                    m_diagnostics.maximumContinuousHorizonSeconds,planningSeconds);
                m_diagnostics.totalContinuousHorizonSeconds+=planningSeconds;
            }
        }

    public:
        explicit TrajectoryPlanner(const TrajectoryLimits limits = {})
            : m_compiler(limits) { }

        void reset(const EpochId epoch, const position_t &position = {}) {
            m_window.clear();
            m_preparedWindow.reset();
            m_compiler.reset(epoch, position);
            m_continuousBoundary={position,{},{}};
            m_lastRollingCandidateError.clear();
            m_nextRollingAttemptWindowSize=2;
            m_lastRollingVelocityFraction.reset();
        }

        void rebase(const EpochId epoch, const position_t &position) {
            m_compiler.reset(epoch,position);
            m_continuousBoundary={position,{},{}};
        }

        void clearDiagnostics() { m_diagnostics = {}; }
        void setLimits(const TrajectoryLimits &limits) {
            m_compiler.setLimits(limits);
            m_lastRollingVelocityFraction.reset();
        }
        void setContinuousPlanningEffort(const ContinuousPlanningEffort &effort) {
            m_compiler.setContinuousPlanningEffort(effort);
        }
        void setContinuousDiagnosticCallback(std::function<void(
                const ContinuousTrajectoryPlan &,
                std::span<const TrajectoryPlannerInput>)> callback) {
            m_continuousDiagnosticCallback=std::move(callback);
        }
        void setProgressCallback(std::function<void()> callback) {
            m_compiler.setProgressCallback(std::move(callback));
        }
        const TrajectoryLimits &limits() const { return m_compiler.limits(); }
        const TrajectoryPlanningDiagnostics &diagnostics() const { return m_diagnostics; }
        const std::string &lastRollingCandidateError() const { return m_lastRollingCandidateError; }
        std::size_t windowSize() const { return m_window.size(); }
        bool hasRollingContinuation() const {
            return !m_window.empty()&&(m_continuousBoundary.velocity.length()>1e-10
                ||m_continuousBoundary.acceleration.length()>1e-10);
        }
        bool shouldPlanRollingPrefix() const {
            if(m_preparedWindow) return false;
            if(m_window.size()<m_nextRollingAttemptWindowSize||m_window.size()<2
               ||!std::ranges::all_of(m_window,continuousMotion)) return false;
            auto nominalDuration=0.0;
            for(const auto &input:m_window) {
                const auto length=continuousMotionLength(input.command);
                const auto feed=continuousFeed(input.command);
                if(!length||feed<=0.0) return false;
                nominalDuration+=*length/feed;
            }
            return nominalDuration>=2.0*m_compiler.limits().lookaheadDuration;
        }
        static bool eligibleForLookahead(const TrajectoryPlannerInput &input) {
            return continuousMotion(input);
        }

        bool canAppend(const TrajectoryPlannerInput &input) const {
            if(m_window.empty()) return true;
            const auto previousEnd=motionEnd(m_window.back().command);
            const auto nextStart=motionStart(input.command);
            return continuousMotion(input) && continuousMotion(m_window.front())
                && input.metadata.pathTolerance == m_window.front().metadata.pathTolerance
                &&sameProtectedTrajectoryPresentation(input.presentation,m_window.back().presentation)
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

        bool enqueuePrepared(const PreparedGeometrySlice &slice) {
            if(m_preparedWindow || !m_window.empty()) return false;
            if(slice.commands.empty() || slice.pieces.empty()) return false;
            const auto exactStop = std::ranges::all_of(slice.commands, [](const auto &record) {
                return record.metadata.pathMode == ExecutablePathMode::ExactStop;
            });
            const auto continuous = std::ranges::all_of(slice.commands, [](const auto &record) {
                return record.metadata.pathMode == ExecutablePathMode::Continuous;
            });
            if(!exactStop && !continuous) return false;
            if(exactStop) {
                if(slice.pieces.size() != slice.commands.size()) return false;
                for(const auto &record : slice.commands) {
                    const auto piece = std::ranges::find_if(slice.pieces, [&](const auto &candidate) {
                        return candidate.primaryCommand == record.id;
                    });
                    if(piece == slice.pieces.end()) return false;
                    if(!enqueue(TrajectoryPlannerInput{record.command, record.metadata,
                            record.presentation, record.presentationActivation,
                            record.continuousScaleOverride, *piece})) return false;
                }
                return true;
            }
            PreparedContinuousGeometry geometry;
            geometry.commands=slice.commands;
            geometry.pieces=slice.pieces;
            geometry.diagnostics.pathLength=slice.pathLength;
            geometry.diagnostics.nominalDuration=slice.nominalDuration;
            for(const auto &record:slice.commands)
                if(!enqueue(TrajectoryPlannerInput{record.command,record.metadata,
                        record.presentation,record.presentationActivation,
                        record.continuousScaleOverride})) return false;
            m_preparedWindow=std::move(geometry);
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
                    auto move = m_compiler.compileTriggeredMove(command);
                    if(!move) return std::unexpected(move.error());
                    return ExecutionItem { *move };
                } else {
                    auto chunk = m_compiler.compile(input.command,
                        input.preparedPiece ? &*input.preparedPiece : nullptr);
                    m_diagnostics.timeLaw+=m_compiler.lastTimeLawDiagnostics();
                    if(!chunk) return std::unexpected(chunk.error());
                    if(auto verified = verifyStopBranch(*chunk); !verified)
                        return std::unexpected(std::format("{}; command {}",
                            verified.error(), input.command.index()));
                    return ExecutionItem { *chunk };
                }
            }, input.command);
            if(!item) return std::unexpected(item.error());
            // Exact-stop commands advance TrajectoryCompiler's internal
            // position. Keep the independently retained rolling PVA boundary at
            // that same held endpoint so a later G64 window does not inherit a
            // stale probe/rebase position.
            m_continuousBoundary={m_compiler.plannedPosition(),{}, {}};
            const auto planningSeconds = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - started).count();
            record(*item);
            recordPlanningTime(planningSeconds,false);
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

        std::expected<std::unique_ptr<PlannedExecution>, std::string> planWindow(
                const bool allowTerminalStop=true) {
            const auto allContinuous=std::ranges::all_of(m_window,continuousMotion);
            const auto movingBoundary=m_continuousBoundary.velocity.length()>1e-10
                ||m_continuousBoundary.acceleration.length()>1e-10;
            if(!allContinuous||(!m_preparedWindow&&m_window.size()<2&&!movingBoundary))
                return planOne();
            const auto started=std::chrono::steady_clock::now();
            constexpr double DEFAULT_BLEND_SCALE=0.001;
            const auto blendScale=std::max(
                m_window.front().metadata.pathTolerance.value_or(DEFAULT_BLEND_SCALE),1e-9);
            const auto context=continuousWindowContext();
            const auto finalize=[&](std::unique_ptr<ContinuousTrajectoryPlan> continuous,
                                    std::vector<TrajectoryPlannerInput> inputs)
                    -> std::expected<std::unique_ptr<PlannedExecution>,std::string> {
                if(auto verified=verifyContinuousPlan(continuous->chunks);!verified)
                    return std::unexpected(std::format(
                        "fatal continuous-motion verification failure: {}; packets={}; cause: {}",
                        context,continuous->chunks.size(),verified.error()));
                for(std::size_t chunk=0;chunk<continuous->chunks.size();++chunk)
                    if(auto verified=verifyStopBranch(continuous->chunks[chunk]);!verified)
                        return std::unexpected(std::format(
                            "fatal continuous-motion stop-branch verification failure: {}; packet={} "
                            "chunk={} normal spans={} stop spans={}; cause: {}",context,chunk,
                            continuous->chunks[chunk].id,
                            continuous->chunks[chunk].normalMotion.size,
                            continuous->chunks[chunk].stopTail.size,verified.error()));

                auto representativeCommand=inputs.front().command;
                auto representativeMetadata=inputs.front().metadata;
                std::vector<ExecutionItem> items;
                items.reserve(continuous->chunks.size());
                for(auto &chunk:continuous->chunks) items.emplace_back(std::move(chunk));
                std::vector<std::size_t> activationItems(continuous->activationSpans.size(),0);
                for(std::size_t input=0;input<continuous->activationSpans.size();++input) {
                    const auto activation=continuous->activationSpans[input];
                    if(activation==0&&!inputs[input].presentationActivation) continue;
                    const auto owner=std::ranges::find_if(items,[&](const auto &item) {
                        const auto &chunk=std::get<PlanChunk>(item);
                        return std::ranges::any_of(chunk.normalMotion,[&](const auto &span) {
                            return span.id==activation;
                        });
                    });
                    if(owner==items.end()) return std::unexpected(std::format(
                        "fatal continuous-motion metadata failure: {}; activation span {} for input {} "
                        "does not belong to any packet",context,activation,input));
                    activationItems[input]=static_cast<std::size_t>(
                        std::distance(items.begin(),owner));
                }
                const auto activeInputs=std::ranges::count_if(inputs,[](const auto &input) {
                    return input.presentationActivation;
                });
                accumulateSplineInverse(
                    m_diagnostics.publishedSplineInverse,continuous->splineInverse);
                accumulateArcInverse(
                    m_diagnostics.publishedArcInverse,continuous->arcInverse);
                m_diagnostics.publishedTimeLaw+=continuous->timeLaw;
                if(m_continuousDiagnosticCallback)
                    m_continuousDiagnosticCallback(*continuous,inputs);
                auto planned=std::make_unique<PlannedExecution>(
                    std::move(representativeCommand),std::move(representativeMetadata),
                    std::move(items),std::move(inputs),std::move(continuous->activationSpans),
                    std::move(activationItems));
                for(std::size_t item=0;item<planned->items.size();++item)
                    record(planned->items[item],item==0?activeInputs:0);
                const auto planningSeconds=std::chrono::duration<double>(
                    std::chrono::steady_clock::now()-started).count();
                ++m_diagnostics.blendedWindows;
                m_diagnostics.blendedCommands+=activeInputs;
                recordPlanningTime(planningSeconds,true);
                return planned;
            };

            std::vector<std::tuple<std::size_t,RollingSplit,double>> splitCandidates;
            auto totalNominalDuration=0.0;
            for(const auto &input:m_window) {
                const auto length=continuousMotionLength(input.command);
                const auto feed=continuousFeed(input.command);
                if(length&&feed>0.0) totalNominalDuration+=*length/feed;
            }
            auto nominalDuration=0.0;
            for(std::size_t index=0;index<m_window.size();++index) {
                const auto length=continuousMotionLength(m_window[index].command);
                const auto feed=continuousFeed(m_window[index].command);
                if(!length||feed<=0.0) break;
                const auto entityScale=m_window[index].continuousScaleOverride>0.0
                    ?m_window[index].continuousScaleOverride:std::min(blendScale,*length/6.0);
                if(auto candidate=std::holds_alternative<MoveLine>(m_window[index].command)
                        ?rollingSplit(m_window[index].command,blendScale):std::nullopt;
                   candidate&&nominalDuration+candidate->prefixLength/feed
                        >=m_compiler.limits().lookaheadDuration
                   &&(allowTerminalStop||totalNominalDuration-nominalDuration
                        -candidate->prefixLength/feed>=m_compiler.limits().lookaheadDuration)) {
                    splitCandidates.emplace_back(index,std::move(*candidate),entityScale);
                    if(splitCandidates.size()>=8) break;
                }
                nominalDuration+=*length/feed;
            }

            for(const auto &[splitIndex,split,splitScale]:splitCandidates) {
                std::vector<MachineCommand> prefixCommands;
                prefixCommands.reserve(splitIndex+1);
                for(std::size_t index=0;index<splitIndex;++index)
                    prefixCommands.push_back(m_window[index].command);
                prefixCommands.push_back(split.prefix);
                std::vector<double> prefixScales;
                prefixScales.reserve(splitIndex+1);
                for(std::size_t index=0;index<splitIndex;++index)
                    prefixScales.push_back(m_window[index].continuousScaleOverride);
                prefixScales.push_back(splitScale);

                std::vector<MachineCommand> suffixProbeCommands{split.suffix};
                std::vector<double> suffixProbeScales{splitScale};
                auto suffixProbeDuration=split.suffixLength
                    /continuousFeed(split.suffix);
                for(std::size_t index=splitIndex+1;
                    index<m_window.size()
                        &&suffixProbeDuration<m_compiler.limits().lookaheadDuration;++index) {
                    suffixProbeCommands.push_back(m_window[index].command);
                    suffixProbeScales.push_back(m_window[index].continuousScaleOverride);
                    const auto length=continuousMotionLength(m_window[index].command);
                    const auto feed=continuousFeed(m_window[index].command);
                    if(length&&feed>0.0) suffixProbeDuration+=*length/feed;
                }

                std::unique_ptr<ContinuousTrajectoryPlan> selected;
                std::optional<TrajectoryCompiler> selectedPlanner;
                MotionState selectedBoundary;
                auto selectedVelocityFraction=0.0;
                const auto initialVelocityFraction=m_lastRollingVelocityFraction
                    ?std::min(1.0,2.0 * *m_lastRollingVelocityFraction):1.0;
                for(unsigned attempt=0;attempt<6&&!selected;++attempt) {
                    const auto velocityFraction=initialVelocityFraction*std::pow(0.5,attempt);
                    const auto velocity=split.velocityLimit*velocityFraction;
                    if(velocity<split.velocityLimit*0.01) break;
                    ++m_diagnostics.rollingBoundaryCandidates;
                    const auto boundary=splitBoundaryState(split,velocity);
                    TrajectoryCompiler suffixProbe(m_compiler.limits());
                    suffixProbe.setContinuousPlanningEffort(
                        m_compiler.continuousPlanningEffort());
                    suffixProbe.setProgressCallback(m_compiler.progressCallback());
                    suffixProbe.reset(1,split.position);
                    auto suffix=suffixProbe.compileContinuous(
                        suffixProbeCommands,blendScale,boundary,std::nullopt,
                        suffixProbeScales,ROLLING_VELOCITY_SEARCH_ITERATIONS);
                    m_diagnostics.timeLaw+=suffixProbe.lastTimeLawDiagnostics();
                    m_diagnostics.rollingSuffixProbeTimeLaw+=
                        suffixProbe.lastTimeLawDiagnostics();
                    if(!suffix) {
                        ++m_diagnostics.rollingSuffixProbeFailures;
                        m_lastRollingCandidateError="suffix: "+suffix.error();
                        continue;
                    }
                    auto prefixPlanner=m_compiler;
                    auto prefix=prefixPlanner.compileContinuous(
                        prefixCommands,blendScale,m_continuousBoundary,boundary,
                        prefixScales,ROLLING_VELOCITY_SEARCH_ITERATIONS);
                    m_diagnostics.timeLaw+=prefixPlanner.lastTimeLawDiagnostics();
                    m_diagnostics.rollingPrefixProbeTimeLaw+=
                        prefixPlanner.lastTimeLawDiagnostics();
                    if(!prefix) {
                        ++m_diagnostics.rollingPrefixProbeFailures;
                        m_lastRollingCandidateError="prefix: "+prefix.error();
                        continue;
                    }
                    selected=std::move(*prefix);
                    selectedPlanner=std::move(prefixPlanner);
                    selectedBoundary=boundary;
                    selectedVelocityFraction=velocityFraction;
                }
                if(selected&&selectedPlanner) {
                    std::vector<TrajectoryPlannerInput> inputs;
                    inputs.reserve(splitIndex+1);
                    for(std::size_t index=0;index<=splitIndex;++index)
                        inputs.push_back(m_window[index]);
                    for(std::size_t index=0;index<splitIndex;++index) m_window.pop_front();
                    m_window.front().command=split.suffix;
                    m_window.front().presentationActivation=false;
                    m_window.front().continuousScaleOverride=splitScale;
                    m_compiler=std::move(*selectedPlanner);
                    m_continuousBoundary=selectedBoundary;
                    m_lastRollingVelocityFraction=selectedVelocityFraction;
                    m_lastRollingCandidateError.clear();
                    m_nextRollingAttemptWindowSize=m_window.size()+1;
                    return finalize(std::move(selected),std::move(inputs));
                }
            }

            if(!allowTerminalStop) {
                const auto planningSeconds=std::chrono::duration<double>(
                    std::chrono::steady_clock::now()-started).count();
                m_diagnostics.totalPlanningSeconds+=planningSeconds;
                m_diagnostics.rollingSearchSeconds+=planningSeconds;
                m_nextRollingAttemptWindowSize=m_window.size()+4;
                return std::unique_ptr<PlannedExecution>{};
            }

            std::vector<MachineCommand> commands;
            std::vector<double> scales;
            commands.reserve(m_window.size());
            scales.reserve(m_window.size());
            for(const auto &input:m_window) {
                commands.push_back(input.command);
                scales.push_back(input.continuousScaleOverride);
            }
            auto continuous=m_preparedWindow
                ?m_compiler.compileContinuous(*m_preparedWindow,blendScale,
                    m_continuousBoundary,std::nullopt,
                    movingBoundary?ROLLING_VELOCITY_SEARCH_ITERATIONS:12U)
                :m_compiler.compileContinuous(commands,blendScale,
                    m_continuousBoundary,std::nullopt,scales,
                    movingBoundary?ROLLING_VELOCITY_SEARCH_ITERATIONS:12U);
            m_diagnostics.timeLaw+=m_compiler.lastTimeLawDiagnostics();
            if(!continuous) return std::unexpected(std::format(
                "fatal continuous-motion compilation failure: {}; cause: {}",context,continuous.error()));
            std::vector<TrajectoryPlannerInput> inputs;
            inputs.reserve(m_window.size());
            while(!m_window.empty()) {
                inputs.push_back(std::move(m_window.front()));
                m_window.pop_front();
            }
            m_preparedWindow.reset();
            const auto &terminal=(*continuous)->chunks.back().branchState;
            m_continuousBoundary=terminal;
            return finalize(std::move(*continuous),std::move(inputs));
        }

        bool shouldPlanImmediately() const {
            return !m_window.empty() && !continuousMotion(m_window.front());
        }
    };
}
