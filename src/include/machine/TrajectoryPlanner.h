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
#include <numeric>
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
    };

    // NRT-only compatible command horizon. RT capacity is imposed later while
    // the verified polynomial stream is packetized into PlanChunk values; it is
    // deliberately not expressed as an arbitrary G-code command count.
    class TrajectoryPlanner {
    private:
        TrajectoryCompiler m_compiler;
        std::deque<TrajectoryPlannerInput> m_window;
        std::optional<PreparedContinuousGeometry> m_preparedWindow;
        std::optional<ContinuousChainId> m_preparedChain;
        bool m_preparedChainEnded = false;
        TrajectoryPlanningDiagnostics m_diagnostics;
        MotionState m_continuousBoundary{};
        std::optional<double> m_lastRollingVelocityFraction;
        std::string m_planningActivity;
        std::string m_lastContinuousPlanSummary;
        std::string m_lastContinuousCorrectionHistory;
        std::string m_lastPreparedEnqueueError;
        std::chrono::steady_clock::time_point m_planningActivityStarted{};
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

        static std::string inputLocation(const TrajectoryPlannerInput &input) {
            if(input.presentation.activeBlocks.empty()) return "<no active source block>";
            const auto &block=input.presentation.activeBlocks.back();
            return std::format("{}:{} block {} '{}'",block.source,block.line,block.id,block.text);
        }

        static std::string inputLocation(const PreparedCommandRecord &record) {
            if(record.presentation.activeBlocks.empty()) return "<no active source block>";
            const auto &block=record.presentation.activeBlocks.back();
            return std::format("{}:{} block {} '{}'",block.source,block.line,block.id,block.text);
        }

        std::string continuousWindowContext() const {
            if(m_preparedWindow) {
                if(m_preparedWindow->commands.empty()) return std::format(
                    "prepared G64 window commands=0 pieces={}",
                    m_preparedWindow->pieces.size());
                const auto &first=m_preparedWindow->commands.front();
                const auto &last=m_preparedWindow->commands.back();
                return std::format(
                    "prepared G64 window commands={} pieces={} P={} first={} last={} start={} end={}",
                    m_preparedWindow->commands.size(),m_preparedWindow->pieces.size(),
                    first.metadata.pathTolerance
                        ?std::format("{}",*first.metadata.pathTolerance):std::string("<default>"),
                    inputLocation(first),inputLocation(last),
                    formatPosition(m_preparedWindow->beginning.position),
                    formatPosition(m_preparedWindow->ending.position));
            }
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

        static PreparedGeometryBoundary preparedBoundary(
                const PreparedPathPiece &piece,const double localDistance) {
            CurveEvaluationWorkspace workspace;
            const auto distance=piece.curveFrom
                +std::clamp(localDistance,0.0,piece.length());
            return {
                positionAtDistance(*piece.curve,distance,workspace),
                tangentAtDistance(*piece.curve,distance,workspace),
                curvatureAtDistance(*piece.curve,distance,workspace),
                curvatureDerivativeAtDistance(*piece.curve,distance,workspace),
            };
        }

        static void resamplePreparedPiece(PreparedPathPiece &piece) {
            constexpr unsigned SAMPLE_INTERVALS=64;
            CurveEvaluationWorkspace workspace;
            piece.geometricSamples.clear();
            piece.geometricSamples.reserve(SAMPLE_INTERVALS+1);
            for(unsigned index=0;index<=SAMPLE_INTERVALS;++index) {
                const auto local=piece.length()*index/static_cast<double>(SAMPLE_INTERVALS);
                const auto distance=piece.curveFrom+local;
                const auto position=positionAtDistance(*piece.curve,distance,workspace);
                const auto tangent=tangentAtDistance(*piece.curve,distance,workspace);
                const auto curvature=curvatureAtDistance(*piece.curve,distance,workspace);
                const auto derivative=curvatureDerivativeAtDistance(
                    *piece.curve,distance,workspace);
                const auto tangential=positionDot(tangent,derivative);
                const auto normal=derivative-scalePosition(tangent,tangential);
                piece.geometricSamples.push_back({local,position,tangent,curvature,
                    derivative,normal.length(),derivative.length()});
            }
        }

        static PreparedContinuousGeometry geometryForPieces(
                const PreparedContinuousGeometry &source,
                std::vector<PreparedPathPiece> pieces) {
            PreparedContinuousGeometry result;
            result.pieces=std::move(pieces);
            std::vector<PreparedCommandId> referenced;
            std::vector<PreparedCommandId> activated;
            for(const auto &piece:result.pieces) {
                referenced.push_back(piece.primaryCommand);
                referenced.insert(referenced.end(),piece.activationCommands.begin(),
                    piece.activationCommands.end());
                referenced.insert(referenced.end(),piece.sourceCommands.begin(),
                    piece.sourceCommands.end());
                activated.insert(activated.end(),piece.activationCommands.begin(),
                    piece.activationCommands.end());
                result.diagnostics.pathLength+=piece.length();
                if(piece.programmedFeed>0.0)
                    result.diagnostics.nominalDuration+=piece.length()/piece.programmedFeed;
                ++result.diagnostics.preparedPieces;
                switch(piece.kind) {
                    case PreparedPieceKind::RetainedLineSection:
                        ++result.diagnostics.retainedLineSections;
                        break;
                    case PreparedPieceKind::RetainedArcSection:
                        ++result.diagnostics.retainedArcSections;
                        break;
                    case PreparedPieceKind::JunctionBlend:
                        ++result.diagnostics.junctionBlends;
                        break;
                    case PreparedPieceKind::ClusterSpline:
                        ++result.diagnostics.clusterSplines;
                        break;
                }
            }
            for(const auto &record:source.commands) {
                if(!std::ranges::contains(referenced,record.id)) continue;
                auto retained=record;
                retained.presentationActivation=retained.presentationActivation
                    &&std::ranges::contains(activated,record.id);
                result.commands.push_back(std::move(retained));
            }
            result.diagnostics.sourceCommands=result.commands.size();
            if(!result.pieces.empty()) {
                result.beginning=preparedBoundary(result.pieces.front(),0.0);
                result.ending=preparedBoundary(
                    result.pieces.back(),result.pieces.back().length());
            }
            return result;
        }

        struct PreparedRollingSplit {
            PreparedContinuousGeometry prefix;
            PreparedContinuousGeometry suffix;
            MotionState unitBoundary{};
            double velocityLimit = 0.0;
        };

        std::optional<PreparedRollingSplit> splitPreparedGeometry(
                const std::size_t pieceIndex,const double localDistance) const {
            if(!m_preparedWindow||pieceIndex>=m_preparedWindow->pieces.size())
                return std::nullopt;
            const auto &source=*m_preparedWindow;
            const auto &piece=source.pieces[pieceIndex];
            if(piece.kind!=PreparedPieceKind::RetainedLineSection
               ||!piece.geometricallyLinear||localDistance<=1e-10
               ||piece.length()-localDistance<=1e-10) return std::nullopt;
            auto prefixPiece=piece;
            auto suffixPiece=piece;
            prefixPiece.curveTo=piece.curveFrom+localDistance;
            suffixPiece.curveFrom=prefixPiece.curveTo;
            suffixPiece.activationCommands.clear();
            resamplePreparedPiece(prefixPiece);
            resamplePreparedPiece(suffixPiece);

            std::vector<PreparedPathPiece> prefixPieces;
            prefixPieces.reserve(pieceIndex+1);
            prefixPieces.insert(prefixPieces.end(),source.pieces.begin(),
                source.pieces.begin()+static_cast<std::ptrdiff_t>(pieceIndex));
            prefixPieces.push_back(std::move(prefixPiece));
            std::vector<PreparedPathPiece> suffixPieces;
            suffixPieces.reserve(source.pieces.size()-pieceIndex);
            suffixPieces.push_back(std::move(suffixPiece));
            suffixPieces.insert(suffixPieces.end(),
                source.pieces.begin()+static_cast<std::ptrdiff_t>(pieceIndex+1),
                source.pieces.end());

            const auto boundary=preparedBoundary(piece,localDistance);
            auto velocityLimit=piece.programmedFeed;
            for(const auto component:AXIS_COMPONENTS) {
                const auto tangent=std::abs(boundary.tangent.*component);
                if(tangent>1e-15) velocityLimit=std::min(velocityLimit,
                    m_compiler.limits().axisVelocity.*component/tangent);
            }
            if(!std::isfinite(velocityLimit)||velocityLimit<=0.0) return std::nullopt;
            return PreparedRollingSplit{
                .prefix=geometryForPieces(source,std::move(prefixPieces)),
                .suffix=geometryForPieces(source,std::move(suffixPieces)),
                .unitBoundary={boundary.position,boundary.tangent,boundary.curvature},
                .velocityLimit=velocityLimit,
            };
        }

        static std::vector<TrajectoryPlannerInput> preparedInputs(
                const PreparedContinuousGeometry &geometry) {
            std::vector<TrajectoryPlannerInput> result;
            result.reserve(geometry.commands.size());
            for(const auto &record:geometry.commands)
                result.emplace_back(record.command,record.metadata,record.presentation,
                    record.presentationActivation,record.continuousScaleOverride);
            return result;
        }

        void retainPreparedInputs(const PreparedContinuousGeometry &geometry) {
            m_window.clear();
            for(auto &input:preparedInputs(geometry)) m_window.push_back(std::move(input));
            m_diagnostics.maximumWindowCommands=std::max(
                m_diagnostics.maximumWindowCommands,m_window.size());
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
                    if(exceeds(trajectory_detail::maximumAxisVelocity(span, component),
                               limits.axisVelocity.*component)
                       || exceeds(trajectory_detail::maximumAxisAcceleration(span, component),
                                  limits.axisAcceleration.*component)
                       || exceeds(trajectory_detail::maximumAxisJerk(span, component),
                                  limits.axisJerk.*component))
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
                    if(exceeds(trajectory_detail::maximumAxisVelocity(span,component),
                               limits.axisVelocity.*component)
                       ||exceeds(trajectory_detail::maximumAxisAcceleration(span,component),
                                 limits.axisAcceleration.*component)
                       ||exceeds(trajectory_detail::maximumAxisJerk(span,component),
                                 limits.axisJerk.*component))
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

        void setPlanningActivity(std::string activity) {
            m_planningActivity=std::move(activity);
            m_planningActivityStarted=std::chrono::steady_clock::now();
        }

    public:
        explicit TrajectoryPlanner(const TrajectoryLimits limits = {})
            : m_compiler(limits) { }

        void reset(const EpochId epoch, const position_t &position = {}) {
            m_window.clear();
            m_preparedWindow.reset();
            m_preparedChain.reset();
            m_preparedChainEnded=false;
            m_compiler.reset(epoch, position);
            m_continuousBoundary={position,{},{}};
            m_lastRollingVelocityFraction.reset();
            m_planningActivity.clear();
            m_lastContinuousPlanSummary.clear();
            m_lastContinuousCorrectionHistory.clear();
            m_lastPreparedEnqueueError.clear();
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
        const std::string &planningActivity() const { return m_planningActivity; }
        const std::string &lastContinuousPlanSummary() const {
            return m_lastContinuousPlanSummary;
        }
        const std::string &lastContinuousCorrectionHistory() const {
            return m_lastContinuousCorrectionHistory;
        }
        const std::string &lastPreparedEnqueueError() const {
            return m_lastPreparedEnqueueError;
        }
        double planningActivitySeconds() const {
            if(m_planningActivity.empty()) return 0.0;
            return std::chrono::duration<double>(
                std::chrono::steady_clock::now()-m_planningActivityStarted).count();
        }
        std::size_t preparedPieceCount() const {
            return m_preparedWindow?m_preparedWindow->pieces.size():0;
        }
        double preparedNominalDuration() const {
            return m_preparedWindow?m_preparedWindow->diagnostics.nominalDuration:0.0;
        }
        bool preparedChainEnded() const { return m_preparedChainEnded; }
        std::size_t windowSize() const {
            return m_preparedWindow?m_preparedWindow->commands.size():m_window.size();
        }
        bool hasRollingContinuation() const {
            return windowSize()!=0&&(m_continuousBoundary.velocity.length()>1e-10
                ||m_continuousBoundary.acceleration.length()>1e-10);
        }
        bool shouldPlanRollingPrefix() const {
            if(!m_preparedWindow) return false;
            if(m_preparedChainEnded) return true;
            return m_preparedWindow->diagnostics.nominalDuration
                >=2.0*m_compiler.limits().lookaheadDuration;
        }
        bool enqueue(TrajectoryPlannerInput input) {
            if(continuousMotion(input)) {
                if(!m_preparedWindow) return false;
                ++m_diagnostics.continuousModeInputs;
            }
            m_window.push_back(std::move(input));
            m_diagnostics.maximumWindowCommands = std::max(
                m_diagnostics.maximumWindowCommands, m_window.size());
            return true;
        }

        bool enqueuePrepared(const PreparedGeometrySlice &slice) {
            m_lastPreparedEnqueueError.clear();
            const auto reject = [&](std::string reason) {
                m_lastPreparedEnqueueError = std::format(
                    "slice sequence={} chain={} commands={} pieces={}: {}",
                    slice.sequence, slice.chain, slice.commands.size(), slice.pieces.size(),
                    std::move(reason));
                return false;
            };
            if(slice.commands.empty() || slice.pieces.empty())
                return reject("slice is empty");
            const auto exactStop = std::ranges::all_of(slice.commands, [](const auto &record) {
                return record.metadata.pathMode == ExecutablePathMode::ExactStop;
            });
            const auto continuous = std::ranges::all_of(slice.commands, [](const auto &record) {
                return record.metadata.pathMode == ExecutablePathMode::Continuous;
            });
            if(!exactStop && !continuous)
                return reject("slice mixes exact-stop and continuous commands");
            if(exactStop) {
                if(m_preparedWindow||!m_window.empty())
                    return reject(std::format(
                        "exact-stop slice overlaps retained work: prepared={} retained_commands={}",
                        m_preparedWindow.has_value(), m_window.size()));
                if(slice.pieces.size() != slice.commands.size())
                    return reject("exact-stop piece count does not match its command count");
                for(const auto &record : slice.commands) {
                    const auto piece = std::ranges::find_if(slice.pieces, [&](const auto &candidate) {
                        return candidate.primaryCommand == record.id;
                    });
                    if(piece == slice.pieces.end()) return reject(std::format(
                        "exact-stop command {} has no owning piece", record.id));
                    if(!enqueue(TrajectoryPlannerInput{record.command, record.metadata,
                            record.presentation, record.presentationActivation,
                            record.continuousScaleOverride, *piece}))
                        return reject(std::format(
                            "exact-stop command {} was rejected by the command window", record.id));
                }
                return true;
            }
            if(m_preparedChainEnded)
                return reject("continuous slice arrived after its prepared chain end");
            if(m_preparedWindow) {
                if(!m_preparedChain||*m_preparedChain!=slice.chain)
                    return reject(std::format(
                        "continuous chain does not match retained chain {}",
                        m_preparedChain.value_or(0)));
            } else {
                if(!m_window.empty()) return reject(std::format(
                    "continuous slice overlaps {} retained non-prepared commands", m_window.size()));
                m_preparedWindow.emplace();
                m_preparedChain=slice.chain;
            }
            auto &geometry=*m_preparedWindow;
            const auto knownCommand=[&](const PreparedCommandId id) {
                return std::ranges::any_of(geometry.commands,
                           [id](const auto &record) { return record.id==id; })
                    ||std::ranges::any_of(slice.commands,
                           [id](const auto &record) { return record.id==id; });
            };
            for(const auto &piece : slice.pieces) {
                if(!piece.curve) return reject(std::format("piece {} has no curve", piece.id));
                if(piece.length()<=1e-12)
                    return reject(std::format("piece {} has non-positive length", piece.id));
                if(!knownCommand(piece.primaryCommand)) return reject(std::format(
                    "piece {} references unknown primary command {}", piece.id,
                    piece.primaryCommand));
                for(const auto id : piece.activationCommands)
                    if(!knownCommand(id)) return reject(std::format(
                        "piece {} references unknown activation command {}", piece.id, id));
                for(const auto id : piece.sourceCommands)
                    if(!knownCommand(id)) return reject(std::format(
                        "piece {} references unknown source command {}", piece.id, id));
                if(std::ranges::any_of(geometry.pieces,
                        [&](const auto &retained) { return retained.id==piece.id; }))
                    return reject(std::format("piece {} overlaps a retained piece ID", piece.id));
            }
            if(!geometry.pieces.empty()) {
                const auto previous=preparedBoundary(
                    geometry.pieces.back(),geometry.pieces.back().length()).position;
                const auto next=preparedBoundary(slice.pieces.front(),0.0).position;
                if(!samePosition(previous,next)) return reject(std::format(
                    "geometry boundary distance {:.17g} exceeds continuity tolerance",
                    (previous-next).length()));
            }
            for(const auto &record:slice.commands) {
                if(std::ranges::any_of(geometry.commands,
                    [&](const auto &retained) { return retained.id==record.id; })) continue;
                geometry.commands.push_back(record);
                if(!enqueue(TrajectoryPlannerInput{record.command,record.metadata,
                        record.presentation,record.presentationActivation,
                        record.continuousScaleOverride})) return reject(std::format(
                            "continuous command {} was rejected by the command window", record.id));
            }
            geometry.pieces.insert(geometry.pieces.end(),
                slice.pieces.begin(),slice.pieces.end());
            geometry.beginning=preparedBoundary(geometry.pieces.front(),0.0);
            geometry.ending=preparedBoundary(
                geometry.pieces.back(),geometry.pieces.back().length());
            geometry.diagnostics.pathLength+=slice.pathLength;
            geometry.diagnostics.nominalDuration+=slice.nominalDuration;
            geometry.diagnostics.sourceCommands=geometry.commands.size();
            geometry.diagnostics.preparedPieces=geometry.pieces.size();
            return true;
        }

        bool endPreparedChain(const ContinuousChainId chain) {
            if(!m_preparedWindow) return true;
            if(!m_preparedChain||*m_preparedChain!=chain) return false;
            m_preparedChainEnded=true;
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
            if(!m_preparedWindow&&m_window.empty())
                return std::unique_ptr<PlannedExecution>{};
            if(m_preparedWindow&&(m_preparedWindow->commands.empty()
                                 ||m_preparedWindow->pieces.empty()))
                return std::unexpected(std::format(
                    "prepared G64 horizon lost its command/piece ownership: commands={} pieces={}",
                    m_preparedWindow->commands.size(),m_preparedWindow->pieces.size()));
            const auto allContinuous=std::ranges::all_of(m_window,continuousMotion);
            const auto movingBoundary=m_continuousBoundary.velocity.length()>1e-10
                ||m_continuousBoundary.acceleration.length()>1e-10;
            if(!allContinuous||(!m_preparedWindow&&m_window.size()<2&&!movingBoundary))
                return planOne();
            const auto started=std::chrono::steady_clock::now();
            constexpr double DEFAULT_BLEND_SCALE=0.001;
            const auto pathTolerance=m_preparedWindow
                ?m_preparedWindow->commands.front().metadata.pathTolerance
                :m_window.front().metadata.pathTolerance;
            const auto blendScale=std::max(
                pathTolerance.value_or(DEFAULT_BLEND_SCALE),1e-9);
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
                m_diagnostics.publishedTimeLaw+=continuous->timeLaw;
                if(m_continuousDiagnosticCallback)
                    m_continuousDiagnosticCallback(*continuous,inputs);
                const auto actualDuration=std::accumulate(
                    continuous->pieceTiming.begin(),continuous->pieceTiming.end(),0.0,
                    [](const double total,const auto &piece) {
                        return total+piece.duration;
                    });
                const auto slowest=std::ranges::max_element(
                    continuous->pieceTiming,{},[](const auto &piece) {
                        const auto nominal=piece.programmedVelocityLimit>0.0
                            ?piece.length/piece.programmedVelocityLimit:0.0;
                        return nominal>0.0?piece.duration/nominal:0.0;
                    });
                if(slowest!=continuous->pieceTiming.end()) {
                    const auto nominal=slowest->programmedVelocityLimit>0.0
                        ?slowest->length/slowest->programmedVelocityLimit:0.0;
                    m_lastContinuousPlanSummary=std::format(
                        "pieces={} actual={:.3f}s seed={:.3f}s slowest_input={} "
                        "length={:.6g} nominal={:.6g}s actual={:.6g}s ratio={:.3f} "
                        "programmed_v={:.6g} local_v={:.6g} entry_v={:.6g} exit_v={:.6g} "
                        "local_a={:.6g} local_j={:.6g} scp_solves={} scp_iterations={} "
                        "scp_accepted={} scp_materializations={} scp_seconds={:.6f}",
                        continuous->pieceTiming.size(),actualDuration,
                        continuous->velocityOnlySeedDuration,slowest->input,
                        slowest->length,nominal,slowest->duration,
                        nominal>0.0?slowest->duration/nominal:0.0,
                        slowest->programmedVelocityLimit,slowest->velocityLimit,
                        slowest->entryVelocity,slowest->exitVelocity,
                        slowest->accelerationLimit,slowest->jerkLimit,
                        continuous->scpSolves,continuous->scpSimplexIterations,
                        continuous->scpAcceptedSteps,continuous->scpMaterializationAttempts,
                        continuous->scpSeconds);
                } else {
                    m_lastContinuousPlanSummary="continuous plan has no piece timing diagnostics";
                }
                m_lastContinuousCorrectionHistory=continuous->correctionHistory;
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

            if(m_preparedWindow) {
                setPlanningActivity(std::format(
                    "selecting prepared G64 boundary: commands={} pieces={} nominal={:.3f}s terminal={}",
                    m_preparedWindow->commands.size(),m_preparedWindow->pieces.size(),
                    m_preparedWindow->diagnostics.nominalDuration,allowTerminalStop));
                struct Candidate {
                    std::size_t piece = 0;
                    double distance = 0.0;
                };
                std::vector<Candidate> candidates;
                auto totalDuration=m_preparedWindow->diagnostics.nominalDuration;
                auto precedingDuration=0.0;
                constexpr std::array FRACTIONS{0.25,0.5,0.75};
                for(std::size_t index=0;index<m_preparedWindow->pieces.size()
                    &&candidates.size()<8&&!m_preparedChainEnded;++index) {
                    const auto &piece=m_preparedWindow->pieces[index];
                    if(piece.programmedFeed<=0.0) break;
                    const auto duration=piece.length()/piece.programmedFeed;
                    if(piece.kind==PreparedPieceKind::RetainedLineSection
                       &&piece.geometricallyLinear) {
                        for(const auto fraction:FRACTIONS) {
                            const auto prefixDuration=precedingDuration+duration*fraction;
                            const auto suffixDuration=totalDuration-prefixDuration;
                            if(prefixDuration>=m_compiler.limits().lookaheadDuration
                               &&suffixDuration>1e-9
                               &&(allowTerminalStop||suffixDuration
                                    >=m_compiler.limits().lookaheadDuration))
                                candidates.push_back({index,piece.length()*fraction});
                            if(candidates.size()>=8) break;
                        }
                    }
                    precedingDuration+=duration;
                }

                for(const auto &candidate:candidates) {
                    auto split=splitPreparedGeometry(candidate.piece,candidate.distance);
                    if(!split) continue;
                    const auto initialVelocityFraction=m_lastRollingVelocityFraction
                        ?std::min(1.0,2.0**m_lastRollingVelocityFraction):1.0;
                    for(unsigned attempt=0;attempt<6;++attempt) {
                        const auto velocityFraction=initialVelocityFraction*std::pow(0.5,attempt);
                        const auto velocity=split->velocityLimit*velocityFraction;
                        if(velocity<split->velocityLimit*0.01) break;
                        ++m_diagnostics.rollingBoundaryCandidates;
                        MotionState boundary{
                            split->unitBoundary.position,
                            scalePosition(split->unitBoundary.velocity,velocity),
                            scalePosition(split->unitBoundary.acceleration,velocity*velocity),
                        };
                        setPlanningActivity(std::format(
                            "proving prepared G64 suffix: candidate_piece={} attempt={} "
                            "commands={} pieces={} nominal={:.3f}s",
                            candidate.piece,attempt,split->suffix.commands.size(),
                            split->suffix.pieces.size(),
                            split->suffix.diagnostics.nominalDuration));
                        TrajectoryCompiler suffixProbe(m_compiler.limits());
                        suffixProbe.setContinuousPlanningEffort(
                            m_compiler.continuousPlanningEffort());
                        suffixProbe.setProgressCallback(m_compiler.progressCallback());
                        suffixProbe.reset(1,boundary.position);
                        auto suffix=suffixProbe.compileContinuous(
                            split->suffix,blendScale,boundary,std::nullopt,
                            ROLLING_VELOCITY_SEARCH_ITERATIONS);
                        m_diagnostics.timeLaw+=suffixProbe.lastTimeLawDiagnostics();
                        m_diagnostics.rollingSuffixProbeTimeLaw+=
                            suffixProbe.lastTimeLawDiagnostics();
                        if(!suffix) {
                            ++m_diagnostics.rollingSuffixProbeFailures;
                            continue;
                        }
                        auto prefixPlanner=m_compiler;
                        setPlanningActivity(std::format(
                            "compiling prepared G64 prefix: candidate_piece={} attempt={} "
                            "commands={} pieces={} nominal={:.3f}s",
                            candidate.piece,attempt,split->prefix.commands.size(),
                            split->prefix.pieces.size(),
                            split->prefix.diagnostics.nominalDuration));
                        auto prefix=prefixPlanner.compileContinuous(
                            split->prefix,blendScale,m_continuousBoundary,boundary,
                            ROLLING_VELOCITY_SEARCH_ITERATIONS);
                        m_diagnostics.timeLaw+=prefixPlanner.lastTimeLawDiagnostics();
                        m_diagnostics.rollingPrefixProbeTimeLaw+=
                            prefixPlanner.lastTimeLawDiagnostics();
                        if(!prefix) {
                            ++m_diagnostics.rollingPrefixProbeFailures;
                            continue;
                        }

                        auto inputs=preparedInputs(split->prefix);
                        m_compiler=std::move(prefixPlanner);
                        m_continuousBoundary=boundary;
                        m_lastRollingVelocityFraction=velocityFraction;
                        m_preparedWindow=std::move(split->suffix);
                        retainPreparedInputs(*m_preparedWindow);
                        auto finalized=finalize(std::move(*prefix),std::move(inputs));
                        if(finalized) m_planningActivity.clear();
                        return finalized;
                    }
                }

                if(!allowTerminalStop) {
                    const auto planningSeconds=std::chrono::duration<double>(
                        std::chrono::steady_clock::now()-started).count();
                    m_diagnostics.totalPlanningSeconds+=planningSeconds;
                    m_diagnostics.rollingSearchSeconds+=planningSeconds;
                    m_planningActivity.clear();
                    return std::unique_ptr<PlannedExecution>{};
                }

                auto inputs=preparedInputs(*m_preparedWindow);
                setPlanningActivity(std::format(
                    "terminal-compiling prepared G64: commands={} pieces={} nominal={:.3f}s",
                    m_preparedWindow->commands.size(),m_preparedWindow->pieces.size(),
                    m_preparedWindow->diagnostics.nominalDuration));
                auto continuous=m_compiler.compileContinuous(*m_preparedWindow,blendScale,
                    m_continuousBoundary,std::nullopt,
                    movingBoundary?ROLLING_VELOCITY_SEARCH_ITERATIONS:12U);
                m_diagnostics.timeLaw+=m_compiler.lastTimeLawDiagnostics();
                if(!continuous) return std::unexpected(std::format(
                    "fatal prepared continuous-motion compilation failure: {}; cause: {}",
                    context,continuous.error()));
                m_window.clear();
                m_preparedWindow.reset();
                m_preparedChain.reset();
                m_preparedChainEnded=false;
                const auto &terminal=(*continuous)->chunks.back().branchState;
                m_continuousBoundary=terminal;
                auto finalized=finalize(std::move(*continuous),std::move(inputs));
                if(finalized) m_planningActivity.clear();
                return finalized;
            }

            return std::unexpected(
                "continuous motion requires prepared geometry");
       }

        bool shouldPlanImmediately() const {
            return !m_window.empty() && !continuousMotion(m_window.front());
        }
    };
}
