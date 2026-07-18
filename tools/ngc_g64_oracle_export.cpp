#include <algorithm>
#include <cmath>
#include <chrono>
#include <filesystem>
#include <format>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "evaluator/Evaluator.h"
#include "evaluator/Preamble.h"
#include "machine/TrajectoryCompiler.h"
#include "machine/TrajectoryPlanner.h"
#include "machine/Machine.h"
#include "machine/MachineConfiguration.h"
#include "parser/Program.h"
#include "utils.h"

namespace {
    struct ExecutedSpanTrace {
        std::size_t horizon=0;
        std::size_t sourceInput=0;
        ngc::position_t start{};
        ngc::position_t end{};
        double length=0.0;
        double duration=0.0;
        double middleSpeed=0.0;
        double feed=0.0;
    };

    struct PlannedPieceTrace {
        std::size_t horizon=0;
        std::size_t sourceInput=0;
        ngc::ContinuousPieceTimingDiagnostic timing;
    };

    struct CapturedSplineTrace {
        std::size_t horizon=0;
        std::size_t firstSource=0;
        std::size_t lastSource=0;
        std::size_t degree=0;
        std::vector<ngc::position_t> controls;
        std::vector<double> pieceBoundaries;
    };

    void printTimeLawDiagnostics(const std::string_view label,
                                 const ngc::TimeLawDiagnostics &diagnostics) {
        const auto total=ngc::totalTimeLawCalls(diagnostics);
        const auto category=[](const ngc::TimeLawCallDiagnostics &value) {
            return std::format("{}/{}/{}/{}/{}/{}/{}/{}/{}@{}s",value.calls,
                value.successes,value.failures,value.solverCalls,value.cacheHits,
                value.cacheMisses,value.cacheCollisions,value.cacheMaterializations,
                value.correctionPassCalls,value.seconds);
        };
        std::cout<<std::format(
            "{} timeLawBetween total calls={} successes={} failures={} solver calls={} "
            "cache hits={} misses={} collisions={} materializations={} correction-pass calls={} "
            "measured={} s; categories "
            "(calls/successes/failures/solver/hits/misses/collisions/materializations/"
            "correction@seconds) "
            "exact-stop={} seed={} current={} cap={} bracket={}\n",
            label,total.calls,total.successes,total.failures,total.solverCalls,total.cacheHits,
            total.cacheMisses,total.cacheCollisions,total.cacheMaterializations,
            total.correctionPassCalls,total.seconds,category(diagnostics.exactStop),
            category(diagnostics.continuousSeed),category(diagnostics.stationCurrentVelocity),
            category(diagnostics.stationCapVelocity),
            category(diagnostics.stationVelocityBracket));
        const auto &endpoints=diagnostics.endpointFeasibility;
        std::cout<<std::format(
            "{} endpoint feasibility cached geometry={} candidate checks={} "
            "acceleration rejections={} jerk rejections={}\n",label,
            endpoints.cachedGeometryEndpoints,endpoints.candidateChecks,
            endpoints.accelerationRejections,endpoints.jerkRejections);
        const auto &passes=diagnostics.correctionPassLocality;
        std::size_t comparedPieces=0;
        std::size_t correctedPieces=0;
        std::size_t changedStations=0;
        std::size_t changedPieceTimings=0;
        std::size_t changedUncorrectedPieceTimings=0;
        std::size_t changedPieceTimingRuns=0;
        std::size_t reusablePieceTimings=0;
        std::size_t reusablePieceTimingRuns=0;
        std::size_t reusableEdgePieces=0;
        std::size_t maximumLeftPropagation=0;
        std::size_t maximumRightPropagation=0;
        std::size_t maximumNearestPropagation=0;
        std::size_t maximumChangedRun=0;
        std::size_t maximumReusableRun=0;
        for(const auto &pass:passes) {
            comparedPieces+=pass.pieceCount;
            correctedPieces+=pass.correctedPieces;
            changedStations+=pass.changedStations;
            changedPieceTimings+=pass.changedPieceTimings;
            changedUncorrectedPieceTimings+=pass.changedUncorrectedPieceTimings;
            changedPieceTimingRuns+=pass.changedPieceTimingRuns;
            reusablePieceTimings+=pass.bitExactReusablePieceTimings;
            reusablePieceTimingRuns+=pass.bitExactReusablePieceTimingRuns;
            reusableEdgePieces+=pass.bitExactReusablePrefixPieces
                +pass.bitExactReusableSuffixPieces;
            maximumLeftPropagation=std::max(
                maximumLeftPropagation,pass.leftPropagationPieces);
            maximumRightPropagation=std::max(
                maximumRightPropagation,pass.rightPropagationPieces);
            maximumNearestPropagation=std::max(maximumNearestPropagation,
                pass.maximumPropagationFromCorrectedPiece);
            maximumChangedRun=std::max(
                maximumChangedRun,pass.maximumChangedPieceTimingRun);
            maximumReusableRun=std::max(maximumReusableRun,
                pass.maximumBitExactReusablePieceTimingRun);
        }
        std::cout<<std::format(
            "{} correction locality replans={} compared pieces={} corrected pieces={} "
            "changed stations={} changed piece timings={} uncorrected changes={} "
            "changed runs={} max changed run={}; bit-exact reusable timings={} ({}%) "
            "reusable runs={} max reusable run={}; reusable edge pieces={} ({}%); "
            "max propagation left={} right={} nearest={}\n",
            label,passes.size(),comparedPieces,correctedPieces,changedStations,
            changedPieceTimings,changedUncorrectedPieceTimings,changedPieceTimingRuns,
            maximumChangedRun,reusablePieceTimings,
            comparedPieces?100.0*reusablePieceTimings/comparedPieces:0.0,
            reusablePieceTimingRuns,maximumReusableRun,reusableEdgePieces,
            comparedPieces?100.0*reusableEdgePieces/comparedPieces:0.0,
            maximumLeftPropagation,maximumRightPropagation,maximumNearestPropagation);
        const auto &replay=diagnostics.stationVisitReplay;
        std::cout<<std::format(
            "{} station replay shadow active visits={} comparable={} exact inputs={} "
            "exact outputs={} output mismatches={}; potential candidate evaluations={} "
            "endpoint checks={} timeLaw calls={} ({}% of correction calls) solver calls={} "
            "({}% of all solver calls) materializations={} timeLaw={} s visit={} s\n",
            label,replay.activeVisits,replay.comparableVisits,replay.exactInputMatches,
            replay.exactOutputMatches,replay.outputMismatches,
            replay.potentialCandidateEvaluations,replay.potentialEndpointChecks,
            replay.potentialTimeLawCalls,
            total.correctionPassCalls
                ?100.0*replay.potentialTimeLawCalls/total.correctionPassCalls:0.0,
            replay.potentialSolverCalls,
            total.solverCalls?100.0*replay.potentialSolverCalls/total.solverCalls:0.0,
            replay.potentialMaterializations,replay.potentialTimeLawSeconds,
            replay.potentialVisitSeconds);
    }

    void printCorrectionPassDetails(const std::string_view label,
            const ngc::TimeLawDiagnostics &diagnostics) {
        for(const auto &pass:diagnostics.correctionPassLocality)
            std::cout<<std::format(
                "{} correction pass={} pieces={} corrected={} corrected_range=[{},{}] "
                "changed_stations={} station_range=[{},{}] changed_timings={} "
                "uncorrected_changes={} changed_runs={} max_changed_run={} "
                "timing_range=[{},{}] reusable={} reusable_runs={} max_reusable_run={} "
                "reusable_edges=[{},{}] propagation=[left={} right={} nearest={}] "
                "max_delta=[v={} a={} duration={}]\n",
                label,pass.pass,pass.pieceCount,pass.correctedPieces,
                pass.firstCorrectedPiece,pass.lastCorrectedPiece,
                pass.changedStations,pass.firstChangedStation,pass.lastChangedStation,
                pass.changedPieceTimings,pass.changedUncorrectedPieceTimings,
                pass.changedPieceTimingRuns,pass.maximumChangedPieceTimingRun,
                pass.firstChangedPieceTiming,
                pass.lastChangedPieceTiming,pass.bitExactReusablePieceTimings,
                pass.bitExactReusablePieceTimingRuns,
                pass.maximumBitExactReusablePieceTimingRun,
                pass.bitExactReusablePrefixPieces,pass.bitExactReusableSuffixPieces,
                pass.leftPropagationPieces,pass.rightPropagationPieces,
                pass.maximumPropagationFromCorrectedPiece,
                pass.maximumVelocityChange,pass.maximumAccelerationChange,
                pass.maximumDurationChange);
    }

    void writePieceTimingSnapshot(const std::filesystem::path &modelPath,
                                  const std::vector<PlannedPieceTrace> &pieces) {
        auto path=modelPath;
        path.replace_extension("piece_timing.csv");
        std::ofstream output(path,std::ios::trunc);
        output.precision(17);
        output<<"horizon,source,length,programmed_cap,initial_cap,final_cap,"
            "entry_velocity,entry_acceleration,exit_velocity,exit_acceleration,duration,"
            "start_x,start_y,start_z,end_x,end_y,end_z\n";
        for(const auto &piece:pieces) {
            const auto &timing=piece.timing;
            output<<piece.horizon<<','<<piece.sourceInput<<','<<timing.length<<','
                <<timing.programmedVelocityLimit<<','<<timing.initialVelocityLimit<<','
                <<timing.velocityLimit<<','<<timing.entryVelocity<<','
                <<timing.entryAcceleration<<','<<timing.exitVelocity<<','
                <<timing.exitAcceleration<<','<<timing.duration<<','
                <<timing.startPosition.x<<','<<timing.startPosition.y<<','
                <<timing.startPosition.z<<','<<timing.endPosition.x<<','
                <<timing.endPosition.y<<','<<timing.endPosition.z<<'\n';
        }
        output.flush();
        if(!output) throw std::runtime_error("incomplete piece timing snapshot write");
    }

    ngc::position_t scaled(const ngc::position_t &value,const double amount) {
        return {value.x*amount,value.y*amount,value.z*amount,
            value.a*amount,value.b*amount,value.c*amount};
    }

    ngc::position_t spanPosition(const ngc::AxisPolynomialSpan &span,const double u) {
        return scaled(scaled(scaled(span.a,u)+span.b,u)+span.c,u)+span.d;
    }

    double spanXyzSpeed(const ngc::AxisPolynomialSpan &span,const double u) {
        const auto derivative=scaled(
            scaled(span.a,3.0*u*u)+scaled(span.b,2.0*u)+span.c,
            span.inverseDuration);
        return std::sqrt(derivative.x*derivative.x+derivative.y*derivative.y
            +derivative.z*derivative.z);
    }

    double spanXyzLength(const ngc::AxisPolynomialSpan &span) {
        auto length=0.0;
        auto previous=span.d;
        for(unsigned sample=1;sample<=8;++sample) {
            const auto current=spanPosition(span,static_cast<double>(sample)/8.0);
            const auto delta=current-previous;
            length+=std::sqrt(delta.x*delta.x+delta.y*delta.y+delta.z*delta.z);
            previous=current;
        }
        return length;
    }

    double commandFeed(const ngc::MachineCommand &command) {
        return std::visit([](const auto &value) {
            using T=std::decay_t<decltype(value)>;
            if constexpr(std::same_as<T,ngc::MoveLine>||std::same_as<T,ngc::MoveArc>)
                return value.speed()/60.0;
            else return 0.0;
        },command);
    }

    std::string_view velocityCauseName(const ngc::ContinuousVelocityLimitCause cause) {
        using enum ngc::ContinuousVelocityLimitCause;
        switch(cause) {
            case ProgrammedFeed:return "programmed-feed";
            case AxisVelocity:return "axis-velocity";
            case AxisCentripetalAcceleration:return "axis-centripetal";
            case PathCentripetalAcceleration:return "path-centripetal";
            case AxisCurvatureDerivativeJerk:return "axis-curvature-derivative";
            case PathCurvatureDerivativeJerk:return "path-curvature-derivative";
        }
        return "unknown";
    }

    void printSlowRuns(const std::vector<ExecutedSpanTrace> &traces) {
        struct SlowRun {
            std::size_t first=0;
            std::size_t last=0;
            std::size_t spans=0;
            double length=0.0;
            double duration=0.0;
            double minimumSpeed=std::numeric_limits<double>::infinity();
            double maximumSpeed=0.0;
        };
        std::vector<SlowRun> runs;
        for(std::size_t index=0;index<traces.size();) {
            if(traces[index].feed<=0.0
               ||traces[index].middleSpeed>=0.75*traces[index].feed) {
                ++index;
                continue;
            }
            SlowRun run{.first=index,.last=index};
            while(index<traces.size()&&traces[index].feed>0.0
                  &&traces[index].middleSpeed<0.75*traces[index].feed) {
                run.last=index;
                ++run.spans;
                run.length+=traces[index].length;
                run.duration+=traces[index].duration;
                run.minimumSpeed=std::min(run.minimumSpeed,traces[index].middleSpeed);
                run.maximumSpeed=std::max(run.maximumSpeed,traces[index].middleSpeed);
                ++index;
            }
            if(run.spans>=8) runs.push_back(run);
        }
        std::ranges::sort(runs,[](const SlowRun &left,const SlowRun &right) {
            return left.spans>right.spans;
        });
        std::cout<<std::format("slow span runs (midpoint XYZ speed <75% active feed; {} found):\n",
            runs.size());
        for(std::size_t rank=0;rank<std::min<std::size_t>(20,runs.size());++rank) {
            const auto &run=runs[rank];
            const auto &first=traces[run.first];
            const auto &last=traces[run.last];
            std::cout<<std::format(
                "slow rank={} horizon={} spans={} sources={}..{} length={} duration={} "
                "average={} midpoint_range=[{},{}] start=[{},{},{}] end=[{},{},{}]\n",
                rank+1,first.horizon,run.spans,first.sourceInput,last.sourceInput,
                run.length,run.duration,run.length/run.duration,run.minimumSpeed,run.maximumSpeed,
                first.start.x,first.start.y,first.start.z,last.end.x,last.end.y,last.end.z);
        }
    }

    void printLimitingPieces(std::vector<PlannedPieceTrace> pieces) {
        pieces.erase(std::remove_if(pieces.begin(),pieces.end(),[](const auto &piece) {
            const auto &timing=piece.timing;
            return timing.linear||timing.programmedVelocityLimit<=0.0||timing.duration<=0.0
                ||timing.length/timing.duration>=0.75*timing.programmedVelocityLimit;
        }),pieces.end());
        std::ranges::sort(pieces,[](const auto &left,const auto &right) {
            const auto excess=[](const auto &piece) {
                return piece.timing.duration
                    -piece.timing.length/piece.timing.programmedVelocityLimit;
            };
            return excess(left)>excess(right);
        });
        std::cout<<std::format("slow curved timing pieces ({} found):\n",pieces.size());
        for(std::size_t rank=0;rank<std::min<std::size_t>(30,pieces.size());++rank) {
            const auto &piece=pieces[rank];
            const auto &timing=piece.timing;
            const auto cause=timing.velocityLimit<0.98*timing.initialVelocityLimit
                ?"exact-correction":velocityCauseName(timing.initialVelocityLimitCause);
            std::cout<<std::format(
                "piece rank={} cause={} initial_cause={} horizon={} source={} length={} duration={} average={} "
                "caps=[programmed={} initial={} final={}] states=[{} -> {}] "
                "curvature_derivative=[analytic={} tangent={} normal={} curvature={} "
                "coarse={}@{} fine={}@{} s={}] "
                "start=[{},{},{}] end=[{},{},{}]\n",
                rank+1,cause,velocityCauseName(timing.initialVelocityLimitCause),
                piece.horizon,piece.sourceInput,timing.length,timing.duration,
                timing.length/timing.duration,timing.programmedVelocityLimit,
                timing.initialVelocityLimit,timing.velocityLimit,
                timing.entryVelocity,timing.exitVelocity,
                timing.curvatureDerivativeMagnitude,
                timing.curvatureDerivativeTangentialMagnitude,
                timing.curvatureDerivativeNormalMagnitude,
                timing.curvatureMagnitudeAtDerivativeSample,
                timing.curvatureDerivativeFiniteDifferenceCoarse,
                timing.curvatureDerivativeFiniteDifferenceCoarseStep,
                timing.curvatureDerivativeFiniteDifferenceFine,
                timing.curvatureDerivativeFiniteDifferenceFineStep,
                timing.curvatureDerivativeSampleDistance,
                timing.startPosition.x,timing.startPosition.y,timing.startPosition.z,
                timing.endPosition.x,timing.endPosition.y,timing.endPosition.z);
        }
    }

    void printSlowPieceRuns(const std::vector<PlannedPieceTrace> &pieces) {
        struct Run {
            std::size_t first=0;
            std::size_t last=0;
            std::size_t count=0;
            std::size_t derivativeLimited=0;
            double length=0.0;
            double duration=0.0;
            double minimumCap=std::numeric_limits<double>::infinity();
            double maximumCap=0.0;
        };
        const auto slow=[](const PlannedPieceTrace &piece) {
            const auto &timing=piece.timing;
            return !timing.linear&&timing.programmedVelocityLimit>0.0
                &&timing.initialVelocityLimit<0.75*timing.programmedVelocityLimit;
        };
        std::vector<Run> runs;
        for(std::size_t index=0;index<pieces.size();) {
            if(!slow(pieces[index])) {
                ++index;
                continue;
            }
            Run run{.first=index,.last=index};
            const auto horizon=pieces[index].horizon;
            while(index<pieces.size()&&pieces[index].horizon==horizon&&slow(pieces[index])) {
                const auto &timing=pieces[index].timing;
                run.last=index;
                ++run.count;
                run.length+=timing.length;
                run.duration+=timing.duration;
                run.minimumCap=std::min(run.minimumCap,timing.initialVelocityLimit);
                run.maximumCap=std::max(run.maximumCap,timing.initialVelocityLimit);
                if(timing.initialVelocityLimitCause
                    ==ngc::ContinuousVelocityLimitCause::PathCurvatureDerivativeJerk
                   ||timing.initialVelocityLimitCause
                    ==ngc::ContinuousVelocityLimitCause::AxisCurvatureDerivativeJerk)
                    ++run.derivativeLimited;
                ++index;
            }
            if(run.count>=3) runs.push_back(run);
        }
        std::ranges::sort(runs,[](const Run &left,const Run &right) {
            return left.length>right.length;
        });
        std::cout<<std::format("consecutive slow curved piece runs ({} found):\n",runs.size());
        for(std::size_t rank=0;rank<std::min<std::size_t>(20,runs.size());++rank) {
            const auto &run=runs[rank];
            const auto &first=pieces[run.first];
            const auto &last=pieces[run.last];
            std::cout<<std::format(
                "piece_run rank={} horizon={} pieces={} derivative_limited={} "
                "sources={}..{} length={} duration={} average={} caps=[{},{}] "
                "start=[{},{},{}] end=[{},{},{}]\n",
                rank+1,first.horizon,run.count,run.derivativeLimited,
                first.sourceInput,last.sourceInput,run.length,run.duration,
                run.length/run.duration,run.minimumCap,run.maximumCap,
                first.timing.startPosition.x,first.timing.startPosition.y,
                first.timing.startPosition.z,last.timing.endPosition.x,
                last.timing.endPosition.y,last.timing.endPosition.z);
        }
    }

    std::pair<std::string_view,ngc::ContinuousPlanningEffort> planningEffort(
            const std::string_view argument) {
        if(argument.empty()||argument=="--effort=current")
            return {"current",{}};
        if(argument=="--effort=replay-shadow")
            return {"replay-shadow",{
                .measureStationVisitReplay=true,
                .enableStationVisitReplay=false,
            }};
        if(argument=="--effort=replay")
            return {"replay",{
                .enableStationVisitReplay=true,
            }};
        if(argument=="--effort=no-replay")
            return {"no-replay",{
                .enableStationVisitReplay=false,
            }};
        if(argument=="--effort=global-time-cache")
            return {"global-time-cache",{
                .shareTimeLawCacheAcrossCompilations=true,
            }};
        if(argument=="--effort=local-time-cache")
            return {"local-time-cache",{
                .shareTimeLawCacheAcrossCompilations=false,
            }};
        if(argument=="--effort=derivative125")
            return {"derivative125",{
                .curvatureDerivativeVelocityCapMultiplier=1.25,
            }};
        if(argument=="--effort=derivative150")
            return {"derivative150",{
                .curvatureDerivativeVelocityCapMultiplier=1.5,
            }};
        if(argument=="--effort=derivative200")
            return {"derivative200",{
                .curvatureDerivativeVelocityCapMultiplier=2.0,
            }};
        if(argument=="--effort=derivative250")
            return {"derivative250",{
                .curvatureDerivativeVelocityCapMultiplier=2.5,
            }};
        if(argument=="--effort=derivative300")
            return {"derivative300",{
                .curvatureDerivativeVelocityCapMultiplier=3.0,
            }};
        if(argument=="--effort=no-derivative-cap")
            return {"no-derivative-cap",{
                .applyCurvatureDerivativeVelocityCap=false,
            }};
        if(argument=="--effort=velocity6")
            return {"velocity6",{
                .reachabilitySweeps=3,
                .minimumVelocitySearchIterations=6,
                .accelerationCandidates=6,
                .candidateBudgetMultiplier=8,
                .capLargeHorizonVelocitySearch=false,
            }};
        if(argument=="--effort=velocity8")
            return {"velocity8",{
                .reachabilitySweeps=3,
                .minimumVelocitySearchIterations=8,
                .accelerationCandidates=6,
                .candidateBudgetMultiplier=16,
                .capLargeHorizonVelocitySearch=false,
            }};
        if(argument=="--effort=velocity10")
            return {"velocity10",{
                .reachabilitySweeps=3,
                .minimumVelocitySearchIterations=10,
                .accelerationCandidates=6,
                .candidateBudgetMultiplier=16,
                .capLargeHorizonVelocitySearch=false,
            }};
        if(argument=="--effort=velocity12")
            return {"velocity12",{
                .reachabilitySweeps=3,
                .minimumVelocitySearchIterations=12,
                .accelerationCandidates=6,
                .candidateBudgetMultiplier=24,
                .capLargeHorizonVelocitySearch=false,
            }};
        if(argument=="--effort=combined")
            return {"combined",{
                .reachabilitySweeps=8,
                .minimumVelocitySearchIterations=8,
                .accelerationCandidates=6,
                .candidateBudgetMultiplier=64,
                .capLargeHorizonVelocitySearch=false,
            }};
        if(argument=="--effort=combined20")
            return {"combined20",{
                .reachabilitySweeps=20,
                .minimumVelocitySearchIterations=8,
                .accelerationCandidates=6,
                .candidateBudgetMultiplier=128,
                .capLargeHorizonVelocitySearch=false,
            }};
        if(argument=="--effort=combined40")
            return {"combined40",{
                .reachabilitySweeps=40,
                .minimumVelocitySearchIterations=8,
                .accelerationCandidates=6,
                .candidateBudgetMultiplier=256,
                .capLargeHorizonVelocitySearch=false,
            }};
        if(argument=="--effort=medium")
            return {"medium",{
                .reachabilitySweeps=8,
                .minimumVelocitySearchIterations=0,
                .accelerationCandidates=6,
                .candidateBudgetMultiplier=32,
                .capLargeHorizonVelocitySearch=true,
            }};
        if(argument=="--effort=high")
            return {"high",{
                .reachabilitySweeps=20,
                .minimumVelocitySearchIterations=0,
                .accelerationCandidates=6,
                .candidateBudgetMultiplier=128,
                .capLargeHorizonVelocitySearch=true,
            }};
        if(argument=="--effort=extreme")
            return {"extreme",{
                .reachabilitySweeps=40,
                .minimumVelocitySearchIterations=0,
                .accelerationCandidates=6,
                .candidateBudgetMultiplier=256,
                .capLargeHorizonVelocitySearch=true,
            }};
        throw std::runtime_error(
            "effort must be --effort=current, derivative125, derivative150, derivative200, "
            "derivative250, derivative300, "
            "no-derivative-cap, velocity6, velocity8, velocity10, velocity12, "
            "combined, combined20, combined40, medium, high, or extreme");
    }

    struct CompatibleWindow {
        std::vector<ngc::MachineCommand> commands;
        std::optional<double> blendScale;
    };

    CompatibleWindow interpret(const std::filesystem::path &path,ngc::Machine &machine) {
        const auto source=ngc::readFile(path);
        if(!source) throw std::runtime_error(source.error().what());
        ngc::Program program(*source,path.string());
        const auto compiled=program.compile();
        if(!compiled) throw std::runtime_error(compiled.error().text());

        CompatibleWindow result;
        bool collecting=false;
        bool finished=false;
        const std::function callback=[&](std::unique_ptr<const ngc::EvaluatorMessage> message,
                                         ngc::Evaluator &) {
            const auto *block=message->as<ngc::BlockMessage>();
            if(!block) return;
            auto emitted=machine.executeBlock(block->block());
            for(auto &command:emitted) {
                if(finished) continue;
                const auto compatible=std::visit([](const auto &value) {
                    using T=std::decay_t<decltype(value)>;
                    if constexpr(std::same_as<T,ngc::MoveLine>)
                        return value.speed()>0.0&&!value.machineCoordinates();
                    else if constexpr(std::same_as<T,ngc::MoveArc>) return value.speed()>0.0;
                    else return false;
                },command);
                const auto positiveG64=machine.state().modePath==ngc::GCPath::G64
                    &&machine.pathTolerance()&&*machine.pathTolerance()>0.0;
                if(!collecting) {
                    if(!compatible||!positiveG64) continue;
                    collecting=true;
                } else if(!compatible||!positiveG64) {
                    finished=true;
                    continue;
                }
                if(result.blendScale
                   &&std::abs(*result.blendScale-*machine.pathTolerance())>1e-12)
                    finished=true;
                if(finished) continue;
                result.blendScale=machine.pathTolerance();
                result.commands.push_back(std::move(command));
            }
        };

        ngc::Evaluator evaluator(machine.memory(),callback);
        const ngc::Preamble preamble(machine.memory());
        evaluator.executeFirstPass(preamble.statements());
        evaluator.executeFirstPass(program.statements());
        evaluator.executeSecondPass(program.statements());
        if(result.commands.size()<2||!result.blendScale)
            throw std::runtime_error("oracle export requires at least two compatible G64 motions");
        return result;
    }

    ngc::position_t startPosition(const ngc::MachineCommand &command) {
        return std::visit([](const auto &value) -> ngc::position_t {
            using T=std::decay_t<decltype(value)>;
            if constexpr(std::same_as<T,ngc::MoveLine>||std::same_as<T,ngc::MoveArc>)
                return value.from();
            else throw std::logic_error("validated oracle window contains non-motion command");
        },command);
    }

    void writeGeometrySnapshot(const std::filesystem::path &path,
            const std::span<const ngc::MachineCommand> commands,
            const std::span<const CapturedSplineTrace> splines) {
        std::ofstream output(path,std::ios::binary|std::ios::trunc);
        if(!output) throw std::runtime_error(std::format("could not open {}",path.string()));
        output.precision(17);
        output<<"ngc_spline_geometry_v2\n";
        output<<"primitive_count "<<commands.size()<<'\n';
        for(std::size_t input=0;input<commands.size();++input)
            std::visit([&](const auto &command) {
                using T=std::decay_t<decltype(command)>;
                if constexpr(std::same_as<T,ngc::MoveLine>) {
                    const auto &a=command.from();
                    const auto &b=command.to();
                    output<<"line "<<input<<' '<<command.speed()<<' '
                        <<a.x<<' '<<a.y<<' '<<a.z<<' '<<a.a<<' '<<a.b<<' '<<a.c<<' '
                        <<b.x<<' '<<b.y<<' '<<b.z<<' '<<b.a<<' '<<b.b<<' '<<b.c<<'\n';
                } else if constexpr(std::same_as<T,ngc::MoveArc>) {
                    const auto &a=command.from();
                    const auto &b=command.to();
                    const auto &center=command.center();
                    const auto &axis=command.axis();
                    output<<"arc "<<input<<' '<<command.speed()<<' '
                        <<a.x<<' '<<a.y<<' '<<a.z<<' '<<a.a<<' '<<a.b<<' '<<a.c<<' '
                        <<b.x<<' '<<b.y<<' '<<b.z<<' '<<b.a<<' '<<b.b<<' '<<b.c<<' '
                        <<center.x<<' '<<center.y<<' '<<center.z<<' '
                        <<axis.x<<' '<<axis.y<<' '<<axis.z<<'\n';
                }
            },commands[input]);
        output<<"spline_count "<<splines.size()<<'\n';
        for(std::size_t index=0;index<splines.size();++index) {
            const auto &spline=splines[index];
            output<<"spline "<<index<<' '<<spline.horizon<<' '
                <<spline.firstSource<<' '<<spline.lastSource<<' '
                <<spline.degree<<' '<<spline.controls.size()<<' '
                <<spline.pieceBoundaries.size()<<'\n';
            for(const auto &control:spline.controls)
                output<<"control "<<control.x<<' '<<control.y<<' '<<control.z<<' '
                    <<control.a<<' '<<control.b<<' '<<control.c<<'\n';
            output<<"boundaries";
            for(const auto boundary:spline.pieceBoundaries) output<<' '<<boundary;
            output<<'\n';
        }
        output.flush();
        if(!output) throw std::runtime_error(std::format(
            "incomplete geometry snapshot write to {}",path.string()));
    }
}

int main(const int argc,char **argv) {
    try {
        if(argc<3||argc>8) {
            std::cerr<<"usage: ngc_g64_oracle_export <program.ngc> <diagnostic-path> "
                "[machine.toml] [jerk-multiplier] [--rolling|--rolling-only] "
                "[--effort=current|replay|no-replay|replay-shadow|global-time-cache|local-time-cache|derivative125|derivative150|derivative200|derivative250|derivative300|no-derivative-cap|velocity6|velocity8|velocity10|velocity12|combined|combined20|combined40|medium|high|extreme] "
                "[--trace-slow]\n";
            return 2;
        }
        const auto rolling=argc>=6&&(std::string_view(argv[5])=="--rolling"
            ||std::string_view(argv[5])=="--rolling-only");
        const auto rollingOnly=argc>=6&&std::string_view(argv[5])=="--rolling-only";
        if(argc>=6&&!rolling)
            throw std::runtime_error("the final argument must be --rolling or --rolling-only");
        auto [effortName,effort]=planningEffort(argc>=7?argv[6]:std::string_view{});
        const auto traceSlow=argc==8&&std::string_view(argv[7])=="--trace-slow";
        effort.measureCurvatureDerivativeNumerics=traceSlow;
        effort.captureSplineGeometry=traceSlow;
        if(argc==8&&!traceSlow) throw std::runtime_error("the final argument must be --trace-slow");
        const std::filesystem::path sourcePath=argv[1];
        const std::filesystem::path diagnosticPath=argv[2];
        const std::filesystem::path configurationPath=argc>=4?argv[3]:"machine.toml";
        auto jerkMultiplier=1.0;
        if(argc>=5) {
            std::size_t consumed=0;
            jerkMultiplier=std::stod(argv[4],&consumed);
            if(consumed!=std::string_view(argv[4]).size()||!std::isfinite(jerkMultiplier)
               ||jerkMultiplier<=0.0)
                throw std::runtime_error("jerk multiplier must be a finite positive number");
        }
        if(jerkMultiplier>1.0) {
            // High-jerk sweeps can require more local proof/correction work.
            // Keep production defaults unchanged and retain finite offline
            // ceilings for these explicit exporter experiments.
            // adaptive.ngc converged at 35 passes and 266,655 cumulative
            // attempts across 2,893 pieces (about 92.17 per piece). These
            // ceilings retain 37% pass and 12.8% geometry headroom.
            effort.maximumLocalCorrectionPasses=48;
            effort.geometryVerificationBudgetMultiplier=104;
            if(jerkMultiplier>=100.0) {
                // The much larger adaptive_pockets.ngc 100x horizon still
                // corrects real emitted acceleration violations at pass 47.
                // Use the library's bounded offline maximum for this extreme
                // sweep; failure remains fatal at either ceiling.
                effort.maximumLocalCorrectionPasses=128;
                effort.geometryVerificationBudgetMultiplier=256;
            }
        }
        const auto configuration=ngc::loadMachineConfiguration(configurationPath);
        if(!configuration) throw std::runtime_error(configuration.error());

        ngc::Machine machine(configuration->unit);
        auto window=interpret(sourcePath,machine);
        auto trajectoryLimits=configuration->trajectory;
        trajectoryLimits.pathJerk*=jerkMultiplier;
        for(auto component:{&ngc::position_t::x,&ngc::position_t::y,&ngc::position_t::z,
                            &ngc::position_t::a,&ngc::position_t::b,&ngc::position_t::c})
            if(std::isfinite(trajectoryLimits.axisJerk.*component))
                trajectoryLimits.axisJerk.*component*=jerkMultiplier;
        const auto runRolling=[&](const std::optional<double> baselineDuration) {
            ngc::TrajectoryPlanner rollingPlanner(trajectoryLimits);
            rollingPlanner.setContinuousPlanningEffort(effort);
            rollingPlanner.reset(2,startPosition(window.commands.front()));
            std::size_t horizons=0;
            std::size_t chunks=0;
            std::size_t nextSourceInput=0;
            std::optional<std::size_t> activeSourceInput;
            std::vector<ExecutedSpanTrace> executedSpans;
            std::size_t diagnosticHorizon=0;
            std::size_t diagnosticNextSourceInput=0;
            std::optional<std::size_t> diagnosticActiveSourceInput;
            std::vector<PlannedPieceTrace> plannedPieces;
            std::vector<CapturedSplineTrace> capturedSplines;
            if(traceSlow) rollingPlanner.setContinuousDiagnosticCallback(
                [&](const ngc::ContinuousTrajectoryPlan &plan,
                    const std::span<const ngc::TrajectoryPlannerInput> inputs) {
                    ++diagnosticHorizon;
                    std::vector<std::optional<std::size_t>> sources;
                    sources.reserve(inputs.size());
                    for(const auto &input:inputs) {
                        if(input.presentationActivation)
                            diagnosticActiveSourceInput=diagnosticNextSourceInput++;
                        sources.push_back(diagnosticActiveSourceInput);
                    }
                    for(const auto &timing:plan.pieceTiming) {
                        if(timing.input>=sources.size()||!sources[timing.input]) continue;
                        plannedPieces.push_back({
                            .horizon=diagnosticHorizon,
                            .sourceInput=*sources[timing.input],
                            .timing=timing,
                        });
                    }
                    for(const auto &spline:plan.splineGeometry) {
                        if(spline.firstInput>=sources.size()||spline.lastInput>=sources.size()
                           ||!sources[spline.firstInput]||!sources[spline.lastInput]) continue;
                        capturedSplines.push_back({
                            .horizon=diagnosticHorizon,
                            .firstSource=*sources[spline.firstInput],
                            .lastSource=*sources[spline.lastInput],
                            .degree=spline.degree,
                            .controls=spline.controls,
                            .pieceBoundaries=spline.pieceBoundaries,
                        });
                    }
                });
            const auto planAndRecord=[&](const bool allowTerminalStop) {
                const auto horizonStart=std::chrono::steady_clock::now();
                auto horizon=rollingPlanner.planWindow(allowTerminalStop);
                const auto horizonEnd=std::chrono::steady_clock::now();
                if(!horizon) throw std::runtime_error(horizon.error());
                if(!*horizon) return false;
                ++horizons;
                chunks+=(*horizon)->items.size();
                auto horizonDuration=0.0;
                for(const auto &item:(*horizon)->items)
                    if(const auto *chunk=std::get_if<ngc::PlanChunk>(&item))
                        for(const auto &span:chunk->normalMotion) horizonDuration+=span.duration;
                const auto sourceInputs=std::ranges::count_if(
                    (*horizon)->inputs,[](const auto &input) {
                        return input.presentationActivation;
                    });
                if(traceSlow) {
                    for(const auto &item:(*horizon)->items) {
                        const auto *chunk=std::get_if<ngc::PlanChunk>(&item);
                        if(!chunk) continue;
                        for(const auto &span:chunk->normalMotion) {
                            for(std::size_t input=0;input<(*horizon)->inputs.size();++input) {
                                if(!(*horizon)->inputs[input].presentationActivation
                                   ||(*horizon)->activationSpans[input]!=span.id) continue;
                                activeSourceInput=nextSourceInput++;
                            }
                            if(!activeSourceInput||*activeSourceInput>=window.commands.size())
                                continue;
                            executedSpans.push_back({
                                .horizon=horizons,
                                .sourceInput=*activeSourceInput,
                                .start=span.d,
                                .end=span.end.position,
                                .length=spanXyzLength(span),
                                .duration=span.duration,
                                .middleSpeed=spanXyzSpeed(span,0.5),
                                .feed=commandFeed(window.commands[*activeSourceInput]),
                            });
                        }
                    }
                }
                std::cout<<std::format(
                    "rolling horizon={} source_inputs={} chunks={} duration={} s calculation={} s\n",
                    horizons,sourceInputs,(*horizon)->items.size(),horizonDuration,
                    std::chrono::duration<double>(horizonEnd-horizonStart).count());
                return true;
            };
            for(const auto &command:window.commands) {
                if(!rollingPlanner.enqueue({
                    command,
                    {ngc::ExecutablePathMode::Continuous,window.blendScale},
                    {},
                })) throw std::runtime_error("rolling planner rejected an oracle window input");
                if(rollingPlanner.shouldPlanRollingPrefix())
                    (void)planAndRecord(false);
            }
            while(rollingPlanner.windowSize()>0) {
                if(!planAndRecord(true))
                    throw std::runtime_error("rolling planner produced no terminal horizon");
            }
            const auto &diagnostics=rollingPlanner.diagnostics();
            const auto delta=baselineDuration
                ?diagnostics.plannedDuration-*baselineDuration:0.0;
            const auto percent=baselineDuration
                ?100.0*(diagnostics.plannedDuration/ *baselineDuration-1.0):0.0;
            std::cout<<std::format(
                "rolling effort={}; lookahead={} s; horizons={}; chunks={}; duration={} s; "
                "duration delta={} s ({}%); first horizon calculation={} s; "
                "last horizon calculation={} s; "
                "maximum horizon calculation={} s; published horizon calculation={} s; "
                "rolling search calculation={} s; total planning calculation={} s; "
                "boundary candidates={} suffix failures={} prefix failures={}; "
                "published spline inverse queries={} endpoints={} exact cache hits={} "
                "construction integrals={} inverse integrals={} Newton iterations={}; "
                "published arc inverse queries={} endpoints={} exact cache hits={} "
                "construction integrals={} inverse integrals={} Newton iterations={}{}\n",
                effortName,trajectoryLimits.lookaheadDuration,horizons,chunks,
                diagnostics.plannedDuration,
                delta,percent,diagnostics.firstContinuousHorizonSeconds,
                diagnostics.lastContinuousHorizonSeconds,
                diagnostics.maximumContinuousHorizonSeconds,
                diagnostics.totalContinuousHorizonSeconds,
                diagnostics.rollingSearchSeconds,
                diagnostics.totalPlanningSeconds,
                diagnostics.rollingBoundaryCandidates,
                diagnostics.rollingSuffixProbeFailures,
                diagnostics.rollingPrefixProbeFailures,
                diagnostics.publishedSplineInverse.queries,
                diagnostics.publishedSplineInverse.endpointQueries,
                diagnostics.publishedSplineInverse.exactCacheHits,
                diagnostics.publishedSplineInverse.constructionIntegralEvaluations,
                diagnostics.publishedSplineInverse.inverseIntegralEvaluations,
                diagnostics.publishedSplineInverse.newtonIterations,
                diagnostics.publishedArcInverse.queries,
                diagnostics.publishedArcInverse.endpointQueries,
                diagnostics.publishedArcInverse.exactCacheHits,
                diagnostics.publishedArcInverse.constructionIntegralEvaluations,
                diagnostics.publishedArcInverse.inverseIntegralEvaluations,
                diagnostics.publishedArcInverse.newtonIterations,
                rollingPlanner.lastRollingCandidateError().empty()?std::string{}:
                    std::format("; last failure={}",rollingPlanner.lastRollingCandidateError()));
            printTimeLawDiagnostics("rolling all attempts",diagnostics.timeLaw);
            printTimeLawDiagnostics("rolling published",diagnostics.publishedTimeLaw);
            printTimeLawDiagnostics("rolling prefix probes",
                diagnostics.rollingPrefixProbeTimeLaw);
            printTimeLawDiagnostics("rolling suffix probes",
                diagnostics.rollingSuffixProbeTimeLaw);
            if(traceSlow) {
                writeGeometrySnapshot(diagnosticPath,window.commands,capturedSplines);
                writePieceTimingSnapshot(diagnosticPath,plannedPieces);
                printSlowRuns(executedSpans);
                printSlowPieceRuns(plannedPieces);
                printLimitingPieces(std::move(plannedPieces));
            }
        };
        if(rollingOnly) {
            runRolling(std::nullopt);
            return 0;
        }
        ngc::TrajectoryCompiler planner(trajectoryLimits);
        planner.setContinuousPlanningEffort(effort);
        planner.reset(1,startPosition(window.commands.front()));
        ngc::InfiniteJerkTrajectoryTimeResult infiniteJerkTime;
        const auto planningStart=std::chrono::steady_clock::now();
        const auto plan=planner.compileContinuous(window.commands,*window.blendScale,
            std::nullopt,std::nullopt,{},12U,&infiniteJerkTime);
        const auto planningEnd=std::chrono::steady_clock::now();
        if(!plan) throw std::runtime_error(plan.error());
        auto plannerDuration=0.0;
        for(const auto &piece:(*plan)->pieceTiming) plannerDuration+=piece.duration;
        std::size_t normalSpans=0;
        for(const auto &chunk:(*plan)->chunks) normalSpans+=chunk.normalMotion.size;
        std::cout<<std::format(
            "planned {} motions into {} verified spans; "
            "planner duration={} s; "
            "smoothed-path infinite-jerk duration={} s (gap={} s, {}%); "
            "infinite-jerk last-refinement delta={} s intervals={} refinements={}; "
            "velocity-only seed={} s; acceleration-aware improvement={} s; "
            "selected Ruckig brake phases={}; "
            "reachability candidate evaluations={}; geometry verification attempts={} "
            "high-water={}; spline inverse queries={} endpoints={} exact cache hits={} "
            "construction integrals={} "
            "inverse integrals={} Newton iterations={} seed convergences={} bisections={} "
            "iteration-limit hits={} maximum iterations={}; arc inverse queries={} endpoints={} "
            "exact cache hits={} construction integrals={} inverse integrals={} Newton iterations={} "
            "seed convergences={} bisections={} iteration-limit hits={} maximum iterations={}; "
            "plan calculation wall={} s\n",
            window.commands.size(),normalSpans,plannerDuration,
            infiniteJerkTime.duration,plannerDuration-infiniteJerkTime.duration,
            100.0*(plannerDuration/infiniteJerkTime.duration-1.0),
            infiniteJerkTime.estimatedDurationError,infiniteJerkTime.intervals,
            infiniteJerkTime.refinements,
            (*plan)->velocityOnlySeedDuration,
            (*plan)->velocityOnlySeedDuration-(*plan)->accelerationAwareDuration,
            (*plan)->ruckigBrakePhases,
            (*plan)->reachabilityCandidateEvaluations,
            (*plan)->geometryVerificationAttempts,(*plan)->geometryVerificationHighWater,
            (*plan)->splineInverse.queries,(*plan)->splineInverse.endpointQueries,
            (*plan)->splineInverse.exactCacheHits,
            (*plan)->splineInverse.constructionIntegralEvaluations,
            (*plan)->splineInverse.inverseIntegralEvaluations,
            (*plan)->splineInverse.newtonIterations,(*plan)->splineInverse.seedConvergences,
            (*plan)->splineInverse.safeguardedBisections,
            (*plan)->splineInverse.iterationLimitHits,
            (*plan)->splineInverse.maximumNewtonIterations,
            (*plan)->arcInverse.queries,(*plan)->arcInverse.endpointQueries,
            (*plan)->arcInverse.exactCacheHits,
            (*plan)->arcInverse.constructionIntegralEvaluations,
            (*plan)->arcInverse.inverseIntegralEvaluations,
            (*plan)->arcInverse.newtonIterations,(*plan)->arcInverse.seedConvergences,
            (*plan)->arcInverse.safeguardedBisections,
            (*plan)->arcInverse.iterationLimitHits,
            (*plan)->arcInverse.maximumNewtonIterations,
            std::chrono::duration<double>(planningEnd-planningStart).count());
        printTimeLawDiagnostics("full horizon",(*plan)->timeLaw);
        printCorrectionPassDetails("full horizon",(*plan)->timeLaw);
        if(rolling) runRolling(plannerDuration);
        return 0;
    } catch(const std::exception &error) {
        std::cerr<<"ERROR: "<<error.what()<<'\n';
        return 1;
    }
}
