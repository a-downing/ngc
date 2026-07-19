#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <print>
#include <string>
#include <string_view>
#include <thread>
#include <tuple>
#include <vector>

#include "SimulationWorker.h"
#include "machine/MachineConfiguration.h"
#include "machine/ToolTable.h"
#include "utils.h"

namespace {
    const char *statusName(const ngc::SimulationStatus status) {
        switch(status) {
            case ngc::SimulationStatus::Stopped: return "Stopped";
            case ngc::SimulationStatus::Running: return "Running";
            case ngc::SimulationStatus::Paused: return "Paused";
            case ngc::SimulationStatus::Completed: return "Completed";
            case ngc::SimulationStatus::Error: return "Error";
        }
        return "Unknown";
    }

    const char *backendName(const ngc::BackendState state) {
        switch(state) {
            case ngc::BackendState::Disabled: return "Disabled";
            case ngc::BackendState::Held: return "Held";
            case ngc::BackendState::Running: return "Running";
            case ngc::BackendState::Faulted: return "Faulted";
        }
        return "Unknown";
    }

    std::expected<std::string,std::string> load(const std::filesystem::path &path) {
        auto source=ngc::readFile(path);
        if(!source) return std::unexpected(
            "failed to read "+path.string()+": "+source.error().what());
        return std::move(*source);
    }

    const char *smootherName(const ngc::spline_detail::SplineFitSolver solver) {
        using ngc::spline_detail::SplineFitSolver;
        switch(solver) {
            case SplineFitSolver::None: return "none";
            case SplineFitSolver::CoordinateSearch: return "coordinate";
            case SplineFitSolver::UniformBandedFairness: return "uniform";
            case SplineFitSolver::PeakTargetedBandedFairness: return "peak-targeted";
            case SplineFitSolver::VelocityTargetedBandedFairness: return "velocity-targeted";
        }
        return "unknown";
    }

    std::expected<ngc::spline_detail::SplineFitSolver,std::string> parseSmoother(
            std::string_view argument) {
        constexpr std::string_view prefix="--smoother=";
        if(!argument.starts_with(prefix))
            return std::unexpected("smoother option must use --smoother=<name>");
        argument.remove_prefix(prefix.size());
        using ngc::spline_detail::SplineFitSolver;
        if(argument=="none") return SplineFitSolver::None;
        if(argument=="coordinate") return SplineFitSolver::CoordinateSearch;
        if(argument=="uniform") return SplineFitSolver::UniformBandedFairness;
        if(argument=="peak-targeted") return SplineFitSolver::PeakTargetedBandedFairness;
        if(argument=="velocity-targeted")
            return SplineFitSolver::VelocityTargetedBandedFairness;
        return std::unexpected(
            "unknown smoother; expected none, coordinate, uniform, peak-targeted, or "
            "velocity-targeted");
    }
}

int main(const int argc,char **argv) {
    if(argc>7) {
        std::println(stderr,"usage: ngc_simulation_diagnostic [program.ngc] "
            "[maximum-program-seconds] [tick-multiplier] "
            "[--smoother=none|coordinate|uniform|peak-targeted|velocity-targeted] "
            "[--scp-reachability-rows=on|off] [--scp-basis-reuse=on|off]");
        return 2;
    }
    const std::filesystem::path program=argc>1?argv[1]:"adaptive_pockets.ngc";
    const auto maximumProgramSeconds=argc>2?std::strtod(argv[2],nullptr):60.0;
    const auto multiplier=argc>3?std::atoi(argv[3]):10;
    auto smoother=ngc::spline_detail::continuousSplineFitSolver();
    auto scpReachabilityRows=false;
    auto scpBasisReuse=ngc::ContinuousPlanningEffort{}.reuseScpBasis;
    for(auto argument=4;argument<argc;++argument) {
        const auto option=std::string_view{argv[argument]};
        if(option.starts_with("--smoother=")) {
            const auto parsed=parseSmoother(option);
            if(!parsed) {
                std::println(stderr,"{}",parsed.error());
                return 2;
            }
            smoother=*parsed;
        } else if(option=="--scp-reachability-rows=on") {
            scpReachabilityRows=true;
        } else if(option=="--scp-reachability-rows=off") {
            scpReachabilityRows=false;
        } else if(option=="--scp-basis-reuse=on") {
            scpBasisReuse=true;
        } else if(option=="--scp-basis-reuse=off") {
            scpBasisReuse=false;
        } else {
            std::println(stderr,"unknown diagnostic option: {}",option);
            return 2;
        }
    }

    auto configuration=ngc::loadMachineConfiguration("machine.toml");
    if(!configuration) {
        std::println(stderr,"machine configuration error: {}",configuration.error());
        return 1;
    }
    ngc::ToolTable tools;
    if(auto loaded=tools.load();!loaded) {
        std::println(stderr,"tool table error: {}",loaded.error());
        return 1;
    }

    std::vector<std::tuple<std::string,std::string>> programs;
    for(const auto &path:std::array{
            std::filesystem::path{"autoload/hello.ngc"},
            std::filesystem::path{"autoload/world.ngc"},
            std::filesystem::path{"autoload/tool_change.ngc"},program}) {
        auto source=load(path);
        if(!source) {
            std::println(stderr,"{}",source.error());
            return 1;
        }
        programs.emplace_back(std::move(*source),path.string());
    }

    SimulationWorker worker(*configuration);
    worker.setTickMultiplier(multiplier);
    if(!worker.setSplineFitSolver(smoother)) {
        std::println(stderr,"simulation worker rejected the smoother selection");
        return 1;
    }
    ngc::ContinuousPlanningEffort planningEffort;
    planningEffort.addScpAdjacentReachabilityRows=scpReachabilityRows;
    planningEffort.reuseScpBasis=scpBasisReuse;
    if(!worker.setContinuousPlanningEffort(planningEffort)) {
        std::println(stderr,"simulation worker rejected the planning-effort selection");
        return 1;
    }
    if(!worker.start(programs,tools,true)) {
        std::println(stderr,"simulation worker rejected the run");
        return 1;
    }

    std::println("program={} multiplier={} stop_after={:.3f}s smoother={} "
        "scp_reachability_rows={} scp_basis_reuse={}",program.string(),multiplier,
        maximumProgramSeconds,smootherName(smoother),scpReachabilityRows,scpBasisReuse);
    ngc::EpochId lastEpoch=0;
    ngc::ChunkId lastChunk=0;
    ngc::SpanId lastSpan=0;
    std::string lastPlan;
    auto lastReport=std::chrono::steady_clock::now();
    for(;;) {
        const auto snapshot=worker.snapshot();
        if(snapshot.trajectoryContinuousPlanSummary!=lastPlan) {
            lastPlan=snapshot.trajectoryContinuousPlanSummary;
            if(!lastPlan.empty()) {
                std::println("PLAN {}",lastPlan);
                std::println("CORRECTIONS {}",
                    snapshot.trajectoryContinuousCorrectionHistory.empty()
                        ?std::string{"<none>"}
                        :snapshot.trajectoryContinuousCorrectionHistory);
            }
        }
        if(snapshot.trajectoryBackendEpoch!=lastEpoch
           ||snapshot.trajectoryBackendChunk!=lastChunk
           ||snapshot.trajectoryBackendSpan!=lastSpan) {
            lastEpoch=snapshot.trajectoryBackendEpoch;
            lastChunk=snapshot.trajectoryBackendChunk;
            lastSpan=snapshot.trajectoryBackendSpan;
            const auto block=snapshot.activeBlocks.empty()
                ?std::string{"<none>"}:snapshot.activeBlocks.back().text;
            std::println(
                "t={:.6f} backend={} epoch={} chunk={} span={} progress={:.6f} "
                "v={:.9g} a={:.9g} committed={:.6f}s active={:.6f}s queued={:.6f}s "
                "stop={:.6f}s queued_items={} xyz=[{:.9g},{:.9g},{:.9g}] block='{}'\n"
                "  span_detail={}\n  driver={}\n  latest_plan={}",
                snapshot.programElapsedSeconds,backendName(snapshot.trajectoryBackendState),
                lastEpoch,lastChunk,lastSpan,snapshot.trajectoryBackendSpanProgress,
                snapshot.trajectoryBackendVelocity,snapshot.trajectoryBackendAcceleration,
                snapshot.trajectoryBackendCommittedNormalSeconds,
                snapshot.trajectoryBackendActiveNormalRemainingSeconds,
                snapshot.trajectoryBackendQueuedNormalSeconds,
                snapshot.trajectoryBackendStopBranchSeconds,
                snapshot.trajectoryBackendQueuedExecutionItems,
                snapshot.machinePosition.x,snapshot.machinePosition.y,snapshot.machinePosition.z,
                block,snapshot.trajectoryBackendSpanDetail,
                snapshot.trajectoryDriverActivity,
                snapshot.trajectoryContinuousPlanSummary);
        }
        const auto now=std::chrono::steady_clock::now();
        if(now-lastReport>=std::chrono::seconds(5)) {
            lastReport=now;
            std::println("progress t={:.3f}s ticks={} status={} committed={:.6f}s "
                "active={:.6f}s queued={:.6f}s queued_items={} driver={}",
                snapshot.programElapsedSeconds,snapshot.servoTicks,
                statusName(snapshot.status),snapshot.trajectoryBackendCommittedNormalSeconds,
                snapshot.trajectoryBackendActiveNormalRemainingSeconds,
                snapshot.trajectoryBackendQueuedNormalSeconds,
                snapshot.trajectoryBackendQueuedExecutionItems,
                snapshot.trajectoryDriverActivity);
        }
        if(snapshot.status==ngc::SimulationStatus::Completed
           ||snapshot.status==ngc::SimulationStatus::Error
           ||snapshot.programElapsedSeconds>=maximumProgramSeconds) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    worker.stop();
    worker.join();
    const auto finalSnapshot=worker.snapshot();
    const auto geometrySeconds=finalSnapshot.geometryStream.preparationSeconds;
    const auto plannerSeconds=finalSnapshot.trajectoryPlanning.totalPlanningSeconds;
    const auto averageWindowSeconds=finalSnapshot.trajectoryPlanning.continuousHorizons==0
        ?0.0:finalSnapshot.trajectoryPlanning.totalContinuousHorizonSeconds
            /static_cast<double>(finalSnapshot.trajectoryPlanning.continuousHorizons);
    std::println("final status={} t={:.6f}s planned={:.6f}s "
        "geometry_processing={:.6f}s planner_processing={:.6f}s "
        "best_window={:.6f}s average_window={:.6f}s worst_window={:.6f}s "
        "total_processing={:.6f}s ticks={} error='{}'",
        statusName(finalSnapshot.status),finalSnapshot.programElapsedSeconds,
        finalSnapshot.trajectoryPlanning.plannedDuration,geometrySeconds,plannerSeconds,
        finalSnapshot.trajectoryPlanning.minimumContinuousHorizonSeconds,
        averageWindowSeconds,
        finalSnapshot.trajectoryPlanning.maximumContinuousHorizonSeconds,
        geometrySeconds+plannerSeconds,finalSnapshot.servoTicks,finalSnapshot.error);
    return 0;
}
