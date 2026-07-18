#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <print>
#include <string>
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
}

int main(const int argc,char **argv) {
    const std::filesystem::path program=argc>1?argv[1]:"adaptive_pockets.ngc";
    const auto maximumProgramSeconds=argc>2?std::strtod(argv[2],nullptr):60.0;
    const auto multiplier=argc>3?std::atoi(argv[3]):10;

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
    if(!worker.start(programs,tools,true)) {
        std::println(stderr,"simulation worker rejected the run");
        return 1;
    }

    std::println("program={} multiplier={} stop_after={:.3f}s",
        program.string(),multiplier,maximumProgramSeconds);
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
                "v={:.9g} a={:.9g} xyz=[{:.9g},{:.9g},{:.9g}] block='{}'\n"
                "  span_detail={}\n  driver={}\n  latest_plan={}",
                snapshot.programElapsedSeconds,backendName(snapshot.trajectoryBackendState),
                lastEpoch,lastChunk,lastSpan,snapshot.trajectoryBackendSpanProgress,
                snapshot.trajectoryBackendVelocity,snapshot.trajectoryBackendAcceleration,
                snapshot.machinePosition.x,snapshot.machinePosition.y,snapshot.machinePosition.z,
                block,snapshot.trajectoryBackendSpanDetail,
                snapshot.trajectoryDriverActivity,
                snapshot.trajectoryContinuousPlanSummary);
        }
        const auto now=std::chrono::steady_clock::now();
        if(now-lastReport>=std::chrono::seconds(5)) {
            lastReport=now;
            std::println("progress t={:.3f}s ticks={} status={} driver={}",
                snapshot.programElapsedSeconds,snapshot.servoTicks,
                statusName(snapshot.status),snapshot.trajectoryDriverActivity);
        }
        if(snapshot.status==ngc::SimulationStatus::Completed
           ||snapshot.status==ngc::SimulationStatus::Error
           ||snapshot.programElapsedSeconds>=maximumProgramSeconds) {
            std::println("final status={} t={:.6f}s ticks={} error='{}'",
                statusName(snapshot.status),snapshot.programElapsedSeconds,
                snapshot.servoTicks,snapshot.error);
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    worker.stop();
    worker.join();
    return 0;
}
