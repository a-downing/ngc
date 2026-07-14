#include <cmath>
#include <filesystem>
#include <format>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "evaluator/Evaluator.h"
#include "evaluator/Preamble.h"
#include "machine/ExactStopTrajectoryPlanner.h"
#include "machine/Machine.h"
#include "machine/MachineConfiguration.h"
#include "parser/Program.h"
#include "utils.h"

namespace {
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
        const std::function callback=[&](std::unique_ptr<const ngc::EvaluatorMessage> message,
                                         ngc::Evaluator &) {
            const auto *block=message->as<ngc::BlockMessage>();
            if(!block) return;
            auto emitted=machine.executeBlock(block->block());
            for(auto &command:emitted) {
                const auto compatible=std::visit([](const auto &value) {
                    using T=std::decay_t<decltype(value)>;
                    if constexpr(std::same_as<T,ngc::MoveLine>)
                        return value.speed()>0.0&&!value.machineCoordinates();
                    else if constexpr(std::same_as<T,ngc::MoveArc>) return value.speed()>0.0;
                    else return false;
                },command);
                if(!compatible)
                    throw std::runtime_error(std::format(
                        "{} is not one compatible G64 feed line/arc horizon",path.string()));
                if(machine.state().modePath!=ngc::GCPath::G64||!machine.pathTolerance()
                   ||*machine.pathTolerance()<=0.0)
                    throw std::runtime_error(std::format(
                        "motion emitted outside positive-P G64 mode in {}",path.string()));
                if(result.blendScale
                   &&std::abs(*result.blendScale-*machine.pathTolerance())>1e-12)
                    throw std::runtime_error(std::format(
                        "G64 P changes inside the oracle horizon in {}",path.string()));
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

    void writeModel(const std::filesystem::path &path,
                    const ngc::ContinuousAccelerationOracleModel &model) {
        std::ofstream output(path,std::ios::binary|std::ios::trunc);
        if(!output) throw std::runtime_error(std::format("could not open {}",path.string()));
        output.precision(17);
        output<<"ngc_clarabel_acceleration_oracle_v1\n";
        output<<"path_acceleration "<<model.pathAcceleration<<'\n';
        output<<"axis_acceleration "<<model.axisAcceleration.x<<' '
              <<model.axisAcceleration.y<<' '<<model.axisAcceleration.z<<' '
              <<model.axisAcceleration.a<<' '<<model.axisAcceleration.b<<' '
              <<model.axisAcceleration.c<<'\n';
        output<<"planner_duration "<<model.plannerDuration<<'\n';
        output<<"segment_count "<<model.segments.size()<<'\n';
        for(const auto &segment:model.segments) {
            output<<"segment "<<segment.piece<<' '<<segment.input<<' '
                  <<segment.length<<' '<<segment.velocityLimit<<' '
                  <<segment.tangent.x<<' '<<segment.tangent.y<<' '
                  <<segment.tangent.z<<' '<<segment.tangent.a<<' '
                  <<segment.tangent.b<<' '<<segment.tangent.c<<' '
                  <<segment.curvature.x<<' '<<segment.curvature.y<<' '
                  <<segment.curvature.z<<' '<<segment.curvature.a<<' '
                  <<segment.curvature.b<<' '<<segment.curvature.c<<'\n';
        }
        output.flush();
        if(!output) throw std::runtime_error(std::format("incomplete write to {}",path.string()));
    }
}

int main(const int argc,char **argv) {
    try {
        if(argc<3||argc>4) {
            std::cerr<<"usage: ngc_g64_oracle_export <program.ngc> <model.txt> [machine.toml]\n";
            return 2;
        }
        const std::filesystem::path sourcePath=argv[1];
        const std::filesystem::path modelPath=argv[2];
        const std::filesystem::path configurationPath=argc==4?argv[3]:"machine.toml";
        const auto configuration=ngc::loadMachineConfiguration(configurationPath);
        if(!configuration) throw std::runtime_error(configuration.error());

        ngc::Machine machine(configuration->unit);
        auto window=interpret(sourcePath,machine);
        ngc::ExactStopTrajectoryPlanner planner(configuration->trajectory);
        planner.reset(1,startPosition(window.commands.front()));
        ngc::ContinuousAccelerationOracleModel model;
        const auto plan=planner.compileContinuous(window.commands,*window.blendScale,&model);
        if(!plan) throw std::runtime_error(plan.error());
        writeModel(modelPath,model);
        std::cout<<std::format(
            "exported {} oracle segments from {} motions; planner duration={} s\n",
            model.segments.size(),window.commands.size(),model.plannerDuration);
        return 0;
    } catch(const std::exception &error) {
        std::cerr<<"ERROR: "<<error.what()<<'\n';
        return 1;
    }
}
