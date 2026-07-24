#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <exception>
#include <expected>
#include <format>
#include <fstream>
#include <filesystem>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <numbers>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "evaluator/Evaluator.h"
#include "evaluator/InterpreterSession.h"
#include "evaluator/Preamble.h"
#include "machine/Machine.h"
#include "machine/MachineConfiguration.h"
#include "machine/ArcInterpolation.h"
#include "machine/TrajectoryCompiler.h"
#include "machine/MockMotionBackend.h"
#include "machine/OwningSpscChannel.h"
#include "machine/PreparedGeometry.h"
#include "machine/SpscChannel.h"
#include "machine/SplineHandleOptimization.h"
#include "machine/ToolTable.h"
#include "memory/Memory.h"
#include "parser/Program.h"
#include "path_tempo/Planner.h"
#include "path_tempo/Types.h"
#include "SimulationWorker.h"
#include "Worker.h"

namespace {
    constexpr double EPSILON = 1e-9;
    constexpr ngc::Machine::Unit UNIT = ngc::Machine::Unit::Inch;

    constexpr std::string_view HELLO_FIXTURE=R"NGC(
%
let #hello = 12345
print["hello.ngc"]
%nothing after here matters
)NGC";

    constexpr std::string_view WORLD_FIXTURE="let #world = 12345\n";

    constexpr std::string_view TOOL_CHANGE_FIXTURE=R"NGC(
sub _tool_change[#tool_number] {
    G90
    G49
    G20
    G53 G0 Z#5163
    G10 L2 P9 X#5381 Y#5382 Z#5383
    G59.3
    G0 X0 Y0
    G0 Z4
    G38.3 F50 Z0
    if #5070 == 0 { return 0 }
    G91 G0 Z0.25
    G90 G38.3 F10 Z0
    if #5070 == 0 { return 0 }
    G53 G0 Z#5163
    return 1
}
)NGC";

    ngc::ToolTable fixtureToolTable() {
        ngc::ToolTable tools;
        tools.set(1,{.number=1,.x=0,.y=0,.z=2.0,.a=0,.b=0,.c=0,
                     .diameter=0.1,.comment={}});
        tools.set(2,{.number=2,.x=0,.y=0,.z=2.0,.a=0,.b=0,.c=0,
                     .diameter=0.25,.comment="fixture tool"});
        tools.set(13,{.number=13,.x=0,.y=0,.z=2.0,.a=0,.b=0,.c=0,
                      .diameter=0.125,.comment={}});
        return tools;
    }

    std::vector<std::tuple<std::string,std::string>> fixturePrograms(std::string main,
                                                                      std::string name) {
        return {{std::string(HELLO_FIXTURE),"fixture/hello.ngc"},
                {std::string(WORLD_FIXTURE),"fixture/world.ngc"},
                {std::string(TOOL_CHANGE_FIXTURE),"fixture/tool_change.ngc"},
                {std::move(main),std::move(name)}};
    }

    std::string boundedPreviewFixture(const int commands,const bool exactStop) {
        std::string source=exactStop?"G61\n":"G64 P0.005\n";
        source+="G90 G1 F60\n";
        for(int command=0;command<commands;++command)
            source+=std::format("G1 X{} Y{}\n",command%2,(command/2)%2);
        return source;
    }

    std::expected<ngc::MachineConfiguration,std::string> fixtureMachineConfiguration() {
        constexpr std::string_view source=R"TOML(
[machine]
units = "inch"
coordinates = ["x", "y", "z"]
[trajectory]
path_acceleration = 25.1
path_jerk = 101
rapid_velocity = 3.33
arc_chord_tolerance = 0.0001
lookahead_duration = 2.0
[simulation]
servo_period = 0.001
scheduler_period = 0.01
[feed_hold]
tangential_acceleration = 5.0
tangential_jerk = 25.0
[jogging]
acceleration = 5.0
jerk = 25.0
[pendant]
enabled = true
driver = "vistacnc_p2s"
[pendant.step]
fine_distance = 0.001
coarse_distance = 0.01
[pendant.velocity]
full_scale_counts_per_second = 400.0
max_velocity_percent = 100
lease_duration = 0.1
[axes.x]
joints = [0]
minimum = -14.0
maximum = 13.75
max_velocity = 3.33333333333
max_acceleration = 5.1
max_jerk = 101
[axes.y]
joints = [1, 2]
minimum = 0
maximum = 33.25
max_velocity = 3.33333333333
max_acceleration = 5.1
max_jerk = 101
[axes.z]
joints = [3]
minimum = -8.5
maximum = 0.001
max_velocity = 3.33333333333
max_acceleration = 5.1
max_jerk = 101
[digital_inputs]
tool_probe = 0
shared_home = 1
y2_home = 2
[probing]
input = "tool_probe"
condition = "active"
debounce = 0.010
[[joints]]
id = 0
name = "x"
axis = "x"
coordinate_scale = 1.0
minimum = -14.0
maximum = 13.75
max_velocity = 3.33333333333
max_acceleration = 5.1
max_jerk = 101
[joints.homing]
input = "shared_home"
condition = "active"
home_position = 12.75
switch_position = 13.85
search_velocity = 2
latch_velocity = 0.2
backoff_distance = 0.25
debounce = 0.010
final_velocity = 0.0
use_index = false
[[joints]]
id = 1
name = "y1"
axis = "y"
coordinate_scale = 1.0
minimum = -0.1
maximum = 33.25
max_velocity = 3.33333333333
max_acceleration = 25.1
max_jerk = 101
[joints.homing]
input = "shared_home"
condition = "active"
home_position = 1
switch_position = -0.1
search_velocity = -2
latch_velocity = -0.2
backoff_distance = 0.25
debounce = 0.010
final_velocity = 0.0
use_index = false
[[joints]]
id = 2
name = "y2"
axis = "y"
coordinate_scale = 1.0
minimum = -0.1
maximum = 33.25
max_velocity = 3.33333333333
max_acceleration = 25.1
max_jerk = 101
[joints.homing]
input = "y2_home"
condition = "active"
home_position = 1
switch_position = -0.14
search_velocity = -2
latch_velocity = -0.2
backoff_distance = 0.25
debounce = 0.010
final_velocity = 0.0
use_index = false
[[joints]]
id = 3
name = "z"
axis = "z"
coordinate_scale = 1.0
minimum = -8.5
maximum = 0.001
max_velocity = 3.33333333333
max_acceleration = 25.1
max_jerk = 101
[joints.homing]
input = "shared_home"
condition = "active"
home_position = -1
switch_position = 0.1
search_velocity = 2
latch_velocity = 0.2
backoff_distance = 0.25
debounce = 0.010
final_velocity = 0.0
use_index = false
[homing]
require_before_motion = false
[[homing.groups]]
name = "z"
sequence = 0
joints = [3]
[[homing.groups]]
name = "x"
sequence = 1
joints = [0]
[[homing.groups]]
name = "y_gantry"
sequence = 2
joints = [1, 2]
start_together = true
stop_each_joint_on_trigger = true
final_move_together = true
)TOML";
        const auto path=std::filesystem::temp_directory_path()/"ngc-machine-fixture.toml";
        {
            std::ofstream file(path,std::ios::binary|std::ios::trunc);
            if(!file) return std::unexpected("failed to create machine configuration fixture");
            file.write(source.data(),static_cast<std::streamsize>(source.size()));
            if(!file) return std::unexpected("failed to write machine configuration fixture");
        }
        auto configuration=ngc::loadMachineConfiguration(path);
        std::filesystem::remove(path);
        return configuration;
    }

    void require(const bool condition, const std::string_view message) {
        if(!condition) {
            throw std::runtime_error(std::string(message));
        }
    }

    void requireNear(const double actual, const double expected, const std::string_view message) {
        require(std::abs(actual - expected) < EPSILON, message);
    }

    std::vector<ngc::MachineCommand> execute(ngc::Machine &machine, const std::string_view source) {
        ngc::Program program(std::string(source), "test.ngc");
        const auto compiled = program.compile();
        require(compiled.has_value(), compiled ? "" : compiled.error().text());
        std::vector<ngc::MachineCommand> commands;

        const std::function callback = [&](std::unique_ptr<const ngc::EvaluatorMessage> message, ngc::Evaluator &) {
            if(const auto block = message->as<ngc::BlockMessage>()) {
                auto emitted = machine.executeBlock(block->block());
                commands.insert(commands.end(), std::make_move_iterator(emitted.begin()), std::make_move_iterator(emitted.end()));
            }
        };

        ngc::Evaluator evaluator(machine.memory(), callback);
        const ngc::Preamble preamble(machine.memory());
        evaluator.executeFirstPass(preamble.statements());
        evaluator.executeFirstPass(program.statements());
        evaluator.executeSecondPass(program.statements());
        return commands;
    }

    std::vector<ngc::MachineCommand> run(const std::string_view source) {
        ngc::Machine machine(UNIT);
        return execute(machine, source);
    }

    void testRapidAndFeedMove() {
        const auto commands = run("G0 X10\nG1 F120 X20\n");
        require(commands.size() == 2, "expected two line moves");

        const auto *rapid = std::get_if<ngc::MoveLine>(&commands[0]);
        const auto *feed = std::get_if<ngc::MoveLine>(&commands[1]);
        require(rapid != nullptr && feed != nullptr, "expected line moves");
        requireNear(rapid->speed(), -1.0, "G0 must retain the rapid sentinel");
        requireNear(rapid->to().x, 10.0, "G0 endpoint is incorrect");
        requireNear(feed->speed(), 120.0, "G1 must use the programmed feedrate");
        requireNear(feed->from().x, 10.0, "G1 start point is incorrect");
        requireNear(feed->to().x, 20.0, "G1 endpoint is incorrect");
    }

    void testG64IsAnInertPathModeFlag() {
        ngc::Machine machine(UNIT);
        const auto commands = execute(machine, "G21 G64 P1\nG1 F120 X1\n");
        require(commands.size() == 1 && std::holds_alternative<ngc::MoveLine>(commands.front()),
                "G64 should not alter or add machine commands");
        require(machine.state().modePath == ngc::GCPath::G64, "G64 should become the active path mode");
        const auto modalCodes = machine.activeModalGCodes();
        require(std::ranges::find(modalCodes, "G64") != modalCodes.end(),
                "G64 should be reported in active modal state");
        require(machine.pathTolerance().has_value(), "G64 P should establish a path tolerance");
        requireNear(*machine.pathTolerance(), 1.0 / 25.4,
                    "G64 P should be converted into configured machine units");

        execute(machine, "G61\n");
        require(!machine.pathTolerance(), "leaving G64 should clear its path tolerance");
    }

    void testG64BlendScaleGeometryProgramIsValid() {
        auto source=boundedPreviewFixture(35,false);
        source+="G0 X3 Y0\nG3 X4 Y1 I0 J1\nG1 X5 Y1\n";
        ngc::Machine machine(UNIT);
        const auto commands=execute(machine,source);
        require(commands.size()>=30,"G64 blend-scale test should retain all geometry sections");
        require(std::ranges::any_of(commands,[](const auto &command) {
            return std::holds_alternative<ngc::MoveArc>(command);
        }),"G64 blend-scale test should exercise arc junctions");
    }

    void testMemoryStackBounds() {
        ngc::Memory memory;
        const auto address = memory.push(12.5);
        requireNear(*memory.read(address), 12.5, "stack value should be readable");

        const auto invalid = memory.read(address + 1);
        require(!invalid && invalid.error() == ngc::Memory::Error::INVALID_STACK_ADDRESS,
                "an out-of-range stack address should return INVALID_STACK_ADDRESS");

        requireNear(memory.pop(), 12.5, "stack pop should return the last value");
        try {
            static_cast<void>(memory.pop());
            require(false, "popping an empty stack should fail");
        } catch(const std::logic_error &) {
        }
    }

    void testToolTableLoadsFinalLineWithoutNewline() {
        const auto path = std::filesystem::temp_directory_path() / "ngc-tool-table-no-newline.txt";
        {
            std::ofstream file(path, std::ios::binary | std::ios::trunc);
            file << "7 1 2 3 4 5 6 0.25 final comment";
        }

        ngc::ToolTable table;
        const auto loaded = table.load(path);
        std::filesystem::remove(path);
        require(loaded.has_value(), loaded ? "" : loaded.error());
        const auto tool = table.get(7);
        require(tool.has_value(), "final tool-table row should be loaded");
        requireNear(tool->diameter, 0.25, "final tool-table numeric value should not be truncated");
        require(tool->comment == "final comment", "final tool-table comment should not be truncated");
    }

    void testToolTableRejectsDuplicateToolNumbers() {
        const auto path = std::filesystem::temp_directory_path() / "ngc-tool-table-duplicate.txt";
        {
            std::ofstream file(path, std::ios::binary | std::ios::trunc);
            file << "7 1 2 3 4 5 6 0.25 first\n"
                    "7 6 5 4 3 2 1 0.5 duplicate\n";
        }

        ngc::ToolTable table;
        table.set(42, { 42, 1, 2, 3, 4, 5, 6, 0.75, "existing" });
        const auto loaded = table.load(path);
        std::filesystem::remove(path);
        require(!loaded.has_value(), "duplicate tool numbers should fail to load");
        require(loaded.error().find("row:2 duplicate tool number 7") != std::string::npos,
                "duplicate tool error should identify its row and tool number");
        require(table.get(42).has_value(), "a failed tool-table load should preserve the existing table");
        require(!table.get(7).has_value(), "a failed tool-table load should not retain partially parsed rows");
    }

    void testNumericParsingRejectsTrailingGarbage() {
        require(ngc::fromChars("12.5").has_value(), "a complete number should parse");
        require(!ngc::fromChars("12.5xyz").has_value(), "trailing garbage should make a number invalid");
        require(!ngc::fromChars("").has_value(), "an empty number should be invalid");
    }

    void testLexerRejectsIncompleteOperators() {
        for(const std::string_view source : { "!1\n", ".\n" }) {
            ngc::Program program(std::string(source), "invalid-token.ngc");
            require(!program.compile().has_value(), "an incomplete operator should produce a parser error");
        }
    }

    void testFileHelpersHandleEmptyAndFailedIo() {
        const auto directory = std::filesystem::temp_directory_path();
        const auto emptyPath = directory / "ngc-empty-file-test.txt";
        const auto contentPath = directory / "ngc-content-file-test.txt";
        const auto missingPath = directory / "ngc-missing-file-test.txt";
        std::filesystem::remove(missingPath);

        require(ngc::writeFile(emptyPath, {}).has_value(), "writing an empty file should succeed");
        const auto empty = ngc::readFile(emptyPath);
        require(empty && empty->empty(), "reading an empty file should return an empty string");

        require(ngc::writeFile(contentPath, "abc\n123").has_value(), "writing file content should succeed");
        require(ngc::writeFile(contentPath, "replacement").has_value(), "atomically replacing file content should succeed");
        const auto content = ngc::readFile(contentPath);
        require(content && *content == "replacement", "file replacement should publish the complete new content");
        require(!ngc::readFile(missingPath).has_value(), "reading a missing file should fail");

        std::filesystem::remove(emptyPath);
        std::filesystem::remove(contentPath);
    }

    void testFeedMotionRequiresFeedrate() {
        ngc::InterpreterSession session(UNIT, ngc::InterpretationMode::RealRun);
        session.setPrograms({ { "G1 X1\n", "missing-feed.ngc" } });
        session.compile([](const auto &callback) { callback(); });
        require(session.compiled(), "missing-feed regression program should compile");
        const auto event = session.next();
        const auto error = std::get_if<ngc::InterpreterError>(&event);
        require(error != nullptr, "G1 without a modal feedrate should produce an interpreter error");
        require(error->message.find("positive feedrate") != std::string::npos,
                "missing G1 feedrate error should explain the requirement");
    }

    void requireInterpreterError(const std::string_view source, const std::string_view expectedText) {
        ngc::InterpreterSession session(UNIT, ngc::InterpretationMode::RealRun);
        session.setPrograms({ { std::string(source), "invalid-code.ngc" } });
        session.compile([](const auto &callback) { callback(); });
        require(session.compiled(), "invalid-code regression program should parse before interpretation");
        const auto event = session.next();
        const auto error = std::get_if<ngc::InterpreterError>(&event);
        require(error != nullptr, "invalid or unsupported G-code should produce an interpreter error");
        require(error->message.find(expectedText) != std::string::npos,
                "interpreter error should explain the unsupported operation");
    }

    void testUnsupportedCodesProduceInterpreterErrors() {
        requireInterpreterError("G99\n", "unsupported G-code G99");
        requireInterpreterError("M99\n", "unsupported M-code M99");
        requireInterpreterError("G93\n", "unsupported feed mode G93");
        requireInterpreterError("G10 L20 P1 X1\n", "unsupported G10 L20");
        requireInterpreterError("G0 O1\n", "unsupported word O1");
    }

    void testFailedBlockRollsBackMachineState() {
        ngc::Machine machine(UNIT);
        machine.memory().write(ngc::Var::G54_X, 10.0);

        bool rejected = false;
        try {
            static_cast<void>(execute(machine, "G21 G10 L2 P1 X20 Q1\n"));
        } catch(const std::exception &) {
            rejected = true;
        }
        require(rejected, "a block with an unsupported word should fail");

        requireNear(machine.memory().read(ngc::Var::G54_X), 10.0,
                    "a failed block must not retain partial persistent-offset writes");
        require(machine.state().modeUnits == ngc::GCUnits::G20,
                "a failed block must restore its previous modal state");
    }

    void testInterpreterCancellationInterruptsEvaluation() {
        ngc::InterpreterSession session(UNIT, ngc::InterpretationMode::Preview);
        session.setPrograms({ { "while 1 {}\n", "cancellation.ngc" } });
        session.compile([](const auto &callback) { callback(); });
        require(session.compiled(), "cancellation regression program should compile");
        session.begin();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        session.requestStop();
        session.stop();
    }

    void testSimulationWorkerStartsPlayback() {
        SimulationWorker worker;
        ngc::ToolTable tools;
        tools.set(1, { 1, 0, 0, 2, 0, 0, 0, 0.5, "simulation tool" });
        require(worker.start({ { "sub _tool_change[#tool] {}\nT1 M6\nG1 F60 X1\n", "simulation-worker.ngc" } }, tools),
                "simulation worker should accept a program");

        auto snapshot = worker.snapshot();
        require(snapshot.status == ngc::SimulationStatus::Running,
                "simulation worker should publish its running state synchronously from start");
        bool observedMovingTool = false;
        for(int attempt = 0; attempt < 5000; ++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            snapshot = worker.snapshot();
            const auto toolPose = ngc::simulationToolPose(snapshot);
            if(toolPose.geometry.number == 1 && snapshot.hasActiveMotion
               && snapshot.machinePosition.x > 1e-6 && snapshot.machinePosition.x < 1.0 - 1e-6) {
                requireNear(toolPose.spindlePosition.x, snapshot.machinePosition.x,
                            "loaded tool spindle pose should follow each backend position snapshot");
                requireNear(toolPose.tipPosition.x, snapshot.machinePosition.x,
                            "loaded tool tip position should follow each backend position snapshot");
                observedMovingTool = true;
                break;
            }
        }
        const auto toolPose = ngc::simulationToolPose(snapshot);
        const auto toolMessage = std::format("simulation worker did not publish tool 1: status {} tool {} error '{}'",
                                             static_cast<int>(snapshot.status), toolPose.geometry.number,
                                             snapshot.error);
        require(toolPose.geometry.number == 1, toolMessage);
        require(observedMovingTool,
                "simulation worker should publish a loaded tool pose during motion");
        requireNear(toolPose.tipPosition.z, toolPose.spindlePosition.z - 2.0,
                    "simulation worker should publish the physical cutter position");
        worker.join();
    }

    void testSimulationPresentationFollowsNestedToolChangeExecution() {
        constexpr std::string_view TOOL_CHANGE = R"NGC(
sub _tool_change[#tool_number] {
    G90 G64 P0.0001
    G55 G1 F60 X0.2
    G56 G1 X0.4
    return 1
}
)NGC";
        constexpr std::string_view MAIN = R"NGC(
G54
T1 M6
G57 G1 X0.6
)NGC";

        SimulationWorker worker;
        ngc::ToolTable tools;
        tools.set(1, {
            .number = 1,
            .x = 0,
            .y = 0,
            .z = 0.5,
            .a = 0,
            .b = 0,
            .c = 0,
            .diameter = 0.25,
            .comment = "marker presentation tool",
        });
        require(worker.start({
                    {std::string(TOOL_CHANGE), "presentation-tool-change.ngc"},
                    {std::string(MAIN), "presentation-main.ngc"},
                }, tools),
                "nested tool-change presentation simulation should start");

        auto snapshot = worker.snapshot();
        auto sawG55Motion = false;
        auto sawG56Motion = false;
        auto sawFinalG57Motion = false;
        auto sawNestedOwnership = false;
        auto priorMaximumX = 0.0;
        for (auto attempt = 0; attempt < 5000
             && snapshot.status != ngc::SimulationStatus::Completed
             && snapshot.status != ngc::SimulationStatus::Error; ++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            snapshot = worker.snapshot();
            priorMaximumX = std::max(priorMaximumX, snapshot.machinePosition.x);
            if (!snapshot.hasActiveMotion
                || !snapshot.activePresentation.workCoordinateSystem) {
                continue;
            }

            const auto &presentation = snapshot.activePresentation;
            const auto &workCoordinateSystem =
                presentation.workCoordinateSystem->name;
            require(presentation.tool.number == 1,
                    "tool-change motion presentation should expose the prepared tool");
            if (snapshot.machinePosition.x > 0.02
                && snapshot.machinePosition.x < 0.18) {
                require(workCoordinateSystem == "G55",
                        "interpreter lookahead must not expose G56 or G57 "
                        "during the G55 tool-change move");
                sawG55Motion = true;
                sawNestedOwnership = sawNestedOwnership
                    || (std::ranges::any_of(
                            presentation.activeBlocks, [](const auto &block) {
                                return block.source
                                        == "presentation-main.ngc"
                                    && block.text == "T1 M6";
                            })
                        && std::ranges::any_of(
                            presentation.activeBlocks, [](const auto &block) {
                                return block.source
                                        == "presentation-tool-change.ngc"
                                    && block.text == "G55 G1 F60 X0.2";
                            }));
            } else if (snapshot.machinePosition.x > 0.22
                       && snapshot.machinePosition.x < 0.38) {
                require(workCoordinateSystem == "G56",
                        "G56 presentation should activate only with its "
                        "owning tool-change move");
                sawG56Motion = true;
            } else if (snapshot.machinePosition.x > 0.42
                       && snapshot.machinePosition.x < 0.58) {
                require(workCoordinateSystem == "G57",
                        "the main program's final WCS should activate only "
                        "with its owning move");
                sawFinalG57Motion = true;
            }
        }

        require(snapshot.status == ngc::SimulationStatus::Completed,
                std::format(
                    "nested tool-change presentation simulation did not "
                    "complete: status={} maximum_x={} error='{}'",
                    static_cast<int>(snapshot.status), priorMaximumX,
                    snapshot.error));
        require(snapshot.trajectoryPlanning.planChunks >= 3,
                "nested and final WCS motion should cross multiple prepared "
                "execution chunks");
        require(sawG55Motion && sawG56Motion && sawFinalG57Motion,
                "timed presentation should expose every nested and final WCS "
                "while its owning prepared motion executes");
        require(sawNestedOwnership,
                "nested tool-change presentation should retain both the M6 "
                "caller and active subroutine block");
        require(snapshot.activePresentation.tool.number == 1
                    && snapshot.activePresentation.workCoordinateSystem
                    && snapshot.activePresentation.workCoordinateSystem->name
                        == "G57",
                "completed presentation should retain the executed tool and "
                "final work-coordinate system");
        worker.join();
    }

    void testAdaptivePocketsStartsSimulation() {
        auto main=boundedPreviewFixture(24,true);
        main="T2 M6\n"+main;
        auto tools=fixtureToolTable();

        SimulationWorker worker;
        worker.setTickMultiplier(1000);
        require(worker.start(fixturePrograms(std::move(main),"fixture/exact-stop.ngc"),tools),
                "adaptive-pockets simulation should start");

        auto snapshot = worker.snapshot();
        for(int attempt=0;attempt<30000
            &&snapshot.status!=ngc::SimulationStatus::Completed
            &&snapshot.status!=ngc::SimulationStatus::Error;++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            snapshot = worker.snapshot();
        }
        const auto message = std::format("adaptive-pockets G61 simulation did not complete: status {} error '{}'",
                                         static_cast<int>(snapshot.status), snapshot.error);
        require(snapshot.status==ngc::SimulationStatus::Completed,message);
        require(ngc::simulationToolPose(snapshot).geometry.number==2,
                "completed adaptive-pockets G61 simulation should activate tool 2");
        require(snapshot.programElapsedSeconds>0.0,
                "completed adaptive-pockets G61 simulation should advance program time");
        worker.join();
    }

    void testTimedSimulationRefillsMultiPacketContinuousBatch() {
        // Keep each line longer than 6P: this fixture exercises packet refill,
        // not the deliberately unsplittable all-short cluster path.
        std::string source="G64 P0.0001\n";
        constexpr int COMMANDS=800;
        for(int command=1;command<=COMMANDS;++command)
            source+=std::format("G1 F60 X{:.6f}\n",static_cast<double>(command)*0.01);
        source+="G0 X9\n";

        SimulationWorker worker;
        worker.setTickMultiplier(100);
        ngc::ToolTable tools;
        require(worker.start({{source,"multi-packet-refill.ngc"}},tools),
                "multi-packet refill simulation should start");
        auto snapshot=worker.snapshot();
        for(int attempt=0;attempt<10000&&snapshot.status!=ngc::SimulationStatus::Completed
            &&snapshot.status!=ngc::SimulationStatus::Error;++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            snapshot=worker.snapshot();
        }
        require(snapshot.status==ngc::SimulationStatus::Completed,std::format(
            "timed simulation should refill and terminal-compile a continuous batch before its rapid: {}",
            snapshot.error));
        requireNear(snapshot.machinePosition.x,9.0,
                    "motion after the ended rolling G64 chain should execute");
        require(snapshot.trajectoryPlanning.planChunks>8
                    &&snapshot.trajectoryPlanning.maximumWindowCommands<COMMANDS
                    &&snapshot.trajectoryPlanning.continuousHorizons>=2
                    &&snapshot.trajectoryPlanning.rollingBoundaryCandidates>0,
                std::format("prepared G64 streaming should publish bounded proved horizons: "
                            "chunks={} maximum commands={} horizons={} rolling candidates={}",
                            snapshot.trajectoryPlanning.planChunks,
                            snapshot.trajectoryPlanning.maximumWindowCommands,
                            snapshot.trajectoryPlanning.continuousHorizons,
                            snapshot.trajectoryPlanning.rollingBoundaryCandidates));
        const auto averageHorizonSeconds=
            snapshot.trajectoryPlanning.totalContinuousHorizonSeconds
                /static_cast<double>(snapshot.trajectoryPlanning.continuousHorizons);
        require(snapshot.trajectoryPlanning.minimumContinuousHorizonSeconds>0.0
                    &&snapshot.trajectoryPlanning.minimumContinuousHorizonSeconds
                        <=averageHorizonSeconds
                    &&averageHorizonSeconds
                        <=snapshot.trajectoryPlanning.maximumContinuousHorizonSeconds,
                "continuous planning diagnostics should retain ordered best, average, and worst window times");
        require(std::ranges::none_of(snapshot.statusMessages,[](const auto &entry) {
            return entry.kind==ngc::InterpreterStatusKind::Error;
        }),"successful multi-packet refill should not publish a GUI error");
        worker.join();
    }

    void testTimedSimulationPublishesSnapshotsDuringPlanning() {
        std::string source="G0 X100\nG64 P0.001\n";
        constexpr int COMMANDS=800;
        for(int command=1;command<=COMMANDS;++command)
            source+=std::format("G1 F60 X{:.6f}\n",100.0+static_cast<double>(command)*0.001);
        SimulationWorker worker;
        ngc::ToolTable tools;
        require(worker.start({{source,"planning-snapshot-progress.ngc"}},tools),
                "planning snapshot progress simulation should start");
        auto snapshot=worker.snapshot();
        auto maximumSnapshotSeconds=0.0;
        auto sawMotionDuringPlanning=false;
        for(int attempt=0;attempt<5000&&snapshot.status!=ngc::SimulationStatus::Error
            &&snapshot.trajectoryPlanning.continuousHorizons==0;++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            const auto snapshotStarted=std::chrono::steady_clock::now();
            snapshot=worker.snapshot();
            maximumSnapshotSeconds=std::max(maximumSnapshotSeconds,
                std::chrono::duration<double>(std::chrono::steady_clock::now()-snapshotStarted).count());
            if(snapshot.hasActiveMotion&&snapshot.machinePosition.x>0.01)
                sawMotionDuringPlanning=true;
        }
        worker.stop();
        worker.join();
        require(snapshot.status!=ngc::SimulationStatus::Error,snapshot.error);
        require(snapshot.trajectoryPlanning.continuousHorizons>0,
                "dense continuous planning should finish within the snapshot regression timeout");
        require(sawMotionDuringPlanning,
                "timed simulation should publish moving tool snapshots while continuous planning runs");
        require(maximumSnapshotSeconds<0.25,std::format(
            "GUI-facing simulation snapshots blocked for {:.6f} seconds during planning",
            maximumSnapshotSeconds));
    }

    void testImmediatePreviewBuildsGeometryWithoutTrajectoryExecution() {
        std::string source="G64 P0.001\n";
        constexpr int COMMANDS=800;
        for(int command=1;command<=COMMANDS;++command)
            source+=std::format("G1 F60 X{:.6f}\n",static_cast<double>(command)*0.001);

        // This is deliberately larger than the former G-code and RT packet
        // boundaries. Geometry preview must not depend on either one.
        Worker worker(UNIT);
        require(worker.compile({{source,"preview-multi-packet-refill.ngc"}}),
                "multi-packet immediate preview should start compilation");
        for(int attempt=0;attempt<3000&&!worker.compiled();++attempt)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        require(worker.compiled(),"multi-packet immediate preview should compile");
        const auto initialRevision=worker.lock([&] { return worker.preparedPreview().revision; });
        require(worker.execute(),"multi-packet immediate preview should start execution");
        bool finished=false;
        for(int attempt=0;attempt<10000;++attempt) {
            const auto revision=worker.lock([&] { return worker.preparedPreview().revision; });
            if(revision>initialRevision&&!worker.busy()) {
                finished=true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        require(finished,"multi-packet immediate preview should finish");
        const auto messages=worker.statusMessages();
        const auto error=std::ranges::find_if(messages,[](const auto &message) {
            return message.kind==ngc::InterpreterStatusKind::Error;
        });
        require(error==messages.end(),error==messages.end()?"":
                "multi-packet immediate preview failed: "+error->text);
        require(worker.lock([&] {
            const auto &scene=worker.preparedPreview();
            auto commands=std::size_t{};
            for(const auto &slice:scene.continuousSlices) {
                commands+=slice.commands.size();
                for(const auto &record:slice.commands)
                    if(record.metadata.pathMode!=ngc::ExecutablePathMode::Continuous
                       ||record.metadata.pathTolerance!=0.001) return false;
            }
            commands+=scene.standaloneCommands.size();
            return commands==COMMANDS;
        }),"geometry-only preview should retain G64/P metadata for display blending");
        const auto preparedRevision=worker.lock([&] {
            require(!worker.preparedPreview().continuousSlices.empty(),
                    "geometry-only preview should publish prepared slices");
            return worker.preparedPreview().revision;
        });
        require(worker.clearPreview(),"Clear Preview should succeed while Preview is idle");
        require(worker.lock([&] {
            return worker.preparedPreview().continuousSlices.empty()
                &&worker.preparedPreview().standaloneCommands.empty()
                &&worker.preparedPreview().revision>preparedRevision;
        }),"Clear Preview should clear both canonical and prepared display geometry");
        worker.join();
    }

    void testGeometryProducerBlendsAcrossMotionModeChanges() {
        Worker worker(UNIT);
        require(worker.compile({{
            "G64 P0.1\nG1 F60 X10\nG2 X20 I5\nG1 X30\n",
            "preview-mixed-motion-g64.ngc"}}),
            "mixed-motion G64 preview should start compilation");
        for(int attempt = 0; attempt < 3000 && !worker.compiled(); ++attempt)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        require(worker.compiled(), "mixed-motion G64 preview should compile");
        const auto initialRevision = worker.lock([&] { return worker.preparedPreview().revision; });
        require(worker.execute(), "mixed-motion G64 preview should start execution");
        auto finished = false;
        for(int attempt = 0; attempt < 3000; ++attempt) {
            const auto revision = worker.lock([&] { return worker.preparedPreview().revision; });
            if(revision > initialRevision && !worker.busy()) {
                finished = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        require(finished, "mixed-motion G64 preview should finish");
        const auto messages = worker.statusMessages();
        const auto error = std::ranges::find_if(messages, [](const auto &message) {
            return message.kind == ngc::InterpreterStatusKind::Error;
        });
        require(error == messages.end(), error == messages.end() ? ""
            : "mixed-motion G64 preview failed: " + error->text);
        worker.lock([&] {
            const auto &slices = worker.preparedPreview().continuousSlices;
            require(!slices.empty(), "mixed-motion G64 preview should publish continuous geometry");
            auto junctionBlends = std::size_t{0};
            for(const auto &slice : slices) {
                for(const auto &piece : slice.pieces) {
                    if(piece.kind != ngc::PreparedPieceKind::JunctionBlend) continue;
                    ++junctionBlends;
                    require(piece.sourceCommands.size() == 2,
                            "each preview junction blend should identify both source entities");
                    require(std::ranges::all_of(piece.sourceCommands, [&](const auto id) {
                        return std::ranges::any_of(slice.commands,
                            [id](const auto &record) { return record.id == id; });
                    }), "a preview slice should retain command presentation for every blend source entity");
                }
            }
            require(junctionBlends == 2,
                    "G1-to-G2 and G2-to-G1 must both produce junction blends");
        });
        worker.join();
    }

    void testGeometryProducerPreparesExactStopPreviewSlices() {
        Worker worker(UNIT);
        require(worker.compile({{
            "G0 X1\nG61\nG1 F60 X2\nG2 X3 I0.5\nG61.1\nG1 X4\n",
            "preview-standalone-motion.ngc"}}),
            "standalone-motion preview should start compilation");
        for(int attempt=0;attempt<3000&&!worker.compiled();++attempt)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        require(worker.compiled(),"standalone-motion preview should compile");
        const auto initialRevision=worker.lock([&] {
            return worker.preparedPreview().revision;
        });
        require(worker.execute(),"standalone-motion preview should start execution");
        for(int attempt=0;attempt<3000;++attempt) {
            if(worker.lock([&] {
                    return worker.preparedPreview().revision>initialRevision;
                })&&!worker.busy()) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        require(!worker.busy(),"standalone-motion preview should finish");
        worker.lock([&] {
            const auto &scene=worker.preparedPreview();
            require(scene.standaloneCommands.empty(),
                    "rapid and exact-stop source entities must not use standalone command messages");
            require(!scene.continuousSlices.empty(),
                    "the exact-stop source sequence should publish prepared geometry slices");
            const auto &slice=scene.continuousSlices.front();
            require(scene.geometryEnds.size()==1
                    &&scene.geometryEnds.front().chain==slice.chain
                    &&scene.geometryEnds.front().sequence
                        >scene.continuousSlices.back().sequence,
                    std::format("the exact-stop slice sequence should finish with its ordered end "
                        "message: slices={} ends={} first_chain={} end_chain={} last_slice_sequence={} "
                        "end_sequence={}",scene.continuousSlices.size(),scene.geometryEnds.size(),
                        slice.chain,scene.geometryEnds.empty()?0:scene.geometryEnds.front().chain,
                        scene.continuousSlices.back().sequence,
                        scene.geometryEnds.empty()?0:scene.geometryEnds.front().sequence));
            std::vector<const ngc::PreparedPathPiece *> pieces;
            auto commandCount=std::size_t{};
            for(const auto &preparedSlice:scene.continuousSlices) {
                commandCount+=preparedSlice.commands.size();
                require(std::ranges::all_of(preparedSlice.commands,[](const auto &record) {
                    return record.metadata.pathMode==ngc::ExecutablePathMode::ExactStop;
                }),"G61/G61.1 slice commands should retain exact-stop timing metadata");
                for(const auto &piece:preparedSlice.pieces) pieces.push_back(&piece);
            }
            require(commandCount==4&&pieces.size()==4,
                    "the exact-stop slice should retain one prepared piece per source entity");
            require(std::ranges::none_of(pieces,[](const auto *piece) {
                return piece->kind==ngc::PreparedPieceKind::JunctionBlend
                    ||piece->kind==ngc::PreparedPieceKind::ClusterSpline;
            }),"exact-stop preparation must preserve complete source entities without blends");
            require(std::holds_alternative<ngc::PreparedLineCurve>(
                        pieces[0]->curve->value),
                    "rapid preview should use a producer-prepared line");
            require(std::holds_alternative<ngc::PreparedLineCurve>(
                        pieces[1]->curve->value),
                    "G61 line preview should use a producer-prepared line");
            require(std::holds_alternative<ngc::PreparedArcCurve>(
                        pieces[2]->curve->value),
                    "G61 arc preview should use the shared producer-prepared arc");
            require(std::holds_alternative<ngc::PreparedLineCurve>(
                        pieces[3]->curve->value),
                    "G61.1 line preview should use a producer-prepared line");
        });
        worker.join();
    }

    void testModalG64RapidsRemainExactPreparedMotion() {
        constexpr auto source="G64 P0.005\nG0 X1\nZ1.7\nG1 F60 X2\n";
        ngc::ToolTable tools;

        Worker preview(UNIT);
        require(preview.compile({{source,"g64-rapid.ngc"}}),
                "G64 rapid preview should compile");
        for(int attempt=0;attempt<3000&&!preview.compiled();++attempt)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        require(preview.compiled(),"G64 rapid preview compilation should finish");
        const auto initialRevision=preview.lock([&] {
            return preview.preparedPreview().revision;
        });
        require(preview.execute(),"G64 rapid preview should execute");
        auto previewFinished=false;
        for(int attempt=0;attempt<3000;++attempt) {
            if(preview.lock([&] {
                    return preview.preparedPreview().revision>initialRevision;
                })&&!preview.busy()) {
                previewFinished=true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        require(previewFinished,"G64 rapid preview should finish");
        preview.lock([&] {
            auto exactCommands=std::size_t{};
            auto continuousCommands=std::size_t{};
            for(const auto &slice:preview.preparedPreview().continuousSlices)
                for(const auto &record:slice.commands) {
                    if(record.metadata.pathMode==ngc::ExecutablePathMode::ExactStop)
                        ++exactCommands;
                    else ++continuousCommands;
                }
            require(exactCommands==2&&continuousCommands==1,std::format(
                "modal G64 should stream its two rapid moves as exact prepared geometry "
                "and its feed move as continuous geometry: exact={} continuous={}",
                exactCommands,continuousCommands));
        });
        preview.join();

        SimulationWorker simulation;
        simulation.setTickMultiplier(1000);
        require(simulation.start({{source,"g64-rapid.ngc"}},tools),
                "G64 rapid simulation should start");
        auto snapshot=simulation.snapshot();
        for(int attempt=0;attempt<5000
            &&snapshot.status!=ngc::SimulationStatus::Completed
            &&snapshot.status!=ngc::SimulationStatus::Error;++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            snapshot=simulation.snapshot();
        }
        require(snapshot.status==ngc::SimulationStatus::Completed,std::format(
            "modal G64 rapid simulation should complete without entering continuous timing: {}",
            snapshot.error));
        requireNear(snapshot.machinePosition.x,2.0,
                    "modal G64 rapid regression should reach the feed endpoint");
        simulation.join();
    }

    void testZeroLengthFeedBetweenPreparedMotionChains() {
        constexpr auto source =
            "G64 P0.001\n"
            "G1 F120 X1\n"
            "G0 X2\n"
            "G1 F120 X2\n"
            "G1 X3\n"
            "M30\n";
        ngc::ToolTable tools;
        SimulationWorker simulation;
        simulation.setTickMultiplier(1000);
        require(simulation.start({{source, "zero-length-chain-transition.ngc"}}, tools),
                "zero-length chain transition simulation should start");
        auto snapshot = simulation.snapshot();
        for(int attempt = 0; attempt < 10000
            && snapshot.status != ngc::SimulationStatus::Completed
            && snapshot.status != ngc::SimulationStatus::Error; ++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            snapshot = simulation.snapshot();
        }
        require(snapshot.status == ngc::SimulationStatus::Completed, std::format(
            "zero-length feed between prepared chains should simulate completely: {}",
            snapshot.error));
        requireNear(snapshot.machinePosition.x, 3.0,
                    "motion after a zero-length feed should reach its endpoint");
        simulation.join();
    }

    void testGeometryPreviewResolvesProbeAtCanonicalTarget() {
        ngc::ToolTable tools;
        const ngc::ToolTable::tool_entry_t tool{
            1,0.0,0.0,2.0,0.0,0.0,0.0,0.25,"preview probe tool"};
        tools.set(tool.number,tool);
        Worker worker(UNIT);
        require(worker.setToolTable(tools),"probe preview should accept its tool table");
        require(worker.compile({{
            "G43 H1\nG38.3 F5 Z-1\nG1 Z0\n","geometry-probe.ngc"}}),
            "geometry probe preview should start compilation");
        for(int attempt=0;attempt<3000&&!worker.compiled();++attempt)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        require(worker.compiled(),"geometry probe preview should compile");
        const auto initialRevision=worker.lock([&] { return worker.preparedPreview().revision; });
        require(worker.execute(),"geometry probe preview should start");
        bool finished=false;
        for(int attempt=0;attempt<3000;++attempt) {
            const auto revision=worker.lock([&] { return worker.preparedPreview().revision; });
            if(revision>initialRevision&&!worker.busy()) {
                finished=true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        require(finished,"geometry probe preview should finish without a backend result");
        const auto messages=worker.statusMessages();
        require(std::ranges::none_of(messages,[](const auto &message) {
            return message.kind==ngc::InterpreterStatusKind::Error;
        }),"canonical-target probe preview should not report an execution error");
        worker.lock([&] {
            const auto &commands=worker.preparedPreview().standaloneCommands;
            require(commands.size()==1,"probe preview should retain the probe as standalone work");
            const auto *probe=std::get_if<ngc::ProbeMove>(&commands[0].command.command);
            require(probe!=nullptr,"probe preview should retain canonical probe geometry");
            require(commands[0].displayGeometry!=nullptr,
                    "geometry producer should prepare probe display geometry");
            ngc::CurveEvaluationWorkspace workspace;
            const auto endpoint=ngc::positionAtDistance(*commands[0].displayGeometry,
                commands[0].displayGeometry->length,workspace)
                -commands[0].command.presentation.activeToolOffset;
            requireNear(endpoint.z,-1.0,
                        "prepared probe preview endpoint should be the commanded tool-tip target");
            require(worker.preparedPreview().continuousSlices.size()==1
                    &&worker.preparedPreview().continuousSlices.front().pieces.size()==1,
                    "motion following a probe should use an exact-stop prepared geometry slice");
        });
        worker.join();
    }

    void testAdaptivePocketsGeometryPreviewAvoidsTrajectoryExecution() {
        auto main="G0 X2\n"+boundedPreviewFixture(120,false)+"G61\nG1 X3\n";
        auto tools=fixtureToolTable();

        Worker worker(UNIT);
        require(worker.setToolTable(tools),
                "adaptive-pockets geometry preview should accept the tool table");
        require(worker.compile(fixturePrograms(std::move(main),"fixture/mixed-preview.ngc")),
                "adaptive-pockets geometry preview should start compilation");
        for(int attempt=0;attempt<5000&&!worker.compiled();++attempt)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        require(worker.compiled(),"adaptive-pockets geometry preview should compile");
        const auto initialRevision=worker.lock([&] { return worker.preparedPreview().revision; });
        const auto started=std::chrono::steady_clock::now();
        require(worker.execute(),"adaptive-pockets geometry preview should start");
        bool finished=false;
        for(int attempt=0;attempt<10000;++attempt) {
            const auto revision=worker.lock([&] { return worker.preparedPreview().revision; });
            if(revision>initialRevision&&!worker.busy()) {
                finished=true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        const auto elapsed=std::chrono::duration<double>(
            std::chrono::steady_clock::now()-started).count();
        require(finished,std::format(
            "adaptive-pockets geometry preview did not finish in {:.3f} seconds",elapsed));
        const auto messages=worker.statusMessages();
        const auto error=std::ranges::find_if(messages,[](const auto &message) {
            return message.kind==ngc::InterpreterStatusKind::Error;
        });
        require(error==messages.end(),error==messages.end()?"":
                "adaptive-pockets geometry preview failed: "+error->text);
        require(worker.lock([&] {
            const auto &slices=worker.preparedPreview().continuousSlices;
            return std::ranges::any_of(slices,[](const auto &slice) {
                return !slice.commands.empty();
            });
        }),"adaptive-pockets preview should retain prepared G64 motion");
        require(worker.lock([&] {
            auto exact=false;
            auto continuous=false;
            for(const auto &slice:worker.preparedPreview().continuousSlices)
                for(const auto &record:slice.commands) {
                    exact|=record.metadata.pathMode==ngc::ExecutablePathMode::ExactStop;
                    continuous|=record.metadata.pathMode==ngc::ExecutablePathMode::Continuous;
                }
            return exact&&continuous;
        }),"adaptive-pockets G64 preview should classify startup rapids as exact and cutting feeds as continuous");
        require(worker.lock([&] {
            return worker.preparedPreview().continuousSlices.size()>1;
        }),"adaptive-pockets geometry should reach Preview through multiple SPSC slices");
        worker.join();
    }

    void testSimulationDriverFailureAppearsInGuiStatusStream() {
        SimulationWorker worker(UNIT,{
            .pathAcceleration=0.0,.rapidSpeed=100.0,.arcChordTolerance=0.0001,.pathJerk=10.0,
        });
        ngc::ToolTable tools;
        require(worker.start({{"G64 P0.01\nG1 F60 X1\nG1 Y1\n","simulation-error.ngc"}},tools),
                "simulation GUI error regression should start");
        auto snapshot=worker.snapshot();
        for(int attempt=0;attempt<3000&&snapshot.status!=ngc::SimulationStatus::Error;++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            snapshot=worker.snapshot();
        }
        require(snapshot.status==ngc::SimulationStatus::Error,
                "fatal simulation planning failure should reach GUI presentation state");
        require(!snapshot.error.empty()&&std::ranges::any_of(snapshot.statusMessages,[&](const auto &entry) {
            return entry.kind==ngc::InterpreterStatusKind::Error&&entry.text==snapshot.error;
        }),"fatal simulation driver error should appear in the GUI chronological status stream");
        require(!std::ranges::contains(
                    snapshot.activePresentation.modalGCodes, "G64"),
                "a planning failure must preserve the last executed modal "
                "presentation instead of exposing interpreter lookahead");
        worker.join();
    }

    void test1001PreviewCompletesBoundedly() {
        auto main="T13\n"+boundedPreviewFixture(160,false);
        auto tools=fixtureToolTable();

        // A coarse mock tick keeps this regression focused on NRT G64 planning
        // rather than the duration of immediate simulated playback.
        Worker worker(UNIT);
        require(worker.setToolTable(tools), "1001 preview should accept the tool table");
        require(worker.compile(fixturePrograms(std::move(main),"fixture/bounded-preview.ngc")),
                "1001 preview should start compilation");
        for(int attempt = 0; attempt < 3000 && !worker.compiled(); ++attempt)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        require(worker.compiled(), "1001 preview should compile");

        const auto initialRevision = worker.lock([&] { return worker.preparedPreview().revision; });
        const auto started = std::chrono::steady_clock::now();
        require(worker.execute(), "1001 preview should start execution");
        auto revision = initialRevision;
        for(int attempt = 0; attempt < 5000; ++attempt) {
            revision = worker.lock([&] { return worker.preparedPreview().revision; });
            if(revision > initialRevision && !worker.busy()) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        const auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - started).count();
        require(revision > initialRevision && !worker.busy(),
                std::format("1001 preview did not finish in {:.3f} seconds", elapsed));
        const auto messages=worker.statusMessages();
        const auto error=std::ranges::find_if(messages,[](const auto &message) {
            return message.kind==ngc::InterpreterStatusKind::Error;
        });
        require(error==messages.end(),error==messages.end()?"":"1001 preview failed: "+error->text);
        require(worker.lock([&] {
            const auto &scene=worker.preparedPreview();
            return !scene.continuousSlices.empty()||!scene.standaloneCommands.empty();
        }),
                "1001 preview should retain displayable canonical geometry");
        worker.join();
    }

    void test1002PreparedSliceBoundaries() {
        std::string main;
        for(int chain=0;chain<24;++chain) {
            main+="G0 X0 Y0\nG64 P0.005\nG1 F60\n";
            for(int command=0;command<20;++command)
                main+=std::format("G1 X{} Y{}\n",command%2,(command/2+chain)%2);
        }
        main+="M30\nprint[\"Hello World!\"]\n";
        auto tools=fixtureToolTable();

        Worker worker(UNIT);
        require(worker.setToolTable(tools), "1002_3d preview should accept the tool table");
        require(worker.compile(fixturePrograms(std::move(main),"fixture/slice-boundaries.ngc")),
                "1002_3d preview should start compilation");
        for(int attempt = 0; attempt < 3000 && !worker.compiled(); ++attempt)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        require(worker.compiled(), "1002_3d preview should compile");
        const auto initialRevision = worker.lock([&] { return worker.preparedPreview().revision; });
        require(worker.execute(), "1002_3d preview should execute");
        for(int attempt = 0; attempt < 60000; ++attempt) {
            if(worker.lock([&] { return worker.preparedPreview().revision > initialRevision; })
               && !worker.busy()) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        require(!worker.busy(), "1002_3d preview should finish");
        const auto messages = worker.statusMessages();
        require(std::ranges::none_of(messages, [](const auto &message) {
            return message.kind == ngc::InterpreterStatusKind::Error;
        }), "1002_3d preview should not report an error");
        require(std::ranges::any_of(messages, [](const auto &message) {
            return message.kind == ngc::InterpreterStatusKind::Print
                && message.text == "Hello World!";
        }), "1002_3d preview should interpret through the print after M30");
        worker.lock([&] {
            const auto &slices = worker.preparedPreview().continuousSlices;
            require(slices.size() > 100,
                    "1002_3d preview should retain all prepared geometry slices");
            require(worker.preparedPreview().geometryEnds.size() >= 20,
                    "1002_3d preview should retain every motion chain");
            std::set<ngc::PreparedPieceId> pieceIds;
            const ngc::PreparedPathPiece *previous = nullptr;
            ngc::ContinuousChainId previousChain = 0;
            for(const auto &slice : slices) {
                for(const auto &piece : slice.pieces) {
                    require(pieceIds.insert(piece.id).second,
                            std::format("1002 duplicate prepared piece {}", piece.id));
                }
                if(previous && previousChain == slice.chain && !slice.pieces.empty()) {
                    ngc::CurveEvaluationWorkspace workspace;
                    const auto prior = ngc::positionAtDistance(
                        *previous->curve, previous->curveTo, workspace);
                    const auto next = ngc::positionAtDistance(
                        *slice.pieces.front().curve, slice.pieces.front().curveFrom, workspace);
                    require((prior - next).length() <= 1e-12, std::format(
                        "1002 slice boundary mismatch before sequence {}: distance={:.17g}",
                        slice.sequence, (prior - next).length()));
                }
                if(!slice.pieces.empty()) previous = &slice.pieces.back();
                previousChain = slice.chain;
            }
        });
        worker.join();
    }

    void testMdiToolChangeUsesAutoloadPrograms() {
        auto tools=fixtureToolTable();

        SimulationWorker worker;
        worker.setTickMultiplier(1000);
        require(worker.start(fixturePrograms("T2 M6\n","<MDI>"),tools),
                "MDI tool change should start");

        auto snapshot = worker.snapshot();
        for(int attempt = 0; attempt < 3000
            && ngc::simulationToolPose(snapshot).geometry.number != 2
            && snapshot.status != ngc::SimulationStatus::Error; ++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            snapshot = worker.snapshot();
        }
        const auto message = std::format("MDI T2 M6 did not select tool 2: status {} error '{}'",
                                         static_cast<int>(snapshot.status), snapshot.error);
        require(ngc::simulationToolPose(snapshot).geometry.number == 2, message);
        worker.join();
    }

    void testSimulationWorkerPersistsUntilReset() {
        SimulationWorker worker;
        ngc::ToolTable tools;
        worker.setTickMultiplier(1000);

        const auto waitForCompletion = [&](const std::string_view message) {
            auto snapshot = worker.snapshot();
            for(int attempt = 0; attempt < 2000 && snapshot.status != ngc::SimulationStatus::Completed
                && snapshot.status != ngc::SimulationStatus::Error; ++attempt) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                snapshot = worker.snapshot();
            }
            require(snapshot.status == ngc::SimulationStatus::Completed, message);
            return snapshot;
        };

        tools.set(1, { .number = 1, .x = 0, .y = 0, .z = 0.5, .a = 0, .b = 0,
                       .c = 0, .diameter = 0.25, .comment = {} });

        require(worker.start({ { "G0 X1\n", "<MDI>" } }, tools, true),
                "first persistent simulation command should start");
        const auto afterFirst = waitForCompletion("first persistent command should complete");
        requireNear(afterFirst.machinePosition.x, 1.0, "first persistent command should end at X1");
        requireNear(afterFirst.servoPeriodSeconds, 0.001,
                    "simulation should advance with the configured fixed servo period");
        require(afterFirst.tickMultiplier == 1000,
                "simulation should expose the integer ticks-per-period multiplier");
        require(afterFirst.servoTicks > 0, "timed simulation should report executed fixed servo ticks");
        requireNear(afterFirst.schedulerPeriodSeconds, 0.01,
                    "simulation should expose its independent 100 Hz scheduler period");
        require(afterFirst.servoTicksPerSchedulerPeriod == 10,
                "each scheduler wake should batch ten 1 ms servo ticks at 1x speed");

        require(worker.start({ { "G43 H1\n", "<MDI>" } }, tools, true),
                "G43 persistent simulation command should start");
        const auto afterG43 = waitForCompletion("G43 persistent command should complete");
        require(std::ranges::find(afterG43.activePresentation.modalGCodes, "G43")
                    != afterG43.activePresentation.modalGCodes.end(),
                "simulation should preserve G43 modal G-code after running non-motion command");
        requireNear(afterG43.activePresentation.activeToolOffset.z, 0.5,
                    "completed state-only presentation should atomically include "
                    "the active tool offset");
        require(ngc::simulationToolPose(afterG43).geometry.number == 0,
                "active tool compensation must remain distinct from physical "
                "tool geometry");

        require(worker.start({ { "G91 G0 X1\n", "<MDI>" } }, tools, true),
                "second persistent simulation command should start");
        const auto afterSecond = waitForCompletion("second persistent command should complete");
        requireNear(afterSecond.machinePosition.x, 2.0,
                    "second command should continue from the prior simulated position");
        require(std::ranges::find(afterSecond.activePresentation.modalGCodes, "G43")
                    != afterSecond.activePresentation.modalGCodes.end(),
                "subsequent commands should preserve G43 modal state");

        require(worker.resetSimulation(), "idle simulation should reset");
        const auto reset = worker.snapshot();
        require(reset.status == ngc::SimulationStatus::Stopped, "reset simulation should report stopped");
        requireNear(reset.machinePosition.x, 0.0, "reset simulation should restore the initial pose");
        worker.join();
    }

    void testSimulationProgramElapsedTimeIsPlaybackSpeedIndependent() {
        const auto runAtMultiplier = [](const int multiplier) {
            SimulationWorker worker;
            ngc::ToolTable tools;
            worker.setTickMultiplier(multiplier);
            require(worker.start({ { "G1 F60 X0.01\n", "elapsed-time.ngc" } }, tools),
                    "elapsed-time simulation should start");

            auto snapshot = worker.snapshot();
            for(int attempt = 0; attempt < 3000
                && snapshot.status != ngc::SimulationStatus::Completed
                && snapshot.status != ngc::SimulationStatus::Error; ++attempt) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                snapshot = worker.snapshot();
            }
            worker.join();
            require(snapshot.status == ngc::SimulationStatus::Completed,
                    std::format("elapsed-time simulation at x{} should complete: {}",
                                multiplier, snapshot.error));
            require(snapshot.programElapsedSeconds > 0.0,
                    "completed motion should report positive G-code elapsed time");
            return snapshot.programElapsedSeconds;
        };

        const auto normalSpeed = runAtMultiplier(1);
        const auto accelerated = runAtMultiplier(1000);
        requireNear(accelerated, normalSpeed,
                    "G-code elapsed time must not depend on the playback multiplier");
    }

    void testInterpreterStatusMessagesPreserveOrder() {
        ngc::InterpreterSession session(UNIT, ngc::InterpretationMode::RealRun);
        session.setPrograms({ { "print[\"before error\"]\nG1 X1\n", "status-order.ngc" } });
        session.compile([](const auto &callback) { callback(); });

        const auto event = session.next();
        require(std::holds_alternative<ngc::InterpreterError>(event),
                "status-order program should terminate with an interpreter error");
        const auto &messages = session.statusMessages();
        require(messages.size() == 2, "print and error should share one status stream");
        require(messages[0].kind == ngc::InterpreterStatusKind::Print,
                "print should retain its position before a later error");
        require(messages[1].kind == ngc::InterpreterStatusKind::Error,
                "terminal error should follow earlier print output");
        require(messages[1].text.find("status-order.ngc:2:1") != std::string::npos,
                "block errors should include source, line, and column");
    }

    void testArcUsesModalFeedrate() {
        const auto commands = run("G1 F120 X10 Y0\nG3 X0 Y10 I-10 J0\n");
        require(commands.size() == 2, "expected one line and one arc");

        const auto *arc = std::get_if<ngc::MoveArc>(&commands[1]);
        require(arc != nullptr, "expected an arc move");
        requireNear(arc->speed(), 120.0, "G2/G3 must use the modal feedrate");
        requireNear(arc->center().x, 0.0, "arc center X is incorrect");
        requireNear(arc->center().y, 0.0, "arc center Y is incorrect");
        requireNear(arc->axis().z, 1.0, "G3 must have a positive G17 axis");
        requireNear(arc->to().x, 0.0, "arc endpoint X is incorrect");
        requireNear(arc->to().y, 10.0, "arc endpoint Y is incorrect");
    }

    void testArcCenterDistanceModes() {
        const auto incremental = run(
            "G90 G91.1 G1 F10 X10 Y0\n"
            "G3 X0 Y10 I-10 J0\n");
        require(incremental.size() == 2, "incremental arc-center mode should emit a line and an arc");
        const auto *incrementalArc = std::get_if<ngc::MoveArc>(&incremental[1]);
        require(incrementalArc != nullptr, "G91.1 should emit an arc");
        requireNear(incrementalArc->center().x, 0.0, "G91.1 I should be relative to the arc start");
        requireNear(incrementalArc->center().y, 0.0, "G91.1 J should be relative to the arc start");
        requireNear(incrementalArc->to().x, 0.0, "G91.1 must not change G90 endpoint mode");

        ngc::Machine machine(UNIT);
        machine.memory().write(ngc::Var::G54_X, 10.0);
        machine.memory().write(ngc::Var::G54_Y, 20.0);
        const auto absolute = execute(machine,
            "G90 G90.1 G1 F10 X1 Y0\n"
            "G3 X0 Y1 I0 J0\n");
        require(absolute.size() == 2, "absolute arc-center mode should emit a line and an arc");
        const auto *absoluteArc = std::get_if<ngc::MoveArc>(&absolute[1]);
        require(absoluteArc != nullptr, "G90.1 should emit an arc");
        requireNear(absoluteArc->center().x, 10.0, "G90.1 I should use the active work-coordinate origin");
        requireNear(absoluteArc->center().y, 20.0, "G90.1 J should use the active work-coordinate origin");
        requireNear(absoluteArc->to().x, 10.0, "G90.1 must retain the active work offset");
        requireNear(absoluteArc->to().y, 21.0, "G90.1 endpoint should remain in G90 work coordinates");
    }

    void testIncrementalMove() {
        const auto commands = run("G0 X10\nG91 G1 F25 X5\n");
        require(commands.size() == 2, "expected two line moves");

        const auto *move = std::get_if<ngc::MoveLine>(&commands[1]);
        require(move != nullptr, "expected an incremental line move");
        requireNear(move->from().x, 10.0, "incremental move start is incorrect");
        requireNear(move->to().x, 15.0, "incremental move endpoint is incorrect");
        requireNear(move->speed(), 25.0, "incremental move feedrate is incorrect");
    }

    void testBeginProgramRunResetsRuntimeState() {
        ngc::Machine machine(UNIT);
        machine.memory().write(ngc::Var::G54_X, 7.0);
        const auto firstProgramAddress = machine.memory().addData(
            ngc::MemoryCell(ngc::MemoryCell::Flags::READ | ngc::MemoryCell::Flags::WRITE, 42.0));
        requireNear(*machine.memory().read(firstProgramAddress), 42.0,
                    "program-owned memory should be readable during its run");

        const auto firstCommands = execute(machine, "G1 F10 X3\n");
        require(firstCommands.size() == 1, "expected one command in the first run");

        machine.beginProgramRun();
        require(machine.state().modeMotion == ngc::GCMotion::G0, "beginProgramRun must restore G0");
        requireNear(machine.memory().read(ngc::Var::G54_X), 7.0, "beginProgramRun must retain coordinate offsets");
        require(!machine.memory().read(firstProgramAddress).has_value(),
                "beginProgramRun must discard program-owned memory");
        const auto secondProgramAddress = machine.memory().addData(
            ngc::MemoryCell(ngc::MemoryCell::Flags::READ | ngc::MemoryCell::Flags::WRITE, 84.0));
        require(secondProgramAddress == firstProgramAddress,
                "new runs should reuse the program-storage address range");

        const auto secondCommands = execute(machine, "G1 F10 X3\n");
        require(secondCommands.size() == 1, "second run should emit one command");

        const auto *move = std::get_if<ngc::MoveLine>(&secondCommands.front());
        require(move != nullptr, "expected a line move in the second run");
        requireNear(move->from().x, 0.0, "second run must begin at the machine origin");
        requireNear(move->to().x, 10.0, "second run must apply the retained G54 offset");
    }

    void testLinearUnitConversion() {
        struct Case {
            ngc::Machine::Unit machineUnit;
            std::string_view gcodeUnit;
            double sourceValue;
            double expectedValue;
        };

        constexpr std::array cases {
            Case { ngc::Machine::Unit::Inch, "G20", 1.0, 1.0 },
            Case { ngc::Machine::Unit::Inch, "G21", 25.4, 1.0 },
            Case { ngc::Machine::Unit::Millimeter, "G20", 1.0, 25.4 },
            Case { ngc::Machine::Unit::Millimeter, "G21", 25.4, 25.4 },
        };

        for(const auto &[machineUnit, gcodeUnit, sourceValue, expectedValue] : cases) {
            ngc::Machine machine(machineUnit);
            const auto source = std::format(
                "{}\nG10 L2 P1 X{} Y{} Z{}\nG1 F{} X{} Y{} Z{}\nG17 G3 X0 Y0 I-{} J0\nG18 G2 X{} Z0 I0 K-{}\n",
                gcodeUnit,
                sourceValue, sourceValue, sourceValue,
                sourceValue, sourceValue, sourceValue, sourceValue,
                sourceValue,
                sourceValue, sourceValue);
            const auto commands = execute(machine, source);

            requireNear(machine.memory().read(ngc::Var::G54_X), expectedValue, "G10 X conversion is incorrect");
            requireNear(machine.memory().read(ngc::Var::G54_Y), expectedValue, "G10 Y conversion is incorrect");
            requireNear(machine.memory().read(ngc::Var::G54_Z), expectedValue, "G10 Z conversion is incorrect");
            require(commands.size() == 3, "expected a line and two arcs");

            const auto *line = std::get_if<ngc::MoveLine>(&commands[0]);
            const auto *g17Arc = std::get_if<ngc::MoveArc>(&commands[1]);
            const auto *g18Arc = std::get_if<ngc::MoveArc>(&commands[2]);
            require(line != nullptr && g17Arc != nullptr && g18Arc != nullptr, "expected one line and two arcs");

            requireNear(line->speed(), expectedValue, "feedrate conversion is incorrect");
            requireNear(line->to().x, expectedValue * 2.0, "X conversion is incorrect");
            requireNear(line->to().y, expectedValue * 2.0, "Y conversion is incorrect");
            requireNear(line->to().z, expectedValue * 2.0, "Z conversion is incorrect");
            requireNear(g17Arc->center().x, expectedValue, "G17 I conversion is incorrect");
            requireNear(g17Arc->center().y, expectedValue * 2.0, "G17 J conversion is incorrect");
            requireNear(g18Arc->center().x, expectedValue, "G18 I conversion is incorrect");
            requireNear(g18Arc->center().z, expectedValue, "G18 K conversion is incorrect");
        }
    }

    void testToolLengthCompensation() {
        const ngc::ToolTable::tool_entry_t tool1 { 1, 0.0, 0.0, 1.234, 0.0, 0.0, 0.0, 0.25, "test tool 1" };

        struct Case {
            std::string_view gcodeUnit;
            double sourceValue;
        };

        constexpr std::array cases {
            Case { "G20", 1.0 },
            Case { "G21", 25.4 },
        };

        for(const auto &[gcodeUnit, sourceValue] : cases) {
            ngc::Machine machine(ngc::Machine::Unit::Inch);
            machine.toolTable().set(tool1.number, tool1);
            const auto commands = execute(machine, std::format(
                "{}\nG43 H1\nG1 F{} X{} Z{}\nG49\nG1 X{} Z{}\n",
                gcodeUnit, sourceValue, sourceValue, sourceValue, sourceValue, sourceValue));

            require(commands.size() == 2, "expected one G43 move and one G49 move");
            const auto *compensated = std::get_if<ngc::MoveLine>(&commands[0]);
            const auto *cancelled = std::get_if<ngc::MoveLine>(&commands[1]);
            require(compensated != nullptr && cancelled != nullptr, "expected line moves");
            requireNear(compensated->to().x, 1.0, "G43 must not rescale the tool-table X offset");
            requireNear(compensated->to().z, 2.234, "G43 H must apply the tool-table Z offset");
            requireNear(cancelled->to().z, 1.0, "G49 must cancel the active tool offset");
        }
    }

    void testToolLengthCompensationUsesActiveTool() {
        ngc::Machine machine(ngc::Machine::Unit::Inch);
        const ngc::ToolTable::tool_entry_t tool2 { 2, 0.0, 0.0, 3.321, 0.0, 0.0, 0.0, 0.5, "test tool 2" };
        machine.toolTable().set(tool2.number, tool2);
        const auto commands = execute(machine, "T2\nG43\nG1 F1 X1 Z1\n");

        require(commands.size() == 1, "expected one compensated move");
        const auto *move = std::get_if<ngc::MoveLine>(&commands.front());
        require(move != nullptr, "expected a line move");
        requireNear(move->to().z, 4.321, "G43 without H must use the active T tool offset");
    }

    void testToolLengthCompensationWithWorkOffset() {
        ngc::Machine machine(ngc::Machine::Unit::Inch);
        const ngc::ToolTable::tool_entry_t tool { 1, 2.0, 3.0, 4.0, 0.0, 0.0, 0.0, 0.25, "test tool" };
        machine.toolTable().set(tool.number, tool);
        machine.memory().write(ngc::Var::G54_X, 10.0);
        machine.memory().write(ngc::Var::G54_Y, 20.0);
        machine.memory().write(ngc::Var::G54_Z, 30.0);

        const auto commands = execute(machine, "G43 H1\nG1 F1 X1 Y1 Z1\n");

        require(commands.size() == 1, "expected one compensated G54 move");
        const auto *move = std::get_if<ngc::MoveLine>(&commands.front());
        require(move != nullptr, "expected a line move");
        requireNear(move->to().x, 13.0, "G43 must combine X tool and G54 offsets");
        requireNear(move->to().y, 24.0, "G43 must combine Y tool and G54 offsets");
        requireNear(move->to().z, 35.0, "G43 must combine Z tool and G54 offsets");
    }

    void testG53BypassesWorkOffsetButRetainsToolOffset() {
        ngc::Machine machine(ngc::Machine::Unit::Inch);
        const ngc::ToolTable::tool_entry_t tool { 1, 2.0, 3.0, 4.0, 0.0, 0.0, 0.0, 0.25, "test tool" };
        machine.toolTable().set(tool.number, tool);
        machine.memory().write(ngc::Var::G54_X, 10.0);
        machine.memory().write(ngc::Var::G54_Y, 20.0);
        machine.memory().write(ngc::Var::G54_Z, 30.0);

        const auto commands = execute(machine, "G43 H1\nG53 G1 F1 X1 Y1 Z1\n");

        require(commands.size() == 1, "expected one compensated G53 move");
        const auto *move = std::get_if<ngc::MoveLine>(&commands.front());
        require(move != nullptr, "expected a line move");
        requireNear(move->to().x, 3.0, "G53 must bypass G54 X while retaining the tool offset");
        requireNear(move->to().y, 4.0, "G53 must bypass G54 Y while retaining the tool offset");
        requireNear(move->to().z, 5.0, "G53 must bypass G54 Z while retaining the tool offset");
        require(move->machineCoordinates(), "G53 motion should retain its machine-coordinate display metadata");
    }

    void testArcAllowsOmittedZeroCenterOffsets() {
        struct Case {
            std::string_view source;
            ngc::vec3_t expectedCenter;
        };

        const std::array cases {
            Case { "G17 G2 F1 X2 I1\n", { 1.0, 0.0, 0.0 } },
            Case { "G17 G2 F1 Y2 J1\n", { 0.0, 1.0, 0.0 } },
            Case { "G18 G2 F1 X2 I1\n", { 1.0, 0.0, 0.0 } },
            Case { "G18 G2 F1 Z2 K1\n", { 0.0, 0.0, 1.0 } },
            Case { "G19 G2 F1 Y2 J1\n", { 0.0, 1.0, 0.0 } },
            Case { "G19 G2 F1 Z2 K1\n", { 0.0, 0.0, 1.0 } },
        };

        for(const auto &[source, expectedCenter] : cases) {
            const auto commands = run(source);
            require(commands.size() == 1, "expected one arc with an omitted zero center offset");
            const auto *arc = std::get_if<ngc::MoveArc>(&commands.front());
            require(arc != nullptr, "expected an arc move");
            requireNear(arc->center().x, expectedCenter.x, "arc center X is incorrect");
            requireNear(arc->center().y, expectedCenter.y, "arc center Y is incorrect");
            requireNear(arc->center().z, expectedCenter.z, "arc center Z is incorrect");
        }
    }

    void testArcGeometryValidation() {
        for(const auto unit : { ngc::Machine::Unit::Inch, ngc::Machine::Unit::Millimeter }) {
            ngc::Machine machine(unit);
            const auto tolerance = machine.arcTolerance();
            const ngc::vec3_t center { 0.0, 0.0, 0.0 };
            const ngc::position_t start { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
            const ngc::position_t exactEnd { 0.0, 1.0, 0.0, 0.0, 0.0, 0.0 };
            const ngc::position_t roundedEnd { 0.0, 1.0 + tolerance * 0.5, 0.0, 0.0, 0.0, 0.0 };
            const ngc::position_t invalidEnd { 0.0, 1.0 + tolerance * 2.0, 0.0, 0.0, 0.0, 0.0 };
            const ngc::position_t zeroRadiusStart { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

            require(machine.validateArc(start, exactEnd, center).has_value(), "equal-radius arc must be valid");
            require(machine.validateArc(start, roundedEnd, center).has_value(), "rounding-sized radius mismatch must be valid");
            require(!machine.validateArc(start, invalidEnd, center).has_value(), "large radius mismatch must be invalid");
            require(!machine.validateArc(zeroRadiusStart, exactEnd, center).has_value(), "zero-radius arc must be invalid");
            require(machine.validateArc(start, start, center).has_value(), "full-circle arc must be valid");
        }

        ngc::Machine inchMachine(ngc::Machine::Unit::Inch);
        ngc::Machine millimeterMachine(ngc::Machine::Unit::Millimeter);
        requireNear(inchMachine.arcTolerance(), 0.0005, "inch arc tolerance is incorrect");
        requireNear(millimeterMachine.arcTolerance(), 0.0127, "millimeter arc tolerance is incorrect");
    }

    void testArcRadiusMismatchIsRecoverableInterpreterError() {
        requireInterpreterError("G3 F1 X1 Y1.001 I1\n", "arc radius mismatch");
    }

    void testInterpreterSessionOwnsCompilationAndExecution() {
        const auto synchronize = [](const auto &callback) { callback(); };
        ngc::InterpreterSession session(UNIT, ngc::InterpretationMode::RealRun);

        session.setPrograms({ { "G0 X10\nG1 F20 X15\n", "session.ngc" } });
        session.compile(synchronize);
        require(session.compiled(), "interpreter session should compile a valid program");
        require(session.parserErrors().empty(), "valid session program should have no parser errors");

        session.begin();
        const auto first = session.next();
        require(std::holds_alternative<ngc::MachineCommand>(first), "first next call should emit the first command");
        require(session.blockMessages().size() == 1, "first next call must not process the second block");
        require(session.blockMessages().size() == 1, "first next call should process exactly one block");

        const auto second = session.next();
        require(std::holds_alternative<ngc::MachineCommand>(second), "second next call should emit the second command");
        require(session.blockMessages().size() == 2, "second next call should process the second block");

        const auto completed = session.next();
        require(std::holds_alternative<ngc::InterpreterCompleted>(completed), "interpreter session should complete after its commands are consumed");
        require(session.blockMessages().size() == 2, "interpreter session should retain executed block messages");

        session.setPrograms({ { "let #i = 0\nwhile #i < 2 {\nG1 F1 X#i\n#i = #i + 1\n}\n", "loop.ngc" } });
        session.compile(synchronize);
        require(session.compiled(), "interpreter session should compile a loop program");
        session.begin();
        require(std::holds_alternative<ngc::MachineCommand>(session.next()), "next should yield from inside a loop");
        require(session.blockMessages().size() == 1, "loop evaluation must pause after its first emitted command");
        require(std::holds_alternative<ngc::MachineCommand>(session.next()), "next should resume the suspended loop");
        require(session.blockMessages().size() == 2, "resumed loop should emit its second command");
        require(std::holds_alternative<ngc::InterpreterCompleted>(session.next()), "loop program should complete after resuming");

        session.setPrograms({ { "G0 X[\n", "invalid.ngc" } });
        session.compile(synchronize);
        require(!session.compiled(), "interpreter session should reject an invalid program");
        require(!session.parserErrors().empty(), "invalid session program should retain its parser error");
    }

    void compileSession(ngc::InterpreterSession &session, const std::string_view source) {
        session.setPrograms({ { std::string(source), "incremental.ngc" } });
        session.compile([](const auto &callback) { callback(); });
        require(session.compiled(), "incremental session test program should compile");
        session.begin();
    }

    ngc::MoveLine nextLine(ngc::InterpreterSession &session, const std::string_view message) {
        const auto event = session.next();
        const auto command = std::get_if<ngc::MachineCommand>(&event);
        require(command != nullptr, message);
        const auto line = std::get_if<ngc::MoveLine>(command);
        require(line != nullptr, message);
        return *line;
    }

    void requireCompleted(ngc::InterpreterSession &session, const std::string_view message) {
        require(std::holds_alternative<ngc::InterpreterCompleted>(session.next()), message);
    }

    void testInterpreterTaskVariable() {
        const auto testMode = [](const ngc::InterpretationMode mode, const double expectedTaskValue) {
            ngc::InterpreterSession session(UNIT, mode);
            compileSession(session,
                "let #task_ptr = &#_task\n"
                "G1 F1 X&#_task\n"
                "G1 X##task_ptr\n");

            requireNear(session.machine().memory().read(ngc::Var::TASK), expectedTaskValue,
                        "session should install the correct _task value");
            requireNear(nextLine(session, "_task address should produce a line move").to().x, 6000.0,
                        "&_task should resolve to parameter 6000");
            requireNear(nextLine(session, "indirect _task read should produce a line move").to().x, expectedTaskValue,
                        "indirect _task read should match the session mode");
            requireCompleted(session, "_task program should complete");
        };

        testMode(ngc::InterpretationMode::Preview, 0.0);
        testMode(ngc::InterpretationMode::Simulation, 1.0);
        testMode(ngc::InterpretationMode::RealRun, 2.0);

        const auto requireWriteRejected = [](const std::string_view source) {
            ngc::InterpreterSession session(UNIT, ngc::InterpretationMode::RealRun);
            compileSession(session, source);
            const auto event = session.next();
            const auto error = std::get_if<ngc::InterpreterError>(&event);
            require(error != nullptr, "writing _task should produce an interpreter error");
            require(error->message.find("WRITE") != std::string::npos,
                    "writing _task should fail because the memory cell is read-only");
            require(error->message.find("incremental.ngc:") != std::string::npos,
                    "writing _task error should identify its source location");
        };

        requireWriteRejected("#_task = 0\n");
        requireWriteRejected("#6000 = 0\n");
        requireWriteRejected("let #task_ptr = &#_task\n##task_ptr = 0\n");
    }

    void testIncrementalSessionControlFlow() {
        ngc::InterpreterSession session(UNIT, ngc::InterpretationMode::RealRun);

        compileSession(session,
            "if 0 {\n"
            "G1 F1 X1\n"
            "} else {\n"
            "G1 F1 X2\n"
            "}\n");
        requireNear(nextLine(session, "if/else should yield its selected branch").to().x, 2.0, "else branch should be selected");
        requireCompleted(session, "if/else session should complete");

        compileSession(session,
            "sub inner[#x] {\n"
            "G1 F1 X#x\n"
            "return #x + 1\n"
            "}\n"
            "sub outer[#x] {\n"
            "return inner[#x]\n"
            "}\n"
            "let #result = outer[3]\n"
            "G1 X#result\n");
        requireNear(nextLine(session, "nested subroutine should yield from its inner call").to().x, 3.0, "inner subroutine move is incorrect");
        require(session.blockMessages().size() == 1, "nested call must remain suspended after its first command");
        requireNear(nextLine(session, "subroutine should resume through its return").to().x, 4.0, "returned subroutine value is incorrect");
        requireCompleted(session, "nested subroutine session should complete");

        compileSession(session,
            "let #i = 0\n"
            "while #i < 5 {\n"
            "#i = #i + 1\n"
            "if #i == 2 { continue }\n"
            "if #i == 4 { break }\n"
            "G1 F1 X#i\n"
            "}\n");
        requireNear(nextLine(session, "loop should yield before continue").to().x, 1.0, "first loop move is incorrect");
        requireNear(nextLine(session, "loop should resume after continue").to().x, 3.0, "continued loop move is incorrect");
        requireCompleted(session, "break should complete the loop without another command");

        compileSession(session,
            "sub return_from_if[] {\n"
            "if 1 {\n"
            "return 7\n"
            "return 8\n"
            "}\n"
            "return 9\n"
            "}\n"
            "G1 F1 X[return_from_if[]]\n"
            "sub return_from_loop[] {\n"
            "let #i = 0\n"
            "while #i < 1 {\n"
            "return 6\n"
            "#i = #i + 1\n"
            "}\n"
            "return 9\n"
            "}\n"
            "G1 X[return_from_loop[]]\n");
        requireNear(nextLine(session, "return should propagate out of an if").to().x, 7.0,
                    "statements after a nested return must not execute");
        requireNear(nextLine(session, "return should propagate out of a loop").to().x, 6.0,
                    "a loop must preserve a nested return value");
        requireCompleted(session, "nested return session should complete");

        compileSession(session,
            "let #i = 0\n"
            "let #side_effect = 0\n"
            "while #i < 2 {\n"
            "#i = #i + 1\n"
            "if #i == 1 {\n"
            "continue\n"
            "#side_effect = 99\n"
            "}\n"
            "#side_effect = #side_effect + 1\n"
            "}\n"
            "G1 F1 X#side_effect\n"
            "while 1 {\n"
            "break\n"
            "#side_effect = 99\n"
            "}\n"
            "G1 X#side_effect\n");
        requireNear(nextLine(session, "continue should skip its remaining compound body").to().x, 1.0,
                    "statements after continue must not execute");
        requireNear(nextLine(session, "break should skip its remaining compound body").to().x, 1.0,
                    "statements after break must not execute");
        requireCompleted(session, "nested break and continue session should complete");

        compileSession(session,
            "let #x = 1\n"
            "G1 F1 X#x\n"
            "#x = 7\n"
            "G1 X#x\n");
        requireNear(nextLine(session, "first parameter-dependent move should yield").to().x, 1.0, "initial parameter value is incorrect");
        requireNear(nextLine(session, "updated parameter-dependent move should yield").to().x, 7.0, "updated parameter value is incorrect");
        requireCompleted(session, "parameter-dependent session should complete");

        compileSession(session, "G1 F1 X1\nG1 X2\n");
        nextLine(session, "session should yield before it is stopped");
        session.stop();
        compileSession(session, "G1 F1 X9\n");
        requireNear(nextLine(session, "stopped session should be reusable").to().x, 9.0, "restarted session move is incorrect");
        requireCompleted(session, "restarted session should complete");

        compileSession(session, "undefined_subroutine[]\n");
        const auto error = session.next();
        const auto interpreterError = std::get_if<ngc::InterpreterError>(&error);
        require(interpreterError != nullptr, "runtime evaluator failure should produce InterpreterError");
        require(interpreterError->message.find("undefined sub") != std::string::npos, "InterpreterError should retain the evaluator failure");
        require(interpreterError->message.find("incremental.ngc:1:1") != std::string::npos,
                "runtime evaluator failure should include its statement location");
    }

    void testProbeCommandAndBarrier() {
        ngc::Machine machine(UNIT);
        const auto commands = execute(machine, "G38.3 F5 Z-1\n");
        require(commands.size() == 1, "G38.3 should emit one probe command");
        const auto probe = std::get_if<ngc::ProbeMove>(&commands.front());
        require(probe != nullptr, "G38.3 should emit ProbeMove");
        require(probe->id() != 0, "probe command should have a nonzero ID");
        requireNear(probe->from().z, 0.0, "probe start position is incorrect");
        requireNear(probe->target().z, -1.0, "probe target position is incorrect");
        requireNear(probe->feed(), 5.0, "probe feedrate is incorrect");
        require(probe->stopOnContact(), "G38.3 should stop on contact");
        require(!probe->errorIfNotFound(), "G38.3 should not error when contact is not found");

        ngc::InterpreterSession session(UNIT, ngc::InterpretationMode::RealRun);
        compileSession(session, "G38.3 F5 Z-1\nG1 X1\n");
        const auto command = session.next();
        const auto emitted = std::get_if<ngc::MachineCommand>(&command);
        require(emitted != nullptr && std::holds_alternative<ngc::ProbeMove>(*emitted), "session should emit ProbeMove before its barrier");
        const auto waiting = session.next();
        const auto probeBarrier = std::get_if<ngc::InterpreterWaitingForProbe>(&waiting);
        require(probeBarrier != nullptr, "session should wait for a probe result");
        require(session.blockMessages().size() == 1, "session must not evaluate beyond a probe barrier");
        const auto sessionProbe = std::get<ngc::ProbeMove>(*emitted);
        require(probeBarrier->commandId == sessionProbe.id(), "probe barrier should identify its command");
        session.provideProbeResult({ sessionProbe.id(), ngc::ProbeStatus::Triggered, sessionProbe.target(), sessionProbe.target() });
        requireNear(session.machine().memory().read(ngc::Var::PROBE_Z), -1.0, "probe result should update #5063");
        requireNear(session.machine().memory().read(ngc::Var::PROBE_SUCCESS), 1.0, "probe result should update #5070");
        require(std::holds_alternative<ngc::MachineCommand>(session.next()), "session should resume after receiving a probe result");
        requireCompleted(session, "session should complete after its probe result");
    }

    void testAutomaticToolChangeReachesProbeBarrier() {
        ngc::InterpreterSession session(UNIT, ngc::InterpretationMode::RealRun);
        session.machine().toolTable()=fixtureToolTable();
        session.setPrograms({
            { std::string(TOOL_CHANGE_FIXTURE), "fixture/tool_change.ngc" },
            { "T2 M6\n", "tool-change-test.ngc" },
        });
        session.compile([](const auto &callback) { callback(); });
        require(session.compiled(), "automatic tool-change program should compile");
        session.begin();

        const auto toolChangeSynchronization = session.nextWithBlocks(
            [](const auto &callback) { callback(); });
        require(std::holds_alternative<ngc::InterpreterWaitingForSynchronization>(toolChangeSynchronization),
                "M6 should synchronize before preparing or entering the tool-change subroutine");
        session.provideSynchronization();
        const auto toolChangeEvent = session.nextWithBlocks([](const auto &callback) { callback(); });
        const auto toolChangeBlock = std::get_if<ngc::InterpreterBlockLifecycle>(&toolChangeEvent);
        require(toolChangeBlock != nullptr && toolChangeBlock->phase == ngc::BlockLifecyclePhase::Entered
                    && toolChangeBlock->block.text == "T2 M6",
                "M6 block should be published before entering the tool-change subroutine");

        const auto safeZ = nextLine(session, "tool change should retract to safe Z");
        requireNear(safeZ.to().z, -0.1, "tool-change safe Z should use parameter #5163");

        const auto probeXY = nextLine(session, "tool change should move over the probe");
        requireNear(probeXY.to().x, -13.940, "probe X should use parameter #5381");
        requireNear(probeXY.to().y, 2.895, "probe Y should use parameter #5382");

        nextLine(session, "tool change should move to its probe approach height");
        const auto probeEvent = session.next();
        const auto command = std::get_if<ngc::MachineCommand>(&probeEvent);
        require(command != nullptr, "tool change should emit a probe command");
        const auto probe = std::get_if<ngc::ProbeMove>(command);
        require(probe != nullptr, "tool change should emit ProbeMove");
        requireNear(probe->target().x, -13.940, "probe target X is incorrect");
        requireNear(probe->target().y, 2.895, "probe target Y is incorrect");
        requireNear(probe->target().z, -6.560, "probe target Z should use parameter #5383");
        requireNear(probe->feed(), 50.0, "fast probe feedrate is incorrect");
        require(std::holds_alternative<ngc::InterpreterWaitingForProbe>(session.next()), "tool change should stop at the probe barrier");
        session.provideProbeResult({ probe->id(), ngc::ProbeStatus::Triggered, probe->target(), probe->target() });

        const auto retract = nextLine(session, "tool change should retract after the fast probe");
        requireNear(retract.to().z, -6.310, "fast-probe retract distance is incorrect");

        const auto slowProbeEvent = session.next();
        const auto slowCommand = std::get_if<ngc::MachineCommand>(&slowProbeEvent);
        require(slowCommand != nullptr, "tool change should emit its slow probe command");
        const auto slowProbe = std::get_if<ngc::ProbeMove>(slowCommand);
        require(slowProbe != nullptr, "slow probe should be a ProbeMove");
        requireNear(slowProbe->feed(), 10.0, "slow probe feedrate is incorrect");
        require(std::holds_alternative<ngc::InterpreterWaitingForProbe>(session.next()), "tool change should wait for the slow probe result");
        session.provideProbeResult({ slowProbe->id(), ngc::ProbeStatus::Triggered, slowProbe->target(), slowProbe->target() });

        const auto finalRetract = nextLine(session, "tool change should retract to safe Z after probing");
        requireNear(finalRetract.to().z, -0.1, "final tool-change retract should use #5163");
        requireCompleted(session, "dummy probe results should allow the tool-change program to finish");
    }

    void testSpscChannelIsBoundedAndOrdered() {
        ngc::SpscChannel<std::uint64_t, 4> bounded;
        require(bounded.tryPush(1) && bounded.tryPush(2) && bounded.tryPush(3) && bounded.tryPush(4),
                "SPSC channel should accept exactly its advertised capacity");
        require(!bounded.tryPush(5), "SPSC channel should report backpressure when full");
        std::uint64_t value = 0;
        require(bounded.tryPop(value) && value == 1, "SPSC channel should preserve FIFO order");

        ngc::SpscChannel<std::uint64_t, 64> channel;
        constexpr std::uint64_t COUNT = 100000;
        std::thread producer([&] {
            for(std::uint64_t i = 0; i < COUNT; ++i) while(!channel.tryPush(i)) std::this_thread::yield();
        });
        for(std::uint64_t expected = 0; expected < COUNT; ++expected) {
            while(!channel.tryPop(value)) std::this_thread::yield();
            require(value == expected, "SPSC channel should preserve cross-thread publication order");
        }
        producer.join();
    }

    void testOwningSpscChannelTransfersMoveOnlyValues() {
        ngc::OwningSpscChannel<std::unique_ptr<std::uint64_t>, 4> bounded;
        require(bounded.capacity() == 4, "owning SPSC must advertise its usable capacity");
        for(std::uint64_t value = 0; value < 4; ++value)
            require(bounded.tryPush(std::make_unique<std::uint64_t>(value)),
                    "owning SPSC should accept every advertised slot");
        auto retained = std::make_unique<std::uint64_t>(99);
        require(!bounded.tryPush(std::move(retained)), "owning SPSC must report full");
        require(retained && *retained == 99,
                "a failed owning SPSC push must preserve caller ownership");
        for(std::uint64_t value = 0; value < 4; ++value) {
            std::unique_ptr<std::uint64_t> popped;
            require(bounded.tryPop(popped) && popped && *popped == value,
                    "owning SPSC must preserve FIFO order");
            require(bounded.tryPush(std::make_unique<std::uint64_t>(value + 4)),
                    "owning SPSC should wrap without losing usable capacity");
        }
        for(std::uint64_t value = 4; value < 8; ++value) {
            std::unique_ptr<std::uint64_t> popped;
            require(bounded.tryPop(popped) && popped && *popped == value,
                    "owning SPSC wraparound order is incorrect");
        }
        require(bounded.empty(), "owning SPSC should be empty after draining");

        ngc::OwningSpscChannel<std::unique_ptr<std::uint64_t>, 64> threaded;
        constexpr std::uint64_t COUNT = 100000;
        std::jthread producer([&] {
            for(std::uint64_t value = 0; value < COUNT; ++value) {
                auto item = std::make_unique<std::uint64_t>(value);
                require(threaded.waitPush(std::move(item), [] { return false; }),
                        "blocking owning SPSC push should not be cancelled");
            }
        });
        for(std::uint64_t value = 0; value < COUNT; ++value) {
            std::unique_ptr<std::uint64_t> item;
            require(threaded.waitPop(item, [] { return false; }),
                    "blocking owning SPSC pop should not be cancelled");
            require(item && *item == value,
                    "threaded owning SPSC transfer must remain ordered and lossless");
        }

        ngc::OwningSpscChannel<std::unique_ptr<std::uint64_t>, 1> cancellationChannel;
        std::atomic<bool> cancelled = false;
        std::atomic<bool> consumerStarted = false;
        std::unique_ptr<std::uint64_t> cancellationValue;
        bool cancellationResult = false;
        std::thread waitingConsumer([&] {
            consumerStarted.store(true, std::memory_order_release);
            cancellationResult = cancellationChannel.waitPop(cancellationValue, [&] {
                return cancelled.load(std::memory_order_acquire);
            });
        });
        while(!consumerStarted.load(std::memory_order_acquire)) std::this_thread::yield();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        require(cancellationChannel.tryPush(std::make_unique<std::uint64_t>(42)),
                "owning SPSC cancellation regression should publish its terminal value");
        cancelled.store(true, std::memory_order_release);
        cancellationChannel.notifyAll();
        waitingConsumer.join();
        require(cancellationResult && cancellationValue && *cancellationValue == 42,
                "owning SPSC cancellation must not discard an already-published terminal value");
    }

    void testIncrementalGeometryDefersAndDoesNotRebuildAnchorSection() {
        const auto lineRecord = [](const ngc::PreparedCommandId id,
                                   const ngc::position_t &from,
                                   const ngc::position_t &to) {
            ngc::PreparedCommandRecord record;
            record.id = id;
            record.command = ngc::MoveLine{from, to, 60.0};
            return record;
        };
        const std::array<ngc::PreparedCommandRecord, 3> records {
            lineRecord(1, {}, {1, 0, 0, 0, 0, 0}),
            lineRecord(2, {1, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0}),
            lineRecord(3, {1, 1, 0, 0, 0, 0}, {2, 1, 0, 0, 0, 0}),
        };
        const ngc::GeometryPreparationEffort effort{
            .certifySourceTube = false,
            .generateSamples = true,
            .lengthTableIntervalsPerKnotSpan = 32,
            .splineVelocityLimits = {},
        };
        const ngc::ContinuousGeometryBoundaries rollingEnd{
            .deferFinalRetainedSection = true,
        };
        const auto first = ngc::prepareContinuousGeometry(
            std::span{records}.first(2), 0.01, {}, effort, rollingEnd);
        require(first.has_value(), first ? "" : first.error());
        require(first->pieces.size() == 2
                && first->pieces[0].kind == ngc::PreparedPieceKind::RetainedLineSection
                && first->pieces[1].kind == ngc::PreparedPieceKind::JunctionBlend,
                "an incremental window should prepare the retained prefix and incoming blend only");
        require(std::ranges::none_of(first->pieces, [](const auto &piece) {
            return piece.primaryCommand == 2
                && piece.kind == ngc::PreparedPieceKind::RetainedLineSection;
        }), "the deferred anchor retained section must not be constructed or sampled");

        const ngc::ContinuousGeometryBoundaries rollingMiddle{
            .incomingReplacement = true,
            .deferFinalRetainedSection = true,
        };
        const auto second = ngc::prepareContinuousGeometry(
            std::span{records}.subspan(1, 2), 0.01,
            ngc::position_t{1, 0, 0, 0, 0, 0}, effort, rollingMiddle);
        require(second.has_value(), second ? "" : second.error());
        require(second->pieces.size() == 2
                && second->pieces[0].primaryCommand == 2
                && second->pieces[0].kind == ngc::PreparedPieceKind::RetainedLineSection
                && second->pieces[1].kind == ngc::PreparedPieceKind::JunctionBlend,
                "the next window should prepare the previous anchor exactly when it becomes final");
        requireNear(second->pieces[0].curveFrom, 0.03,
                    "an incoming replacement should trim the retained anchor beginning by 3P");

        ngc::CurveEvaluationWorkspace workspace;
        const auto firstEnd = ngc::positionAtDistance(
            *first->pieces.back().curve, first->pieces.back().curveTo, workspace);
        const auto secondBeginning = ngc::positionAtDistance(
            *second->pieces.front().curve, second->pieces.front().curveFrom, workspace);
        require((firstEnd - secondBeginning).length() <= 1e-12,
                "incremental windows must meet at the ordinary adjacent-piece boundary");

        const ngc::ContinuousGeometryBoundaries finalBoundary{
            .incomingReplacement = true,
        };
        const auto final = ngc::prepareContinuousGeometry(
            std::span{records}.last(1), 0.01,
            ngc::position_t{1, 1, 0, 0, 0, 0}, effort, finalBoundary);
        require(final.has_value(), final ? "" : final.error());
        require(final->pieces.size() == 1
                && final->pieces.front().primaryCommand == 3
                && final->pieces.front().kind == ngc::PreparedPieceKind::RetainedLineSection,
                "the terminal flush should prepare the last deferred retained section once");
        const auto secondEnd = ngc::positionAtDistance(
            *second->pieces.back().curve, second->pieces.back().curveTo, workspace);
        const auto finalBeginning = ngc::positionAtDistance(
            *final->pieces.front().curve, final->pieces.front().curveFrom, workspace);
        require((secondEnd - finalBeginning).length() <= 1e-12,
                "the final deferred retained section must meet its incoming blend directly");
    }

    ngc::AxisPolynomialSpan linearSpan(const ngc::SpanId id, const double from, const double to,
                                       const double duration) {
        ngc::AxisPolynomialSpan span;
        span.id = id;
        span.duration = duration;
        span.inverseDuration = 1.0 / duration;
        span.inverseDurationSquared = span.inverseDuration * span.inverseDuration;
        span.inverseDurationCubed = span.inverseDurationSquared * span.inverseDuration;
        span.origin.x = from;
        span.coefficients[0].x = to - from;
        return span;
    }

    void testMockMotionBackendUsesProductionTransportContract() {
        ngc::MockMotionBackend backend;
        ngc::PlanChunk invalid;
        require(backend.tryPublish(invalid) == ngc::PublishResult::Invalid,
                "backend should reject an incomplete trajectory chunk");

        ngc::PlanChunk chunk;
        chunk.epoch = 7;
        chunk.id = 11;
        chunk.branch = 21;
        require(chunk.normalMotion.push(linearSpan(100, 0.0, 1.0, 1.0)),
                "test normal span should fit in a chunk");
        auto stop = linearSpan(101, 1.0, 1.0, 0.25);
        require(chunk.stopTail.push(stop), "test stop tail should fit in a chunk");
        chunk.branchState.position.x = 1.0;
        chunk.stopState.position.x = 1.0;

        require(backend.tryPublish(chunk) == ngc::PublishResult::Published,
                "NRT should publish a validated chunk without invoking execution");
        require(backend.trySubmit(ngc::StartRequest { 1, 7 }) == ngc::SubmitResult::Submitted,
                "NRT should submit control through its independent SPSC channel");

        backend.advance(0.5);
        ngc::ExecutionSnapshot snapshot;
        require(backend.tryTakeSnapshot(snapshot), "mock backend should publish an observational snapshot");
        require(snapshot.state == ngc::BackendState::Running, "start request should place backend in running state");
        requireNear(snapshot.commanded.position.x, 0.5, "mock RT backend should evaluate the timed polynomial");

        backend.runUntilIdle();
        bool sawStopBranch = false;
        bool sawHeld = false;
        ngc::ExecutionEvent event;
        while(backend.tryTakeEvent(event)) {
            if(const auto *branch = std::get_if<ngc::BranchSelected>(&event)) {
                sawStopBranch = branch->choice == ngc::BranchChoice::Stop;
            }
            if(std::holds_alternative<ngc::BackendHeld>(event)) sawHeld = true;
        }
        require(sawStopBranch, "backend should irrevocably select its stop tail without a continuation");
        require(sawHeld, "backend should report held only after completing the stop tail");
        const auto trajectory = backend.trajectorySnapshot();
        require(trajectory.revision > 0 && trajectory.spans.size() == 2,
                "normal and stop-tail execution should retain backend-calculated position buffers");
        require(trajectory.spans.front().positions.size() >= 2,
                "mock diagnostics should retain backend-calculated positions");
        requireNear(trajectory.spans.front().positions.back().x, 1.0,
                    "mock diagnostics should end normal motion at its terminal position");
        require(trajectory.spans.back().stopTail,
                "mock diagnostics should distinguish the executed stop tail");
    }

    void testMockBackendEmitsOrderedInSpanExecutionMarkers() {
        ngc::MockMotionBackend backend;
        ngc::PlanChunk chunk;
        chunk.epoch = 8;
        chunk.id = 12;
        chunk.branch = 22;
        require(chunk.normalMotion.push(
                    linearSpan(110, 0.0, 1.0, 1.0))
                && chunk.normalMotion.push(
                    linearSpan(111, 1.0, 2.0, 1.0)),
                "execution-marker test normal spans should fit");
        auto stop = linearSpan(112, 2.0, 2.0, 1e-6);
        require(chunk.stopTail.push(stop),
                "execution-marker test stop tail should fit");
        chunk.branchState = ngc::executionSpanEnd(chunk.normalMotion[1]);
        chunk.stopState.position.x = 2.0;
        const std::array markers{
            ngc::ExecutionMarker{1, 0, 0.0},
            ngc::ExecutionMarker{2, 0, 0.25},
            ngc::ExecutionMarker{3, 0, 0.25},
            ngc::ExecutionMarker{4, 0, 1.0},
            ngc::ExecutionMarker{5, 1, 0.0},
            ngc::ExecutionMarker{6, 1, 0.75},
        };
        for (const auto &marker : markers) {
            require(chunk.markers.push(marker),
                    "execution marker should fit in its packet");
        }

        auto unordered = chunk;
        std::swap(unordered.markers[1], unordered.markers[4]);
        require(backend.tryPublish(unordered) == ngc::PublishResult::Invalid,
                "backend should reject unordered execution markers");
        require(backend.tryPublish(chunk) == ngc::PublishResult::Published,
                "backend should accept ordered in-span execution markers");
        require(backend.trySubmit(ngc::StartRequest{1, 8})
                    == ngc::SubmitResult::Submitted,
                "execution-marker test should submit start");

        const auto takeMarkers = [&] {
            std::vector<ngc::ExecutionMarkerReached> result;
            ngc::ExecutionEvent event;
            while (backend.tryTakeEvent(event)) {
                if (const auto *marker =
                        std::get_if<ngc::ExecutionMarkerReached>(&event)) {
                    result.push_back(*marker);
                }
            }
            return result;
        };

        backend.advanceTick(0.0, true);
        auto reached = takeMarkers();
        require(reached.size() == 1 && reached[0].marker == 1
                && reached[0].span == 110
                && reached[0].parameter == 0.0,
                "a parameter-zero marker should fire when its packet starts");

        backend.advanceTick(0.3, true);
        reached = takeMarkers();
        require(reached.size() == 2 && reached[0].marker == 2
                && reached[1].marker == 3,
                "coincident in-span markers should fire once in source order");

        backend.advanceTick(0.8, true);
        reached = takeMarkers();
        require(reached.size() == 2 && reached[0].marker == 4
                && reached[1].marker == 5
                && reached[0].span == 110
                && reached[1].span == 111,
                "one servo advance should preserve markers on both sides "
                "of a crossed span boundary");

        backend.advanceTick(0.65, true);
        reached = takeMarkers();
        require(reached.size() == 1 && reached[0].marker == 6
                && reached[0].span == 111,
                "an interior marker in the next span should fire at its "
                "normalized trajectory progress");
    }

    void testContinuousMarkerBoundPacketsExecuteWithoutIntermediateStops() {
        constexpr std::size_t COMMANDS = 600;
        constexpr std::size_t COMMANDS_PER_SOURCE = 200;
        const std::array points{
            ngc::position_t{},
            ngc::position_t{4.0, 0, 0, 0, 0, 0},
            ngc::position_t{8.0, 1.0, 0, 0, 0, 0},
            ngc::position_t{12.0, 0, 0, 0, 0, 0},
        };
        std::array<ngc::PreparedCommandRecord, 3> sourceCommands;
        for (std::size_t source = 0; source < sourceCommands.size(); ++source) {
            sourceCommands[source].id = source * COMMANDS_PER_SOURCE + 1;
            sourceCommands[source].command =
                ngc::MoveLine{points[source], points[source + 1], 2.0};
        }
        const auto prepared = ngc::prepareContinuousGeometry(
            sourceCommands, 0.01, points.front());
        require(prepared.has_value(), prepared ? "" : prepared.error());

        auto geometry = *prepared;
        geometry.commands.clear();
        geometry.commands.reserve(COMMANDS);
        for (std::size_t command = 1; command <= COMMANDS; ++command) {
            auto record =
                sourceCommands[(command - 1) / COMMANDS_PER_SOURCE];
            record.id = command;
            geometry.commands.push_back(std::move(record));
        }
        for (std::size_t source = 0; source < sourceCommands.size(); ++source) {
            const auto primary = source * COMMANDS_PER_SOURCE + 1;
            const auto retained = std::ranges::find_if(
                geometry.pieces, [&](const auto &piece) {
                    return piece.kind
                            == ngc::PreparedPieceKind::RetainedLineSection
                        && piece.primaryCommand == primary;
                });
            require(retained != geometry.pieces.end(),
                    "marker-bound packetization fixture should retain every "
                    "source line");
            retained->activationStations.reserve(COMMANDS_PER_SOURCE - 1);
            for (std::size_t offset = 1;
                    offset < COMMANDS_PER_SOURCE; ++offset) {
                retained->activationStations.push_back({
                    .command = primary + offset,
                    .curveDistance = retained->curveFrom + retained->length()
                        * static_cast<double>(offset)
                        / static_cast<double>(COMMANDS_PER_SOURCE),
                });
            }
        }

        const ngc::TrajectoryLimits limits{
            .pathAcceleration = 4.0,
            .rapidSpeed = 10.0,
            .arcChordTolerance = 0.0001,
            .pathJerk = 20.0,
            .axisVelocity = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0},
            .axisAcceleration = {4.0, 4.0, 4.0, 4.0, 4.0, 4.0},
            .axisJerk = {20.0, 20.0, 20.0, 20.0, 20.0, 20.0},
        };
        ngc::TrajectoryCompiler compiler(limits);
        compiler.reset(95, points.front());
        const auto planned = compiler.compileContinuous(geometry, 0.01);
        require(planned && *planned, planned ? "" : planned.error());
        auto normalSpans = std::size_t{0};
        for (const auto &chunk : (*planned)->chunks) {
            normalSpans += chunk.normalMotion.size;
        }
        const auto spanBoundPacketCount =
            (normalSpans + ngc::MAX_NORMAL_SPANS_PER_CHUNK - 1)
            / ngc::MAX_NORMAL_SPANS_PER_CHUNK;
        require((*planned)->chunks.size() > spanBoundPacketCount
                    && (*planned)->executionMarkers == COMMANDS,
                "dense prepared activations should force marker-bound "
                "continuous packetization");

        std::vector<ngc::ExecutionMarkerId> expectedMarkers;
        expectedMarkers.reserve(COMMANDS);
        for (std::size_t chunk = 0; chunk < (*planned)->chunks.size(); ++chunk) {
            const auto &packet = (*planned)->chunks[chunk];
            for (const auto &marker : packet.markers) {
                expectedMarkers.push_back(marker.id);
            }
            require(packet.markers.size
                        <= ngc::MAX_EXECUTION_MARKERS_PER_CHUNK,
                    "every continuous packet must respect marker capacity");
            if (chunk + 1 < (*planned)->chunks.size()) {
                const auto &continuation = (*planned)->chunks[chunk + 1];
                require(packet.stopTail.size > 0
                            && continuation.predecessorBranch == packet.branch,
                        "every moving intermediate packet must retain a stop "
                        "tail and dependent continuation");
                const auto continuationStart =
                    ngc::executionSpanStart(continuation.normalMotion[0]);
                require((packet.branchState.position
                            - continuationStart.position).length() < 1e-10
                            && (packet.branchState.velocity
                                - continuationStart.velocity).length() < 1e-10
                            && (packet.branchState.acceleration
                                - continuationStart.acceleration).length()
                                < 1e-9,
                        "continuous packet boundaries must preserve PVA");
            }
        }
        require(expectedMarkers.size() == COMMANDS,
                "packet marker arrays should retain every prepared activation");

        ngc::MockMotionBackend backend({}, limits);
        for (const auto &packet : (*planned)->chunks) {
            require(backend.tryPublish(packet)
                        == ngc::PublishResult::Published,
                    "mock backend should accept every dependent continuous "
                    "packet");
        }
        require(backend.trySubmit(ngc::StartRequest{
                    (*planned)->chunks.front().id,
                    (*planned)->chunks.front().epoch,
                }) == ngc::SubmitResult::Submitted,
                "marker-bound continuous execution should start");

        backend.runUntilIdle();
        std::vector<ngc::ExecutionMarkerId> reachedMarkers;
        auto intermediateStops = std::size_t{0};
        auto faults = std::size_t{0};
        ngc::ExecutionEvent event;
        while (backend.tryTakeEvent(event)) {
            if (const auto *marker =
                    std::get_if<ngc::ExecutionMarkerReached>(&event)) {
                reachedMarkers.push_back(marker->marker);
            } else if (const auto *branch =
                           std::get_if<ngc::BranchSelected>(&event);
                       branch && branch->choice == ngc::BranchChoice::Stop
                       && branch->branch
                           != (*planned)->chunks.back().branch) {
                ++intermediateStops;
            } else if (std::holds_alternative<ngc::BackendFault>(event)) {
                ++faults;
            }
        }

        require(reachedMarkers == expectedMarkers,
                "mock execution should emit every marker exactly once in "
                "prepared order across packet boundaries");
        require(intermediateStops == 0 && faults == 0,
                "dependent marker-bound packets should execute without an "
                "intermediate stop or backend fault");
    }

    void testMockBackendFeedHoldBrakesAlongActiveTrajectory() {
        ngc::TrajectoryLimits limits;
        limits.pathAcceleration = 4.0;
        limits.axisAcceleration = ngc::position_t { 4.0, 4.0, 4.0, 4.0, 4.0, 4.0 };
        ngc::MockMotionBackend backend(
            ngc::FeedHoldConfiguration { 2.0, 10.0 }, limits);

        ngc::PlanChunk chunk;
        chunk.epoch = 17;
        chunk.id = 31;
        chunk.branch = 41;
        require(chunk.normalMotion.push(linearSpan(201, 0.0, 10.0, 10.0)),
                "feed-hold test normal span should fit");
        auto stop = linearSpan(202, 10.0, 10.0, 1e-6);
        require(chunk.stopTail.push(stop), "feed-hold test stop tail should fit");
        chunk.branchState = ngc::executionSpanEnd(chunk.normalMotion[0]);
        chunk.stopState.position.x = 10.0;
        require(chunk.markers.push({7, 0, 0.2}),
                "feed-hold execution marker should fit");

        require(backend.tryPublish(chunk) == ngc::PublishResult::Published,
                "feed-hold test chunk should publish");
        require(backend.trySubmit(ngc::StartRequest { 1, 17 }) == ngc::SubmitResult::Submitted,
                "feed-hold test should submit start");
        backend.advanceTick(0.5, true);
        ngc::ExecutionSnapshot snapshot;
        while(backend.tryTakeSnapshot(snapshot)) { }
        const auto holdStart = snapshot.commanded;
        requireNear(holdStart.position.x, 0.5,
                    "feed hold should begin from the currently executed position");

        require(backend.trySubmit(ngc::FeedHoldRequest { 2 }) == ngc::SubmitResult::Submitted,
                "feed-hold request should fit in the control channel");
        bool sawHolding = false;
        bool sawVelocityReduction = false;
        for(int tick = 0; tick < 5000 && snapshot.state != ngc::BackendState::Held; ++tick) {
            backend.advanceTick(0.001, true);
            while(backend.tryTakeSnapshot(snapshot)) {
                sawHolding = sawHolding || snapshot.state == ngc::BackendState::Holding;
                sawVelocityReduction = sawVelocityReduction
                    || (snapshot.commanded.velocity.x > 0.0
                        && snapshot.commanded.velocity.x < holdStart.velocity.x - 1e-8);
                require(std::abs(snapshot.commanded.acceleration.x) <= 4.0 + 1e-8,
                        "feed hold should preserve the configured X acceleration limit");
                require(snapshot.executionRate >= -1e-12 && snapshot.executionRate <= 1.0 + 1e-12,
                        "feed-hold execution rate should remain bounded");
            }
        }
        require(sawHolding, "feed hold should expose a moving Holding state");
        require(sawVelocityReduction, "feed hold should reduce velocity before becoming held");
        require(snapshot.state == ngc::BackendState::Held,
                "feed hold should reach a stationary backend state");
        require(snapshot.commanded.position.x > holdStart.position.x
                && snapshot.commanded.position.x < 10.0,
                "feed hold should brake forward on normal motion before its branch");
        require(std::abs(snapshot.commanded.velocity.x) <= 1e-12
                && std::abs(snapshot.commanded.acceleration.x) <= 1e-12,
                "BackendHeld should publish zero velocity and acceleration");

        bool sawFeedHeld = false;
        bool selectedStopTail = false;
        bool markerReachedBeforeResume = false;
        ngc::ExecutionEvent event;
        while(backend.tryTakeEvent(event)) {
            if(const auto *held = std::get_if<ngc::BackendHeld>(&event))
                sawFeedHeld = held->reason == ngc::BackendHoldReason::FeedHold;
            if(const auto *branch = std::get_if<ngc::BranchSelected>(&event))
                selectedStopTail = branch->choice == ngc::BranchChoice::Stop;
            if (const auto *marker =
                    std::get_if<ngc::ExecutionMarkerReached>(&event)) {
                markerReachedBeforeResume =
                    markerReachedBeforeResume || marker->marker == 7;
            }
        }
        require(sawFeedHeld, "feed hold should identify its held-state reason");
        require(!selectedStopTail,
                "a feed hold completed inside normal motion should not select the stop tail");
        require(!markerReachedBeforeResume,
                "feed hold should not emit a marker beyond its held "
                "trajectory cursor");

        const auto heldPosition = snapshot.commanded.position.x;
        require(backend.trySubmit(ngc::ResumeRequest { 3, 17 }) == ngc::SubmitResult::Submitted,
                "held normal motion should accept a resume request");
        bool sawResumeAcceleration = false;
        bool resumeAccepted = false;
        auto markerReachCount = 0U;
        for(int tick = 0; tick < 5000
            && !(snapshot.state == ngc::BackendState::Running
                 && snapshot.executionRate >= 1.0 - 1e-10); ++tick) {
            backend.advanceTick(0.001, true);
            while(backend.tryTakeSnapshot(snapshot)) {
                sawResumeAcceleration = sawResumeAcceleration
                    || (snapshot.executionRate > 0.0
                        && snapshot.executionRate < 1.0
                        && snapshot.executionRateAcceleration > 0.0);
                require(snapshot.commanded.position.x + 1e-12 >= heldPosition,
                        "feed resume should continue from the held normal-motion cursor");
                require(std::abs(snapshot.commanded.acceleration.x) <= 4.0 + 1e-8,
                        "feed resume should preserve the configured X acceleration limit");
            }
            while(backend.tryTakeEvent(event)) {
                if(const auto *completed = std::get_if<ngc::RequestCompleted>(&event))
                    if(completed->request == 3) resumeAccepted = completed->succeeded;
                if (const auto *marker =
                        std::get_if<ngc::ExecutionMarkerReached>(&event)) {
                    if (marker->marker == 7) {
                        ++markerReachCount;
                    }
                }
            }
        }
        require(resumeAccepted, "backend should acknowledge feed resume");
        require(sawResumeAcceleration,
                "feed resume should accelerate through an intermediate execution rate");
        require(snapshot.state == ngc::BackendState::Running
                && snapshot.executionRate >= 1.0 - 1e-10,
                "feed resume should restore the normal execution rate");
        require(snapshot.commanded.position.x > heldPosition,
                "feed resume should advance along the retained normal branch");
        for (auto tick = 0; tick < 5000 && markerReachCount == 0; ++tick) {
            backend.advanceTick(0.001, true);
            while (backend.tryTakeEvent(event)) {
                if (const auto *marker =
                        std::get_if<ngc::ExecutionMarkerReached>(&event)) {
                    if (marker->marker == 7) {
                        ++markerReachCount;
                    }
                }
            }
        }
        require(markerReachCount == 1,
                "feed resume should emit a retained in-span marker exactly "
                "once when the trajectory cursor reaches it");

        const auto samples = backend.takeExecutedJerkSamples();
        require(std::ranges::any_of(samples, [](const auto &sample) {
                    return sample.feedHolding && sample.executionRate < 1.0;
                }),
                "mock servo telemetry should retain feed-hold execution-rate samples");
        require(std::ranges::any_of(samples, [](const auto &sample) {
                    return !sample.feedHolding && sample.executionRate > 0.0
                        && sample.executionRate < 1.0
                        && sample.executionRateAcceleration > 0.0;
                }),
                "mock servo telemetry should retain feed-resume acceleration samples");
    }

    void testMockBackendFeedHoldStopBranchIsFatal() {
        ngc::TrajectoryLimits limits;
        limits.pathAcceleration = 4.0;
        limits.axisAcceleration = ngc::position_t { 4.0, 4.0, 4.0, 4.0, 4.0, 4.0 };
        ngc::MockMotionBackend backend(
            ngc::FeedHoldConfiguration { 0.01, 0.1 }, limits);

        ngc::PlanChunk chunk;
        chunk.epoch = 18;
        chunk.id = 32;
        chunk.branch = 42;
        require(chunk.normalMotion.push(linearSpan(203, 0.0, 0.1, 0.1)),
                "feed-hold branch-fault normal span should fit");
        require(chunk.stopTail.push(linearSpan(204, 0.1, 0.1, 1e-6)),
                "feed-hold branch-fault stop tail should fit");
        chunk.branchState = ngc::executionSpanEnd(chunk.normalMotion[0]);
        chunk.stopState = chunk.branchState;

        require(backend.tryPublish(chunk) == ngc::PublishResult::Published,
                "feed-hold branch-fault chunk should publish");
        require(backend.trySubmit(ngc::StartRequest { 4, 18 }) == ngc::SubmitResult::Submitted,
                "feed-hold branch-fault test should submit start");
        backend.advanceTick(0.09, true);
        require(backend.trySubmit(ngc::FeedHoldRequest { 5 }) == ngc::SubmitResult::Submitted,
                "feed-hold branch-fault request should fit");

        ngc::ExecutionSnapshot snapshot;
        for(int tick = 0; tick < 1000 && snapshot.state != ngc::BackendState::Faulted; ++tick) {
            backend.advanceTick(0.001, true);
            while(backend.tryTakeSnapshot(snapshot)) { }
        }
        require(snapshot.state == ngc::BackendState::Faulted && snapshot.faultCode == 5,
                "feed hold reaching a stop branch should be a fatal backend condition");

        bool selectedStop = false;
        bool sawFault = false;
        bool sawHeld = false;
        ngc::ExecutionEvent event;
        while(backend.tryTakeEvent(event)) {
            if(const auto *branch = std::get_if<ngc::BranchSelected>(&event))
                selectedStop = branch->choice == ngc::BranchChoice::Stop;
            if(const auto *fault = std::get_if<ngc::BackendFault>(&event))
                sawFault = fault->code == 5;
            sawHeld = sawHeld || std::holds_alternative<ngc::BackendHeld>(event);
        }
        require(selectedStop && sawFault && !sawHeld,
                "feed-hold branch exhaustion should report STOP and fault without entering Held");
        require(backend.trySubmit(ngc::ResumeRequest { 6, 18 }) == ngc::SubmitResult::Submitted,
                "post-fault resume request should fit for rejection testing");
        backend.advanceTick(0.0, true);
        bool resumeRejected = false;
        while(backend.tryTakeEvent(event))
            if(const auto *completed = std::get_if<ngc::RequestCompleted>(&event))
                if(completed->request == 6) resumeRejected = !completed->succeeded;
        require(resumeRejected, "feed resume should be rejected after stop-branch failure");
    }

    void testMockBackendFeedHoldPausesAndResumesProbeApproach() {
        ngc::MockMotionBackend backend;
        const ngc::TriggeredMove probe {
            .epoch = 19,
            .id = 33,
            .branch = 43,
            .moveId = 53,
            .target = { 10.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
            .limits = {
                .velocity = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
                .acceleration = { 2.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
                .jerk = { 10.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
            },
            .input = 0,
            .condition = ngc::InputCondition::Active,
            .triggerRequired = false,
        };
        require(backend.tryPublish(probe) == ngc::PublishResult::Published,
                "probe feed-hold fixture should publish");
        require(backend.trySubmit(ngc::StartRequest { 7, 19 }) == ngc::SubmitResult::Submitted,
                "probe feed-hold fixture should start");
        backend.advanceTick(0.5, true);
        ngc::ExecutionSnapshot snapshot;
        while(backend.tryTakeSnapshot(snapshot)) { }
        require(snapshot.state == ngc::BackendState::Running
                && snapshot.commanded.velocity.x > 0.0,
                "probe approach should be moving before feed hold");

        require(backend.trySubmit(ngc::FeedHoldRequest { 8 }) == ngc::SubmitResult::Submitted,
                "moving probe approach should accept feed hold");
        for(int tick = 0; tick < 5000 && snapshot.state != ngc::BackendState::Held; ++tick) {
            backend.advanceTick(0.001, true);
            while(backend.tryTakeSnapshot(snapshot)) { }
        }
        require(snapshot.state == ngc::BackendState::Held
                && snapshot.commanded.velocity.length() <= 1e-12
                && snapshot.commanded.acceleration.length() <= 1e-12,
                "probe feed hold should reach a stationary held state");
        const auto heldPosition = snapshot.commanded.position.x;

        bool sawFeedHeld = false;
        bool completedProbe = false;
        bool selectedBranch = false;
        bool holdAccepted = false;
        ngc::ExecutionEvent event;
        while(backend.tryTakeEvent(event)) {
            if(const auto *held = std::get_if<ngc::BackendHeld>(&event))
                sawFeedHeld = held->reason == ngc::BackendHoldReason::FeedHold;
            completedProbe = completedProbe
                || std::holds_alternative<ngc::TriggeredMoveCompleted>(event);
            selectedBranch = selectedBranch || std::holds_alternative<ngc::BranchSelected>(event);
            if(const auto *completed = std::get_if<ngc::RequestCompleted>(&event))
                if(completed->request == 8) holdAccepted = completed->succeeded;
        }
        require(holdAccepted && sawFeedHeld && !completedProbe && !selectedBranch,
                std::format("holding a probe should retain it without reporting completion or a "
                            "branch (accepted={} held={} completed={} branch={} position={})",
                            holdAccepted, sawFeedHeld, completedProbe, selectedBranch,
                            snapshot.commanded.position.x));

        require(backend.trySubmit(ngc::ResumeRequest { 9, 19 }) == ngc::SubmitResult::Submitted,
                "held probe approach should accept resume");
        bool resumeAccepted = false;
        for(int tick = 0; tick < 5000
            && !(snapshot.state == ngc::BackendState::Running
                 && snapshot.commanded.velocity.x > 1e-4); ++tick) {
            backend.advanceTick(0.001, true);
            while(backend.tryTakeSnapshot(snapshot)) { }
            while(backend.tryTakeEvent(event))
                if(const auto *completed = std::get_if<ngc::RequestCompleted>(&event))
                    if(completed->request == 9) resumeAccepted = completed->succeeded;
        }
        require(resumeAccepted && snapshot.state == ngc::BackendState::Running
                && snapshot.commanded.velocity.x > 1e-4,
                "probe resume should generate a new moving approach from rest");
        require(snapshot.commanded.position.x >= heldPosition && snapshot.commanded.position.x < 10.0,
                "probe resume should continue toward the original target from its held position");
    }

    void testMockBackendProbeContactDuringFeedHoldStopIsDetected() {
        ngc::MockMotionBackend backend;
        const ngc::TriggeredMove probe {
            .epoch = 20,
            .id = 34,
            .branch = 44,
            .moveId = 54,
            .target = { 10.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
            .limits = {
                .velocity = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
                .acceleration = { 2.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
                .jerk = { 10.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
            },
            .input = 0,
            .condition = ngc::InputCondition::Active,
            .triggerRequired = true,
        };
        require(backend.tryPublish(probe) == ngc::PublishResult::Published,
                "feed-hold contact fixture should publish");
        require(backend.trySubmit(ngc::StartRequest { 10, 20 }) == ngc::SubmitResult::Submitted,
                "feed-hold contact fixture should start");
        backend.advanceTick(0.5, true);
        ngc::ExecutionSnapshot snapshot;
        while(backend.tryTakeSnapshot(snapshot)) { }
        require(snapshot.commanded.velocity.x > 0.0,
                "feed-hold contact fixture should be moving before hold");
        const auto contactPosition = snapshot.commanded.position.x + 0.01;
        require(backend.configureSyntheticInput(
                    probe.moveId, { contactPosition, 0.0, 0.0, 0.0, 0.0, 0.0 }),
                "feed-hold contact transition should fit");
        require(backend.trySubmit(ngc::FeedHoldRequest { 11 }) == ngc::SubmitResult::Submitted,
                "feed-hold contact request should fit");

        for(int tick = 0; tick < 5000 && snapshot.state != ngc::BackendState::Held; ++tick) {
            backend.advanceTick(0.001, true);
            while(backend.tryTakeSnapshot(snapshot)) { }
        }
        require(snapshot.state == ngc::BackendState::Held,
                "contact during feed-hold braking should finish in Held");

        bool holdAccepted = false;
        bool sawFeedHeld = false;
        bool selectedStop = false;
        std::optional<ngc::TriggeredMoveCompleted> completion;
        ngc::ExecutionEvent event;
        while(backend.tryTakeEvent(event)) {
            if(const auto *completed = std::get_if<ngc::RequestCompleted>(&event))
                if(completed->request == 11) holdAccepted = completed->succeeded;
            if(const auto *held = std::get_if<ngc::BackendHeld>(&event))
                sawFeedHeld = sawFeedHeld
                    || held->reason == ngc::BackendHoldReason::FeedHold;
            if(const auto *branch = std::get_if<ngc::BranchSelected>(&event))
                selectedStop = selectedStop || branch->choice == ngc::BranchChoice::Stop;
            if(const auto *triggered = std::get_if<ngc::TriggeredMoveCompleted>(&event))
                completion = *triggered;
        }
        require(holdAccepted, "probe feed hold should be accepted before contact");
        require(completion && completion->status == ngc::TriggeredMoveStatus::Triggered,
                "contact sampled during feed-hold braking should complete the probe as triggered");
        require(completion->triggerState.position.x + 1e-12 >= contactPosition
                && completion->triggerState.position.x <= contactPosition + 0.0011,
                "probe contact should latch at the first servo sample crossing the transition");
        require(selectedStop && !sawFeedHeld,
                "probe contact should supersede feed hold and complete through the probe branch");
    }

    void testSimulationWorkerFeedHoldReachesPausedAtRest() {
        const auto configuration = ngc::loadMachineConfiguration("machine.toml");
        require(configuration.has_value(), configuration ? "" : configuration.error());
        SimulationWorker worker(*configuration);
        ngc::ToolTable tools;
        const std::vector<std::tuple<std::string, std::string>> program {
            { "G1 F60 G53 X10\n", "feed-hold-worker.ngc" },
        };
        require(worker.start(program, tools, true),
                "feed-hold worker regression should start a program");

        auto snapshot = worker.snapshot();
        for(int attempt = 0; attempt < 10000
            && !(snapshot.status == ngc::SimulationStatus::Running
                 && snapshot.trajectoryBackendVelocity > 1e-4); ++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            snapshot = worker.snapshot();
        }
        require(snapshot.status == ngc::SimulationStatus::Running
                && snapshot.trajectoryBackendVelocity > 1e-4,
                "feed-hold worker regression should observe active program motion");
        const auto holdStart = snapshot.machinePosition.x;
        require(worker.feedHold(), "active program motion should accept Feed Hold");

        bool sawHolding = false;
        for(int attempt = 0; attempt < 10000
            && snapshot.status != ngc::SimulationStatus::Paused
            && snapshot.status != ngc::SimulationStatus::Error; ++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            snapshot = worker.snapshot();
            sawHolding = sawHolding || snapshot.status == ngc::SimulationStatus::Holding;
        }
        require(snapshot.status == ngc::SimulationStatus::Paused,
                std::format("feed hold should reach Paused rather than freezing or failing: {}",
                            snapshot.error));
        require(sawHolding, "SimulationWorker should expose Holding while the backend brakes");
        require(snapshot.machinePosition.x > holdStart && snapshot.machinePosition.x < 10.0,
                "SimulationWorker feed hold should stop forward on the active path");
        require(snapshot.trajectoryBackendVelocity <= 1e-10
                && snapshot.trajectoryBackendAcceleration <= 1e-10,
                "SimulationWorker should publish Paused only at rest");
        const auto heldPosition = snapshot.machinePosition.x;
        require(worker.resume(), "a completed feed hold should accept Resume");
        require(!worker.feedHold(),
                "a second feed hold should wait until resume restores normal execution rate");
        for(int attempt = 0; attempt < 10000
            && !(snapshot.status == ngc::SimulationStatus::Running
                 && snapshot.trajectoryBackendVelocity > 1e-4); ++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            snapshot = worker.snapshot();
        }
        require(snapshot.status == ngc::SimulationStatus::Running
                && snapshot.trajectoryBackendVelocity > 1e-4,
                std::format("feed resume should return to moving Running state: {}",
                            snapshot.error));
        require(snapshot.machinePosition.x >= heldPosition && snapshot.machinePosition.x < 10.0,
                "feed resume should continue forward from the held path cursor");
        worker.stop();
        worker.join();
    }

    void testSimulationWorkerFeedHoldResumesProbeApproach() {
        const auto configuration = ngc::loadMachineConfiguration("machine.toml");
        require(configuration.has_value(), configuration ? "" : configuration.error());
        SimulationWorker worker(*configuration);
        ngc::ToolTable tools;
        const std::vector<std::tuple<std::string, std::string>> program {
            { "G38.3 F60 X10\n", "feed-hold-probe-worker.ngc" },
        };
        require(worker.start(program, tools, true),
                "probe feed-hold worker regression should start");

        auto snapshot = worker.snapshot();
        for(int attempt = 0; attempt < 10000
            && !(snapshot.status == ngc::SimulationStatus::Running
                 && snapshot.trajectoryBackendVelocity > 1e-4); ++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            snapshot = worker.snapshot();
        }
        require(snapshot.status == ngc::SimulationStatus::Running
                && snapshot.trajectoryBackendVelocity > 1e-4,
                std::format("probe approach should begin moving before feed hold: {}",
                            snapshot.error));
        require(snapshot.trajectoryBackendSpan == 0,
                "triggered probe motion should remain distinct from normal execution spans");
        const auto holdStart = snapshot.machinePosition.x;
        require(worker.feedHold(), "moving probe approach should accept Feed Hold");
        for(int attempt = 0; attempt < 10000
            && snapshot.status != ngc::SimulationStatus::Paused
            && snapshot.status != ngc::SimulationStatus::Error; ++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            snapshot = worker.snapshot();
        }
        require(snapshot.status == ngc::SimulationStatus::Paused,
                std::format("probe feed hold should reach Paused: {}", snapshot.error));
        require(snapshot.machinePosition.x >= holdStart && snapshot.machinePosition.x < 10.0,
                "probe feed hold should stop before the probe target");
        const auto heldPosition = snapshot.machinePosition.x;

        require(worker.resume(), "held probe approach should accept Resume");
        for(int attempt = 0; attempt < 10000
            && !(snapshot.status == ngc::SimulationStatus::Running
                 && snapshot.trajectoryBackendVelocity > 1e-4); ++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            snapshot = worker.snapshot();
        }
        require(snapshot.status == ngc::SimulationStatus::Running
                && snapshot.trajectoryBackendVelocity > 1e-4,
                std::format("resumed probe approach should return to moving Running state: {}",
                            snapshot.error));
        require(snapshot.machinePosition.x >= heldPosition && snapshot.machinePosition.x < 10.0,
                "resumed probe should continue toward its original target");
        worker.stop();
        worker.join();
    }

    void testSimulationWorkerProbeContactSupersedesFeedHold() {
        const auto configuration = ngc::loadMachineConfiguration("machine.toml");
        require(configuration.has_value(), configuration ? "" : configuration.error());
        SimulationWorker worker(*configuration);
        ngc::ToolTable tools;
        const std::vector<std::tuple<std::string, std::string>> program {
            { "G38.3 F200 X13\n", "feed-hold-probe-contact-worker.ngc" },
        };
        require(worker.start(program, tools, true),
                "probe-contact feed-hold worker regression should start");

        auto snapshot = worker.snapshot();
        for(int attempt = 0; attempt < 10000
            && !(snapshot.status == ngc::SimulationStatus::Running
                 && snapshot.machinePosition.x >= 12.85
                 && snapshot.trajectoryBackendVelocity > 1.0); ++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            snapshot = worker.snapshot();
        }
        require(snapshot.status == ngc::SimulationStatus::Running
                && snapshot.machinePosition.x >= 12.85
                && snapshot.machinePosition.x < 13.0,
                std::format("probe should approach contact before the late feed hold: {}",
                            snapshot.error));
        require(worker.feedHold(), "late probe feed hold should be accepted");

        bool sawPaused = false;
        for(int attempt = 0; attempt < 10000
            && snapshot.status != ngc::SimulationStatus::Completed
            && snapshot.status != ngc::SimulationStatus::Error; ++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            snapshot = worker.snapshot();
            sawPaused = sawPaused || snapshot.status == ngc::SimulationStatus::Paused;
        }
        require(snapshot.status == ngc::SimulationStatus::Completed,
                std::format("probe contact during feed-hold braking should complete normally: {}",
                            snapshot.error));
        require(!sawPaused,
                "probe contact during feed-hold braking should supersede Paused/Resume state");
        worker.join();
    }

    void testJogControlUsesBoundedBackendTransport() {
        const ngc::JogTarget group {
            .type = ngc::JogTargetType::JointGroup,
            .axis = ngc::AxisId::Y,
            .joints = ngc::JointMask { (1U << 1U) | (1U << 2U) },
        };
        const ngc::StartContinuousJogRequest jog {
            .id = 41,
            .jog = 73,
            .target = group,
            .signedVelocity = -0.5,
            .limits = { .velocity = 1.0, .acceleration = 2.0, .jerk = 10.0 },
            .stopLimits = { .velocity = 1.0, .acceleration = 8.0, .jerk = 40.0 },
            .leaseTicks = 3,
        };
        const ngc::ControlRequest control = jog;
        const auto *transported = std::get_if<ngc::StartContinuousJogRequest>(&control);
        require(transported != nullptr && transported->target.joints == group.joints,
                "one bounded control request should retain the complete atomic joint group");

        ngc::MockMotionBackend backend;
        require(backend.trySubmit(ngc::ResetRequest { 1, 9 }) == ngc::SubmitResult::Submitted,
                "jog test reset should fit in the control channel");
        require(backend.trySubmit(ngc::EnableRequest { 2 }) == ngc::SubmitResult::Submitted,
                "jog test enable should fit in the control channel");
        backend.advance(0.0);
        ngc::ExecutionEvent event;
        while(backend.tryTakeEvent(event)) { }

        require(backend.trySubmit(control) == ngc::SubmitResult::Submitted,
                "jog control should use the existing bounded backend control channel");
        backend.advance(0.0);
        for(int tick = 0; tick < 3; ++tick) backend.advanceTick(0.001, true);
        backend.runUntilIdle(0.001);

        bool accepted = false;
        std::optional<ngc::JogStopped> stopped;
        while(backend.tryTakeEvent(event)) {
            if(const auto *completed = std::get_if<ngc::RequestCompleted>(&event))
                accepted = completed->request == jog.id && completed->succeeded;
            if(const auto *jogEvent = std::get_if<ngc::JogStopped>(&event)) stopped = *jogEvent;
        }
        require(accepted, "held mock backend should accept a valid grouped jog");
        require(stopped && stopped->reason == ngc::JogStopReason::LeaseExpired,
                "an unrenewed continuous jog must stop when its backend lease expires");
        require(stopped->jointState.position[1] < 0.0,
                "lease-protected jog should calculate motion before stopping");
        requireNear(stopped->jointState.position[1], stopped->jointState.position[2],
                    "one scalar grouped-jog profile must keep both gantry joints synchronized");
        requireNear(stopped->jointState.velocity[1], 0.0,
                    "lease expiration must finish with a constrained zero-velocity state");

        const ngc::StartIncrementalJogRequest incremental {
            .id = 42,
            .jog = 74,
            .target = group,
            .distance = 0.1,
            .velocity = 0.5,
            .limits = { .velocity = 1.0, .acceleration = 2.0, .jerk = 10.0 },
            .stopLimits = { .velocity = 1.0, .acceleration = 8.0, .jerk = 40.0 },
        };
        require(backend.trySubmit(incremental) == ngc::SubmitResult::Submitted,
                "incremental jog should use the same bounded control channel");
        backend.runUntilIdle(0.001);
        std::optional<ngc::JogStopped> incrementalStopped;
        while(backend.tryTakeEvent(event))
            if(const auto *jogEvent = std::get_if<ngc::JogStopped>(&event)) incrementalStopped = *jogEvent;
        require(incrementalStopped
                    && incrementalStopped->reason == ngc::JogStopReason::TargetReached,
                "incremental jog should report bounded target completion");
        requireNear(incrementalStopped->jointState.position[1],
                    stopped->jointState.position[1] + incremental.distance,
                    "incremental jog should move the requested group distance");

        auto heldJog = jog;
        heldJog.id = 43;
        heldJog.jog = 75;
        heldJog.signedVelocity = 0.5;
        heldJog.leaseTicks = 1000;
        require(backend.trySubmit(heldJog) == ngc::SubmitResult::Submitted,
                "second continuous jog should submit from held state");
        backend.advanceTick(0.01, true);
        require(backend.trySubmit(ngc::SetContinuousJogVelocityRequest {
                    44, heldJog.jog, -0.25 }) == ngc::SubmitResult::Submitted,
                "matching continuous-jog velocity update should use the bounded control channel");
        backend.advance(0.0);
        ngc::ExecutionSnapshot reversed;
        bool haveReversed = false;
        while(backend.tryTakeSnapshot(reversed)) { }
        for(int tick = 0; tick < 500; ++tick) {
            backend.advanceTick(0.001, true);
            while(backend.tryTakeSnapshot(reversed)) haveReversed = true;
        }
        require(haveReversed && reversed.commandedJoints.velocity[1] < 0.0,
                "velocity update should reverse the active stable-token jog through constrained motion");
        require(backend.trySubmit(ngc::StopJogRequest { 45, 999 }) == ngc::SubmitResult::Submitted,
                "stale-token stop should still traverse the bounded control channel");
        require(backend.trySubmit(ngc::StopJogRequest { 46, heldJog.jog }) == ngc::SubmitResult::Submitted,
                "matching stop should traverse the bounded control channel");
        backend.runUntilIdle(0.001);
        bool staleStopRejected = false;
        std::optional<ngc::JogStopped> requestedStop;
        while(backend.tryTakeEvent(event)) {
            if(const auto *completed = std::get_if<ngc::RequestCompleted>(&event))
                if(completed->request == 45) staleStopRejected = !completed->succeeded;
            if(const auto *jogEvent = std::get_if<ngc::JogStopped>(&event)) requestedStop = *jogEvent;
        }
        require(staleStopRejected, "a delayed token must not stop a newer jog");
        require(requestedStop && requestedStop->reason == ngc::JogStopReason::RequestedStop,
                "matching StopJog should produce a constrained requested stop");
    }

    ngc::position_t evaluateSpan(const ngc::AxisPolynomialSpan &span, const double u) {
        return ngc::evaluateExecutionPolynomial(span, u).state.position;
    }

    double spanAcceleration(const ngc::AxisPolynomialSpan &span, const double u) {
        return ngc::evaluateExecutionPolynomial(
            span, u).state.acceleration.length();
    }

    double spanJerk(const ngc::AxisPolynomialSpan &span) {
        return ngc::trajectory_detail::maximumPathJerk(span);
    }

    double spanAxisVelocity(const ngc::AxisPolynomialSpan &span,
                            const double ngc::position_t::*component) {
        return ngc::trajectory_detail::maximumAxisVelocity(span, component);
    }

    double spanAxisAcceleration(const ngc::AxisPolynomialSpan &span,
                                const double ngc::position_t::*component) {
        return ngc::trajectory_detail::maximumAxisAcceleration(
            span, component);
    }

    double spanAxisJerk(const ngc::AxisPolynomialSpan &span,
                        const double ngc::position_t::*component) {
        return ngc::trajectory_detail::maximumAxisJerk(span, component);
    }

    void requireAxisLimits(const ngc::PlanChunk &chunk, const double ngc::position_t::*component,
                           const double velocity, const double acceleration, const double jerk,
                           const std::string_view context) {
        for(const auto &span : chunk.normalMotion) {
            require(spanAxisVelocity(span, component) <= velocity + 1e-8,
                    std::format("{} should respect axis velocity", context));
            require(spanAxisAcceleration(span, component) <= acceleration + 1e-8,
                    std::format("{} should respect axis acceleration", context));
            require(spanAxisJerk(span, component) <= jerk + 1e-8,
                    std::format("{} should respect axis jerk", context));
        }
    }

    void testExactStopPlannerCompilesLinesAndArcs() {
        constexpr double ACCELERATION = 4.0;
        constexpr double JERK = 8.0;
        ngc::TrajectoryCompiler planner({
            .pathAcceleration = ACCELERATION,
            .rapidSpeed = 120.0,
            .arcChordTolerance = 0.0001,
            .pathJerk = JERK,
        });
        planner.reset(9);

        const auto line = planner.compile(ngc::MoveLine { {}, { 2, 0, 0, 0, 0, 0 }, 60.0 });
        require(line.has_value(), line ? "" : line.error());
        const auto lineTimeLaw=ngc::totalTimeLawCalls(planner.lastTimeLawDiagnostics());
        require(lineTimeLaw.calls==1&&lineTimeLaw.successes==1&&lineTimeLaw.failures==0
                    &&lineTimeLaw.solverCalls==1&&lineTimeLaw.cacheHits==0
                    &&planner.lastTimeLawDiagnostics().exactStop.calls==1,
                "exact-stop line should report its successful scalar Ruckig solve");
        require(line->normalMotion.size >= 2, "exact-stop line should contain acceleration and deceleration spans");
        requireNear(evaluateSpan(line->normalMotion[0], 0.0).x, 0.0,
                    "compiled line should begin at the canonical start");
        requireNear(evaluateSpan(line->normalMotion[line->normalMotion.size-1], 1.0).x, 2.0,
                    "compiled line should end at the canonical endpoint");
        requireNear(line->normalMotion[0].coefficients[0].x, 0.0,
                    "exact-stop line should start at zero velocity");
        requireNear(ngc::executionSpanEnd(
                        line->normalMotion[line->normalMotion.size - 1])
                        .velocity.x,
                    0.0,
                    "exact-stop line should finish at zero velocity");
        for(const auto &span : line->normalMotion) {
            require(spanAcceleration(span, 0.0) <= ACCELERATION + 1e-8,
                    "compiled line should respect acceleration at its span start");
            require(spanAcceleration(span, 1.0) <= ACCELERATION + 1e-8,
                    "compiled line should respect acceleration at its span end");
            require(spanJerk(span) <= JERK + 1e-8,
                    "compiled line should respect the configured jerk limit");
        }
        for(std::size_t i = 1; i < line->normalMotion.size; ++i) {
            const auto &previous = line->normalMotion[i-1];
            const auto &current = line->normalMotion[i];
            const auto previousEndAcceleration =
                ngc::executionSpanEnd(previous).acceleration.x;
            const auto currentStartAcceleration =
                ngc::executionSpanStart(current).acceleration.x;
            requireNear(previousEndAcceleration, currentStartAcceleration,
                        "Ruckig phases should have continuous acceleration");
        }

        const auto arc = planner.compile(ngc::MoveArc {
            { 2, 0, 0, 0, 0, 0 }, { 1, 1, 0, 0, 0, 0 }, { 1, 0, 0 }, { 0, 0, 1 }, 60.0 });
        require(arc.has_value(), arc ? "" : arc.error());
        require(arc->normalMotion.size > 2, "arc should be adaptively represented by multiple polynomial spans");
        const auto arcStart = evaluateSpan(arc->normalMotion[0], 0.0);
        const auto arcEnd = evaluateSpan(arc->normalMotion[arc->normalMotion.size-1], 1.0);
        requireNear(arcStart.x, 2.0, "compiled arc should preserve its start X");
        requireNear(arcStart.y, 0.0, "compiled arc should preserve its start Y");
        requireNear(arcEnd.x, 1.0, "compiled arc should preserve its endpoint X");
        requireNear(arcEnd.y, 1.0, "compiled arc should preserve its endpoint Y");
        requireNear(arc->normalMotion[0].coefficients[0].x, 0.0,
                    "exact-stop arc should begin at zero velocity");
        const auto arcTerminal = ngc::executionSpanEnd(
            arc->normalMotion[arc->normalMotion.size - 1]);
        requireNear(arcTerminal.velocity.x, 0.0,
                    "exact-stop arc should end with zero X velocity");
        requireNear(arcTerminal.velocity.y, 0.0,
                    "exact-stop arc should end with zero Y velocity");
        for(const auto &span : arc->normalMotion) {
            require(spanAcceleration(span, 0.0) <= ACCELERATION + 1e-8,
                    "compiled arc should respect acceleration at its span start");
            require(spanAcceleration(span, 1.0) <= ACCELERATION + 1e-8,
                    "compiled arc should respect acceleration at its span end");
            require(spanJerk(span) <= JERK + 1e-8,
                    "compiled arc should respect the configured jerk limit");
        }

        ngc::TrajectoryCompiler instantaneous({
            .pathAcceleration = std::numeric_limits<double>::infinity(),
            .rapidSpeed = 120.0,
            .arcChordTolerance = 0.0001,
        });
        instantaneous.reset(10);
        const auto constantSpeed = instantaneous.compile(
            ngc::MoveLine { {}, { 2, 0, 0, 0, 0, 0 }, 60.0 });
        require(constantSpeed.has_value(), constantSpeed ? "" : constantSpeed.error());
        require(constantSpeed->normalMotion.size == 1,
                "infinite acceleration should compile a line into one constant-speed span");
        requireNear(constantSpeed->normalMotion[0].coefficients[2].x, 0.0,
                    "constant-speed line should have no cubic position term");
        requireNear(constantSpeed->normalMotion[0].coefficients[1].x, 0.0,
                    "constant-speed line should have no quadratic position term");
        requireNear(constantSpeed->normalMotion[0].duration, 2.0,
                    "constant-speed line duration should be distance divided by feed");
    }

    void testExactStopPlannerEnforcesIndependentAxisLimits() {
        constexpr auto infinity = std::numeric_limits<double>::infinity();
        ngc::TrajectoryCompiler planner({
            .pathAcceleration = infinity,
            .rapidSpeed = 600.0,
            .arcChordTolerance = 0.0001,
            .pathJerk = infinity,
            .axisVelocity = { 0.25, 0.5, infinity, 0.1, infinity, infinity },
            .axisAcceleration = { 0.5, 1.0, infinity, 0.2, infinity, infinity },
            .axisJerk = { 2.0, 4.0, infinity, 0.5, infinity, infinity },
        });
        planner.reset(11);

        const ngc::position_t diagonalEnd { 1, 1, 0, 1, 0, 0 };
        const auto diagonal = planner.compile(ngc::MoveLine { {}, diagonalEnd, 600.0 });
        require(diagonal.has_value(), diagonal ? "" : diagonal.error());
        requireAxisLimits(*diagonal, &ngc::position_t::x, 0.25, 0.5, 2.0, "diagonal X");
        requireAxisLimits(*diagonal, &ngc::position_t::y, 0.5, 1.0, 4.0, "diagonal Y");
        requireAxisLimits(*diagonal, &ngc::position_t::a, 0.1, 0.2, 0.5, "diagonal A");

        auto rapidEnd = diagonalEnd;
        rapidEnd.x += 1.0;
        const auto rapid = planner.compile(ngc::MoveLine { diagonalEnd, rapidEnd, -1.0 });
        require(rapid.has_value(), rapid ? "" : rapid.error());
        requireAxisLimits(*rapid, &ngc::position_t::x, 0.25, 0.5, 2.0, "rapid X");

        ngc::TrajectoryCompiler arcPlanner({
            .pathAcceleration = infinity,
            .rapidSpeed = 600.0,
            .arcChordTolerance = 0.0001,
            .pathJerk = infinity,
            .axisVelocity = { 0.2, 0.35, infinity, infinity, infinity, infinity },
            .axisAcceleration = { 0.3, 0.6, infinity, infinity, infinity, infinity },
            .axisJerk = { 2.0, 3.0, infinity, infinity, infinity, infinity },
        });
        const ngc::position_t arcStart { 1, 0, 0, 0, 0, 0 };
        arcPlanner.reset(12, arcStart);
        const auto arc = arcPlanner.compile(ngc::MoveArc {
            arcStart, { 0, 1, 0, 0, 0, 0 }, {}, { 0, 0, 1 }, 600.0 });
        require(arc.has_value(), arc ? "" : arc.error());
        requireAxisLimits(*arc, &ngc::position_t::x, 0.2, 0.3, 2.0, "arc X");
        requireAxisLimits(*arc, &ngc::position_t::y, 0.35, 0.6, 3.0, "arc Y");
    }

    void testMockBackendAdvancesOneFixedServoTick() {
        ngc::MockMotionBackend backend;
        ngc::PlanChunk chunk;
        chunk.epoch = 31;
        chunk.id = 1;
        chunk.branch = 1;
        require(chunk.normalMotion.push(linearSpan(1,0.0,1.0,0.01)),
                "fixed-tick test motion should fit in its chunk");
        require(chunk.stopTail.push(linearSpan(2, 1.0, 1.0, 1e-6)),
                "fixed-tick test stop tail should fit in its chunk");
        chunk.branchState.position.x = 1.0;
        chunk.stopState.position.x = 1.0;
        require(backend.tryPublish(chunk) == ngc::PublishResult::Published,
                "fixed-tick test chunk should publish");
        ngc::PlanChunk continuation;
        continuation.epoch=31;
        continuation.id=2;
        continuation.predecessorBranch=1;
        continuation.branch=2;
        require(continuation.normalMotion.push(linearSpan(3,1.0,3.0,0.02)),
                "fixed-tick continuation should fit in its chunk");
        require(continuation.stopTail.push(linearSpan(4,3.0,3.0,0.005)),
                "fixed-tick continuation stop tail should fit in its chunk");
        continuation.branchState.position.x=3.0;
        continuation.stopState.position.x=3.0;
        require(backend.tryPublish(continuation)==ngc::PublishResult::Published,
                "fixed-tick continuation should publish");
        require(backend.trySubmit(ngc::StartRequest { 1, 31 }) == ngc::SubmitResult::Submitted,
                "fixed-tick test backend should start");

        backend.advanceTick(0.001, true);
        ngc::ExecutionSnapshot snapshot;
        while(backend.tryTakeSnapshot(snapshot)) { }
        requireNear(snapshot.commanded.position.x, 0.1,
                    "one 1 ms servo tick should advance one tenth of a 10 ms linear span");
        requireNear(snapshot.activeNormalMotionRemainingSeconds,0.009,
                    "backend status should report the active normal-motion remainder");
        requireNear(snapshot.queuedNormalMotionSeconds,0.02,
                    "backend status should report queued normal-motion time");
        requireNear(snapshot.committedNormalMotionSeconds,0.029,
                    "backend status should sum active and queued normal-motion time");
        requireNear(snapshot.stopBranchRemainingSeconds,1e-6,
                    "backend status should report stop-branch time separately");
        require(snapshot.queuedExecutionItems==1,
                "backend status should report one queued continuation");

        for(int tick = 0; tick < 9; ++tick) backend.advanceTick(0.001, tick == 8);
        while(backend.tryTakeSnapshot(snapshot)) { }
        requireNear(snapshot.commanded.position.x, 1.0,
                    "ten fixed servo ticks should complete the 10 ms span exactly");
        requireNear(snapshot.activeNormalMotionRemainingSeconds,0.02,
                    "continuation activation should transfer queued time to active time");
        requireNear(snapshot.queuedNormalMotionSeconds,0.0,
                    "continuation activation should remove its time from the queue");
        requireNear(snapshot.committedNormalMotionSeconds,0.02,
                    "committed time should remain continuous across a packet boundary");
        require(snapshot.queuedExecutionItems==0,
                "continuation activation should decrement the queued item count");

        ngc::MockMotionBackend jerkBackend;
        ngc::PlanChunk jerkChunk;
        jerkChunk.epoch=32;
        jerkChunk.id=2;
        jerkChunk.branch=2;
        ngc::AxisPolynomialSpan cubic;
        cubic.id=3;
        cubic.duration=1.0;
        cubic.inverseDuration=1.0;
        cubic.inverseDurationSquared=1.0;
        cubic.inverseDurationCubed=1.0;
        cubic.coefficients[2].x = 1.0;
        require(jerkChunk.normalMotion.push(cubic),
                "jerk telemetry cubic should fit in its chunk");
        require(jerkChunk.stopTail.push(linearSpan(4,1.0,1.0,1e-6)),
                "jerk telemetry stop tail should fit in its chunk");
        jerkChunk.branchState = ngc::executionSpanEnd(cubic);
        jerkChunk.stopState.position.x=1.0;
        require(jerkBackend.tryPublish(jerkChunk)==ngc::PublishResult::Published,
                "jerk telemetry chunk should publish");
        require(jerkBackend.trySubmit(ngc::StartRequest{2,32})==ngc::SubmitResult::Submitted,
                "jerk telemetry backend should start");
        jerkBackend.advanceTick(0.1,true);
        requireNear(jerkBackend.currentProgramJerkMagnitude(),6.0,
                    "mock presentation telemetry should report analytic active-span jerk");
        const auto jerkSamples=jerkBackend.takeExecutedJerkSamples();
        require(jerkSamples.size()==1,
                std::format("mock jerk diagnostics should retain every calculated servo position (got {})",
                            jerkSamples.size()));
        requireNear(jerkSamples.front().position.x,0.001,
                    "mock jerk diagnostic should use the backend-calculated position");
        requireNear(jerkSamples.front().magnitude,6.0,
                    "mock jerk diagnostic should retain the active cubic jerk");
        require(jerkBackend.takeExecutedJerkSamples().empty(),
                "taking mock jerk diagnostics should consume the incremental samples");
    }

    void testPreparedArcJunctionMatchesSourceCurvature() {
        constexpr double RADIUS=0.05;
        const ngc::position_t first{RADIUS,0,0,0,0,0};
        const ngc::position_t junction{0,RADIUS,0,0,0,0};
        const ngc::position_t last{-RADIUS,0,0,0,0,0};
        const auto arcRecord=[](const ngc::PreparedCommandId id,
                                const ngc::position_t &from,
                                const ngc::position_t &to) {
            ngc::PreparedCommandRecord record;
            record.id=id;
            record.command=ngc::MoveArc{from,to,{0,0,0},{0,0,1},60.0};
            return record;
        };
        const std::array<ngc::PreparedCommandRecord,2> records{
            arcRecord(1,first,junction),arcRecord(2,junction,last),
        };
        const auto prepared=ngc::prepareContinuousGeometry(records,0.1,first);
        require(prepared.has_value(),prepared?"":prepared.error());
        require(prepared->pieces.size()==3
                    &&prepared->pieces[0].kind==ngc::PreparedPieceKind::RetainedArcSection
                    &&prepared->pieces[1].kind==ngc::PreparedPieceKind::JunctionBlend
                    &&prepared->pieces[2].kind==ngc::PreparedPieceKind::RetainedArcSection,
                "two long arcs should produce retained sections around one junction blend");
        require(prepared->pieces[1].sourceCommands
                    ==std::vector<ngc::PreparedCommandId>{1,2},
                "a junction blend should retain both adjacent source entities for preview selection");
        const auto &replaced = prepared->pieces[1].replacedSourceIntervals;
        require(replaced.size() == 2 && replaced[0].command == 1
                    && replaced[1].command == 2 && replaced[0].curve && replaced[1].curve,
                "a junction blend should retain both replaced source-curve intervals");
        requireNear(replaced[0].curveTo, replaced[0].curve->length,
                    "junction source preview should end at the incoming source endpoint");
        requireNear(replaced[1].curveFrom, 0.0,
                    "junction source preview should begin at the outgoing source endpoint");
        require(replaced[0].curveFrom > 0.0
                    && replaced[1].curveTo < replaced[1].curve->length,
                "junction source preview should retain only the replaced portions");

        ngc::CurveEvaluationWorkspace workspace;
        const auto &incoming=prepared->pieces[0];
        const auto &blend=prepared->pieces[1];
        const auto &outgoing=prepared->pieces[2];
        const auto *blendSpline=std::get_if<ngc::PreparedSplineCurve>(&blend.curve->value);
        require(blendSpline&&blendSpline->degree==3,
                "a junction blend should retain its cubic B-spline representation");
        require(blend.splineKnotIntervals.size()
                    ==blendSpline->controls.size()-blendSpline->degree,
                "a cubic junction blend should prepare one timing piece per knot interval");
        require(blend.geometricSamples.size()
                    ==(ngc::PREPARED_CURVE_SAMPLE_INTERVALS+1)
                        *blend.splineKnotIntervals.size(),
                "each cubic junction interval should retain 17 owned samples");
        auto foundOneSidedThirdDerivativeJump=false;
        for(std::size_t interval=0;interval<blend.splineKnotIntervals.size();++interval) {
            const auto &metadata=blend.splineKnotIntervals[interval];
            require(metadata.parameterSpan==interval,
                "junction knot metadata should retain its original parameter span");
            require(metadata.firstGeometricSample
                        ==(ngc::PREPARED_CURVE_SAMPLE_INTERVALS+1)*interval,
                "junction knot intervals should own disjoint sample ranges");
            if(interval==0) continue;
            const auto &previous=blend.splineKnotIntervals[interval-1];
            const auto &left=blend.geometricSamples[
                previous.firstGeometricSample+previous.geometricSampleCount-1];
            const auto &right=blend.geometricSamples[metadata.firstGeometricSample];
            requireNear(left.distance,right.distance,
                "adjacent junction intervals should sample the same knot distance");
            require((left.tangent-right.tangent).length()<1e-9,
                "a cubic junction knot should retain its continuous tangent");
            require((left.curvature-right.curvature).length()<1e-8,
                "a cubic junction knot should retain its continuous curvature");
            foundOneSidedThirdDerivativeJump |=
                (left.curvatureDerivative-right.curvatureDerivative).length()>1e-6;
        }
        require(foundOneSidedThirdDerivativeJump,
            "cubic junction knot samples should preserve distinct one-sided q''' values");
        require(incoming.geometricSamples.size()
                    ==ngc::PREPARED_CURVE_SAMPLE_INTERVALS+1
                    &&outgoing.geometricSamples.size()
                        ==ngc::PREPARED_CURVE_SAMPLE_INTERVALS+1,
                "arcs and spline knot intervals should use the same sampling density");
        const auto incomingCurvature=ngc::curvatureAtDistance(
            *incoming.curve,incoming.curveTo,workspace);
        const auto blendStartCurvature=ngc::curvatureAtDistance(
            *blend.curve,blend.curveFrom,workspace);
        const auto blendEndCurvature=ngc::curvatureAtDistance(
            *blend.curve,blend.curveTo,workspace);
        const auto outgoingCurvature=ngc::curvatureAtDistance(
            *outgoing.curve,outgoing.curveFrom,workspace);
        require((blendStartCurvature-incomingCurvature).length()<1e-8,
                "an arc-to-arc junction blend must match incoming arc curvature");
        require((blendEndCurvature-outgoingCurvature).length()<1e-8,
                "an arc-to-arc junction blend must match outgoing arc curvature");
        require(blendStartCurvature.length()>19.0&&blendEndCurvature.length()>19.0,
                "arc junction blend endpoint curvature must not collapse to zero");
        const auto &geometricSample=incoming.geometricSamples.back();
        const auto tangential=geometricSample.tangent.x*geometricSample.curvatureDerivative.x
            +geometricSample.tangent.y*geometricSample.curvatureDerivative.y
            +geometricSample.tangent.z*geometricSample.curvatureDerivative.z
            +geometricSample.tangent.a*geometricSample.curvatureDerivative.a
            +geometricSample.tangent.b*geometricSample.curvatureDerivative.b
            +geometricSample.tangent.c*geometricSample.curvatureDerivative.c;
        const ngc::position_t normal{
            geometricSample.curvatureDerivative.x-geometricSample.tangent.x*tangential,
            geometricSample.curvatureDerivative.y-geometricSample.tangent.y*tangential,
            geometricSample.curvatureDerivative.z-geometricSample.tangent.z*tangential,
            geometricSample.curvatureDerivative.a-geometricSample.tangent.a*tangential,
            geometricSample.curvatureDerivative.b-geometricSample.tangent.b*tangential,
            geometricSample.curvatureDerivative.c-geometricSample.tangent.c*tangential};
        require(normal.length()<1e-8,
                "constant-curvature arc samples should have zero normal sharpness");
        require(std::abs(geometricSample.curvatureDerivative.length()
                         -1.0/(RADIUS*RADIUS))<1e-6,
                "prepared samples must retain the tangential curvature-squared jerk component");
    }

    void testNoneSplineSmoothingPreservesCubicControls() {
        const std::vector<ngc::position_t> controls{
            {0.0,0.0,0.0,0.0,0.0,0.0},
            {0.1,0.2,0.0,0.0,0.0,0.0},
            {0.2,0.3,0.0,0.0,0.0,0.0},
            {0.3,0.4,0.0,0.0,0.0,0.0},
            {0.4,0.3,0.0,0.0,0.0,0.0},
            {0.5,0.2,0.0,0.0,0.0,0.0},
            {0.6,0.0,0.0,0.0,0.0,0.0},
        };
        const auto reconstructed=ngc::spline_detail::reconstructSpline(controls,
            ngc::spline_detail::SplineReconstructionSource{},0.01,true,
            ngc::spline_detail::SplineFitSolver::None);
        require(reconstructed.has_value(),
            reconstructed?"":reconstructed.error());
        require(reconstructed->degree==3&&reconstructed->controls.size()==controls.size(),
            "none spline smoothing must retain the input cubic representation");
        for(std::size_t index=0;index<controls.size();++index)
            require((reconstructed->controls[index]-controls[index]).length()==0.0,
                "none spline smoothing must preserve every cubic control exactly");
    }

    void testSingleShortEntityClusterRetainsMidpointControl() {
        const auto lineRecord=[](const ngc::PreparedCommandId id,
                                 const ngc::position_t &from,
                                 const ngc::position_t &to) {
            ngc::PreparedCommandRecord record;
            record.id=id;
            record.command=ngc::MoveLine{from,to,60.0};
            return record;
        };
        const std::array points{
            ngc::position_t{0,0,0,0,0,0},
            ngc::position_t{1,0,0,0,0,0},
            ngc::position_t{1.1,0.01,0,0,0,0},
            ngc::position_t{2.1,0.01,0,0,0,0},
        };
        const std::array records{
            lineRecord(1,points[0],points[1]),
            lineRecord(2,points[1],points[2]),
            lineRecord(3,points[2],points[3]),
        };
        ngc::GeometryPreparationEffort effort;
        effort.certifySourceTube=false;
        effort.generateSamples=false;
        effort.splineFitSolver=ngc::spline_detail::SplineFitSolver::None;
        const auto unsmoothed=ngc::prepareContinuousGeometry(
            records,0.05,points.front(),effort);
        require(unsmoothed.has_value(),unsmoothed?"":unsmoothed.error());
        const auto findCluster=[](const ngc::PreparedContinuousGeometry &geometry) {
            return std::ranges::find_if(geometry.pieces,[](const auto &piece) {
                return piece.kind==ngc::PreparedPieceKind::ClusterSpline;
            });
        };
        const auto unsmoothedCluster=findCluster(*unsmoothed);
        require(unsmoothedCluster!=unsmoothed->pieces.end(),
            "one short source entity between long anchors must form a cluster spline");
        const auto *cubic=std::get_if<ngc::PreparedSplineCurve>(
            &unsmoothedCluster->curve->value);
        require(cubic&&cubic->degree==3&&cubic->controls.size()==7,
            "a one-entity unsmoothed cluster must retain one interior control");
        const ngc::position_t midpoint{1.05,0.005,0,0,0,0};
        require((cubic->controls[3]-midpoint).length()<1e-12,
            "the cluster seed control must be the short source entity midpoint");

        effort.splineFitSolver=ngc::spline_detail::continuousSplineFitSolver();
        const auto smoothed=ngc::prepareContinuousGeometry(
            records,0.05,points.front(),effort);
        require(smoothed.has_value(),smoothed?"":smoothed.error());
        const auto smoothedCluster=findCluster(*smoothed);
        const auto *quintic=smoothedCluster==smoothed->pieces.end()?nullptr
            :std::get_if<ngc::PreparedSplineCurve>(&smoothedCluster->curve->value);
        require(quintic&&quintic->degree==5&&quintic->controls.size()==9,
            "smoothing a one-entity cluster must retain one optimizable interior control");
    }

    void testClusterSplinePreparesKnotIntervalSamplesAndFeeds() {
        static_assert(ngc::spline_detail::continuousSplineFitSolver()
            ==ngc::spline_detail::SplineFitSolver::VelocityTargetedBandedFairness);
        const auto lineRecord=[](const ngc::PreparedCommandId id,
                                 const ngc::position_t &from,
                                 const ngc::position_t &to,const double feed) {
            ngc::PreparedCommandRecord record;
            record.id=id;
            record.command=ngc::MoveLine{from,to,feed};
            return record;
        };
        std::array points{
            ngc::position_t{0,0,0,0,0,0},
            ngc::position_t{1,0,0,0,0,0},
            ngc::position_t{1.04,0.03,0,0,0,0},
            ngc::position_t{1.08,0,0,0,0,0},
            ngc::position_t{1.12,0.03,0,0,0,0},
            ngc::position_t{1.16,0,0,0,0,0},
            ngc::position_t{1.20,0.03,0,0,0,0},
            ngc::position_t{1.24,0,0,0,0,0},
            ngc::position_t{1.28,0.03,0,0,0,0},
            ngc::position_t{2.28,0.03,0,0,0,0},
        };
        const ngc::position_t translation{2.3,9.1,1.1,0,0,0};
        for (auto &point : points) {
            point = point + translation;
        }
        constexpr std::array feeds{60.0,120.0,120.0,30.0,120.0,
                                   120.0,120.0,120.0,180.0};
        std::vector<ngc::PreparedCommandRecord> records;
        records.reserve(feeds.size());
        for(std::size_t index=0;index<feeds.size();++index)
            records.push_back(lineRecord(index+1,points[index],points[index+1],feeds[index]));
        const ngc::GeometryPreparationEffort effort{
            .certifySourceTube=false,
            .generateSamples=true,
            .lengthTableIntervalsPerKnotSpan=32,
            .splineVelocityLimits={
                .pathAcceleration=20.0,.pathJerk=501.0,
                .axisVelocity={3.333333333,3.333333333,3.333333333,
                               3.333333333,3.333333333,3.333333333},
                .axisAcceleration={20,20,20,20,20,20},
                .axisJerk={501,501,501,501,501,501},
            },
        };
        const auto prepared=ngc::prepareContinuousGeometry(records,0.05,points.front(),effort);
        require(prepared.has_value(),prepared?"":prepared.error());
        const auto found=std::ranges::find_if(prepared->pieces,[](const auto &piece) {
            return piece.kind==ngc::PreparedPieceKind::ClusterSpline;
        });
        require(found!=prepared->pieces.end(),
                "mixed-feed short source entities should produce a cluster spline");
        std::vector<ngc::PreparedCommandId> expectedSourceCommands(records.size());
        std::iota(expectedSourceCommands.begin(),expectedSourceCommands.end(),1);
        require(found->sourceCommands==expectedSourceCommands,
                "a cluster spline should retain every reconstructed source entity for preview selection");
        require(found->activationStations.size()+1==records.size(),
                "cluster preparation should retain one curve-distance activation for every "
                "source command after the incoming anchor");
        for(std::size_t activation=0;activation<found->activationStations.size();++activation) {
            const auto &station=found->activationStations[activation];
            require(station.command==activation+2
                        &&station.curveDistance>found->curveFrom
                        &&station.curveDistance<found->curveTo,
                    "cluster activation stations should preserve source-command order inside "
                    "the prepared curve");
            if(activation>0)
                require(station.curveDistance
                            >found->activationStations[activation-1].curveDistance,
                        "cluster activation curve distances should be strictly ordered");
        }
        require(found->replacedSourceIntervals.size()==records.size(),
                "a cluster spline should retain one replaced interval per contributing source entity");
        for(std::size_t source=0;source<records.size();++source) {
            const auto &interval=found->replacedSourceIntervals[source];
            require(interval.command==source+1&&interval.curve,
                    "cluster replaced intervals should preserve source-command order");
            if(source==0) {
                requireNear(interval.curveFrom,interval.curve->length-0.15,
                            "cluster source preview should show only the incoming long-entity trim");
                requireNear(interval.curveTo,interval.curve->length,
                            "cluster incoming source preview should reach its endpoint");
            } else if(source+1==records.size()) {
                requireNear(interval.curveFrom,0.0,
                            "cluster outgoing source preview should begin at its endpoint");
                requireNear(interval.curveTo,0.15,
                            "cluster source preview should show only the outgoing long-entity trim");
            } else {
                requireNear(interval.curveFrom,0.0,
                            "cluster source preview should show each complete short source entity");
                requireNear(interval.curveTo,interval.curve->length,
                            "cluster source preview should show each complete short source entity");
            }
        }
        const auto *spline=std::get_if<ngc::PreparedSplineCurve>(&found->curve->value);
        require(spline&&spline->degree==5,
                "the focused cluster sampling case should use quintic reconstruction");
        require(found->geometricSamples.front().curvature.length()<1e-9
                    &&found->geometricSamples.back().curvature.length()<1e-9,
                "translated cluster endpoint curvature must match its straight anchors");
        auto noneEffort=effort;
        noneEffort.splineFitSolver=ngc::spline_detail::SplineFitSolver::None;
        const auto unsmoothed=ngc::prepareContinuousGeometry(
            records,0.05,points.front(),noneEffort);
        require(unsmoothed.has_value(),unsmoothed?"":unsmoothed.error());
        const auto unsmoothedCluster=std::ranges::find_if(
            unsmoothed->pieces,[](const auto &piece) {
                return piece.kind==ngc::PreparedPieceKind::ClusterSpline;
            });
        const auto *unsmoothedSpline=unsmoothedCluster==unsmoothed->pieces.end()
            ?nullptr:std::get_if<ngc::PreparedSplineCurve>(&unsmoothedCluster->curve->value);
        require(unsmoothedSpline&&unsmoothedSpline->degree==3,
            "none smoothing must preserve the cluster's unsmoothed cubic construction");
        const auto intervalCount=spline->controls.size()-spline->degree;
        require(found->splineKnotIntervals.size()==intervalCount,
                "cluster preparation should retain one metadata record per knot interval");
        require(found->geometricSamples.size()
                    ==(ngc::PREPARED_CURVE_SAMPLE_INTERVALS+1)*intervalCount,
                "cluster preparation should retain 17 owned samples per knot interval");
        requireNear(found->programmedFeed,2.0,
                    "cluster-wide programmed feed must remain unchanged");
        auto minimumIntervalFeed=std::numeric_limits<double>::infinity();
        for(std::size_t interval=0;interval<intervalCount;++interval) {
            const auto &metadata=found->splineKnotIntervals[interval];
            require(metadata.firstGeometricSample
                        ==(ngc::PREPARED_CURVE_SAMPLE_INTERVALS+1)*interval
                    &&metadata.geometricSampleCount
                        ==ngc::PREPARED_CURVE_SAMPLE_INTERVALS+1,
                    "cluster knot interval should address its configured samples without a search");
            require(metadata.parameterSpan==interval,
                "cluster knot metadata should retain its original parameter span");
            require(metadata.curveTo>metadata.curveFrom,
                    "cluster knot interval distances should be strictly ordered");
            require(metadata.geometricVelocityLimit>0.0
                        &&!std::isnan(metadata.geometricVelocityLimit),
                    "geometry preparation should attach a reusable static velocity cap to "
                    "each cluster knot interval");
            if(interval>0)
                requireNear(metadata.curveFrom,
                    found->splineKnotIntervals[interval-1].curveTo,
                    "adjacent cluster knot intervals should share one curve boundary");
            requireNear(found->geometricSamples[metadata.firstGeometricSample].distance,
                        metadata.curveFrom-found->curveFrom,
                        "cluster knot interval first sample distance is incorrect");
            requireNear(found->geometricSamples[
                            metadata.firstGeometricSample+metadata.geometricSampleCount-1].distance,
                        metadata.curveTo-found->curveFrom,
                        "cluster knot interval last sample distance is incorrect");
            minimumIntervalFeed=std::min(minimumIntervalFeed,metadata.programmedFeed);
        }
        requireNear(minimumIntervalFeed,0.5,
                    "a knot interval overlapping the slow source entity should retain its minimum feed");

        std::vector<std::pair<double,double>> expectedTimingIntervals;
        for(const auto &piece:prepared->pieces) {
            if(piece.kind==ngc::PreparedPieceKind::ClusterSpline) {
                for(const auto &interval:piece.splineKnotIntervals)
                    expectedTimingIntervals.emplace_back(
                        interval.curveTo-interval.curveFrom,interval.programmedFeed);
            } else {
                expectedTimingIntervals.emplace_back(piece.length(),piece.programmedFeed);
            }
        }
        const ngc::TrajectoryLimits trajectoryLimits{
            .pathAcceleration=20.0,.rapidSpeed=199.8,.arcChordTolerance=0.0001,
            .pathJerk=501.0,
            .axisVelocity={3.333333333,3.333333333,3.333333333,
                           3.333333333,3.333333333,3.333333333},
            .axisAcceleration={20,20,20,20,20,20},
            .axisJerk={501,501,501,501,501,501},
        };
        const std::array accelerationControls{
            ngc::position_t{},
            ngc::position_t{0.04,0,0,0,0,0},
            ngc::position_t{0.02,0,0,0,0,0},
            ngc::position_t{},
        };
        requireNear(
            ngc::trajectory_detail::accelerationControlHullExcursionRatio(
                accelerationControls, 0.001, 100.0,
                ngc::position_t{50,50,50,50,50,50}),
            0.8,
            "the isolated quintic acceleration-control hull should use the "
            "strictest path or axis servo-period jerk budget");
        require(ngc::trajectory_detail::servoAwareJerkAccepted(
                    0.0005, 1.5, 0.8, 0.001),
                "a sub-servo jerk peak with a bounded acceleration excursion "
                "should pass the production policy");
        require(!ngc::trajectory_detail::servoAwareJerkAccepted(
                    0.001, 1.5, 0.8, 0.001),
                "a full-servo jerk violation should remain a strict failure");
        require(!ngc::trajectory_detail::servoAwareJerkAccepted(
                    0.0005, 1.5, 1.1, 0.001),
                "a sub-servo jerk peak with excessive acceleration excursion "
                "should fail the production policy");
        require(ngc::trajectory_detail::servoAwareJerkAccepted(
                    0.001, 1.01, 2.0, 0.001)
                    && !ngc::trajectory_detail::servoAwareJerkAccepted(
                        0.001, 1.0101, 0.0, 0.001),
                "the production jerk policy should allow one percent but no more");
        require(ngc::trajectory_detail::dynamicLimitRatioAccepted(1.01)
                    && !ngc::trajectory_detail::dynamicLimitRatioAccepted(1.0101),
                "the production acceleration policy should allow one percent "
                "but no more");
        require(ngc::trajectory_detail::velocityRatioAccepted(1.00000005)
                    && !ngc::trajectory_detail::velocityRatioAccepted(1.0000002),
                "the production velocity policy should allow only its small "
                "numerical tolerance");
        const auto independentlyClassified =
            ngc::trajectory_detail::classifyQuinticConstraints(
                0.0005, 1.0000002, 1.005, 1.5, 0.8, 0.001);
        require(!independentlyClassified.velocityAccepted
                    && independentlyClassified.accelerationAccepted
                    && independentlyClassified.jerkAccepted
                    && independentlyClassified.subServoJerkAccepted
                    && !independentlyClassified.constraintsVerified,
                "a strict velocity failure must not suppress the independent "
                "sub-servo jerk exception");
        require(independentlyClassified.correctionCategory
                    == ngc::trajectory_detail::QuinticConstraintCategory::Velocity,
                "an accepted sub-servo jerk peak must leave correction ownership "
                "with the failed velocity category");
        requireNear(independentlyClassified.maximumFailedCorrectionRatio,
            1.0000002,
            "an accepted sub-servo jerk peak must retain only the velocity "
            "correction ratio");
        const std::vector<std::size_t> denseMarkerCounts(200, 2);
        const auto markerBoundPackets =
            ngc::trajectory_detail::continuousPacketRanges(
                denseMarkerCounts);
        require(markerBoundPackets && markerBoundPackets->size() == 2
                    && (*markerBoundPackets)[0].firstSpan == 0
                    && (*markerBoundPackets)[0].pastLastSpan == 128
                    && (*markerBoundPackets)[0].markerCount
                        == ngc::MAX_EXECUTION_MARKERS_PER_CHUNK
                    && (*markerBoundPackets)[1].firstSpan == 128
                    && (*markerBoundPackets)[1].pastLastSpan == 200
                    && (*markerBoundPackets)[1].markerCount == 144,
                "continuous packetization should split at marker capacity "
                "before execution-span capacity");
        const std::vector<std::size_t> sparseMarkerCounts(300, 0);
        const auto spanBoundPackets =
            ngc::trajectory_detail::continuousPacketRanges(
                sparseMarkerCounts);
        require(spanBoundPackets && spanBoundPackets->size() == 2
                    && (*spanBoundPackets)[0].pastLastSpan
                        == ngc::MAX_NORMAL_SPANS_PER_CHUNK
                    && (*spanBoundPackets)[1].firstSpan
                        == ngc::MAX_NORMAL_SPANS_PER_CHUNK
                    && (*spanBoundPackets)[1].pastLastSpan == 300,
                "continuous packetization should retain the independent "
                "execution-span capacity");
        const std::array oversizedSpanMarkerCounts {
            ngc::MAX_EXECUTION_MARKERS_PER_CHUNK + 1,
        };
        const auto oversizedSpanPackets =
            ngc::trajectory_detail::continuousPacketRanges(
                oversizedSpanMarkerCounts);
        require(!oversizedSpanPackets && oversizedSpanPackets.error() == 0,
                "a single emitted span that exceeds marker capacity must "
                "remain a bounded fatal planning error");
        ngc::TrajectoryCompiler compiler(trajectoryLimits);
        auto planningEffort = compiler.continuousPlanningEffort();
        require(planningEffort.boundaryAccelerationMode
                    == ngc::ContinuousBoundaryAccelerationMode::Optimized
                    && name(planningEffort.boundaryAccelerationMode) == "optimized"
                    && name(ngc::ContinuousBoundaryAccelerationMode::Zero) == "zero",
                "continuous planning should default to PathTempo's optimized boundary mode");
        compiler.setContinuousPlanningEffort(planningEffort);
        compiler.reset(94,points.front());
        const auto planned=compiler.compileContinuous(*prepared,0.05);
        require(planned&&*planned,planned?"":planned.error());
        require(std::ranges::all_of((*planned)->chunks, [](const auto &chunk) {
                    return std::ranges::all_of(
                               chunk.normalMotion, [](const auto &span) {
                                   return span.degree
                                       == ngc::ExecutionPolynomialDegree::Quintic;
                               })
                        && std::ranges::all_of(
                               chunk.stopTail, [](const auto &span) {
                                   return span.degree
                                       == ngc::ExecutionPolynomialDegree::Cubic;
                               });
                }),
                "materialized continuous planning should emit proved quintic "
                "normal motion with cubic stop tails");

        ngc::TrajectoryCompiler zeroBoundaryCompiler(trajectoryLimits);
        auto zeroBoundaryEffort = planningEffort;
        zeroBoundaryEffort.boundaryAccelerationMode =
            ngc::ContinuousBoundaryAccelerationMode::Zero;
        zeroBoundaryCompiler.setContinuousPlanningEffort(zeroBoundaryEffort);
        zeroBoundaryCompiler.reset(94, points.front());
        const auto zeroBoundaryPlan = zeroBoundaryCompiler.compileContinuous(*prepared, 0.05);
        require(zeroBoundaryPlan && *zeroBoundaryPlan,
                zeroBoundaryPlan ? "" : zeroBoundaryPlan.error());
        require(std::ranges::all_of((*zeroBoundaryPlan)->pieceTiming, [](const auto &piece) {
                    return std::abs(piece.entryAcceleration) < 1e-9
                        && std::abs(piece.exitAcceleration) < 1e-9;
                }),
                "zero boundary mode should force zero acceleration at every PathPiece boundary");

        auto roundedEndpointGeometry=*prepared;
        auto roundedCluster=std::ranges::find_if(
            roundedEndpointGeometry.pieces,[](const auto &piece) {
                return piece.kind==ngc::PreparedPieceKind::ClusterSpline;
            });
        require(roundedCluster!=roundedEndpointGeometry.pieces.end()
                    &&roundedCluster->splineKnotIntervals.size()>1,
            "endpoint-rounding regression requires a multi-interval cluster spline");
        const auto &roundedInterval=roundedCluster->splineKnotIntervals[1];
        const auto roundedSample=roundedInterval.firstGeometricSample
            +roundedInterval.geometricSampleCount-1;
        roundedCluster->geometricSamples[roundedSample].distance+=5e-11;
        ngc::TrajectoryCompiler roundedEndpointCompiler(trajectoryLimits);
        roundedEndpointCompiler.setContinuousPlanningEffort(planningEffort);
        roundedEndpointCompiler.reset(94,points.front());
        const auto roundedEndpointPlan=roundedEndpointCompiler.compileContinuous(
            roundedEndpointGeometry,0.05);
        require(roundedEndpointPlan&&*roundedEndpointPlan,
            roundedEndpointPlan?"":roundedEndpointPlan.error());

        require((*planned)->activations.size()==records.size(),
                "continuous timing should resolve every prepared command activation");
        std::vector<ngc::SpanId> commandActivationSpans(records.size());
        auto interiorExecutionMarkers = 0U;
        for (const auto &activation : (*planned)->activations) {
            commandActivationSpans[activation.input]=activation.span;
            require(activation.marker != 0
                        && activation.chunk < (*planned)->chunks.size()
                        && activation.parameter >= 0.0
                        && activation.parameter <= 1.0,
                    "continuous command activation should retain a bounded "
                    "backend execution marker");
            const auto &markers =
                (*planned)->chunks[activation.chunk].markers;
            const auto marker = std::ranges::find(
                markers, activation.marker, &ngc::ExecutionMarker::id);
            require(marker != markers.end()
                        && (*planned)->chunks[activation.chunk]
                                .normalMotion[marker->span].id
                            == activation.span
                        && marker->parameter == activation.parameter,
                    "continuous activation metadata should match its packet marker");
            if (activation.parameter > 0.0
               && activation.parameter < 1.0) {
                ++interiorExecutionMarkers;
            }
        }
        require(std::ranges::all_of(commandActivationSpans,[](const auto span) {
                    return span!=0;
                })&&std::ranges::is_sorted(commandActivationSpans)
                &&commandActivationSpans[1]!=commandActivationSpans.back(),
                "cluster source commands should activate progressively through emitted spans");
        require(interiorExecutionMarkers > 0,
                "cluster source commands should use exact in-span backend "
                "markers instead of early span-start activation");
        require((*planned)->executionMarkers == (*planned)->activations.size()
                    && (*planned)->interiorExecutionMarkers
                        == interiorExecutionMarkers
                    && (*planned)->maximumExecutionMarkersPerChunk > 0
                    && (*planned)->maximumExecutionMarkersPerChunk
                        <= ngc::MAX_EXECUTION_MARKERS_PER_CHUNK,
                "continuous marker diagnostics should account for the "
                "bounded production packet representation");
        require(std::ranges::any_of((*planned)->pieceTiming,[](const auto &piece) {
                    return std::isfinite(piece.staticVelocityLimit);
                })
                    &&std::ranges::all_of((*planned)->pieceTiming,[](const auto &piece) {
                        return piece.velocityLimit
                            <=std::min(piece.programmedVelocityLimit,
                                piece.staticVelocityLimit)+1e-12;
                    }),
                "PathTempo should retain prepared static velocity caps without caller-side "
                "acceleration or jerk reduction");
        require((*planned)->correctionPasses >= 1
                    && (*planned)->correctionHistory.contains("pass 0:")
                    && (*planned)->materialization.callbackPasses >= 1
                    && (*planned)->correctionPasses
                        >= (*planned)->materialization.callbackPasses
                    && (*planned)->materialization.quintic
                            .constraintBoundNodes > 0,
                "PathTempo should run its sampled passes before production "
                "materialization proves a candidate's quintics");

        const auto planFingerprint=[](const ngc::ContinuousTrajectoryPlan &plan) {
            std::vector<std::uint64_t> result;
            const auto appendInteger=[&](const auto value) {
                result.push_back(static_cast<std::uint64_t>(value));
            };
            const auto appendDouble=[&](const double value) {
                result.push_back(std::bit_cast<std::uint64_t>(value));
            };
            const auto appendPosition=[&](const ngc::position_t &position) {
                appendDouble(position.x);
                appendDouble(position.y);
                appendDouble(position.z);
                appendDouble(position.a);
                appendDouble(position.b);
                appendDouble(position.c);
            };
            const auto appendState=[&](const ngc::MotionState &state) {
                appendPosition(state.position);
                appendPosition(state.velocity);
                appendPosition(state.acceleration);
            };
            const auto appendSpan=[&](const ngc::AxisPolynomialSpan &span) {
                appendInteger(span.id);
                appendDouble(span.duration);
                appendDouble(span.inverseDuration);
                appendDouble(span.inverseDurationSquared);
                appendDouble(span.inverseDurationCubed);
                appendInteger(std::to_underlying(span.degree));
                appendPosition(span.origin);
                for (const auto &coefficient : span.coefficients) {
                    appendPosition(coefficient);
                }
            };

            appendInteger(plan.pieceTiming.size());
            for(const auto &piece:plan.pieceTiming) {
                appendInteger(piece.input);
                appendDouble(piece.length);
                appendInteger(piece.linear);
                appendPosition(piece.startPosition);
                appendPosition(piece.endPosition);
                appendDouble(piece.programmedVelocityLimit);
                appendDouble(piece.staticVelocityLimit);
                appendDouble(piece.velocityLimit);
                appendDouble(piece.accelerationLimit);
                appendDouble(piece.jerkLimit);
                appendDouble(piece.entryVelocity);
                appendDouble(piece.entryAcceleration);
                appendDouble(piece.exitVelocity);
                appendDouble(piece.exitAcceleration);
                appendDouble(piece.duration);
            }
            appendInteger(plan.activations.size());
            for(const auto &activation:plan.activations) {
                appendInteger(activation.input);
                appendInteger(activation.span);
                appendInteger(activation.chunk);
                appendInteger(activation.marker);
                appendDouble(activation.parameter);
            }
            appendInteger(plan.chunks.size());
            for(const auto &chunk:plan.chunks) {
                appendInteger(chunk.epoch);
                appendInteger(chunk.id);
                appendInteger(chunk.predecessorBranch);
                appendInteger(chunk.branch);
                appendInteger(chunk.normalMotion.size);
                for(const auto &span:chunk.normalMotion) appendSpan(span);
                appendInteger(chunk.stopTail.size);
                for(const auto &span:chunk.stopTail) appendSpan(span);
                appendInteger(chunk.events.size);
                appendInteger(chunk.markers.size);
                for (const auto &marker : chunk.markers) {
                    appendInteger(marker.id);
                    appendInteger(marker.span);
                    appendDouble(marker.parameter);
                }
                require(chunk.events.size==0,
                    "focused continuous-path plan should not contain scheduled events");
                appendState(chunk.branchState);
                appendState(chunk.stopState);
            }
            return result;
        };
        ngc::TrajectoryCompiler repeatedCompiler(trajectoryLimits);
        repeatedCompiler.setContinuousPlanningEffort(planningEffort);
        repeatedCompiler.reset(94,points.front());
        const auto repeatedPlan=repeatedCompiler.compileContinuous(*prepared,0.05);
        require(repeatedPlan&&*repeatedPlan,repeatedPlan?"":repeatedPlan.error());
        require(planFingerprint(**repeatedPlan)==planFingerprint(**planned)
                    &&(*repeatedPlan)->correctionHistory==(*planned)->correctionHistory
                    &&(*repeatedPlan)->geometryVerificationAttempts
                        ==(*planned)->geometryVerificationAttempts
                    &&(*repeatedPlan)->geometryVerificationHighWater
                        ==(*planned)->geometryVerificationHighWater,
                "PathTempo production planning must preserve exact timing, emitted spans, "
                "and verification outcomes across repeated compilations");

        require((*planned)->pieceTiming.size()==expectedTimingIntervals.size(),
                "continuous timing should create one timing interval per cluster knot interval");
        for(std::size_t interval=0;interval<expectedTimingIntervals.size();++interval) {
            requireNear((*planned)->pieceTiming[interval].length,
                        expectedTimingIntervals[interval].first,
                        "continuous timing interval length should match prepared geometry");
            requireNear((*planned)->pieceTiming[interval].programmedVelocityLimit,
                        expectedTimingIntervals[interval].second,
                        "continuous timing interval should use its prepared programmed feed");
        }

        auto clusterCommands=records;
        for(auto &record:clusterCommands) {
            record.metadata.pathMode=ngc::ExecutablePathMode::Continuous;
            record.metadata.pathTolerance=0.05;
        }
        const auto clusterNominalDuration=std::accumulate(
            found->splineKnotIntervals.begin(),found->splineKnotIntervals.end(),0.0,
            [](const double total,const ngc::PreparedSplineKnotInterval &interval) {
                return total+(interval.curveTo-interval.curveFrom)/interval.programmedFeed;
            });
        auto rollingLimits=trajectoryLimits;
        rollingLimits.lookaheadDuration=0.2*clusterNominalDuration;
        ngc::TrajectoryPlanner clusterPlanner(rollingLimits);
        clusterPlanner.setContinuousPlanningEffort(planningEffort);
        ngc::CurveEvaluationWorkspace clusterWorkspace;
        const auto clusterStart=ngc::positionAtDistance(
            *found->curve,found->curveFrom,clusterWorkspace);
        clusterPlanner.reset(98,clusterStart);
        const ngc::PreparedGeometrySlice clusterSlice{
            .epoch=98,
            .sequence=1,
            .chain=1,
            .commands=std::move(clusterCommands),
            .pieces={*found},
            .nominalDuration=clusterNominalDuration,
        };
        require(clusterPlanner.enqueuePrepared(clusterSlice),
                clusterPlanner.lastPreparedEnqueueError());
        require(clusterPlanner.endPreparedChain(clusterSlice.chain),
                "the focused cluster rolling chain should end cleanly");
        const auto rolledCluster=clusterPlanner.planWindow();
        require(rolledCluster&&*rolledCluster,
                rolledCluster?"cluster spline did not produce a rolling prefix"
                    :rolledCluster.error());
        require(clusterPlanner.diagnostics().rollingBoundaryCandidates>0
                    &&clusterPlanner.diagnostics().maximumRollingSuffixProbePieces>0
                    &&clusterPlanner.diagnostics().maximumRollingSuffixProbePieces
                        <found->splineKnotIntervals.size()
                    &&clusterPlanner.preparedPieceCount()==1
                    &&clusterPlanner.preparedNominalDuration()<clusterNominalDuration,
                std::format("an ended cluster spline should roll at a prepared knot boundary "
                    "and retain only the exact suffix and its future activation stations: "
                    "candidates={} proof_pieces={}/{} retained_pieces={} commands={} "
                    "duration={}/{}",
                    clusterPlanner.diagnostics().rollingBoundaryCandidates,
                    clusterPlanner.diagnostics().maximumRollingSuffixProbePieces,
                    found->splineKnotIntervals.size(),clusterPlanner.preparedPieceCount(),
                    clusterPlanner.windowSize(),
                    clusterPlanner.preparedNominalDuration(),clusterNominalDuration));

        auto cappedClusterPiece = *found;
        constexpr auto LOW_CLUSTER_VELOCITY_CAP = 0.001;
        for (auto &interval : cappedClusterPiece.splineKnotIntervals) {
            interval.geometricVelocityLimit = LOW_CLUSTER_VELOCITY_CAP;
        }
        auto sourceOnlyRecord = records.back();
        sourceOnlyRecord.id = 10000;
        sourceOnlyRecord.presentationActivation = false;
        cappedClusterPiece.sourceCommands.push_back(sourceOnlyRecord.id);
        ngc::TrajectoryPlanner cappedClusterPlanner(rollingLimits);
        cappedClusterPlanner.setContinuousPlanningEffort(planningEffort);
        cappedClusterPlanner.reset(99, clusterStart);
        auto cappedClusterCommands = records;
        for (auto &record : cappedClusterCommands) {
            record.metadata.pathMode = ngc::ExecutablePathMode::Continuous;
            record.metadata.pathTolerance = 0.05;
        }
        sourceOnlyRecord.metadata.pathMode =
            ngc::ExecutablePathMode::Continuous;
        sourceOnlyRecord.metadata.pathTolerance = 0.05;
        cappedClusterCommands.push_back(std::move(sourceOnlyRecord));
        const ngc::PreparedGeometrySlice cappedClusterSlice {
            .epoch = 99,
            .sequence = 1,
            .chain = 1,
            .commands = std::move(cappedClusterCommands),
            .pieces = {std::move(cappedClusterPiece)},
            .nominalDuration = clusterNominalDuration,
        };
        require(cappedClusterPlanner.enqueuePrepared(cappedClusterSlice),
            cappedClusterPlanner.lastPreparedEnqueueError());
        require(cappedClusterPlanner.endPreparedChain(cappedClusterSlice.chain),
            "the low-cap cluster rolling chain should end cleanly");
        const auto cappedClusterPlan = cappedClusterPlanner.planWindow();
        require(cappedClusterPlan && *cappedClusterPlan,
            cappedClusterPlan ? "low-cap cluster spline did not produce a rolling prefix"
                              : cappedClusterPlan.error());
        const auto &cappedBoundaryChunk =
            std::get<ngc::PlanChunk>((*cappedClusterPlan)->items.back());
        require(cappedClusterPlanner.preparedPieceCount() == 1
                    && cappedBoundaryChunk.branchState.velocity.length()
                        <= LOW_CLUSTER_VELOCITY_CAP * (1.0 + 1e-7),
            "a cluster rolling boundary must honor the adjacent prepared "
            "geometric velocity caps");

        auto missingSamples=*prepared;
        const auto missingCluster=std::ranges::find_if(
            missingSamples.pieces,[](const auto &piece) {
                return piece.kind==ngc::PreparedPieceKind::ClusterSpline;
            });
        require(missingCluster!=missingSamples.pieces.end(),
                "focused missing-sample case requires a cluster spline");
        missingCluster->geometricSamples.clear();
        compiler.reset(95,points.front());
        const auto rejected=compiler.compileContinuous(missingSamples,0.05);
        require(!rejected&&rejected.error().contains("no usable geometric samples"),
                "continuous timing must reject missing producer samples without a fallback");
    }

    void testCollinearJunctionBlendUsesLinearTiming() {
        const ngc::position_t first{-0.8375,4.0929,-1.3,0,0,0};
        const ngc::position_t junction{-0.8375,4.0929,-1.4795,0,0,0};
        const ngc::position_t last{-0.8375,4.0929,-2.1881,0,0,0};
        const auto lineRecord=[](const ngc::PreparedCommandId id,
                                 const ngc::position_t &from,
                                 const ngc::position_t &to,const double feed) {
            ngc::PreparedCommandRecord record;
            record.id=id;
            record.command=ngc::MoveLine{from,to,feed};
            return record;
        };
        const std::array records{
            lineRecord(1,first,junction,80.0),
            lineRecord(2,junction,last,40.0),
        };
        const auto prepared=ngc::prepareContinuousGeometry(records,0.005,first);
        require(prepared.has_value(),prepared?"":prepared.error());
        const auto blend=std::ranges::find_if(prepared->pieces,[](const auto &piece) {
            return piece.kind==ngc::PreparedPieceKind::JunctionBlend;
        });
        require(blend!=prepared->pieces.end()&&blend->curve->geometricallyLinear,
                "a monotone collinear junction blend must retain its spline identity but use "
                "the exact linear timing emitter");
        ngc::CurveEvaluationWorkspace workspace;
        const auto blendStart = ngc::positionAtDistance(
            *blend->curve,blend->curveFrom,workspace);
        const auto blendEnd = ngc::positionAtDistance(
            *blend->curve,blend->curveTo,workspace);
        for (const auto fraction : {0.1,0.25,0.5,0.75,0.9}) {
            const auto actual = ngc::positionAtDistance(
                *blend->curve,
                std::lerp(blend->curveFrom,blend->curveTo,fraction),workspace);
            const ngc::position_t expected{
                std::lerp(blendStart.x,blendEnd.x,fraction),
                std::lerp(blendStart.y,blendEnd.y,fraction),
                std::lerp(blendStart.z,blendEnd.z,fraction),
                std::lerp(blendStart.a,blendEnd.a,fraction),
                std::lerp(blendStart.b,blendEnd.b,fraction),
                std::lerp(blendStart.c,blendEnd.c,fraction),
            };
            require((actual-expected).length() < 1e-11,
                    "a collinear multi-knot spline must invert chord distance "
                    "within its owning knot interval");
        }

        ngc::TrajectoryCompiler compiler({
            .pathAcceleration=20.0,.rapidSpeed=199.8,.arcChordTolerance=0.0001,
            .pathJerk=501.0,
            .axisVelocity={3.333333333,3.333333333,3.333333333,
                           3.333333333,3.333333333,3.333333333},
            .axisAcceleration={20,20,20,20,20,20},
            .axisJerk={501,501,501,501,501,501},
        });
        compiler.reset(93,first);
        const auto planned=compiler.compileContinuous(*prepared,0.005);
        require(planned&&*planned,planned?"":planned.error());
        const auto expectedTimingPieces=std::accumulate(prepared->pieces.begin(),
            prepared->pieces.end(),std::size_t{0},[](const std::size_t total,
                                                     const auto &piece) {
                return total+(piece.splineKnotIntervals.empty()
                    ?std::size_t{1}:piece.splineKnotIntervals.size());
            });
        require((*planned)->pieceTiming.size()==expectedTimingPieces,
                "continuous timing should emit one piece per cubic knot interval");
        const auto duration=std::accumulate((*planned)->pieceTiming.begin(),
            (*planned)->pieceTiming.end(),0.0,[](const double total,const auto &piece) {
                return total+piece.duration;
            });
        require(duration<5.0,std::format(
            "collinear feed-transition timing must remain bounded, got {:.6f} seconds",duration));

    }

    void testExecutionPolynomialEvaluation() {
        ngc::AxisPolynomialSpan span;
        span.degree = ngc::ExecutionPolynomialDegree::Quintic;
        span.duration = 2.0;
        span.inverseDuration = 0.5;
        span.inverseDurationSquared = 0.25;
        span.inverseDurationCubed = 0.125;
        span.origin = {10.0, -4.0, 3.0, 0.0, 0.0, 0.0};
        span.coefficients = {
            ngc::position_t{1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            ngc::position_t{2.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            ngc::position_t{3.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            ngc::position_t{4.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            ngc::position_t{5.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        };
        constexpr auto parameter = 0.4;
        const auto evaluated =
            ngc::evaluateExecutionPolynomial(span, parameter);
        const auto position = 10.0 + parameter * (1.0 + parameter
            * (2.0 + parameter * (3.0 + parameter * (4.0 + parameter * 5.0))));
        const auto velocity = (1.0 + parameter * (4.0 + parameter
            * (9.0 + parameter * (16.0 + parameter * 25.0)))) * 0.5;
        const auto acceleration = (4.0 + parameter
            * (18.0 + parameter * (48.0 + parameter * 100.0))) * 0.25;
        const auto jerk =
            (18.0 + parameter * (96.0 + parameter * 300.0)) * 0.125;
        require(std::abs(evaluated.state.position.x - position) < 1e-12
                    && std::abs(evaluated.state.velocity.x - velocity) < 1e-12
                    && std::abs(evaluated.state.acceleration.x
                        - acceleration) < 1e-12
                    && std::abs(evaluated.jerk.x - jerk) < 1e-12
                    && evaluated.state.position.y == -4.0
                    && evaluated.state.position.z == 3.0,
                "the production quintic span must evaluate PVA and jerk "
                "from normalized local coefficients");
        auto cubic = span;
        cubic.degree = ngc::ExecutionPolynomialDegree::Cubic;
        cubic.coefficients[3] = {};
        cubic.coefficients[4] = {};
        const auto cubicEvaluation =
            ngc::evaluateExecutionPolynomial(cubic, parameter);
        const auto cubicPosition = 10.0 + parameter
            * (1.0 + parameter * (2.0 + parameter * 3.0));
        require(std::abs(cubicEvaluation.state.position.x
                        - cubicPosition) < 1e-12,
                "the degree-aware evaluator should accept padded cubic spans");
    }

    void testShortLineMidpointCurvatureInference() {
        constexpr double DEFLECTION=0.1;
        constexpr double LENGTH=0.02;
        const auto curvature=ngc::spline_detail::inferShortLineMidpointCurvature(
            {std::cos(DEFLECTION),-std::sin(DEFLECTION),0.0},{1.0,0.0,0.0},
            {std::cos(DEFLECTION),std::sin(DEFLECTION),0.0},LENGTH);
        require(curvature.has_value(),
                "consistent short-line turns should infer a midpoint curvature");
        const auto expected=2.0*std::tan(0.5*DEFLECTION)/LENGTH;
        requireNear((*curvature)[0],0.0,
                    "short-line midpoint curvature should remain normal to the line");
        requireNear((*curvature)[1],expected,
                    "short-line midpoint curvature should use the tangent-circle radius");
        requireNear((*curvature)[2],0.0,
                    "planar short-line curvature should remain in its turn plane");
        require(!ngc::spline_detail::inferShortLineMidpointCurvature(
                    {std::cos(DEFLECTION),-std::sin(DEFLECTION),0.0},{1.0,0.0,0.0},
                    {std::cos(DEFLECTION),-std::sin(DEFLECTION),0.0},LENGTH),
                "a reversing short-line turn should retain zero midpoint curvature");
    }

    void testInfiniteJerkTrajectoryTimeMatchesAnalyticLine() {
        constexpr auto infinity=std::numeric_limits<double>::infinity();
        const ngc::InfiniteJerkPathPiece line {
            .length=10.0,
            .velocityLimit=3.0,
            .tangentAt=[](double) { return ngc::position_t{1,0,0,0,0,0}; },
            .curvatureAt=[](double) { return ngc::position_t{}; },
        };
        const auto timed=ngc::infiniteJerkTrajectoryTime(std::span{&line,std::size_t {1}}, {
            .pathAcceleration=2.0,
            .axisVelocity={infinity,infinity,infinity,infinity,infinity,infinity},
            .axisAcceleration={infinity,infinity,infinity,infinity,infinity,infinity},
        },0.0,0.0,{.maximumRefinements=11,.relativeTolerance=1e-7});
        require(timed.has_value(),timed?"":timed.error());
        require(std::abs(timed->duration-29.0/6.0)<5e-6,std::format(
            "infinite-jerk line time should match the analytic trapezoidal profile: "
            "actual={} expected={} error={} intervals={} refinements={}",timed->duration,
            29.0/6.0,timed->estimatedDurationError,timed->intervals,timed->refinements));
        requireNear(timed->maximumVelocity,3.0,
            "infinite-jerk line time should reach its programmed velocity");

        const ngc::InfiniteJerkPathPiece shortLine {
            .length=2.0,
            .velocityLimit=10.0,
            .tangentAt=line.tangentAt,
            .curvatureAt=line.curvatureAt,
        };
        const auto triangular=ngc::infiniteJerkTrajectoryTime(
            std::span{&shortLine,std::size_t {1}}, {
                .pathAcceleration=2.0,
                .axisVelocity={infinity,infinity,infinity,infinity,infinity,infinity},
                .axisAcceleration={infinity,infinity,infinity,infinity,infinity,infinity},
            });
        require(triangular.has_value(),triangular?"":triangular.error());
        requireNear(triangular->duration,2.0,
            "infinite-jerk line time should match the analytic triangular profile");

        const ngc::simulation_detail::ArcReference quarterCircle(ngc::MoveArc {
            {1,0,0,0,0,0},{0,1,0,0,0,0},{},{0,0,1},60.0});
        require(quarterCircle.valid(),"infinite-jerk curvature fixture should be a valid arc");
        const auto curvature=quarterCircle.curvatureAtDistance(quarterCircle.length()/2.0);
        requireNear(curvature.length(),1.0,
            "analytic unit-circle arc-length curvature should have unit magnitude");
        require(std::abs(curvature.x+std::numbers::sqrt2/2.0)<1e-9
                    &&std::abs(curvature.y+std::numbers::sqrt2/2.0)<1e-9,
                "analytic unit-circle curvature should point toward the center");
    }

    void testVerifiedCubicArcSpanCounts() {
        constexpr double TOLERANCE = 0.0001;
        struct ArcCase {
            std::string_view name;
            double radius;
            double sweep;
            double rise;
            std::uint32_t expectedSpans;
        };
        constexpr std::array cases {
            ArcCase { "quarter circle", 1.0, std::numbers::pi / 2.0, 0.0, 12 },
            ArcCase { "half circle", 1.0, std::numbers::pi, 0.0, 24 },
            ArcCase { "major three-quarter circle", 1.0, 3.0 * std::numbers::pi / 2.0, 0.0, 24 },
            ArcCase { "full circle", 1.0, 2.0 * std::numbers::pi, 0.0, 48 },
            ArcCase { "full helical turn", 1.0, 2.0 * std::numbers::pi, 1.0, 48 },
            ArcCase { "large-radius quarter circle", 10.0, std::numbers::pi / 2.0, 0.0, 12 },
        };

        for(const auto &test : cases) {
            const ngc::position_t from { test.radius, 0, 0, 0, 0, 0 };
            const ngc::position_t to {
                test.radius * std::cos(test.sweep), test.radius * std::sin(test.sweep), test.rise, 0, 0, 0,
            };
            ngc::TrajectoryCompiler planner({
                .pathAcceleration = std::numeric_limits<double>::infinity(),
                .rapidSpeed = 120.0,
                .arcChordTolerance = TOLERANCE,
            });
            planner.reset(20, from);
            const auto chunk = planner.compile(ngc::MoveArc { from, to, {}, { 0, 0, 1 }, 60.0 });
            require(chunk.has_value(), chunk ? "" : chunk.error());
            require(chunk->normalMotion.size == test.expectedSpans,
                    std::format("{} should compile to {} verified cubic spans, got {}",
                                test.name, test.expectedSpans, chunk->normalMotion.size));

            const auto pathLength = std::hypot(test.radius * test.sweep, test.rise);
            double elapsed = 0.0;
            double maximumError = 0.0;
            for(const auto &span : chunk->normalMotion) {
                for(int sample = 0; sample <= 64; ++sample) {
                    const auto local = static_cast<double>(sample) / 64.0;
                    const auto progress = std::clamp((elapsed + local*span.duration) / pathLength, 0.0, 1.0);
                    const auto actual = evaluateSpan(span, local);
                    const ngc::position_t expected {
                        test.radius * std::cos(test.sweep*progress),
                        test.radius * std::sin(test.sweep*progress),
                        test.rise * progress, 0, 0, 0,
                    };
                    maximumError = std::max(maximumError, ngc::vec3_t {
                        actual.x-expected.x, actual.y-expected.y, actual.z-expected.z,
                    }.length());
                }
                elapsed += span.duration;
            }
            require(maximumError <= TOLERANCE + 1e-10,
                    std::format("{} cubic error {} exceeds tolerance {}", test.name, maximumError, TOLERANCE));
        }
    }

    void testPlannedArcsPreserveCanonicalEndpointContinuity() {
        constexpr double TOLERANCE = 0.0001;
        const ngc::position_t start { 1, 0, 0, 0, 0, 0 };
        const ngc::position_t junction { 0, 1.0004, 0, 0, 0, 0 };
        const ngc::position_t secondEnd { -1.0002, 0, 0, 0, 0, 0 };
        const ngc::MoveArc firstArc { start, junction, {}, { 0, 0, 1 }, 60.0 };
        const ngc::MoveArc secondArc { junction, secondEnd, {}, { 0, 0, 1 }, 60.0 };

        ngc::TrajectoryCompiler planner({
            .pathAcceleration = std::numeric_limits<double>::infinity(),
            .rapidSpeed = 120.0,
            .arcChordTolerance = TOLERANCE,
        });
        planner.reset(41, start);
        const auto first = planner.compile(firstArc);
        const auto second = planner.compile(secondArc);
        require(first.has_value(), first ? "" : first.error());
        require(second.has_value(), second ? "" : second.error());
        const auto firstStart = evaluateSpan(first->normalMotion[0], 0.0);
        const auto firstEnd = evaluateSpan(first->normalMotion[first->normalMotion.size-1], 1.0);
        const auto secondStart = evaluateSpan(second->normalMotion[0], 0.0);
        requireNear(firstStart.x, start.x, "rounded arc polynomial should start at canonical X");
        requireNear(firstStart.y, start.y, "rounded arc polynomial should start at canonical Y");
        requireNear(firstEnd.x, junction.x, "rounded arc polynomial should end at canonical X");
        requireNear(firstEnd.y, junction.y, "rounded arc polynomial should end at canonical Y");
        requireNear(firstEnd.x, secondStart.x, "consecutive arcs should have continuous X");
        requireNear(firstEnd.y, secondStart.y, "consecutive arcs should have continuous Y");
        requireNear(ngc::executionSpanEnd(
                        first->normalMotion[first->normalMotion.size - 1])
                        .position.y,
                    junction.y,
                    "cached arc terminal state should preserve the canonical endpoint");

        const ngc::position_t lineEnd { -2, 0, 0, 0, 0, 0 };
        const auto lineAfterArc = planner.compile(ngc::MoveLine { secondEnd, lineEnd, 60.0 });
        require(lineAfterArc.has_value(), lineAfterArc ? "" : lineAfterArc.error());
        const auto secondFinal = evaluateSpan(second->normalMotion[second->normalMotion.size-1], 1.0);
        const auto followingLineStart = evaluateSpan(lineAfterArc->normalMotion[0], 0.0);
        requireNear(secondFinal.x, followingLineStart.x, "arc-to-line junction should have continuous X");
        requireNear(secondFinal.y, followingLineStart.y, "arc-to-line junction should have continuous Y");

        ngc::TrajectoryCompiler lineThenArc({
            .pathAcceleration = std::numeric_limits<double>::infinity(),
            .rapidSpeed = 120.0,
            .arcChordTolerance = TOLERANCE,
        });
        const ngc::position_t lineStart { 2, 0, 0, 0, 0, 0 };
        lineThenArc.reset(42, lineStart);
        const auto precedingLine = lineThenArc.compile(ngc::MoveLine { lineStart, start, 60.0 });
        const auto followingArc = lineThenArc.compile(firstArc);
        require(precedingLine.has_value(), precedingLine ? "" : precedingLine.error());
        require(followingArc.has_value(), followingArc ? "" : followingArc.error());
        const auto precedingLineEnd = evaluateSpan(
            precedingLine->normalMotion[precedingLine->normalMotion.size-1], 1.0);
        const auto followingArcStart = evaluateSpan(followingArc->normalMotion[0], 0.0);
        requireNear(precedingLineEnd.x, followingArcStart.x, "line-to-arc junction should have continuous X");
        requireNear(precedingLineEnd.y, followingArcStart.y, "line-to-arc junction should have continuous Y");

        const ngc::simulation_detail::ArcReference reference(firstArc);
        double elapsed = 0.0;
        double maximumError = 0.0;
        for(const auto &span : first->normalMotion) {
            for(int index = 0; index <= 128; ++index) {
                const auto local = static_cast<double>(index)/128.0;
                const auto actual = evaluateSpan(span, local);
                const auto expected = reference.positionAtDistance(elapsed + local*span.duration);
                maximumError = std::max(maximumError, ngc::vec3_t {
                    actual.x-expected.x, actual.y-expected.y, actual.z-expected.z,
                }.length());
            }
            elapsed += span.duration;
        }
        require(maximumError <= TOLERANCE + 1e-10,
                std::format("rounded-radius arc error {} exceeds tolerance {}", maximumError, TOLERANCE));

    }

    void testRoundedRadiusArcPreservesDynamicLimits() {
        constexpr double ACCELERATION = 4.0;
        constexpr double JERK = 8.0;
        ngc::TrajectoryCompiler planner({
            .pathAcceleration = ACCELERATION,
            .rapidSpeed = 120.0,
            .arcChordTolerance = 0.0001,
            .pathJerk = JERK,
        });
        const ngc::position_t start { 1, 0, 0, 0, 0, 0 };
        planner.reset(43, start);
        const auto arc = planner.compile(ngc::MoveArc {
            start, { 0, 1.0004, 0, 0, 0, 0 }, {}, { 0, 0, 1 }, 60.0,
        });
        require(arc.has_value(), arc ? "" : arc.error());
        for(const auto &span : arc->normalMotion) {
            require(spanAcceleration(span, 0.0) <= ACCELERATION + 1e-8,
                    "rounded-radius arc should respect acceleration at span start");
            require(spanAcceleration(span, 1.0) <= ACCELERATION + 1e-8,
                    "rounded-radius arc should respect acceleration at span end");
            require(spanJerk(span) <= JERK + 1e-8,
                    "rounded-radius arc should respect the configured jerk limit");
        }
    }

    void testEndpointExactArcReferenceGeometryVariants() {
        constexpr double MISMATCH = 0.0004;
        const std::array arcs {
            ngc::MoveArc { { 1, 0, 0, 0, 0, 0 }, { 0, 1+MISMATCH, 0, 0, 0, 0 }, {}, { 0, 0, 1 }, 60.0 },
            ngc::MoveArc { { 1, 0, 0, 0, 0, 0 }, { 0, -1-MISMATCH, 0, 0, 0, 0 }, {}, { 0, 0, -1 }, 60.0 },
            ngc::MoveArc { { 1, 0, 0, 0, 0, 0 }, { 0, -1-MISMATCH, 0, 0, 0, 0 }, {}, { 0, 0, 1 }, 60.0 },
            ngc::MoveArc { { 1, 0, 0, 0, 0, 0 }, { 1, 0, 0, 0, 0, 0 }, {}, { 0, 0, 1 }, 60.0 },
            ngc::MoveArc { { 1, 0, 0, 0, 0, 0 }, { 0, 1+MISMATCH, 2, 0, 0, 0 }, {}, { 0, 0, 1 }, 60.0 },
            ngc::MoveArc { { 1, 0, 0, 0, 0, 0 }, { 0, 0, 1+MISMATCH, 0, 0, 0 }, {}, { 0, -1, 0 }, 60.0 },
        };
        for(const auto &arc : arcs) {
            ngc::simulation_detail::ArcInverseDiagnostics inverse;
            const ngc::simulation_detail::ArcReference reference(arc,&inverse);
            require(reference.valid(), "arc reference geometry variant should be valid");
            const auto from = reference.positionAtDistance(0.0);
            const auto to = reference.positionAtDistance(reference.length());
            requireNear(from.x, arc.from().x, "arc reference should preserve start X");
            requireNear(from.y, arc.from().y, "arc reference should preserve start Y");
            requireNear(from.z, arc.from().z, "arc reference should preserve start Z");
            requireNear(to.x, arc.to().x, "arc reference should preserve end X");
            requireNear(to.y, arc.to().y, "arc reference should preserve end Y");
            requireNear(to.z, arc.to().z, "arc reference should preserve end Z");
            const auto middleDistance=reference.length()*0.375;
            const auto firstParameter=reference.parameterAtDistance(middleDistance);
            const auto cachedParameter=reference.parameterAtDistance(middleDistance);
            require(firstParameter==cachedParameter&&inverse.exactCacheHits==1,
                    "arc inverse cache should return the bit-exact certified parameter");
            require(inverse.inverseIntegralEvaluations==inverse.newtonIterations
                        &&inverse.iterationLimitHits==0
                        &&inverse.maximumNewtonIterations<=12,
                    "arc geometry variants should retain bounded exact inverse correction");
        }
    }

    void testMockDiagnosticPositionsFollowServoPeriod() {
        const auto positionsAtPeriod = [](const double period) {
            ngc::MockMotionBackend backend;
            ngc::PlanChunk chunk;
            chunk.epoch = 4;
            chunk.id = 8;
            chunk.branch = 12;
            require(chunk.normalMotion.push(linearSpan(20, 0.0, 1.0, 0.1)),
                    "diagnostic normal span should fit in a chunk");
            require(chunk.stopTail.push(linearSpan(21, 1.0, 1.0, 0.01)),
                    "diagnostic stop span should fit in a chunk");
            require(backend.trySubmit(ngc::StartRequest { 1, 4 }) == ngc::SubmitResult::Submitted,
                    "diagnostic start request should publish");
            require(backend.tryPublish(chunk) == ngc::PublishResult::Published,
                    "diagnostic chunk should publish");
            backend.runUntilIdle(period);
            const auto trajectory = backend.trajectorySnapshot();
            require(!trajectory.spans.empty(), "mock backend should retain calculated position buffers");
            return trajectory.spans.front().positions;
        };

        const auto coarse = positionsAtPeriod(0.01);
        const auto fine = positionsAtPeriod(0.001);
        require(coarse.size() == 11,
                "a 100 ms span should retain its start and ten calculated 10 ms positions");
        require(fine.size() == 101,
                "a 100 ms span should retain its start and one hundred calculated 1 ms positions");
        require(fine.size() > coarse.size(),
                "shortening the servo period should produce shorter diagnostic line segments");
        requireNear(coarse.front().x, 0.0,
                    "diagnostic position buffer should retain the span start");
        requireNear(coarse.back().x, 1.0,
                    "diagnostic position buffer should retain the span end");
    }

    void testMachineConfigurationLoadsTrajectoryLimits() {
        const auto configuration=fixtureMachineConfiguration();
        require(configuration.has_value(), configuration ? "" : configuration.error());
        require(configuration->trajectory.pathAcceleration > 0.0,
                "machine configuration should load a validated path acceleration");
        require(configuration->trajectory.pathJerk > 0.0,
                "machine configuration should load a validated path jerk");
        require(configuration->trajectory.rapidSpeed > 0.0,
                "machine configuration should load and convert a validated rapid velocity");
        require(configuration->trajectory.arcChordTolerance > 0.0,
                "machine configuration should load a validated arc chord tolerance");
        require(configuration->trajectory.lookaheadDuration>0.0,
                "machine configuration should load a positive rolling lookahead duration");
        require(configuration->simulation.servoPeriod>0.0
                    &&configuration->simulation.schedulerPeriod
                        >=configuration->simulation.servoPeriod,
                "machine configuration should load positive compatible simulation periods");
        const auto schedulerTicks=configuration->simulation.schedulerPeriod
            /configuration->simulation.servoPeriod;
        requireNear(schedulerTicks,std::round(schedulerTicks),
                    "configured scheduler period should contain an integer number of servo periods");
        require(configuration->jogging.acceleration>0.0
                    &&configuration->jogging.jerk>0.0,
                "machine configuration should load positive global jog limits");
        require(configuration->feedHold.tangentialAcceleration > 0.0
                    && configuration->feedHold.tangentialJerk > 0.0,
                "machine configuration should load positive feed-hold braking limits");
        require(configuration->pendant.enabled
                    && configuration->pendant.driver == ngc::PendantDriver::VistaCncP2s
                    && configuration->pendant.step.fineDistance > 0.0
                    && configuration->pendant.step.coarseDistance > 0.0
                    && configuration->pendant.velocity.maxVelocityScale > 0.0
                    && configuration->pendant.velocity.maxVelocityScale <= 1.0
                    && configuration->pendant.velocity.fullScaleCountsPerSecond > 0.0
                    && configuration->pendant.velocity.leaseDuration
                        >= configuration->simulation.servoPeriod,
                "machine configuration should load validated VistaCNC P2-S jog settings");

        require(!configuration->coordinates.empty(),
                "machine configuration should load at least one logical coordinate");
        const auto axis = [&](const ngc::Machine::Axis value) -> const ngc::AxisConfiguration & {
            const auto found = std::ranges::find(configuration->axes, value, &ngc::AxisConfiguration::axis);
            require(found != configuration->axes.end(), "configured logical axis should exist");
            return *found;
        };
        const auto component=[](const ngc::Machine::Axis value) {
            switch(value) {
                case ngc::Machine::Axis::X: return &ngc::position_t::x;
                case ngc::Machine::Axis::Y: return &ngc::position_t::y;
                case ngc::Machine::Axis::Z: return &ngc::position_t::z;
                case ngc::Machine::Axis::A: return &ngc::position_t::a;
                case ngc::Machine::Axis::B: return &ngc::position_t::b;
                case ngc::Machine::Axis::C: return &ngc::position_t::c;
            }
            return &ngc::position_t::x;
        };
        for(const auto coordinate:configuration->coordinates) {
            const auto &configured=axis(coordinate);
            require(configured.minimum<configured.maximum&&!configured.joints.empty(),
                    "each logical axis should load a valid range and at least one joint");
            require(configured.maxVelocity>0.0&&configured.maxAcceleration>0.0
                        &&configured.maxJerk>0.0,
                    "each logical axis should load positive independent motion limits");
            const auto member=component(coordinate);
            requireNear(configuration->trajectory.axisVelocity.*member,configured.maxVelocity,
                        "trajectory limits should retain each configured axis velocity");
            requireNear(configuration->trajectory.axisAcceleration.*member,configured.maxAcceleration,
                        "trajectory limits should retain each configured axis acceleration");
            requireNear(configuration->trajectory.axisJerk.*member,configured.maxJerk,
                        "trajectory limits should retain each configured axis jerk");
        }

        const auto probingInput=std::ranges::find(
            configuration->digitalInputs,configuration->probing.input,
            &ngc::DigitalInputConfiguration::id);
        require(probingInput!=configuration->digitalInputs.end(),
                "probing should resolve to a configured digital input");
        require(configuration->probing.debounce>=0.0,
                "probing should load a non-negative debounce duration");
        const auto joint = [&](const ngc::JointId id) -> const ngc::JointConfiguration & {
            const auto found = std::ranges::find(configuration->joints, id, &ngc::JointConfiguration::id);
            require(found != configuration->joints.end(), "configured joint should exist");
            return *found;
        };
        require(!configuration->joints.empty()&&!configuration->homing.groups.empty(),
                "machine configuration should load physical joints and homing groups");
        for(const auto &configuredAxis:configuration->axes)
            for(const auto id:configuredAxis.joints)
                require(joint(id).axis==configuredAxis.axis,
                        "each logical axis should reference matching configured joints");
        for(const auto &configuredJoint:configuration->joints) {
            require(configuredJoint.minimum<configuredJoint.maximum
                        &&configuredJoint.maxVelocity>0.0
                        &&configuredJoint.maxAcceleration>0.0
                        &&configuredJoint.maxJerk>0.0,
                    "each joint should load a valid range and positive motion limits");
            require(configuredJoint.homing.searchVelocity!=0.0
                        &&configuredJoint.homing.latchVelocity!=0.0
                        &&configuredJoint.homing.backoffDistance>0.0
                        &&configuredJoint.homing.debounce>=0.0,
                    "each joint should load valid homing motion values");
            require(std::ranges::find(configuration->digitalInputs,
                        configuredJoint.homing.input,&ngc::DigitalInputConfiguration::id)
                        !=configuration->digitalInputs.end(),
                    "each joint home input should resolve to a configured digital input");
            const auto groupCount=std::ranges::count_if(configuration->homing.groups,
                [&](const auto &group) { return std::ranges::contains(group.joints,configuredJoint.id); });
            require(groupCount==1,"each configured joint should belong to exactly one homing group");
        }
    }

    void testConfiguredHomingMovesFromPowerUpPosition() {
        const auto configuration=fixtureMachineConfiguration();
        require(configuration.has_value(), configuration ? "" : configuration.error());
        SimulationWorker worker(*configuration);
        worker.setTickMultiplier(1000);
        auto snapshot = worker.snapshot();
        requireNear(snapshot.machinePosition.x, 6.0, "mock machine should power up at X6");
        requireNear(snapshot.machinePosition.y, 6.0, "mock machine should power up at Y6");
        requireNear(snapshot.machinePosition.z, -6.0, "mock machine should power up at Z-6");
        require(worker.home(), "configured simulated homing should start");
        for(int attempt = 0; attempt < 5000
            && snapshot.status != ngc::SimulationStatus::Completed
            && snapshot.status != ngc::SimulationStatus::Error; ++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            snapshot = worker.snapshot();
        }
        require(snapshot.status == ngc::SimulationStatus::Completed,
                std::format("configured simulated homing should complete: {}", snapshot.error));
        const auto configuredHome = [&](const ngc::Machine::Axis axis) {
            double sum = 0.0;
            std::size_t count = 0;
            for(const auto &joint : configuration->joints) if(joint.axis == axis) {
                sum += joint.homing.homePosition;
                ++count;
            }
            require(count != 0, "homing regression axis should have a configured joint");
            return sum / count;
        };
        requireNear(snapshot.machinePosition.x, configuredHome(ngc::Machine::Axis::X),
                    "X should finish at its configured home position");
        requireNear(snapshot.machinePosition.y, configuredHome(ngc::Machine::Axis::Y),
                    "both Y joints should finish squared at their configured home position");
        requireNear(snapshot.machinePosition.z, configuredHome(ngc::Machine::Axis::Z),
                    "Z should finish at its configured home position");
        const auto toolPose = ngc::simulationToolPose(snapshot);
        require(toolPose.geometry.number == 0,
                "homing without a loaded tool should retain the no-tool presentation state");
        requireNear(toolPose.tipPosition.x, snapshot.machinePosition.x,
                    "the no-tool position marker should track the machine position");
        worker.join();
    }

    void testSimulationWorkerJogsCoupledJointsBeforeHoming() {
        auto configuration=fixtureMachineConfiguration();
        require(configuration.has_value(), configuration ? "" : configuration.error());
        configuration->simulation.schedulerPeriod = configuration->simulation.servoPeriod;
        SimulationWorker worker(*configuration);

        const auto axis = std::ranges::find(configuration->axes, ngc::Machine::Axis::Y,
                                            &ngc::AxisConfiguration::axis);
        require(axis != configuration->axes.end(), "pre-home jog test should find configured Y");
        ngc::JointMask joints = 0;
        double jerk = std::numeric_limits<double>::infinity();
        for(const auto id : axis->joints) {
            joints |= static_cast<ngc::JointMask>(ngc::JointMask { 1 } << id);
            const auto joint = std::ranges::find(configuration->joints, id,
                                                 &ngc::JointConfiguration::id);
            require(joint != configuration->joints.end(), "Y jog group joint should be configured");
            jerk = std::min(jerk, joint->maxJerk);
        }
        const ngc::StartContinuousJogRequest request {
            .id = 100,
            .jog = 200,
            .target = { ngc::JogTargetType::JointGroup, ngc::AxisId::Y, joints },
            .signedVelocity = 0.5,
            .limits = { axis->maxVelocity, configuration->jogging.acceleration,
                        configuration->jogging.jerk },
            .stopLimits = { axis->maxVelocity, axis->maxAcceleration, jerk },
            .leaseTicks = 5,
        };
        require(worker.startJog(ngc::ControlRequest { request }),
                "coupled Y jogging should be allowed before homing");

        auto snapshot = worker.snapshot();
        for(int attempt = 0; attempt < 2000
            && snapshot.status != ngc::SimulationStatus::Completed
            && snapshot.status != ngc::SimulationStatus::Error; ++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            snapshot = worker.snapshot();
        }
        require(snapshot.status == ngc::SimulationStatus::Completed,
                std::format("pre-home coupled jog should stop safely: {}", snapshot.error));
        require(snapshot.lastJogStopReason == ngc::JogStopReason::LeaseExpired,
                "worker should expose backend lease expiry as the jog stop reason");
        require(snapshot.homedJoints == 0,
                "joint jogging must not falsely mark the machine as homed");
        require(snapshot.machinePosition.y > 6.0,
                "pre-home coupled jogging should visibly move the simulated Y axis");
        requireNear(snapshot.joints.position[1], snapshot.joints.position[2],
                    "pre-home coupled Y jogging should keep both ball-screw joints together");
        worker.join();
    }

    void testProbeCompilesAsBackendOwnedTriggeredMove() {
        constexpr double ACCELERATION = 2.0;
        constexpr double JERK = 10.0;
        ngc::TrajectoryCompiler planner({
            .pathAcceleration = ACCELERATION,
            .rapidSpeed = 100.0,
            .arcChordTolerance = 0.0001,
            .pathJerk = JERK,
            .axisVelocity = { std::numeric_limits<double>::infinity(),
                std::numeric_limits<double>::infinity(), 0.1,
                std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
                std::numeric_limits<double>::infinity() },
            .axisAcceleration = { std::numeric_limits<double>::infinity(),
                std::numeric_limits<double>::infinity(), 0.5,
                std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
                std::numeric_limits<double>::infinity() },
            .axisJerk = { std::numeric_limits<double>::infinity(),
                std::numeric_limits<double>::infinity(), 1.0,
                std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
                std::numeric_limits<double>::infinity() },
        });
        planner.reset(3, {});
        const ngc::ProbeMove probeCommand {
            77, {}, { 0, 0, -1, 0, 0, 0 }, 10.0, true, false };
        const auto move = planner.compileTriggeredMove(probeCommand);
        require(move.has_value(), move ? "" : move.error());
        require(move->moveId == 77, "triggered move should retain the interpreter probe ID");
        requireNear(move->target.z, -1.0, "triggered move should retain its machine target");
        requireNear(std::abs(move->limits.velocity.z), 0.1,
                    "triggered moves should use the limiting physical-axis velocity");
        requireNear(std::abs(move->limits.acceleration.z), 0.5,
                    "triggered moves should use the limiting physical-axis acceleration");
        requireNear(std::abs(move->limits.jerk.z), 1.0,
                    "triggered moves should use the limiting physical-axis jerk");

        ngc::MockMotionBackend backend;
        require(backend.trySubmit(ngc::StartRequest { 1, 3 }) == ngc::SubmitResult::Submitted,
                "triggered-move backend should start");
        require(backend.configureSyntheticInput(move->moveId, { 0, 0, -0.5, 0, 0, 0 }),
                "mock input transition should publish outside the production RT interface");
        require(backend.tryPublish(ngc::ExecutionItem { *move }) == ngc::PublishResult::Published,
                "triggered move should publish through the unified execution transport");
        ngc::ExecutionSnapshot previous;
        bool havePrevious = false;
        double maximumAcceleration = 0.0;
        double maximumJerk = 0.0;
        for(int tick = 0; tick < 100000; ++tick) {
            backend.advanceTick(0.001, true);
            ngc::ExecutionSnapshot snapshot;
            while(backend.tryTakeSnapshot(snapshot)) {
                const auto norm = [](const ngc::position_t &value) {
                    return std::sqrt(value.x*value.x + value.y*value.y + value.z*value.z
                        + value.a*value.a + value.b*value.b + value.c*value.c);
                };
                maximumAcceleration = std::max(maximumAcceleration, norm(snapshot.commanded.acceleration));
                if(havePrevious)
                    maximumJerk = std::max(maximumJerk,
                        norm(snapshot.commanded.acceleration - previous.commanded.acceleration) / 0.001);
                previous = snapshot;
                havePrevious = true;
            }
            if(havePrevious && previous.state == ngc::BackendState::Held) break;
        }
        bool completed = false;
        ngc::TriggeredMoveCompleted result{};
        ngc::ExecutionEvent event;
        while(backend.tryTakeEvent(event)) if(const auto *value = std::get_if<ngc::TriggeredMoveCompleted>(&event)) {
            completed = true;
            result = *value;
        }
        require(completed && result.status == ngc::TriggeredMoveStatus::Triggered,
                "mock input transition should complete the triggered move");
        require(result.triggerState.position.z <= -0.5 && result.triggerState.position.z > -0.51,
                "trigger state should be latched on the first servo sample after contact");
        require(result.stoppedState.position.z < result.triggerState.position.z,
                "triggered motion should overshoot contact while stopping within constraints");
        require(maximumAcceleration <= ACCELERATION * 1.001,
                "triggered approach and stop should respect the configured acceleration limit");
        require(maximumJerk <= JERK * 1.02,
                std::format("triggered approach and stop should respect the configured jerk limit ({})", maximumJerk));
        const auto diagnostics = backend.trajectorySnapshot();
        require(diagnostics.spans.size() >= 2 && diagnostics.spans.back().stopTail,
                "triggered stop should be retained in the reusable diagnostic position buffer");
    }

    void testDualScrewJointMoveStopsEachMotorOnItsOwnSwitch() {
        ngc::TriggeredJointMove move;
        move.epoch = 9;
        move.id = 1;
        move.branch = 1;
        move.moveId = 400;
        move.joints = (ngc::JointMask{1} << 0) | (ngc::JointMask{1} << 1);
        move.targetMode = ngc::JointTargetMode::Relative;
        move.target[0] = 2.0;
        move.target[1] = 2.0;
        for(const ngc::JointId joint : { ngc::JointId{0}, ngc::JointId{1} }) {
            move.limits.velocity[joint] = 1.0;
            move.limits.acceleration[joint] = 2.0;
            move.limits.jerk[joint] = 10.0;
        }
        require(move.triggers.push({ 0, 10, ngc::InputCondition::Active, 0.010 }),
                "first Y joint trigger should fit");
        require(move.triggers.push({ 1, 11, ngc::InputCondition::Active, 0.010 }),
                "second Y joint trigger should fit");
        move.triggerRequired = true;

        ngc::MockMotionBackend backend;
        require(backend.trySubmit(ngc::EnableRequest { 1 }) == ngc::SubmitResult::Submitted,
                "joint backend should enable into held state");
        backend.advance(0.0);
        ngc::JointVector initial{};
        initial[0] = 5.0;
        initial[1] = 5.0;
        require(backend.trySubmit(ngc::SetJointPositionRequest { 2, move.joints, initial })
                    == ngc::SubmitResult::Submitted,
                "arbitrary pre-home joint coordinates should be accepted while held");
        backend.advance(0.0);
        require(backend.trySubmit(ngc::StartRequest { 3, 9 }) == ngc::SubmitResult::Submitted,
                "joint backend should start");
        require(backend.configureSyntheticJointInput(move.moveId, 0, 5.7),
                "first Y home switch should configure");
        require(backend.configureSyntheticJointInput(move.moveId, 1, 6.0),
                "second Y home switch should configure");
        require(backend.tryPublish(ngc::ExecutionItem { move }) == ngc::PublishResult::Published,
                "joint triggered move should publish");

        bool sawIndependentStop = false;
        ngc::ExecutionSnapshot latest;
        for(int tick = 0; tick < 10000; ++tick) {
            backend.advanceTick(0.001, true);
            ngc::ExecutionSnapshot snapshot;
            while(backend.tryTakeSnapshot(snapshot)) {
                latest = snapshot;
                if(std::abs(snapshot.commandedJoints.velocity[0]) <= 1e-9
                   && snapshot.commandedJoints.position[0] > 5.7
                   && snapshot.commandedJoints.velocity[1] > 1e-6)
                    sawIndependentStop = true;
            }
            if(latest.state == ngc::BackendState::Held) break;
        }

        bool completed = false;
        ngc::TriggeredJointMoveCompleted result{};
        ngc::ExecutionEvent event;
        while(backend.tryTakeEvent(event))
            if(const auto *value = std::get_if<ngc::TriggeredJointMoveCompleted>(&event)) {
                completed = true;
                result = *value;
            }
        require(completed && result.status == ngc::TriggeredMoveStatus::Triggered,
                "both Y joints should complete from their own switch transitions");
        require(result.triggeredJoints == move.joints,
                "joint completion should identify both triggered Y motors");
        require(sawIndependentStop,
                "the first Y motor should remain stopped while the second continues toward its switch");
        require(result.triggerState.position[0] >= 5.7 - 1e-9 && result.triggerState.position[0] < 5.71,
                std::format("first Y motor should latch its own switch position ({})",
                            result.triggerState.position[0]));
        require(result.triggerState.position[1] >= 6.0 - 1e-9 && result.triggerState.position[1] < 6.01,
                std::format("second Y motor should latch its own switch position ({})",
                            result.triggerState.position[1]));
        require(result.stoppedState.position[0] > result.triggerState.position[0]
               && result.stoppedState.position[1] > result.triggerState.position[1],
                "each Y motor should decelerate after its switch transition");
        require(std::abs(result.stoppedState.velocity[0]) <= 1e-12
               && std::abs(result.stoppedState.velocity[1]) <= 1e-12,
                "both Y motors should be stationary when joint completion is reported");

        ngc::JointVector squared{};
        require(backend.trySubmit(ngc::SetJointPositionRequest { 4, move.joints, squared })
                    == ngc::SubmitResult::Submitted,
                "held-state joint position request should publish");
        backend.advance(0.0);
        bool positionSet = false;
        while(backend.tryTakeEvent(event))
            if(const auto *completedRequest = std::get_if<ngc::RequestCompleted>(&event))
                if(completedRequest->request == 4) positionSet = completedRequest->succeeded;
        require(positionSet, "squaring offsets should be established only after both joints are held");
    }

    void testMotionWordSynchronizationPolicy() {
        struct Case {
            std::string_view expression;
            bool shouldSynchronize;
        };
        for(const auto &[expression, shouldSynchronize] : {
                Case { "[1]", false },
                Case { "[1 + 1]", false },
                Case { "[#5400]", true },
                Case { "#5400", true },
            }) {
            ngc::InterpreterSession session(UNIT, ngc::InterpretationMode::Simulation);
            compileSession(session, std::format("G1 F60 X0.5\nG1 X{}\n", expression));

            bool sawFirstMotion = false;
            for(int guard = 0; guard < 20 && !sawFirstMotion; ++guard) {
                const auto event = session.nextWithBlocks([](const auto &callback) { callback(); });
                sawFirstMotion = std::holds_alternative<ngc::MachineCommand>(event);
            }
            require(sawFirstMotion, "bracket synchronization test should emit its preceding motion");

            bool synchronized = false;
            bool emittedSecondMotion = false;
            for(int guard = 0; guard < 20 && !synchronized && !emittedSecondMotion; ++guard) {
                const auto event = session.nextWithBlocks([](const auto &callback) { callback(); });
                synchronized = std::holds_alternative<ngc::InterpreterWaitingForSynchronization>(event);
                emittedSecondMotion = std::holds_alternative<ngc::MachineCommand>(event);
            }
            require(synchronized == shouldSynchronize,
                    std::format("motion word {} synchronization policy mismatch", expression));
            require(emittedSecondMotion != shouldSynchronize,
                    std::format("motion word {} emitted at the wrong side of synchronization", expression));
        }
    }
}

int main() {
    try {
        testMemoryStackBounds();
        testToolTableLoadsFinalLineWithoutNewline();
        testToolTableRejectsDuplicateToolNumbers();
        testNumericParsingRejectsTrailingGarbage();
        testLexerRejectsIncompleteOperators();
        testFileHelpersHandleEmptyAndFailedIo();
        testRapidAndFeedMove();
        testG64IsAnInertPathModeFlag();
        testG64BlendScaleGeometryProgramIsValid();
        testFeedMotionRequiresFeedrate();
        testUnsupportedCodesProduceInterpreterErrors();
        testFailedBlockRollsBackMachineState();
        testInterpreterCancellationInterruptsEvaluation();
        std::cerr << "checkpoint simulation start\n";
        testSimulationWorkerStartsPlayback();
        testSimulationPresentationFollowsNestedToolChangeExecution();
        std::cerr << "checkpoint adaptive start\n";
        testAdaptivePocketsStartsSimulation();
        std::cerr << "checkpoint prepared G64 refill\n";
        testTimedSimulationRefillsMultiPacketContinuousBatch();
        std::cerr << "checkpoint snapshots\n";
        testTimedSimulationPublishesSnapshotsDuringPlanning();
        std::cerr << "checkpoint preview\n";
        testImmediatePreviewBuildsGeometryWithoutTrajectoryExecution();
        testGeometryProducerBlendsAcrossMotionModeChanges();
        testGeometryProducerPreparesExactStopPreviewSlices();
        testModalG64RapidsRemainExactPreparedMotion();
        testZeroLengthFeedBetweenPreparedMotionChains();
        testGeometryPreviewResolvesProbeAtCanonicalTarget();
        testAdaptivePocketsGeometryPreviewAvoidsTrajectoryExecution();
        testIncrementalGeometryDefersAndDoesNotRebuildAnchorSection();
        testSimulationDriverFailureAppearsInGuiStatusStream();
        testMockBackendFeedHoldBrakesAlongActiveTrajectory();
        testMockBackendFeedHoldStopBranchIsFatal();
        testMockBackendFeedHoldPausesAndResumesProbeApproach();
        testMockBackendProbeContactDuringFeedHoldStopIsDetected();
        testSimulationWorkerFeedHoldReachesPausedAtRest();
        testSimulationWorkerFeedHoldResumesProbeApproach();
        testSimulationWorkerProbeContactSupersedesFeedHold();
        test1001PreviewCompletesBoundedly();
        testSingleShortEntityClusterRetainsMidpointControl();
        test1002PreparedSliceBoundaries();
        testMdiToolChangeUsesAutoloadPrograms();
        testSimulationWorkerPersistsUntilReset();
        testSimulationProgramElapsedTimeIsPlaybackSpeedIndependent();
        testInterpreterStatusMessagesPreserveOrder();
        testArcUsesModalFeedrate();
        testArcCenterDistanceModes();
        testIncrementalMove();
        testBeginProgramRunResetsRuntimeState();
        testLinearUnitConversion();
        testToolLengthCompensation();
        testToolLengthCompensationUsesActiveTool();
        testToolLengthCompensationWithWorkOffset();
        testG53BypassesWorkOffsetButRetainsToolOffset();
        testArcAllowsOmittedZeroCenterOffsets();
        testArcGeometryValidation();
        testArcRadiusMismatchIsRecoverableInterpreterError();
        testInterpreterSessionOwnsCompilationAndExecution();
        testInterpreterTaskVariable();
        testIncrementalSessionControlFlow();
        testProbeCommandAndBarrier();
        testAutomaticToolChangeReachesProbeBarrier();
        testSpscChannelIsBoundedAndOrdered();
        testOwningSpscChannelTransfersMoveOnlyValues();
        testMockMotionBackendUsesProductionTransportContract();
        testMockBackendEmitsOrderedInSpanExecutionMarkers();
        testContinuousMarkerBoundPacketsExecuteWithoutIntermediateStops();
        testJogControlUsesBoundedBackendTransport();
        testMockBackendAdvancesOneFixedServoTick();
        testExactStopPlannerCompilesLinesAndArcs();
        testInfiniteJerkTrajectoryTimeMatchesAnalyticLine();
        testExactStopPlannerEnforcesIndependentAxisLimits();
        testPreparedArcJunctionMatchesSourceCurvature();
        testNoneSplineSmoothingPreservesCubicControls();
        testClusterSplinePreparesKnotIntervalSamplesAndFeeds();
        testExecutionPolynomialEvaluation();
        testCollinearJunctionBlendUsesLinearTiming();
        testShortLineMidpointCurvatureInference();
        testVerifiedCubicArcSpanCounts();
        testPlannedArcsPreserveCanonicalEndpointContinuity();
        testRoundedRadiusArcPreservesDynamicLimits();
        testEndpointExactArcReferenceGeometryVariants();
        testMockDiagnosticPositionsFollowServoPeriod();
        testMachineConfigurationLoadsTrajectoryLimits();
        testConfiguredHomingMovesFromPowerUpPosition();
        testSimulationWorkerJogsCoupledJointsBeforeHoming();
        testProbeCompilesAsBackendOwnedTriggeredMove();
        testDualScrewJointMoveStopsEachMotorOnItsOwnSwitch();
        testMotionWordSynchronizationPolicy();
    } catch(const std::exception &error) {
        std::cerr << "ngc_tests failed: " << error.what() << '\n';
        return 1;
    }

    std::cout << "ngc_tests passed\n";
    return 0;
}
