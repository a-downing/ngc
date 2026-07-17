#include <array>
#include <chrono>
#include <cmath>
#include <exception>
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
#include "machine/ExactStopTrajectoryPlanner.h"
#include "machine/MockMotionBackend.h"
#include "machine/SpscChannel.h"
#include "machine/ToolpathRecorder.h"
#include "machine/ToolTable.h"
#include "machine/TrajectoryExecutionDriver.h"
#include "memory/Memory.h"
#include "parser/Program.h"
#include "PreviewSpline.h"
#include "SimulationWorker.h"
#include "Worker.h"

namespace {
    constexpr double EPSILON = 1e-9;
    constexpr ngc::Machine::Unit UNIT = ngc::Machine::Unit::Inch;

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

        ngc::ToolpathRecorder recorder;
        recorder.consume(commands.front(), {}, std::nullopt, true, machine.pathTolerance());
        require(recorder.g64Active(0), "preview recording should retain the command's G64 flag");
        requireNear(*recorder.g64Tolerance(0), 1.0 / 25.4,
                    "preview recording should retain the active G64 tolerance");

        execute(machine, "G61\n");
        require(!machine.pathTolerance(), "leaving G64 should clear its path tolerance");
    }

    void testG64BlendScaleGeometryProgramIsValid() {
        const auto source=ngc::readFile("g64_blend_scale_test.ngc");
        require(source.has_value(),source ? "" : source.error().what());
        ngc::Machine machine(UNIT);
        const auto commands=execute(machine,*source);
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
        require(worker.start({ { "sub _tool_change[#tool] {}\nT1 M6\nG0 X1\n", "simulation-worker.ngc" } }, tools),
                "simulation worker should accept a program");

        auto snapshot = worker.snapshot();
        require(snapshot.status == ngc::SimulationStatus::Running,
                "simulation worker should publish its running state synchronously from start");
        for(int attempt = 0; attempt < 5000 && snapshot.toolPose.geometry.number != 1; ++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            snapshot = worker.snapshot();
        }
        const auto toolMessage = std::format("simulation worker did not publish tool 1: status {} tool {} error '{}'",
                                             static_cast<int>(snapshot.status), snapshot.toolPose.geometry.number,
                                             snapshot.error);
        require(snapshot.toolPose.geometry.number == 1, toolMessage);
        requireNear(snapshot.toolPose.tipPosition.z, snapshot.toolPose.spindlePosition.z - 2.0,
                    "simulation worker should publish the physical cutter position");
        worker.join();
    }

    void testAdaptivePocketsStartsSimulation() {
        const auto hello = ngc::readFile("autoload/hello.ngc");
        const auto world = ngc::readFile("autoload/world.ngc");
        const auto toolChange = ngc::readFile("autoload/tool_change.ngc");
        auto main = ngc::readFile("adaptive_pockets.ngc");
        require(hello && world && toolChange && main, "adaptive-pockets simulation inputs should load");
        const auto g64=main->find("N40 G64 P");
        require(g64!=std::string::npos,"adaptive-pockets startup fixture should contain its G64 block");
        const auto g64End=main->find('\n',g64);
        main->replace(g64,g64End==std::string::npos?main->size()-g64:g64End-g64,"N40 G61");
        ngc::ToolTable tools;
        const auto loadedTools = tools.load();
        require(loadedTools.has_value(), loadedTools ? "" : loadedTools.error());

        SimulationWorker worker;
        require(worker.start({ { *hello, "autoload/hello.ngc" }, { *world, "autoload/world.ngc" },
                               { *toolChange, "autoload/tool_change.ngc" }, { *main, "adaptive_pockets.ngc" } }, tools),
                "adaptive-pockets simulation should start");

        auto snapshot = worker.snapshot();
        for(int attempt = 0; attempt < 2000 && snapshot.activeBlocks.empty()
            && snapshot.completedLineFlags.empty() && snapshot.status != ngc::SimulationStatus::Error; ++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            snapshot = worker.snapshot();
        }
        const auto message = std::format("adaptive-pockets simulation did not publish block lifecycle: status {} error '{}'",
                                         static_cast<int>(snapshot.status), snapshot.error);
        require(!snapshot.activeBlocks.empty() || !snapshot.completedLineFlags.empty(), message);
        worker.join();
    }

    void testTimedSimulationRefillsMultiPacketContinuousBatch() {
        // Keep each line longer than 6P: this fixture exercises packet refill,
        // not the deliberately unsplittable all-short cluster path.
        std::string source="G64 P0.0001\n";
        constexpr int COMMANDS=800;
        for(int command=1;command<=COMMANDS;++command)
            source+=std::format("G1 F60 X{:.6f}\n",static_cast<double>(command)*0.001);

        SimulationWorker worker;
        worker.setTickMultiplier(1000);
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
            "timed simulation should refill a continuous batch larger than its eight-slot queue: {}",
            snapshot.error));
        require(snapshot.trajectoryPlanning.planChunks>8
                    &&snapshot.trajectoryPlanning.maximumWindowCommands==COMMANDS
                    &&snapshot.trajectoryPlanning.maximumNormalSpans==ngc::MAX_NORMAL_SPANS_PER_CHUNK,
                std::format("multi-packet refill regression should exceed both old command and RT packet boundaries: "
                            "chunks={} commands={} maximum spans={}",
                            snapshot.trajectoryPlanning.planChunks,
                            snapshot.trajectoryPlanning.maximumWindowCommands,
                            snapshot.trajectoryPlanning.maximumNormalSpans));
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
        const auto initialRevision=worker.lock([&] { return worker.toolpath().revision(); });
        require(worker.execute(),"multi-packet immediate preview should start execution");
        bool finished=false;
        for(int attempt=0;attempt<10000;++attempt) {
            const auto revision=worker.lock([&] { return worker.toolpath().revision(); });
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
        require(worker.lock([&] { return worker.toolpath().commands().size()==COMMANDS; }),
                "multi-packet immediate preview should retain every canonical command");
        require(worker.lock([&] {
            const auto &toolpath=worker.toolpath();
            for(std::size_t index=0;index<toolpath.commands().size();++index)
                if(!toolpath.g64Active(index)||toolpath.g64Tolerance(index)!=0.001) return false;
            return true;
        }),"geometry-only preview should retain G64/P metadata for display blending");
        worker.join();
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
        const auto initialRevision=worker.lock([&] { return worker.toolpath().revision(); });
        require(worker.execute(),"geometry probe preview should start");
        bool finished=false;
        for(int attempt=0;attempt<3000;++attempt) {
            const auto revision=worker.lock([&] { return worker.toolpath().revision(); });
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
            const auto &commands=worker.toolpath().commands();
            require(commands.size()==2,"probe preview should retain the probe and following line");
            const auto *probe=std::get_if<ngc::ProbeMove>(&commands[0]);
            require(probe!=nullptr,"probe preview should retain canonical probe geometry");
            requireNear(probe->target().z,-1.0,
                        "probe preview endpoint should be the commanded probe target");
        });
        worker.join();
    }

    void testAdaptivePocketsGeometryPreviewAvoidsTrajectoryExecution() {
        const auto hello=ngc::readFile("autoload/hello.ngc");
        const auto world=ngc::readFile("autoload/world.ngc");
        const auto toolChange=ngc::readFile("autoload/tool_change.ngc");
        const auto main=ngc::readFile("adaptive_pockets.ngc");
        require(hello&&world&&toolChange&&main,
                "adaptive-pockets geometry preview inputs should load");
        ngc::ToolTable tools;
        const auto loadedTools=tools.load();
        require(loadedTools.has_value(),loadedTools?"":loadedTools.error());

        Worker worker(UNIT);
        require(worker.setToolTable(tools),
                "adaptive-pockets geometry preview should accept the tool table");
        require(worker.compile({{*hello,"autoload/hello.ngc"},{*world,"autoload/world.ngc"},
                                {*toolChange,"autoload/tool_change.ngc"},
                                {*main,"adaptive_pockets.ngc"}}),
                "adaptive-pockets geometry preview should start compilation");
        for(int attempt=0;attempt<5000&&!worker.compiled();++attempt)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        require(worker.compiled(),"adaptive-pockets geometry preview should compile");
        const auto initialRevision=worker.lock([&] { return worker.toolpath().revision(); });
        const auto started=std::chrono::steady_clock::now();
        require(worker.execute(),"adaptive-pockets geometry preview should start");
        bool finished=false;
        for(int attempt=0;attempt<10000;++attempt) {
            const auto revision=worker.lock([&] { return worker.toolpath().revision(); });
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
            const auto &toolpath=worker.toolpath();
            return !toolpath.commands().empty()&&std::ranges::any_of(
                std::views::iota(std::size_t{0},toolpath.commands().size()),
                [&](const auto index) { return toolpath.g64Active(index); });
        }),"adaptive-pockets preview should retain canonical motion and G64 display metadata");
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
        worker.join();
    }

    void test1001PreviewCompletesBoundedly() {
        const auto hello = ngc::readFile("autoload/hello.ngc");
        const auto world = ngc::readFile("autoload/world.ngc");
        const auto toolChange = ngc::readFile("autoload/tool_change.ngc");
        auto main = ngc::readFile("1001.ngc");
        require(hello && world && toolChange && main, "1001 preview inputs should load");
        const auto toolChangeBlock = main->find("N25 T13 M6");
        require(toolChangeBlock != std::string::npos, "1001 should retain its expected tool-change block");
        main->replace(toolChangeBlock, std::string_view("N25 T13 M6").size(), "N25 T13");

        ngc::ToolTable tools;
        const auto loadedTools = tools.load();
        require(loadedTools.has_value(), loadedTools ? "" : loadedTools.error());

        // A coarse mock tick keeps this regression focused on NRT G64 planning
        // rather than the duration of immediate simulated playback.
        Worker worker(UNIT);
        require(worker.setToolTable(tools), "1001 preview should accept the tool table");
        require(worker.compile({ { *hello, "autoload/hello.ngc" }, { *world, "autoload/world.ngc" },
                                 { *toolChange, "autoload/tool_change.ngc" }, { *main, "1001.ngc" } }),
                "1001 preview should start compilation");
        for(int attempt = 0; attempt < 3000 && !worker.compiled(); ++attempt)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        require(worker.compiled(), "1001 preview should compile");

        const auto initialRevision = worker.lock([&] { return worker.toolpath().revision(); });
        const auto started = std::chrono::steady_clock::now();
        require(worker.execute(), "1001 preview should start execution");
        auto revision = initialRevision;
        for(int attempt = 0; attempt < 5000; ++attempt) {
            revision = worker.lock([&] { return worker.toolpath().revision(); });
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
        require(worker.lock([&] { return !worker.toolpath().commands().empty(); }),
                "1001 preview should retain displayable canonical geometry");
        worker.join();
    }

    void testMdiToolChangeUsesAutoloadPrograms() {
        const auto hello = ngc::readFile("autoload/hello.ngc");
        const auto world = ngc::readFile("autoload/world.ngc");
        const auto toolChange = ngc::readFile("autoload/tool_change.ngc");
        require(hello && world && toolChange, "MDI autoload programs should load");

        ngc::ToolTable tools;
        const auto loadedTools = tools.load();
        require(loadedTools.has_value(), loadedTools ? "" : loadedTools.error());

        SimulationWorker worker;
        worker.setTickMultiplier(1000);
        require(worker.start({ { *hello, "autoload/hello.ngc" }, { *world, "autoload/world.ngc" },
                               { *toolChange, "autoload/tool_change.ngc" }, { "T2 M6\n", "<MDI>" } }, tools),
                "MDI tool change should start");

        auto snapshot = worker.snapshot();
        for(int attempt = 0; attempt < 3000 && snapshot.toolPose.geometry.number != 2
            && snapshot.status != ngc::SimulationStatus::Error; ++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            snapshot = worker.snapshot();
        }
        const auto message = std::format("MDI T2 M6 did not select tool 2: status {} error '{}'",
                                         static_cast<int>(snapshot.status), snapshot.error);
        require(snapshot.toolPose.geometry.number == 2, message);
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

        tools.set(1, { .number = 1, .x = 0, .y = 0, .z = 0.5, .a = 0, .b = 0, .c = 0, .diameter = 0.25 });

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
        require(std::ranges::find(afterG43.activeModalGCodes, "G43") != afterG43.activeModalGCodes.end(),
                "simulation should preserve G43 modal G-code after running non-motion command");

        require(worker.start({ { "G91 G0 X1\n", "<MDI>" } }, tools, true),
                "second persistent simulation command should start");
        const auto afterSecond = waitForCompletion("second persistent command should complete");
        requireNear(afterSecond.machinePosition.x, 2.0,
                    "second command should continue from the prior simulated position");
        require(std::ranges::find(afterSecond.activeModalGCodes, "G43") != afterSecond.activeModalGCodes.end(),
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
        std::ifstream toolChangeFile("autoload/tool_change.ngc");
        const std::string toolChangeSource {
            std::istreambuf_iterator<char>(toolChangeFile),
            std::istreambuf_iterator<char>() };
        require(!toolChangeSource.empty(), "tool-change autoload program should be readable");

        ngc::InterpreterSession session(UNIT, ngc::InterpretationMode::RealRun);
        session.setPrograms({
            { toolChangeSource, "autoload/tool_change.ngc" },
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

    void testToolpathRecorderIsASeparateConsumer() {
        const auto commands = run("G0 X1\nG1 F2 X3\n");
        ngc::ToolpathRecorder recorder;

        for(const auto &command : commands) {
            recorder.consume(command);
        }

        require(recorder.commands().size() == 2, "toolpath recorder should retain commands supplied by its consumer");
        std::size_t observed = 0;
        recorder.foreachCommand([&](const ngc::MachineCommand &) { ++observed; });
        require(observed == 2, "toolpath recorder should enumerate its recorded commands");

        recorder.clear();
        require(recorder.commands().empty(), "toolpath recorder should clear between preview runs");
        require(recorder.workCoordinateSystems().empty(), "toolpath recorder should clear retained WCS frames");
    }

    void testToolpathRecorderRetainsUsedWorkCoordinateSystems() {
        ngc::ToolpathRecorder recorder;
        const ngc::MoveLine move { {}, { 1, 0, 0, 0, 0, 0 }, 1.0 };
        recorder.consume(move, {}, ngc::WorkCoordinateSystem { "G59.3", { 10, 20, 30, 0, 0, 0 } });
        recorder.consume(move, {}, ngc::WorkCoordinateSystem { "G54", { 1, 2, 3, 0, 0, 0 } });

        require(recorder.workCoordinateSystems().size() == 2,
                "preview should retain each WCS used by recorded motion");
        require(recorder.workCoordinateSystems()[0].name == "G59.3",
                "nested tool-change WCS should remain in preview metadata");
        require(recorder.workCoordinateSystems()[1].name == "G54",
                "final program WCS should remain in preview metadata");
    }

    void testToolChangePreviewRetainsNestedAndFinalWorkCoordinateSystems() {
        std::ifstream toolChangeFile("autoload/tool_change.ngc");
        const std::string toolChangeSource {
            std::istreambuf_iterator<char>(toolChangeFile),
            std::istreambuf_iterator<char>() };
        ngc::InterpreterSession session(UNIT, ngc::InterpretationMode::Preview);
        session.setPrograms({
            { toolChangeSource, "autoload/tool_change.ngc" },
            { "T2 M6\nG54\nG0 X1\n", "wcs-preview.ngc" },
        });
        session.compile([](const auto &callback) { callback(); });
        require(session.compiled(), "tool-change WCS preview should compile");
        session.begin();

        ngc::MockMotionBackend backend;
        ngc::TrajectoryExecutionDriver driver(session, backend);
        require(driver.begin(1), "preview trajectory backend should start");
        ngc::ToolpathRecorder recorder;
        while(driver.state() == ngc::TrajectoryDriverState::Running) {
            for(int fill = 0; fill < 64; ++fill) {
                if(!driver.pumpOne([](const auto &callback) { callback(); },
                    [&](const ngc::MachineCommand &command, const ngc::ExecutionItem &) {
                        recorder.consume(command, session.machine().toolOffset(), ngc::WorkCoordinateSystem {
                            std::string(ngc::name(*session.machine().state().modeCoordSys)), session.machine().workOffset() });
                    })) break;
            }
            backend.runUntilIdle();
            driver.serviceBackend();
        }

        require(driver.state() == ngc::TrajectoryDriverState::Completed,
                "tool-change WCS preview should complete through simulated probe barriers");
        const auto &systems = recorder.workCoordinateSystems();
        require(std::ranges::find_if(systems, [](const auto &value) { return value.name == "G59.3"; }) != systems.end(),
                "preview should retain G59.3 used inside the nested tool-change call");
        require(std::ranges::find_if(systems, [](const auto &value) { return value.name == "G54"; }) != systems.end(),
                "preview should retain G54 selected by the main program after tool change");
    }

    void testToolpathRecorderAppliesPerCommandToolOffset() {
        ngc::InterpreterSession session(UNIT, ngc::InterpretationMode::Preview);
        ngc::ToolpathRecorder recorder;
        compileSession(session,
            "G43 H7 G1 F1 Z3\n"
            "G49 G1 Z4\n");

        for(int i = 0; i < 2; i++) {
            const auto event = session.next();
            const auto command = std::get_if<ngc::MachineCommand>(&event);
            require(command != nullptr, "tool-offset preview should emit a motion command");
            recorder.consume(*command, session.machine().toolOffset());
        }

        requireCompleted(session, "tool-offset preview should complete");
        require(recorder.commands().size() == 2, "tool-offset preview should retain both motions");

        const auto *compensated = std::get_if<ngc::MoveLine>(&recorder.commands()[0]);
        const auto *uncompensated = std::get_if<ngc::MoveLine>(&recorder.commands()[1]);
        require(compensated != nullptr && uncompensated != nullptr, "tool-offset preview should retain line motions");
        requireNear(compensated->to().z, 3.0, "G43 preview endpoint should be the cutter-tip position");
        requireNear(uncompensated->to().z, 4.0, "G49 preview endpoint should not retain the previous tool offset");
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

    ngc::AxisPolynomialSpan linearSpan(const ngc::SpanId id, const double from, const double to,
                                       const double duration) {
        ngc::AxisPolynomialSpan span;
        span.id = id;
        span.duration = duration;
        span.inverseDuration = 1.0 / duration;
        span.inverseDurationSquared = span.inverseDuration * span.inverseDuration;
        span.inverseDurationCubed = span.inverseDurationSquared * span.inverseDuration;
        span.c.x = to - from;
        span.d.x = from;
        span.end.position.x = to;
        span.end.velocity.x = (to - from) / duration;
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
        stop.end = {};
        stop.end.position.x = 1.0;
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
        require(backend.trySubmit(ngc::StopJogRequest { 44, 999 }) == ngc::SubmitResult::Submitted,
                "stale-token stop should still traverse the bounded control channel");
        require(backend.trySubmit(ngc::StopJogRequest { 45, heldJog.jog }) == ngc::SubmitResult::Submitted,
                "matching stop should traverse the bounded control channel");
        backend.runUntilIdle(0.001);
        bool staleStopRejected = false;
        std::optional<ngc::JogStopped> requestedStop;
        while(backend.tryTakeEvent(event)) {
            if(const auto *completed = std::get_if<ngc::RequestCompleted>(&event))
                if(completed->request == 44) staleStopRejected = !completed->succeeded;
            if(const auto *jogEvent = std::get_if<ngc::JogStopped>(&event)) requestedStop = *jogEvent;
        }
        require(staleStopRejected, "a delayed token must not stop a newer jog");
        require(requestedStop && requestedStop->reason == ngc::JogStopReason::RequestedStop,
                "matching StopJog should produce a constrained requested stop");
    }

    void testMockBackendDrainsAFullPlanHorizonWithoutEventOverflow() {
        ngc::MockMotionBackend backend;
        require(backend.trySubmit(ngc::ResetRequest { 1, 5 }) == ngc::SubmitResult::Submitted,
                "full-horizon test reset should fit in the control channel");
        require(backend.trySubmit(ngc::StartRequest { 2, 5 }) == ngc::SubmitResult::Submitted,
                "full-horizon test start should fit in the control channel");
        backend.advance(0.0);
        ngc::ExecutionEvent startupEvent;
        while(backend.tryTakeEvent(startupEvent)) { }

        for(std::uint64_t index = 0; index < 8; ++index) {
            ngc::PlanChunk chunk;
            chunk.epoch = 5;
            chunk.id = index + 1;
            chunk.predecessorBranch = index;
            chunk.branch = index + 1;
            require(chunk.normalMotion.push(linearSpan(100 + index, static_cast<double>(index),
                                                       static_cast<double>(index + 1), 0.01)),
                    "full-horizon normal span should fit");
            auto stop = linearSpan(200 + index, static_cast<double>(index + 1),
                                   static_cast<double>(index + 1), 1e-6);
            stop.end.velocity = {};
            require(chunk.stopTail.push(stop), "full-horizon stop span should fit");
            chunk.stopState.position.x = static_cast<double>(index + 1);
            require(backend.tryPublish(chunk) == ngc::PublishResult::Published,
                    "all eight preallocated plan slots should publish");
        }

        backend.runUntilIdle();
        int retired = 0;
        bool held = false;
        ngc::ExecutionEvent event;
        while(backend.tryTakeEvent(event)) {
            if(std::holds_alternative<ngc::ChunkRetired>(event)) ++retired;
            if(std::holds_alternative<ngc::BackendHeld>(event)) held = true;
        }
        ngc::ExecutionSnapshot snapshot;
        ngc::ExecutionSnapshot latest;
        while(backend.tryTakeSnapshot(snapshot)) latest = snapshot;
        require(latest.state != ngc::BackendState::Faulted,
                "a full forward plan horizon must not overflow its return event channel");
        require(retired == 8 && held, "a full horizon should retire every chunk and finish held");
    }

    void testImmediateDrainStopsAtHeldWithStaleDescendants() {
        ngc::MockMotionBackend backend;
        require(backend.trySubmit(ngc::ResetRequest { 1, 6 }) == ngc::SubmitResult::Submitted, "reset should publish");
        require(backend.trySubmit(ngc::StartRequest { 2, 6 }) == ngc::SubmitResult::Submitted, "start should publish");
        backend.advance(0.0);
        ngc::ExecutionEvent event;
        while(backend.tryTakeEvent(event)) { }

        ngc::PlanChunk first;
        first.epoch = 6; first.id = 1; first.branch = 1;
        first.normalMotion.push(linearSpan(1, 0.0, 1.0, 0.01));
        first.stopTail.push(linearSpan(2, 1.0, 1.0, 1e-6));
        ngc::PlanChunk stale = first;
        stale.id = 2; stale.branch = 2; stale.predecessorBranch = 999;
        auto staleBehindIt = stale;
        staleBehindIt.id = 3; staleBehindIt.branch = 3;
        require(backend.tryPublish(first) == ngc::PublishResult::Published, "first chunk should publish");
        require(backend.tryPublish(stale) == ngc::PublishResult::Published, "stale descendant should publish");
        require(backend.tryPublish(staleBehindIt) == ngc::PublishResult::Published,
                "a second stale descendant should remain queued behind the rejected one");

        const auto started = std::chrono::steady_clock::now();
        backend.runUntilIdle();
        const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
        require(elapsed < 0.1, "immediate preview must not spin while held with stale queued descendants");
        bool held = false;
        while(backend.tryTakeEvent(event)) if(std::holds_alternative<ngc::BackendHeld>(event)) held = true;
        require(held, "mismatched continuation should select the stop tail and report held");
    }

    struct PreviewPipelineMeasurement {
        int loops = 0;
        int commands = 0;
        double seconds = 0.0;
        ngc::TrajectoryDriverState state = ngc::TrajectoryDriverState::Running;
        std::optional<std::string> error;
        std::size_t outstanding = 0;
        bool interpreted = false;
        bool probePending = false;
        bool waitingHeld = false;
        ngc::EpochId epoch = 0;
        ngc::BackendState backendState = ngc::BackendState::Disabled;
    };

    PreviewPipelineMeasurement measurePreviewPrefix(const int lastLine) {
        std::ifstream input("1002_3d.ngc");
        std::string source;
        std::string line;
        for(int number = 1; number <= lastLine && std::getline(input, line); ++number) {
            if(const auto mode = line.find("G64"); mode != std::string::npos)
                line.replace(mode, std::string("G64 P0.001").size(), "G61");
            source += line + '\n';
        }
        source += "%\n";
        const auto hello = ngc::readFile("autoload/hello.ngc");
        const auto world = ngc::readFile("autoload/world.ngc");
        const auto toolChange = ngc::readFile("autoload/tool_change.ngc");
        require(hello && world && toolChange, "preview measurement should load autoload programs");

        ngc::InterpreterSession session(UNIT, ngc::InterpretationMode::Preview);
        session.setPrograms({ { *hello, "autoload/hello.ngc" }, { *world, "autoload/world.ngc" },
                              { *toolChange, "autoload/tool_change.ngc" }, { source, "preview-prefix.ngc" } });
        session.compile([](const auto &callback) { callback(); });
        require(session.compiled(), "preview-prefix measurement should compile");
        session.begin();
        ngc::MockMotionBackend backend;
        ngc::TrajectoryExecutionDriver driver(session, backend);
        ngc::ToolpathRecorder recorder;
        require(driver.begin(1), "preview-prefix backend should start");

        PreviewPipelineMeasurement result;
        const auto start = std::chrono::steady_clock::now();
        for(; result.loops < 1000 && driver.state() == ngc::TrajectoryDriverState::Running; ++result.loops) {
            for(int fill = 0; fill < 64; ++fill) {
                if(!driver.pumpOne([](const auto &callback) { callback(); },
                    [&](const ngc::MachineCommand &command, const ngc::ExecutionItem &) {
                        ++result.commands;
                        recorder.consume(command, session.machine().toolOffset());
                    })) break;
            }
            backend.runUntilIdle();
            driver.serviceBackend();
        }
        result.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        result.state = driver.state();
        result.error = driver.error();
        result.outstanding = driver.outstandingChunks();
        result.interpreted = driver.interpretationComplete();
        result.probePending = driver.probePending();
        result.waitingHeld = driver.waitingForHeld();
        result.epoch = driver.epoch();
        ngc::ExecutionSnapshot backendSnapshot;
        while(backend.tryTakeSnapshot(backendSnapshot)) result.backendState = backendSnapshot.state;
        session.stop();
        return result;
    }

    void testN70N75PreviewPrefixesCompleteBoundedly() {
        const auto n70 = measurePreviewPrefix(24);
        const auto n75 = measurePreviewPrefix(25);
        const auto diagnostic = std::format(
            "N70 loops={} commands={} time={:.6f}; N75 loops={} commands={} time={:.6f}; outstanding={} interpreted={} probe={} waitingHeld={} epoch={} backend={} error='{}'",
            n70.loops, n70.commands, n70.seconds, n75.loops, n75.commands, n75.seconds,
            n75.outstanding, n75.interpreted, n75.probePending, n75.waitingHeld, n75.epoch,
            static_cast<int>(n75.backendState), n75.error.value_or(""));
        require(n70.state == ngc::TrajectoryDriverState::Completed, diagnostic);
        require(n75.state == ngc::TrajectoryDriverState::Completed, diagnostic);
        require(n75.loops < 100 && n75.seconds < 1.0, diagnostic);
    }

    ngc::position_t evaluateSpan(const ngc::AxisPolynomialSpan &span, const double u) {
        const auto evaluate = [&](const double ngc::position_t::*member) {
            return ((span.a.*member*u + span.b.*member)*u + span.c.*member)*u + span.d.*member;
        };
        return { evaluate(&ngc::position_t::x), evaluate(&ngc::position_t::y), evaluate(&ngc::position_t::z),
                 evaluate(&ngc::position_t::a), evaluate(&ngc::position_t::b), evaluate(&ngc::position_t::c) };
    }

    double spanAcceleration(const ngc::AxisPolynomialSpan &span, const double u) {
        const auto component = [&](const double ngc::position_t::*member) {
            return (6.0*span.a.*member*u + 2.0*span.b.*member) * span.inverseDurationSquared;
        };
        const auto x = component(&ngc::position_t::x);
        const auto y = component(&ngc::position_t::y);
        const auto z = component(&ngc::position_t::z);
        const auto a = component(&ngc::position_t::a);
        const auto b = component(&ngc::position_t::b);
        const auto c = component(&ngc::position_t::c);
        return std::sqrt(x*x + y*y + z*z + a*a + b*b + c*c);
    }

    double spanJerk(const ngc::AxisPolynomialSpan &span) {
        const auto component = [&](const double ngc::position_t::*member) {
            return 6.0*span.a.*member * span.inverseDurationCubed;
        };
        const auto x = component(&ngc::position_t::x);
        const auto y = component(&ngc::position_t::y);
        const auto z = component(&ngc::position_t::z);
        const auto a = component(&ngc::position_t::a);
        const auto b = component(&ngc::position_t::b);
        const auto c = component(&ngc::position_t::c);
        return std::sqrt(x*x + y*y + z*z + a*a + b*b + c*c);
    }

    double spanAxisVelocity(const ngc::AxisPolynomialSpan &span,
                            const double ngc::position_t::*component) {
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

    double spanAxisAcceleration(const ngc::AxisPolynomialSpan &span,
                                const double ngc::position_t::*component) {
        const auto at = [&](const double u) {
            return std::abs((6.0*span.a.*component*u + 2.0*span.b.*component)
                * span.inverseDurationSquared);
        };
        return std::max(at(0.0), at(1.0));
    }

    double spanAxisJerk(const ngc::AxisPolynomialSpan &span,
                        const double ngc::position_t::*component) {
        return std::abs(6.0*span.a.*component * span.inverseDurationCubed);
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
        ngc::ExactStopTrajectoryPlanner planner({
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
        requireNear(line->normalMotion[0].c.x, 0.0, "exact-stop line should start at zero velocity");
        requireNear(line->normalMotion[line->normalMotion.size-1].end.velocity.x, 0.0,
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
                (6.0*previous.a.x + 2.0*previous.b.x) * previous.inverseDurationSquared;
            const auto currentStartAcceleration =
                2.0*current.b.x * current.inverseDurationSquared;
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
        requireNear(arc->normalMotion[0].c.x, 0.0, "exact-stop arc should begin at zero velocity");
        requireNear(arc->normalMotion[arc->normalMotion.size-1].end.velocity.x, 0.0,
                    "exact-stop arc should end with zero X velocity");
        requireNear(arc->normalMotion[arc->normalMotion.size-1].end.velocity.y, 0.0,
                    "exact-stop arc should end with zero Y velocity");
        for(const auto &span : arc->normalMotion) {
            require(spanAcceleration(span, 0.0) <= ACCELERATION + 1e-8,
                    "compiled arc should respect acceleration at its span start");
            require(spanAcceleration(span, 1.0) <= ACCELERATION + 1e-8,
                    "compiled arc should respect acceleration at its span end");
            require(spanJerk(span) <= JERK + 1e-8,
                    "compiled arc should respect the configured jerk limit");
        }

        ngc::ExactStopTrajectoryPlanner instantaneous({
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
        requireNear(constantSpeed->normalMotion[0].a.x, 0.0,
                    "constant-speed line should have no cubic position term");
        requireNear(constantSpeed->normalMotion[0].b.x, 0.0,
                    "constant-speed line should have no quadratic position term");
        requireNear(constantSpeed->normalMotion[0].duration, 2.0,
                    "constant-speed line duration should be distance divided by feed");
    }

    void testExactStopPlannerEnforcesIndependentAxisLimits() {
        constexpr auto infinity = std::numeric_limits<double>::infinity();
        ngc::ExactStopTrajectoryPlanner planner({
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

        ngc::ExactStopTrajectoryPlanner arcPlanner({
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

    void testBoundedLookaheadCarriesG64MetadataAndSingleCommandExactStops() {
        ngc::BoundedLookaheadTrajectoryPlanner planner({
            .pathAcceleration = 4.0,
            .rapidSpeed = 120.0,
            .arcChordTolerance = 0.0001,
            .pathJerk = 8.0,
        });
        planner.reset(21);
        require(planner.enqueue({
            ngc::MoveLine { {}, { 1, 0, 0, 0, 0, 0 }, 60.0 },
            { ngc::ExecutablePathMode::Continuous, 0.01 },
            {},
        }), "bounded lookahead should accept a G64 motion");
        require(planner.enqueue({
            ngc::MoveLine { { 1, 0, 0, 0, 0, 0 }, { 2, 0, 0, 0, 0, 0 }, 60.0 },
            { ngc::ExecutablePathMode::ExactStop, std::nullopt },
            {},
        }), "bounded lookahead should accept a following exact-stop motion");
        require(planner.windowSize() == 2,
                "lookahead should retain canonical inputs in source order");

        const auto first = planner.planOne();
        require(first.has_value() && *first, first ? "lookahead should produce its first item" : first.error());
        require((*first)->metadata.pathMode == ngc::ExecutablePathMode::Continuous
                    && (*first)->metadata.pathTolerance == 0.01,
                "planned execution should retain its G64 mode and tolerance");
        const auto *firstChunk = std::get_if<ngc::PlanChunk>(&(*first)->primaryItem());
        require(firstChunk && firstChunk->stopTail.size == 1,
                "an individually planned G64 command should use a verified exact stop");

        const auto second = planner.planOne();
        require(second.has_value() && *second, second ? "lookahead should produce its second item" : second.error());
        require(planner.windowSize() == 0, "lookahead should consume planned inputs in order");
        const auto &diagnostics = planner.diagnostics();
        require(diagnostics.commandsPlanned == 2 && diagnostics.continuousModeInputs == 1
                    && diagnostics.continuousExactStops == 1,
                "lookahead diagnostics should distinguish individually planned G64 exact stops");
        require(diagnostics.maximumWindowCommands == 2 && diagnostics.maximumStopSpans == 1
                    && diagnostics.maximumNormalSpans > 0 && diagnostics.plannedDuration > 0.0,
                "lookahead diagnostics should retain window, span, and duration high-water marks");

        ngc::BoundedLookaheadTrajectoryPlanner commandCountIndependent;
        commandCountIndependent.reset(22);
        for(std::size_t index=0;index<64;++index)
            require(commandCountIndependent.enqueue({ngc::SpindleStop{},{},{}}),
                    "the NRT horizon should not impose the removed 32-command packet boundary");
        require(commandCountIndependent.windowSize()==64,
                "RT PlanChunk capacity should be independent of buffered G-code command count");

        ngc::BoundedLookaheadTrajectoryPlanner protectedPlanner;
        protectedPlanner.reset(23);
        ngc::TrajectoryPlannerInput g54 {
            ngc::MoveLine{{},{1,0,0,0,0,0},60.0},
            {ngc::ExecutablePathMode::Continuous,0.1},
            {},
        };
        g54.presentation.workCoordinateSystem=ngc::WorkCoordinateSystem{"G54",{}};
        auto g55=g54;
        g55.command=ngc::MoveLine{{1,0,0,0,0,0},{1,1,0,0,0,0},60.0};
        g55.presentation.workCoordinateSystem=ngc::WorkCoordinateSystem{"G55",{}};
        require(protectedPlanner.enqueue(g54),"protected-boundary test should enqueue its first motion");
        require(!protectedPlanner.canAppend(g55),
                "work-coordinate changes should remain protected G64 boundaries");
        ngc::TrajectoryPlannerInput rapid {
            ngc::MoveLine{{1,0,0,0,0,0},{2,0,0,0,0,0},-1.0},
            {ngc::ExecutablePathMode::Continuous,0.1}, {},
        };
        require(!ngc::BoundedLookaheadTrajectoryPlanner::eligibleForLookahead(rapid),
                "rapid moves should remain protected from G64 preview blending");
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
        require(backend.trySubmit(ngc::StartRequest { 1, 31 }) == ngc::SubmitResult::Submitted,
                "fixed-tick test backend should start");

        backend.advanceTick(0.001, true);
        ngc::ExecutionSnapshot snapshot;
        while(backend.tryTakeSnapshot(snapshot)) { }
        requireNear(snapshot.commanded.position.x, 0.1,
                    "one 1 ms servo tick should advance one tenth of a 10 ms linear span");

        for(int tick = 0; tick < 9; ++tick) backend.advanceTick(0.001, tick == 8);
        while(backend.tryTakeSnapshot(snapshot)) { }
        requireNear(snapshot.commanded.position.x, 1.0,
                    "ten fixed servo ticks should complete the 10 ms span exactly");

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
        cubic.a.x=1.0;
        cubic.end.position.x=1.0;
        cubic.end.velocity.x=3.0;
        cubic.end.acceleration.x=6.0;
        require(jerkChunk.normalMotion.push(cubic),
                "jerk telemetry cubic should fit in its chunk");
        require(jerkChunk.stopTail.push(linearSpan(4,1.0,1.0,1e-6)),
                "jerk telemetry stop tail should fit in its chunk");
        jerkChunk.branchState=cubic.end;
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

    void testPreviewJunctionUsesOneSixControlSpline() {
        const ngc::experimental::JunctionEntity incoming {
            .length = 10.0,
            .stateAtDistance = [](const double distance) {
                return ngc::experimental::JunctionState {
                    .position={distance,0.0,0.0},.tangent={1.0,0.0,0.0},.curvature={} };
            },
        };
        const ngc::experimental::JunctionEntity outgoing {
            .length = 10.0,
            .stateAtDistance = [](const double distance) {
                return ngc::experimental::JunctionState {
                    .position={10.0,distance,0.0},.tangent={0.0,1.0,0.0},.curvature={} };
            },
        };
        constexpr double scale=1.0;
        const auto blend=ngc::experimental::fitJunction(incoming,outgoing,scale);
        require(blend.has_value(),"preview corner should fit a single clamped spline");
        require(blend->controlPoints.size()==6,
                "a preview blend should have two clamped endpoint and four interior controls");
        requireNear(blend->incomingTrim,3.0,
                    "a long incoming entity should be trimmed by three times P");
        requireNear(blend->outgoingTrim,3.0,
                    "a long outgoing entity should be trimmed by three times P");
        const auto &controls=blend->controlPoints;
        const std::array expectedControls {
            glm::dvec3(7,0,0),glm::dvec3(8,0,0),glm::dvec3(9,0,0),
            glm::dvec3(10,1,0),glm::dvec3(10,2,0),glm::dvec3(10,3,0),
        };
        for(std::size_t control=0;control<expectedControls.size();++control)
            require(glm::length(controls[control]-expectedControls[control])<1e-9,
                    "line blend controls should be sampled at 3P, 2P, P / P, 2P, 3P");
        require(std::ranges::none_of(controls,[](const auto &control) {
            return glm::length(control-glm::dvec3(10,0,0))<1e-9;
        }),"the junction should not be a spline control point");
        require(glm::length(controls[1]-controls[0]-glm::dvec3(scale,0,0))<1e-9,
                "incoming line controls should be spaced by its local P");
        require(glm::length(controls[2]-controls[1]-glm::dvec3(scale,0,0))<1e-9,
                "the first three incoming controls should remain equally spaced");
        require(glm::length(controls[5]-controls[4]-glm::dvec3(0,scale,0))<1e-9,
                "outgoing line controls should be spaced by its local P");
        require(glm::length(controls[4]-controls[3]-glm::dvec3(0,scale,0))<1e-9,
                "the last three outgoing controls should remain equally spaced");

        const ngc::experimental::JunctionEntity curvedIncoming {
            .length=10.0,
            .stateAtDistance=[](const double distance) {
                constexpr double radius=5.0;
                const auto angle=(distance-10.0)/radius;
                return ngc::experimental::JunctionState {
                    .position={radius*std::sin(angle),radius*(1.0-std::cos(angle)),0.0},
                    .tangent={std::cos(angle),std::sin(angle),0.0},
                    .curvature={-std::sin(angle)/radius,std::cos(angle)/radius,0.0} };
            },
        };
        const ngc::experimental::JunctionEntity curvedOutgoing {
            .length=10.0,
            .stateAtDistance=[](const double distance) {
                constexpr double radius=5.0;
                const auto angle=distance/radius;
                return ngc::experimental::JunctionState {
                    .position={radius*std::sin(angle),radius*(1.0-std::cos(angle)),0.0},
                    .tangent={std::cos(angle),std::sin(angle),0.0},
                    .curvature={-std::sin(angle)/radius,std::cos(angle)/radius,0.0} };
            },
        };
        const auto curvedBlend=ngc::experimental::fitJunction(curvedIncoming,curvedOutgoing,scale);
        require(curvedBlend.has_value(),"curved neighboring entities should produce a blend");
        const auto geometricCurvature=[](const glm::dvec3 velocity,const glm::dvec3 acceleration) {
            const auto speed=glm::length(velocity);
            const auto tangent=velocity/speed;
            return (acceleration-tangent*glm::dot(acceleration,tangent))/(speed*speed);
        };
        const auto &curvedControls=curvedBlend->controlPoints;
        const auto incomingVelocity=3.0*(curvedControls[1]-curvedControls[0]);
        const auto incomingAcceleration=3.0*curvedControls[2]-9.0*curvedControls[1]
            +6.0*curvedControls[0];
        const auto outgoingVelocity=3.0*(curvedControls[5]-curvedControls[4]);
        const auto outgoingAcceleration=6.0*curvedControls[5]-9.0*curvedControls[4]
            +3.0*curvedControls[3];
        require(glm::length(geometricCurvature(incomingVelocity,incomingAcceleration)
                            -curvedIncoming.stateAtDistance(7.0).curvature)<1e-9,
                "incoming cubic B-spline endpoint should match arc curvature");
        require(glm::length(geometricCurvature(outgoingVelocity,outgoingAcceleration)
                            -curvedOutgoing.stateAtDistance(3.0).curvature)<1e-9,
                "outgoing cubic B-spline endpoint should match arc curvature");
        require(glm::length(curvedControls[1]-curvedControls[0])<scale
                    &&glm::length(curvedControls[5]-curvedControls[4])<scale,
                "arc geometry should shorten tangent handles when that better follows the arc");
        const auto incomingControlDeparture=
            glm::length(curvedControls[2]-curvedIncoming.stateAtDistance(9.0).position);
        const auto outgoingControlDeparture=
            glm::length(curvedControls[3]-curvedOutgoing.stateAtDistance(1.0).position);
        require(std::max(incomingControlDeparture,outgoingControlDeparture)<0.25*scale,
                "optimized curved controls should remain close to the neighboring arc geometry");
        require(std::max(incomingControlDeparture,outgoingControlDeparture)>1e-6,
                "curved handle optimization should improve on the initial nearby-point fit");

        const auto lineEntity=[](const glm::dvec3 from,const glm::dvec3 tangent,const double length) {
            return ngc::experimental::JunctionEntity {
                .length=length,
                .stateAtDistance=[=](const double distance) {
                    return ngc::experimental::JunctionState {
                        .position=from+tangent*distance,.tangent=tangent,.curvature={} };
                },
            };
        };
        const auto longBefore=lineEntity({0,0,0},{1,0,0},12.0);
        const auto shortEntity=lineEntity({12,0,0},{0,1,0},3.0);
        const auto longAfter=lineEntity({12,3,0},{1,0,0},12.0);
        const auto intoShort=ngc::experimental::fitJunction(longBefore,shortEntity,scale);
        const auto outOfShort=ngc::experimental::fitJunction(shortEntity,longAfter,scale);
        require(intoShort&&outOfShort,"both sides of a short entity should produce local blends");
        requireNear(intoShort->outgoingScale,0.5,"a short entity should use length divided by six");
        requireNear(outOfShort->incomingScale,0.5,"both sides should use the same short-entity scale");
        const auto &leftMidpoint=intoShort->controlPoints;
        const auto &rightMidpoint=outOfShort->controlPoints;
        require(glm::length(leftMidpoint[5]-rightMidpoint[0])<1e-9,
                "neighboring blends should meet at the short entity midpoint");
        require(glm::length((leftMidpoint[5]-leftMidpoint[4])
                            -(rightMidpoint[1]-rightMidpoint[0]))<1e-9,
                "neighboring short-entity blends should share their midpoint tangent scale");
        require(glm::length((leftMidpoint[5]-2.0*leftMidpoint[4]+leftMidpoint[3])
                            -(rightMidpoint[2]-2.0*rightMidpoint[1]+rightMidpoint[0]))<1e-8,
                "neighboring short-entity blends should be C2 at the midpoint");

        const auto differentShort=lineEntity({12,3,0},{1,0,0},6.0);
        const auto unequal=ngc::experimental::fitJunction(shortEntity,differentShort,scale);
        require(unequal.has_value(),"unequal short sides should produce a blend");
        requireNear(unequal->incomingScale,0.5,"incoming side should retain its own local scale");
        requireNear(unequal->outgoingScale,1.0,"outgoing side should retain its own local scale");
    }

    void testBoundedPlannerExecutesPiecewiseG64Geometry() {
        ngc::BoundedLookaheadTrajectoryPlanner planner({
            .pathAcceleration=5.0, .rapidSpeed=120.0, .arcChordTolerance=0.001, .pathJerk=20.0,
            .axisVelocity={10,10,10,10,10,10},
            .axisAcceleration={10,10,10,10,10,10},
            .axisJerk={50,50,50,50,50,50},
            .lookaheadDuration=1000.0,
        });
        const ngc::position_t p0{0,0,0,0,0,0}, p1{5,0,0,0,0,0}, p2{5,5,0,0,0,0};
        planner.reset(70,p0);
        const ngc::TrajectoryPlanningMetadata g64 {
            .pathMode=ngc::ExecutablePathMode::Continuous, .pathTolerance=0.1,
        };
        require(planner.enqueue({ngc::MoveLine{p0,p1,120.0},g64,{}}), "first G64 line should enter lookahead");
        require(planner.enqueue({ngc::MoveLine{p1,p2,120.0},g64,{}}), "second G64 line should enter lookahead");
        const auto planned=planner.planWindow();
        require(planned.has_value() && *planned, planned ? "G64 window produced no plan" : planned.error());
        require((*planned)->inputs.size()==2, "G64 should plan both compatible commands in one window");
        const auto *chunk=std::get_if<ngc::PlanChunk>(&(*planned)->primaryItem());
        require(chunk&&chunk->normalMotion.size>2&&chunk->stopTail.size==1,
                "G64 should emit bounded normal motion and a verified terminal stop branch");
        require(planner.diagnostics().continuousExactStops==0
                    &&planner.diagnostics().blendedWindows==1
                    &&planner.diagnostics().blendedCommands==2,
                "compatible G64 commands should use executable blending");
        require((*planned)->activationSpans.size()==2
                    &&(*planned)->activationSpans[0]!=0&&(*planned)->activationSpans[1]!=0
                    &&(*planned)->activationSpans[0]<(*planned)->activationSpans[1],
                "blended commands should retain ordered activation spans");
        require(std::ranges::none_of(chunk->normalMotion,[&](const auto &span) {
            return (span.end.position-p1).length()<1e-8;
        }),"local G64 geometry should round the junction instead of stopping on it");
        for(std::size_t span=1;span<chunk->normalMotion.size;++span) {
            const auto &previous=chunk->normalMotion[span-1];
            const auto &current=chunk->normalMotion[span];
            const auto scaled=[](const ngc::position_t &value,const double amount) {
                return ngc::position_t{value.x*amount,value.y*amount,value.z*amount,
                    value.a*amount,value.b*amount,value.c*amount};
            };
            const auto currentVelocity=scaled(current.c,current.inverseDuration);
            const auto currentAcceleration=scaled(current.b,2.0*current.inverseDurationSquared);
            require((previous.end.position-current.d).length()<1e-8,
                    "G64 polynomial spans should be position-continuous");
            require((previous.end.velocity-currentVelocity).length()<1e-7,
                    "G64 polynomial spans should be velocity-continuous");
            require((previous.end.acceleration-currentAcceleration).length()<1e-7,
                std::format("G64 polynomial spans should be acceleration-continuous (jump {})",
                    (previous.end.acceleration-currentAcceleration).length()));
        }
        requireAxisLimits(*chunk,&ngc::position_t::x,10,10,50,"G64 X motion");
        requireAxisLimits(*chunk,&ngc::position_t::y,10,10,50,"G64 Y motion");
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

    void testCubicSplineInteriorControlConditioning() {
        using Control=ngc::spline_detail::Vector3;
        const std::array<Control,9> controls {{
            {0.0,0.0,0.0}, {1.0,0.0,0.0}, {2.0,0.1,0.0},
            {3.0,0.8,0.0}, {4.0,-0.7,0.0}, {5.0,0.9,0.0},
            {6.0,0.1,0.0}, {7.0,0.0,0.0}, {8.0,0.0,0.0},
        }};
        const auto thirdDifferenceEnergy=[](const auto &values) {
            auto energy=0.0;
            for(std::size_t first=0;first+3<values.size();++first)
                for(std::size_t axis=0;axis<3;++axis) {
                    const auto difference=-values[first][axis]+3.0*values[first+1][axis]
                        -3.0*values[first+2][axis]+values[first+3][axis];
                    energy+=difference*difference;
                }
            return energy;
        };
        const auto conditioned=
            ngc::spline_detail::conditionCubicSplineInteriorControls<3>(controls,1.0);
        require(conditioned.size()==controls.size(),
            "cubic control conditioning should preserve the control count");
        for(const auto index:{0U,1U,2U,6U,7U,8U})
            require(conditioned[index]==controls[index],
                "cubic control conditioning should preserve the constrained endpoint controls");
        for(std::size_t control=3;control<=5;++control)
            for(std::size_t axis=0;axis<3;++axis)
                require(std::abs(conditioned[control][axis]-controls[control][axis])
                            <=0.2+1e-12,
                    "cubic control conditioning should bound every interior-control displacement");
        require(thirdDifferenceEnergy(conditioned)<thirdDifferenceEnergy(controls),
            "cubic control conditioning should reduce control-polygon third-difference energy");
    }

    void testGeometricJerkCombIncludesTangentialComponent() {
        // One cubic Bezier span for q(t)=(t,t^2). At t=0 its curvature is 2,
        // normal sharpness is zero, and q''' is the tangential vector (-4,0).
        const ngc::experimental::JunctionBlend parabola{
            .controlPoints={
                {0.0,0.0,0.0},{1.0/3.0,0.0,0.0},
                {2.0/3.0,1.0/3.0,0.0},{1.0,1.0,0.0},
            },
            .degree=3,
        };
        const std::array feedSections{
            ngc::experimental::ClusterFeedSection{.length=1.0,.speed=6.0},
        };
        const auto comb=ngc::experimental::sampleGeometricJerkComb(
            parabola,feedSections,100.0);
        require(comb.size()==65,
                "one planner piece should show exactly its 65 jerk sample points");
        requireNear(comb.front().magnitude,4.0,
                    "geometric jerk comb should measure the complete q''' magnitude");
        requireNear(comb.front().normalMagnitude,0.0,
                    "symmetric parabola endpoint should have zero normal sharpness");
        requireNear(comb.front().tangentialMagnitude,4.0,
                    "geometric jerk comb must retain the tangential kappa-squared component");
        requireNear(comb.front().programmedSpeed,6.0,
                    "geometric jerk comb should retain the owning programmed feed");
        require(comb.front().geometricSpeedLimit<comb.front().programmedSpeed,
                "a sample whose geometric jerk cap is below feed should be marked limiting");
        requireNear(comb.front().normalDirection.x,0.0,
                    "comb display tooth should follow the curvature normal");
        requireNear(comb.front().normalDirection.y,1.0,
                    "comb display tooth should follow the curvature normal");
    }

    void testMicroEntityClusterCollapsesToOneSpline() {
        constexpr double SCALE=0.05;
        const std::array lengths{1.0,0.02,0.01,1.0,0.06,1.0};
        const auto clusters=ngc::spline_detail::detectMicroEntityClusters(lengths,SCALE);
        require(clusters.size()==1&&clusters[0].left==0
                    &&clusters[0].firstCollapsed==1&&clusters[0].lastCollapsed==2
                    &&clusters[0].right==3,
                "only a micro-entity chain with total length at most P should collapse");

        const auto clusterEntity=[](const double length,
                const ngc::spline_detail::Vector3 &start,
                const ngc::spline_detail::Vector3 &middle,
                const ngc::spline_detail::Vector3 &end,
                const ngc::spline_detail::Vector3 &tangent) {
            return ngc::spline_detail::ClusterEntity3{
                .length=length,.positions={start,middle,end},
                .tangents={tangent,tangent,tangent},
            };
        };
        const auto normalized=[](const double x,const double y) {
            const auto magnitude=std::hypot(x,y);
            return ngc::spline_detail::Vector3{x/magnitude,y/magnitude,0.0};
        };
        const std::array inflection{
            clusterEntity(1.0,{0,0,0},{0.5,-0.025,0},{1,-0.05,0},normalized(1,-0.05)),
            clusterEntity(0.08,{1,-0.05,0},{1.04,-0.049,0},{1.08,-0.05,0},
                normalized(1,0)),
            clusterEntity(1.0,{1.08,-0.05,0},{1.58,-0.075,0},{2.08,-0.1,0},
                normalized(1,-0.05)),
        };
        const auto inflectionClusters=ngc::spline_detail::detectMicroEntityClusters(
            inflection,SCALE);
        require(inflectionClusters.size()==1&&inflectionClusters[0].left==0
                    &&inflectionClusters[0].right==2,
                "a shallow, deviation-bounded inflection should collapse");

        const std::array clusterLengths{1.0,0.04,0.06,0.05,1.0};
        const auto midpointClusters=ngc::spline_detail::detectShortEntitySplineClusters(
            clusterLengths,SCALE);
        require(midpointClusters.size()==1&&midpointClusters[0].left==0
                    &&midpointClusters[0].firstInterior==1
                    &&midpointClusters[0].lastInterior==3
                    &&midpointClusters[0].right==4,
                "consecutive entities at most 6P should form one midpoint-control cluster");
        std::array<double,20> denseLengths{};
        denseLengths.fill(0.005);
        const auto denseControlDistances=
            ngc::spline_detail::evenlySpacedCompositeControlDistances(denseLengths,SCALE);
        require(denseControlDistances.size()==denseLengths.size()
                    &&std::abs(denseControlDistances.front()-0.0025)<1e-12
                    &&std::abs(denseControlDistances.back()-0.0975)<1e-12,
                "dense tiny entities should use one midpoint control per entity");

        const auto previewLine=[](const glm::dvec3 start,const glm::dvec3 tangent) {
            return ngc::experimental::JunctionEntity {
                .length=1.0,
                .stateAtDistance=[=](const double distance) {
                    return ngc::experimental::JunctionState {
                        .position=start+tangent*distance,.tangent=tangent,.curvature={} };
                },
                .linear=true,
            };
        };
        const auto previewIncoming=previewLine({0,0,0},{1,0,0});
        const auto previewOutgoing=previewLine({1.02,0.01,0},{1,0.1,0});
        require(!ngc::experimental::fitJunction(previewIncoming,previewOutgoing,SCALE),
                "an ordinary preview junction should require a shared canonical endpoint");
        require(ngc::experimental::fitJunction(
                    previewIncoming,previewOutgoing,SCALE,false).has_value(),
                "a detected micro-cluster should bridge its separated long-entity endpoints");

        const auto clusterLine=[](const double from,const double length) {
            return ngc::experimental::JunctionEntity{
                .length=length,
                .stateAtDistance=[=](const double distance) {
                    return ngc::experimental::JunctionState{
                        .position={from+distance,0,0},.tangent={1,0,0},.curvature={}};
                },
                .linear=true,
            };
        };
        const std::array clusterEntities{
            clusterLine(0.0,1.0),clusterLine(1.0,0.2),
            clusterLine(1.2,0.2),clusterLine(1.4,1.0),
        };
        const auto directCluster=ngc::experimental::buildEvenlySpacedControlCluster(
            clusterEntities,0,3,SCALE);
        require(directCluster&&directCluster->degree==5
                    &&directCluster->controlPoints.size()==10
                    &&std::abs(directCluster->controlPoints.front().x-0.85)<1e-12
                    &&std::abs(directCluster->controlPoints.back().x-1.55)<1e-12,
                "preview clusters should reconstruct the selected quintic reference");
        const std::array shortClusterEntities{
            clusterLine(0.0,1.0),clusterLine(1.0,0.04),clusterLine(1.04,1.0),
        };
        const auto omittedShortCluster=ngc::experimental::buildEvenlySpacedControlCluster(
            shortClusterEntities,0,2,SCALE);
        require(omittedShortCluster&&omittedShortCluster->controlPoints.size()==6,
                "a short run totaling less than 6P should add no interior controls");

        const ngc::position_t p0{0,0,0,0,0,0},p1{1,0,0,0,0,0};
        const ngc::position_t p2{1.04,0.004,0,0,0,0},p3{2,0.1,0,0,0,0};
        const std::array<ngc::MachineCommand,3> commands{
            ngc::MoveLine{p0,p1,60.0},ngc::MoveLine{p1,p2,60.0},
            ngc::MoveLine{p2,p3,60.0},
        };
        ngc::ExactStopTrajectoryPlanner planner({
            .pathAcceleration=20.0,.rapidSpeed=120.0,.arcChordTolerance=0.0001,
            .pathJerk=200.0,
            .axisVelocity={20,20,20,20,20,20},
            .axisAcceleration={40,40,40,40,40,40},
            .axisJerk={400,400,400,400,400,400},
        });
        planner.reset(38,p0);
        const auto planned=planner.compileContinuous(commands,SCALE);
        require(planned&&*planned,planned?
                "micro-cluster fixture produced no plan":planned.error());
        require((*planned)->pieceTiming.size()==5,
                "a short run totaling less than 6P should time each of its three spline "
                "knot spans independently between the retained bounding pieces");
        require((*planned)->activationSpans.size()==commands.size()
                    &&(*planned)->activationSpans[0]<(*planned)->activationSpans[1]
                    &&(*planned)->activationSpans[1]==(*planned)->activationSpans[2],
                "collapsed command activations should remain ordered on the replacement spline");

        std::vector<ngc::MachineCommand> longStrip;
        longStrip.emplace_back(ngc::MoveLine{{},{1,0,0,0,0,0},60.0});
        auto stripPosition=1.0;
        for(unsigned segment=0;segment<8;++segment) {
            const auto next=stripPosition+0.04;
            longStrip.emplace_back(ngc::MoveLine{
                {stripPosition,0,0,0,0,0},{next,0,0,0,0,0},60.0});
            stripPosition=next;
        }
        longStrip.emplace_back(ngc::MoveLine{
            {stripPosition,0,0,0,0,0},{stripPosition+1.0,0,0,0,0,0},60.0});
        auto effort=planner.continuousPlanningEffort();
        effort.captureSplineGeometry=true;
        planner.setContinuousPlanningEffort(effort);
        planner.reset(39,{});
        const auto fittedStrip=planner.compileContinuous(longStrip,SCALE);
        require(fittedStrip&&*fittedStrip,fittedStrip?
            "long-strip fixture produced no plan":fittedStrip.error());
        const auto quintic=std::ranges::find_if((*fittedStrip)->splineGeometry,
            [](const auto &spline) { return spline.degree==5; });
        require(quintic!=(*fittedStrip)->splineGeometry.end()
                    &&quintic->controls.size()==16,
                "a variable-control short-entity strip should use the selected quintic fitter");
        std::vector<ngc::experimental::JunctionEntity> previewStrip;
        previewStrip.push_back(clusterLine(0.0,1.0));
        auto previewPosition=1.0;
        for(unsigned segment=0;segment<8;++segment) {
            previewStrip.push_back(clusterLine(previewPosition,0.04));
            previewPosition+=0.04;
        }
        previewStrip.push_back(clusterLine(previewPosition,1.0));
        const auto previewFitted=ngc::experimental::buildEvenlySpacedControlCluster(
            previewStrip,0,previewStrip.size()-1,SCALE);
        require(previewFitted&&previewFitted->degree==quintic->degree
                    &&previewFitted->controlPoints.size()==quintic->controls.size(),
                "preview and simulation should select the same spline degree and layout");
        for(std::size_t control=0;control<quintic->controls.size();++control) {
            const auto &previewControl=previewFitted->controlPoints[control];
            const auto &plannedControl=quintic->controls[control];
            require(glm::length(previewControl-glm::dvec3(
                        plannedControl.x,plannedControl.y,plannedControl.z))<1e-12,
                    "preview and simulation should use identical reconstructed XYZ controls");
        }
    }

    void testRollingContinuousHorizonsPreserveMovingPvaBoundary() {
        ngc::BoundedLookaheadTrajectoryPlanner planner({
            .pathAcceleration=10.0,.rapidSpeed=120.0,.arcChordTolerance=0.0001,.pathJerk=100.0,
            .axisVelocity={10,10,10,10,10,10},
            .axisAcceleration={20,20,20,20,20,20},
            .axisJerk={100,100,100,100,100,100},
            .lookaheadDuration=2.0,
        });
        const ngc::TrajectoryPlanningMetadata g64{
            .pathMode=ngc::ExecutablePathMode::Continuous,.pathTolerance=0.01,
        };
        const ngc::position_t p0{0,0,0,0,0,0},p1{1,0,0,0,0,0};
        const ngc::position_t p2{1,1,0,0,0,0},p3{5,1,0,0,0,0};
        planner.reset(74,p0);
        require(planner.enqueue({ngc::MoveLine{p0,p1,60.0},g64,{}}),
                "rolling fixture line 1 should enqueue");
        require(planner.enqueue({ngc::MoveLine{p1,p2,60.0},g64,{}}),
                "rolling fixture line 2 should enqueue");
        require(planner.enqueue({ngc::MoveLine{p2,p3,60.0},g64,{}}),
                "rolling fixture line 3 should enqueue");

        const auto first=planner.planWindow();
        require(first&&*first,first?"rolling fixture produced no first horizon":first.error());
        require(planner.windowSize()==1&&(*first)->inputs.size()==3,
                "the first rolling horizon should commit the original commands and retain one suffix");
        const auto &firstChunk=std::get<ngc::PlanChunk>((*first)->items.back());
        const auto firstEnd=firstChunk.normalMotion[firstChunk.normalMotion.size-1].end;
        require(firstEnd.velocity.length()>0.5,
                "a rolling horizon boundary should remain moving rather than insert an exact stop");
        require(firstChunk.stopTail.size>1,
                "a moving rolling horizon boundary should retain a real bounded stop branch");

        const auto second=planner.planWindow();
        require(second&&*second,second?"rolling fixture produced no final horizon":second.error());
        require(planner.windowSize()==0&&(*second)->inputs.size()==1
                    &&!(*second)->inputs.front().presentationActivation,
                "the final horizon should consume the already-activated retained suffix");
        const auto &secondChunk=std::get<ngc::PlanChunk>((*second)->items.front());
        const auto scaled=[](const ngc::position_t &value,const double amount) {
            return ngc::position_t{value.x*amount,value.y*amount,value.z*amount,
                value.a*amount,value.b*amount,value.c*amount};
        };
        const ngc::MotionState secondStart{
            secondChunk.normalMotion[0].d,
            scaled(secondChunk.normalMotion[0].c,secondChunk.normalMotion[0].inverseDuration),
            scaled(secondChunk.normalMotion[0].b,
                2.0*secondChunk.normalMotion[0].inverseDurationSquared),
        };
        require((firstEnd.position-secondStart.position).length()<1e-8,
                "rolling horizons should preserve position continuity");
        require((firstEnd.velocity-secondStart.velocity).length()<1e-7,
                "rolling horizons should preserve velocity continuity");
        require((firstEnd.acceleration-secondStart.acceleration).length()<1e-7,
                "rolling horizons should preserve acceleration continuity");
        const auto &finalChunk=std::get<ngc::PlanChunk>((*second)->items.back());
        const auto finalState=finalChunk.normalMotion[finalChunk.normalMotion.size-1].end;
        require((finalState.position-p3).length()<1e-8&&finalState.velocity.length()<1e-8
                    &&finalState.acceleration.length()<1e-8,
                "the final rolling horizon should reach the canonical endpoint at rest");
        const auto &diagnostics=planner.diagnostics();
        require(diagnostics.continuousHorizons==2&&diagnostics.blendedCommands==3
                    &&diagnostics.totalContinuousHorizonSeconds
                        >=diagnostics.maximumContinuousHorizonSeconds
                    &&diagnostics.totalPlanningSeconds>=diagnostics.totalContinuousHorizonSeconds,
                "rolling diagnostics should retain per-horizon and total planning computation time");
        require(diagnostics.publishedSplineInverse.queries>0
                    &&diagnostics.publishedSplineInverse.exactCacheHits>0,
                "rolling diagnostics should accumulate published spline inverse cache work");
        const auto allTimeLaw=ngc::totalTimeLawCalls(diagnostics.timeLaw);
        const auto publishedTimeLaw=ngc::totalTimeLawCalls(diagnostics.publishedTimeLaw);
        const auto prefixTimeLaw=ngc::totalTimeLawCalls(diagnostics.rollingPrefixProbeTimeLaw);
        const auto suffixTimeLaw=ngc::totalTimeLawCalls(diagnostics.rollingSuffixProbeTimeLaw);
        require(allTimeLaw.calls>=publishedTimeLaw.calls&&publishedTimeLaw.calls>0
                    &&prefixTimeLaw.calls>0&&suffixTimeLaw.calls>0
                    &&allTimeLaw.calls==allTimeLaw.successes+allTimeLaw.failures,
                "rolling diagnostics should classify published and provisional time-law attempts");
    }

    void testRollingPrefixRejectsShortEntityAnchors() {
        ngc::BoundedLookaheadTrajectoryPlanner planner({
            .pathAcceleration=10.0,.rapidSpeed=120.0,.arcChordTolerance=0.0001,.pathJerk=100.0,
            .axisVelocity={10,10,10,10,10,10},
            .axisAcceleration={20,20,20,20,20,20},
            .axisJerk={100,100,100,100,100,100},
            .lookaheadDuration=0.05,
        });
        constexpr double SCALE=0.01;
        constexpr double SHORT_LENGTH=0.05;
        const ngc::TrajectoryPlanningMetadata g64{
            .pathMode=ngc::ExecutablePathMode::Continuous,.pathTolerance=SCALE,
        };
        ngc::position_t from{};
        planner.reset(75,from);
        constexpr std::size_t COMMANDS=8;
        for(std::size_t command=0;command<COMMANDS;++command) {
            auto to=from;
            to.x+=SHORT_LENGTH;
            require(planner.enqueue({ngc::MoveLine{from,to,60.0},g64,{}}),
                    "short-anchor fixture command should enqueue");
            from=to;
        }
        require(planner.shouldPlanRollingPrefix(),
                "short-anchor fixture should exceed the rolling lookahead duration");
        const auto prefix=planner.planWindow(false);
        require(prefix&&!*prefix,prefix?
                "short entities must not publish a rolling prefix":prefix.error());
        require(planner.windowSize()==COMMANDS
                    &&planner.diagnostics().rollingBoundaryCandidates==0,
                "rejecting short rolling anchors should retain the complete smoothing window");
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

    void testExactStopLeadInAdvancesRollingBoundary() {
        ngc::BoundedLookaheadTrajectoryPlanner planner({
            .pathAcceleration=10.0,.rapidSpeed=120.0,.arcChordTolerance=0.0001,.pathJerk=100.0,
            .axisVelocity={10,10,10,10,10,10},
            .axisAcceleration={20,20,20,20,20,20},
            .axisJerk={100,100,100,100,100,100},
            .lookaheadDuration=2.0,
        });
        const ngc::TrajectoryPlanningMetadata exactStop{
            .pathMode=ngc::ExecutablePathMode::ExactStop,.pathTolerance=std::nullopt,
        };
        const ngc::TrajectoryPlanningMetadata g64{
            .pathMode=ngc::ExecutablePathMode::Continuous,.pathTolerance=0.01,
        };
        const ngc::position_t probeStop{-10,2,-4,0,0,0};
        const ngc::position_t p0{0,0,0,0,0,0},p1{1,0,0,0,0,0};
        const ngc::position_t p2{1,1,0,0,0,0},p3{5,1,0,0,0,0};
        planner.reset(76,probeStop);
        std::size_t progressCallbacks=0;
        planner.setProgressCallback([&] { ++progressCallbacks; });
        require(planner.enqueue({ngc::MoveLine{probeStop,p0,60.0},exactStop,{}}),
                "exact-stop lead-in should enqueue");
        const auto leadIn=planner.planWindow();
        require(leadIn&&*leadIn,leadIn?"exact-stop lead-in produced no plan":leadIn.error());

        require(planner.enqueue({ngc::MoveLine{p0,p1,60.0},g64,{}})
                    &&planner.enqueue({ngc::MoveLine{p1,p2,60.0},g64,{}})
                    &&planner.enqueue({ngc::MoveLine{p2,p3,60.0},g64,{}}),
                "post-probe rolling fixture should enqueue");
        const auto rolling=planner.planWindow();
        require(rolling&&*rolling,rolling?
                "post-probe rolling fixture produced no horizon":rolling.error());
        const auto &firstChunk=std::get<ngc::PlanChunk>((*rolling)->items.front());
        require((firstChunk.normalMotion[0].d-p0).length()<1e-9,
                "a G64 horizon after exact-stop motion should start at the exact-stop endpoint");
        require(progressCallbacks>0,
                "continuous planning should cooperatively report NRT presentation progress");
    }

    void testG64C2PrecisionUsesLocalCoordinates() {
        ngc::BoundedLookaheadTrajectoryPlanner planner({
            .pathAcceleration=5.0,.rapidSpeed=120.0,.arcChordTolerance=0.0001,.pathJerk=20.0,
            .axisVelocity={10,10,10,10,10,10},
            .axisAcceleration={10,10,10,10,10,10},
            .axisJerk={50,50,50,50,50,50},
        });
        const ngc::TrajectoryPlanningMetadata g64 {
            .pathMode=ngc::ExecutablePathMode::Continuous,.pathTolerance=0.001,
        };
        const ngc::position_t p0{1000000.0,-1000000.0,500000.0,0,0,0};
        const ngc::position_t p1{1000000.010,-999999.998,499999.999,0,0,0};
        const ngc::position_t p2{1000000.015,-999999.991,499999.997,0,0,0};
        const ngc::position_t p3{1000000.021,-999999.987,499999.992,0,0,0};
        const ngc::position_t p4{1000000.028,-999999.980,499999.990,0,0,0};
        planner.reset(73,p0);
        require(planner.enqueue({ngc::MoveLine{p0,p1,72.0},g64,{}}),"precision line 1 should enqueue");
        require(planner.enqueue({ngc::MoveLine{p1,p2,72.0},g64,{}}),"precision line 2 should enqueue");
        require(planner.enqueue({ngc::MoveLine{p2,p3,72.0},g64,{}}),"precision line 3 should enqueue");
        require(planner.enqueue({ngc::MoveLine{p3,p4,72.0},g64,{}}),"precision line 4 should enqueue");
        const auto planned=planner.planWindow();
        require(planned&&*planned,planned?"precision G64 window produced no plan":planned.error());
        const auto &chunk=std::get<ngc::PlanChunk>((*planned)->primaryItem());
        const auto scaled=[](const ngc::position_t &value,const double amount) {
            return ngc::position_t{value.x*amount,value.y*amount,value.z*amount,
                value.a*amount,value.b*amount,value.c*amount};
        };
        for(std::size_t index=1;index<chunk.normalMotion.size;++index) {
            const auto &previous=chunk.normalMotion[index-1];
            const auto &current=chunk.normalMotion[index];
            const auto currentVelocity=scaled(current.c,current.inverseDuration);
            const auto currentAcceleration=scaled(current.b,2.0*current.inverseDurationSquared);
            require((previous.end.position-current.d).length()<1e-8,
                    "large-coordinate G64 position continuity should remain exact");
            require((previous.end.velocity-currentVelocity).length()<1e-7,
                    "large-coordinate G64 velocity continuity should remain within tolerance");
            require((previous.end.acceleration-currentAcceleration).length()<1e-7,
                std::format("large-coordinate G64 acceleration jump {} exceeds tolerance",
                    (previous.end.acceleration-currentAcceleration).length()));
        }
    }

    void testContinuousTrajectoryPacketizesBeyondNormalSpanCapacity() {
        ngc::ExactStopTrajectoryPlanner planner({
            .pathAcceleration=5.0,.rapidSpeed=120.0,.arcChordTolerance=0.0001,.pathJerk=20.0,
            .axisVelocity={10,10,10,10,10,10},
            .axisAcceleration={10,10,10,10,10,10},
            .axisJerk={50,50,50,50,50,50},
        });
        constexpr std::size_t COMMANDS=150;
        std::vector<ngc::MachineCommand> commands;
        commands.reserve(COMMANDS);
        ngc::position_t from{};
        for(std::size_t command=0;command<COMMANDS;++command) {
            auto to=from;
            to.x+=1.0;
            commands.emplace_back(ngc::MoveLine{from,to,120.0});
            from=to;
        }
        planner.reset(76);
        const auto planned=planner.compileContinuous(commands,0.01);
        require(planned&&*planned,planned?"large continuous plan produced no packets":planned.error());
        require((*planned)->chunks.size()>=2,
                "a staged continuous trajectory larger than 256 spans should use multiple RT packets");
        require((*planned)->chunks.front().normalMotion.size==ngc::MAX_NORMAL_SPANS_PER_CHUNK,
                "the first continuous packet should use the available normal-span capacity");
        ngc::BranchSequence predecessor=0;
        std::optional<ngc::MotionState> previous;
        for(std::size_t packet=0;packet<(*planned)->chunks.size();++packet) {
            const auto &chunk=(*planned)->chunks[packet];
            require(chunk.predecessorBranch==predecessor,
                    "continuous packets should retain an ordered branch chain");
            require(chunk.normalMotion.size>0
                        &&chunk.normalMotion.size<=ngc::MAX_NORMAL_SPANS_PER_CHUNK,
                    "every continuous packet should respect the RT normal-span bound");
            require(chunk.stopTail.size>0&&chunk.stopTail.size<=ngc::MAX_STOP_SPANS_PER_CHUNK,
                    "every continuous packet should contain a complete bounded stop tail");
            const auto scale=[](const ngc::position_t &value,const double amount) {
                return ngc::position_t{value.x*amount,value.y*amount,value.z*amount,
                    value.a*amount,value.b*amount,value.c*amount};
            };
            const ngc::MotionState start{
                chunk.normalMotion[0].d,
                scale(chunk.normalMotion[0].c,chunk.normalMotion[0].inverseDuration),
                scale(chunk.normalMotion[0].b,2.0*chunk.normalMotion[0].inverseDurationSquared),
            };
            if(previous) {
                require((previous->position-start.position).length()<1e-8,
                        "continuous packet boundaries should preserve position");
                require((previous->velocity-start.velocity).length()<1e-7,
                        "continuous packet boundaries should preserve velocity");
                require((previous->acceleration-start.acceleration).length()<1e-7,
                        "continuous packet boundaries should preserve acceleration");
            }
            require(chunk.stopState.velocity.length()<1e-8
                        &&chunk.stopState.acceleration.length()<1e-8,
                    "every packet stop branch should terminate stationary");
            previous=chunk.normalMotion[chunk.normalMotion.size-1].end;
            predecessor=chunk.branch;
        }
        require(previous&&previous->velocity.length()<1e-8&&previous->acceleration.length()<1e-8,
                "the final continuous packet should retain the window's rest boundary");
    }

    double maximumContinuousXVelocityInRange(const ngc::ContinuousTrajectoryPlan &plan,
                                              const double from,const double to) {
        auto maximum=0.0;
        for(const auto &chunk:plan.chunks) for(const auto &span:chunk.normalMotion) {
            const auto middle=(span.d.x+span.end.position.x)/2.0;
            if(middle>=from&&middle<=to)
                maximum=std::max(maximum,spanAxisVelocity(span,&ngc::position_t::x));
        }
        return maximum;
    }

    double continuousDuration(const ngc::ContinuousTrajectoryPlan &plan) {
        auto duration=0.0;
        for(const auto &chunk:plan.chunks) for(const auto &span:chunk.normalMotion)
            duration+=span.duration;
        return duration;
    }

    void testDenseContinuousTimingFixture() {
        const auto source=ngc::readFile("g64_dense_timing_test.ngc");
        require(source.has_value(),source ? "" : source.error().what());
        ngc::Machine machine(UNIT);
        const auto commands=execute(machine,*source);
        require(commands.size()==22,"dense timing fixture should emit 22 motion commands");
        ngc::ExactStopTrajectoryPlanner planner({
            .pathAcceleration=20.0,.rapidSpeed=199.8,.arcChordTolerance=0.0001,.pathJerk=100.0,
            .axisVelocity={3.333333333333333,3.333333333333333,3.333333333333333,
                           3.333333333333333,3.333333333333333,3.333333333333333},
            .axisAcceleration={100,100,100,100,100,100},
            .axisJerk={100,100,100,100,100,100},
        });
        planner.setContinuousPlanningEffort({
            .measureStationVisitReplay=true,
            .enableStationVisitReplay=false,
        });
        planner.reset(79);
        ngc::ContinuousAccelerationOracleModel oracleModel;
        ngc::InfiniteJerkTrajectoryTimeResult infiniteJerkTime;
        const auto planned=planner.compileContinuous(commands,0.001,&oracleModel,
            std::nullopt,std::nullopt,{},12U,&infiniteJerkTime);
        require(planned&&*planned,planned ? "dense timing fixture produced no plan" : planned.error());
        require(infiniteJerkTime.duration>0.0
                    &&infiniteJerkTime.duration<continuousDuration(**planned),
                "dense smoothed-path infinite-jerk oracle should be faster than the "
                "executable jerk-limited plan");
        const auto entry=maximumContinuousXVelocityInRange(**planned,0.2,0.8);
        const auto denseLines=maximumContinuousXVelocityInRange(**planned,1.01,1.19);
        const auto denseArcs=maximumContinuousXVelocityInRange(**planned,1.21,1.39);
        const auto exit=maximumContinuousXVelocityInRange(**planned,1.6,2.2);
        require(entry>1.9&&exit>1.9,"long reference lines should attain the programmed F120 speed");
        require(denseLines>0.4&&denseLines<0.6&&denseArcs>0.4&&denseArcs<0.6,
                "dense fixture should expose its small-P cubic-blend jerk ceiling");
        require(continuousDuration(**planned)<2.5,
                "dense line/arc timing fixture should remain quick enough for focused debugging");
        require((*planned)->accelerationAwareDuration
                    <(*planned)->velocityOnlySeedDuration-1e-4,
                std::format("acceleration-aware reachability should improve the velocity-only seed: "
                    "aware={} seed={}",(*planned)->accelerationAwareDuration,
                    (*planned)->velocityOnlySeedDuration));
        require(std::ranges::any_of((*planned)->pieceTiming,[](const auto &piece) {
            return std::abs(piece.entryAcceleration)>1e-3
                ||std::abs(piece.exitAcceleration)>1e-3;
        }),"dense timing fixture should carry nonzero acceleration across a piece boundary");
        require(std::ranges::any_of((*planned)->pieceTiming,[](const auto &piece) {
            return piece.velocityLimit<0.55;
        }),"dense timing diagnostics should expose the local jerk-derived blend cap");
        require(oracleModel.segments.size()==(*planned)->pieceTiming.size()*32,
                "the optional Clarabel model should subdivide every geometry piece");
        require(oracleModel.pieceTiming.size()==(*planned)->pieceTiming.size(),
                "the optional Clarabel model should retain planner station states");
        requireNear(oracleModel.pathJerk,100.0,
                "the optional Clarabel model should retain aggregate jerk");
        requireNear(oracleModel.plannerDuration,continuousDuration(**planned),
                "the Clarabel model should retain the current planner duration");
        require(std::ranges::all_of(oracleModel.segments,[](const auto &segment) {
            return segment.length>0.0&&segment.velocityLimit>0.0
                &&std::abs(segment.tangent.length()-1.0)<1e-8;
        }),"the Clarabel model should contain finite positive unit-tangent intervals");
        const auto &inverse=(*planned)->splineInverse;
        require(inverse.queries>inverse.endpointQueries&&inverse.exactCacheHits>0,
                "dense timing should exercise the exact spline inverse cache");
        require(inverse.constructionIntegralEvaluations>0
                    &&inverse.inverseIntegralEvaluations==inverse.newtonIterations
                    &&inverse.inverseIntegralEvaluations
                        <inverse.queries-inverse.endpointQueries,
                "exact spline inverse caching should avoid repeated adaptive integrals");
        require(inverse.iterationLimitHits==0&&inverse.maximumNewtonIterations<=12,
                "cached spline inverses should retain bounded safeguarded Newton correction");
        const auto &arcInverse=(*planned)->arcInverse;
        require(arcInverse.queries>0&&arcInverse.exactCacheHits>0,
                "dense timing should exercise the exact arc inverse cache");
        require(arcInverse.constructionIntegralEvaluations>0
                    &&arcInverse.inverseIntegralEvaluations==arcInverse.newtonIterations
                    &&arcInverse.inverseIntegralEvaluations<arcInverse.queries,
                "exact arc inverse caching should avoid repeated adaptive integrals");
        require(arcInverse.iterationLimitHits==0&&arcInverse.maximumNewtonIterations<=12,
                "cached arc inverses should retain bounded safeguarded Newton correction");
        const auto &timeLaw=(*planned)->timeLaw;
        const auto totalTimeLaw=ngc::totalTimeLawCalls(timeLaw);
        const auto resultsAreComplete=[](const ngc::TimeLawCallDiagnostics &value) {
            return value.calls==value.successes+value.failures;
        };
        require(totalTimeLaw.calls>0&&totalTimeLaw.calls==totalTimeLaw.successes+totalTimeLaw.failures
                    &&totalTimeLaw.cacheHits>0&&totalTimeLaw.solverCalls<totalTimeLaw.calls
                    &&totalTimeLaw.cacheMisses+totalTimeLaw.cacheHits
                        ==totalTimeLaw.calls-timeLaw.continuousSeed.calls
                    &&timeLaw.exactStop.calls==0
                    &&timeLaw.continuousSeed.calls>=(*planned)->pieceTiming.size()
                    &&timeLaw.stationCurrentVelocity.calls>0
                    &&timeLaw.stationCapVelocity.calls>0
                    &&timeLaw.stationVelocityBracket.calls>0
                    &&resultsAreComplete(timeLaw.continuousSeed)
                    &&resultsAreComplete(timeLaw.stationCurrentVelocity)
                    &&resultsAreComplete(timeLaw.stationCapVelocity)
                    &&resultsAreComplete(timeLaw.stationVelocityBracket),
                "continuous time-law instrumentation should classify every solver result");
        const auto &endpointFeasibility=timeLaw.endpointFeasibility;
        require(endpointFeasibility.cachedGeometryEndpoints==2*(*planned)->pieceTiming.size()
                    &&endpointFeasibility.candidateChecks>0
                    &&endpointFeasibility.accelerationRejections
                        +endpointFeasibility.jerkRejections
                        <=endpointFeasibility.candidateChecks,
                "dense timing should reuse cached one-sided geometry and classify coupled "
                "endpoint rejections before local timing");
        const auto &locality=timeLaw.correctionPassLocality;
        require((totalTimeLaw.correctionPassCalls==0)==locality.empty(),
                "correction locality should record every pass after the initial solve");
        for(const auto &pass:locality) {
            require(pass.pass>0&&pass.pieceCount==(*planned)->pieceTiming.size()
                        &&pass.correctedPieces>0
                        &&pass.changedPieceTimings+pass.bitExactReusablePieceTimings
                            ==pass.pieceCount
                        &&pass.changedUncorrectedPieceTimings<=pass.changedPieceTimings
                        &&pass.firstCorrectedPiece<=pass.lastCorrectedPiece
                        &&pass.lastCorrectedPiece<pass.pieceCount
                        &&pass.bitExactReusablePrefixPieces
                            +pass.bitExactReusableSuffixPieces
                            <=pass.bitExactReusablePieceTimings,
                    "correction locality should partition exact timing reuse and retain valid "
                    "corrected-piece bounds");
            require(pass.changedPieceTimings==0
                        ||(pass.changedPieceTimingRuns>0
                           &&pass.maximumChangedPieceTimingRun>0
                           &&pass.firstChangedPieceTiming<=pass.lastChangedPieceTiming
                           &&pass.lastChangedPieceTiming<pass.pieceCount),
                    "changed correction timings should retain bounded affected runs");
        }
        const auto &replay=timeLaw.stationVisitReplay;
        require(replay.exactOutputMatches+replay.outputMismatches
                    ==replay.exactInputMatches
                    &&replay.exactInputMatches<=replay.comparableVisits
                    &&replay.comparableVisits<=replay.activeVisits
                    &&replay.potentialTimeLawCalls<=totalTimeLaw.correctionPassCalls
                    &&replay.potentialSolverCalls<=totalTimeLaw.solverCalls
                    &&replay.potentialMaterializations<=totalTimeLaw.cacheMaterializations,
                "station replay shadow diagnostics should compare every exact input and bound "
                "the work attributed to verified output matches");

        planner.reset(80);
        const auto repeated=planner.compileContinuous(commands,0.001);
        require(repeated&&*repeated,
                repeated?"repeated timing-cache plan was empty":repeated.error());
        const auto repeatedTimeLaw=ngc::totalTimeLawCalls((*repeated)->timeLaw);
        require(repeatedTimeLaw.cacheHits>0
                    &&repeatedTimeLaw.solverCalls<totalTimeLaw.solverCalls,
                "thread-local timing cache should reuse exact results across compilations");
        requireNear(continuousDuration(**repeated),continuousDuration(**planned),
                "cross-compilation timing-cache reuse should preserve duration");

    }

    void testContinuousTimingRetainsRuckigBrakePreProfile() {
        const auto source=ngc::readFile("1001.ngc");
        require(source.has_value(),source ? "" : source.error().what());
        ngc::Machine machine(UNIT);
        const auto emitted=execute(machine,*source);
        std::vector<ngc::MachineCommand> commands;
        for(const auto &command:emitted) {
            const auto compatible=std::visit([](const auto &value) {
                using T=std::decay_t<decltype(value)>;
                if constexpr(std::same_as<T,ngc::MoveLine>)
                    return value.speed()>0.0&&!value.machineCoordinates();
                else if constexpr(std::same_as<T,ngc::MoveArc>) return value.speed()>0.0;
                else return false;
            },command);
            if(compatible) commands.push_back(command);
        }
        require(commands.size()==242,
                "Ruckig regression fixture should retain its 242-motion G64 horizon");

        const auto configuration=ngc::loadMachineConfiguration("machine.toml");
        require(configuration.has_value(),configuration ? "" : configuration.error());
        const auto start=std::visit([](const auto &value) -> ngc::position_t {
            using T=std::decay_t<decltype(value)>;
            if constexpr(std::same_as<T,ngc::MoveLine>||std::same_as<T,ngc::MoveArc>)
                return value.from();
            else return {};
        },commands.front());
        ngc::ExactStopTrajectoryPlanner planner(configuration->trajectory);
        planner.reset(80,start);
        const auto planned=planner.compileContinuous(commands,0.005);
        require(planned&&*planned,planned
            ? "Ruckig brake regression fixture produced no plan" : planned.error());

        const auto scaled=[](const ngc::position_t &value,const double amount) {
            return ngc::position_t{value.x*amount,value.y*amount,value.z*amount,
                value.a*amount,value.b*amount,value.c*amount};
        };
        std::optional<ngc::MotionState> previous;
        for(const auto &chunk:(*planned)->chunks) for(const auto &span:chunk.normalMotion) {
            require(std::isfinite(span.duration)&&span.duration>0.0,
                    "Ruckig brake regression spans should have finite positive durations");
            const ngc::MotionState startState{
                span.d,
                scaled(span.c,span.inverseDuration),
                scaled(span.b,2.0*span.inverseDurationSquared),
            };
            if(previous) {
                require((previous->position-startState.position).length()<1e-8,
                        "Ruckig brake timing should preserve position continuity");
                require((previous->velocity-startState.velocity).length()<1e-7,
                        "Ruckig brake timing should preserve velocity continuity");
                require((previous->acceleration-startState.acceleration).length()<1e-7,
                        "Ruckig brake timing should preserve acceleration continuity");
            }
            previous=span.end;
        }
        require(previous.has_value(),"Ruckig brake regression should emit normal motion");
    }

    void testContinuousTimingUsesLocalEntityAndAverageBlendFeeds() {
        ngc::ExactStopTrajectoryPlanner planner({
            .pathAcceleration=100.0,.rapidSpeed=120.0,.arcChordTolerance=0.0001,.pathJerk=1000.0,
            .axisVelocity={100,100,100,100,100,100},
            .axisAcceleration={100,100,100,100,100,100},
            .axisJerk={1000,1000,1000,1000,1000,1000},
        });
        const std::array<ngc::MachineCommand,2> commands {
            ngc::MoveLine{{},{100,0,0,0,0,0},60.0},
            ngc::MoveLine{{100,0,0,0,0,0},{200,0,0,0,0,0},180.0},
        };
        planner.reset(77);
        const auto planned=planner.compileContinuous(commands,1.0);
        require(planned&&*planned,planned?"local-feed plan produced no trajectory":planned.error());
        const auto incoming=maximumContinuousXVelocityInRange(**planned,10.0,90.0);
        const auto blend=maximumContinuousXVelocityInRange(**planned,97.0,103.0);
        const auto outgoing=maximumContinuousXVelocityInRange(**planned,110.0,190.0);
        require(incoming<=1.0+1e-8&&incoming>0.95,
                "the retained incoming line should use its F60 local cap");
        require(blend<=2.0+1e-8&&blend>1.9,
                "the blend between F60 and F180 should use their F120 arithmetic mean");
        require(outgoing<=3.0+1e-8&&outgoing>2.9,
                "the retained outgoing line should use its F180 local cap");
    }

    void testCollapsedClusterRetainsLocalFeedChanges() {
        ngc::ExactStopTrajectoryPlanner planner({
            .pathAcceleration=100.0,.rapidSpeed=120.0,.arcChordTolerance=0.0001,
            .pathJerk=1000.0,
            .axisVelocity={100,100,100,100,100,100},
            .axisAcceleration={100,100,100,100,100,100},
            .axisJerk={1000,1000,1000,1000,1000,1000},
        });
        const std::array<ngc::MachineCommand,3> commands {
            ngc::MoveLine{{},{1,0,0,0,0,0},40.0},
            ngc::MoveLine{{1,0,0,0,0,0},{1.04,0,0,0,0,0},72.0},
            ngc::MoveLine{{1.04,0,0,0,0,0},{2.04,0,0,0,0,0},72.0},
        };
        planner.reset(80);
        const auto planned=planner.compileContinuous(commands,0.01);
        require(planned&&*planned,planned?
                "local cluster-feed plan produced no trajectory":planned.error());
        require((*planned)->pieceTiming.size()==6,
                "a feed change inside a collapsed cluster should preserve per-knot timing "
                "pieces and subdivide only the span containing the feed boundary");
        const auto &slowCluster=(*planned)->pieceTiming[2];
        const auto &fastCluster=(*planned)->pieceTiming[3];
        requireNear(slowCluster.velocityLimit,40.0/60.0,
                "the incoming cluster portion should retain its F40 cap");
        require(fastCluster.velocityLimit>slowCluster.velocityLimit+1e-8
                &&fastCluster.velocityLimit<=72.0/60.0+1e-8,std::format(
                "the remaining cluster portion should recover toward its F72 cap: [{}, {}, {}, "
                "{}, {}, {}]",(*planned)->pieceTiming[0].velocityLimit,
                (*planned)->pieceTiming[1].velocityLimit,
                (*planned)->pieceTiming[2].velocityLimit,
                (*planned)->pieceTiming[3].velocityLimit,
                (*planned)->pieceTiming[4].velocityLimit,
                (*planned)->pieceTiming[5].velocityLimit));
        require(slowCluster.exitVelocity<=40.0/60.0+1e-8,
                "the shared feed-change station should respect the lower adjacent cap");
    }

    void testDistantSlowEntityDoesNotThrottleFastContinuousPrefix() {
        ngc::ExactStopTrajectoryPlanner planner({
            .pathAcceleration=10.0,.rapidSpeed=120.0,.arcChordTolerance=0.0001,.pathJerk=100.0,
            .axisVelocity={10,10,10,10,10,10},
            .axisAcceleration={10,10,10,10,10,10},
            .axisJerk={100,100,100,100,100,100},
        });
        const std::array<ngc::MachineCommand,3> commands {
            ngc::MoveLine{{},{100,0,0,0,0,0},80.0},
            ngc::MoveLine{{100,0,0,0,0,0},{101,0,0,0,0,0},1.0},
            ngc::MoveLine{{101,0,0,0,0,0},{201,0,0,0,0,0},80.0},
        };
        planner.reset(78);
        const auto planned=planner.compileContinuous(commands,0.01);
        require(planned&&*planned,planned?"local-lookahead plan produced no trajectory":planned.error());
        const auto fastPrefix=maximumContinuousXVelocityInRange(**planned,10.0,80.0);
        const auto slowMiddle=maximumContinuousXVelocityInRange(**planned,100.1,100.9);
        require(fastPrefix>1.25&&fastPrefix<=80.0/60.0+1e-8,
                "a distant F1 entity should not globally throttle the earlier F80 line");
        require(slowMiddle<=1.0/60.0+1e-8&&slowMiddle>0.0,
                "the retained F1 entity should still enforce its own local feed cap");
    }

    void testContinuousPlanningFailureIsFatalAndReported() {
        ngc::InterpreterSession session(UNIT,ngc::InterpretationMode::Preview);
        compileSession(session,"G64 P0.01\nG1 F60 X1\nG1 Y1\n");
        session.begin();
        ngc::MockMotionBackend backend;
        ngc::TrajectoryExecutionDriver driver(session,backend,{
            .pathAcceleration=0.0,.rapidSpeed=100.0,.arcChordTolerance=0.0001,.pathJerk=10.0,
        });
        require(driver.begin(74),"fatal G64 test should initialize the backend");
        backend.advance(0.0);
        driver.serviceBackend();
        int observed=0;
        for(int guard=0;guard<100&&driver.state()==ngc::TrajectoryDriverState::Running;++guard)
            (void)driver.pumpOne([](const auto &callback){callback();},
                [&](const ngc::MachineCommand &){++observed;});
        require(driver.state()==ngc::TrajectoryDriverState::Error,
                "failed continuous planning must stop the G-code run");
        require(driver.error()&&driver.error()->contains("fatal continuous-motion compilation failure")
                    &&driver.error()->contains("G64 window commands=2")
                    &&driver.error()->contains("trajectory limits must be positive"),
                driver.error()?*driver.error():"fatal G64 failure did not retain an error");
        require(observed==0,"a failed continuous window must not publish any commands");
        require(driver.planningDiagnostics().continuousExactStops==0,
                "a failed continuous window must not be counted as an individually planned exact stop");
        const auto &messages=session.statusMessages();
        require(!messages.empty()&&messages.back().kind==ngc::InterpreterStatusKind::Error
                    &&messages.back().text==*driver.error(),
                "fatal trajectory planning errors must enter the chronological UI status stream");
        session.stop();
    }

    void testG64ExecutesRetainedArcAndLocalSpline() {
        const ngc::TrajectoryPlanningMetadata g64 {
            .pathMode=ngc::ExecutablePathMode::Continuous,.pathTolerance=0.1,
        };
        ngc::BoundedLookaheadTrajectoryPlanner arcPlanner({
            .pathAcceleration=5.0,.rapidSpeed=120.0,.arcChordTolerance=0.0001,.pathJerk=20.0,
            .lookaheadDuration=1000.0,
        });
        const ngc::position_t a0{-2,0,0,0,0,0},a1{-1,0,0,0,0,0},a2{0,-1,0,0,0,0};
        arcPlanner.reset(71,a0);
        require(arcPlanner.enqueue({ngc::MoveLine{a0,a1,60.0},g64,{}}),"G64 line should enter arc window");
        require(arcPlanner.enqueue({ngc::MoveArc{a1,a2,{}, {0,0,1},60.0},g64,{}}),
                "G64 arc should enter arc window");
        const auto arcPlan=arcPlanner.planWindow();
        require(arcPlan.has_value()&&*arcPlan,arcPlan?"arc G64 window produced no plan":arcPlan.error());
        require((*arcPlan)->inputs.size()==2&&arcPlanner.diagnostics().continuousExactStops==0
                    &&arcPlanner.diagnostics().blendedWindows==1,
                "line/arc G64 should execute as one piecewise blended window");
        const auto *chunk=std::get_if<ngc::PlanChunk>(&(*arcPlan)->primaryItem());
        require(chunk&&chunk->normalMotion.size>3,
                "line/arc G64 should retain adaptively verified curved geometry");
        requireNear(chunk->normalMotion[0].d.x,a0.x,
                    "line/arc G64 should begin at the canonical line endpoint");
        requireNear(chunk->normalMotion[chunk->normalMotion.size-1].end.position.x,a2.x,
                    "line/arc G64 should finish at the canonical arc endpoint");
        requireNear(chunk->normalMotion[chunk->normalMotion.size-1].end.position.y,a2.y,
                    "line/arc G64 should retain the canonical arc endpoint");
    }

    void testExecutableG64RetainsExactPrimitiveMiddles() {
        ngc::BoundedLookaheadTrajectoryPlanner planner({
            .pathAcceleration=5.0,.rapidSpeed=120.0,.arcChordTolerance=0.0001,.pathJerk=20.0,
            .lookaheadDuration=1000.0,
        });
        const ngc::position_t p0{0,0,0,0,0,0},p1{10,0,0,0,0,0};
        const ngc::position_t p2{10,10,0,0,0,0},p3{20,10,0,0,0,0};
        const ngc::TrajectoryPlanningMetadata g64 {
            .pathMode=ngc::ExecutablePathMode::Continuous,.pathTolerance=1.0,
        };
        planner.reset(72,p0);
        require(planner.enqueue({ngc::MoveLine{p0,p1,120.0},g64,{}}),"first retained line should enqueue");
        require(planner.enqueue({ngc::MoveLine{p1,p2,120.0},g64,{}}),"middle retained line should enqueue");
        require(planner.enqueue({ngc::MoveLine{p2,p3,120.0},g64,{}}),"last retained line should enqueue");
        const auto planned=planner.planWindow();
        require(planned&&*planned,planned?"retained-line G64 produced no plan":planned.error());
        require((*planned)->inputs.size()==3,
                std::format("retained-line G64 should blend all commands (single exact stops={}, spans={})",
                    planner.diagnostics().continuousExactStops,
                    std::get<ngc::PlanChunk>((*planned)->primaryItem()).normalMotion.size));
        const auto *chunk=std::get_if<ngc::PlanChunk>(&(*planned)->primaryItem());
        require(chunk,"retained-line G64 should produce a plan chunk");
        bool foundExactMiddle=false;
        for(const auto &span:chunk->normalMotion) {
            const auto midpoint=evaluateSpan(span,0.5);
            if(midpoint.y>3.25&&midpoint.y<6.75&&std::abs(midpoint.x-10.0)<1e-10) {
                require(std::abs(span.a.x)<1e-10&&std::abs(span.b.x)<1e-10
                            &&std::abs(span.c.x)<1e-10&&std::abs(span.d.x-10.0)<1e-10,
                        std::format("the middle of a long entity should remain its exact source line ({}, {}, {}, {})",
                            span.a.x,span.b.x,span.c.x,span.d.x));
                foundExactMiddle=true;
            }
        }
        require(foundExactMiddle,
                "G64 should retain an exact middle section instead of refitting the whole window");
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
            ArcCase { "quarter circle", 1.0, std::numbers::pi / 2.0, 0.0, 4 },
            ArcCase { "half circle", 1.0, std::numbers::pi, 0.0, 8 },
            ArcCase { "major three-quarter circle", 1.0, 3.0 * std::numbers::pi / 2.0, 0.0, 16 },
            ArcCase { "full circle", 1.0, 2.0 * std::numbers::pi, 0.0, 16 },
            ArcCase { "full helical turn", 1.0, 2.0 * std::numbers::pi, 1.0, 16 },
            ArcCase { "large-radius quarter circle", 10.0, std::numbers::pi / 2.0, 0.0, 8 },
        };

        for(const auto &test : cases) {
            const ngc::position_t from { test.radius, 0, 0, 0, 0, 0 };
            const ngc::position_t to {
                test.radius * std::cos(test.sweep), test.radius * std::sin(test.sweep), test.rise, 0, 0, 0,
            };
            ngc::ExactStopTrajectoryPlanner planner({
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

        ngc::ExactStopTrajectoryPlanner planner({
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
        requireNear(first->normalMotion[first->normalMotion.size-1].end.position.y, junction.y,
                    "cached arc terminal state should preserve the canonical endpoint");

        const ngc::position_t lineEnd { -2, 0, 0, 0, 0, 0 };
        const auto lineAfterArc = planner.compile(ngc::MoveLine { secondEnd, lineEnd, 60.0 });
        require(lineAfterArc.has_value(), lineAfterArc ? "" : lineAfterArc.error());
        const auto secondFinal = evaluateSpan(second->normalMotion[second->normalMotion.size-1], 1.0);
        const auto followingLineStart = evaluateSpan(lineAfterArc->normalMotion[0], 0.0);
        requireNear(secondFinal.x, followingLineStart.x, "arc-to-line junction should have continuous X");
        requireNear(secondFinal.y, followingLineStart.y, "arc-to-line junction should have continuous Y");

        ngc::ExactStopTrajectoryPlanner lineThenArc({
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
        ngc::ExactStopTrajectoryPlanner planner({
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
        const auto configuration = ngc::loadMachineConfiguration("machine.toml");
        require(configuration.has_value(), configuration ? "" : configuration.error());
        require(configuration->unit == ngc::Machine::Unit::Inch,
                "machine configuration should load its explicit internal unit");
        require(configuration->trajectory.pathAcceleration > 0.0,
                "machine configuration should load a validated path acceleration");
        require(configuration->trajectory.pathJerk > 0.0,
                "machine configuration should load a validated path jerk");
        require(configuration->trajectory.rapidSpeed > 0.0,
                "machine configuration should load and convert a validated rapid velocity");
        require(configuration->trajectory.arcChordTolerance > 0.0,
                "machine configuration should load a validated arc chord tolerance");
        requireNear(configuration->trajectory.lookaheadDuration,2.0,
                    "machine configuration should load the rolling lookahead duration in seconds");
        requireNear(configuration->simulation.servoPeriod, 0.001,
                    "machine configuration should load the fixed simulation servo period");
        requireNear(configuration->simulation.schedulerPeriod, 0.01,
                    "machine configuration should load the independent scheduler period");
        requireNear(configuration->jogging.acceleration, 5.0,
                    "machine configuration should load the global jog acceleration");
        requireNear(configuration->jogging.jerk, 25.0,
                    "machine configuration should load the global jog jerk");

        require(configuration->coordinates == std::vector {
                    ngc::Machine::Axis::X, ngc::Machine::Axis::Y, ngc::Machine::Axis::Z },
                "machine configuration should load the logical coordinate order");
        const auto axis = [&](const ngc::Machine::Axis value) -> const ngc::AxisConfiguration & {
            const auto found = std::ranges::find(configuration->axes, value, &ngc::AxisConfiguration::axis);
            require(found != configuration->axes.end(), "configured logical axis should exist");
            return *found;
        };
        require(axis(ngc::Machine::Axis::Y).joints == std::vector<ngc::JointId> { 1, 2 },
                "Y should load as one logical axis backed by two joints");
        requireNear(axis(ngc::Machine::Axis::X).minimum, -14.0,
                    "logical X minimum should load in machine units");
        requireNear(axis(ngc::Machine::Axis::Z).maximum, 0.001,
                    "logical Z maximum should load in machine units");
        requireNear(axis(ngc::Machine::Axis::X).maxJerk, 501.0,
                    "logical axes should load an independent jerk limit");
        requireNear(configuration->trajectory.axisVelocity.x, axis(ngc::Machine::Axis::X).maxVelocity,
                    "trajectory limits should retain configured X velocity");
        requireNear(configuration->trajectory.axisAcceleration.y, axis(ngc::Machine::Axis::Y).maxAcceleration,
                    "trajectory limits should retain configured Y acceleration");
        requireNear(configuration->trajectory.axisJerk.z, axis(ngc::Machine::Axis::Z).maxJerk,
                    "trajectory limits should retain configured Z jerk");

        const auto input = [&](const std::string_view name) -> const ngc::DigitalInputConfiguration & {
            const auto found = std::ranges::find(configuration->digitalInputs, name,
                                                 &ngc::DigitalInputConfiguration::name);
            require(found != configuration->digitalInputs.end(), "configured digital input should exist");
            return *found;
        };
        require(input("tool_probe").id == 0 && input("shared_home").id == 1 && input("y2_home").id == 2,
                "logical digital input names should resolve to stable backend IDs");
        require(configuration->probing.input == input("tool_probe").id
                    && configuration->probing.condition == ngc::InputCondition::Active,
                "probing should load its resolved input and condition");
        requireNear(configuration->probing.debounce, 0.010,
                    "probing should load its debounce duration");

        const auto joint = [&](const ngc::JointId id) -> const ngc::JointConfiguration & {
            const auto found = std::ranges::find(configuration->joints, id, &ngc::JointConfiguration::id);
            require(found != configuration->joints.end(), "configured joint should exist");
            return *found;
        };
        require(configuration->joints.size() == 4,
                "machine configuration should load all four physical joints");
        require(joint(1).axis == ngc::Machine::Axis::Y && joint(2).axis == ngc::Machine::Axis::Y,
                "both gantry joints should map to logical Y");
        require(joint(1).homing.input == input("shared_home").id
                    && joint(2).homing.input == input("y2_home").id,
                "each gantry joint should load its own resolved home input");
        requireNear(joint(2).homing.switchPosition, -0.140,
                    "the second Y switch calibration should load independently");
        require(joint(0).homing.searchVelocity > 0.0 && joint(1).homing.searchVelocity < 0.0,
                "signed homing search directions should load without fixing tunable speeds in the test");
        require(joint(0).homing.backoffDistance > 0.0,
                "fixed homing backoff distance should load in machine units");
        require(joint(0).homing.debounce >= 0.0,
                "per-joint home switch debounce should load in seconds");

        require(!configuration->homing.requireBeforeMotion,
                "configured pre-home motion policy should load");
        const auto group = std::ranges::find(configuration->homing.groups, "y_gantry",
                                             &ngc::HomingGroupConfiguration::name);
        require(group != configuration->homing.groups.end(), "Y gantry homing group should load");
        require(group->sequence == 2 && group->joints == std::vector<ngc::JointId> { 1, 2 }
                    && group->startTogether && group->stopEachJointOnTrigger && group->finalMoveTogether,
                "Y gantry homing policy should preserve joint-independent switch stopping");
    }

    void testConfiguredHomingMovesFromPowerUpPosition() {
        const auto configuration = ngc::loadMachineConfiguration("machine.toml");
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
        require(snapshot.toolPose.geometry.number == 0,
                "homing without a loaded tool should retain the no-tool presentation state");
        requireNear(snapshot.toolPosition.x, snapshot.machinePosition.x,
                    "the no-tool position marker should track the machine position");
        worker.join();
    }

    void testSimulationWorkerJogsCoupledJointsBeforeHoming() {
        auto configuration = ngc::loadMachineConfiguration("machine.toml");
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
        ngc::ExactStopTrajectoryPlanner planner({
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

    void testTrajectoryDriverConnectsInterpreterToMockRtBackend() {
        ngc::InterpreterSession session(UNIT, ngc::InterpretationMode::Simulation);
        compileSession(session, "G64 P0.01\nG1 F60 X1\nG1 Z-1\nG1 X2\n");
        session.begin();

        ngc::MockMotionBackend backend;
        ngc::TrajectoryExecutionDriver driver(session, backend, {
            .pathAcceleration = 10.0,
            .rapidSpeed = 100.0,
            .arcChordTolerance = 0.0001,
        });
        require(driver.begin(12), "trajectory driver should initialize bounded backend control channels");

        int observedCommands = 0;
        int continuousCommands = 0;
        std::vector<ngc::SpanId> activationSpans;
        ngc::ExecutionSnapshot latest;
        bool haveSnapshot = false;
        for(int guard = 0; guard < 10000 && driver.state() == ngc::TrajectoryDriverState::Running; ++guard) {
            for(int fill = 0; fill < 64; ++fill) {
                if(!driver.pumpOne([](const auto &callback) { callback(); },
                                   [&](const ngc::MachineCommand &, const ngc::ExecutionItem &,
                                       const ngc::TrajectoryPlanningMetadata &metadata,
                                       const ngc::TrajectoryCommandPresentation &presentation,
                                       const ngc::SpanId activationSpan) {
                                       ++observedCommands;
                                       activationSpans.push_back(activationSpan);
                                       require(!presentation.modalGCodes.empty(),
                                               "lookahead should capture command-boundary modal presentation");
                                       if(metadata.pathMode == ngc::ExecutablePathMode::Continuous
                                          && metadata.pathTolerance == 0.01) ++continuousCommands;
                                   })) break;
            }
            backend.advance(0.01);
            driver.serviceBackend();
            ngc::ExecutionSnapshot snapshot;
            while(backend.tryTakeSnapshot(snapshot)) {
                latest = snapshot;
                haveSnapshot = true;
            }
        }

        const auto completionDiagnostic = std::format(
            "trajectory execution should complete (outstanding={}, interpreted={}, probePending={}, observed={}, driverEpoch={}, backendState={}, epoch={}, chunk={})",
            driver.outstandingChunks(), driver.interpretationComplete(), driver.probePending(), observedCommands,
            driver.epoch(), static_cast<int>(latest.state), latest.epoch, latest.activeChunk);
        require(driver.state() == ngc::TrajectoryDriverState::Completed,
                driver.error() ? *driver.error() : completionDiagnostic);
        require(observedCommands == 3, "trajectory driver should compile every canonical motion command once");
        require(continuousCommands == 3,
                "trajectory driver should capture G64 metadata at each canonical command boundary");
        require(std::ranges::is_sorted(activationSpans)&&std::ranges::all_of(activationSpans,
                    [](const auto span) { return span!=0; }),
                "combined G64 commands should retain ordered activation span identities");
        require(driver.planningDiagnostics().continuousExactStops == 0
                    && driver.planningDiagnostics().blendedWindows == 2
                    && driver.planningDiagnostics().blendedCommands == 3
                    && driver.planningDiagnostics().maximumWindowCommands == 3
                    && driver.planningDiagnostics().continuousHorizons == 2
                    && driver.planningDiagnostics().totalContinuousHorizonSeconds
                        >=driver.planningDiagnostics().maximumContinuousHorizonSeconds,
                std::format("driver G64 diagnostics mismatch (single exact stops={}, windows={}, commands={}, window={})",
                    driver.planningDiagnostics().continuousExactStops,
                    driver.planningDiagnostics().blendedWindows,
                    driver.planningDiagnostics().blendedCommands,
                    driver.planningDiagnostics().maximumWindowCommands));
        require(haveSnapshot, "mock RT backend should expose execution snapshots through SPSC communication");
        requireNear(latest.commanded.position.x, 2.0, "mock RT execution should reach the final planned X position");
        requireNear(latest.commanded.position.z, -1.0,
                    "mock RT execution should preserve its final Z position");
    }

    void testTrajectoryDriverPublishesRollingPrefixBeforeProtectedBoundary() {
        std::string source="G64 P0.01\n";
        constexpr int COMMANDS=12;
        for(int command=1;command<=COMMANDS;++command)
            source+=std::format("G1 F60 X{}\n",command);
        ngc::InterpreterSession session(UNIT,ngc::InterpretationMode::Simulation);
        compileSession(session,source);
        session.begin();

        ngc::MockMotionBackend backend;
        ngc::TrajectoryExecutionDriver driver(session,backend,{
            .pathAcceleration=10.0,.rapidSpeed=100.0,.arcChordTolerance=0.0001,
            .pathJerk=100.0,.lookaheadDuration=2.0,
        });
        require(driver.begin(13),"incremental rolling driver should initialize");
        auto observed=0;
        auto publishedBeforeCompletion=false;
        for(int guard=0;guard<10000&&driver.state()==ngc::TrajectoryDriverState::Running;++guard) {
            for(int fill=0;fill<64;++fill) {
                if(!driver.pumpOne([](const auto &callback){callback();},
                    [&](const ngc::MachineCommand &) {
                        ++observed;
                        if(!driver.interpretationComplete()) publishedBeforeCompletion=true;
                    })) break;
            }
            backend.advance(0.01);
            driver.serviceBackend();
        }
        require(driver.state()==ngc::TrajectoryDriverState::Completed,
                driver.error()?*driver.error():"incremental rolling execution did not complete");
        require(publishedBeforeCompletion,
                "rolling lookahead should publish an immutable prefix before reaching the G64 boundary");
        require(observed==COMMANDS,
                "incremental rolling publication should activate every source command exactly once");
        require(driver.planningDiagnostics().continuousHorizons>=2
                    &&driver.planningDiagnostics().firstContinuousHorizonSeconds>0.0
                    &&driver.planningDiagnostics().totalPlanningSeconds
                        >=driver.planningDiagnostics().totalContinuousHorizonSeconds,
                "incremental rolling execution should retain first/per-horizon/total timing diagnostics");
        session.stop();
    }

    void testHeldRecoveryPreservesBufferedG64Commands() {
        std::string source="G64 P0.001\n";
        constexpr int COMMANDS=70;
        for(int command=1;command<=COMMANDS;++command)
            source+=std::format("G1 F60 X{:.3f}\n",static_cast<double>(command)*0.01);
        ngc::InterpreterSession session(UNIT,ngc::InterpretationMode::Preview);
        compileSession(session,source);
        session.begin();
        ngc::MockMotionBackend backend;
        ngc::TrajectoryExecutionDriver driver(session,backend,{
            .pathAcceleration=10.0,.rapidSpeed=100.0,.arcChordTolerance=0.0001,.pathJerk=100.0,
        });
        require(driver.begin(75),"held-recovery G64 driver should initialize");
        int observed=0;
        for(int guard=0;guard<1000&&driver.state()==ngc::TrajectoryDriverState::Running;++guard) {
            for(int fill=0;fill<64;++fill)
                if(!driver.pumpOne([](const auto &callback){callback();},
                    [&](const ngc::MachineCommand &){++observed;})) break;
            backend.runUntilIdle(0.1);
            driver.serviceBackend();
        }
        require(driver.state()==ngc::TrajectoryDriverState::Completed,
                driver.error()?*driver.error():"held recovery did not complete");
        require(observed==COMMANDS,std::format(
            "held recovery lost or duplicated buffered G64 commands: observed {} expected {}",
            observed,COMMANDS));
        require(driver.planningDiagnostics().commandsPlanned==COMMANDS,
                "held recovery should plan every buffered G64 command exactly once");
        require(driver.planningDiagnostics().blendedWindows==1
                    &&driver.planningDiagnostics().maximumWindowCommands==COMMANDS
                    &&driver.planningDiagnostics().maximumNormalSpans==ngc::MAX_NORMAL_SPANS_PER_CHUNK
                    &&driver.planningDiagnostics().maximumStopSpans>1
                    &&driver.planningDiagnostics().maximumStopSpans<=ngc::MAX_STOP_SPANS_PER_CHUNK,
                "a continuous run beyond the old 32-command/256-span boundaries should publish "
                "one blended horizon as multiple bounded RT packets with a real moving stop tail");
        session.stop();
    }

    void testParameterReadWaitsForPriorMotionCompletion() {
        ngc::InterpreterSession session(UNIT, ngc::InterpretationMode::Simulation);
        compileSession(session, "G1 F60 X1\n#100 = #101\nG1 X2\n");
        require(session.machine().memory().write(101, 7.0).has_value(),
                "parameter-read synchronization test should seed its source parameter");

        ngc::MockMotionBackend backend;
        ngc::TrajectoryExecutionDriver driver(session, backend, {
            .pathAcceleration = 2.0,
            .rapidSpeed = 100.0,
            .arcChordTolerance = 0.0001,
            .pathJerk = 10.0,
        });
        require(driver.begin(40), "parameter-read synchronization driver should initialize");
        backend.advance(0.0);
        driver.serviceBackend();
        const auto parameter = [&] { return session.machine().memory().read(100).value(); };

        int observedCommands = 0;
        for(int guard = 0; guard < 100 && !driver.synchronizationPending(); ++guard)
            require(driver.pumpOne([](const auto &callback) { callback(); },
                                   [&](const ngc::MachineCommand &) { ++observedCommands; }),
                    "driver should reach the parameter-read synchronization barrier");
        require(driver.synchronizationPending(),
                "a parameter read following motion should publish a synchronization wait");
        require(observedCommands == 1,
                "motion after a parameter read must not be interpreted before synchronization");
        requireNear(parameter(), 0.0,
                    "assignment containing the read must not execute while prior motion is queued");

        backend.advanceTick(0.001, true);
        driver.serviceBackend();
        require(driver.synchronizationPending(),
                "a partially executed move must not release a parameter read");
        requireNear(parameter(), 0.0,
                    "parameter read must remain pending while prior motion is moving");

        backend.runUntilIdle(0.001);
        driver.serviceBackend();
        require(!driver.synchronizationPending(),
                "held state after motion completion should satisfy the synchronization wait");
        requireNear(parameter(), 0.0,
                    "satisfying the barrier should not run procedural code on the backend thread");

        backend.advance(0.0);
        driver.serviceBackend();
        for(int guard = 0; guard < 100 && parameter() != 7.0; ++guard)
            (void)driver.pumpOne([](const auto &callback) { callback(); },
                                 [&](const ngc::MachineCommand &) { ++observedCommands; });
        requireNear(parameter(), 7.0,
                    "parameter read should execute after prior motion reaches held state");
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
        testSimulationWorkerStartsPlayback();
        testAdaptivePocketsStartsSimulation();
        testTimedSimulationRefillsMultiPacketContinuousBatch();
        testTimedSimulationPublishesSnapshotsDuringPlanning();
        testImmediatePreviewBuildsGeometryWithoutTrajectoryExecution();
        testGeometryPreviewResolvesProbeAtCanonicalTarget();
        testAdaptivePocketsGeometryPreviewAvoidsTrajectoryExecution();
        testSimulationDriverFailureAppearsInGuiStatusStream();
        test1001PreviewCompletesBoundedly();
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
        testToolpathRecorderIsASeparateConsumer();
        testToolpathRecorderRetainsUsedWorkCoordinateSystems();
        testToolChangePreviewRetainsNestedAndFinalWorkCoordinateSystems();
        testToolpathRecorderAppliesPerCommandToolOffset();
        testSpscChannelIsBoundedAndOrdered();
        testMockMotionBackendUsesProductionTransportContract();
        testJogControlUsesBoundedBackendTransport();
        testMockBackendAdvancesOneFixedServoTick();
        testMockBackendDrainsAFullPlanHorizonWithoutEventOverflow();
        testImmediateDrainStopsAtHeldWithStaleDescendants();
        testN70N75PreviewPrefixesCompleteBoundedly();
        testExactStopPlannerCompilesLinesAndArcs();
        testInfiniteJerkTrajectoryTimeMatchesAnalyticLine();
        testExactStopPlannerEnforcesIndependentAxisLimits();
        testBoundedLookaheadCarriesG64MetadataAndSingleCommandExactStops();
        testPreviewJunctionUsesOneSixControlSpline();
        testShortLineMidpointCurvatureInference();
        testCubicSplineInteriorControlConditioning();
        testGeometricJerkCombIncludesTangentialComponent();
        testMicroEntityClusterCollapsesToOneSpline();
        testBoundedPlannerExecutesPiecewiseG64Geometry();
        testRollingContinuousHorizonsPreserveMovingPvaBoundary();
        testRollingPrefixRejectsShortEntityAnchors();
        testExactStopLeadInAdvancesRollingBoundary();
        testG64C2PrecisionUsesLocalCoordinates();
        testContinuousTrajectoryPacketizesBeyondNormalSpanCapacity();
        testContinuousTimingUsesLocalEntityAndAverageBlendFeeds();
        testCollapsedClusterRetainsLocalFeedChanges();
        testDistantSlowEntityDoesNotThrottleFastContinuousPrefix();
        testDenseContinuousTimingFixture();
        testContinuousTimingRetainsRuckigBrakePreProfile();
        testContinuousPlanningFailureIsFatalAndReported();
        testG64ExecutesRetainedArcAndLocalSpline();
        testExecutableG64RetainsExactPrimitiveMiddles();
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
        testTrajectoryDriverConnectsInterpreterToMockRtBackend();
        testTrajectoryDriverPublishesRollingPrefixBeforeProtectedBoundary();
        testHeldRecoveryPreservesBufferedG64Commands();
        testParameterReadWaitsForPriorMotionCompletion();
        testMotionWordSynchronizationPolicy();
    } catch(const std::exception &error) {
        std::cerr << "ngc_tests failed: " << error.what() << '\n';
        return 1;
    }

    std::cout << "ngc_tests passed\n";
    return 0;
}
