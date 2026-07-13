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
#include "SimulationWorker.h"

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
        const auto main = ngc::readFile("adaptive_pockets.ngc");
        require(hello && world && toolChange && main, "adaptive-pockets simulation inputs should load");

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

    void testMockBackendAdvancesOneFixedServoTick() {
        ngc::MockMotionBackend backend;
        ngc::PlanChunk chunk;
        chunk.epoch = 31;
        chunk.id = 1;
        chunk.branch = 1;
        require(chunk.normalMotion.push(linearSpan(1, 0.0, 1.0, 0.01)),
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
            const ngc::simulation_detail::ArcReference reference(arc);
            require(reference.valid(), "arc reference geometry variant should be valid");
            const auto from = reference.positionAtDistance(0.0);
            const auto to = reference.positionAtDistance(reference.length());
            requireNear(from.x, arc.from().x, "arc reference should preserve start X");
            requireNear(from.y, arc.from().y, "arc reference should preserve start Y");
            requireNear(from.z, arc.from().z, "arc reference should preserve start Z");
            requireNear(to.x, arc.to().x, "arc reference should preserve end X");
            requireNear(to.y, arc.to().y, "arc reference should preserve end Y");
            requireNear(to.z, arc.to().z, "arc reference should preserve end Z");
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
        requireNear(configuration->simulation.servoPeriod, 0.001,
                    "machine configuration should load the fixed simulation servo period");
        requireNear(configuration->simulation.schedulerPeriod, 0.01,
                    "machine configuration should load the independent scheduler period");
    }

    void testProbeCompilesAsBackendOwnedTriggeredMove() {
        constexpr double ACCELERATION = 2.0;
        constexpr double JERK = 10.0;
        ngc::ExactStopTrajectoryPlanner planner({
            .pathAcceleration = ACCELERATION,
            .rapidSpeed = 100.0,
            .arcChordTolerance = 0.0001,
            .pathJerk = JERK,
        });
        planner.reset(3, {});
        const ngc::ProbeMove probeCommand {
            77, {}, { 0, 0, -1, 0, 0, 0 }, 10.0, true, false };
        const auto move = planner.compileTriggeredMove(probeCommand);
        require(move.has_value(), move ? "" : move.error());
        require(move->moveId == 77, "triggered move should retain the interpreter probe ID");
        requireNear(move->target.z, -1.0, "triggered move should retain its machine target");

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

    void testTrajectoryDriverConnectsInterpreterToMockRtBackend() {
        ngc::InterpreterSession session(UNIT, ngc::InterpretationMode::Simulation);
        compileSession(session, "G1 F60 X1\nG1 Z-1\nG1 X2\n");
        session.begin();

        ngc::MockMotionBackend backend;
        ngc::TrajectoryExecutionDriver driver(session, backend, {
            .pathAcceleration = 10.0,
            .rapidSpeed = 100.0,
            .arcChordTolerance = 0.0001,
        });
        require(driver.begin(12), "trajectory driver should initialize bounded backend control channels");

        int observedCommands = 0;
        ngc::ExecutionSnapshot latest;
        bool haveSnapshot = false;
        for(int guard = 0; guard < 10000 && driver.state() == ngc::TrajectoryDriverState::Running; ++guard) {
            for(int fill = 0; fill < 64; ++fill) {
                if(!driver.pumpOne([](const auto &callback) { callback(); },
                                   [&](const ngc::MachineCommand &) { ++observedCommands; })) break;
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
        require(haveSnapshot, "mock RT backend should expose execution snapshots through SPSC communication");
        requireNear(latest.commanded.position.x, 2.0, "mock RT execution should reach the final planned X position");
        requireNear(latest.commanded.position.z, -1.0,
                    "mock RT execution should preserve its final Z position");
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
        testFeedMotionRequiresFeedrate();
        testUnsupportedCodesProduceInterpreterErrors();
        testFailedBlockRollsBackMachineState();
        testInterpreterCancellationInterruptsEvaluation();
        testSimulationWorkerStartsPlayback();
        testAdaptivePocketsStartsSimulation();
        testMdiToolChangeUsesAutoloadPrograms();
        testSimulationWorkerPersistsUntilReset();
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
        testMockBackendAdvancesOneFixedServoTick();
        testMockBackendDrainsAFullPlanHorizonWithoutEventOverflow();
        testImmediateDrainStopsAtHeldWithStaleDescendants();
        testN70N75PreviewPrefixesCompleteBoundedly();
        testExactStopPlannerCompilesLinesAndArcs();
        testVerifiedCubicArcSpanCounts();
        testPlannedArcsPreserveCanonicalEndpointContinuity();
        testRoundedRadiusArcPreservesDynamicLimits();
        testEndpointExactArcReferenceGeometryVariants();
        testMockDiagnosticPositionsFollowServoPeriod();
        testMachineConfigurationLoadsTrajectoryLimits();
        testProbeCompilesAsBackendOwnedTriggeredMove();
        testTrajectoryDriverConnectsInterpreterToMockRtBackend();
    } catch(const std::exception &error) {
        std::cerr << "ngc_tests failed: " << error.what() << '\n';
        return 1;
    }

    std::cout << "ngc_tests passed\n";
    return 0;
}
