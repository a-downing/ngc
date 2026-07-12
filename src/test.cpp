#include <array>
#include <cmath>
#include <exception>
#include <format>
#include <fstream>
#include <filesystem>
#include <functional>
#include <iostream>
#include <iterator>
#include <string>
#include <string_view>
#include <vector>

#include "evaluator/Evaluator.h"
#include "evaluator/InterpreterSession.h"
#include "evaluator/Preamble.h"
#include "machine/Machine.h"
#include "machine/ExecutionDriver.h"
#include "machine/SimulationExecutor.h"
#include "machine/ToolpathRecorder.h"
#include "machine/ToolTable.h"
#include "memory/Memory.h"
#include "parser/Program.h"

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
        const auto loaded = table.load(path);
        std::filesystem::remove(path);
        require(!loaded.has_value(), "duplicate tool numbers should fail to load");
        require(loaded.error().find("row:2 duplicate tool number 7") != std::string::npos,
                "duplicate tool error should identify its row and tool number");
    }

    void testNumericParsingRejectsTrailingGarbage() {
        require(ngc::fromChars("12.5").has_value(), "a complete number should parse");
        require(!ngc::fromChars("12.5xyz").has_value(), "trailing garbage should make a number invalid");
        require(!ngc::fromChars("").has_value(), "an empty number should be invalid");
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
        const auto content = ngc::readFile(contentPath);
        require(content && *content == "abc\n123", "file helpers should preserve exact binary content");
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

        ngc::SimulationExecutor executor;
        ngc::ExecutionDriver driver(session, executor);
        ngc::ToolpathRecorder recorder;
        while(driver.state() == ngc::ExecutionDriverState::Running) {
            driver.pumpOne([](const auto &callback) { callback(); },
                [&](const ngc::MachineCommand &command, const ngc::position_t &toolOffset,
                    const ngc::ToolGeometry &, const ngc::WorkCoordinateSystem &workCoordinateSystem) {
                    recorder.consume(command, toolOffset, workCoordinateSystem);
                });
            executor.completeQueued();
            driver.deliverProbeResult();
        }

        require(driver.state() == ngc::ExecutionDriverState::Completed,
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

    void testSimulationExecutorInterpolatesAndAppliesToolOffset() {
        ngc::SimulationExecutor simulator;
        simulator.reset();
        simulator.advance(1.0);
        require(simulator.empty(), "advancing an empty simulator should be a no-op");
        simulator.setStatus(ngc::SimulationStatus::Running);
        simulator.consume({
            ngc::MoveLine { { 0, 0, 0, 0, 0, 0 }, { 2, 0, 0, 0, 0, 0 }, 60.0 },
            { 0, 0, 0.5, 0, 0, 0 },
            { .number = 1, .offset = { 0, 0, 0.5, 0, 0, 0 }, .diameter = 0.25 },
            ngc::WorkCoordinateSystem { "G54", { 1, 2, 3, 0, 0, 0 } },
            { "G1", "G17", "G90", "G94", "G20", "G54" },
            ngc::BlockExecution { 1, "G1 F60 X2", "test.ngc", 1 },
        });

        simulator.advance(1.0);
        const auto halfway = simulator.snapshot();
        requireNear(halfway.machinePosition.x, 1.0, "simulation should interpolate feed motion using elapsed time");
        requireNear(halfway.commandProgress, 0.5, "simulation should report active command progress");
        requireNear(halfway.toolPosition.z, -0.5, "simulation should apply the captured tool offset to its display position");
        require(halfway.activeWorkCoordinateSystem && halfway.activeWorkCoordinateSystem->name == "G54",
                "simulation should expose the WCS captured with its active command");
        require(halfway.activeModalGCodes.front() == "G1",
                "simulation should expose modal G-codes captured with its active command");

        simulator.advance(1.0);
        const auto finished = simulator.snapshot();
        requireNear(finished.machinePosition.x, 2.0, "simulation should finish at the commanded endpoint");
        require(simulator.empty(), "completed simulated motion should leave the executor empty");
    }

    void testSimulationExecutorCompletesProbeAtTarget() {
        ngc::SimulationExecutor simulator;
        simulator.reset();
        simulator.consume({ ngc::ProbeMove { 42, { 0, 0, 0, 0, 0, 0 }, { 0, 0, -1, 0, 0, 0 }, 60, true, false }, {} });
        simulator.advance(1.0);

        const auto result = simulator.takeProbeResult();
        require(result.has_value(), "completed simulated probe should produce a result");
        require(result->id == 42, "simulated probe result should match its command");
        require(result->status == ngc::ProbeStatus::Triggered, "simulated G38.3 should trigger at its target");
        requireNear(result->triggerPosition.z, -1.0, "simulated probe trigger should be the requested target");
        requireNear(result->stoppedPosition.z, -1.0, "simulated probe should stop at the requested target");
    }

    void testSimulationProbeUsesPhysicalToolLengthWithoutG43() {
        ngc::SimulationExecutor simulator;
        simulator.reset();
        simulator.consume({
            ngc::ProbeMove { 7, { 0, 0, 4, 0, 0, 0 }, { 0, 0, 0, 0, 0, 0 }, 60, true, false },
            {},
            { .number = 7, .offset = { 0, 0, 2, 0, 0, 0 }, .diameter = 0.5 },
        });
        simulator.advance(2.0);

        const auto snapshot = simulator.snapshot();
        const auto result = simulator.takeProbeResult();
        require(result.has_value(), "physical tool probe should reach its simulated contact");
        requireNear(result->stoppedPosition.z, 2.0, "spindle should stop one tool length above the probe target");
        requireNear(snapshot.toolPosition.z, 0.0, "physical tool tip should stop at the probe target");

        simulator.consume({
            ngc::MoveLine { { 0, 0, 0, 0, 0, 0 }, { 0, 0, 4, 0, 0, 0 }, 60.0 },
            {},
            { .number = 7, .offset = { 0, 0, 2, 0, 0, 0 }, .diameter = 0.5 },
        });
        simulator.advance(1.0);
        requireNear(simulator.snapshot().machinePosition.z, 3.0,
                    "post-probe travel should start at the actual stopped position");
        requireNear(simulator.snapshot().toolPosition.z, 1.0,
                    "post-probe tool-tip travel should retract away from the probe");
    }

    void testImmediateExecutionDriverUsesSimulationProbePath() {
        ngc::InterpreterSession session(UNIT, ngc::InterpretationMode::Preview);
        ngc::SimulationExecutor executor;
        ngc::ExecutionDriver driver(session, executor);
        compileSession(session, "G0 Z4\nG38.3 F60 Z0\nG91 G0 Z1\n");
        session.begin();
        session.machine().prepareToolChange(7);
        executor.reset();
        driver.reset();

        int probeCount = 0;
        while(driver.state() == ngc::ExecutionDriverState::Running) {
            driver.pumpOne([](const auto &callback) { callback(); },
                [&](const ngc::MachineCommand &command, const ngc::position_t &, const ngc::ToolGeometry &,
                    const ngc::WorkCoordinateSystem &) {
                    if(std::holds_alternative<ngc::ProbeMove>(command)) probeCount++;
                });
            executor.completeQueued();
            driver.deliverProbeResult();
        }

        require(driver.state() == ngc::ExecutionDriverState::Completed,
                "immediate execution driver should complete the preview");
        require(probeCount == 1, "immediate execution should use the shared probe path");
        const auto completedBlocks = executor.snapshot().completedBlocks;
        require(std::ranges::find_if(completedBlocks, [](const ngc::BlockExecution &block) {
                    return block.text == "G91 G0 Z1";
                }) != completedBlocks.end(),
                "execution metadata should retain every interpreted G-code block");
        requireNear(session.machine().memory().read(ngc::Var::TASK), 0.0,
                    "immediate preview should retain the preview _task value");
    }
}

int main() {
    try {
        testMemoryStackBounds();
        testToolTableLoadsFinalLineWithoutNewline();
        testToolTableRejectsDuplicateToolNumbers();
        testNumericParsingRejectsTrailingGarbage();
        testFileHelpersHandleEmptyAndFailedIo();
        testRapidAndFeedMove();
        testFeedMotionRequiresFeedrate();
        testUnsupportedCodesProduceInterpreterErrors();
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
        testInterpreterSessionOwnsCompilationAndExecution();
        testInterpreterTaskVariable();
        testIncrementalSessionControlFlow();
        testProbeCommandAndBarrier();
        testAutomaticToolChangeReachesProbeBarrier();
        testToolpathRecorderIsASeparateConsumer();
        testToolpathRecorderRetainsUsedWorkCoordinateSystems();
        testToolChangePreviewRetainsNestedAndFinalWorkCoordinateSystems();
        testToolpathRecorderAppliesPerCommandToolOffset();
        testSimulationExecutorInterpolatesAndAppliesToolOffset();
        testSimulationExecutorCompletesProbeAtTarget();
        testSimulationProbeUsesPhysicalToolLengthWithoutG43();
        testImmediateExecutionDriverUsesSimulationProbePath();
    } catch(const std::exception &error) {
        std::cerr << "ngc_tests failed: " << error.what() << '\n';
        return 1;
    }

    std::cout << "ngc_tests passed\n";
    return 0;
}
