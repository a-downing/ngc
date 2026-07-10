#include <array>
#include <cmath>
#include <exception>
#include <format>
#include <functional>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

#include "evaluator/Evaluator.h"
#include "evaluator/Preamble.h"
#include "machine/Machine.h"
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

    void execute(ngc::Machine &machine, const std::string_view source) {
        ngc::Program program(std::string(source), "test.ngc");
        const auto compiled = program.compile();
        require(compiled.has_value(), compiled ? "" : compiled.error().text());

        const std::function callback = [&](std::unique_ptr<const ngc::EvaluatorMessage> message, ngc::Evaluator &) {
            if(const auto block = message->as<ngc::BlockMessage>()) {
                machine.executeBlock(block->block());
            }
        };

        ngc::Evaluator evaluator(machine.memory(), callback);
        const ngc::Preamble preamble(machine.memory());
        evaluator.executeFirstPass(preamble.statements());
        evaluator.executeFirstPass(program.statements());
        evaluator.executeSecondPass(program.statements());
    }

    std::vector<std::unique_ptr<ngc::MachineCommand>> run(const std::string_view source) {
        ngc::Machine machine(UNIT);
        execute(machine, source);
        return std::move(machine.commands());
    }

    void testRapidAndFeedMove() {
        const auto commands = run("G0 X10\nG1 F120 X20\n");
        require(commands.size() == 2, "expected two line moves");

        const auto *rapid = commands[0]->as<ngc::MoveLine>();
        const auto *feed = commands[1]->as<ngc::MoveLine>();
        require(rapid != nullptr && feed != nullptr, "expected line moves");
        requireNear(rapid->speed(), -1.0, "G0 must retain the rapid sentinel");
        requireNear(rapid->to().x, 10.0, "G0 endpoint is incorrect");
        requireNear(feed->speed(), 120.0, "G1 must use the programmed feedrate");
        requireNear(feed->from().x, 10.0, "G1 start point is incorrect");
        requireNear(feed->to().x, 20.0, "G1 endpoint is incorrect");
    }

    void testArcUsesModalFeedrate() {
        const auto commands = run("G1 F120 X10 Y0\nG3 X0 Y10 I-10 J0\n");
        require(commands.size() == 2, "expected one line and one arc");

        const auto *arc = commands[1]->as<ngc::MoveArc>();
        require(arc != nullptr, "expected an arc move");
        requireNear(arc->speed(), 120.0, "G2/G3 must use the modal feedrate");
        requireNear(arc->center().x, 0.0, "arc center X is incorrect");
        requireNear(arc->center().y, 0.0, "arc center Y is incorrect");
        requireNear(arc->axis().z, 1.0, "G3 must have a positive G17 axis");
        requireNear(arc->to().x, 0.0, "arc endpoint X is incorrect");
        requireNear(arc->to().y, 10.0, "arc endpoint Y is incorrect");
    }

    void testIncrementalMove() {
        const auto commands = run("G0 X10\nG91 G1 F25 X5\n");
        require(commands.size() == 2, "expected two line moves");

        const auto *move = commands[1]->as<ngc::MoveLine>();
        require(move != nullptr, "expected an incremental line move");
        requireNear(move->from().x, 10.0, "incremental move start is incorrect");
        requireNear(move->to().x, 15.0, "incremental move endpoint is incorrect");
        requireNear(move->speed(), 25.0, "incremental move feedrate is incorrect");
    }

    void testBeginProgramRunResetsRuntimeState() {
        ngc::Machine machine(UNIT);
        machine.memory().write(ngc::Var::G54_X, 7.0);

        execute(machine, "G1 F10 X3\n");
        require(machine.commands().size() == 1, "expected one command in the first run");

        machine.beginProgramRun();
        require(machine.commands().empty(), "beginProgramRun must clear generated commands");
        require(machine.state().modeMotion == ngc::GCMotion::G0, "beginProgramRun must restore G0");
        requireNear(machine.memory().read(ngc::Var::G54_X), 7.0, "beginProgramRun must retain coordinate offsets");

        execute(machine, "G1 F10 X3\n");
        require(machine.commands().size() == 1, "second run must not retain commands from the first run");

        const auto *move = machine.commands().front()->as<ngc::MoveLine>();
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
                "{}\nG10 L2 P1 X{} Y{} Z{}\nG1 F{} X{} Y{} Z{}\nG17 G3 X0 Y{} I-{} J0\nG18 G2 X{} Z0 I0 K-{}\n",
                gcodeUnit,
                sourceValue, sourceValue, sourceValue,
                sourceValue, sourceValue, sourceValue, sourceValue,
                sourceValue, sourceValue,
                sourceValue, sourceValue);
            execute(machine, source);

            requireNear(machine.memory().read(ngc::Var::G54_X), expectedValue, "G10 X conversion is incorrect");
            requireNear(machine.memory().read(ngc::Var::G54_Y), expectedValue, "G10 Y conversion is incorrect");
            requireNear(machine.memory().read(ngc::Var::G54_Z), expectedValue, "G10 Z conversion is incorrect");
            require(machine.commands().size() == 3, "expected a line and two arcs");

            const auto *line = machine.commands()[0]->as<ngc::MoveLine>();
            const auto *g17Arc = machine.commands()[1]->as<ngc::MoveArc>();
            const auto *g18Arc = machine.commands()[2]->as<ngc::MoveArc>();
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
            execute(machine, std::format(
                "{}\nG43 H1\nG1 F{} X{} Z{}\nG49\nG1 X{} Z{}\n",
                gcodeUnit, sourceValue, sourceValue, sourceValue, sourceValue, sourceValue));

            require(machine.commands().size() == 2, "expected one G43 move and one G49 move");
            const auto *compensated = machine.commands()[0]->as<ngc::MoveLine>();
            const auto *cancelled = machine.commands()[1]->as<ngc::MoveLine>();
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
        execute(machine, "T2\nG43\nG1 F1 X1 Z1\n");

        require(machine.commands().size() == 1, "expected one compensated move");
        const auto *move = machine.commands().front()->as<ngc::MoveLine>();
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

        execute(machine, "G43 H1\nG1 F1 X1 Y1 Z1\n");

        require(machine.commands().size() == 1, "expected one compensated G54 move");
        const auto *move = machine.commands().front()->as<ngc::MoveLine>();
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

        execute(machine, "G43 H1\nG53 G1 F1 X1 Y1 Z1\n");

        require(machine.commands().size() == 1, "expected one compensated G53 move");
        const auto *move = machine.commands().front()->as<ngc::MoveLine>();
        require(move != nullptr, "expected a line move");
        requireNear(move->to().x, 3.0, "G53 must bypass G54 X while retaining the tool offset");
        requireNear(move->to().y, 4.0, "G53 must bypass G54 Y while retaining the tool offset");
        requireNear(move->to().z, 5.0, "G53 must bypass G54 Z while retaining the tool offset");
    }
}

int main() {
    try {
        testRapidAndFeedMove();
        testArcUsesModalFeedrate();
        testIncrementalMove();
        testBeginProgramRunResetsRuntimeState();
        testLinearUnitConversion();
        testToolLengthCompensation();
        testToolLengthCompensationUsesActiveTool();
        testToolLengthCompensationWithWorkOffset();
        testG53BypassesWorkOffsetButRetainsToolOffset();
    } catch(const std::exception &error) {
        std::cerr << "ngc_tests failed: " << error.what() << '\n';
        return 1;
    }

    std::cout << "ngc_tests passed\n";
    return 0;
}
