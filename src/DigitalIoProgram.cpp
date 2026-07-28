#include "machine/DigitalIoProgram.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <format>
#include <limits>
#include <sstream>
#include <vector>

namespace ngc {
    namespace {
        struct ParsedLine {
            std::size_t number = 0;
            std::vector<std::string> tokens;
        };

        std::expected<std::uint16_t, std::string> parseIndex(
            const std::string_view text,
            const std::string_view prefix,
            const std::size_t capacity,
            const std::size_t line) {
            if (!text.starts_with(prefix)
                || text.size() == prefix.size()) {
                return std::unexpected(std::format(
                    "digital I/O program line {} has invalid operand '{}'",
                    line, text));
            }

            auto value = std::uint32_t{0};
            const auto digits = text.substr(prefix.size());
            const auto [end, error] = std::from_chars(
                digits.data(), digits.data() + digits.size(), value);
            if (error != std::errc{}
                || end != digits.data() + digits.size()
                || value >= capacity) {
                return std::unexpected(std::format(
                    "digital I/O program line {} operand '{}' is out of range",
                    line, text));
            }

            return static_cast<std::uint16_t>(value);
        }

        std::expected<double, std::string> parseDuration(
            const std::string_view text,
            const std::size_t line) {
            constexpr std::array units{
                std::pair{std::string_view{"ns"}, 1e-9},
                std::pair{std::string_view{"us"}, 1e-6},
                std::pair{std::string_view{"ms"}, 1e-3},
                std::pair{std::string_view{"s"}, 1.0},
            };
            for (const auto &[suffix, scale] : units) {
                if (!text.ends_with(suffix)
                    || text.size() == suffix.size()) {
                    continue;
                }

                const auto number =
                    text.substr(0, text.size() - suffix.size());
                auto value = 0.0;
                const auto [end, error] = std::from_chars(
                    number.data(), number.data() + number.size(), value);
                if (error != std::errc{}
                    || end != number.data() + number.size()
                    || !std::isfinite(value) || value < 0.0) {
                    break;
                }

                return value * scale;
            }

            return std::unexpected(std::format(
                "digital I/O program line {} has invalid duration '{}'",
                line, text));
        }

        std::vector<ParsedLine> tokenize(
            const std::string_view source) {
            std::vector<ParsedLine> result;
            std::istringstream stream{std::string(source)};
            std::string line;
            auto lineNumber = std::size_t{0};
            while (std::getline(stream, line)) {
                ++lineNumber;
                if (const auto comment = line.find_first_of("#;");
                    comment != std::string::npos) {
                    line.resize(comment);
                }
                std::ranges::replace(line, ',', ' ');

                ParsedLine parsed{
                    .number = lineNumber,
                    .tokens = {},
                };
                std::istringstream words{line};
                std::string word;
                while (words >> word) {
                    parsed.tokens.push_back(std::move(word));
                }
                if (!parsed.tokens.empty()) {
                    result.push_back(std::move(parsed));
                }
            }

            return result;
        }
    }

    std::expected<DigitalIoProgram, std::string>
    DigitalIoProgram::compile(
        const std::string_view source,
        const std::size_t fieldInputCount,
        const std::span<const DigitalInputId> logicalInputs,
        const std::size_t fieldOutputCount,
        const std::span<const DigitalOutputId> logicalOutputs,
        const double servoPeriod) {
        if (fieldInputCount
            > DIGITAL_IO_PROGRAM_FIELD_INPUT_CAPACITY) {
            return std::unexpected(
                "digital I/O program field-input count exceeds capacity");
        }
        if (fieldOutputCount
            > DIGITAL_IO_PROGRAM_FIELD_OUTPUT_CAPACITY) {
            return std::unexpected(
                "digital I/O program field-output count exceeds capacity");
        }
        if (!std::isfinite(servoPeriod) || servoPeriod <= 0.0) {
            return std::unexpected(
                "digital I/O program servo period must be positive");
        }

        auto declaredInputs =
            std::bitset<LOGICAL_DIGITAL_INPUT_CAPACITY>{};
        for (const auto input : logicalInputs) {
            if (declaredInputs[input]) {
                return std::unexpected(std::format(
                    "digital I/O program declares logical input {} more than once",
                    input));
            }
            declaredInputs[input] = true;
        }
        auto declaredOutputs =
            std::bitset<LOGICAL_DIGITAL_OUTPUT_CAPACITY>{};
        for (const auto output : logicalOutputs) {
            if (declaredOutputs[output]) {
                return std::unexpected(std::format(
                    "digital I/O program declares logical output {} more than once",
                    output));
            }
            declaredOutputs[output] = true;
        }

        auto result = DigitalIoProgram{};
        result.m_fieldInputCount = fieldInputCount;
        result.m_logicalInputCount = logicalInputs.size();
        result.m_fieldOutputCount = fieldOutputCount;
        result.m_logicalOutputCount = logicalOutputs.size();
        auto assignedRegisters =
            std::bitset<DIGITAL_IO_PROGRAM_REGISTER_CAPACITY>{};
        auto assignedInputs =
            std::bitset<LOGICAL_DIGITAL_INPUT_CAPACITY>{};
        auto assignedFieldOutputs =
            std::bitset<DIGITAL_IO_PROGRAM_FIELD_OUTPUT_CAPACITY>{};

        const auto lines = tokenize(source);
        if (lines.size()
            > DIGITAL_IO_PROGRAM_INSTRUCTION_CAPACITY) {
            return std::unexpected(
                "digital I/O program instruction count exceeds capacity");
        }

        const auto parseOperand = [&](const std::string_view text,
                                      const std::size_t line)
            -> std::expected<Operand, std::string> {
            if (text == "0" || text == "1") {
                return Operand{
                    .kind = OperandKind::Constant,
                    .constant = text == "1",
                };
            }
            if (text.starts_with("fieldin")) {
                const auto index = parseIndex(
                    text, "fieldin", fieldInputCount, line);
                if (!index) {
                    return std::unexpected(index.error());
                }

                return Operand{
                    .kind = OperandKind::FieldInput,
                    .index = *index,
                };
            }
            if (text.starts_with("fieldout")) {
                const auto index = parseIndex(
                    text, "fieldout", fieldOutputCount, line);
                if (!index) {
                    return std::unexpected(index.error());
                }

                return Operand{
                    .kind = OperandKind::FieldOutput,
                    .index = *index,
                };
            }
            if (text.starts_with("in")) {
                const auto index = parseIndex(
                    text, "in", LOGICAL_DIGITAL_INPUT_CAPACITY,
                    line);
                if (!index) {
                    return std::unexpected(index.error());
                }

                return Operand{
                    .kind = OperandKind::LogicalInput,
                    .index = *index,
                };
            }
            if (text.starts_with("out")) {
                const auto index = parseIndex(
                    text, "out", LOGICAL_DIGITAL_OUTPUT_CAPACITY,
                    line);
                if (!index) {
                    return std::unexpected(index.error());
                }

                return Operand{
                    .kind = OperandKind::LogicalOutput,
                    .index = *index,
                };
            }

            const auto index = parseIndex(
                text, "r", DIGITAL_IO_PROGRAM_REGISTER_CAPACITY,
                line);
            if (!index) {
                return std::unexpected(index.error());
            }

            return Operand{
                .kind = OperandKind::Register,
                .index = *index,
            };
        };

        const auto requireSource = [&](
            const Operand &operand,
            const std::size_t line)
            -> std::expected<void, std::string> {
            if (operand.kind == OperandKind::LogicalInput
                || operand.kind == OperandKind::FieldOutput) {
                return std::unexpected(std::format(
                    "digital I/O program line {} reads a write-only operand",
                    line));
            }
            if (operand.kind == OperandKind::LogicalOutput
                && !declaredOutputs[operand.index]) {
                return std::unexpected(std::format(
                    "digital I/O program line {} reads undeclared logical output {}",
                    line, operand.index));
            }
            if (operand.kind == OperandKind::Register
                && !assignedRegisters[operand.index]) {
                return std::unexpected(std::format(
                    "digital I/O program line {} reads uninitialized register r{}",
                    line, operand.index));
            }

            return {};
        };

        const auto assignDestination = [&](
            const Operand &operand,
            const bool logicalAllowed,
            const std::size_t line)
            -> std::expected<void, std::string> {
            if (operand.kind == OperandKind::Register) {
                assignedRegisters[operand.index] = true;

                return {};
            }
            if (!logicalAllowed
                || (operand.kind != OperandKind::LogicalInput
                    && operand.kind != OperandKind::FieldOutput)) {
                return std::unexpected(std::format(
                    "digital I/O program line {} has an invalid destination",
                    line));
            }
            if (operand.kind == OperandKind::LogicalInput) {
                if (!declaredInputs[operand.index]) {
                    return std::unexpected(std::format(
                        "digital I/O program line {} writes undeclared logical input {}",
                        line, operand.index));
                }
                if (assignedInputs[operand.index]) {
                    return std::unexpected(std::format(
                        "digital I/O program line {} writes logical input {} more than once",
                        line, operand.index));
                }
                assignedInputs[operand.index] = true;

                return {};
            }
            if (assignedFieldOutputs[operand.index]) {
                return std::unexpected(std::format(
                    "digital I/O program line {} writes field output {} more than once",
                    line, operand.index));
            }
            assignedFieldOutputs[operand.index] = true;

            return {};
        };

        for (const auto &line : lines) {
            const auto &tokens = line.tokens;
            auto instruction = Instruction{};
            auto expectedTokens = std::size_t{0};
            if (tokens[0] == "mov") {
                instruction.opcode = Opcode::Move;
                expectedTokens = 3;
            } else if (tokens[0] == "not") {
                instruction.opcode = Opcode::Not;
                expectedTokens = 3;
            } else if (tokens[0] == "and") {
                instruction.opcode = Opcode::And;
                expectedTokens = 4;
            } else if (tokens[0] == "or") {
                instruction.opcode = Opcode::Or;
                expectedTokens = 4;
            } else if (tokens[0] == "xor") {
                instruction.opcode = Opcode::Xor;
                expectedTokens = 4;
            } else if (tokens[0] == "debounce") {
                instruction.opcode = Opcode::Debounce;
                expectedTokens = 4;
            } else {
                return std::unexpected(std::format(
                    "digital I/O program line {} has unknown opcode '{}'",
                    line.number, tokens[0]));
            }
            if (tokens.size() != expectedTokens) {
                return std::unexpected(std::format(
                    "digital I/O program line {} opcode '{}' expects {} operands",
                    line.number, tokens[0], expectedTokens - 1));
            }

            const auto destination =
                parseOperand(tokens[1], line.number);
            const auto first =
                parseOperand(tokens[2], line.number);
            if (!destination) {
                return std::unexpected(destination.error());
            }
            if (!first) {
                return std::unexpected(first.error());
            }
            instruction.destination = *destination;
            instruction.first = *first;
            if (const auto valid = requireSource(
                    instruction.first, line.number);
                !valid) {
                return std::unexpected(valid.error());
            }

            if (instruction.opcode == Opcode::And
                || instruction.opcode == Opcode::Or
                || instruction.opcode == Opcode::Xor) {
                const auto second =
                    parseOperand(tokens[3], line.number);
                if (!second) {
                    return std::unexpected(second.error());
                }
                instruction.second = *second;
                if (const auto valid = requireSource(
                        instruction.second, line.number);
                    !valid) {
                    return std::unexpected(valid.error());
                }
            }
            if (instruction.opcode == Opcode::Debounce) {
                const auto duration =
                    parseDuration(tokens[3], line.number);
                if (!duration) {
                    return std::unexpected(duration.error());
                }
                const auto ticks = *duration == 0.0
                    ? 0.0
                    : std::max(
                        1.0,
                        std::ceil(
                            *duration / servoPeriod - 1e-12));
                if (ticks
                    > static_cast<double>(
                        std::numeric_limits<std::uint32_t>::max())) {
                    return std::unexpected(std::format(
                        "digital I/O program line {} debounce duration exceeds capacity",
                        line.number));
                }
                instruction.debounceTicks =
                    static_cast<std::uint32_t>(ticks);
            }

            const auto logicalDestinationAllowed =
                instruction.opcode == Opcode::Move;
            if (const auto valid = assignDestination(
                    instruction.destination,
                    logicalDestinationAllowed, line.number);
                !valid) {
                return std::unexpected(valid.error());
            }

            result.m_instructions[result.m_instructionCount++] =
                instruction;
        }

        for (const auto input : logicalInputs) {
            if (!assignedInputs[input]) {
                return std::unexpected(std::format(
                    "digital I/O program does not write logical input {}",
                    input));
            }
        }
        for (std::size_t output = 0;
             output < fieldOutputCount; ++output) {
            if (!assignedFieldOutputs[output]) {
                return std::unexpected(std::format(
                    "digital I/O program does not write field output {}",
                    output));
            }
        }

        return result;
    }

    bool DigitalIoProgram::value(
        const Operand &operand,
        const FieldDigitalInputImage &fieldInputs,
        const LogicalDigitalOutputImage &logicalOutputs,
        const std::bitset<
            DIGITAL_IO_PROGRAM_REGISTER_CAPACITY> &registers) const noexcept {
        switch (operand.kind) {
            case OperandKind::Register:
                return registers[operand.index];
            case OperandKind::FieldInput:
                return fieldInputs[operand.index];
            case OperandKind::LogicalOutput:
                return logicalOutputs[operand.index];
            case OperandKind::Constant:
                return operand.constant;
            case OperandKind::LogicalInput:
            case OperandKind::FieldOutput:
                return false;
        }

        return false;
    }

    void DigitalIoProgram::evaluate(
        const FieldDigitalInputImage &fieldInputs,
        const LogicalDigitalOutputImage &logicalOutputs,
        LogicalDigitalInputImage &logicalInputs,
        FieldDigitalOutputImage &fieldOutputs,
        std::array<
            DebounceState,
            DIGITAL_IO_PROGRAM_INSTRUCTION_CAPACITY> &debounce,
        const bool advanceDebounce) const noexcept {
        auto registers =
            std::bitset<DIGITAL_IO_PROGRAM_REGISTER_CAPACITY>{};
        auto nextInputs = LogicalDigitalInputImage{};
        auto nextFieldOutputs = FieldDigitalOutputImage{};
        for (std::size_t index = 0;
             index < m_instructionCount; ++index) {
            const auto &instruction = m_instructions[index];
            const auto first =
                value(
                    instruction.first, fieldInputs,
                    logicalOutputs, registers);
            auto result = false;
            switch (instruction.opcode) {
                case Opcode::Move:
                    result = first;
                    break;
                case Opcode::Not:
                    result = !first;
                    break;
                case Opcode::And:
                    result = first && value(
                        instruction.second, fieldInputs,
                        logicalOutputs, registers);
                    break;
                case Opcode::Or:
                    result = first || value(
                        instruction.second, fieldInputs,
                        logicalOutputs, registers);
                    break;
                case Opcode::Xor:
                    result = first != value(
                        instruction.second, fieldInputs,
                        logicalOutputs, registers);
                    break;
                case Opcode::Debounce: {
                    auto &state = debounce[index];
                    if (!advanceDebounce) {
                        result = state.output;
                        break;
                    }
                    if (!state.initialized) {
                        state.initialized = true;
                        state.output = first;
                        state.candidate = first;
                        state.stableTicks = 0;
                    } else if (instruction.debounceTicks == 0
                               || first == state.output) {
                        state.output = first;
                        state.candidate = first;
                        state.stableTicks = 0;
                    } else {
                        if (first != state.candidate) {
                            state.candidate = first;
                            state.stableTicks = 1;
                        } else if (
                            state.stableTicks
                            < instruction.debounceTicks) {
                            ++state.stableTicks;
                        }
                        if (state.stableTicks
                            >= instruction.debounceTicks) {
                            state.output = state.candidate;
                            state.stableTicks = 0;
                        }
                    }
                    result = state.output;
                    break;
                }
            }

            if (instruction.destination.kind
                == OperandKind::Register) {
                registers[instruction.destination.index] = result;
            } else if (instruction.destination.kind
                       == OperandKind::LogicalInput) {
                nextInputs[instruction.destination.index] = result;
            } else {
                nextFieldOutputs[instruction.destination.index] = result;
            }
        }

        logicalInputs = nextInputs;
        fieldOutputs = nextFieldOutputs;
    }

    void DigitalIoProgram::executeInputs(
        const FieldDigitalInputImage &fieldInputs,
        const LogicalDigitalOutputImage &logicalOutputs,
        LogicalDigitalInputImage &logicalInputs) noexcept {
        auto unusedFieldOutputs = FieldDigitalOutputImage{};
        evaluate(
            fieldInputs, logicalOutputs, logicalInputs,
            unusedFieldOutputs, m_debounce, true);
    }

    void DigitalIoProgram::executeOutputs(
        const FieldDigitalInputImage &fieldInputs,
        const LogicalDigitalOutputImage &logicalOutputs,
        FieldDigitalOutputImage &fieldOutputs) const noexcept {
        auto unusedInputs = LogicalDigitalInputImage{};
        auto debounce = m_debounce;
        evaluate(
            fieldInputs, logicalOutputs, unusedInputs,
            fieldOutputs, debounce, false);
    }

    void DigitalIoProgram::reset() noexcept {
        m_debounce = {};
    }

    std::size_t DigitalIoProgram::instructionCount() const noexcept {
        return m_instructionCount;
    }

    std::size_t DigitalIoProgram::fieldInputCount() const noexcept {
        return m_fieldInputCount;
    }

    std::size_t DigitalIoProgram::logicalInputCount() const noexcept {
        return m_logicalInputCount;
    }

    std::size_t DigitalIoProgram::fieldOutputCount() const noexcept {
        return m_fieldOutputCount;
    }

    std::size_t DigitalIoProgram::logicalOutputCount() const noexcept {
        return m_logicalOutputCount;
    }
}
