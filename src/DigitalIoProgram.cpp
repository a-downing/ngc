#include "machine/DigitalIoProgram.h"

#include <algorithm>
#include <charconv>
#include <cctype>
#include <cmath>
#include <format>
#include <limits>
#include <optional>
#include <sstream>
#include <vector>

#include "machine/ProductionExecutorCore.h"

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

        std::optional<double> parseNumber(
            const std::string_view text) noexcept {
            auto value = 0.0;
            const auto [end, error] = std::from_chars(
                text.data(), text.data() + text.size(), value);
            if (error != std::errc{}
                || end != text.data() + text.size()
                || !std::isfinite(value)) {
                return std::nullopt;
            }

            return value;
        }

        double axisComponent(
            const position_t &position,
            const std::size_t axis) noexcept {
            switch (static_cast<AxisId>(axis)) {
                case AxisId::X: return position.x;
                case AxisId::Y: return position.y;
                case AxisId::Z: return position.z;
                case AxisId::A: return position.a;
                case AxisId::B: return position.b;
                case AxisId::C: return position.c;
            }

            return 0.0;
        }

        bool booleanValue(const double value) noexcept {
            return std::isfinite(value) && value != 0.0;
        }

        bool testMask(
            const double value, const double mask) noexcept {
            constexpr auto maximumExactInteger =
                std::uint64_t{1} << 53;
            if (!std::isfinite(value) || !std::isfinite(mask)
                || value < 0.0 || mask < 0.0
                || value > static_cast<double>(maximumExactInteger)
                || mask > static_cast<double>(maximumExactInteger)
                || std::trunc(value) != value
                || std::trunc(mask) != mask) {
                return false;
            }

            return (static_cast<std::uint64_t>(value)
                    & static_cast<std::uint64_t>(mask))
                != 0;
        }

        bool validSymbolName(const std::string_view name) {
            if (name.empty()
                || (!std::isalpha(
                        static_cast<unsigned char>(name.front()))
                    && name.front() != '_')) {
                return false;
            }
            return std::ranges::all_of(
                name.substr(1), [](const char character) {
                    return std::isalnum(
                        static_cast<unsigned char>(character))
                        || character == '_';
                });
        }

        bool reservedSymbolName(const std::string_view name) {
            constexpr std::array words{
                std::string_view{"0"}, std::string_view{"1"},
                std::string_view{"mov"}, std::string_view{"not"},
                std::string_view{"and"}, std::string_view{"or"},
                std::string_view{"xor"},
                std::string_view{"debounce"},
                std::string_view{"sub"},
                std::string_view{"abs"},
                std::string_view{"le"},
                std::string_view{"ge"},
                std::string_view{"test"},
                std::string_view{"motion_flags"},
                std::string_view{"move_joints"},
                std::string_view{"trigger_joints"},
                std::string_view{"IS_PROBE"},
                std::string_view{"IS_HOMING"},
            };
            if (std::ranges::contains(words, name)) {
                return true;
            }
            constexpr std::array axisPrefixes{
                std::string_view{"apos_"},
                std::string_view{"astart_"},
                std::string_view{"atarget_"},
            };
            constexpr std::string_view axisSuffixes = "xyzabc";
            if (std::ranges::any_of(
                    axisPrefixes,
                    [&](const std::string_view prefix) {
                        return name.size() == prefix.size() + 1
                            && name.starts_with(prefix)
                            && axisSuffixes.contains(name.back());
                    })) {
                return true;
            }
            constexpr std::array prefixes{
                std::string_view{"fieldin"},
                std::string_view{"fieldout"},
                std::string_view{"in"},
                std::string_view{"out"},
                std::string_view{"r"},
                std::string_view{"jpos_"},
                std::string_view{"jstart_"},
                std::string_view{"jtarget_"},
                std::string_view{"JOINT_"},
            };
            return std::ranges::any_of(
                prefixes, [&](const std::string_view prefix) {
                    const auto suffix = name.substr(
                        std::min(name.size(), prefix.size()));
                    return name.starts_with(prefix)
                        && !suffix.empty()
                        && std::ranges::all_of(
                            suffix, [](const char character) {
                                return std::isdigit(
                                    static_cast<unsigned char>(
                                        character));
                            });
                });
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
        const double servoPeriod,
        const std::span<const DigitalIoSymbol> symbols) {
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
        for (std::size_t index = 0;
             index < symbols.size(); ++index) {
            const auto &symbol = symbols[index];
            if (!validSymbolName(symbol.name)
                || reservedSymbolName(symbol.name)) {
                return std::unexpected(std::format(
                    "digital I/O program symbol '{}' is invalid or reserved",
                    symbol.name));
            }
            if (std::ranges::find(
                    symbols.first(index), symbol.name,
                    &DigitalIoSymbol::name)
                != symbols.first(index).end()) {
                return std::unexpected(std::format(
                    "digital I/O program symbol '{}' is declared more than once",
                    symbol.name));
            }
            if (symbol.kind == DigitalIoSymbolKind::LogicalInput
                && !declaredInputs[symbol.id]) {
                return std::unexpected(std::format(
                    "digital I/O program symbol '{}' references undeclared logical input {}",
                    symbol.name, symbol.id));
            }
            if (symbol.kind == DigitalIoSymbolKind::LogicalOutput
                && !declaredOutputs[symbol.id]) {
                return std::unexpected(std::format(
                    "digital I/O program symbol '{}' references undeclared logical output {}",
                    symbol.name, symbol.id));
            }
            if (symbol.kind == DigitalIoSymbolKind::FieldInput
                && symbol.id >= fieldInputCount) {
                return std::unexpected(std::format(
                    "digital I/O program symbol '{}' references unavailable field input {}",
                    symbol.name, symbol.id));
            }
            if (symbol.kind == DigitalIoSymbolKind::FieldOutput
                && symbol.id >= fieldOutputCount) {
                return std::unexpected(std::format(
                    "digital I/O program symbol '{}' references unavailable field output {}",
                    symbol.name, symbol.id));
            }
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
            if (const auto number = parseNumber(text)) {
                return Operand{
                    .kind = OperandKind::Constant,
                    .constant = *number,
                };
            }
            if (text == "IS_PROBE") {
                return Operand{
                    .kind = OperandKind::Constant,
                    .constant = static_cast<double>(
                        PRODUCTION_EXECUTOR_MOTION_IS_PROBE),
                };
            }
            if (text == "IS_HOMING") {
                return Operand{
                    .kind = OperandKind::Constant,
                    .constant = static_cast<double>(
                        PRODUCTION_EXECUTOR_MOTION_IS_HOMING),
                };
            }
            if (text == "motion_flags") {
                return Operand{.kind = OperandKind::MotionFlags};
            }
            if (text == "move_joints") {
                return Operand{.kind = OperandKind::MoveJoints};
            }
            if (text == "trigger_joints") {
                return Operand{.kind = OperandKind::TriggerJoints};
            }
            if (text.starts_with("JOINT_")) {
                const auto index = parseIndex(
                    text, "JOINT_", MAX_JOINTS, line);
                if (!index) {
                    return std::unexpected(index.error());
                }

                return Operand{
                    .kind = OperandKind::Constant,
                    .constant = static_cast<double>(
                        std::uint64_t{1} << *index),
                };
            }
            constexpr std::array axisNames{
                std::string_view{"x"}, std::string_view{"y"},
                std::string_view{"z"}, std::string_view{"a"},
                std::string_view{"b"}, std::string_view{"c"},
            };
            const auto parseAxis = [&](
                const std::string_view prefix,
                const OperandKind kind)
                -> std::optional<Operand> {
                if (!text.starts_with(prefix)) {
                    return std::nullopt;
                }
                const auto suffix = text.substr(prefix.size());
                const auto found =
                    std::ranges::find(axisNames, suffix);
                if (found == axisNames.end()) {
                    return std::nullopt;
                }

                return Operand{
                    .kind = kind,
                    .index = static_cast<std::uint16_t>(
                        found - axisNames.begin()),
                };
            };
            if (const auto operand =
                    parseAxis("apos_", OperandKind::AxisPosition)) {
                return *operand;
            }
            if (const auto operand =
                    parseAxis("astart_", OperandKind::AxisStart)) {
                return *operand;
            }
            if (const auto operand =
                    parseAxis("atarget_", OperandKind::AxisTarget)) {
                return *operand;
            }
            constexpr std::array jointPrefixes{
                std::pair{
                    std::string_view{"jpos_"},
                    OperandKind::JointPosition},
                std::pair{
                    std::string_view{"jstart_"},
                    OperandKind::JointStart},
                std::pair{
                    std::string_view{"jtarget_"},
                    OperandKind::JointTarget},
            };
            for (const auto &[prefix, kind] : jointPrefixes) {
                if (!text.starts_with(prefix)) {
                    continue;
                }
                const auto index = parseIndex(
                    text, prefix, MAX_JOINTS, line);
                if (!index) {
                    return std::unexpected(index.error());
                }

                return Operand{
                    .kind = kind,
                    .index = *index,
                };
            }
            if (const auto symbol = std::ranges::find(
                    symbols, text, &DigitalIoSymbol::name);
                symbol != symbols.end()) {
                return Operand{
                    .kind = [&] {
                        switch (symbol->kind) {
                            case DigitalIoSymbolKind::FieldInput:
                                return OperandKind::FieldInput;
                            case DigitalIoSymbolKind::LogicalInput:
                                return OperandKind::LogicalInput;
                            case DigitalIoSymbolKind::LogicalOutput:
                                return OperandKind::LogicalOutput;
                            case DigitalIoSymbolKind::FieldOutput:
                                return OperandKind::FieldOutput;
                        }

                        return OperandKind::Constant;
                    }(),
                    .index = symbol->id,
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
            const std::size_t line)
            -> std::expected<void, std::string> {
            if (operand.kind == OperandKind::Register) {
                assignedRegisters[operand.index] = true;

                return {};
            }
            if (operand.kind != OperandKind::LogicalInput
                && operand.kind != OperandKind::FieldOutput) {
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
            } else if (tokens[0] == "sub") {
                instruction.opcode = Opcode::Subtract;
                expectedTokens = 4;
            } else if (tokens[0] == "abs") {
                instruction.opcode = Opcode::Absolute;
                expectedTokens = 3;
            } else if (tokens[0] == "le") {
                instruction.opcode = Opcode::LessEqual;
                expectedTokens = 4;
            } else if (tokens[0] == "ge") {
                instruction.opcode = Opcode::GreaterEqual;
                expectedTokens = 4;
            } else if (tokens[0] == "test") {
                instruction.opcode = Opcode::Test;
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
                || instruction.opcode == Opcode::Xor
                || instruction.opcode == Opcode::Subtract
                || instruction.opcode == Opcode::LessEqual
                || instruction.opcode == Opcode::GreaterEqual
                || instruction.opcode == Opcode::Test) {
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

            if (const auto valid = assignDestination(
                    instruction.destination, line.number);
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

    double DigitalIoProgram::value(
        const Operand &operand,
        const FieldDigitalInputImage &fieldInputs,
        const LogicalDigitalOutputImage &logicalOutputs,
        const ProductionExecutorMotionContext &motion,
        const std::array<
            double,
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
            case OperandKind::MotionFlags:
                return static_cast<double>(motion.flags);
            case OperandKind::MoveJoints:
                return static_cast<double>(motion.moveJoints);
            case OperandKind::TriggerJoints:
                return static_cast<double>(motion.triggerJoints);
            case OperandKind::AxisPosition:
                return axisComponent(
                    motion.axisPosition, operand.index);
            case OperandKind::AxisStart:
                return axisComponent(
                    motion.axisStart, operand.index);
            case OperandKind::AxisTarget:
                return axisComponent(
                    motion.axisTarget, operand.index);
            case OperandKind::JointPosition:
                return motion.jointPosition[operand.index];
            case OperandKind::JointStart:
                return motion.jointStart[operand.index];
            case OperandKind::JointTarget:
                return motion.jointTarget[operand.index];
            case OperandKind::LogicalInput:
            case OperandKind::FieldOutput:
                return 0.0;
        }

        return 0.0;
    }

    void DigitalIoProgram::evaluate(
        const FieldDigitalInputImage &fieldInputs,
        const LogicalDigitalOutputImage &logicalOutputs,
        const ProductionExecutorMotionContext &motion,
        LogicalDigitalInputImage &logicalInputs,
        FieldDigitalOutputImage &fieldOutputs,
        std::array<
            DebounceState,
            DIGITAL_IO_PROGRAM_INSTRUCTION_CAPACITY> &debounce,
        const bool advanceDebounce) const noexcept {
        auto registers = std::array<
            double,
            DIGITAL_IO_PROGRAM_REGISTER_CAPACITY>{};
        auto nextInputs = LogicalDigitalInputImage{};
        auto nextFieldOutputs = FieldDigitalOutputImage{};
        for (std::size_t index = 0;
             index < m_instructionCount; ++index) {
            const auto &instruction = m_instructions[index];
            const auto first =
                value(
                    instruction.first, fieldInputs,
                    logicalOutputs, motion, registers);
            auto result = 0.0;
            switch (instruction.opcode) {
                case Opcode::Move:
                    result = first;
                    break;
                case Opcode::Not:
                    result = std::isfinite(first)
                            && !booleanValue(first)
                        ? 1.0 : 0.0;
                    break;
                case Opcode::And: {
                    const auto second = value(
                        instruction.second, fieldInputs,
                        logicalOutputs, motion, registers);
                    result = std::isfinite(first)
                            && std::isfinite(second)
                            && booleanValue(first)
                            && booleanValue(second)
                        ? 1.0 : 0.0;
                    break;
                }
                case Opcode::Or: {
                    const auto second = value(
                        instruction.second, fieldInputs,
                        logicalOutputs, motion, registers);
                    result = std::isfinite(first)
                            && std::isfinite(second)
                            && (booleanValue(first)
                                || booleanValue(second))
                        ? 1.0 : 0.0;
                    break;
                }
                case Opcode::Xor: {
                    const auto second = value(
                        instruction.second, fieldInputs,
                        logicalOutputs, motion, registers);
                    result = std::isfinite(first)
                            && std::isfinite(second)
                            && (booleanValue(first)
                                != booleanValue(second))
                        ? 1.0 : 0.0;
                    break;
                }
                case Opcode::Debounce: {
                    const auto firstBoolean =
                        booleanValue(first);
                    auto &state = debounce[index];
                    if (!advanceDebounce) {
                        result = state.output ? 1.0 : 0.0;
                        break;
                    }
                    if (!state.initialized) {
                        state.initialized = true;
                        state.output = firstBoolean;
                        state.candidate = firstBoolean;
                        state.stableTicks = 0;
                    } else if (instruction.debounceTicks == 0
                               || firstBoolean == state.output) {
                        state.output = firstBoolean;
                        state.candidate = firstBoolean;
                        state.stableTicks = 0;
                    } else {
                        if (firstBoolean != state.candidate) {
                            state.candidate = firstBoolean;
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
                    result = state.output ? 1.0 : 0.0;
                    break;
                }
                case Opcode::Subtract:
                    result = first - value(
                        instruction.second, fieldInputs,
                        logicalOutputs, motion, registers);
                    break;
                case Opcode::Absolute:
                    result = std::abs(first);
                    break;
                case Opcode::LessEqual: {
                    const auto second = value(
                        instruction.second, fieldInputs,
                        logicalOutputs, motion, registers);
                    result = std::isfinite(first)
                            && std::isfinite(second)
                            && first <= second
                        ? 1.0 : 0.0;
                    break;
                }
                case Opcode::GreaterEqual: {
                    const auto second = value(
                        instruction.second, fieldInputs,
                        logicalOutputs, motion, registers);
                    result = std::isfinite(first)
                            && std::isfinite(second)
                            && first >= second
                        ? 1.0 : 0.0;
                    break;
                }
                case Opcode::Test:
                    result = testMask(
                            first,
                            value(
                                instruction.second, fieldInputs,
                                logicalOutputs, motion, registers))
                        ? 1.0 : 0.0;
                    break;
            }

            if (instruction.destination.kind
                == OperandKind::Register) {
                registers[instruction.destination.index] = result;
            } else if (instruction.destination.kind
                       == OperandKind::LogicalInput) {
                nextInputs[instruction.destination.index] =
                    booleanValue(result);
            } else {
                nextFieldOutputs[instruction.destination.index] =
                    booleanValue(result);
            }
        }

        logicalInputs = nextInputs;
        fieldOutputs = nextFieldOutputs;
    }

    void DigitalIoProgram::executeInputs(
        const FieldDigitalInputImage &fieldInputs,
        const LogicalDigitalOutputImage &logicalOutputs,
        LogicalDigitalInputImage &logicalInputs) noexcept {
        executeInputs(
            fieldInputs, logicalOutputs,
            ProductionExecutorMotionContext{}, logicalInputs);
    }

    void DigitalIoProgram::executeInputs(
        const FieldDigitalInputImage &fieldInputs,
        const LogicalDigitalOutputImage &logicalOutputs,
        const ProductionExecutorMotionContext &motion,
        LogicalDigitalInputImage &logicalInputs) noexcept {
        auto unusedFieldOutputs = FieldDigitalOutputImage{};
        evaluate(
            fieldInputs, logicalOutputs, motion, logicalInputs,
            unusedFieldOutputs, m_debounce, true);
    }

    void DigitalIoProgram::executeOutputs(
        const FieldDigitalInputImage &fieldInputs,
        const LogicalDigitalOutputImage &logicalOutputs,
        FieldDigitalOutputImage &fieldOutputs) const noexcept {
        executeOutputs(
            fieldInputs, logicalOutputs,
            ProductionExecutorMotionContext{}, fieldOutputs);
    }

    void DigitalIoProgram::executeOutputs(
        const FieldDigitalInputImage &fieldInputs,
        const LogicalDigitalOutputImage &logicalOutputs,
        const ProductionExecutorMotionContext &motion,
        FieldDigitalOutputImage &fieldOutputs) const noexcept {
        auto unusedInputs = LogicalDigitalInputImage{};
        auto debounce = m_debounce;
        evaluate(
            fieldInputs, logicalOutputs, motion, unusedInputs,
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
