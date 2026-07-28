#pragma once

#include <array>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>
#include <string>
#include <string_view>

#include "machine/MotionBackend.h"

namespace ngc {
    inline constexpr std::size_t
        DIGITAL_IO_PROGRAM_FIELD_INPUT_CAPACITY = 128;
    inline constexpr std::size_t
        DIGITAL_IO_PROGRAM_FIELD_OUTPUT_CAPACITY = 128;
    inline constexpr std::size_t DIGITAL_IO_PROGRAM_REGISTER_CAPACITY = 64;
    inline constexpr std::size_t DIGITAL_IO_PROGRAM_INSTRUCTION_CAPACITY = 256;
    using FieldDigitalInputImage =
        std::bitset<DIGITAL_IO_PROGRAM_FIELD_INPUT_CAPACITY>;
    using FieldDigitalOutputImage =
        std::bitset<DIGITAL_IO_PROGRAM_FIELD_OUTPUT_CAPACITY>;

    class DigitalIoProgram {
    public:
        [[nodiscard]] static std::expected<DigitalIoProgram, std::string>
        compile(
            std::string_view source,
            std::size_t fieldInputCount,
            std::span<const DigitalInputId> logicalInputs,
            std::size_t fieldOutputCount,
            std::span<const DigitalOutputId> logicalOutputs,
            double servoPeriod);

        void executeInputs(
            const FieldDigitalInputImage &fieldInputs,
            const LogicalDigitalOutputImage &logicalOutputs,
            LogicalDigitalInputImage &logicalInputs) noexcept;
        void executeOutputs(
            const FieldDigitalInputImage &fieldInputs,
            const LogicalDigitalOutputImage &logicalOutputs,
            FieldDigitalOutputImage &fieldOutputs) const noexcept;
        void reset() noexcept;

        [[nodiscard]] std::size_t instructionCount() const noexcept;
        [[nodiscard]] std::size_t fieldInputCount() const noexcept;
        [[nodiscard]] std::size_t logicalInputCount() const noexcept;
        [[nodiscard]] std::size_t fieldOutputCount() const noexcept;
        [[nodiscard]] std::size_t logicalOutputCount() const noexcept;

    private:
        enum class Opcode : std::uint8_t {
            Move,
            Not,
            And,
            Or,
            Xor,
            Debounce,
        };

        enum class OperandKind : std::uint8_t {
            Register,
            FieldInput,
            LogicalInput,
            LogicalOutput,
            FieldOutput,
            Constant,
        };

        struct Operand {
            OperandKind kind = OperandKind::Constant;
            std::uint16_t index = 0;
            bool constant = false;
        };

        struct Instruction {
            Opcode opcode = Opcode::Move;
            Operand destination;
            Operand first;
            Operand second;
            std::uint32_t debounceTicks = 0;
        };

        struct DebounceState {
            bool initialized = false;
            bool output = false;
            bool candidate = false;
            std::uint32_t stableTicks = 0;
        };

        [[nodiscard]] bool value(
            const Operand &operand,
            const FieldDigitalInputImage &fieldInputs,
            const LogicalDigitalOutputImage &logicalOutputs,
            const std::bitset<
                DIGITAL_IO_PROGRAM_REGISTER_CAPACITY> &registers) const noexcept;
        void evaluate(
            const FieldDigitalInputImage &fieldInputs,
            const LogicalDigitalOutputImage &logicalOutputs,
            LogicalDigitalInputImage &logicalInputs,
            FieldDigitalOutputImage &fieldOutputs,
            std::array<
                DebounceState,
                DIGITAL_IO_PROGRAM_INSTRUCTION_CAPACITY> &debounce,
            bool advanceDebounce) const noexcept;

        std::array<
            Instruction,
            DIGITAL_IO_PROGRAM_INSTRUCTION_CAPACITY> m_instructions{};
        std::array<
            DebounceState,
            DIGITAL_IO_PROGRAM_INSTRUCTION_CAPACITY> m_debounce{};
        std::size_t m_instructionCount = 0;
        std::size_t m_fieldInputCount = 0;
        std::size_t m_logicalInputCount = 0;
        std::size_t m_fieldOutputCount = 0;
        std::size_t m_logicalOutputCount = 0;
    };
}
