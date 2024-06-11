
#ifndef VARS_H
#define VARS_H

namespace ngc {
    class MemoryCell {
    public:
        enum Flags {
            READ = 1,
            WRITE = 2,
            VOLATILE = 4
        };

        MemoryCell(const Flags flags, const double value) : m_flags(flags), m_value(value) { }
        explicit MemoryCell(const Flags flags) : m_flags(flags), m_value(0.0) { }
        [[nodiscard]] bool readFlag() const { return m_flags & READ; }
        [[nodiscard]] bool writeFlag() const { return m_flags & WRITE; }
        [[nodiscard]] bool volatileFlag() const { return m_flags & VOLATILE; }

        [[nodiscard]] double read() const { return m_value; }
        void write(const double value) { m_value = value; }

    private:
        Flags m_flags;
        double m_value;
    };

    enum class Var {
        G54_X,
        G54_Y,
        G54_Z,
    };

    static constexpr std::initializer_list<std::tuple<Var, std::string_view, MemoryCell::Flags>> GLOBALS = {
        { Var::G54_X, "_g54_x", MemoryCell::Flags::READ },
        { Var::G54_Y, "_g54_y", MemoryCell::Flags::READ },
        { Var::G54_Z, "_g54_z", MemoryCell::Flags::READ }
    };
}

#endif
