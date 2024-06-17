module;

#include <utility>

export module memory:MemoryCell;

export namespace ngc
{
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

    constexpr MemoryCell::Flags operator|(const MemoryCell::Flags &a, const MemoryCell::Flags &b) {
        return static_cast<MemoryCell::Flags>(std::to_underlying(a) | std::to_underlying(b));
    }
}
