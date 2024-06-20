module;

#include <utility>
#include <string>
#include <format>
#include <stdexcept>

export module machine:MachineCommand;

export namespace ngc {
    class MachineCommand {
    public:
        virtual ~MachineCommand() = default;
        [[nodiscard]] virtual std::string text() const = 0;
    };

    class SpindleStart final : public MachineCommand {
    public:
        enum class Dir {
            CW,
            CCW
        };

    private:
        Dir m_dir;
        double m_speed;

    public:
        explicit SpindleStart(const Dir dir, const double speed) : m_dir(dir), m_speed(speed) { }
        ~SpindleStart() override = default;

        [[nodiscard]] std::string text() const override { return std::format("Spindle({}, {})", name(m_dir), m_speed); }

    private:
        static const char *name(const Dir state) {
            switch(state) {
            case Dir::CW: return "START_CW";
            case Dir::CCW: return "START_CCW";
            default: throw std::runtime_error(std::format("SpindleStart::{} missing case statement for Dir::{}", __func__, std::to_underlying(state)));
            }
        }
    };

    class SpindleStop final : public MachineCommand {
    public:
        ~SpindleStop() override = default;
        [[nodiscard]] std::string text() const override { return "SpindleStop()"; }
    };
}