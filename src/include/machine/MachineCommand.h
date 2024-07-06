#pragma once

#include <cmath>
#include <utility>
#include <string>
#include <format>

#include "utils.h"

namespace ngc {
    struct vec3_t {
        double x, y, z;
        vec3_t() = default;
        vec3_t(const double x, const double y, const double z) : x(x), y(y), z(z) {};

        double length() const { return std::sqrt(x*x + y*y + z*z); }

        vec3_t operator+(const vec3_t &p) const {
            return { x+p.x, y+p.y, z+p.z };
        }

        vec3_t operator-(const vec3_t &p) const {
            return { x-p.x, y-p.y, z-p.z };
        }

        vec3_t operator*(const double s) const {
            return { x*s, y*s, z*s };
        }

        std::string text() const {
            return std::format("vec3_t({}, {}, {})", x, y, z);
        }
    };

    struct position_t {
        double x, y, z, a, b, c;
        position_t() = default;
        position_t(const double x, const double y, const double z, const double a, const double b, const double c) : x(x), y(y), z(z), a(a), b(b), c(c) {};

        double length() const { return std::sqrt(x*x + y*y + z*z + a*a + b*b + c*c); }

        position_t operator+(const position_t &p) const {
            return { x+p.x, y+p.y, z+p.z, a+p.a, b+p.b, x+p.c };
        }

        position_t operator-(const position_t &p) const {
            return { x-p.x, y-p.y, z-p.z, a-p.a, b-p.b, x-p.c };
        }

        std::string text() const {
            return std::format("position_t({}, {}, {}, {}, {}, {})", x, y, z, a, b, c);
        }
    };

    enum class Direction {
        CW,
        CCW
    };

    inline std::string_view name(const Direction direction) {
        switch(direction) {
            case Direction::CW: return "CW";
            case Direction::CCW: return "CCW";
        }

        PANIC("{} missing case statement for Direction::{}", __func__, std::to_underlying(direction));
    }

    class MachineCommand {
    public:
        virtual ~MachineCommand() = default;
        virtual std::string text() const = 0;

        virtual bool is(const class SpindleStart *) const { return false; }
        virtual bool is(const class SpindleStop*) const { return false; }
        virtual bool is(const class MoveLine *) const { return false; }
        virtual bool is(const class MoveArc *) const { return false; }

        template<typename T>
        bool is() const requires std::derived_from<T, MachineCommand> {
            return this->is(static_cast<const T *>(nullptr));
        }

        template<typename T>
        const T *as() const requires std::derived_from<T, MachineCommand> {
            return this->is(static_cast<const T *>(nullptr)) ? static_cast<const T *>(this) : nullptr;
        }
    };

    class SpindleStart final : public MachineCommand {
        Direction m_direction;
        double m_speed;

    public:
        explicit SpindleStart(const Direction dir, const double speed) : m_direction(dir), m_speed(speed) { }
        ~SpindleStart() override = default;

        bool is(const SpindleStart *) const override { return true; }

        Direction direction() const { return m_direction; }
        double speed() const { return m_speed; }

        std::string text() const override { return std::format("Spindle({}, {})", name(m_direction), m_speed); }
    };

    class SpindleStop final : public MachineCommand {
    public:
        ~SpindleStop() override = default;

        bool is(const SpindleStop *) const override { return true; }

        std::string text() const override { return "SpindleStop()"; }
    };

    class MoveLine final : public MachineCommand {
        position_t m_from;
        position_t m_to;
        double m_speed;

    public:
        MoveLine(const position_t &from, const position_t &to, const double speed) : m_from(from), m_to(to), m_speed(speed) { }
        ~MoveLine() override = default;

        bool is(const MoveLine *) const override { return true; }

        const position_t &from() const { return m_from; }
        const position_t &to() const { return m_to; }
        double speed() const { return m_speed; }

        std::string text() const override { return std::format("MoveLine(from: {}, to: {}, speed: {})", m_from.text(), m_to.text(), m_speed); }
    };

    class MoveArc final : public MachineCommand {
        position_t m_from;
        position_t m_to;
        vec3_t m_center;
        vec3_t m_axis;
        double m_speed;

    public:
        MoveArc(const position_t &from, const position_t &to, const vec3_t &center, const vec3_t &axis, const double speed) : m_from(from), m_to(to), m_center(center), m_axis(axis), m_speed(speed) { }
        ~MoveArc() override = default;

        bool is(const MoveArc *) const override { return true; }

        const position_t &from() const { return m_from; }
        const position_t &to() const { return m_to; }
        const vec3_t &center() const { return m_center; }
        const vec3_t &axis() const { return m_axis; }
        double speed() const { return m_speed; }

        std::string text() const override { return std::format("MoveArc(from: {}, to: {}, center: {}, axis: {}, speed: {})", m_from.text(), m_to.text(), m_center.text(), m_axis.text(), m_speed); }
    };
}