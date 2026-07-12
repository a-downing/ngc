#pragma once

#include <cmath>
#include <cstdint>
#include <utility>
#include <string>
#include <format>
#include <variant>

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
            return { x+p.x, y+p.y, z+p.z, a+p.a, b+p.b, c+p.c };
        }

        position_t operator-(const position_t &p) const {
            return { x-p.x, y-p.y, z-p.z, a-p.a, b-p.b, c-p.c };
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

    class SpindleStart {
        Direction m_direction;
        double m_speed;

    public:
        explicit SpindleStart(const Direction dir, const double speed) : m_direction(dir), m_speed(speed) { }
        Direction direction() const { return m_direction; }
        double speed() const { return m_speed; }

        std::string text() const { return std::format("Spindle({}, {})", name(m_direction), m_speed); }
    };

    struct ToolGeometry {
        int number = 0;
        position_t offset{};
        double diameter = 0.0;
    };

    struct ToolPose {
        ToolGeometry geometry{};
        position_t spindlePosition{};
        position_t tipPosition{};
    };

    struct WorkCoordinateSystem {
        std::string name;
        position_t offset{};
    };

    class SpindleStop {
    public:
        std::string text() const { return "SpindleStop()"; }
    };

    class MoveLine {
        position_t m_from;
        position_t m_to;
        double m_speed;
        bool m_machineCoordinates;

    public:
        MoveLine(const position_t &from, const position_t &to, const double speed, const bool machineCoordinates = false)
            : m_from(from), m_to(to), m_speed(speed), m_machineCoordinates(machineCoordinates) { }
        const position_t &from() const { return m_from; }
        const position_t &to() const { return m_to; }
        double speed() const { return m_speed; }
        bool machineCoordinates() const { return m_machineCoordinates; }

        std::string text() const {
            return std::format("MoveLine(from: {}, to: {}, speed: {}, machineCoordinates: {})",
                               m_from.text(), m_to.text(), m_speed, m_machineCoordinates);
        }
    };

    class MoveArc {
        position_t m_from;
        position_t m_to;
        vec3_t m_center;
        vec3_t m_axis;
        double m_speed;

    public:
        MoveArc(const position_t &from, const position_t &to, const vec3_t &center, const vec3_t &axis, const double speed) : m_from(from), m_to(to), m_center(center), m_axis(axis), m_speed(speed) { }
        const position_t &from() const { return m_from; }
        const position_t &to() const { return m_to; }
        const vec3_t &center() const { return m_center; }
        const vec3_t &axis() const { return m_axis; }
        double speed() const { return m_speed; }

        std::string text() const { return std::format("MoveArc(from: {}, to: {}, center: {}, axis: {}, speed: {})", m_from.text(), m_to.text(), m_center.text(), m_axis.text(), m_speed); }
    };

    class ProbeMove {
        std::uint64_t m_id;
        position_t m_from;
        position_t m_target;
        double m_feed;
        bool m_stopOnContact;
        bool m_errorIfNotFound;

    public:
        ProbeMove(const std::uint64_t id, const position_t &from, const position_t &target, const double feed,
                  const bool stopOnContact, const bool errorIfNotFound)
            : m_id(id), m_from(from), m_target(target), m_feed(feed),
              m_stopOnContact(stopOnContact), m_errorIfNotFound(errorIfNotFound) { }

        std::uint64_t id() const { return m_id; }
        const position_t &from() const { return m_from; }
        const position_t &target() const { return m_target; }
        double feed() const { return m_feed; }
        bool stopOnContact() const { return m_stopOnContact; }
        bool errorIfNotFound() const { return m_errorIfNotFound; }

        std::string text() const {
            return std::format("ProbeMove(id: {}, from: {}, target: {}, feed: {})", m_id, m_from.text(), m_target.text(), m_feed);
        }
    };

    enum class ProbeStatus {
        Triggered,
        ReachedTarget,
        Aborted,
        Fault
    };

    struct ProbeResult {
        std::uint64_t id;
        ProbeStatus status;
        position_t triggerPosition;
        position_t stoppedPosition;
    };

    using MachineCommand = std::variant<SpindleStart, SpindleStop, MoveLine, MoveArc, ProbeMove>;
}
