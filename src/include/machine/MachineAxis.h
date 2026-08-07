#pragma once

#include "machine/Machine.h"

namespace ngc {
    inline double &machineAxisPositionComponent(position_t &position, const Machine::Axis axis) {
        switch (axis) {
            case Machine::Axis::X: return position.x;
            case Machine::Axis::Y: return position.y;
            case Machine::Axis::Z: return position.z;
            case Machine::Axis::A: return position.a;
            case Machine::Axis::B: return position.b;
            case Machine::Axis::C: return position.c;
        }

        PANIC("invalid Machine::Axis value");
    }

    inline const double &machineAxisPositionComponent(const position_t &position, const Machine::Axis axis) {
        switch (axis) {
            case Machine::Axis::X: return position.x;
            case Machine::Axis::Y: return position.y;
            case Machine::Axis::Z: return position.z;
            case Machine::Axis::A: return position.a;
            case Machine::Axis::B: return position.b;
            case Machine::Axis::C: return position.c;
        }

        PANIC("invalid Machine::Axis value");
    }
}
