#pragma once

#include <optional>

#include "machine/MachineCommand.h"

namespace ngc::simulation_detail {
    struct ArcGeometry {
        vec3_t center;
        vec3_t axisUnit;
        vec3_t startArm;
        vec3_t endArm;
        vec3_t axial;
        double sweep;
    };

    position_t mix(const position_t &from, const position_t &to, double t);
    double linearDistance(const position_t &from, const position_t &to);
    std::optional<ArcGeometry> arcGeometry(const MoveArc &arc);
    position_t interpolate(const MoveArc &arc, double t);
    double pathLength(const MoveArc &arc);
}
