#pragma once

#include <optional>
#include <vector>

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

    class ArcReference {
        struct LengthNode {
            double parameter;
            double distance;
        };

        MoveArc m_arc;
        std::optional<ArcGeometry> m_geometry;
        std::vector<LengthNode> m_lengthNodes;
        double m_length = 0.0;

        double speed(double parameter) const;
        double integratedLength(double from, double to) const;

    public:
        explicit ArcReference(const MoveArc &arc);

        bool valid() const { return m_geometry.has_value(); }
        double length() const { return m_length; }
        position_t position(double parameter) const;
        position_t derivative(double parameter) const;
        double parameterAtDistance(double distance) const;
        position_t positionAtDistance(double distance) const;
        position_t tangentAtDistance(double distance) const;
        double chordErrorBound(double fromDistance, double toDistance) const;
        double curvatureAccelerationBound() const;
    };

    position_t mix(const position_t &from, const position_t &to, double t);
    double linearDistance(const position_t &from, const position_t &to);
    std::optional<ArcGeometry> arcGeometry(const MoveArc &arc);
    position_t interpolate(const MoveArc &arc, double t);
    double pathLength(const MoveArc &arc);
}
