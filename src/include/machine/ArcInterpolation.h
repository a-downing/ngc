#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

#include "machine/MachineCommand.h"

namespace ngc::simulation_detail {
    struct ArcInverseDiagnostics {
        std::size_t constructionIntegralEvaluations = 0;
        std::size_t queries = 0;
        std::size_t endpointQueries = 0;
        std::size_t exactCacheHits = 0;
        std::size_t inverseIntegralEvaluations = 0;
        std::size_t newtonIterations = 0;
        std::size_t seedConvergences = 0;
        std::size_t safeguardedBisections = 0;
        std::size_t iterationLimitHits = 0;
        std::size_t maximumNewtonIterations = 0;
    };

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
        ArcInverseDiagnostics *m_inverseDiagnostics = nullptr;
        struct InverseCacheEntry {
            double distance = 0.0;
            double parameter = 0.0;
            bool valid = false;
        };
        static constexpr std::size_t INVERSE_CACHE_SIZE = 16;
        mutable std::array<InverseCacheEntry, INVERSE_CACHE_SIZE> m_inverseCache;

        double speed(double parameter) const;
        double integratedLength(double from, double to, bool inverseEvaluation = false) const;
        static std::size_t inverseCacheIndex(double distance);

    public:
        explicit ArcReference(const MoveArc &arc,
                              ArcInverseDiagnostics *inverseDiagnostics = nullptr);

        bool valid() const { return m_geometry.has_value(); }
        double length() const { return m_length; }
        position_t position(double parameter) const;
        position_t derivative(double parameter) const;
        position_t secondDerivative(double parameter) const;
        position_t thirdDerivative(double parameter) const;
        double parameterAtDistance(double distance) const;
        position_t positionAtDistance(double distance) const;
        position_t tangentAtDistance(double distance) const;
        position_t curvatureAtDistance(double distance) const;
        position_t curvatureDerivativeAtDistance(double distance) const;
        double chordErrorBound(double fromDistance, double toDistance) const;
        double curvatureAccelerationBound() const;
    };

    position_t mix(const position_t &from, const position_t &to, double t);
    double linearDistance(const position_t &from, const position_t &to);
    std::optional<ArcGeometry> arcGeometry(const MoveArc &arc);
    position_t interpolate(const MoveArc &arc, double t);
    double pathLength(const MoveArc &arc);
}
