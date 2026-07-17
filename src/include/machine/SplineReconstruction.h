#pragma once

#include <cstddef>
#include <expected>
#include <functional>
#include <span>
#include <string>
#include <vector>

#include "machine/MachineCommand.h"

namespace ngc::spline_detail {
    enum class SplineFitSolver {
        CubicBaseline,
        CoordinateSearch,
        UniformBandedFairness,
        PeakTargetedBandedFairness,
    };

    // One selection point shared by timed planning and geometry-only Preview.
    constexpr SplineFitSolver continuousSplineFitSolver() {
        return SplineFitSolver::UniformBandedFairness;
    }

    struct SplineReconstructionSource {
        double length=0.0;
        std::function<position_t(double)> positionAt;
        std::function<double(double,double)> chordErrorBound;
        std::vector<double> boundaries;
        position_t startTangent{};
        position_t startCurvature{};
        position_t startCurvatureDerivative{};
        position_t endTangent{};
        position_t endCurvature{};
        position_t endCurvatureDerivative{};
    };

    struct ReconstructedSpline {
        std::size_t degree=0;
        std::vector<position_t> controls;
    };

    std::expected<ReconstructedSpline,std::string> reconstructSpline(
        std::span<const position_t> cubicControls,
        const SplineReconstructionSource &source,
        double programmedScale,
        bool certifyTube);
}
