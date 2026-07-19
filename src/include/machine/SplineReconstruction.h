#pragma once

#include <cstddef>
#include <expected>
#include <functional>
#include <limits>
#include <span>
#include <string>
#include <vector>

#include "machine/MachineCommand.h"

namespace ngc::spline_detail {
    enum class SplineFitSolver {
        None,
        CoordinateSearch,
        UniformBandedFairness,
        PeakTargetedBandedFairness,
        VelocityTargetedBandedFairness,
    };

    // One selection point shared by timed planning and geometry-only Preview.
    constexpr SplineFitSolver continuousSplineFitSolver() {
        return SplineFitSolver::VelocityTargetedBandedFairness;
    }

    struct SplineVelocityLimits {
        static position_t unlimitedAxes() {
            constexpr auto infinity=std::numeric_limits<double>::infinity();
            return {infinity,infinity,infinity,infinity,infinity,infinity};
        }

        double pathAcceleration=std::numeric_limits<double>::infinity();
        double pathJerk=std::numeric_limits<double>::infinity();
        position_t axisVelocity=unlimitedAxes();
        position_t axisAcceleration=unlimitedAxes();
        position_t axisJerk=unlimitedAxes();
    };

    struct SplineReconstructionSource {
        double length=0.0;
        std::function<position_t(double)> positionAt;
        std::function<double(double,double)> chordErrorBound;
        std::vector<double> boundaries;
        // One programmed feed for each consecutive source-boundary interval.
        std::vector<double> programmedFeeds;
        SplineVelocityLimits velocityLimits;
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
        bool certifyTube,
        SplineFitSolver solver = continuousSplineFitSolver());
}
