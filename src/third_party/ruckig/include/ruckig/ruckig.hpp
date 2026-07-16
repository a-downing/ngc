#pragma once

#include <cstddef>
#include <type_traits>

#include <ruckig/calculator_target.hpp>
#include <ruckig/error.hpp>
#include <ruckig/input_parameter.hpp>
#include <ruckig/trajectory.hpp>


namespace ruckig {

//! NGC's fixed-DoF, offline-only facade for Ruckig's state-to-state calculator.
template<size_t DOFs, template<class, size_t> class CustomVector = StandardVector,
         bool throw_error = false>
class Ruckig {
    static_assert(DOFs >= 1, "NGC's vendored Ruckig facade requires fixed degrees of freedom");

    TargetCalculator<DOFs, CustomVector> calculator;

public:
    //! Time step used only when discrete-duration calculation is requested.
    double delta_time {-1.0};

    Ruckig() = default;
    explicit Ruckig(const double delta_time): delta_time(delta_time) {}

    template<bool throw_validation_error = true>
    bool validate_input(const InputParameter<DOFs, CustomVector>& input,
                        const bool check_current_state_within_limits = false,
                        const bool check_target_state_within_limits = true) const {
        if (!input.template validate<throw_validation_error>(
                check_current_state_within_limits, check_target_state_within_limits)) {
            return false;
        }

        if (!input.intermediate_positions.empty()) {
            if constexpr (throw_validation_error) {
                throw RuckigError("intermediate waypoints are not included in NGC's vendored Ruckig subset");
            }
            return false;
        }

        if (delta_time <= 0.0
            && input.duration_discretization != DurationDiscretization::Continuous) {
            if constexpr (throw_validation_error) {
                throw RuckigError("delta time should be larger than zero for discrete duration");
            }
            return false;
        }

        return true;
    }

    Result calculate(const InputParameter<DOFs, CustomVector>& input,
                     Trajectory<DOFs, CustomVector>& trajectory) {
        bool was_interrupted {false};
        return calculate(input, trajectory, was_interrupted);
    }

    Result calculate(const InputParameter<DOFs, CustomVector>& input,
                     Trajectory<DOFs, CustomVector>& trajectory,
                     bool& was_interrupted) {
        if (!validate_input<throw_error>(input, false, true)) {
            return Result::ErrorInvalidInput;
        }

        return calculator.template calculate<throw_error>(
            input, trajectory, delta_time, was_interrupted);
    }
};

} // namespace ruckig
