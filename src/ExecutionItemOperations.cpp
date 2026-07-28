#include "ExecutionItemOperations.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <ranges>
#include <type_traits>
#include <utility>
#include <variant>

namespace ngc::execution_item {
    namespace {
        bool finitePosition(const position_t &position) noexcept {
            return std::isfinite(position.x) && std::isfinite(position.y)
                && std::isfinite(position.z) && std::isfinite(position.a)
                && std::isfinite(position.b) && std::isfinite(position.c);
        }

        bool finiteMotionState(const MotionState &state) noexcept {
            return finitePosition(state.position)
                && finitePosition(state.velocity)
                && finitePosition(state.acceleration);
        }

        bool approximatelyEqual(const double actual,
                                const double expected) noexcept {
            return std::abs(actual - expected)
                <= 1e-12 * std::max(1.0, std::abs(expected));
        }

        bool validInputCondition(const InputCondition condition) noexcept {
            switch (condition) {
                case InputCondition::Active:
                case InputCondition::Inactive:
                case InputCondition::RisingEdge:
                case InputCondition::FallingEdge: return true;
            }

            return false;
        }

        double magnitude(const position_t &value) noexcept {
            return std::sqrt(value.x * value.x + value.y * value.y
                + value.z * value.z + value.a * value.a
                + value.b * value.b + value.c * value.c);
        }

        bool validSpan(const AxisPolynomialSpan &span) noexcept {
            if (span.id == 0
                || (span.degree != ExecutionPolynomialDegree::Cubic
                    && span.degree != ExecutionPolynomialDegree::Quintic)
                || !std::isfinite(span.duration) || span.duration <= 0.0
                || !finitePosition(span.origin)) {
                return false;
            }
            for (const auto &coefficient : span.coefficients) {
                if (!finitePosition(coefficient)) {
                    return false;
                }
            }

            const auto inverseDuration = 1.0 / span.duration;

            return std::isfinite(span.inverseDuration)
                && approximatelyEqual(
                    span.inverseDuration, inverseDuration)
                && std::isfinite(span.inverseDurationSquared)
                && approximatelyEqual(
                    span.inverseDurationSquared,
                    inverseDuration * inverseDuration)
                && std::isfinite(span.inverseDurationCubed)
                && approximatelyEqual(
                    span.inverseDurationCubed,
                    inverseDuration * inverseDuration * inverseDuration)
                && (span.degree != ExecutionPolynomialDegree::Cubic
                    || (span.coefficients[3].length() == 0.0
                        && span.coefficients[4].length() == 0.0));
        }

        bool validPlanChunk(const PlanChunk &chunk) noexcept {
            if (chunk.epoch == 0 || chunk.id == 0 || chunk.branch == 0
                || chunk.normalMotion.size == 0
                || chunk.stopTail.size == 0
                || !finiteMotionState(chunk.branchState)
                || !finiteMotionState(chunk.stopState)
                || !std::ranges::all_of(chunk.normalMotion, validSpan)
                || !std::ranges::all_of(chunk.stopTail, validSpan)) {
                return false;
            }

            std::optional<std::uint32_t> previousEventSpan;
            for (const auto &event : chunk.events) {
                const auto valid = std::visit([](const auto &value) {
                    using T = std::decay_t<decltype(value)>;
                    if constexpr (std::same_as<T, SpindleEvent>) {
                        const auto validDirection =
                            value.direction == Direction::CW
                            || value.direction == Direction::CCW;

                        return validDirection && std::isfinite(value.speed)
                            && value.speed >= 0.0;
                    } else {
                        return false;
                    }
                }, event.value);
                if (!valid || event.span >= chunk.normalMotion.size
                    || (previousEventSpan.has_value()
                        && event.span < *previousEventSpan)) {
                    return false;
                }
                previousEventSpan = event.span;
            }

            std::optional<std::pair<std::uint32_t, double>> previousMarker;
            for (const auto &marker : chunk.markers) {
                const auto location =
                    std::pair{marker.span, marker.parameter};
                if (marker.id == 0
                    || marker.span >= chunk.normalMotion.size
                    || !std::isfinite(marker.parameter)
                    || marker.parameter < 0.0
                    || marker.parameter > 1.0
                    || (previousMarker.has_value()
                        && location < *previousMarker)) {
                    return false;
                }
                previousMarker = location;
            }

            return true;
        }

        bool validTriggeredMove(const TriggeredMove &move) noexcept {
            return move.epoch != 0 && move.id != 0 && move.branch != 0
                && move.moveId != 0 && finitePosition(move.target)
                && std::isfinite(magnitude(move.limits.velocity))
                && magnitude(move.limits.velocity) > 0.0
                && std::isfinite(magnitude(move.limits.acceleration))
                && magnitude(move.limits.acceleration) > 0.0
                && std::isfinite(magnitude(move.limits.jerk))
                && magnitude(move.limits.jerk) > 0.0
                && validInputCondition(move.condition);
        }

        bool validTriggeredJointMove(const TriggeredJointMove &move) noexcept {
            constexpr auto validJointMask =
                static_cast<JointMask>((JointMask{1} << MAX_JOINTS) - 1);
            if (move.epoch == 0 || move.id == 0 || move.branch == 0
                || move.moveId == 0 || move.joints == 0
                || (move.joints & ~validJointMask) != 0
                || (move.targetMode != JointTargetMode::Absolute
                    && move.targetMode != JointTargetMode::Relative)) {
                return false;
            }

            for (JointId joint = 0; joint < MAX_JOINTS; ++joint) {
                const auto mask =
                    static_cast<JointMask>(JointMask{1} << joint);
                if ((move.joints & mask) == 0) {
                    continue;
                }
                if (!std::isfinite(move.target[joint])
                    || !std::isfinite(move.limits.velocity[joint])
                    || move.limits.velocity[joint] <= 0.0
                    || !std::isfinite(move.limits.acceleration[joint])
                    || move.limits.acceleration[joint] <= 0.0
                    || !std::isfinite(move.limits.jerk[joint])
                    || move.limits.jerk[joint] <= 0.0) {
                    return false;
                }
            }

            JointMask triggeredJoints = 0;
            for (const auto &trigger : move.triggers) {
                if (trigger.joint >= MAX_JOINTS
                    || !validInputCondition(trigger.condition)) {
                    return false;
                }

                const auto mask =
                    static_cast<JointMask>(
                        JointMask{1} << trigger.joint);
                if ((move.joints & mask) == 0
                    || (triggeredJoints & mask) != 0) {
                    return false;
                }
                triggeredJoints |= mask;
            }

            return !move.triggerRequired
                || triggeredJoints == move.joints;
        }

        std::uint64_t secondsToNanoseconds(const double seconds) noexcept {
            constexpr auto scale = 1.0e9;
            if (!std::isfinite(seconds) || seconds <= 0.0) {
                return 0;
            }

            const auto maximum = static_cast<double>(
                std::numeric_limits<std::uint64_t>::max());
            if (seconds >= maximum / scale) {
                return std::numeric_limits<std::uint64_t>::max();
            }

            return static_cast<std::uint64_t>(
                std::llround(seconds * scale));
        }
    }

    bool valid(const ExecutionItem &item) noexcept {
        return std::visit([](const auto &value) {
            using T = std::decay_t<decltype(value)>;
            if constexpr (std::same_as<T, PlanChunk>) {
                return validPlanChunk(value);
            } else if constexpr (std::same_as<T, TriggeredMove>) {
                return validTriggeredMove(value);
            } else {
                return validTriggeredJointMove(value);
            }
        }, item);
    }

    std::uint64_t normalMotionNanoseconds(const ExecutionItem &item) noexcept {
        const auto *chunk = std::get_if<PlanChunk>(&item);
        if (chunk == nullptr) {
            return 0;
        }

        auto seconds = 0.0;
        for (const auto &span : chunk->normalMotion) {
            seconds += std::max(span.duration, 0.0);
        }

        return secondsToNanoseconds(seconds);
    }

    EpochId epoch(const ExecutionItem &item) noexcept {
        return std::visit([](const auto &value) {
            return value.epoch;
        }, item);
    }

    ChunkId id(const ExecutionItem &item) noexcept {
        return std::visit([](const auto &value) {
            return value.id;
        }, item);
    }

    BranchSequence predecessor(const ExecutionItem &item) noexcept {
        return std::visit([](const auto &value) {
            return value.predecessorBranch;
        }, item);
    }
}
