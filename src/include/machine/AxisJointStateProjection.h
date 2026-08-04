#pragma once

#include <array>
#include <cstddef>

#include "machine/MotionBackend.h"

namespace ngc {
    struct AxisJointMapping {
        JointMask joints = 0;
        JointVector coordinateScale{};
    };

    using AxisJointMappings = std::array<
        AxisJointMapping, static_cast<std::size_t>(AxisId::C) + 1>;

    namespace axis_joint_state_projection {
        inline double &axisComponent(position_t &value, const AxisId axis) noexcept {
            switch (axis) {
                case AxisId::X: return value.x;
                case AxisId::Y: return value.y;
                case AxisId::Z: return value.z;
                case AxisId::A: return value.a;
                case AxisId::B: return value.b;
                case AxisId::C: return value.c;
            }

            return value.x;
        }

        inline double axisComponent(const position_t &value, const AxisId axis) noexcept {
            auto copy = value;

            return axisComponent(copy, axis);
        }

        inline void advanceFromAxes(const AxisJointMappings &mappings,
                                    const MotionState &previousAxes,
                                    const MotionState &nextAxes,
                                    JointMotionState &joints) noexcept {
            for (std::size_t axisIndex = 0; axisIndex < mappings.size(); ++axisIndex) {
                const auto axis = static_cast<AxisId>(axisIndex);
                const auto &mapping = mappings[axisIndex];
                const auto positionDelta = axisComponent(nextAxes.position, axis)
                    - axisComponent(previousAxes.position, axis);
                for (JointId joint = 0; joint < MAX_JOINTS; ++joint) {
                    const auto mask = static_cast<JointMask>(JointMask{1} << joint);
                    if ((mapping.joints & mask) == 0) {
                        continue;
                    }

                    const auto scale = mapping.coordinateScale[joint];
                    joints.position[joint] += positionDelta * scale;
                    joints.velocity[joint] = axisComponent(nextAxes.velocity, axis) * scale;
                    joints.acceleration[joint] = axisComponent(nextAxes.acceleration, axis) * scale;
                }
            }
        }

        inline void projectFromJoints(const AxisJointMappings &mappings,
                                      const JointMotionState *previousJoints,
                                      const JointMotionState &nextJoints,
                                      MotionState &axes) noexcept {
            for (std::size_t axisIndex = 0; axisIndex < mappings.size(); ++axisIndex) {
                const auto axis = static_cast<AxisId>(axisIndex);
                const auto &mapping = mappings[axisIndex];
                double position = 0.0;
                double velocity = 0.0;
                double acceleration = 0.0;
                std::size_t count = 0;
                for (JointId joint = 0; joint < MAX_JOINTS; ++joint) {
                    const auto mask = static_cast<JointMask>(JointMask{1} << joint);
                    if ((mapping.joints & mask) == 0) {
                        continue;
                    }

                    const auto scale = mapping.coordinateScale[joint];
                    position += previousJoints == nullptr
                        ? nextJoints.position[joint] / scale
                        : (nextJoints.position[joint]
                            - previousJoints->position[joint]) / scale;
                    velocity += nextJoints.velocity[joint] / scale;
                    acceleration += nextJoints.acceleration[joint] / scale;
                    ++count;
                }
                if (count == 0) {
                    continue;
                }

                const auto divisor = static_cast<double>(count);
                if (previousJoints == nullptr) {
                    axisComponent(axes.position, axis) = position / divisor;
                } else {
                    axisComponent(axes.position, axis) += position / divisor;
                }
                axisComponent(axes.velocity, axis) = velocity / divisor;
                axisComponent(axes.acceleration, axis) = acceleration / divisor;
            }
        }

        inline void advanceFromJoints(const AxisJointMappings &mappings,
                                      const JointMotionState &previousJoints,
                                      const JointMotionState &nextJoints,
                                      MotionState &axes) noexcept {
            projectFromJoints(mappings, &previousJoints, nextJoints, axes);
        }

        inline void assignFromJoints(const AxisJointMappings &mappings,
                                     const JointMotionState &joints,
                                     MotionState &axes) noexcept {
            projectFromJoints(mappings, nullptr, joints, axes);
        }
    }
}
