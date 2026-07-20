#include "operator/OperatorJogController.h"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <ranges>
#include <type_traits>
#include <utility>

namespace ngc::operator_control {
    namespace {
        std::optional<Machine::Axis> machineAxis(const pendant::Axis axis) noexcept {
            switch(axis) {
                case pendant::Axis::X: return Machine::Axis::X;
                case pendant::Axis::Y: return Machine::Axis::Y;
                case pendant::Axis::Z: return Machine::Axis::Z;
                case pendant::Axis::A: return Machine::Axis::A;
                case pendant::Axis::B: return Machine::Axis::B;
                case pendant::Axis::C: return Machine::Axis::C;
            }
            return std::nullopt;
        }

        AxisId backendAxis(const Machine::Axis axis) noexcept {
            return static_cast<AxisId>(static_cast<std::uint8_t>(axis));
        }

        std::string_view axisName(const Machine::Axis axis) noexcept {
            switch(axis) {
                case Machine::Axis::X: return "X";
                case Machine::Axis::Y: return "Y";
                case Machine::Axis::Z: return "Z";
                case Machine::Axis::A: return "A";
                case Machine::Axis::B: return "B";
                case Machine::Axis::C: return "C";
            }
            return "?";
        }
    }

    JogController::JogController(const MachineConfiguration &configuration)
        : m_jogging(configuration.jogging), m_step(configuration.pendant.step),
          m_velocity(configuration.pendant.velocity),
          m_servoPeriod(configuration.simulation.servoPeriod),
          m_axes(configuration.axes), m_joints(configuration.joints) { }

    void JogController::consume(const pendant::Intent &intent) {
        std::visit([&](const auto &value) {
            using T = std::decay_t<decltype(value)>;
            if constexpr(std::same_as<T, pendant::JogWheel>) {
                // Retain at most one increment beyond the active/submitted one.
                // This keeps continuous turning responsive without allowing an
                // unbounded motion backlog after the wheel stops.
                if(m_pending || (m_activeJog && m_activeContinuous)) return;
                const auto increment = value.increment == pendant::JogIncrement::Coarse
                    ? m_step.coarseDistance : m_step.fineDistance;
                if(value.counts == 0 || increment <= 0.0) return;
                const auto distance = value.counts < 0 ? -increment : increment;
                if(!std::isfinite(distance)) {
                    m_error = "pendant step distance overflowed";
                    m_pending.reset();
                    return;
                }
                m_pending = PendingIncrement { value.axis, distance };
                if(!std::isfinite(m_pending->distance)) {
                    m_error = "accumulated pendant step distance overflowed";
                    m_pending.reset();
                } else if(std::abs(m_pending->distance) <= 1e-12) {
                    m_pending.reset();
                }
            } else if constexpr(std::same_as<T, pendant::JogVelocity>) {
                if(value.countsPerSecond == 0) {
                    m_pendingVelocity.reset();
                    m_velocityRefreshRequested = false;
                    if(m_activeJog && m_activeContinuous) m_stopRequested = true;
                    return;
                }
                m_pendingVelocity = PendingVelocity { value.axis, value.countsPerSecond };
                m_velocityRefreshRequested = true;
            } else if constexpr(std::same_as<T, pendant::CancelPendantActivity>) {
                m_pending.reset();
                m_pendingVelocity.reset();
                m_velocityRefreshRequested = false;
                if(m_activeJog) m_stopRequested = true;
            }
        }, intent);
    }

    std::optional<JogAction> JogController::next(const SimulationSnapshot &snapshot) {
        if(m_actionOutstanding) return std::nullopt;
        if(m_activeJog && !snapshot.jogging) {
            m_activeJog.reset();
            m_activeContinuous = false;
            m_activeVelocity = 0.0;
            m_pendingVelocity.reset();
            m_velocityRefreshRequested = false;
            m_stopRequested = false;
            m_stopSubmitted = false;
        }
        if(m_activeJog) {
            if(m_stopRequested && !m_stopSubmitted) {
                JogAction action = StopJogRequest { nextRequestId(), *m_activeJog };
                m_actionOutstanding = true;
                return action;
            }
            if(m_activeContinuous && m_pendingVelocity && m_velocityRefreshRequested) {
                auto velocity = velocityFor(*m_pendingVelocity);
                if(!velocity) {
                    m_error = std::move(velocity.error());
                    m_stopRequested = true;
                    return std::nullopt;
                }
                m_velocityRefreshRequested = false;
                JogAction action;
                if(std::abs(*velocity - m_activeVelocity) <= 1e-12)
                    action = RenewJogLeaseRequest { nextRequestId(), *m_activeJog };
                else
                    action = SetContinuousJogVelocityRequest {
                        nextRequestId(), *m_activeJog, *velocity,
                    };
                m_actionOutstanding = true;
                return action;
            }
            return std::nullopt;
        }
        m_stopRequested = false;
        m_stopSubmitted = false;
        if(!m_pending && !m_pendingVelocity) return std::nullopt;
        if(snapshot.jogging) {
            m_error = "pendant jog ignored because another jog owns motion";
            m_pending.reset();
            m_pendingVelocity.reset();
            m_velocityRefreshRequested = false;
            return std::nullopt;
        }
        if(snapshot.status == SimulationStatus::Running
           || snapshot.status == SimulationStatus::Paused
           || snapshot.status == SimulationStatus::Error) {
            m_error = "pendant jog ignored because the machine is not idle";
            m_pending.reset();
            m_pendingVelocity.reset();
            m_velocityRefreshRequested = false;
            return std::nullopt;
        }

        JogAction action;
        if(m_pending) {
            auto request = makeIncrementalRequest(*m_pending, snapshot);
            m_pending.reset();
            if(!request) {
                m_error = std::move(request.error());
                return std::nullopt;
            }
            action = *request;
        } else {
            auto request = makeContinuousRequest(*m_pendingVelocity, snapshot);
            if(!request) {
                m_error = std::move(request.error());
                m_pendingVelocity.reset();
                m_velocityRefreshRequested = false;
                return std::nullopt;
            }
            m_velocityRefreshRequested = false;
            action = *request;
        }
        m_actionOutstanding = true;
        return action;
    }

    void JogController::submitted(const JogAction &action, const bool accepted) {
        if(!m_actionOutstanding) {
            m_error = "operator jog submission acknowledgement had no outstanding action";
            return;
        }
        m_actionOutstanding = false;
        if(const auto *start = std::get_if<StartIncrementalJogRequest>(&action)) {
            if(accepted) {
                m_activeJog = start->jog;
                m_activeContinuous = false;
            }
            else m_error = "simulation rejected a pendant incremental jog";
        } else if(const auto *start = std::get_if<StartContinuousJogRequest>(&action)) {
            if(accepted) {
                m_activeJog = start->jog;
                m_activeContinuous = true;
                m_activeVelocity = start->signedVelocity;
            } else m_error = "simulation rejected a pendant velocity jog";
        } else if(const auto *update = std::get_if<SetContinuousJogVelocityRequest>(&action)) {
            if(accepted) m_activeVelocity = update->signedVelocity;
            else {
                m_error = "simulation rejected a pendant velocity update";
                m_stopRequested = true;
            }
        } else if(std::holds_alternative<RenewJogLeaseRequest>(action)) {
            if(!accepted) {
                m_error = "simulation rejected a pendant jog lease renewal";
                m_stopRequested = true;
            }
        } else if(const auto *stop = std::get_if<StopJogRequest>(&action)) {
            if(!m_activeJog || *m_activeJog != stop->jog) {
                m_error = "pendant stop acknowledgement did not match its active jog";
                return;
            }
            if(accepted) m_stopSubmitted = true;
            else m_error = "simulation rejected a pendant jog stop; it will be retried";
        }
    }

    std::expected<double, std::string>
    JogController::velocityFor(const PendingVelocity &pending) const {
        const auto selectedAxis = machineAxis(pending.axis);
        if(!selectedAxis) return std::unexpected("pendant selected an invalid axis");
        const auto axis = std::ranges::find(m_axes, *selectedAxis, &AxisConfiguration::axis);
        if(axis == m_axes.end())
            return std::unexpected(std::format(
                "pendant selected unconfigured axis {}", axisName(*selectedAxis)));
        auto velocityLimit = axis->maxVelocity;
        for(const auto id : axis->joints) {
            const auto joint = std::ranges::find(m_joints, id, &JointConfiguration::id);
            if(joint == m_joints.end())
                return std::unexpected(std::format(
                    "pendant axis {} references missing joint {}", axisName(*selectedAxis), id));
            const auto scale = std::abs(joint->coordinateScale);
            if(scale <= 1e-12)
                return std::unexpected(std::format(
                    "pendant axis {} references joint {} with an invalid coordinate scale",
                    axisName(*selectedAxis), id));
            velocityLimit = std::min(velocityLimit, joint->maxVelocity / scale);
        }
        const auto fraction = std::clamp(
            std::abs(static_cast<double>(pending.countsPerSecond))
                / m_velocity.fullScaleCountsPerSecond,
            0.0, 1.0);
        const auto sign = pending.countsPerSecond < 0 ? -1.0 : 1.0;
        return sign * velocityLimit * m_velocity.maxVelocityScale * fraction;
    }

    std::expected<StartContinuousJogRequest, std::string>
    JogController::makeContinuousRequest(const PendingVelocity &pending,
                                          const SimulationSnapshot &snapshot) {
        const auto selectedAxis = machineAxis(pending.axis);
        if(!selectedAxis) return std::unexpected("pendant selected an invalid axis");
        const auto axis = std::ranges::find(m_axes, *selectedAxis, &AxisConfiguration::axis);
        if(axis == m_axes.end())
            return std::unexpected(std::format(
                "pendant selected unconfigured axis {}", axisName(*selectedAxis)));

        JointMask joints = 0;
        auto velocityLimit = axis->maxVelocity;
        auto acceleration = axis->maxAcceleration;
        auto jerk = axis->maxJerk;
        for(const auto id : axis->joints) {
            joints |= JointMask { 1 } << id;
            const auto joint = std::ranges::find(m_joints, id, &JointConfiguration::id);
            if(joint == m_joints.end())
                return std::unexpected(std::format(
                    "pendant axis {} references missing joint {}", axisName(*selectedAxis), id));
            const auto scale = std::abs(joint->coordinateScale);
            if(scale <= 1e-12)
                return std::unexpected(std::format(
                    "pendant axis {} references joint {} with an invalid coordinate scale",
                    axisName(*selectedAxis), id));
            velocityLimit = std::min(velocityLimit, joint->maxVelocity / scale);
            acceleration = std::min(acceleration, joint->maxAcceleration / scale);
            jerk = std::min(jerk, joint->maxJerk / scale);
        }
        if(joints == 0) return std::unexpected("pendant selected an axis without configured joints");
        if((snapshot.homedJoints & joints) != joints)
            return std::unexpected(std::format(
                "pendant cannot jog unhomed axis {}", axisName(*selectedAxis)));

        const auto fraction = std::clamp(
            std::abs(static_cast<double>(pending.countsPerSecond))
                / m_velocity.fullScaleCountsPerSecond,
            0.0, 1.0);
        const auto sign = pending.countsPerSecond < 0 ? -1.0 : 1.0;
        const auto leaseTicksValue = std::ceil(m_velocity.leaseDuration / m_servoPeriod);
        const auto leaseTicks = static_cast<std::uint32_t>(std::clamp(
            leaseTicksValue, 1.0,
            static_cast<double>(std::numeric_limits<std::uint32_t>::max())));
        return StartContinuousJogRequest {
            .id = nextRequestId(),
            .jog = nextJogId(),
            .target = { JogTargetType::Axis, backendAxis(*selectedAxis), 0 },
            .signedVelocity = sign * velocityLimit * m_velocity.maxVelocityScale * fraction,
            .limits = {
                velocityLimit,
                std::min(acceleration, m_jogging.acceleration),
                std::min(jerk, m_jogging.jerk),
            },
            .stopLimits = { velocityLimit, acceleration, jerk },
            .travel = { axis->minimum, axis->maximum, true },
            .leaseTicks = leaseTicks,
        };
    }

    std::optional<std::string> JogController::takeError() {
        return std::exchange(m_error, std::nullopt);
    }

    std::expected<StartIncrementalJogRequest, std::string>
    JogController::makeIncrementalRequest(const PendingIncrement &pending,
                                          const SimulationSnapshot &snapshot) {
        const auto selectedAxis = machineAxis(pending.axis);
        if(!selectedAxis) return std::unexpected("pendant selected an invalid axis");
        const auto axis = std::ranges::find(m_axes, *selectedAxis, &AxisConfiguration::axis);
        if(axis == m_axes.end())
            return std::unexpected(std::format("pendant selected unconfigured axis {}", axisName(*selectedAxis)));

        JointMask joints = 0;
        auto velocity = axis->maxVelocity;
        auto acceleration = axis->maxAcceleration;
        auto jerk = axis->maxJerk;
        for(const auto id : axis->joints) {
            joints |= JointMask { 1 } << id;
            const auto joint = std::ranges::find(m_joints, id, &JointConfiguration::id);
            if(joint == m_joints.end())
                return std::unexpected(std::format(
                    "pendant axis {} references missing joint {}", axisName(*selectedAxis), id));
            const auto scale = std::abs(joint->coordinateScale);
            if(scale <= 1e-12)
                return std::unexpected(std::format(
                    "pendant axis {} references joint {} with an invalid coordinate scale",
                    axisName(*selectedAxis), id));
            velocity = std::min(velocity, joint->maxVelocity / scale);
            acceleration = std::min(acceleration, joint->maxAcceleration / scale);
            jerk = std::min(jerk, joint->maxJerk / scale);
        }
        if(joints == 0) return std::unexpected("pendant selected an axis without configured joints");
        if((snapshot.homedJoints & joints) != joints)
            return std::unexpected(std::format(
                "pendant cannot jog unhomed axis {}", axisName(*selectedAxis)));

        const JogMotionLimits stopLimits { velocity, acceleration, jerk };
        return StartIncrementalJogRequest {
            .id = nextRequestId(),
            .jog = nextJogId(),
            .target = { JogTargetType::Axis, backendAxis(*selectedAxis), 0 },
            .distance = pending.distance,
            .velocity = velocity,
            .limits = stopLimits,
            .stopLimits = stopLimits,
            .travel = { axis->minimum, axis->maximum, true },
        };
    }

    RequestId JogController::nextRequestId() {
        if(m_nextRequest == std::numeric_limits<RequestId>::max()) m_nextRequest = RequestId { 1 } << 62;
        return m_nextRequest++;
    }

    JogId JogController::nextJogId() {
        if(m_nextJog == std::numeric_limits<JogId>::max()) m_nextJog = JogId { 1 } << 62;
        return m_nextJog++;
    }
}
