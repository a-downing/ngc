#pragma once

#include <expected>
#include <optional>
#include <string>
#include <variant>

#include "machine/MachineConfiguration.h"
#include "machine/SimulationPresentation.h"
#include "pendant/PendantIntent.h"

namespace ngc::operator_control {
    using JogAction = std::variant<StartIncrementalJogRequest, StartContinuousJogRequest,
                                   RenewJogLeaseRequest, SetContinuousJogVelocityRequest,
                                   StopJogRequest>;

    // NRT ownership and request-building layer shared by physical operator
    // inputs. It never calls a backend and requires submission acknowledgement.
    class JogController {
    public:
        explicit JogController(const MachineConfiguration &configuration);

        void consume(const pendant::Intent &intent);
        std::optional<JogAction> next(const SimulationSnapshot &snapshot);
        void submitted(const JogAction &action, bool accepted);
        std::optional<std::string> takeError();

    private:
        struct PendingIncrement {
            pendant::Axis axis = pendant::Axis::X;
            double distance = 0.0;
        };

        struct PendingVelocity {
            pendant::Axis axis = pendant::Axis::X;
            std::int32_t countsPerSecond = 0;
        };

        std::expected<StartIncrementalJogRequest, std::string>
        makeIncrementalRequest(const PendingIncrement &pending,
                               const SimulationSnapshot &snapshot);
        std::expected<StartContinuousJogRequest, std::string>
        makeContinuousRequest(const PendingVelocity &pending,
                              const SimulationSnapshot &snapshot);
        std::expected<double, std::string>
        velocityFor(const PendingVelocity &pending) const;
        RequestId nextRequestId();
        JogId nextJogId();

        JoggingConfiguration m_jogging;
        PendantStepConfiguration m_step;
        PendantVelocityConfiguration m_velocity;
        double m_servoPeriod = 0.0;
        std::vector<AxisConfiguration> m_axes;
        std::vector<JointConfiguration> m_joints;
        std::optional<PendingIncrement> m_pending;
        std::optional<PendingVelocity> m_pendingVelocity;
        std::optional<JogId> m_activeJog;
        bool m_activeContinuous = false;
        double m_activeVelocity = 0.0;
        bool m_velocityRefreshRequested = false;
        bool m_stopRequested = false;
        bool m_stopSubmitted = false;
        bool m_actionOutstanding = false;
        RequestId m_nextRequest = RequestId { 1 } << 62;
        JogId m_nextJog = JogId { 1 } << 62;
        std::optional<std::string> m_error;
    };
}
