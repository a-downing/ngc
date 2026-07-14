#include "machine/MockMotionBackend.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <type_traits>

#include <ruckig/ruckig.hpp>

#include "machine/SpscChannel.h"

namespace ngc {
    namespace {
        constexpr std::size_t PLAN_CAPACITY = 8;
        constexpr std::size_t CONTROL_CAPACITY = 16;
        // One drain interval can retire the complete plan horizon. Each chunk can
        // produce accepted, branch, retired, and probe/held records, in addition
        // to control acknowledgements. Keep enough return capacity for that
        // worst-case burst so the bounded forward horizon cannot deadlock itself.
        constexpr std::size_t EVENT_CAPACITY = 64;
        constexpr std::size_t SNAPSHOT_CAPACITY = 4;
        constexpr std::size_t SYNTHETIC_INPUT_CAPACITY = 16;

        struct SyntheticInputTransition {
            TriggeredMoveId move = 0;
            JointId joint = MAX_JOINTS;
            position_t position{};
            double jointPosition = 0.0;
        };

        double magnitude(const position_t &value) {
            return std::sqrt(value.x*value.x + value.y*value.y + value.z*value.z
                + value.a*value.a + value.b*value.b + value.c*value.c);
        }

        double dot(const position_t &left, const position_t &right) {
            return left.x*right.x + left.y*right.y + left.z*right.z
                + left.a*right.a + left.b*right.b + left.c*right.c;
        }

        position_t scaled(const position_t &value, const double scale) {
            return { value.x*scale, value.y*scale, value.z*scale,
                     value.a*scale, value.b*scale, value.c*scale };
        }

        position_t evaluate(const AxisPolynomialSpan &span, const double u) {
            const auto component = [&](const double position_t::*member) {
                return ((span.a.*member * u + span.b.*member) * u + span.c.*member) * u + span.d.*member;
            };
            return { component(&position_t::x), component(&position_t::y), component(&position_t::z),
                     component(&position_t::a), component(&position_t::b), component(&position_t::c) };
        }

        position_t derivative(const AxisPolynomialSpan &span, const double u) {
            const auto component = [&](const double position_t::*member) {
                return (3.0 * span.a.*member * u * u + 2.0 * span.b.*member * u + span.c.*member)
                    * span.inverseDuration;
            };
            return { component(&position_t::x), component(&position_t::y), component(&position_t::z),
                     component(&position_t::a), component(&position_t::b), component(&position_t::c) };
        }

        position_t secondDerivative(const AxisPolynomialSpan &span, const double u) {
            const auto component = [&](const double position_t::*member) {
                return (6.0 * span.a.*member * u + 2.0 * span.b.*member) * span.inverseDurationSquared;
            };
            return { component(&position_t::x), component(&position_t::y), component(&position_t::z),
                     component(&position_t::a), component(&position_t::b), component(&position_t::c) };
        }
    }

    class MockMotionBackend::Impl {
        struct PlanSlot {
            std::atomic<bool> occupied{false};
            ExecutionItem item{};
        };
        std::array<PlanSlot, PLAN_CAPACITY> m_planSlots;
        SpscChannel<std::uint8_t, PLAN_CAPACITY> m_plans;
        SpscChannel<ControlRequest, CONTROL_CAPACITY> m_controls;
        SpscChannel<ExecutionEvent, EVENT_CAPACITY> m_events;
        SpscChannel<ExecutionSnapshot, SNAPSHOT_CAPACITY> m_snapshots;
        SpscChannel<SyntheticInputTransition, SYNTHETIC_INPUT_CAPACITY> m_syntheticInputs;
        FixedArray<SyntheticInputTransition, SYNTHETIC_INPUT_CAPACITY> m_pendingSyntheticInputs;
        ExecutionSnapshot m_snapshot;
        std::optional<std::uint8_t> m_active;
        bool m_stopping = false;
        std::uint32_t m_span = 0;
        std::uint32_t m_nextEvent = 0;
        double m_spanElapsed = 0.0;
        std::uint64_t m_continuationSequence = 0;
        MockTrajectorySnapshot m_trajectoryDiagnostics;
        struct TriggeredRuntime {
            position_t start{};
            position_t direction{};
            double length = 0.0;
            double elapsed = 0.0;
            bool stopping = false;
            TriggeredMoveStatus completionStatus = TriggeredMoveStatus::ReachedTarget;
            MotionState triggerState{};
            ruckig::Trajectory<1> trajectory;
        } m_triggered;
        struct JointRuntime {
            double start = 0.0;
            double target = 0.0;
            double direction = 0.0;
            double elapsed = 0.0;
            bool stopping = false;
            bool finished = false;
            bool triggered = false;
            double triggerPosition = 0.0;
            double triggerVelocity = 0.0;
            double triggerAcceleration = 0.0;
            double debounceElapsed = 0.0;
            ruckig::Trajectory<1> trajectory;
        };
        std::array<JointRuntime, MAX_JOINTS> m_jointRuntime;
        JointMask m_triggeredJoints = 0;
        struct JogRuntime {
            JogId id = 0;
            JogTarget target{};
            JogMotionLimits limits{};
            JogMotionLimits stopLimits{};
            JogTravelRange travel{};
            JointVector jointOrigin{};
            double axisOrigin = 0.0;
            double position = 0.0;
            double velocity = 0.0;
            double acceleration = 0.0;
            double elapsed = 0.0;
            double cruisePosition = 0.0;
            double cruiseVelocity = 0.0;
            std::uint32_t leaseTicks = 0;
            std::uint32_t leasePeriod = 0;
            bool continuous = false;
            bool cruising = false;
            bool stopping = false;
            JogStopReason stopReason = JogStopReason::RequestedStop;
            ruckig::Trajectory<1> trajectory;
        };
        std::optional<JogRuntime> m_jog;

    public:
        PublishResult publish(const ExecutionItem &item) noexcept {
            const auto valid = std::visit([](const auto &value) {
                using T = std::decay_t<decltype(value)>;
                if constexpr(std::same_as<T, PlanChunk>)
                    return value.normalMotion.size != 0 && value.stopTail.size != 0
                        && value.epoch != 0 && value.id != 0;
                else if constexpr(std::same_as<T, TriggeredMove>)
                    return value.epoch != 0 && value.id != 0 && value.moveId != 0
                        && magnitude(value.target) < std::numeric_limits<double>::infinity()
                        && magnitude(value.limits.velocity) > 0.0
                        && magnitude(value.limits.acceleration) > 0.0
                        && magnitude(value.limits.jerk) > 0.0;
                else {
                    if(value.epoch == 0 || value.id == 0 || value.moveId == 0 || value.joints == 0) return false;
                    for(JointId joint = 0; joint < MAX_JOINTS; ++joint) {
                        if((value.joints & (JointMask{1} << joint)) == 0) continue;
                        if(!std::isfinite(value.target[joint]) || value.limits.velocity[joint] <= 0.0
                           || value.limits.acceleration[joint] <= 0.0 || value.limits.jerk[joint] <= 0.0)
                            return false;
                    }
                    JointMask boundJoints = 0;
                    for(const auto &trigger : value.triggers) {
                        if(trigger.joint >= MAX_JOINTS
                           || (value.joints & (JointMask{1} << trigger.joint)) == 0
                           || (boundJoints & (JointMask{1} << trigger.joint)) != 0
                           || !std::isfinite(trigger.debounce) || trigger.debounce < 0.0) return false;
                        boundJoints |= JointMask{1} << trigger.joint;
                    }
                    if(value.triggerRequired && boundJoints != value.joints) return false;
                    return true;
                }
            }, item);
            if(!valid) return PublishResult::Invalid;
            for(std::uint8_t index = 0; index < m_planSlots.size(); ++index) {
                bool expected = false;
                if(!m_planSlots[index].occupied.compare_exchange_strong(expected, true, std::memory_order_acquire)) continue;
                m_planSlots[index].item = item;
                if(m_plans.tryPush(index)) return PublishResult::Published;
                m_planSlots[index].occupied.store(false, std::memory_order_release);
                return PublishResult::Full;
            }
            return PublishResult::Full;
        }

        SubmitResult submit(const ControlRequest &request) noexcept {
            return m_controls.tryPush(request) ? SubmitResult::Submitted : SubmitResult::Full;
        }

        bool takeEvent(ExecutionEvent &event) noexcept { return m_events.tryPop(event); }
        bool takeSnapshot(ExecutionSnapshot &snapshot) noexcept { return m_snapshots.tryPop(snapshot); }
        bool configureSyntheticInput(const SyntheticInputTransition &input) noexcept {
            return m_syntheticInputs.tryPush(input);
        }
        void clearDiagnostics() { m_trajectoryDiagnostics = {}; }
        MockTrajectorySnapshot diagnostics() const { return m_trajectoryDiagnostics; }
        void advance(double seconds, const bool shouldPublishSnapshot = true) {
            serviceControls();
            SyntheticInputTransition input;
            while(m_syntheticInputs.tryPop(input)) {
                bool replaced = false;
                for(auto &pending : m_pendingSyntheticInputs)
                    if(pending.move == input.move && pending.joint == input.joint) {
                    pending = input; replaced = true; break;
                }
                if(!replaced) (void)m_pendingSyntheticInputs.push(input);
            }
            seconds = std::max(seconds, 0.0);
            if(m_jog) {
                advanceJog(seconds);
                if(shouldPublishSnapshot || !m_jog) publishSnapshot();
                return;
            }
            if(m_snapshot.state != BackendState::Running) {
                if(shouldPublishSnapshot) publishSnapshot();
                return;
            }
            if(!m_active) activateNext();
            const auto executedActiveMotion = m_active.has_value();
            while(m_active && seconds > 0.0) {
                if(std::holds_alternative<TriggeredMove>(activeItem())) {
                    advanceTriggered(seconds);
                    continue;
                }
                if(std::holds_alternative<TriggeredJointMove>(activeItem())) {
                    advanceTriggeredJoints(seconds);
                    continue;
                }
                const auto &span = currentSpan();
                const auto remaining = std::max(span.duration - m_spanElapsed, 0.0);
                const auto consumed = std::min(seconds, remaining);
                seconds -= consumed;
                m_spanElapsed += consumed;
                const auto u = span.duration > 0.0 ? std::clamp(m_spanElapsed * span.inverseDuration, 0.0, 1.0) : 1.0;
                m_snapshot.activeSpan = span.id;
                m_snapshot.spanProgress = u;
                m_snapshot.commanded = { evaluate(span, u), derivative(span, u), secondDerivative(span, u) };
                m_snapshot.feedback = m_snapshot.commanded;
                recordCalculatedPosition();
                if(m_spanElapsed + 1e-12 < span.duration) break;
                completeSpan();
            }
            // Always publish terminal/held state even inside a decimated batch so
            // NRT cannot observe completion before its matching final snapshot.
            if(shouldPublishSnapshot || (executedActiveMotion && !m_active)
               || m_snapshot.state != BackendState::Running)
                publishSnapshot();
        }

        bool advanceTick(const double seconds,const bool shouldPublishSnapshot) {
            const auto before=m_continuationSequence;
            advance(seconds,shouldPublishSnapshot);
            return m_continuationSequence!=before;
        }

        void runUntilIdle() {
            runUntilIdle(3600.0);
        }

        void runUntilIdle(const double tickSeconds) {
            const auto step = tickSeconds > 0.0 ? tickSeconds : 3600.0;
            for(std::size_t guard = 0; guard < 100000000 && m_snapshot.state != BackendState::Faulted
                && (m_active || m_jog || !m_plans.empty() || !m_controls.empty()); ++guard) {
                advance(step, false);
                // STOP is irrevocable. Queued descendants are now stale and may
                // remain in the forward channel until NRT observes HELD and sends
                // the recovery reset. Spinning on them cannot make progress.
                if(m_snapshot.state == BackendState::Held && m_controls.empty()) break;
            }
        }

    private:
        ExecutionItem &activeItem() { return m_planSlots[*m_active].item; }
        const ExecutionItem &activeItem() const { return m_planSlots[*m_active].item; }
        PlanChunk &activeChunk() { return std::get<PlanChunk>(activeItem()); }
        const PlanChunk &activeChunk() const { return std::get<PlanChunk>(activeItem()); }
        static EpochId itemEpoch(const ExecutionItem &item) {
            return std::visit([](const auto &value) { return value.epoch; }, item);
        }
        static ChunkId itemId(const ExecutionItem &item) {
            return std::visit([](const auto &value) { return value.id; }, item);
        }
        static BranchSequence itemPredecessor(const ExecutionItem &item) {
            return std::visit([](const auto &value) { return value.predecessorBranch; }, item);
        }
        static BranchSequence itemBranch(const ExecutionItem &item) {
            return std::visit([](const auto &value) { return value.branch; }, item);
        }

        const AxisPolynomialSpan &currentSpan() const {
            const auto &chunk = activeChunk();
            return m_stopping ? chunk.stopTail[m_span] : chunk.normalMotion[m_span];
        }
        void release(const std::uint8_t index) { m_planSlots[index].occupied.store(false, std::memory_order_release); }

        void publishSnapshot() { (void)m_snapshots.tryPush(m_snapshot); }

        static double &axisComponent(position_t &position, const AxisId axis) {
            switch(axis) {
                case AxisId::X: return position.x;
                case AxisId::Y: return position.y;
                case AxisId::Z: return position.z;
                case AxisId::A: return position.a;
                case AxisId::B: return position.b;
                case AxisId::C: return position.c;
            }
            return position.x;
        }

        static double axisComponent(const position_t &position, const AxisId axis) {
            auto copy = position;
            return axisComponent(copy, axis);
        }

        double jogCoordinate(const JogTarget &target) const {
            if(target.type == JogTargetType::Axis) return axisComponent(m_snapshot.commanded.position, target.axis);
            for(JointId joint = 0; joint < MAX_JOINTS; ++joint)
                if((target.joints & (JointMask{1} << joint)) != 0)
                    return m_snapshot.commandedJoints.position[joint];
            return 0.0;
        }

        double jogVelocity(const JogTarget &target) const {
            if(target.type == JogTargetType::Axis) return axisComponent(m_snapshot.commanded.velocity, target.axis);
            for(JointId joint = 0; joint < MAX_JOINTS; ++joint)
                if((target.joints & (JointMask{1} << joint)) != 0)
                    return m_snapshot.commandedJoints.velocity[joint];
            return 0.0;
        }

        double jogAcceleration(const JogTarget &target) const {
            if(target.type == JogTargetType::Axis) return axisComponent(m_snapshot.commanded.acceleration, target.axis);
            for(JointId joint = 0; joint < MAX_JOINTS; ++joint)
                if((target.joints & (JointMask{1} << joint)) != 0)
                    return m_snapshot.commandedJoints.acceleration[joint];
            return 0.0;
        }

        bool validJogTarget(const JogTarget &target) const {
            if(target.type == JogTargetType::Axis) return target.joints == 0;
            if(target.joints == 0) return false;
            if(target.type == JogTargetType::Joint)
                return (target.joints & (target.joints - 1)) == 0;
            return true;
        }

        bool validJogLimits(const JogMotionLimits &limits) const {
            return std::isfinite(limits.velocity) && limits.velocity > 0.0
                && std::isfinite(limits.acceleration) && limits.acceleration > 0.0
                && std::isfinite(limits.jerk) && limits.jerk > 0.0;
        }

        void applyJogState() {
            const auto &jog = *m_jog;
            if(jog.target.type == JogTargetType::Axis) {
                axisComponent(m_snapshot.commanded.position, jog.target.axis) = jog.axisOrigin + jog.position;
                axisComponent(m_snapshot.commanded.velocity, jog.target.axis) = jog.velocity;
                axisComponent(m_snapshot.commanded.acceleration, jog.target.axis) = jog.acceleration;
                m_snapshot.feedback = m_snapshot.commanded;
                return;
            }
            for(JointId joint = 0; joint < MAX_JOINTS; ++joint) {
                if((jog.target.joints & (JointMask{1} << joint)) == 0) continue;
                m_snapshot.commandedJoints.position[joint] = jog.jointOrigin[joint] + jog.position;
                m_snapshot.commandedJoints.velocity[joint] = jog.velocity;
                m_snapshot.commandedJoints.acceleration[joint] = jog.acceleration;
            }
            m_snapshot.feedbackJoints = m_snapshot.commandedJoints;
        }

        bool calculateJogPosition(const double distance, const double velocity) {
            auto &jog = *m_jog;
            ruckig::InputParameter<1> input;
            input.current_position = {0.0};
            input.current_velocity = {jogVelocity(jog.target)};
            input.current_acceleration = {jogAcceleration(jog.target)};
            input.target_position = {distance};
            input.target_velocity = {0.0};
            input.target_acceleration = {0.0};
            input.max_velocity = {std::min(velocity, jog.limits.velocity)};
            input.max_acceleration = {jog.limits.acceleration};
            input.max_jerk = {jog.limits.jerk};
            ruckig::Ruckig<1> generator;
            return generator.calculate(input, jog.trajectory) == ruckig::Result::Working;
        }

        bool calculateJogVelocity(const double targetVelocity) {
            auto &jog = *m_jog;
            ruckig::InputParameter<1> input;
            input.control_interface = ruckig::ControlInterface::Velocity;
            input.current_position = {jog.position};
            input.current_velocity = {jog.velocity};
            input.current_acceleration = {jog.acceleration};
            input.target_position = {jog.position};
            input.target_velocity = {targetVelocity};
            input.target_acceleration = {0.0};
            const auto &limits = jog.stopping ? jog.stopLimits : jog.limits;
            input.max_velocity = {std::max(limits.velocity, std::abs(jog.velocity))};
            input.max_acceleration = {limits.acceleration};
            input.max_jerk = {limits.jerk};
            ruckig::Ruckig<1> generator;
            return generator.calculate(input, jog.trajectory) == ruckig::Result::Working;
        }

        template<typename Request>
        bool initializeJog(const Request &request) {
            if(m_snapshot.state != BackendState::Held || m_active || m_jog || request.jog == 0
               || !validJogTarget(request.target) || !validJogLimits(request.limits)
               || !validJogLimits(request.stopLimits)) return false;
            m_jog.emplace();
            auto &jog = *m_jog;
            jog.id = request.jog;
            jog.target = request.target;
            jog.limits = request.limits;
            jog.stopLimits = request.stopLimits;
            jog.travel = request.travel;
            jog.axisOrigin = jogCoordinate(request.target);
            jog.jointOrigin = m_snapshot.commandedJoints.position;
            jog.velocity = jogVelocity(request.target);
            jog.acceleration = jogAcceleration(request.target);
            bool calculated = false;
            if constexpr(std::same_as<Request, StartContinuousJogRequest>) {
                if(!std::isfinite(request.signedVelocity) || std::abs(request.signedVelocity) <= 1e-12
                   || request.leaseTicks == 0) { m_jog.reset(); return false; }
                jog.continuous = true;
                jog.leaseTicks = request.leaseTicks;
                jog.leasePeriod = request.leaseTicks;
                jog.cruiseVelocity = std::clamp(request.signedVelocity,
                                                -jog.limits.velocity, jog.limits.velocity);
                if(request.travel.enabled) {
                    const auto coordinate = jogCoordinate(request.target);
                    const auto target = jog.cruiseVelocity < 0.0 ? request.travel.minimum : request.travel.maximum;
                    calculated = std::isfinite(target) && ((jog.cruiseVelocity < 0.0 && target <= coordinate)
                        || (jog.cruiseVelocity > 0.0 && target >= coordinate))
                        && calculateJogPosition(target - coordinate, std::abs(jog.cruiseVelocity));
                } else {
                    calculated = calculateJogVelocity(jog.cruiseVelocity);
                }
            } else {
                if(!std::isfinite(request.distance) || std::abs(request.distance) <= 1e-12
                   || !std::isfinite(request.velocity) || request.velocity <= 0.0) {
                    m_jog.reset(); return false;
                }
                auto distance = request.distance;
                if(request.travel.enabled) {
                    const auto coordinate = jogCoordinate(request.target);
                    distance = std::clamp(coordinate + distance, request.travel.minimum,
                                          request.travel.maximum) - coordinate;
                }
                calculated = std::abs(distance) > 1e-12
                    && calculateJogPosition(distance, request.velocity);
            }
            if(!calculated) { m_jog.reset(); return false; }
            jog.elapsed = 0.0;
            m_snapshot.state = BackendState::Running;
            m_snapshot.activeJoints = request.target.type == JogTargetType::Axis ? 0 : request.target.joints;
            return true;
        }

        bool beginJogStop(const JogStopReason reason) {
            if(!m_jog || m_jog->stopping) return false;
            m_jog->stopping = true;
            m_jog->cruising = false;
            m_jog->stopReason = reason;
            m_jog->elapsed = 0.0;
            if(std::abs(m_jog->velocity) <= 1e-12 && std::abs(m_jog->acceleration) <= 1e-12) {
                completeJog();
                return true;
            }
            if(!calculateJogVelocity(0.0)) {
                m_snapshot.state = BackendState::Faulted;
                m_snapshot.faultCode = 3;
                emit(BackendFault { 3 });
                return false;
            }
            return true;
        }

        void completeJog() {
            const auto jog = *m_jog;
            m_snapshot.activeJoints = 0;
            m_snapshot.state = BackendState::Held;
            emit(JogStopped { jog.id, jog.target, jog.stopReason,
                              m_snapshot.commanded, m_snapshot.commandedJoints });
            m_jog.reset();
        }

        void advanceJog(double seconds) {
            if(!m_jog || seconds <= 0.0) return;
            if(m_jog->continuous && !m_jog->stopping && m_jog->leaseTicks != 0) {
                --m_jog->leaseTicks;
                if(m_jog->leaseTicks == 0) {
                    (void)beginJogStop(JogStopReason::LeaseExpired);
                    if(!m_jog) return;
                }
            }
            auto &jog = *m_jog;
            const auto duration = jog.trajectory.get_duration();
            if(jog.cruising) {
                jog.position += jog.cruiseVelocity * seconds;
                jog.velocity = jog.cruiseVelocity;
                jog.acceleration = 0.0;
                applyJogState();
                return;
            }
            const auto consumed = std::min(seconds, std::max(duration - jog.elapsed, 0.0));
            jog.elapsed += consumed;
            jog.trajectory.at_time(jog.elapsed, jog.position, jog.velocity, jog.acceleration);
            applyJogState();
            if(jog.elapsed + 1e-12 < duration) return;
            const auto remaining = seconds - consumed;
            if(jog.stopping) {
                jog.velocity = 0.0;
                jog.acceleration = 0.0;
                applyJogState();
                completeJog();
            } else if(jog.continuous && !jog.travel.enabled) {
                jog.cruising = true;
                if(remaining > 0.0) advanceJog(remaining);
            } else {
                jog.stopReason = jog.continuous ? JogStopReason::LimitReached
                                                : JogStopReason::TargetReached;
                completeJog();
            }
        }

        MotionState triggeredStateAt(const double elapsed, const position_t &origin) const {
            double position = 0.0, velocity = 0.0, acceleration = 0.0;
            m_triggered.trajectory.at_time(elapsed, position, velocity, acceleration);
            return { origin + scaled(m_triggered.direction, position),
                     scaled(m_triggered.direction, velocity),
                     scaled(m_triggered.direction, acceleration) };
        }

        bool initializeTriggered() {
            const auto &move = std::get<TriggeredMove>(activeItem());
            m_triggered = {};
            m_triggered.start = m_snapshot.commanded.position;
            const auto delta = move.target - m_triggered.start;
            m_triggered.length = magnitude(delta);
            if(m_triggered.length <= 1e-12) {
                m_triggered.direction = {};
                return true;
            }
            m_triggered.direction = scaled(delta, 1.0 / m_triggered.length);
            const auto velocityLimit = magnitude(move.limits.velocity);
            const auto accelerationLimit = magnitude(move.limits.acceleration);
            const auto jerkLimit = magnitude(move.limits.jerk);
            ruckig::InputParameter<1> input;
            input.current_position = {0.0};
            input.current_velocity = {dot(m_snapshot.commanded.velocity, m_triggered.direction)};
            input.current_acceleration = {dot(m_snapshot.commanded.acceleration, m_triggered.direction)};
            input.target_position = {m_triggered.length};
            input.target_velocity = {0.0};
            input.target_acceleration = {0.0};
            input.max_velocity = {velocityLimit};
            input.max_acceleration = {accelerationLimit};
            input.max_jerk = {jerkLimit};
            ruckig::Ruckig<1> generator;
            if(generator.calculate(input, m_triggered.trajectory) != ruckig::Result::Working) return false;
            return true;
        }

        std::optional<double> syntheticTransitionDistance(const TriggeredMoveId move) const {
            for(const auto &input : m_pendingSyntheticInputs) {
                if(input.move == move)
                    return dot(input.position - m_triggered.start, m_triggered.direction);
            }
            return std::nullopt;
        }

        void removeSyntheticInput(const TriggeredMoveId move) {
            for(std::uint32_t index = 0; index < m_pendingSyntheticInputs.size; ++index) {
                if(m_pendingSyntheticInputs[index].move != move) continue;
                m_pendingSyntheticInputs[index] = m_pendingSyntheticInputs[m_pendingSyntheticInputs.size - 1];
                --m_pendingSyntheticInputs.size;
                return;
            }
        }

        bool beginTriggeredStop(const TriggeredMoveStatus status) {
            const auto &move = std::get<TriggeredMove>(activeItem());
            m_triggered.triggerState = m_snapshot.commanded;
            m_triggered.completionStatus = status;
            m_triggered.stopping = true;
            m_triggered.elapsed = 0.0;
            const auto scalarVelocity = dot(m_snapshot.commanded.velocity, m_triggered.direction);
            const auto scalarAcceleration = dot(m_snapshot.commanded.acceleration, m_triggered.direction);
            if(std::abs(scalarVelocity) <= 1e-12 && std::abs(scalarAcceleration) <= 1e-12) return true;
            ruckig::InputParameter<1> input;
            input.control_interface = ruckig::ControlInterface::Velocity;
            input.current_position = {0.0};
            input.current_velocity = {scalarVelocity};
            input.current_acceleration = {scalarAcceleration};
            input.target_position = {0.0};
            input.target_velocity = {0.0};
            input.target_acceleration = {0.0};
            input.max_velocity = {std::max(magnitude(move.limits.velocity), std::abs(scalarVelocity))};
            input.max_acceleration = {magnitude(move.limits.acceleration)};
            input.max_jerk = {magnitude(move.limits.jerk)};
            ruckig::Ruckig<1> generator;
            return generator.calculate(input, m_triggered.trajectory) == ruckig::Result::Working;
        }

        void completeTriggered(const TriggeredMoveStatus status) {
            const auto move = std::get<TriggeredMove>(activeItem());
            const auto stopped = m_snapshot.commanded;
            const auto trigger = status == TriggeredMoveStatus::Triggered
                ? m_triggered.triggerState : stopped;
            emit(TriggeredMoveCompleted { move.epoch, move.moveId, status, trigger, stopped });
            emit(BranchSelected { move.epoch, move.branch, BranchChoice::Stop, 0 });
            emit(ChunkRetired { move.epoch, move.id });
            m_snapshot.state = BackendState::Held;
            m_snapshot.lastBranch = move.branch;
            emit(BackendHeld { move.epoch, stopped, move.cursor });
            removeSyntheticInput(move.moveId);
            release(*m_active);
            m_active.reset();
        }

        void faultTriggered() {
            m_snapshot.state = BackendState::Faulted;
            m_snapshot.faultCode = 2;
            emit(BackendFault { 2 });
        }

        void recordTriggeredPosition(const bool stopTail) {
            const auto &move = std::get<TriggeredMove>(activeItem());
            if(m_trajectoryDiagnostics.spans.empty()
               || m_trajectoryDiagnostics.spans.back().epoch != move.epoch
               || m_trajectoryDiagnostics.spans.back().chunk != move.id
               || m_trajectoryDiagnostics.spans.back().stopTail != stopTail) {
                ExecutedTrajectorySpan executed { .epoch = move.epoch, .chunk = move.id,
                    .span = move.id, .stopTail = stopTail, .positions = {} };
                executed.positions.push_back(stopTail
                    ? m_triggered.triggerState.position : m_triggered.start);
                m_trajectoryDiagnostics.spans.push_back(std::move(executed));
            }
            m_trajectoryDiagnostics.spans.back().positions.push_back(m_snapshot.commanded.position);
            ++m_trajectoryDiagnostics.revision;
        }

        void advanceTriggered(double &seconds) {
            const auto &move = std::get<TriggeredMove>(activeItem());
            if(m_triggered.length <= 1e-12 && !m_triggered.stopping) {
                m_snapshot.commanded.position = move.target;
                m_snapshot.commanded.velocity = {};
                m_snapshot.commanded.acceleration = {};
                m_snapshot.feedback = m_snapshot.commanded;
                completeTriggered(TriggeredMoveStatus::ReachedTarget);
                return;
            }
            const auto duration = m_triggered.trajectory.get_duration();
            const auto consumed = std::min(seconds, std::max(duration - m_triggered.elapsed, 0.0));
            m_triggered.elapsed += consumed;
            const auto origin = m_triggered.stopping
                ? m_triggered.triggerState.position : m_triggered.start;
            m_snapshot.commanded = triggeredStateAt(m_triggered.elapsed, origin);
            m_snapshot.feedback = m_snapshot.commanded;
            m_snapshot.spanProgress = duration > 0.0
                ? std::clamp(m_triggered.elapsed / duration, 0.0, 1.0) : 1.0;
            recordTriggeredPosition(m_triggered.stopping);
            seconds -= consumed;
            if(!m_triggered.stopping) {
                const auto transition = syntheticTransitionDistance(move.moveId);
                const auto progress = dot(m_snapshot.commanded.position - m_triggered.start,
                                          m_triggered.direction);
                if(transition && progress + 1e-12 >= *transition) {
                    if(!beginTriggeredStop(TriggeredMoveStatus::Triggered)) { faultTriggered(); return; }
                    if(m_triggered.trajectory.get_duration() <= 1e-12)
                        completeTriggered(TriggeredMoveStatus::Triggered);
                    return;
                }
                if(m_triggered.elapsed + 1e-12 >= duration) {
                    m_snapshot.commanded.position = move.target;
                    m_snapshot.commanded.velocity = {};
                    m_snapshot.commanded.acceleration = {};
                    m_snapshot.feedback = m_snapshot.commanded;
                    completeTriggered(TriggeredMoveStatus::ReachedTarget);
                }
            } else if(m_triggered.elapsed + 1e-12 >= duration) {
                m_snapshot.commanded.velocity = {};
                m_snapshot.commanded.acceleration = {};
                m_snapshot.feedback = m_snapshot.commanded;
                completeTriggered(m_triggered.completionStatus);
            }
        }

        std::optional<double> syntheticJointTransition(const TriggeredMoveId move, const JointId joint) const {
            for(const auto &input : m_pendingSyntheticInputs)
                if(input.move == move && input.joint == joint) return input.jointPosition;
            return std::nullopt;
        }

        double jointTriggerDebounce(const TriggeredJointMove &move, const JointId joint) const {
            for(const auto &trigger : move.triggers)
                if(trigger.joint == joint) return trigger.debounce;
            return 0.0;
        }

        bool calculateJointApproach(const TriggeredJointMove &move, const JointId joint) {
            auto &runtime = m_jointRuntime[joint];
            runtime = {};
            runtime.start = m_snapshot.commandedJoints.position[joint];
            runtime.target = move.targetMode == JointTargetMode::Relative
                ? runtime.start + move.target[joint] : move.target[joint];
            const auto distance = runtime.target - runtime.start;
            runtime.direction = distance < 0.0 ? -1.0 : 1.0;
            if(std::abs(distance) <= 1e-12) { runtime.finished = true; return true; }
            ruckig::InputParameter<1> input;
            input.current_position = {runtime.start};
            input.current_velocity = {m_snapshot.commandedJoints.velocity[joint]};
            input.current_acceleration = {m_snapshot.commandedJoints.acceleration[joint]};
            input.target_position = {runtime.target};
            input.target_velocity = {0.0};
            input.target_acceleration = {0.0};
            input.max_velocity = {move.limits.velocity[joint]};
            input.max_acceleration = {move.limits.acceleration[joint]};
            input.max_jerk = {move.limits.jerk[joint]};
            ruckig::Ruckig<1> generator;
            return generator.calculate(input, runtime.trajectory) == ruckig::Result::Working;
        }

        bool initializeTriggeredJoints() {
            const auto &move = std::get<TriggeredJointMove>(activeItem());
            m_triggeredJoints = 0;
            m_snapshot.activeJoints = move.joints;
            for(JointId joint = 0; joint < MAX_JOINTS; ++joint) {
                if((move.joints & (JointMask{1} << joint)) == 0) continue;
                if(!calculateJointApproach(move, joint)) return false;
            }
            return true;
        }

        bool beginJointStop(const TriggeredJointMove &move, const JointId joint) {
            auto &runtime = m_jointRuntime[joint];
            runtime.stopping = true;
            runtime.triggered = true;
            runtime.elapsed = 0.0;
            runtime.triggerPosition = m_snapshot.commandedJoints.position[joint];
            runtime.triggerVelocity = m_snapshot.commandedJoints.velocity[joint];
            runtime.triggerAcceleration = m_snapshot.commandedJoints.acceleration[joint];
            m_triggeredJoints |= JointMask{1} << joint;
            if(std::abs(runtime.triggerVelocity) <= 1e-12
               && std::abs(runtime.triggerAcceleration) <= 1e-12) {
                runtime.finished = true;
                return true;
            }
            ruckig::InputParameter<1> input;
            input.control_interface = ruckig::ControlInterface::Velocity;
            input.current_position = {runtime.triggerPosition};
            input.current_velocity = {runtime.triggerVelocity};
            input.current_acceleration = {runtime.triggerAcceleration};
            input.target_position = {runtime.triggerPosition};
            input.target_velocity = {0.0};
            input.target_acceleration = {0.0};
            input.max_velocity = {std::max(move.limits.velocity[joint], std::abs(runtime.triggerVelocity))};
            input.max_acceleration = {move.limits.acceleration[joint]};
            input.max_jerk = {move.limits.jerk[joint]};
            ruckig::Ruckig<1> generator;
            return generator.calculate(input, runtime.trajectory) == ruckig::Result::Working;
        }

        void removeSyntheticJointInputs(const TriggeredMoveId move) {
            for(std::uint32_t index = 0; index < m_pendingSyntheticInputs.size;) {
                if(m_pendingSyntheticInputs[index].move != move) { ++index; continue; }
                m_pendingSyntheticInputs[index] = m_pendingSyntheticInputs[m_pendingSyntheticInputs.size - 1];
                --m_pendingSyntheticInputs.size;
            }
        }

        void completeTriggeredJoints() {
            const auto move = std::get<TriggeredJointMove>(activeItem());
            JointMask expectedTriggers = 0;
            JointMotionState triggerState = m_snapshot.commandedJoints;
            for(const auto &trigger : move.triggers) expectedTriggers |= JointMask{1} << trigger.joint;
            for(JointId joint = 0; joint < MAX_JOINTS; ++joint) {
                if((m_triggeredJoints & (JointMask{1} << joint)) == 0) continue;
                const auto &runtime = m_jointRuntime[joint];
                triggerState.position[joint] = runtime.triggerPosition;
                triggerState.velocity[joint] = runtime.triggerVelocity;
                triggerState.acceleration[joint] = runtime.triggerAcceleration;
            }
            const auto status = expectedTriggers != 0 && (m_triggeredJoints & expectedTriggers) == expectedTriggers
                ? TriggeredMoveStatus::Triggered : TriggeredMoveStatus::ReachedTarget;
            emit(TriggeredJointMoveCompleted { move.epoch, move.moveId, status, m_triggeredJoints,
                                                triggerState, m_snapshot.commandedJoints });
            emit(BranchSelected { move.epoch, move.branch, BranchChoice::Stop, 0 });
            emit(ChunkRetired { move.epoch, move.id });
            m_snapshot.state = BackendState::Held;
            m_snapshot.lastBranch = move.branch;
            m_snapshot.activeJoints = 0;
            emit(BackendHeld { move.epoch, m_snapshot.commanded, move.cursor });
            removeSyntheticJointInputs(move.moveId);
            release(*m_active);
            m_active.reset();
        }

        void advanceTriggeredJoints(double &seconds) {
            const auto &move = std::get<TriggeredJointMove>(activeItem());
            const auto elapsed = seconds;
            bool allFinished = true;
            for(JointId joint = 0; joint < MAX_JOINTS; ++joint) {
                if((move.joints & (JointMask{1} << joint)) == 0) continue;
                auto &runtime = m_jointRuntime[joint];
                if(runtime.finished) continue;
                allFinished = false;
                const auto duration = runtime.trajectory.get_duration();
                runtime.elapsed = std::min(runtime.elapsed + elapsed, duration);
                double position = 0.0, velocity = 0.0, acceleration = 0.0;
                runtime.trajectory.at_time(runtime.elapsed, position, velocity, acceleration);
                m_snapshot.commandedJoints.position[joint] = position;
                m_snapshot.commandedJoints.velocity[joint] = velocity;
                m_snapshot.commandedJoints.acceleration[joint] = acceleration;
                if(!runtime.stopping) {
                    const auto transition = syntheticJointTransition(move.moveId, joint);
                    const auto crossed = transition && runtime.direction * (position - *transition) >= -1e-12;
                    if(crossed) {
                        runtime.debounceElapsed += elapsed;
                        if(runtime.debounceElapsed + 1e-12 >= jointTriggerDebounce(move, joint)
                           && !beginJointStop(move, joint)) { faultTriggered(); return; }
                    } else {
                        runtime.debounceElapsed = 0.0;
                        if(runtime.elapsed + 1e-12 >= duration) {
                            m_snapshot.commandedJoints.position[joint] = runtime.target;
                            m_snapshot.commandedJoints.velocity[joint] = 0.0;
                            m_snapshot.commandedJoints.acceleration[joint] = 0.0;
                            runtime.finished = true;
                        }
                    }
                } else if(runtime.elapsed + 1e-12 >= duration) {
                    m_snapshot.commandedJoints.velocity[joint] = 0.0;
                    m_snapshot.commandedJoints.acceleration[joint] = 0.0;
                    runtime.finished = true;
                }
            }
            m_snapshot.feedbackJoints = m_snapshot.commandedJoints;
            seconds = 0.0;
            if(!allFinished) {
                allFinished = true;
                for(JointId joint = 0; joint < MAX_JOINTS; ++joint)
                    if((move.joints & (JointMask{1} << joint)) != 0 && !m_jointRuntime[joint].finished)
                        allFinished = false;
            }
            if(allFinished) completeTriggeredJoints();
        }

        void emit(const ExecutionEvent &event) {
            if(!m_events.tryPush(event)) {
                m_snapshot.state = BackendState::Faulted;
                m_snapshot.faultCode = 1; // Backend-to-NRT event overflow.
            }
        }

        void serviceControls() {
            ControlRequest request;
            while(m_controls.tryPop(request)) {
                std::visit([&](const auto &value) {
                    using T = std::decay_t<decltype(value)>;
                    bool success = true;
                    if constexpr(std::same_as<T, EnableRequest>) {
                        success = !m_jog;
                        if(success) m_snapshot.state = BackendState::Held;
                    }
                    else if constexpr(std::same_as<T, DisableRequest>) {
                        if(m_jog) {
                            m_jog->velocity = 0.0;
                            m_jog->acceleration = 0.0;
                            m_jog->stopReason = JogStopReason::Disabled;
                            applyJogState();
                            completeJog();
                        }
                        if(m_active) release(*m_active);
                        m_active.reset(); m_snapshot.state = BackendState::Disabled;
                    } else if constexpr(std::same_as<T, StartRequest> || std::same_as<T, ResumeRequest>) {
                        success = !m_jog;
                        if(success) { m_snapshot.epoch = value.epoch; m_snapshot.state = BackendState::Running; }
                    } else if constexpr(std::same_as<T, FeedHoldRequest>) {
                        if(m_active) { m_stopping = true; m_span = 0; m_spanElapsed = 0.0; }
                        else if(m_jog) success = beginJogStop(JogStopReason::RequestedStop);
                        else m_snapshot.state = BackendState::Held;
                    } else if constexpr(std::same_as<T, AbortRequest>) {
                        if(m_jog) success = beginJogStop(JogStopReason::Aborted);
                        else {
                            if(m_active) release(*m_active);
                            m_active.reset(); m_snapshot.state = BackendState::Held;
                        }
                    } else if constexpr(std::same_as<T, ResetRequest>) {
                        if(m_jog) {
                            m_jog->velocity = 0.0;
                            m_jog->acceleration = 0.0;
                            m_jog->stopReason = JogStopReason::Aborted;
                            applyJogState();
                            completeJog();
                        }
                        const auto commandedJoints = m_snapshot.commandedJoints;
                        const auto feedbackJoints = m_snapshot.feedbackJoints;
                        if(m_active) release(*m_active);
                        m_active.reset();
                        std::uint8_t discarded;
                        while(m_plans.tryPop(discarded)) release(discarded);
                        m_stopping = false; m_span = 0; m_nextEvent = 0; m_spanElapsed = 0.0;
                        m_snapshot = {}; m_snapshot.epoch = value.nextEpoch;
                        m_snapshot.commandedJoints = commandedJoints;
                        m_snapshot.feedbackJoints = feedbackJoints;
                    } else if constexpr(std::same_as<T, SetJointPositionRequest>) {
                        success = m_snapshot.state == BackendState::Held && !m_active;
                        if(success) for(JointId joint = 0; joint < MAX_JOINTS; ++joint) {
                            if((value.joints & (JointMask{1} << joint)) == 0) continue;
                            m_snapshot.commandedJoints.position[joint] = value.position[joint];
                            m_snapshot.commandedJoints.velocity[joint] = 0.0;
                            m_snapshot.commandedJoints.acceleration[joint] = 0.0;
                            m_snapshot.feedbackJoints = m_snapshot.commandedJoints;
                        }
                    } else if constexpr(std::same_as<T, StartContinuousJogRequest>
                                        || std::same_as<T, StartIncrementalJogRequest>) {
                        success = initializeJog(value);
                    } else if constexpr(std::same_as<T, RenewJogLeaseRequest>) {
                        success = m_jog && m_jog->continuous && !m_jog->stopping
                            && m_jog->id == value.jog;
                        if(success) m_jog->leaseTicks = m_jog->leasePeriod;
                    } else if constexpr(std::same_as<T, StopJogRequest>) {
                        success = m_jog && m_jog->id == value.jog
                            && beginJogStop(JogStopReason::RequestedStop);
                    }
                    emit(RequestCompleted { value.id, success });
                }, request);
            }
        }

        void activateNext() {
            std::uint8_t index;
            if(!m_plans.tryPop(index)) return;
            const auto &item = m_planSlots[index].item;
            if(itemEpoch(item) != m_snapshot.epoch) {
                emit(ChunkRejected { itemEpoch(item), itemId(item) });
                release(index);
                return;
            }
            m_active = index;
            m_stopping = false;
            m_span = 0;
            m_nextEvent = 0;
            m_spanElapsed = 0.0;
            m_snapshot.activeChunk = itemId(item);
            m_snapshot.activeSpan = 0;
            emit(ChunkAccepted { itemEpoch(item), itemId(item) });
            if(std::holds_alternative<TriggeredMove>(item) && !initializeTriggered()) faultTriggered();
            if(std::holds_alternative<TriggeredJointMove>(item) && !initializeTriggeredJoints()) faultTriggered();
        }

        void completeSpan() {
            m_spanElapsed = 0.0;
            ++m_span;
            const auto count = m_stopping ? activeChunk().stopTail.size : activeChunk().normalMotion.size;
            if(m_span < count) return;
            if(m_stopping) {
                auto &chunk = activeChunk();
                emit(ChunkRetired { chunk.epoch, chunk.id });
                m_snapshot.state = BackendState::Held;
                m_snapshot.lastBranch = chunk.branch;
                emit(BackendHeld { chunk.epoch, chunk.stopState, chunk.stopCursor });
                release(*m_active);
                m_active.reset();
                return;
            }

            std::uint8_t continuationIndex = 0;
            const auto hasContinuation = m_plans.tryPop(continuationIndex);
            const auto &current = activeChunk();
            if(hasContinuation
               && itemEpoch(m_planSlots[continuationIndex].item) == current.epoch
               && itemPredecessor(m_planSlots[continuationIndex].item) == current.branch) {
                const auto oldIndex = *m_active;
                const auto &continuation = m_planSlots[continuationIndex].item;
                emit(BranchSelected { current.epoch, current.branch, BranchChoice::Continue, itemId(continuation) });
                ++m_continuationSequence;
                emit(ChunkRetired { current.epoch, current.id });
                m_active = continuationIndex;
                release(oldIndex);
                m_span = 0;
                m_nextEvent = 0;
                m_spanElapsed = 0.0;
                m_snapshot.activeChunk = itemId(continuation);
                m_snapshot.activeSpan = 0;
                emit(ChunkAccepted { itemEpoch(continuation), itemId(continuation) });
                if(std::holds_alternative<TriggeredMove>(continuation) && !initializeTriggered()) faultTriggered();
                if(std::holds_alternative<TriggeredJointMove>(continuation)
                   && !initializeTriggeredJoints()) faultTriggered();
            } else {
                if(hasContinuation) {
                    const auto &continuation = m_planSlots[continuationIndex].item;
                    emit(ChunkRejected { itemEpoch(continuation), itemId(continuation) });
                    release(continuationIndex);
                }
                emit(BranchSelected { current.epoch, current.branch, BranchChoice::Stop, 0 });
                m_stopping = true;
                m_span = 0;
            }
        }

        void recordCalculatedPosition() {
            const auto &chunk = activeChunk();
            const auto stopTail = m_stopping;
            const auto spanId = currentSpan().id;
            if(m_trajectoryDiagnostics.spans.empty()
               || m_trajectoryDiagnostics.spans.back().epoch != chunk.epoch
               || m_trajectoryDiagnostics.spans.back().chunk != chunk.id
               || m_trajectoryDiagnostics.spans.back().span != spanId
               || m_trajectoryDiagnostics.spans.back().stopTail != stopTail) {
                ExecutedTrajectorySpan executed {
                    .epoch = chunk.epoch,
                    .chunk = chunk.id,
                    .span = spanId,
                    .stopTail = stopTail,
                    .positions = {},
                };
                executed.positions.push_back(evaluate(currentSpan(), 0.0));
                m_trajectoryDiagnostics.spans.push_back(std::move(executed));
            }
            m_trajectoryDiagnostics.spans.back().positions.push_back(m_snapshot.commanded.position);
            ++m_trajectoryDiagnostics.revision;
        }

    };

    MockMotionBackend::MockMotionBackend() : m_impl(std::make_unique<Impl>()) { }
    MockMotionBackend::~MockMotionBackend() = default;
    PublishResult MockMotionBackend::tryPublish(const ExecutionItem &item) noexcept { return m_impl->publish(item); }
    SubmitResult MockMotionBackend::trySubmit(const ControlRequest &request) noexcept { return m_impl->submit(request); }
    bool MockMotionBackend::tryTakeEvent(ExecutionEvent &event) noexcept { return m_impl->takeEvent(event); }
    bool MockMotionBackend::tryTakeSnapshot(ExecutionSnapshot &snapshot) noexcept { return m_impl->takeSnapshot(snapshot); }
    void MockMotionBackend::advance(const double seconds) { m_impl->advance(seconds); }
    bool MockMotionBackend::advanceTick(const double seconds, const bool publishSnapshot) {
        return m_impl->advanceTick(seconds,publishSnapshot);
    }
    void MockMotionBackend::runUntilIdle() { m_impl->runUntilIdle(); }
    void MockMotionBackend::runUntilIdle(const double tickSeconds) { m_impl->runUntilIdle(tickSeconds); }
    bool MockMotionBackend::configureSyntheticInput(const TriggeredMoveId move,
                                                    const position_t &transitionPosition) noexcept {
        return m_impl->configureSyntheticInput({ .move = move, .joint = MAX_JOINTS,
                                                  .position = transitionPosition });
    }
    bool MockMotionBackend::configureSyntheticJointInput(const TriggeredMoveId move, const JointId joint,
                                                         const double transitionPosition) noexcept {
        return m_impl->configureSyntheticInput({ .move = move, .joint = joint,
                                                  .jointPosition = transitionPosition });
    }
    void MockMotionBackend::clearTrajectoryDiagnostics() { m_impl->clearDiagnostics(); }
    MockTrajectorySnapshot MockMotionBackend::trajectorySnapshot() const { return m_impl->diagnostics(); }
}
