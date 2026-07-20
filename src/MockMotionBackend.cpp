#include "machine/MockMotionBackend.h"

#include "machine/MachineConfiguration.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
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

        position_t thirdDerivative(const AxisPolynomialSpan &span) {
            return scaled(span.a, 6.0 * span.inverseDurationCubed);
        }
    }

    class MockMotionBackend::Impl {
        struct PlanSlot {
            std::atomic<bool> occupied{false};
            ExecutionItem item{};
            std::uint64_t normalMotionNanoseconds = 0;
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
        std::atomic<std::uint64_t> m_queuedNormalMotionNanoseconds{0};
        std::atomic<std::uint32_t> m_queuedExecutionItems{0};
        bool m_stopping = false;
        std::uint32_t m_span = 0;
        std::uint32_t m_nextEvent = 0;
        double m_spanElapsed = 0.0;
        double m_lastAdvanceProgramSeconds = 0.0;
        std::atomic<double> m_currentProgramJerkMagnitude{0.0};
        std::mutex m_executedJerkSamplesMutex;
        std::vector<ExecutedJerkSample> m_executedJerkSamples;
        std::optional<ExecutedJerkSample> m_latestExecutedJerkSample;
        std::uint64_t m_continuationSequence = 0;
        FeedHoldConfiguration m_feedHoldLimits;
        TrajectoryLimits m_trajectoryLimits;
        bool m_feedHolding = false;
        bool m_feedHeld = false;
        bool m_feedResuming = false;
        double m_executionRate = 1.0;
        double m_executionRateAcceleration = 0.0;
        double m_executionRateJerk = 0.0;
        double m_physicalElapsed = 0.0;
        double m_referenceElapsed = 0.0;
        MockTrajectorySnapshot m_trajectoryDiagnostics;
        struct TriggeredRuntime {
            position_t start{};
            position_t direction{};
            double length = 0.0;
            double elapsed = 0.0;
            bool stopping = false;
            bool feedHoldStopping = false;
            TriggeredMoveStatus completionStatus = TriggeredMoveStatus::ReachedTarget;
            MotionState stopOrigin{};
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
        std::vector<AxisConfiguration> m_axes;
        std::vector<JointConfiguration> m_joints;

    public:
        Impl() = default;
        Impl(const std::vector<AxisConfiguration> &axes,
             const std::vector<JointConfiguration> &joints)
            : m_axes(axes), m_joints(joints) { }

    private:

        JointMask axisJoints(const AxisId axis) const {
            const auto machineAxis = static_cast<Machine::Axis>(static_cast<std::uint8_t>(axis));
            const auto configured = std::ranges::find(m_axes, machineAxis, &AxisConfiguration::axis);
            if(configured == m_axes.end()) return 0;
            JointMask result = 0;
            for(const auto joint : configured->joints) result |= JointMask { 1 } << joint;
            return result;
        }

        void updateConfiguredAxisFromJoints(const Machine::Axis axis) {
            double position = 0.0;
            double velocity = 0.0;
            double acceleration = 0.0;
            std::size_t count = 0;
            for(const auto &joint : m_joints) {
                if(joint.axis != axis || std::abs(joint.coordinateScale) <= 1e-12) continue;
                position += m_snapshot.commandedJoints.position[joint.id] / joint.coordinateScale;
                velocity += m_snapshot.commandedJoints.velocity[joint.id] / joint.coordinateScale;
                acceleration += m_snapshot.commandedJoints.acceleration[joint.id] / joint.coordinateScale;
                ++count;
            }
            if(count == 0) return;
            const auto id = static_cast<AxisId>(static_cast<std::uint8_t>(axis));
            axisComponent(m_snapshot.commanded.position, id) = position / static_cast<double>(count);
            axisComponent(m_snapshot.commanded.velocity, id) = velocity / static_cast<double>(count);
            axisComponent(m_snapshot.commanded.acceleration, id) = acceleration / static_cast<double>(count);
            m_snapshot.feedback = m_snapshot.commanded;
        }

        void updateConfiguredJointsFromAxis(const AxisId axis) {
            const auto machineAxis = static_cast<Machine::Axis>(static_cast<std::uint8_t>(axis));
            for(const auto &joint : m_joints) {
                if(joint.axis != machineAxis) continue;
                m_snapshot.commandedJoints.position[joint.id]
                    = axisComponent(m_snapshot.commanded.position, axis) * joint.coordinateScale;
                m_snapshot.commandedJoints.velocity[joint.id]
                    = axisComponent(m_snapshot.commanded.velocity, axis) * joint.coordinateScale;
                m_snapshot.commandedJoints.acceleration[joint.id]
                    = axisComponent(m_snapshot.commanded.acceleration, axis) * joint.coordinateScale;
            }
            m_snapshot.feedbackJoints = m_snapshot.commandedJoints;
        }

        static std::uint64_t secondsToNanoseconds(const double seconds) {
            constexpr auto scale=1.0e9;
            if(!std::isfinite(seconds)||seconds<=0.0) return 0;
            const auto maximum=static_cast<double>(std::numeric_limits<std::uint64_t>::max());
            if(seconds>=maximum/scale) return std::numeric_limits<std::uint64_t>::max();
            return static_cast<std::uint64_t>(std::llround(seconds*scale));
        }

        static std::uint64_t normalMotionNanoseconds(const ExecutionItem &item) {
            const auto *chunk=std::get_if<PlanChunk>(&item);
            if(!chunk) return 0;
            auto seconds=0.0;
            for(const auto &span:chunk->normalMotion)
                seconds+=std::max(span.duration,0.0);
            return secondsToNanoseconds(seconds);
        }

    public:
        Impl(const FeedHoldConfiguration &feedHold, const TrajectoryLimits &trajectory)
            : m_feedHoldLimits(feedHold), m_trajectoryLimits(trajectory) { }
        Impl(const FeedHoldConfiguration &feedHold, const TrajectoryLimits &trajectory,
             const std::vector<AxisConfiguration> &axes,
             const std::vector<JointConfiguration> &joints)
            : m_feedHoldLimits(feedHold), m_trajectoryLimits(trajectory),
              m_axes(axes), m_joints(joints) { }

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
                    return !value.triggerRequired || boundJoints == value.joints;
                }
            }, item);
            if(!valid) return PublishResult::Invalid;
            for(std::uint8_t index = 0; index < m_planSlots.size(); ++index) {
                bool expected = false;
                if(!m_planSlots[index].occupied.compare_exchange_strong(expected, true, std::memory_order_acquire)) continue;
                auto &slot=m_planSlots[index];
                slot.item = item;
                slot.normalMotionNanoseconds=normalMotionNanoseconds(item);
                m_queuedNormalMotionNanoseconds.fetch_add(
                    slot.normalMotionNanoseconds,std::memory_order_release);
                m_queuedExecutionItems.fetch_add(1,std::memory_order_release);
                if(m_plans.tryPush(index)) return PublishResult::Published;
                m_queuedNormalMotionNanoseconds.fetch_sub(
                    slot.normalMotionNanoseconds,std::memory_order_acq_rel);
                m_queuedExecutionItems.fetch_sub(1,std::memory_order_acq_rel);
                slot.occupied.store(false, std::memory_order_release);
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
        void clearDiagnostics() {
            m_trajectoryDiagnostics = {};
            std::scoped_lock lock(m_executedJerkSamplesMutex);
            m_executedJerkSamples.clear();
        }
        MockTrajectorySnapshot diagnostics() const { return m_trajectoryDiagnostics; }
        std::vector<ExecutedJerkSample> takeExecutedJerkSamples() {
            std::scoped_lock lock(m_executedJerkSamplesMutex);
            std::vector<ExecutedJerkSample> result;
            result.swap(m_executedJerkSamples);
            return result;
        }
        void advance(double seconds, const bool shouldPublishSnapshot = true) {
            m_lastAdvanceProgramSeconds = 0.0;
            m_currentProgramJerkMagnitude.store(0.0,std::memory_order_relaxed);
            m_latestExecutedJerkSample.reset();
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
            const auto physicalSeconds = seconds;
            m_physicalElapsed += physicalSeconds;
            if(m_jog) {
                advanceJog(seconds);
                if(shouldPublishSnapshot || !m_jog) publishSnapshot();
                return;
            }
            if(m_snapshot.state != BackendState::Running
               && m_snapshot.state != BackendState::Holding) {
                if(shouldPublishSnapshot) publishSnapshot();
                return;
            }
            if(!m_active) activateNext();
            if((m_feedHolding || m_feedResuming) && !m_active) {
                if(m_feedHolding) finishFeedHold();
                else faultFeedRetimingAtStopBranch();
                if(shouldPublishSnapshot) publishSnapshot();
                return;
            }
            const auto executedActiveMotion = m_active.has_value();
            if((m_feedHolding || m_feedResuming) && m_active
               && std::holds_alternative<PlanChunk>(activeItem()))
                seconds = feedRetimingReferenceAdvance(physicalSeconds);
            const auto availableProgramSeconds = seconds;
            while(m_active && seconds > 0.0) {
                if(std::holds_alternative<TriggeredMove>(activeItem())) {
                    advanceTriggered(seconds);
                    if(m_snapshot.state != BackendState::Running
                       && m_snapshot.state != BackendState::Holding)
                        seconds = 0.0;
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
                m_snapshot.commanded = retimedState(span, u);
                m_snapshot.feedback = m_snapshot.commanded;
                const auto jerk = retimedJerk(span, u);
                m_currentProgramJerkMagnitude.store(magnitude(jerk), std::memory_order_relaxed);
                m_lastAdvanceProgramSeconds = availableProgramSeconds - seconds;
                recordCalculatedPosition();
                if(m_spanElapsed + 1e-12 < span.duration) break;
                completeSpan();
                if(m_snapshot.state == BackendState::Faulted) break;
            }
            if(!m_active)
                m_currentProgramJerkMagnitude.store(0.0,std::memory_order_relaxed);
            m_lastAdvanceProgramSeconds = availableProgramSeconds - seconds;
            m_referenceElapsed += m_lastAdvanceProgramSeconds;
            m_snapshot.executionRate = m_executionRate;
            m_snapshot.executionRateAcceleration = m_executionRateAcceleration;
            if(m_feedHolding && m_snapshot.state != BackendState::Faulted
               && m_executionRate == 0.0)
                finishFeedHold();
            // Always publish terminal/held state even inside a decimated batch so
            // NRT cannot observe completion before its matching final snapshot.
            if(shouldPublishSnapshot || (executedActiveMotion && !m_active)
               || (m_snapshot.state != BackendState::Running
                   && m_snapshot.state != BackendState::Holding))
                publishSnapshot();
        }

        bool advanceTick(const double seconds,const bool shouldPublishSnapshot) {
            const auto before=m_continuationSequence;
            advance(seconds,shouldPublishSnapshot);
            if(m_latestExecutedJerkSample) {
                std::scoped_lock lock(m_executedJerkSamplesMutex);
                m_executedJerkSamples.push_back(*m_latestExecutedJerkSample);
            }
            return m_continuationSequence!=before;
        }

        double lastAdvanceProgramSeconds() const noexcept {
            return m_lastAdvanceProgramSeconds;
        }

        double currentProgramJerkMagnitude() const noexcept {
            return m_currentProgramJerkMagnitude.load(std::memory_order_relaxed);
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
        const AxisPolynomialSpan &currentSpan() const {
            const auto &chunk = activeChunk();
            return m_stopping ? chunk.stopTail[m_span] : chunk.normalMotion[m_span];
        }

        bool feedRetimingAccelerationInterval(const position_t &referenceVelocity,
                                              const position_t &referenceAcceleration,
                                              double &lower, double &upper) const {
            lower = -std::numeric_limits<double>::infinity();
            upper = std::numeric_limits<double>::infinity();
            const auto rateSquared = m_executionRate * m_executionRate;
            const auto constrain = [&](const double velocity, const double acceleration,
                                       const double limit) {
                if(!std::isfinite(limit)) return true;
                const auto base = acceleration * rateSquared;
                if(std::abs(velocity) <= 1e-12) return std::abs(base) <= limit * (1.0 + 1e-9) + 1e-9;
                auto first = (-limit - base) / velocity;
                auto second = (limit - base) / velocity;
                if(first > second) std::swap(first, second);
                lower = std::max(lower, first);
                upper = std::min(upper, second);
                return lower <= upper + 1e-12;
            };
            if(!constrain(referenceVelocity.x, referenceAcceleration.x,
                          m_trajectoryLimits.axisAcceleration.x)
               || !constrain(referenceVelocity.y, referenceAcceleration.y,
                             m_trajectoryLimits.axisAcceleration.y)
               || !constrain(referenceVelocity.z, referenceAcceleration.z,
                             m_trajectoryLimits.axisAcceleration.z)
               || !constrain(referenceVelocity.a, referenceAcceleration.a,
                             m_trajectoryLimits.axisAcceleration.a)
               || !constrain(referenceVelocity.b, referenceAcceleration.b,
                             m_trajectoryLimits.axisAcceleration.b)
               || !constrain(referenceVelocity.c, referenceAcceleration.c,
                             m_trajectoryLimits.axisAcceleration.c)) return false;

            const auto base = scaled(referenceAcceleration, rateSquared);
            const auto quadratic = dot(referenceVelocity, referenceVelocity);
            const auto linear = 2.0 * dot(base, referenceVelocity);
            const auto constant = dot(base, base)
                - m_trajectoryLimits.pathAcceleration * m_trajectoryLimits.pathAcceleration;
            if(quadratic <= 1e-24) return constant <= 1e-9;
            const auto discriminant = linear * linear - 4.0 * quadratic * constant;
            if(discriminant < -1e-9) return false;
            const auto root = std::sqrt(std::max(discriminant, 0.0));
            lower = std::max(lower, (-linear - root) / (2.0 * quadratic));
            upper = std::min(upper, (-linear + root) / (2.0 * quadratic));
            return lower <= upper + 1e-12;
        }

        double feedRetimingReferenceAdvance(const double physicalSeconds) {
            const auto &span = currentSpan();
            const auto u = span.duration > 0.0
                ? std::clamp(m_spanElapsed * span.inverseDuration, 0.0, 1.0) : 1.0;
            const auto referenceVelocity = derivative(span, u);
            const auto referenceAcceleration = secondDerivative(span, u);
            const auto referenceSpeed = magnitude(referenceVelocity);
            if(referenceSpeed <= 1e-10
               && magnitude(scaled(referenceAcceleration,
                                   m_executionRate * m_executionRate)) <= 1e-9) {
                m_executionRate = m_feedResuming ? 1.0 : 0.0;
                m_executionRateAcceleration = 0.0;
                m_executionRateJerk = 0.0;
                m_feedResuming = false;
                return m_executionRate * physicalSeconds;
            }

            double feasibleLower = 0.0;
            double feasibleUpper = 0.0;
            if(!feedRetimingAccelerationInterval(referenceVelocity, referenceAcceleration,
                                                 feasibleLower, feasibleUpper)) {
                m_snapshot.state = BackendState::Faulted;
                m_snapshot.faultCode = 4;
                emit(BackendFault { 4 });
                return 0.0;
            }

            const auto safeReferenceSpeed = std::max(referenceSpeed, 1e-9);
            const auto rateJerkLimit = m_feedHoldLimits.tangentialJerk / safeReferenceSpeed;
            const auto accelerationMagnitude = m_feedHoldLimits.tangentialAcceleration
                / safeReferenceSpeed;
            const auto release = m_feedResuming
                ? m_executionRateAcceleration > 0.0
                    && 1.0 - m_executionRate
                        <= m_executionRateAcceleration * m_executionRateAcceleration
                            / (2.0 * std::max(rateJerkLimit, 1e-12)) + 1e-12
                : m_executionRateAcceleration < 0.0
                    && m_executionRate
                        <= m_executionRateAcceleration * m_executionRateAcceleration
                            / (2.0 * std::max(rateJerkLimit, 1e-12)) + 1e-12;
            const auto targetAcceleration = std::clamp(
                m_feedResuming ? accelerationMagnitude : -accelerationMagnitude,
                feasibleLower, feasibleUpper);
            const auto requestedJerk = m_feedResuming
                ? (release ? -rateJerkLimit : rateJerkLimit)
                : (release ? rateJerkLimit : -rateJerkLimit);
            auto nextAcceleration = m_executionRateAcceleration
                + requestedJerk * physicalSeconds;
            if(m_feedResuming) {
                if(release && nextAcceleration < 0.0) nextAcceleration = 0.0;
                if(!release && nextAcceleration > targetAcceleration)
                    nextAcceleration = targetAcceleration;
            } else {
                if(release && nextAcceleration > 0.0) nextAcceleration = 0.0;
                if(!release && nextAcceleration < targetAcceleration)
                    nextAcceleration = targetAcceleration;
            }
            nextAcceleration = std::clamp(nextAcceleration, feasibleLower, feasibleUpper);

            auto nextRate = m_executionRate
                + 0.5 * (m_executionRateAcceleration + nextAcceleration) * physicalSeconds;
            if(!m_feedResuming && nextRate <= 1e-10) {
                nextRate = 0.0;
                nextAcceleration = 0.0;
            } else if(m_feedResuming && nextRate >= 1.0 - 1e-10) {
                nextRate = 1.0;
                nextAcceleration = 0.0;
                m_feedResuming = false;
            }
            nextRate = std::clamp(nextRate, 0.0, 1.0);
            m_executionRateJerk = physicalSeconds > 0.0
                ? (nextAcceleration - m_executionRateAcceleration) / physicalSeconds : 0.0;
            const auto referenceAdvance = 0.5 * (m_executionRate + nextRate) * physicalSeconds;
            m_executionRate = nextRate;
            m_executionRateAcceleration = nextAcceleration;
            return referenceAdvance;
        }

        MotionState retimedState(const AxisPolynomialSpan &span, const double u) const {
            const auto referenceVelocity = derivative(span, u);
            const auto referenceAcceleration = secondDerivative(span, u);
            return {
                evaluate(span, u),
                scaled(referenceVelocity, m_executionRate),
                scaled(referenceAcceleration, m_executionRate * m_executionRate)
                    + scaled(referenceVelocity, m_executionRateAcceleration),
            };
        }

        position_t retimedJerk(const AxisPolynomialSpan &span, const double u) const {
            const auto referenceVelocity = derivative(span, u);
            const auto referenceAcceleration = secondDerivative(span, u);
            return scaled(thirdDerivative(span),
                          m_executionRate * m_executionRate * m_executionRate)
                + scaled(referenceAcceleration,
                         3.0 * m_executionRate * m_executionRateAcceleration)
                + scaled(referenceVelocity, m_executionRateJerk);
        }

        void finishFeedHold() {
            m_feedHolding = false;
            m_feedHeld = true;
            m_executionRate = 0.0;
            m_executionRateAcceleration = 0.0;
            m_executionRateJerk = 0.0;
            m_snapshot.state = BackendState::Held;
            m_snapshot.commanded.velocity = {};
            m_snapshot.commanded.acceleration = {};
            m_snapshot.feedback = m_snapshot.commanded;
            m_snapshot.executionRate = 0.0;
            m_snapshot.executionRateAcceleration = 0.0;
            emit(BackendHeld {
                m_snapshot.epoch, m_snapshot.commanded, BackendHoldReason::FeedHold,
            });
        }

        void faultFeedRetimingAtStopBranch() {
            m_feedHolding = false;
            m_feedHeld = false;
            m_feedResuming = false;
            m_snapshot.state = BackendState::Faulted;
            m_snapshot.faultCode = 5;
            emit(BackendFault { 5 });
            if(m_active) release(*m_active);
            m_active.reset();
        }
        void release(const std::uint8_t index) { m_planSlots[index].occupied.store(false, std::memory_order_release); }

        void accountForDequeued(const std::uint8_t index) {
            m_queuedNormalMotionNanoseconds.fetch_sub(
                m_planSlots[index].normalMotionNanoseconds,std::memory_order_acq_rel);
            m_queuedExecutionItems.fetch_sub(1,std::memory_order_acq_rel);
        }

        template<typename Spans>
        static double remainingDuration(const Spans &spans,const std::uint32_t current,
                                        const double elapsed) {
            if(current>=spans.size) return 0.0;
            auto result=std::max(spans[current].duration-elapsed,0.0);
            for(auto index=current+1;index<spans.size;++index)
                result+=std::max(spans[index].duration,0.0);
            return result;
        }

        void refreshBufferSnapshot() {
            constexpr auto nanosecondsToSeconds=1.0e-9;
            m_snapshot.activeNormalMotionRemainingSeconds=0.0;
            m_snapshot.stopBranchRemainingSeconds=0.0;
            m_snapshot.queuedNormalMotionSeconds=nanosecondsToSeconds
                *static_cast<double>(m_queuedNormalMotionNanoseconds.load(
                    std::memory_order_acquire));
            m_snapshot.queuedExecutionItems=m_queuedExecutionItems.load(
                std::memory_order_acquire);
            if(m_active) if(const auto *chunk=std::get_if<PlanChunk>(&activeItem())) {
                if(m_stopping)
                    m_snapshot.stopBranchRemainingSeconds=remainingDuration(
                        chunk->stopTail,m_span,m_spanElapsed);
                else {
                    m_snapshot.activeNormalMotionRemainingSeconds=remainingDuration(
                        chunk->normalMotion,m_span,m_spanElapsed);
                    m_snapshot.stopBranchRemainingSeconds=remainingDuration(
                        chunk->stopTail,0,0.0);
                }
            }
            m_snapshot.committedNormalMotionSeconds=
                m_snapshot.state==BackendState::Running&&!m_stopping
                ?m_snapshot.activeNormalMotionRemainingSeconds
                    +m_snapshot.queuedNormalMotionSeconds
                :0.0;
        }

        void publishSnapshot() {
            refreshBufferSnapshot();
            (void)m_snapshots.tryPush(m_snapshot);
        }

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
            if(target.type == JogTargetType::Axis)
                return target.joints == 0 && (m_axes.empty() || axisJoints(target.axis) != 0);
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
                updateConfiguredJointsFromAxis(jog.target.axis);
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
            input.current_position = {jog.position};
            input.current_velocity = {jog.velocity};
            input.current_acceleration = {jog.acceleration};
            input.target_position = {distance};
            input.target_velocity = {0.0};
            input.target_acceleration = {0.0};
            input.max_velocity = {std::max(std::min(velocity, jog.limits.velocity),
                                           std::abs(jog.velocity))};
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

        bool setContinuousJogVelocity(const double signedVelocity) {
            if(!m_jog || !m_jog->continuous || m_jog->stopping
               || !std::isfinite(signedVelocity) || std::abs(signedVelocity) <= 1e-12)
                return false;
            auto &jog = *m_jog;
            jog.cruiseVelocity = std::clamp(
                signedVelocity, -jog.limits.velocity, jog.limits.velocity);
            bool calculated = false;
            if(jog.travel.enabled) {
                const auto target = jog.cruiseVelocity < 0.0
                    ? jog.travel.minimum - jog.axisOrigin
                    : jog.travel.maximum - jog.axisOrigin;
                calculated = calculateJogPosition(target, std::abs(jog.cruiseVelocity));
            } else {
                calculated = calculateJogVelocity(jog.cruiseVelocity);
            }
            if(!calculated) return false;
            jog.elapsed = 0.0;
            jog.cruising = false;
            jog.leaseTicks = jog.leasePeriod;
            return true;
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
            m_snapshot.activeJoints = request.target.type == JogTargetType::Axis
                ? axisJoints(request.target.axis) : request.target.joints;
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
            return generator.calculate(input, m_triggered.trajectory) == ruckig::Result::Working;
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

        bool beginTriggeredStop(const TriggeredMoveStatus status,
                                const bool feedHold = false) {
            const auto &move = std::get<TriggeredMove>(activeItem());
            m_triggered.stopOrigin = m_snapshot.commanded;
            if(!feedHold) m_triggered.triggerState = m_snapshot.commanded;
            m_triggered.completionStatus = status;
            m_triggered.stopping = true;
            m_triggered.feedHoldStopping = feedHold;
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
            emit(BackendHeld { move.epoch, stopped, BackendHoldReason::StopBranch });
            removeSyntheticInput(move.moveId);
            release(*m_active);
            m_active.reset();
        }

        void finishTriggeredFeedHold() {
            m_triggered.stopping = false;
            m_triggered.feedHoldStopping = false;
            m_feedHolding = false;
            m_feedHeld = true;
            m_snapshot.state = BackendState::Held;
            m_snapshot.commanded.velocity = {};
            m_snapshot.commanded.acceleration = {};
            m_snapshot.feedback = m_snapshot.commanded;
            emit(BackendHeld {
                m_snapshot.epoch, m_snapshot.commanded, BackendHoldReason::FeedHold,
            });
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
                ? m_triggered.stopOrigin.position : m_triggered.start;
            m_snapshot.commanded = triggeredStateAt(m_triggered.elapsed, origin);
            m_snapshot.feedback = m_snapshot.commanded;
            m_snapshot.spanProgress = duration > 0.0
                ? std::clamp(m_triggered.elapsed / duration, 0.0, 1.0) : 1.0;
            recordTriggeredPosition(m_triggered.stopping && !m_triggered.feedHoldStopping);
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
            } else {
                if(m_triggered.feedHoldStopping) {
                    const auto transition = syntheticTransitionDistance(move.moveId);
                    const auto progress = dot(m_snapshot.commanded.position - m_triggered.start,
                                              m_triggered.direction);
                    if(transition && progress + 1e-12 >= *transition) {
                        m_triggered.triggerState = m_snapshot.commanded;
                        m_triggered.completionStatus = TriggeredMoveStatus::Triggered;
                        m_triggered.feedHoldStopping = false;
                    }
                }
                if(m_triggered.elapsed + 1e-12 >= duration) {
                    m_snapshot.commanded.velocity = {};
                    m_snapshot.commanded.acceleration = {};
                    m_snapshot.feedback = m_snapshot.commanded;
                    if(m_triggered.feedHoldStopping) finishTriggeredFeedHold();
                    else completeTriggered(m_triggered.completionStatus);
                }
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
            emit(BackendHeld {
                move.epoch, m_snapshot.commanded, BackendHoldReason::StopBranch,
            });
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
                        m_feedHolding = false;
                        m_feedHeld = false;
                        m_feedResuming = false;
                    } else if constexpr(std::same_as<T, StartRequest>) {
                        success = !m_jog;
                        if(success) {
                            m_snapshot.epoch = value.epoch;
                            m_snapshot.state = BackendState::Running;
                            m_feedHolding = false;
                            m_feedHeld = false;
                            m_feedResuming = false;
                            m_executionRate = 1.0;
                            m_executionRateAcceleration = 0.0;
                            m_executionRateJerk = 0.0;
                        }
                    } else if constexpr(std::same_as<T, ResumeRequest>) {
                        success = !m_jog && m_snapshot.state == BackendState::Held
                            && value.epoch == m_snapshot.epoch;
                        if(success && m_feedHeld) {
                            if(m_active && !m_stopping
                               && std::holds_alternative<PlanChunk>(activeItem())) {
                                m_snapshot.state = BackendState::Running;
                                m_feedHeld = false;
                                m_feedResuming = true;
                                m_executionRateAcceleration = 0.0;
                                m_executionRateJerk = 0.0;
                            } else if(m_active
                                      && std::holds_alternative<TriggeredMove>(activeItem())) {
                                success = initializeTriggered();
                                if(success) {
                                    m_snapshot.state = BackendState::Running;
                                    m_feedHeld = false;
                                    m_feedResuming = false;
                                    m_executionRate = 1.0;
                                    m_executionRateAcceleration = 0.0;
                                    m_executionRateJerk = 0.0;
                                }
                            } else success = false;
                        } else if(success) {
                            m_snapshot.state = BackendState::Running;
                            m_feedHolding = false;
                            m_feedResuming = false;
                            m_executionRate = 1.0;
                            m_executionRateAcceleration = 0.0;
                            m_executionRateJerk = 0.0;
                        }
                    } else if constexpr(std::same_as<T, FeedHoldRequest>) {
                        success = !m_jog
                            && m_snapshot.state == BackendState::Running
                            && !m_feedResuming
                            && m_active.has_value();
                        if(success) {
                            if(std::holds_alternative<PlanChunk>(activeItem()))
                                success = !m_stopping;
                            else if(std::holds_alternative<TriggeredMove>(activeItem()))
                                success = !m_triggered.stopping
                                    && beginTriggeredStop(
                                        TriggeredMoveStatus::ReachedTarget, true);
                            else success = false;
                            if(success) {
                                m_feedHolding = true;
                                m_feedHeld = false;
                                m_snapshot.state = BackendState::Holding;
                            }
                        }
                    } else if constexpr(std::same_as<T, AbortRequest>) {
                        if(m_jog) success = beginJogStop(JogStopReason::Aborted);
                        else {
                            if(m_active) release(*m_active);
                            m_active.reset(); m_snapshot.state = BackendState::Held;
                            m_feedHolding = false;
                            m_feedHeld = false;
                            m_feedResuming = false;
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
                        while(m_plans.tryPop(discarded)) {
                            accountForDequeued(discarded);
                            release(discarded);
                        }
                        m_stopping = false; m_span = 0; m_nextEvent = 0; m_spanElapsed = 0.0;
                        m_feedHolding = false;
                        m_feedHeld = false;
                        m_feedResuming = false;
                        m_executionRate = 1.0;
                        m_executionRateAcceleration = 0.0;
                        m_executionRateJerk = 0.0;
                        m_physicalElapsed = 0.0;
                        m_referenceElapsed = 0.0;
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
                        if(success) for(const auto &axis : m_axes)
                            updateConfiguredAxisFromJoints(axis.axis);
                    } else if constexpr(std::same_as<T, StartContinuousJogRequest>
                                        || std::same_as<T, StartIncrementalJogRequest>) {
                        success = initializeJog(value);
                    } else if constexpr(std::same_as<T, RenewJogLeaseRequest>) {
                        success = m_jog && m_jog->continuous && !m_jog->stopping
                            && m_jog->id == value.jog;
                        if(success) m_jog->leaseTicks = m_jog->leasePeriod;
                    } else if constexpr(std::same_as<T, SetContinuousJogVelocityRequest>) {
                        success = m_jog && m_jog->id == value.jog
                            && setContinuousJogVelocity(value.signedVelocity);
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
            accountForDequeued(index);
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
                const auto reason = m_feedHolding
                    ? BackendHoldReason::FeedHold : BackendHoldReason::StopBranch;
                if(m_feedHolding) {
                    m_feedHolding = false;
                    m_executionRate = 0.0;
                    m_executionRateAcceleration = 0.0;
                    m_executionRateJerk = 0.0;
                    m_snapshot.executionRate = 0.0;
                    m_snapshot.executionRateAcceleration = 0.0;
                }
                emit(BackendHeld {
                    chunk.epoch, chunk.stopState, reason,
                });
                release(*m_active);
                m_active.reset();
                return;
            }

            std::uint8_t continuationIndex = 0;
            const auto hasContinuation = m_plans.tryPop(continuationIndex);
            if(hasContinuation) accountForDequeued(continuationIndex);
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
                if(m_feedHolding || m_feedResuming) {
                    faultFeedRetimingAtStopBranch();
                    return;
                }
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
            m_latestExecutedJerkSample=ExecutedJerkSample{
                .epoch=chunk.epoch,
                .chunk=chunk.id,
                .span=spanId,
                .position=m_snapshot.commanded.position,
                .velocity=m_snapshot.commanded.velocity,
                .acceleration=m_snapshot.commanded.acceleration,
                .jerk=retimedJerk(currentSpan(), m_snapshot.spanProgress),
                .magnitude=m_currentProgramJerkMagnitude.load(std::memory_order_relaxed),
                .physicalTime=m_physicalElapsed,
                .referenceTime=m_referenceElapsed + m_lastAdvanceProgramSeconds,
                .executionRate=m_executionRate,
                .executionRateAcceleration=m_executionRateAcceleration,
                .executionRateJerk=m_executionRateJerk,
                .feedHolding=m_feedHolding,
                .stopTail=stopTail,
            };
        }

    };

    MockMotionBackend::MockMotionBackend(const FeedHoldConfiguration &feedHold,
                                         const TrajectoryLimits &trajectory)
        : m_impl(std::make_unique<Impl>(feedHold, trajectory)) { }
    MockMotionBackend::MockMotionBackend(const std::vector<AxisConfiguration> &axes,
                                         const std::vector<JointConfiguration> &joints)
        : m_impl(std::make_unique<Impl>(axes, joints)) { }
    MockMotionBackend::MockMotionBackend(const FeedHoldConfiguration &feedHold,
                                         const TrajectoryLimits &trajectory,
                                         const std::vector<AxisConfiguration> &axes,
                                         const std::vector<JointConfiguration> &joints)
        : m_impl(std::make_unique<Impl>(feedHold, trajectory, axes, joints)) { }
    MockMotionBackend::~MockMotionBackend() = default;
    PublishResult MockMotionBackend::tryPublish(const ExecutionItem &item) noexcept { return m_impl->publish(item); }
    SubmitResult MockMotionBackend::trySubmit(const ControlRequest &request) noexcept { return m_impl->submit(request); }
    bool MockMotionBackend::tryTakeEvent(ExecutionEvent &event) noexcept { return m_impl->takeEvent(event); }
    bool MockMotionBackend::tryTakeSnapshot(ExecutionSnapshot &snapshot) noexcept { return m_impl->takeSnapshot(snapshot); }
    void MockMotionBackend::advance(const double seconds) { m_impl->advance(seconds); }
    bool MockMotionBackend::advanceTick(const double seconds, const bool publishSnapshot) {
        return m_impl->advanceTick(seconds,publishSnapshot);
    }
    double MockMotionBackend::lastAdvanceProgramSeconds() const noexcept {
        return m_impl->lastAdvanceProgramSeconds();
    }
    double MockMotionBackend::currentProgramJerkMagnitude() const noexcept {
        return m_impl->currentProgramJerkMagnitude();
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
    std::vector<ExecutedJerkSample> MockMotionBackend::takeExecutedJerkSamples() {
        return m_impl->takeExecutedJerkSamples();
    }
}
