#pragma once

#include <memory>

#include "machine/MotionBackend.h"
#include "machine/MockTrajectoryDiagnostics.h"

namespace ngc {
    class MockMotionBackend final : public MotionBackend, public SimulatedClockControl,
                                    public MockTrajectoryDiagnostics {
    public:
        MockMotionBackend();
        ~MockMotionBackend() override;
        MockMotionBackend(const MockMotionBackend &) = delete;
        MockMotionBackend &operator=(const MockMotionBackend &) = delete;

        PublishResult tryPublish(const ExecutionItem &item) noexcept override;
        SubmitResult trySubmit(const ControlRequest &request) noexcept override;
        bool tryTakeEvent(ExecutionEvent &event) noexcept override;
        bool tryTakeSnapshot(ExecutionSnapshot &snapshot) noexcept override;

        void advance(double seconds) override;
        // Fixed-tick simulation may decimate presentation snapshots while still
        // executing every servo update and event transition.
        // Returns true when this tick continued across at least one PlanChunk
        // boundary. Timed simulation uses this mock-only signal to give its NRT
        // producer a refill opportunity during accelerated scheduler batches.
        bool advanceTick(double seconds, bool publishSnapshot);
        void runUntilIdle() override;
        // Immediate preview still executes at fixed mock-servo intervals so its
        // diagnostic position buffer reflects the values calculated by the backend.
        void runUntilIdle(double tickSeconds);
        // Mock-only signal model. Production backends sample their configured HAL
        // input directly; this data never enters MotionBackend.
        bool configureSyntheticInput(TriggeredMoveId move, const position_t &transitionPosition) noexcept;
        bool configureSyntheticJointInput(TriggeredMoveId move, JointId joint,
                                          double transitionPosition) noexcept;
        void clearTrajectoryDiagnostics() override;
        MockTrajectorySnapshot trajectorySnapshot() const override;

    private:
        class Impl;
        std::unique_ptr<Impl> m_impl;
    };
}
