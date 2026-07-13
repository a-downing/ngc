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

        PublishResult tryPublish(const PlanChunk &chunk) noexcept override;
        SubmitResult trySubmit(const ControlRequest &request) noexcept override;
        bool tryTakeEvent(ExecutionEvent &event) noexcept override;
        bool tryTakeSnapshot(ExecutionSnapshot &snapshot) noexcept override;

        void advance(double seconds) override;
        void runUntilIdle() override;
        void setPlaybackRate(double rate) override;
        bool configureSyntheticProbe(std::uint64_t probeId, const position_t &physicalToolOffset,
                                     const position_t &activeToolOffset) noexcept;
        void clearTrajectoryDiagnostics() override;
        MockTrajectorySnapshot trajectorySnapshot() const override;

    private:
        class Impl;
        std::unique_ptr<Impl> m_impl;
    };
}
