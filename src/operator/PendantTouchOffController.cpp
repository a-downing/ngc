#include "operator/PendantTouchOffController.h"

#include <cmath>
#include <type_traits>
#include <utility>

namespace ngc::operator_control {
    TouchOffController::TouchOffController(const MachineConfiguration &configuration)
        : m_increment(configuration.pendant.step.fineDistance) { }

    void TouchOffController::consume(const pendant::Intent &intent) {
        std::visit([&](const auto &value) {
            using T = std::decay_t<decltype(value)>;
            if constexpr(std::same_as<T, pendant::SelectionChanged>) {
                if(value.mode != pendant::Mode::Zero || value.selectorTransient || !value.axis) {
                    m_snapshot = {};
                    m_commit.reset();
                } else if(m_snapshot.axis != value.axis) {
                    m_snapshot = { value.axis, 0.0 };
                    m_commit.reset();
                }
            } else if constexpr(std::same_as<T, pendant::AdjustTouchOff>) {
                if(m_snapshot.axis != value.axis || value.counts == 0) return;
                const auto next = m_snapshot.workPosition
                    + static_cast<double>(value.counts) * m_increment;
                if(!std::isfinite(next)) {
                    m_error = "pendant touch-off value overflowed";
                    return;
                }
                m_snapshot.workPosition = next;
            } else if constexpr(std::same_as<T, pendant::CommitTouchOff>) {
                if(m_snapshot.axis != value.axis) return;
                m_commit = TouchOffCommit { value.axis, m_snapshot.workPosition };
                m_snapshot.workPosition = 0.0;
            } else if constexpr(std::same_as<T, pendant::CancelPendantActivity>) {
                // Releasing the button is necessary before the double-click and
                // therefore does not discard a staged touch-off value.
                if(value.reason != pendant::CancelReason::ButtonReleased
                   && value.reason != pendant::CancelReason::SelectionChanged) {
                    m_snapshot = {};
                    m_commit.reset();
                }
            }
        }, intent);
    }

    std::optional<TouchOffCommit> TouchOffController::takeCommit() {
        return std::exchange(m_commit, std::nullopt);
    }

    std::optional<std::string> TouchOffController::takeError() {
        return std::exchange(m_error, std::nullopt);
    }
}
