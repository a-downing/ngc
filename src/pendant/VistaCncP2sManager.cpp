#include "pendant/VistaCncP2sManager.h"

#include <utility>

namespace ngc::pendant::vista_cnc_p2s {
    namespace {
        InputChange changesBetween(const InputState &previous, const InputState &current) noexcept {
            auto changes = InputChange::None;

            if (current.wheelDeltaValid && current.wheelDelta != 0) {
                changes |= InputChange::Wheel;
            }

            if (current.leftSelector != previous.leftSelector) {
                changes |= InputChange::LeftSelector;
            }

            if (current.rightSelector != previous.rightSelector) {
                changes |= InputChange::RightSelector;
            }

            if (current.wheelButton != previous.wheelButton) {
                changes |= InputChange::WheelButton;
            }

            if (current.machineOff != previous.machineOff) {
                changes |= InputChange::MachineOff;
            }

            if (current.emergencyStop != previous.emergencyStop) {
                changes |= InputChange::EmergencyStop;
            }

            if (current.wheelRateCode != previous.wheelRateCode) {
                changes |= InputChange::WheelRate;
            }

            if (current.wheelMotionAccumulator != previous.wheelMotionAccumulator) {
                changes |= InputChange::WheelMotionAccumulator;
            }

            return changes;
        }

        HidError stoppedError() {
            return { HidErrorCode::Cancelled, 0, "pendant manager is stopped" };
        }
    }

    Manager::Manager(std::unique_ptr<Driver> driver, const std::chrono::milliseconds reportTimeout) : m_driver(std::move(driver)), m_reportTimeout(reportTimeout) {
        m_thread = std::thread(&Manager::work, this);
        m_displayThread = std::thread(&Manager::displayWork, this);
    }

    Manager::~Manager() { stop(); }

    std::expected<std::unique_ptr<Manager>, HidError> Manager::open() {
        auto driver = Driver::open();

        if (!driver) {
            return std::unexpected(std::move(driver.error()));
        }

        return std::make_unique<Manager>(std::move(*driver));
    }

    void Manager::stop() {
        const auto alreadyRequested = m_stopRequested.exchange(true);

        if (!alreadyRequested) {
            m_driver->cancel();
        }

        m_displayChanged.notify_all();

        if (m_thread.joinable()) {
            m_thread.join();
        }

        if (m_displayThread.joinable()) {
            m_displayThread.join();
        }
    }

    std::expected<void, HidError> Manager::setDisplay(const std::string_view text) {
        {
            std::scoped_lock displayLock(m_displayMutex);

            if (m_stopRequested.load() || m_inputEnded.load()) {
                return std::unexpected(stoppedError());
            }

            if (m_displayError) {
                return std::unexpected(*m_displayError);
            }

            const auto next = std::string(text.substr(0, 16));

            if (next == m_displayText) {
                return {};
            }

            m_displayText = next;
            ++m_displayRevision;
        }

        m_displayChanged.notify_one();

        return {};
    }

    bool Manager::tryTakeEvent(ManagerEvent &event) {
        std::scoped_lock lock(m_mutex);

        if (m_events.empty()) {
            return false;
        }

        event = std::move(m_events.front());
        m_events.pop_front();

        return true;
    }

    std::optional<ManagerEvent> Manager::waitTakeEvent(const std::chrono::milliseconds timeout) {
        std::unique_lock lock(m_mutex);

        if (!m_eventAvailable.wait_for(lock, timeout, [&] { return !m_events.empty(); })) {
            return std::nullopt;
        }

        auto event = std::move(m_events.front());
        m_events.pop_front();

        return event;
    }

    ManagerSnapshot Manager::snapshot() const {
        std::scoped_lock lock(m_mutex);
        return m_snapshot;
    }

    void Manager::publish(ManagerEvent event) {
        {
            std::scoped_lock lock(m_mutex);
            m_events.emplace_back(std::move(event));
        }

        m_eventAvailable.notify_one();
    }

    void Manager::work() {
        std::optional<InputState> previous;
        std::uint64_t reportSequence = 0;
        std::int64_t cumulativeWheelCounts = 0;

        while (true) {
            auto input = m_driver->read(m_reportTimeout);

            if (!input) {
                if (m_stopRequested.load() && input.error().code == HidErrorCode::Cancelled) {
                    {
                        std::scoped_lock lock(m_mutex);
                        m_snapshot.connected = false;
                        m_snapshot.stopped = true;
                    }

                    publish(Stopped {});
                } else {
                    {
                        std::scoped_lock lock(m_mutex);
                        m_snapshot.connected = false;
                        m_snapshot.error = input.error();
                    }

                    publish(Disconnected { std::move(input.error()) });
                }

                m_inputEnded.store(true);
                m_displayChanged.notify_all();

                return;
            }

            const auto arrivalTime = std::chrono::steady_clock::now();
            ++reportSequence;

            if (input->wheelDeltaValid) {
                cumulativeWheelCounts += input->wheelDelta;
            }

            if (!previous) {
                {
                    std::scoped_lock lock(m_mutex);
                    m_snapshot.connected = true;
                    m_snapshot.reportSequence = reportSequence;
                    m_snapshot.cumulativeWheelCounts = cumulativeWheelCounts;
                    m_snapshot.state = *input;
                    m_snapshot.error.reset();
                }

                publish(Connected { reportSequence, cumulativeWheelCounts, *input, arrivalTime });
            } else {
                const auto changes = changesBetween(*previous, *input);

                {
                    std::scoped_lock lock(m_mutex);
                    m_snapshot.reportSequence = reportSequence;
                    m_snapshot.cumulativeWheelCounts = cumulativeWheelCounts;
                    m_snapshot.state = *input;
                }

                if (changes != InputChange::None) {
                    publish(InputChanged {
                        reportSequence, cumulativeWheelCounts, changes, *input, arrivalTime,
                    });
                }
            }

            previous = *input;
        }
    }

    void Manager::displayWork() {
        std::unique_lock lock(m_displayMutex);

        m_displayChanged.wait(lock, [&] {
            return m_stopRequested.load() || m_inputEnded.load() || m_displayRevision != 0;
        });

        while (!m_stopRequested.load() && !m_inputEnded.load()) {
            const auto text = m_displayText;
            const auto revision = m_displayRevision;

            lock.unlock();
            auto written = m_driver->writeDisplay(text);

            if (!written) {
                {
                    std::scoped_lock displayLock(m_displayMutex);
                    m_displayError = written.error();
                }
                {
                    std::scoped_lock snapshotLock(m_mutex);
                    m_snapshot.error = written.error();
                }

                publish(DisplayFailed { std::move(written.error()) });
                return;
            }

            lock.lock();

            m_displayChanged.wait_for(lock, std::chrono::milliseconds(20), [&] {
                return m_stopRequested.load() || m_inputEnded.load()
                    || m_displayRevision != revision;
            });
        }
    }
}
