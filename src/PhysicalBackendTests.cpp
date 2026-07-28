#include <atomic>
#include <chrono>
#include <filesystem>
#include <format>
#include <memory>
#include <print>
#include <stdexcept>
#include <string_view>
#include <thread>

#include "machine/PhysicalProductionExecutorIo.h"
#include "physical/PhysicalBackendConfiguration.h"

namespace {
    using namespace std::chrono_literals;

    void require(const bool condition, const std::string_view message) {
        if (!condition) {
            throw std::runtime_error(std::string(message));
        }
    }

    struct SpindleObservation {
        std::atomic<std::uint32_t> commands{0};
        std::atomic<std::uint32_t> stops{0};
        std::atomic<bool> enabled{false};
        std::atomic<double> speed{0.0};
    };

    class ObservedSpindle final : public ngc::SpindleHardware {
    public:
        explicit ObservedSpindle(
            std::shared_ptr<SpindleObservation> observation,
            const bool failCommands = false)
            : m_observation(std::move(observation)),
              m_failCommands(failCommands) { }

        bool applyDesired(
            const ngc::SpindleEvent &desired) noexcept override {
            m_observation->enabled.store(
                desired.enabled, std::memory_order_release);
            m_observation->speed.store(
                desired.speed, std::memory_order_release);
            m_observation->commands.fetch_add(
                1, std::memory_order_release);

            return !m_failCommands;
        }

        bool pollStatus(
            ngc::SpindleHardwareStatus &status) noexcept override {
            status = {
                .communicationHealthy = true,
                .atSpeed = m_observation->enabled.load(
                    std::memory_order_acquire),
                .speed = m_observation->speed.load(
                    std::memory_order_acquire),
            };

            return true;
        }

        void safeStop() noexcept override {
            m_observation->enabled.store(
                false, std::memory_order_release);
            m_observation->stops.fetch_add(
                1, std::memory_order_release);
        }

    private:
        std::shared_ptr<SpindleObservation> m_observation;
        bool m_failCommands = false;
    };

    class NullMotionIo final : public ngc::ProductionExecutorIo {
    public:
        void sampleDigitalInputs(
            ngc::ProductionExecutorDigitalInputs
                &inputs) noexcept override {
            inputs = {};
        }

        void applyOutputs(
            const ngc::ProductionExecutorOutputState
                &outputs) noexcept override {
            lastEnabled.store(
                outputs.executorEnabled,
                std::memory_order_release);
        }

        std::atomic<bool> lastEnabled{false};
    };

    void waitFor(
        const auto &predicate,
        const std::string_view message) {
        const auto deadline =
            std::chrono::steady_clock::now() + 1s;
        while (std::chrono::steady_clock::now() < deadline) {
            if (predicate()) {
                return;
            }
            std::this_thread::sleep_for(1ms);
        }

        throw std::runtime_error(std::string(message));
    }

    void testLoadsPhysicalBackendConfiguration() {
        const auto configuration =
            ngc::physical::loadPhysicalBackendConfiguration(
                std::filesystem::path(NGC_SOURCE_DIR)
                / "physical_backend.toml");
        require(
            configuration.has_value(),
            configuration.error_or(
                "physical backend configuration did not load"));
        require(
            configuration->motion.expectedBoard == "7i96",
            "physical configuration lost its Mesa motion role");
        require(
            configuration->spindle.has_value(),
            "physical configuration lost its spindle role");
        const auto &spindle = *configuration->spindle;
        require(
            !spindle.enabled,
            "uncommissioned spindle must remain disabled");
        require(
            spindle.device == "/dev/ttyUSB0"
                && spindle.baud == 9600
                && spindle.dataBits == 8
                && spindle.parity
                    == ngc::physical::SerialParity::None
                && spindle.stopBits == 1
                && spindle.slaveAddress == 1,
            "legacy Huanyang serial settings changed");
        require(
            spindle.maximumSpeed == 24000.0
                && spindle.atSpeedTolerance == 0.02,
            "legacy Huanyang speed settings changed");
    }

    void testPhysicalIoPublishesSpindleOffServoThread() {
        auto observation =
            std::make_shared<SpindleObservation>();
        {
            auto io = ngc::PhysicalProductionExecutorIo(
                std::make_unique<NullMotionIo>(),
                std::make_unique<ObservedSpindle>(
                    observation));
            auto outputs =
                ngc::ProductionExecutorOutputState{};
            outputs.executorEnabled = true;
            outputs.spindle = {
                .enabled = true,
                .direction = ngc::Direction::CW,
                .speed = 12000.0,
            };
            io.applyOutputs(outputs);
            waitFor(
                [&] {
                    return observation->commands.load(
                        std::memory_order_acquire) == 1;
                },
                "spindle worker did not apply desired state");
            require(
                observation->enabled.load(
                    std::memory_order_acquire)
                    && observation->speed.load(
                        std::memory_order_acquire)
                        == 12000.0,
                "spindle worker changed desired state");

            io.applyOutputs(outputs);
            std::this_thread::sleep_for(30ms);
            require(
                observation->commands.load(
                    std::memory_order_acquire) == 1,
                "unchanged spindle state was republished");
        }
        require(
            observation->stops.load(
                std::memory_order_acquire) != 0
                && !observation->enabled.load(
                    std::memory_order_acquire),
            "spindle worker shutdown did not establish safe stop");
    }

    void testSpindleCommunicationFailureEstablishesSafeStop() {
        auto observation =
            std::make_shared<SpindleObservation>();
        auto io = ngc::PhysicalProductionExecutorIo(
            std::make_unique<NullMotionIo>(),
            std::make_unique<ObservedSpindle>(
                observation, true));
        auto outputs =
            ngc::ProductionExecutorOutputState{};
        outputs.executorEnabled = true;
        outputs.spindle = {
            .enabled = true,
            .direction = ngc::Direction::CCW,
            .speed = 6000.0,
        };
        io.applyOutputs(outputs);
        waitFor(
            [&] {
                return io.faultCode()
                    == ngc::SPINDLE_COMMUNICATION_FAULT;
            },
            "spindle communication failure did not latch");
        waitFor(
            [&] {
                return observation->stops.load(
                    std::memory_order_acquire) != 0;
            },
            "spindle communication failure did not stop hardware");
        require(
            !observation->enabled.load(
                std::memory_order_acquire),
            "faulted spindle did not retain safe state");
    }
}

int main() {
    try {
        testLoadsPhysicalBackendConfiguration();
        testPhysicalIoPublishesSpindleOffServoThread();
        testSpindleCommunicationFailureEstablishesSafeStop();
    } catch (const std::exception &error) {
        std::println(
            stderr, "physical backend test failed: {}",
            error.what());

        return 1;
    }

    std::println("physical backend tests passed");

    return 0;
}
