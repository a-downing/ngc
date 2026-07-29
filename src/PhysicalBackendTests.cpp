#include <atomic>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <format>
#include <memory>
#include <optional>
#include <print>
#include <span>
#include <stdexcept>
#include <string_view>
#include <thread>

#ifdef __linux__
#include <cstdlib>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#endif

#include "machine/PhysicalExecutorIo.h"
#include "physical/HuanyangSpindleHardware.h"
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
            const ngc::ProductionExecutorMotionContext &,
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

    struct HuanyangObservation {
        std::uint8_t control = 0;
        std::uint16_t frequency = 0;
        std::uint16_t outputFrequency = 0;
        std::uint16_t outputCurrent = 0;
        std::uint32_t parameterReads = 0;
        bool failExchange = false;
        bool corruptCrc = false;
        bool corruptFunction = false;
        bool corruptEcho = false;
    };

    class FakeSerialTransport final
        : public ngc::physical::SerialTransport {
    public:
        explicit FakeSerialTransport(
            std::shared_ptr<HuanyangObservation> observation)
            : m_observation(std::move(observation)) { }

        bool exchange(
            const std::span<const std::uint8_t> request,
            const std::span<std::uint8_t> response,
            std::size_t &responseSize) noexcept override {
            responseSize = 0;
            if (m_observation->failExchange
                || request.size() < 6
                || request[0] != 1) {
                return false;
            }
            const auto requestCrc =
                ngc::physical::huanyangCrc16(
                    request.first(request.size() - 2));
            if (request[request.size() - 2]
                    != static_cast<std::uint8_t>(requestCrc)
                || request[request.size() - 1]
                    != static_cast<std::uint8_t>(
                        requestCrc >> 8)) {
                return false;
            }

            auto payload = std::array<std::uint8_t, 6>{};
            auto payloadSize = std::size_t{0};
            switch (request[1]) {
                case 0x01: {
                    const auto value = parameter(request[3]);
                    if (!value.has_value()) {
                        return false;
                    }
                    payload = {
                        1, 0x01, 3, request[3],
                        static_cast<std::uint8_t>(*value >> 8),
                        static_cast<std::uint8_t>(*value),
                    };
                    payloadSize = 6;
                    ++m_observation->parameterReads;
                    break;
                }
                case 0x03:
                    m_observation->control = request[3];
                    payload = {
                        1, 0x03, 1, request[3], 0, 0,
                    };
                    payloadSize = 4;
                    break;
                case 0x04: {
                    const auto value = request[3] == 1
                        ? m_observation->outputFrequency
                        : m_observation->outputCurrent;
                    payload = {
                        1, 0x04, 3, request[3],
                        static_cast<std::uint8_t>(value >> 8),
                        static_cast<std::uint8_t>(value),
                    };
                    payloadSize = 6;
                    break;
                }
                case 0x05:
                    m_observation->frequency =
                        static_cast<std::uint16_t>(
                            request[3] << 8 | request[4]);
                    payload = {
                        1, 0x05, 2, request[3], request[4], 0,
                    };
                    payloadSize = 5;
                    break;
                default: return false;
            }
            if (payloadSize + 2 > response.size()) {
                return false;
            }
            if (m_observation->corruptFunction) {
                payload[1] ^= 1;
            }
            if (m_observation->corruptEcho
                && payloadSize > 3) {
                payload[3] ^= 1;
            }
            std::ranges::copy(
                std::span(payload).first(payloadSize),
                response.begin());
            auto crc = ngc::physical::huanyangCrc16(
                response.first(payloadSize));
            if (m_observation->corruptCrc) {
                crc ^= 1;
            }
            response[payloadSize] =
                static_cast<std::uint8_t>(crc);
            response[payloadSize + 1] =
                static_cast<std::uint8_t>(crc >> 8);
            responseSize = payloadSize + 2;

            return true;
        }

    private:
        static std::optional<std::uint16_t> parameter(
            const std::uint8_t number) noexcept {
            switch (number) {
                case 4: return 40000;
                case 5: return 40000;
                case 11: return 1000;
                case 141: return 2200;
                case 142: return 100;
                case 143: return 2;
                case 144: return 3000;
                default: return std::nullopt;
            }
        }

        std::shared_ptr<HuanyangObservation> m_observation;
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
            configuration->runtime.realtimeEnabled
                && configuration->runtime.realtimeCpu == 15
                && configuration->runtime.realtimePriority == 98
                && configuration->runtime.lockMemory,
            "physical configuration lost its executor host policy");
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
            auto io = ngc::PhysicalExecutorIo(
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
        auto io = ngc::PhysicalExecutorIo(
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

    ngc::physical::HuanyangSpindleConfiguration
    huanyangConfiguration() {
        return {
            .enabled = true,
            .device = "fake",
            .baud = 9600,
            .dataBits = 8,
            .parity = ngc::physical::SerialParity::None,
            .stopBits = 1,
            .slaveAddress = 1,
            .maximumSpeed = 24000.0,
            .atSpeedTolerance = 0.02,
        };
    }

    void testHuanyangCrcAndProtocolScaling() {
        const std::array stopRequest{
            std::uint8_t{1}, std::uint8_t{3},
            std::uint8_t{1}, std::uint8_t{8},
        };
        require(
            ngc::physical::huanyangCrc16(stopRequest) == 0x8EF1,
            "Huanyang CRC changed");

        auto observation =
            std::make_shared<HuanyangObservation>();
        auto hardware =
            ngc::physical::HuanyangSpindleHardware::create(
                huanyangConfiguration(),
                std::make_unique<FakeSerialTransport>(
                    observation));
        require(
            hardware.has_value(),
            hardware.error_or(
                "Huanyang spindle did not initialize"));
        require(
            observation->control == 0x08
                && observation->parameterReads == 7,
            "Huanyang initialization did not stop and read setup");
        const auto &setup = (*hardware)->setup();
        require(
            setup.baseFrequency == 400.0
                && setup.maximumFrequency == 400.0
                && setup.minimumFrequency == 10.0
                && setup.ratedVoltage == 220.0
                && setup.ratedCurrent == 10.0
                && setup.motorPoles == 2
                && setup.ratedSpeedAt50Hz == 3000.0
                && setup.ratedMaximumSpeed == 24000.0,
            "Huanyang setup values were scaled incorrectly");

        require(
            (*hardware)->applyDesired({
                .enabled = true,
                .direction = ngc::Direction::CW,
                .speed = 12000.0,
            }),
            "Huanyang forward command failed");
        require(
            observation->frequency == 20000
                && observation->control == 0x01,
            "Huanyang forward command used incorrect wire values");

        observation->outputFrequency = 20000;
        observation->outputCurrent = 75;
        auto status = ngc::SpindleHardwareStatus{};
        require(
            (*hardware)->pollStatus(status),
            "Huanyang status polling failed");
        require(
            status.communicationHealthy
                && status.atSpeed
                && std::abs(status.speed - 12000.0) < 1e-9
                && std::abs(status.current - 7.5) < 1e-9,
            "Huanyang status scaling was incorrect");

        require(
            (*hardware)->applyDesired({
                .enabled = true,
                .direction = ngc::Direction::CCW,
                .speed = 6000.0,
            }),
            "Huanyang reverse command failed");
        require(
            observation->frequency == 10000
                && observation->control == 0x11,
            "Huanyang reverse command used incorrect wire values");
        require(
            (*hardware)->applyDesired({}),
            "Huanyang stop command failed");
        require(
            observation->control == 0x08,
            "Huanyang stop command used incorrect wire value");
    }

    void testHuanyangRejectsInvalidResponsesAndCommands() {
        auto observation =
            std::make_shared<HuanyangObservation>();
        auto hardware =
            ngc::physical::HuanyangSpindleHardware::create(
                huanyangConfiguration(),
                std::make_unique<FakeSerialTransport>(
                    observation));
        require(
            hardware.has_value(),
            "Huanyang spindle did not initialize");
        require(
            !(*hardware)->applyDesired({
                .enabled = true,
                .direction = ngc::Direction::CW,
                .speed = 24000.1,
            }),
            "Huanyang spindle accepted excessive speed");

        observation->corruptCrc = true;
        auto status = ngc::SpindleHardwareStatus{};
        require(
            !(*hardware)->pollStatus(status),
            "Huanyang spindle accepted a corrupt response");
        observation->corruptCrc = false;
        observation->corruptFunction = true;
        require(
            !(*hardware)->pollStatus(status),
            "Huanyang spindle accepted the wrong function");
        observation->corruptFunction = false;
        observation->corruptEcho = true;
        require(
            !(*hardware)->pollStatus(status),
            "Huanyang spindle accepted the wrong selector echo");
        observation->corruptEcho = false;
        observation->failExchange = true;
        (*hardware)->safeStop();
        require(
            observation->control == 0x08,
            "failed Huanyang safe stop changed prior safe state");
    }

#ifdef __linux__
    void testPosixSerialTransportWithPseudoTerminal() {
        const auto master = posix_openpt(
            O_RDWR | O_NOCTTY | O_CLOEXEC);
        require(master >= 0, "failed to open pseudo-terminal");
        require(
            grantpt(master) == 0 && unlockpt(master) == 0,
            "failed to prepare pseudo-terminal");
        const auto *slaveName = ptsname(master);
        require(
            slaveName != nullptr,
            "failed to resolve pseudo-terminal slave");

        auto configuration = huanyangConfiguration();
        configuration.device = slaveName;
        auto transport = ngc::physical::openSerialTransport(
            configuration, 50ms);
        require(
            transport.has_value(),
            transport.error_or(
                "failed to open pseudo-terminal transport"));

        const std::array request{
            std::uint8_t{1}, std::uint8_t{3},
            std::uint8_t{1}, std::uint8_t{8},
            std::uint8_t{0xF1}, std::uint8_t{0x8E},
        };
        auto responderSucceeded = std::atomic<bool>{false};
        auto responder = std::thread([&] {
            auto descriptor = pollfd{
                .fd = master,
                .events = POLLIN,
                .revents = 0,
            };
            if (poll(&descriptor, 1, 1000) <= 0) {
                return;
            }
            auto received =
                std::array<std::uint8_t, request.size()>{};
            const auto count = read(
                master, received.data(), received.size());
            if (count != static_cast<ssize_t>(received.size())
                || received != request) {
                return;
            }
            auto response = std::array<std::uint8_t, 6>{
                1, 3, 1, 8, 0, 0,
            };
            const auto crc = ngc::physical::huanyangCrc16(
                std::span(response).first(4));
            response[4] = static_cast<std::uint8_t>(crc);
            response[5] =
                static_cast<std::uint8_t>(crc >> 8);
            if (write(
                    master, response.data(), response.size())
                == static_cast<ssize_t>(response.size())) {
                responderSucceeded.store(
                    true, std::memory_order_release);
            }
        });

        auto response = std::array<std::uint8_t, 8>{};
        auto responseSize = std::size_t{0};
        const auto exchanged = (*transport)->exchange(
            request, response, responseSize);
        responder.join();
        require(
            exchanged
                && responderSucceeded.load(
                    std::memory_order_acquire)
                && responseSize == 6
                && response[0] == 1
                && response[1] == 3
                && response[2] == 1
                && response[3] == 8,
            "pseudo-terminal serial exchange failed");

        const auto timeoutStart =
            std::chrono::steady_clock::now();
        require(
            !(*transport)->exchange(
                request, response, responseSize),
            "serial transport did not time out");
        require(
            std::chrono::steady_clock::now() - timeoutStart
                < 500ms,
            "serial timeout exceeded its bound");

        close(master);
        require(
            !(*transport)->exchange(
                request, response, responseSize),
            "serial transport accepted a disconnected peer");
    }
#endif
}

int main() {
    try {
        testLoadsPhysicalBackendConfiguration();
        testPhysicalIoPublishesSpindleOffServoThread();
        testSpindleCommunicationFailureEstablishesSafeStop();
        testHuanyangCrcAndProtocolScaling();
        testHuanyangRejectsInvalidResponsesAndCommands();
#ifdef __linux__
        testPosixSerialTransportWithPseudoTerminal();
#endif
    } catch (const std::exception &error) {
        std::println(
            stderr, "physical backend test failed: {}",
            error.what());

        return 1;
    }

    std::println("physical backend tests passed");

    return 0;
}
