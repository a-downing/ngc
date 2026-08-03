#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>

#include "machine/EmergencyStop.h"
#include "machine/MachineSessionManager.h"

namespace {
    void require(const bool condition, const std::string_view message) {
        if (!condition) {
            throw std::runtime_error(std::string(message));
        }
    }

    void testSharedLatchAndResetHandshake() {
        auto control = ngc::EmergencyStopControlBlock{};
        auto emergencyControl = ngc::EmergencyStopInterface(control);
        auto state = ngc::EmergencyStopState(control);

        emergencyControl.request(ngc::EmergencyStopSource::Gui);
        require(state.update(),
                "the shared emergency-stop state did not latch a GUI request");
        emergencyControl.release(ngc::EmergencyStopSource::Gui);
        require(state.update(),
                "releasing the GUI emergency stop cleared its latch");
        const auto resetGeneration = emergencyControl.requestReset();
        require(!state.update(),
                "an explicit clear-source reset did not clear the emergency stop");
        const auto resetStatus = emergencyControl.status();
        require(resetStatus.acknowledgedResetGeneration == resetGeneration
                    && resetStatus.resetResult
                        == ngc::EmergencyStopResetResult::Cleared,
                "the shared emergency-stop reset was not acknowledged");

        const auto physicalSource = ngc::emergencyStopSourceMask(
            ngc::EmergencyStopSource::PhysicalExternalEnable);
        require(state.update(physicalSource),
                "the shared emergency-stop state did not latch a physical source");
        const auto blockedGeneration = emergencyControl.requestReset();
        require(state.update(physicalSource),
                "reset cleared an active physical emergency-stop source");
        const auto blockedStatus = emergencyControl.status();
        require(blockedStatus.acknowledgedResetGeneration == blockedGeneration
                    && blockedStatus.resetResult
                        == ngc::EmergencyStopResetResult::BlockedByActiveSource,
                "reset did not report the active physical emergency-stop source");
        static_cast<void>(emergencyControl.requestReset());
        require(!state.update(),
                "reset did not clear the released physical emergency-stop source");
    }

    void testSimulationSessionRecoveryRequiresPowerOn() {
        ngc::MachineSessionManager manager;
        const auto authority = manager.state().authority;
        require(manager.powerOn(authority),
                "emergency-stop fixture could not power Simulation on");
        manager.setTickMultiplier(1);
        require(manager.start(
                    authority, {{"G1 F60 X10\n", "<MDI>"}}, {}, true),
                "emergency-stop fixture could not start Simulation motion");
        require(manager.emergencyStop(),
                "the global GUI emergency stop was rejected");
        const auto faulted = manager.snapshot();
        require(faulted.powerState == ngc::MachinePowerState::Faulted
                    && faulted.machineActivity
                        == ngc::MachineActivity::Faulted,
                "GUI emergency stop did not fault the Simulation session");
        require(!manager.powerOn(authority),
                "a latched GUI emergency stop allowed power on");

        auto reset = manager.resetEmergencyStop();
        for (auto attempt = 0; !reset && attempt < 2'000; ++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            reset = manager.resetEmergencyStop();
        }
        require(reset.has_value(), reset ? "" : reset.error());
        const auto off = manager.snapshot();
        require(off.powerState == ngc::MachinePowerState::Off
                    && off.emergencyStopLatchedSources == 0,
                "emergency-stop Reset did not leave Simulation off and clear");
        require(manager.powerOn(authority),
                "Simulation did not power on explicitly after emergency-stop Reset");
        require(manager.powerOff(authority),
                "emergency-stop fixture could not power Simulation off");
    }
}

int main() {
    try {
        testSharedLatchAndResetHandshake();
        testSimulationSessionRecoveryRequiresPowerOn();
    } catch (const std::exception &error) {
        std::cerr << "Emergency-stop test failure: " << error.what() << '\n';

        return 1;
    }

    std::cout << "Emergency-stop tests passed\n";

    return 0;
}
