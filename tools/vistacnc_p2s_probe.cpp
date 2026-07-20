#include <chrono>
#include <cstdint>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>

#include "pendant/VistaCncP2sDriver.h"

namespace {
    namespace protocol = ngc::pendant::vista_cnc_p2s;

    std::string hexReport(const protocol::InputReport &report) {
        std::ostringstream out;
        out << std::hex << std::setfill('0');
        for(const auto byte : report) out << std::setw(2) << static_cast<unsigned>(byte) << ' ';
        return out.str();
    }

    void describe(std::ostream &out, const protocol::InputState &state,
                  const protocol::InputReport *previous, const double milliseconds) {
        const auto &report = state.raw;
        out << std::fixed << std::setprecision(1) << milliseconds << " ms  " << hexReport(report)
            << " wheel=" << static_cast<unsigned>(state.wheelPosition);
        if(state.wheelDeltaValid && state.wheelDelta)
            out << " delta=" << static_cast<int>(state.wheelDelta);
        out << " rate_code=" << static_cast<unsigned>(state.wheelRateCode)
            << " rate_counts_s=" << static_cast<unsigned>(state.wheelRateCode) * 4U
            << " accumulator=" << state.wheelMotionAccumulator;
        out << " left=" << protocol::name(state.leftSelector)
            << " right=" << protocol::name(state.rightSelector)
            << " button=" << static_cast<unsigned>(state.wheelButton)
            << " estop=" << state.emergencyStop << " off=" << state.machineOff;
        if(previous) {
            out << " changed=";
            bool any = false;
            for(std::size_t index = 0; index < report.size(); ++index) {
                if(report[index] == (*previous)[index]) continue;
                out << (any ? "," : "") << index;
                any = true;
            }
            if(!any) out << "none";
        }
        out << '\n' << std::flush;
    }
}

int main(const int argc, char **argv) {
    std::cout << "VistaCNC P2-S decoded report probe (VID 04d8, PID fce2)\n"
                 "Press Ctrl+C to stop. Reports are printed only when bytes change.\n";
    auto opened = protocol::Driver::open();
    if(!opened) {
        std::cerr << "Could not open pendant: " << opened.error().message;
        if(opened.error().systemCode != 0) std::cerr << " (error " << opened.error().systemCode << ')';
        std::cerr << '\n';
        return 1;
    }
    auto driver = std::move(*opened);

    std::ofstream capture;
    std::ostream *reports = &std::cout;
    for(int index = 1; index + 1 < argc; ++index) {
        if(std::string_view(argv[index]) != "--output") continue;
        capture.open(argv[index + 1], std::ios::trunc);
        if(!capture) {
            std::cerr << "Could not open capture output: " << argv[index + 1] << '\n';
            return 5;
        }
        reports = &capture;
    }

    const std::string displayText = argc >= 3 && std::string_view(argv[1]) == "--display" ? argv[2] : "";
    if(!displayText.empty()) {
        auto written = driver->writeDisplay(displayText);
        if(!written) {
            std::cerr << "Display write failed: " << written.error().message << '\n';
            return 2;
        }
        std::cout << "Streaming display text: " << displayText << '\n';
    }

    const auto start = std::chrono::steady_clock::now();
    protocol::InputReport previous{};
    bool havePrevious = false;
    while(true) {
        auto decoded = driver->read(protocol::REPORT_TIMEOUT);
        if(!decoded) {
            std::cerr << "Pendant read stopped: " << decoded.error().message << '\n';
            return decoded.error().code == ngc::pendant::HidErrorCode::Cancelled ? 0 : 3;
        }
        if(!havePrevious || decoded->raw != previous) {
            const auto elapsed = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
            describe(*reports, *decoded, havePrevious ? &previous : nullptr, elapsed);
            previous = decoded->raw;
            havePrevious = true;
        }
        if(!displayText.empty()) {
            auto written = driver->writeDisplay(displayText);
            if(!written) {
                std::cerr << "Display refresh failed: " << written.error().message << '\n';
                return 4;
            }
        }
    }
}
