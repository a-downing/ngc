#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <string_view>

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#include <hidsdi.h>
#include <setupapi.h>
#else
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <filesystem>
#include <fstream>
#endif

namespace {
constexpr std::uint16_t vendor = 0x04d8;
constexpr std::uint16_t product = 0xfce2;
std::atomic_bool stopRequested = false;

std::string hexReport(const std::array<std::uint8_t, 8> &r) {
    std::ostringstream out;
    out << std::hex << std::setfill('0');
    for(const auto byte : r) out << std::setw(2) << static_cast<unsigned>(byte) << ' ';
    return out.str();
}

void describe(const std::array<std::uint8_t, 8> &r, const std::array<std::uint8_t, 8> *previous,
              const double milliseconds) {
    static constexpr const char *left[] = {"undefined", "step", "velocity", "continuous", "speed", "zero", "function", "off"};
    static constexpr const char *right[] = {"undefined", "X/F1", "Y/F2", "Z/F3", "start/pause", "stop/back", "spindle", "A/F4"};
    const auto wheelDelta = previous ? static_cast<int>(static_cast<std::int8_t>(r[0] - (*previous)[0])) : 0;
    std::cout << std::fixed << std::setprecision(1) << milliseconds << " ms  " << hexReport(r)
              << " wheel=" << static_cast<unsigned>(r[0]);
    if(previous && wheelDelta) std::cout << " delta=" << wheelDelta;
    std::cout << " left=" << left[r[3] & 7] << " right=" << right[r[2] & 7]
              << " button=" << static_cast<unsigned>((r[2] >> 6) & 3)
              << " estop=" << ((r[2] >> 5) & 1) << " off=" << ((r[2] >> 4) & 1);
    if(previous) {
        std::cout << " changed=";
        bool any = false;
        for(std::size_t i = 0; i < r.size(); ++i) if(r[i] != (*previous)[i]) {
            std::cout << (any ? "," : "") << i; any = true;
        }
        if(!any) std::cout << "none";
    }
    std::cout << '\n' << std::flush;
}

#ifdef _WIN32
HANDLE openPendant() {
    GUID guid{}; HidD_GetHidGuid(&guid);
    HDEVINFO set = SetupDiGetClassDevsW(&guid, nullptr, nullptr, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if(set == INVALID_HANDLE_VALUE) return INVALID_HANDLE_VALUE;
    for(DWORD index = 0;; ++index) {
        SP_DEVICE_INTERFACE_DATA item{};
        item.cbSize = sizeof(item);
        if(!SetupDiEnumDeviceInterfaces(set, nullptr, &guid, index, &item)) break;
        DWORD size = 0; SetupDiGetDeviceInterfaceDetailW(set, &item, nullptr, 0, &size, nullptr);
        std::string storage(size, '\0');
        auto *detail = reinterpret_cast<SP_DEVICE_INTERFACE_DETAIL_DATA_W *>(storage.data());
        detail->cbSize = sizeof(*detail);
        if(!SetupDiGetDeviceInterfaceDetailW(set, &item, detail, size, nullptr, nullptr)) continue;
        HANDLE handle = CreateFileW(detail->DevicePath, GENERIC_READ | GENERIC_WRITE,
            FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, 0, nullptr);
        if(handle == INVALID_HANDLE_VALUE) continue;
        HIDD_ATTRIBUTES attributes{};
        attributes.Size = sizeof(attributes);
        if(HidD_GetAttributes(handle, &attributes) && attributes.VendorID == vendor && attributes.ProductID == product) {
            SetupDiDestroyDeviceInfoList(set); return handle;
        }
        CloseHandle(handle);
    }
    SetupDiDestroyDeviceInfoList(set); return INVALID_HANDLE_VALUE;
}
#else
int openPendant() {
    for(const auto &entry : std::filesystem::directory_iterator("/sys/class/hidraw")) {
        std::ifstream uevent(entry.path() / "device/uevent");
        std::string line; bool matches = false;
        while(std::getline(uevent, line)) if(line.find("HID_ID=0003:000004D8:0000FCE2") != std::string::npos) matches = true;
        if(matches) return ::open(("/dev/" + entry.path().filename().string()).c_str(), O_RDWR);
    }
    return -1;
}
#endif

bool writeDisplay(
#ifdef _WIN32
        const HANDLE device,
#else
        const int device,
#endif
        const std::string_view text, const std::uint8_t sequence) {
    std::array<std::uint8_t, 20> payload{};
    std::fill_n(payload.begin(), 17, static_cast<std::uint8_t>(' '));
    std::memcpy(payload.data(), text.data(), std::min(text.size(), std::size_t{16}));
    payload[17] = sequence;
#ifdef _WIN32
    // HID WriteFile buffers include report ID at byte zero. This device has no
    // numbered reports, so prepend report ID zero to the 20-byte USB payload.
    std::array<std::uint8_t, 21> report{};
    std::memcpy(report.data() + 1, payload.data(), payload.size());
    DWORD count = 0;
    const auto written = WriteFile(device, report.data(), static_cast<DWORD>(report.size()), &count, nullptr)
        && count == report.size();
    // Some HID firmware accepts output reports only via the class control
    // transfer even though WriteFile reports success.
    const auto controlled = HidD_SetOutputReport(device, report.data(), static_cast<ULONG>(report.size())) != FALSE;
    return written || controlled;
#else
    std::array<std::uint8_t, 21> report{};
    std::memcpy(report.data() + 1, payload.data(), payload.size());
    return ::write(device, report.data(), report.size()) == static_cast<ssize_t>(report.size());
#endif
}
}

int main(const int argc, char **argv) {
    std::cout << "MPG P2S raw report probe (VID 04d8, PID fce2)\n"
                 "Press Ctrl+C to stop. Reports are printed only when bytes change.\n";
    auto device = openPendant();
#ifdef _WIN32
    if(device == INVALID_HANDLE_VALUE) { std::cerr << "Pendant not found or could not be opened (Windows error " << GetLastError() << ").\n"; return 1; }
#else
    if(device < 0) { std::perror("Pendant not found or could not be opened"); return 1; }
#endif
#ifdef _WIN32
    PHIDP_PREPARSED_DATA preparsed = nullptr;
    HIDP_CAPS caps{};
    if(HidD_GetPreparsedData(device, &preparsed)) {
        if(HidP_GetCaps(preparsed, &caps) == HIDP_STATUS_SUCCESS)
            std::cout << "HID report lengths: input=" << caps.InputReportByteLength
                      << " output=" << caps.OutputReportByteLength
                      << " feature=" << caps.FeatureReportByteLength << '\n';
        HidD_FreePreparsedData(preparsed);
    }
#endif
    const std::string displayText = argc >= 3 && std::string_view(argv[1]) == "--display" ? argv[2] : "";
    std::uint8_t displaySequence = 0;
    if(!displayText.empty()) {
        if(!writeDisplay(device, displayText, displaySequence++)) {
#ifdef _WIN32
            std::cerr << "Display write failed (Windows error " << GetLastError() << ").\n";
#else
            std::perror("Display write failed");
#endif
            return 2;
        }
        std::cout << "Streaming display text: " << displayText << '\n';
    }
    const auto start = std::chrono::steady_clock::now();
    std::array<std::uint8_t, 8> previous{}; bool havePrevious = false;
    while(!stopRequested) {
        // Windows HID APIs include a leading report-ID byte; this device uses report ID zero.
        std::array<std::uint8_t, 9> input{};
#ifdef _WIN32
        DWORD count = 0;
        if(!ReadFile(device, input.data(), static_cast<DWORD>(input.size()), &count, nullptr)) {
            std::cerr << "Read failed (Windows error " << GetLastError() << ").\n"; break;
        }
#else
        const auto count = ::read(device, input.data(), input.size());
        if(count < 0) { std::perror("Read failed"); break; }
#endif
        std::array<std::uint8_t, 8> report{};
        const std::size_t offset = count >= 9 ? 1 : 0;
        if(static_cast<std::size_t>(count) < offset + report.size()) continue;
        std::memcpy(report.data(), input.data() + offset, report.size());
        if(!havePrevious || report != previous) {
            const auto elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
            describe(report, havePrevious ? &previous : nullptr, elapsed);
            previous = report; havePrevious = true;
        }
        if(!displayText.empty() && !writeDisplay(device, displayText, displaySequence++)) {
            std::cerr << "Display refresh failed.\n";
            break;
        }
    }
#ifdef _WIN32
    CloseHandle(device);
#else
    ::close(device);
#endif
}
