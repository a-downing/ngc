#include "pendant/HidTransport.h"

namespace ngc::pendant {
    std::expected<std::unique_ptr<HidTransport>, HidError>
    openHidTransport(const HidDeviceSelector &) {
        return std::unexpected(HidError {
            HidErrorCode::Unsupported, 0,
            "the HID transport is not implemented on this platform",
        });
    }
}
