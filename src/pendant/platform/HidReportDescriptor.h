#pragma once

#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>
#include <string>

namespace ngc::pendant::hid_detail {
    struct HidApiReportLengths {
        std::size_t input = 0;
        std::size_t output = 0;
    };

    std::expected<HidApiReportLengths, std::string>
    hidApiReportLengths(std::span<const std::uint8_t> descriptor);
}
