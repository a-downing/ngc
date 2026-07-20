#include "pendant/VistaCncP2sDisplay.h"

#include <cmath>
#include <format>
#include <string_view>

namespace ngc::pendant::vista_cnc_p2s {
    namespace {
        char axisLetter(const Axis axis) noexcept {
            switch (axis) {
                case Axis::X: return 'X';
                case Axis::Y: return 'Y';
                case Axis::Z: return 'Z';
                case Axis::A: return 'A';
                case Axis::B: return 'B';
                case Axis::C: return 'C';
            }

            return '?';
        }

        std::string positionRow(double value) {
            if (!std::isfinite(value)) {
                return "  ERROR ";
            }

            if (std::abs(value) < 0.00005) {
                value = 0.0;
            }

            for (int precision = 4; precision >= 0; --precision) {
                auto result = std::format("{:+.{}f}", value, precision);

                if (result.size() <= 8) {
                    result.insert(result.begin(), 8 - result.size(), ' ');
                    return result;
                }
            }

            auto result = std::format("{:+.0e}", value);

            if (result.size() > 8) {
                result.resize(8);
            } else {
                result.insert(result.begin(), 8 - result.size(), ' ');
            }

            return result;
        }

        std::string heading(std::string_view workCoordinateSystem, const Axis axis) {
            std::string result(workCoordinateSystem.substr(0, 5));

            result.push_back(' ');
            result.push_back(axisLetter(axis));

            if (result.size() > 8) {
                result.resize(8);
            } else {
                result.append(8 - result.size(), ' ');
            }

            return result;
        }

        std::string compactSigned(double value, const std::size_t width) {
            if (!std::isfinite(value)) {
                return std::string(width, '?');
            }

            if (std::abs(value) < 0.0000005) {
                value = 0.0;
            }

            for (int precision = 6; precision >= 0; --precision) {
                auto result = std::format("{:+.{}f}", value, precision);

                if (result.size() > width && result.size() >= 3 && result[1] == '0' && result[2] == '.') {
                    result.erase(1, 1);
                }

                if (result.size() <= width) {
                    result.append(width - result.size(), ' ');
                    return result;
                }
            }

            auto result = std::format("{:+.0e}", value);

            if (result.size() > width) {
                result.resize(width);
            } else {
                result.append(width - result.size(), ' ');
            }

            return result;
        }
    }

    std::string formatPositionDisplay(const std::string_view workCoordinateSystem, const Axis axis, const double workPosition) {
        return heading(workCoordinateSystem, axis) + positionRow(workPosition);
    }


    std::string formatZeroDisplay(const std::string_view workCoordinateSystem, const Axis axis, const double workPosition, const double stagedWorkPosition) {
        std::string top(1, axisLetter(axis));

        top += compactSigned(workPosition, 7);

        auto name = std::string(workCoordinateSystem.substr(0, 7));
        const auto valueWidth = 8 - name.size();

        return top + name + compactSigned(stagedWorkPosition, valueWidth);
    }
}
