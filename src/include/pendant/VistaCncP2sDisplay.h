#pragma once

#include <string>
#include <string_view>

#include "pendant/PendantIntent.h"

namespace ngc::pendant::vista_cnc_p2s {
    // Formats the two eight-character P2-S LCD rows. The first row identifies
    // the active WCS and selected axis; the second contains its signed position.
    std::string formatPositionDisplay(std::string_view workCoordinateSystem,
                                      Axis axis, double workPosition);

    // Zero mode shows the live work position on row one and the staged work
    // coordinate to apply at the current point on row two.
    std::string formatZeroDisplay(std::string_view workCoordinateSystem,
                                  Axis axis, double workPosition,
                                  double stagedWorkPosition);
}
