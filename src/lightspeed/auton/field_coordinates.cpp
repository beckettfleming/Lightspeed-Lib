#include "lightspeed/auton/field_coordinates.hpp"

namespace lightspeed::auton {

ScreenPoint fieldToScreen(double fieldXInches, double fieldYInches, const FieldToScreenConfig& config,
                           const ScreenRegion& region) {
    const double dx = fieldXInches - config.originXInches;
    const double dy = fieldYInches - config.originYInches;
    return ScreenPoint{
        .x = static_cast<std::int16_t>(region.x0 + dx * config.pixelsPerInch),
        .y = static_cast<std::int16_t>(region.y1 - dy * config.pixelsPerInch),
    };
}

}  // namespace lightspeed::auton
