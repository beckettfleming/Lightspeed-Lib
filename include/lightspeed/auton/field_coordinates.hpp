/**
 * \file lightspeed/auton/field_coordinates.hpp
 *
 * Field-inches -> screen-pixels mapping shared by both selector screens.
 */

#pragma once

#include <cstdint>

namespace lightspeed::auton {

// Standard V5 Brain touch screen resolution.
inline constexpr std::int16_t kScreenWidth = 480;
inline constexpr std::int16_t kScreenHeight = 272;

struct ScreenPoint {
    std::int16_t x;
    std::int16_t y;
};

struct ScreenRegion {
    std::int16_t x0;
    std::int16_t y0;
    std::int16_t x1;
    std::int16_t y1;
};

// Field inches -> screen pixels: the field point (originXInches,
// originYInches) maps to a drawing region's bottom-left corner, scaled by
// pixelsPerInch (uniform on both axes). Field +y (forward at heading 0)
// maps to screen -y (up) since screen y grows downward.
//
// One config value is used for both screens: screen 1 draws at full size
// with this config as-is; screen 2's smaller preview pane derives a scaled
// copy (same origin, reduced pixelsPerInch) -- see SelectorGui.
struct FieldToScreenConfig {
    double originXInches;
    double originYInches;
    double pixelsPerInch;
};

struct FieldDimensions {
    double widthInches;
    double lengthInches;
};

[[nodiscard]] ScreenPoint fieldToScreen(double fieldXInches, double fieldYInches, const FieldToScreenConfig& config,
                                         const ScreenRegion& region);

}  // namespace lightspeed::auton
