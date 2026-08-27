/**
 * \file lightspeed/auton/screen_layout.hpp
 *
 * Pixel layout for both selector screens: zone rects/radii, list geometry.
 * Centralized here so nothing in selector_gui.cpp hardcodes a raw pixel
 * number.
 */

#pragma once

#include "lightspeed/auton/field_coordinates.hpp"

namespace lightspeed::auton {

// -- Screen 1: start-location picker --
inline constexpr std::int16_t kTitleLine = 0;
inline constexpr ScreenRegion kScreen1FieldRegion{.x0 = 125, .y0 = 24, .x1 = 355, .y1 = 254};
inline constexpr std::int16_t kStartZoneRadiusPixels = 10;

// -- Screen 2: routine picker + preview --
inline constexpr ScreenRegion kPreviewRegion{.x0 = 10, .y0 = 30, .x1 = 160, .y1 = 180};
inline constexpr double kPreviewScaleFactor = 0.65;  // applied to kFieldToScreenConfig.pixelsPerInch

// Routine rows occupy a column to the right of the preview pane; a
// separate reserved column further right holds the scroll buttons, so
// neither ever overlaps a row.
inline constexpr std::int16_t kRoutineListX0 = 170;
inline constexpr std::int16_t kRoutineListX1 = 420;
inline constexpr std::int16_t kRoutineListY0 = 30;
inline constexpr std::int16_t kRoutineRowHeight = 32;
inline constexpr std::uint8_t kMaxVisibleRoutineRows = 5;
inline constexpr std::int16_t kRoutineListY1 = kRoutineListY0 + kMaxVisibleRoutineRows * kRoutineRowHeight;  // 190

// Scroll buttons, shown only when the filtered routine list overflows
// kMaxVisibleRoutineRows.
inline constexpr ScreenRegion kScrollUpZone{.x0 = 430, .y0 = kRoutineListY0, .x1 = 470, .y1 = kRoutineListY0 + 40};
inline constexpr ScreenRegion kScrollDownZone{.x0 = 430, .y0 = kRoutineListY1 - 40, .x1 = 470, .y1 = kRoutineListY1};

inline constexpr ScreenRegion kBackZone{.x0 = 10, .y0 = 236, .x1 = 110, .y1 = 266};
inline constexpr ScreenRegion kConfirmZone{.x0 = 370, .y0 = 236, .x1 = 470, .y1 = 266};

[[nodiscard]] inline bool insideRegion(std::int16_t x, std::int16_t y, const ScreenRegion& region) {
    return x >= region.x0 && x <= region.x1 && y >= region.y0 && y <= region.y1;
}

}  // namespace lightspeed::auton
