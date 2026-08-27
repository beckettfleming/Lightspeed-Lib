/**
 * \file lightspeed/telemetry/dashboard_layout.hpp
 *
 * Pixel layout for the live driver-control dashboard: just a handful of
 * text lines and one fault indicator, since (unlike the Step 7 selector)
 * nothing here is tappable.
 */

#pragma once

#include <cstdint>

namespace lightspeed::telemetry {

// Standard V5 Brain touch screen resolution (see also
// lightspeed::auton::kScreenWidth/kScreenHeight -- duplicated here rather
// than shared across modules, same as every other hardware constant in this
// project).
inline constexpr std::int16_t kDashboardScreenWidth = 480;
inline constexpr std::int16_t kDashboardScreenHeight = 272;

// pros::screen::print's "line" overload addresses text by line index
// (0, 1, 2, ...), not raw pixels -- these are line indices, not offsets.
inline constexpr std::int16_t kDashboardTitleLine = 0;
inline constexpr std::int16_t kDashboardBatteryLine = 1;
inline constexpr std::int16_t kDashboardPoseLine = 2;
inline constexpr std::int16_t kDashboardConfidenceLine = 3;
inline constexpr std::int16_t kDashboardSubsystemLine = 4;

// Fault indicator: a plain pixel-space rect (uses the x,y print overload
// for its label), not a text line.
inline constexpr std::int16_t kDashboardFaultBoxX0 = 10;
inline constexpr std::int16_t kDashboardFaultBoxY0 = 140;
inline constexpr std::int16_t kDashboardFaultBoxX1 = 300;
inline constexpr std::int16_t kDashboardFaultBoxY1 = 170;

}  // namespace lightspeed::telemetry
