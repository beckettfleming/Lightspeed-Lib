/**
 * \file lightspeed/telemetry/dashboard_layout.hpp
 *
 * Pixel layout for the live driver-control dashboard. Nothing here is
 * tappable, unlike the auton selector.
 */

#pragma once

#include <cstdint>

namespace lightspeed::telemetry {

// Duplicated from auton::kScreenWidth/kScreenHeight rather than shared --
// each module owns its own hardware constants.
inline constexpr std::int16_t kDashboardScreenWidth = 480;
inline constexpr std::int16_t kDashboardScreenHeight = 240;

// pros::screen::print's "line" overload addresses text by line index
// (0, 1, 2, ...), not raw pixels -- these are line indices, not offsets.
inline constexpr std::int16_t kDashboardTitleLine = 0;
inline constexpr std::int16_t kDashboardBatteryLine = 1;
inline constexpr std::int16_t kDashboardPoseLine = 2;
inline constexpr std::int16_t kDashboardConfidenceLine = 3;

}  // namespace lightspeed::telemetry
