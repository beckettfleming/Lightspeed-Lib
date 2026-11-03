/**
 * \file lightspeed/auton/start_location.hpp
 *
 * Named starting locations for the GUI's screen 1. Tapping one calls
 * odometry.setPose(), which is how the robot knows where it starts.
 *
 * TODO: replace with the actual season field layout -- these are
 * placeholder labels and poses only.
 */

#pragma once

#include <cstddef>
#include <vector>

#include "lightspeed/odom/types.hpp"

namespace lightspeed::auton {

struct StartLocation {
    const char* name;
    odom::Pose pose;  // field-relative starting pose
};

// TODO: replace with actual season field layout.
inline const std::vector<StartLocation> kStartLocations{
    StartLocation{.name = "Placeholder Start A", .pose = {.xInches = 12.0, .yInches = 12.0, .headingDegrees = 0.0}},
    StartLocation{.name = "Placeholder Start B", .pose = {.xInches = 132.0, .yInches = 12.0, .headingDegrees = 180.0}},
    StartLocation{.name = "Close Alliance", .pose = {.xInches = 0.0, .yInches = 11.375, .headingDegrees = 180.0}},
};

inline constexpr std::size_t kCloseAllianceStartLocationIndex = 2;

}  // namespace lightspeed::auton
