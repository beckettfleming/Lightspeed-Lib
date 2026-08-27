/**
 * \file lightspeed/motion/path.hpp
 *
 * Path construction for pure pursuit: point injection + gradient-descent
 * smoothing, the standard lightweight alternative to true splines at this
 * scale (a small set of hand-placed waypoints in, a dense smoothed point
 * list out).
 */

#pragma once

#include <cstdint>
#include <vector>

namespace lightspeed::motion {

struct Waypoint {
    double x;
    double y;
};

struct PathSmoothingConfig {
    double injectionSpacingInches;         // spacing between injected points along each original segment
    double weightData;                     // pull toward the original injected position; weightData + weightSmooth should stay < 1
    double weightSmooth;                   // pull toward neighboring points' average
    double smoothingToleranceInches;       // stop iterating once the largest per-point move drops below this
    std::uint32_t maxSmoothingIterations;
};

// Builds a smoothed, densely-spaced path from a small set of raw waypoints:
// injects evenly-spaced points along each segment, then iteratively pulls
// each interior point toward the average of its neighbors (weightSmooth)
// while keeping it near its originally-injected position (weightData).
// Endpoints are never moved. Returns rawWaypoints unchanged if it has fewer
// than 2 points.
[[nodiscard]] std::vector<Waypoint> buildSmoothPath(const std::vector<Waypoint>& rawWaypoints, const PathSmoothingConfig& config);

}  // namespace lightspeed::motion
