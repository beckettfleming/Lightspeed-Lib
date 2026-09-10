/**
 * \file lightspeed/motion/path.hpp
 *
 * Path construction for pure pursuit: point injection + gradient-descent
 * smoothing. A small set of hand-placed waypoints in, a dense smoothed point
 * list out.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/motion/
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

// Endpoints are never moved. Returns rawWaypoints unchanged if it has fewer
// than 2 points.
[[nodiscard]] std::vector<Waypoint> buildSmoothPath(const std::vector<Waypoint>& rawWaypoints, const PathSmoothingConfig& config);

}  // namespace lightspeed::motion
