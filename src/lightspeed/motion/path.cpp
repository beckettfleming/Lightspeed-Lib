#include "lightspeed/motion/path.hpp"

#include <algorithm>
#include <cmath>

namespace lightspeed::motion {

std::vector<Waypoint> buildSmoothPath(const std::vector<Waypoint>& rawWaypoints, const PathSmoothingConfig& config) {
    if (rawWaypoints.size() < 2) {
        return rawWaypoints;
    }

    // 1. Point injection: evenly-spaced points along each original segment.
    std::vector<Waypoint> injected;
    injected.push_back(rawWaypoints.front());
    for (std::size_t i = 1; i < rawWaypoints.size(); ++i) {
        const Waypoint& a = rawWaypoints[i - 1];
        const Waypoint& b = rawWaypoints[i];
        const double segmentLength = std::hypot(b.x - a.x, b.y - a.y);
        const std::size_t numPoints =
            segmentLength > 0.0 ? static_cast<std::size_t>(std::ceil(segmentLength / config.injectionSpacingInches)) : 1;
        for (std::size_t p = 1; p <= numPoints; ++p) {
            const double t = static_cast<double>(p) / static_cast<double>(numPoints);
            injected.push_back(Waypoint{.x = a.x + (b.x - a.x) * t, .y = a.y + (b.y - a.y) * t});
        }
    }

    // 2. Gradient-descent smoothing.
    std::vector<Waypoint> smoothed = injected;
    for (std::uint32_t iteration = 0; iteration < config.maxSmoothingIterations; ++iteration) {
        double maxChange = 0.0;
        for (std::size_t i = 1; i + 1 < smoothed.size(); ++i) {
            const double oldX = smoothed[i].x;
            const double oldY = smoothed[i].y;

            const double newX = oldX + config.weightData * (injected[i].x - oldX) +
                                 config.weightSmooth * (smoothed[i - 1].x + smoothed[i + 1].x - 2.0 * oldX);
            const double newY = oldY + config.weightData * (injected[i].y - oldY) +
                                 config.weightSmooth * (smoothed[i - 1].y + smoothed[i + 1].y - 2.0 * oldY);

            smoothed[i].x = newX;
            smoothed[i].y = newY;

            maxChange = std::max({maxChange, std::abs(newX - oldX), std::abs(newY - oldY)});
        }
        if (maxChange < config.smoothingToleranceInches) {
            break;
        }
    }

    return smoothed;
}

}  // namespace lightspeed::motion
