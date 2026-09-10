/**
 * \file lightspeed/driver/accel_limit_resolver.hpp
 *
 * Maps flag-registry conditions to a max-acceleration value (RPM/s) for the
 * driver-control slew limiter. Every rule whose flag is currently true
 * contributes; the minimum (most restrictive) wins. No true flags -> the
 * configured default.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/driver-control/
 */

#pragma once

#include <vector>

namespace lightspeed::driver {

struct AccelLimitRule {
    const char* flagName;  // queried from lightspeed::subsystem::FlagRegistry
    double maxRpmPerSecond;
};

struct AccelLimitConfig {
    double defaultMaxRpmPerSecond;
    std::vector<AccelLimitRule> rules;
};

class AccelLimitResolver {
public:
    explicit AccelLimitResolver(const AccelLimitConfig& config);

    // Cheap -- safe to call every driver-control cycle.
    [[nodiscard]] double resolve() const;

private:
    AccelLimitConfig config_;
};

}  // namespace lightspeed::driver
