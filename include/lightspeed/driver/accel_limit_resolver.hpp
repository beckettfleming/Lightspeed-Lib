/**
 * \file lightspeed/driver/accel_limit_resolver.hpp
 *
 * Maps Step 4 flag-registry conditions to a max-acceleration value (RPM/s)
 * for the driver-control slew limiter. Every registered rule whose flag is
 * currently true contributes its value; the minimum (most restrictive)
 * across all of them wins. No true flags -> the configured default.
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

    // Minimum maxRpmPerSecond across every currently-true rule's flag, or
    // the configured default if none are true. Cheap -- safe to call every
    // driver-control cycle.
    [[nodiscard]] double resolve() const;

private:
    AccelLimitConfig config_;
};

}  // namespace lightspeed::driver
