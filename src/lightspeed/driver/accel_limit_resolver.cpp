#include "lightspeed/driver/accel_limit_resolver.hpp"

#include <algorithm>
#include <cstdio>
#include <optional>

#include "lightspeed/subsystem/flag_registry.hpp"

namespace lightspeed::driver {

AccelLimitResolver::AccelLimitResolver(const AccelLimitConfig& config) : config_(config) {
    for (const AccelLimitRule& rule : config_.rules) {
        if (!subsystem::FlagRegistry::instance().isRegistered(rule.flagName)) {
            std::printf(
                "[lightspeed::driver] WARNING: AccelLimitResolver rule references unregistered flag '%s' -- it "
                "will never trigger until something registers it.\n",
                rule.flagName);
        }
    }
}

double AccelLimitResolver::resolve() const {
    std::optional<double> mostRestrictive;
    for (const AccelLimitRule& rule : config_.rules) {
        if (subsystem::FlagRegistry::instance().getFlag(rule.flagName)) {
            mostRestrictive =
                mostRestrictive.has_value() ? std::min(*mostRestrictive, rule.maxRpmPerSecond) : rule.maxRpmPerSecond;
        }
    }
    return mostRestrictive.value_or(config_.defaultMaxRpmPerSecond);
}

}  // namespace lightspeed::driver
