#include "lightspeed/driver/input_profile.hpp"

#include <algorithm>
#include <cmath>

namespace lightspeed::driver {

InputProfile::InputProfile(const InputProfileConfig& config) : config_(config) {}

double InputProfile::apply(double rawInput) const {
    const double clamped = std::clamp(rawInput, -1.0, 1.0);
    const double magnitude = std::abs(clamped);

    if (magnitude < config_.deadband) {
        return 0.0;
    }

    // Rescale so output ramps continuously from 0 at the deadband edge to
    // 1 at full deflection, rather than jumping to a nonzero value right
    // past the deadband threshold.
    const double deadbandSpan = 1.0 - config_.deadband;
    const double rescaled = deadbandSpan > 0.0 ? (magnitude - config_.deadband) / deadbandSpan : magnitude;
    const double curved = std::pow(rescaled, config_.curveExponent);

    return std::copysign(curved, clamped);
}

void InputProfile::setConfig(const InputProfileConfig& config) {
    config_ = config;
}

const InputProfileConfig& InputProfile::getConfig() const {
    return config_;
}

}  // namespace lightspeed::driver
