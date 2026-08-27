#include "lightspeed/control/slew_rate_limiter.hpp"

#include <algorithm>

namespace lightspeed::control {

SlewRateLimiter::SlewRateLimiter(double maxRatePerSecond) : maxRatePerSecond_(maxRatePerSecond) {}

double SlewRateLimiter::calculate(double target, double dtSeconds) {
    const double maxDelta = maxRatePerSecond_ * dtSeconds;
    const double delta = std::clamp(target - previousOutput_, -maxDelta, maxDelta);
    previousOutput_ += delta;
    return previousOutput_;
}

void SlewRateLimiter::setMaxRate(double maxRatePerSecond) {
    maxRatePerSecond_ = maxRatePerSecond;
}

double SlewRateLimiter::getMaxRate() const {
    return maxRatePerSecond_;
}

void SlewRateLimiter::reset(double value) {
    previousOutput_ = value;
}

}  // namespace lightspeed::control
