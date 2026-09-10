/**
 * \file lightspeed/control/slew_rate_limiter.hpp
 *
 * Standalone rate-of-change limiter, independent of PIDFController so it can
 * be reused anywhere a value needs ramping rather than stepping.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/control/
 */

#pragma once

namespace lightspeed::control {

class SlewRateLimiter {
public:
    explicit SlewRateLimiter(double maxRatePerSecond);

    // Advances the output toward target by at most maxRate * dtSeconds.
    // Call once per control cycle.
    double calculate(double target, double dtSeconds);

    // Swappable at runtime, not fixed at construction -- driver control
    // tightens this while a mechanism is extended.
    void setMaxRate(double maxRatePerSecond);
    [[nodiscard]] double getMaxRate() const;

    // Snaps the internal output to `value` without ramping.
    void reset(double value = 0.0);

private:
    double maxRatePerSecond_;
    double previousOutput_ = 0.0;
};

}  // namespace lightspeed::control
