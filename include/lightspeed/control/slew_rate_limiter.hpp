/**
 * \file lightspeed/control/slew_rate_limiter.hpp
 *
 * Standalone rate-of-change limiter, independent of PIDFController so it can
 * be reused anywhere a value needs to be ramped rather than stepped -- this
 * step uses it on the drivetrain velocity controller's voltage output;
 * driver control will later reuse this same class with a dynamically
 * swapped max-rate parameter to cap acceleration while a mechanism is
 * active.
 */

#pragma once

namespace lightspeed::control {

class SlewRateLimiter {
public:
    explicit SlewRateLimiter(double maxRatePerSecond);

    // Advances the output toward target by at most maxRate * dtSeconds and
    // returns the new limited output. Call once per control cycle.
    double calculate(double target, double dtSeconds);

    // The max rate is swappable at runtime (e.g. driver control tightening
    // it while a mechanism is active), not fixed at construction.
    void setMaxRate(double maxRatePerSecond);
    [[nodiscard]] double getMaxRate() const;

    // Snaps the limiter's internal output to `value` without ramping,
    // e.g. when (re-)enabling a controller that shouldn't ramp up from
    // whatever the last output happened to be.
    void reset(double value = 0.0);

private:
    double maxRatePerSecond_;
    double previousOutput_ = 0.0;
};

}  // namespace lightspeed::control
