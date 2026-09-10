/**
 * \file lightspeed/driver/input_profile.hpp
 *
 * Curve (expo) scaling + deadband for a single joystick axis. Exponent 1.0
 * is linear; greater than 1.0 compresses the low end while the endpoints
 * (-1, 0, 1) stay fixed.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/driver-control/
 */

#pragma once

namespace lightspeed::driver {

struct InputProfileConfig {
    double curveExponent = 1.0;  // 1.0 = linear; > 1.0 = more low-speed precision
    double deadband = 0.0;       // ignore |input| below this, in [0, 1)
};

class InputProfile {
public:
    explicit InputProfile(const InputProfileConfig& config);

    // Deadband, then curve scaling. Sign-preserving; output stays in [-1, 1].
    [[nodiscard]] double apply(double rawInput) const;

    void setConfig(const InputProfileConfig& config);
    [[nodiscard]] const InputProfileConfig& getConfig() const;

private:
    InputProfileConfig config_;
};

}  // namespace lightspeed::driver
