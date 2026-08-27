/**
 * \file lightspeed/control/pidf_controller.hpp
 *
 * Generic PIDF/feedforward controller. It has no notion of "position" or
 * "velocity" -- it just tracks a measurement against a setpoint, so the same
 * class backs both the drivetrain's velocity control (this step) and a
 * future subsystem's position control, with gains supplied per use.
 */

#pragma once

#include <cstdint>

namespace lightspeed::control {

// All gains and tunable parameters, supplied at construction -- never
// hardcoded in the controller itself.
struct PIDFConfig {
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
    double kV = 0.0;  // feedforward: volts per unit of target velocity
    double kA = 0.0;  // feedforward: volts per unit of target acceleration
    double kS = 0.0;  // feedforward: volts to overcome static friction, applied as kS * sign(targetVelocity)

    // Windup guard for the integral term. integralZone <= 0 disables
    // zone-gating (always integrate); integralMax <= 0 disables the hard
    // clamp on the accumulated integral. Both may be set together.
    double integralZone = 0.0;
    double integralMax = 0.0;

    // isSettled() reports true once |error| <= settleTolerance for
    // settleCycles consecutive calculate() calls.
    double settleTolerance = 0.0;
    std::uint32_t settleCycles = 0;
};

// What the feedback loop tracks, plus the feedforward terms alongside it.
// `target` is a position when this controller is used for position control,
// or a velocity when used for velocity control -- the class itself has no
// opinion, it just tracks target vs. measurement.
struct Setpoint {
    double target = 0.0;
    double targetVelocity = 0.0;
    double targetAcceleration = 0.0;
};

class PIDFController {
public:
    explicit PIDFController(const PIDFConfig& config);

    // Advances the controller by dtSeconds and returns the new control
    // output (feedback + feedforward). Call once per control cycle.
    double calculate(double measurement, const Setpoint& setpoint, double dtSeconds);

    // Clears the integral accumulator, derivative history, and settle
    // counter. Call when switching to a new setpoint that shouldn't inherit
    // old accumulated state.
    void reset();

    // True once the last settleCycles consecutive calculate() calls were
    // within settleTolerance of the setpoint.
    [[nodiscard]] bool isSettled() const;

    void setConfig(const PIDFConfig& config);
    [[nodiscard]] const PIDFConfig& getConfig() const;

private:
    PIDFConfig config_;
    double integral_ = 0.0;
    double previousError_ = 0.0;
    bool hasPreviousError_ = false;
    std::uint32_t settledCycles_ = 0;
};

}  // namespace lightspeed::control
