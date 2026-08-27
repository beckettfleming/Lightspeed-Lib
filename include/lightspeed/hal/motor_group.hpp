/**
 * \file lightspeed/hal/motor_group.hpp
 *
 * Generic HAL wrapper around a ganged group of V5 motors, controlled by raw
 * voltage. Reusable by any subsystem (drivetrain, lift, intake, ...) --
 * construct one per logical device named in lightspeed::hal::config.
 */

#pragma once

#include <cstdint>

#include "lightspeed/hal/config.hpp"
#include "pros/motor_group.hpp"

namespace lightspeed::hal {

// Single-flag health summary for a motor group, so callers don't need to
// interpret raw PROS telemetry (temperature, faults, velocity vs. commanded
// voltage) themselves.
enum class HealthStatus : std::uint8_t {
    ok = 0,
    stalled,          // commanded significant voltage but isn't turning
    overTemperature,
    disconnected,
};

const char* toString(HealthStatus status);

// All reads below are cheap, side-effect-free queries against the
// underlying pros::MotorGroup and safe to call from multiple places or at
// different rates -- a future shared polling/caching layer can sit in front
// of this class without changing its interface.
class MotorGroup {
public:
    explicit MotorGroup(const config::MotorGroupConfig& config);

    // Raw encoder position, averaged across the group's motors, in the
    // units configured for this group (see config::MotorGroupConfig). NOT
    // converted to inches or any derived unit -- that conversion (wheel
    // diameter, gear ratio) belongs to a higher layer that doesn't exist
    // yet.
    [[nodiscard]] double getPositionRaw() const;

    // Actual (measured) velocity, averaged across the group's motors, in
    // RPM.
    [[nodiscard]] double getVelocityRpm() const;

    // Commands raw voltage to every motor in the group, clamped to the V5
    // motor's supported range (+-12000 mV).
    void writeVoltage(std::int32_t millivolts) const;

    // Combined stall / over-temperature / disconnection check across every
    // motor in the group. Deliberately coarse: `disconnected` is reported
    // the instant even ONE motor in the group drops out, collapsing "1 of 3
    // gone" and "3 of 3 gone" into the same status -- callers that need to
    // tell those apart (e.g. to keep driving on the motors that remain
    // rather than stopping outright) should use getConnectedMotorCount()
    // alongside this, not instead of it.
    [[nodiscard]] HealthStatus getHealth() const;

    // Total physical motors configured in this group, fixed at construction.
    [[nodiscard]] std::uint8_t getMotorCount() const;

    // How many of those motors are reporting a finite temperature (i.e. not
    // disconnected) right now. Compare against getMotorCount() to
    // distinguish a partial failure (some motors still responding, group
    // still drivable in a degraded state) from a total one.
    [[nodiscard]] std::uint8_t getConnectedMotorCount() const;

    [[nodiscard]] const char* name() const {
        return name_;
    }

private:
    const char* name_;
    pros::MotorGroup group_;
};

}  // namespace lightspeed::hal
