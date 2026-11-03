/**
 * \file lightspeed/subsystem/lift.hpp
 *
 * Lift height model backed by the paired winch motor encoders.
 */

#pragma once

#include "lightspeed/hal/motor_group.hpp"

namespace lightspeed::subsystem {

class Lift {
public:
    Lift(hal::MotorGroup& leftWinch, hal::MotorGroup& rightWinch, double drumRadiusInches);

    // Starts a non-blocking relative height move. Positive inches raise the lift.
    [[nodiscard]] bool moveByInches(double deltaInches);

    [[nodiscard]] double getHeightInches() const;
    [[nodiscard]] bool isAtTarget() const;
    void stop();

private:
    hal::MotorGroup& leftWinch_;
    hal::MotorGroup& rightWinch_;
    double drumRadiusInches_;
    double leftStartingPositionDegrees_;
    double rightStartingPositionDegrees_;
    double leftTargetPositionDegrees_ = 0.0;
    double rightTargetPositionDegrees_ = 0.0;
    bool hasTarget_ = false;
};

}  // namespace lightspeed::subsystem
