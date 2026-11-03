#include "lightspeed/subsystem/lift.hpp"

#include <cmath>
#include <cstdio>

namespace lightspeed::subsystem {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTargetToleranceDegrees = 3.0;
constexpr std::int32_t kMoveVelocityRpm = 100;

}  // namespace

Lift::Lift(hal::MotorGroup& leftWinch, hal::MotorGroup& rightWinch, double drumRadiusInches)
    : leftWinch_(leftWinch),
      rightWinch_(rightWinch),
      drumRadiusInches_(drumRadiusInches),
      leftStartingPositionDegrees_(leftWinch.getPositionRaw()),
      rightStartingPositionDegrees_(rightWinch.getPositionRaw()) {}

bool Lift::moveByInches(double deltaInches) {
    if (!std::isfinite(deltaInches) || !std::isfinite(drumRadiusInches_) || drumRadiusInches_ <= 0.0) {
        std::printf("[lift] invalid height move: %.2fin, drum radius %.2fin\n", deltaInches, drumRadiusInches_);
        return false;
    }

    const double targetHeightInches = getHeightInches() + deltaInches;
    const double targetDrumDegrees = targetHeightInches / drumRadiusInches_ * 180.0 / kPi;

    // The winches are mirrored: raising winds the left down and right up.
    leftTargetPositionDegrees_ = leftStartingPositionDegrees_ - targetDrumDegrees;
    rightTargetPositionDegrees_ = rightStartingPositionDegrees_ + targetDrumDegrees;
    hasTarget_ = false;

    const std::int32_t leftResult = leftWinch_.moveAbsolute(leftTargetPositionDegrees_, kMoveVelocityRpm);
    const std::int32_t rightResult = rightWinch_.moveAbsolute(rightTargetPositionDegrees_, kMoveVelocityRpm);
    if (leftResult < 0 || rightResult < 0) {
        leftWinch_.writeVoltage(0);
        rightWinch_.writeVoltage(0);
        std::printf("[lift] failed to command winches: left=%ld right=%ld\n", static_cast<long>(leftResult),
                    static_cast<long>(rightResult));
        return false;
    }

    hasTarget_ = true;
    std::printf("[lift] target height=%.2fin winch targets=(%.1f, %.1f)deg\n", targetHeightInches,
                leftTargetPositionDegrees_, rightTargetPositionDegrees_);
    return true;
}

double Lift::getHeightInches() const {
    const double leftWindingDegrees = leftStartingPositionDegrees_ - leftWinch_.getPositionRaw();
    const double rightWindingDegrees = rightWinch_.getPositionRaw() - rightStartingPositionDegrees_;
    const double averageWindingDegrees = (leftWindingDegrees + rightWindingDegrees) / 2.0;
    return averageWindingDegrees * kPi / 180.0 * drumRadiusInches_;
}

bool Lift::isAtTarget() const {
    return hasTarget_ &&
           std::abs(leftWinch_.getPositionRaw() - leftTargetPositionDegrees_) <= kTargetToleranceDegrees &&
           std::abs(rightWinch_.getPositionRaw() - rightTargetPositionDegrees_) <= kTargetToleranceDegrees;
}

void Lift::stop() {
    leftWinch_.writeVoltage(0);
    rightWinch_.writeVoltage(0);
    hasTarget_ = false;
}

}  // namespace lightspeed::subsystem
