#include "lightspeed/control/pidf_controller.hpp"

#include <algorithm>
#include <cmath>

namespace lightspeed::control {

namespace {

double sign(double value) {
    if (value > 0.0) return 1.0;
    if (value < 0.0) return -1.0;
    return 0.0;
}

}  // namespace

PIDFController::PIDFController(const PIDFConfig& config) : config_(config) {}

double PIDFController::calculate(double measurement, const Setpoint& setpoint, double dtSeconds) {
    const double error = setpoint.target - measurement;

    if (config_.integralZone <= 0.0 || std::abs(error) <= config_.integralZone) {
        integral_ += error * dtSeconds;
        if (config_.integralMax > 0.0) {
            integral_ = std::clamp(integral_, -config_.integralMax, config_.integralMax);
        }
    }

    const double derivative = (hasPreviousError_ && dtSeconds > 0.0) ? (error - previousError_) / dtSeconds : 0.0;
    previousError_ = error;
    hasPreviousError_ = true;

    const double feedback = config_.kP * error + config_.kI * integral_ + config_.kD * derivative;
    const double feedforward =
        config_.kV * setpoint.targetVelocity + config_.kA * setpoint.targetAcceleration + config_.kS * sign(setpoint.targetVelocity);

    if (std::abs(error) <= config_.settleTolerance) {
        ++settledCycles_;
    } else {
        settledCycles_ = 0;
    }

    return feedback + feedforward;
}

void PIDFController::reset() {
    integral_ = 0.0;
    previousError_ = 0.0;
    hasPreviousError_ = false;
    settledCycles_ = 0;
}

bool PIDFController::isSettled() const {
    return settledCycles_ >= config_.settleCycles;
}

void PIDFController::setConfig(const PIDFConfig& config) {
    config_ = config;
}

const PIDFConfig& PIDFController::getConfig() const {
    return config_;
}

}  // namespace lightspeed::control
