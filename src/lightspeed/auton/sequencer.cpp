#include "lightspeed/auton/sequencer.hpp"

#include "pros/rtos.hpp"

namespace lightspeed::auton {

void waitUntilSettled(const std::function<bool()>& isSettled, double timeoutSeconds, std::uint32_t loopPeriodMs) {
    const std::uint32_t startTime = pros::millis();
    std::uint32_t previousTime = startTime;

    while (!isSettled()) {
        if ((pros::millis() - startTime) / 1000.0 >= timeoutSeconds) {
            return;
        }
        pros::Task::delay_until(&previousTime, loopPeriodMs);
    }
}

}  // namespace lightspeed::auton
