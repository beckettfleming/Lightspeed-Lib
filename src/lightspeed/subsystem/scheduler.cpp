#include "lightspeed/subsystem/scheduler.hpp"

#include <cstdio>

namespace lightspeed::subsystem {

Scheduler& Scheduler::instance() {
    static Scheduler scheduler;
    return scheduler;
}

void Scheduler::registerSubsystem(SchedulableSubsystem& subsystem) {
    registrationMutex_.take();

    if (subsystemCount_ >= kMaxSubsystems) {
        registrationMutex_.give();
        std::printf("[lightspeed::subsystem] WARNING: Scheduler is full (%u max) -- '%s' was not registered.\n",
                     kMaxSubsystems, subsystem.getName());
        return;
    }

    // Write the pointer before publishing the new count, so a scheduler
    // cycle that observes the incremented count also sees a valid pointer.
    subsystems_[subsystemCount_] = &subsystem;
    ++subsystemCount_;

    registrationMutex_.give();
}

void Scheduler::start() {
    if (task_.has_value()) {
        return;
    }
    task_.emplace([this] { schedulerLoop(); }, "lightspeed_subsystem_scheduler");
}

void Scheduler::schedulerLoop() {
    std::uint32_t previousTime = pros::millis();
    constexpr double dtSeconds = kLoopPeriodMs / 1000.0;

    while (true) {
        pros::Task::delay_until(&previousTime, kLoopPeriodMs);

        // Snapshot the count under the lock, then iterate without holding
        // it -- registerSubsystem() never removes or reorders entries, so
        // reading subsystems_[0..count) after releasing the lock is safe,
        // and a registration that races this cycle just gets included
        // starting next cycle instead.
        registrationMutex_.take();
        const std::uint8_t count = subsystemCount_;
        registrationMutex_.give();

        for (std::uint8_t i = 0; i < count; ++i) {
            subsystems_[i]->update(dtSeconds);
        }
    }
}

}  // namespace lightspeed::subsystem
