/**
 * \file lightspeed/subsystem/scheduler.hpp
 *
 * Central scheduler: one PROS task iterates every registered subsystem and
 * calls update(), rather than each subsystem running its own task.
 * Subsystems register themselves automatically at construction (see
 * Subsystem<StateEnum>) -- call start() once, after all subsystems that
 * should run this session are constructed.
 */

#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "lightspeed/subsystem/schedulable_subsystem.hpp"
#include "pros/rtos.hpp"

namespace lightspeed::subsystem {

class Scheduler {
public:
    static Scheduler& instance();

    Scheduler(const Scheduler&) = delete;
    Scheduler& operator=(const Scheduler&) = delete;

    // Registers a subsystem to be updated every cycle. Safe to call from
    // any task, including after start() (the new subsystem is simply
    // picked up starting the next cycle).
    void registerSubsystem(SchedulableSubsystem& subsystem);

    // Starts the scheduler's background task. No-op if already started.
    void start();

private:
    Scheduler() = default;
    void schedulerLoop();

    static constexpr std::uint8_t kMaxSubsystems = 8;
    static constexpr std::uint32_t kLoopPeriodMs = 20;  // ~50Hz

    std::array<SchedulableSubsystem*, kMaxSubsystems> subsystems_{};
    std::uint8_t subsystemCount_ = 0;
    pros::Mutex registrationMutex_;

    std::optional<pros::Task> task_;
};

}  // namespace lightspeed::subsystem
