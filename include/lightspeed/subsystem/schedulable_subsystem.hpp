/**
 * \file lightspeed/subsystem/schedulable_subsystem.hpp
 *
 * Non-template interface the Scheduler stores and calls. Concrete
 * subsystems don't implement this directly -- they inherit from the
 * templated Subsystem<StateEnum>, which implements this and adds the
 * state-machine machinery. This separation exists so subsystems with
 * different state enum types can still be stored in one polymorphic
 * collection by the scheduler.
 */

#pragma once

namespace lightspeed::subsystem {

class SchedulableSubsystem {
public:
    virtual ~SchedulableSubsystem() = default;

    // Called once per scheduler cycle (~50-100Hz). Must be fast and
    // non-blocking -- no busy-waits.
    virtual void update(double dtSeconds) = 0;

    [[nodiscard]] virtual const char* getName() const = 0;
};

}  // namespace lightspeed::subsystem
