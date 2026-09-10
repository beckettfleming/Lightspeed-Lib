/**
 * \file lightspeed/subsystem/schedulable_subsystem.hpp
 *
 * Non-template interface the Scheduler stores and calls. Concrete subsystems
 * inherit from Subsystem<StateEnum>, not this. The separation exists so
 * subsystems with different state enum types can share one collection.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/subsystem/
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
