/**
 * \file lightspeed/subsystem/subsystem.hpp
 *
 * Generic subsystem base class: owns a HAL motor group and a Step 2 PIDF
 * controller for position control, an explicit state machine (state enum
 * supplied by the concrete subsystem), a named preset-position table, and
 * automatic scheduler registration.
 *
 * Header-only: because each concrete subsystem supplies its own StateEnum
 * type, this can't be split into a .cpp the way the rest of the project is
 * -- there's no single set of template arguments to explicitly instantiate.
 * SchedulableSubsystem is the non-template interface the Scheduler actually
 * stores and calls.
 *
 * Thread-safety: a subsystem's update() runs on the scheduler task while
 * command methods (e.g. a concrete subsystem's moveToPreset()) may be
 * called from another task (bench harness now, driver control later).
 * transitionTo() and getState() are guarded by an internal recursive
 * mutex; concrete subsystems should guard their own cross-task-touched
 * state (e.g. a cached target position) the same way -- see lock()/unlock()
 * and lightspeed::subsystem::demo::ExampleArm for the pattern.
 */

#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <optional>
#include <vector>

#include "lightspeed/control/pidf_controller.hpp"
#include "lightspeed/hal/motor_group.hpp"
#include "lightspeed/subsystem/flag_registry.hpp"
#include "lightspeed/subsystem/schedulable_subsystem.hpp"
#include "lightspeed/subsystem/scheduler.hpp"
#include "lightspeed/telemetry/telemetry_bus.hpp"
#include "pros/rtos.hpp"

namespace lightspeed::subsystem {

struct Preset {
    const char* name;
    double positionRaw;  // raw HAL units (e.g. motor degrees) -- unit-agnostic, per HAL convention
};

struct SubsystemConfig {
    const char* name;
    control::PIDFConfig pidf;
    std::vector<Preset> presets;
};

template <typename StateEnum>
class Subsystem : public SchedulableSubsystem {
public:
    Subsystem(hal::MotorGroup& motors, const SubsystemConfig& config, StateEnum initialState)
        : motors_(motors), pidf_(config.pidf), name_(config.name), presets_(config.presets), state_(initialState) {
        Scheduler::instance().registerSubsystem(*this);
    }

    [[nodiscard]] const char* getName() const override {
        return name_;
    }

    // Safe to call from any task.
    [[nodiscard]] StateEnum getState() const {
        mutex_.take();
        const StateEnum result = state_;
        mutex_.give();
        return result;
    }

    // Final: concrete subsystems implement their per-cycle logic in
    // onUpdate(), not update() directly, so this can guard the whole cycle
    // against a concurrent external command (e.g. moveToPreset()) landing
    // mid-cycle.
    void update(double dtSeconds) final {
        mutex_.take();
        onUpdate(dtSeconds);
        // Generic across every concrete subsystem: the raw state ordinal,
        // published under the subsystem's own name (already a stable
        // literal, see SubsystemConfig::name) -- no per-subsystem telemetry
        // wiring needed. Flags get the same treatment via FlagRegistry
        // forwarding into the bus (see flag_registry.cpp).
        telemetry::TelemetryBus::instance().record(name_, static_cast<std::int32_t>(state_));
        mutex_.give();
    }

protected:
    // Called once per scheduler cycle with the internal lock already held.
    virtual void onUpdate(double dtSeconds) = 0;

    // Transitions to newState: onExit(current), then onEnter(newState),
    // resetting the PIDF controller in between (fresh
    // integral/derivative/settle history for whatever the new state drives
    // toward). A same-state "transition" is a no-op. Safe to call from any
    // task -- recursive-mutex-guarded, so it's also safe called from
    // within onUpdate() on the scheduler task (e.g. via onFault()).
    void transitionTo(StateEnum newState) {
        mutex_.take();
        if (newState != state_) {
            onExit(state_);
            state_ = newState;
            pidf_.reset();
            onEnter(state_);
        }
        mutex_.give();
    }

    // Override to react to entering/exiting a state -- e.g. latch the
    // current position as the PIDF target on entering a "holding" state.
    virtual void onEnter(StateEnum /*state*/) {}
    virtual void onExit(StateEnum /*state*/) {}

    // Override to react to the motor group reporting a non-ok health
    // status (stalled / over-temperature / disconnected) -- e.g. transition
    // to a fault state so a jammed mechanism stops being commanded voltage.
    virtual void onFault(hal::HealthStatus /*status*/) {}

    // Drives toward targetPositionRaw via the PIDF controller in position
    // mode. `stalled`/`overTemperature` always zero voltage and call
    // onFault() -- unconditionally, regardless of whether the concrete
    // subsystem's onFault() override reacts to that specific status. This
    // used to rely entirely on onFault() to react (e.g. ExampleArm's only
    // handles `stalled`) -- for any status an override doesn't explicitly
    // handle, the base class now still fails safe immediately rather than
    // leaving the last PIDF-computed voltage commanded indefinitely while
    // repeatedly re-detecting the same fault. `disconnected` only stops the
    // mechanism if EVERY motor in the group is gone (see
    // hal::MotorGroup::getConnectedMotorCount()) -- a partial disconnect on
    // a multi-motor mechanism keeps driving in a degraded state rather than
    // stopping one that's still partially drivable. Call only from
    // onUpdate() (the lock is already held there).
    void driveToPosition(double targetPositionRaw, double dtSeconds) {
        const hal::HealthStatus health = motors_.getHealth();
        const bool mustStop = health == hal::HealthStatus::stalled || health == hal::HealthStatus::overTemperature ||
                               (health == hal::HealthStatus::disconnected && motors_.getConnectedMotorCount() == 0);
        if (mustStop) {
            motors_.writeVoltage(0);
            onFault(health);
            return;
        }

        const double currentPosition = motors_.getPositionRaw();
        const control::Setpoint setpoint{.target = targetPositionRaw, .targetVelocity = 0.0, .targetAcceleration = 0.0};
        const double outputMv = pidf_.calculate(currentPosition, setpoint, dtSeconds);
        motors_.writeVoltage(static_cast<std::int32_t>(std::clamp(outputMv, -12000.0, 12000.0)));
    }

    // Looks up a named preset's raw position. nullopt if presetName isn't
    // in this subsystem's preset table.
    [[nodiscard]] std::optional<double> getPresetPosition(const char* presetName) const {
        for (const Preset& preset : presets_) {
            if (std::strcmp(preset.name, presetName) == 0) {
                return preset.positionRaw;
            }
        }
        return std::nullopt;
    }

    // Thin convenience forwards to the flag registry singleton, so
    // subclasses don't need to spell out FlagRegistry::instance() everywhere.
    bool registerFlag(const char* flagName, bool initialValue = false) const {
        return FlagRegistry::instance().registerFlag(flagName, initialValue);
    }
    bool setFlag(const char* flagName, bool value) const {
        return FlagRegistry::instance().setFlag(flagName, value);
    }

    // Exposes the subsystem's internal recursive lock so a concrete
    // subsystem can guard its own state (e.g. a cached target position)
    // against races between onUpdate() (scheduler task) and its own
    // externally-callable command methods. Recursive, so nesting a call
    // that itself calls transitionTo()/getState() on the same task is safe.
    void lock() const {
        mutex_.take();
    }
    void unlock() const {
        mutex_.give();
    }

    hal::MotorGroup& motors_;
    control::PIDFController pidf_;

private:
    const char* name_;
    std::vector<Preset> presets_;
    StateEnum state_;
    mutable pros::RecursiveMutex mutex_;
};

}  // namespace lightspeed::subsystem
