/**
 * \file lightspeed/subsystem/subsystem.hpp
 *
 * Generic subsystem base class: a HAL motor group plus a PIDF controller for
 * position control, an explicit state machine, a named preset table, and
 * automatic scheduler registration.
 *
 * Header-only because each concrete subsystem supplies its own StateEnum --
 * there's no single set of template arguments to explicitly instantiate.
 *
 * Thread-safety: update() runs on the scheduler task while command methods
 * may be called from another task. transitionTo() and getState() are guarded
 * by an internal recursive mutex; concrete subsystems must guard their own
 * cross-task state the same way via lock()/unlock().
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/subsystem/
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
    double positionRaw;  // raw HAL units, e.g. motor degrees
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

    // Safe from any task.
    [[nodiscard]] StateEnum getState() const {
        mutex_.take();
        const StateEnum result = state_;
        mutex_.give();
        return result;
    }

    // Final: concrete subsystems implement per-cycle logic in onUpdate(), so
    // this can guard the whole cycle against a concurrent external command
    // landing mid-cycle.
    void update(double dtSeconds) final {
        mutex_.take();
        onUpdate(dtSeconds);
        // Every subsystem's state ordinal reaches the bus with no
        // per-subsystem wiring; flags do the same via flag_registry.cpp.
        telemetry::TelemetryBus::instance().record(name_, static_cast<std::int32_t>(state_));
        mutex_.give();
    }

protected:
    // Once per scheduler cycle, with the internal lock already held.
    virtual void onUpdate(double dtSeconds) = 0;

    // onExit(current) -> state change -> pidf_.reset() -> onEnter(new). A
    // same-state transition is a no-op. Recursive-mutex-guarded, so it's
    // safe from any task including from within onUpdate() via onFault().
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

    virtual void onEnter(StateEnum /*state*/) {}
    virtual void onExit(StateEnum /*state*/) {}

    // Override to react to a non-ok motor health status, e.g. transition to
    // a fault state.
    virtual void onFault(hal::HealthStatus /*status*/) {}

    // PIDF position control toward targetPositionRaw. `stalled` and
    // `overTemperature` zero voltage UNCONDITIONALLY -- regardless of
    // whether the concrete onFault() override reacts to that status -- so an
    // unhandled status can't leave stale voltage commanded indefinitely.
    // `disconnected` only stops the mechanism if EVERY motor is gone; a
    // partial disconnect keeps driving in a degraded state. Call only from
    // onUpdate(), where the lock is already held.
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

    // nullopt if presetName isn't in this subsystem's preset table.
    [[nodiscard]] std::optional<double> getPresetPosition(const char* presetName) const {
        for (const Preset& preset : presets_) {
            if (std::strcmp(preset.name, presetName) == 0) {
                return preset.positionRaw;
            }
        }
        return std::nullopt;
    }

    bool registerFlag(const char* flagName, bool initialValue = false) const {
        return FlagRegistry::instance().registerFlag(flagName, initialValue);
    }
    bool setFlag(const char* flagName, bool value) const {
        return FlagRegistry::instance().setFlag(flagName, value);
    }

    // Guards a concrete subsystem's own state against races between
    // onUpdate() (scheduler task) and its externally-callable commands.
    // Recursive, so nesting transitionTo()/getState() inside is safe.
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
