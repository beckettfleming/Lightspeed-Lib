/**
 * \file lightspeed/driver/button_macro.hpp
 *
 * Non-blocking button-macro system: binds a controller button to a short
 * scripted sequence of subsystem actions, advanced one step per opcontrol
 * tick rather than blocking it. auton::sequencer is deliberately NOT reused
 * here -- it blocks the calling task, which would stall opcontrol's whole
 * loop (including the drivetrain's own setTargetVelocity calls) for the
 * macro's duration.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/driver-control/
 */

#pragma once

#include <cstddef>
#include <functional>
#include <vector>

namespace lightspeed::driver {

struct MacroStep {
    // Called once, the instant this step becomes active.
    std::function<void()> action;

    // Polled every update() while this step is active. The default (always
    // true) advances immediately after the action runs -- useful for a final
    // "fire this and finish" step.
    std::function<bool()> isDone = [] { return true; };
};

struct ButtonMacro {
    const char* name;
    std::vector<MacroStep> steps;
};

// Only one macro runs at a time; triggering a new one mid-sequence replaces
// it rather than queuing or interleaving.
class ButtonMacroRunner {
public:
    // Starts from step 0, replacing any macro already in progress.
    // Re-triggering the SAME running macro restarts it rather than resuming
    // -- a second press generally means "run it again", not "continue".
    void trigger(const ButtonMacro& macro);

    // Advances the active macro by at most one step. Call once per opcontrol
    // tick regardless of whether one is active; a no-op when idle.
    void update();

    // True between trigger() and the last step completing -- use it to
    // suppress a conflicting direct button binding.
    [[nodiscard]] bool isRunning() const;

private:
    const ButtonMacro* activeMacro_ = nullptr;
    std::size_t currentStepIndex_ = 0;
    bool currentStepActionRun_ = false;
};

}  // namespace lightspeed::driver
