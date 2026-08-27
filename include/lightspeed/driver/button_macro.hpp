/**
 * \file lightspeed/driver/button_macro.hpp
 *
 * Non-blocking button-macro system for driver control: binds a controller
 * button to a short scripted sequence of subsystem actions, advanced one
 * step per opcontrol tick rather than blocking it. lightspeed::auton's
 * sequencer.hpp (fireAndForget/waitUntilSettled) is deliberately NOT reused
 * here -- it blocks the calling task, which is fine for the autonomous
 * task but would stall opcontrol's entire ~50Hz loop (including the
 * drivetrain's own setTargetVelocity calls) for the macro's whole duration.
 * This was flagged "not yet scoped" in Step 5 -- this is that scope: a
 * fixed list of steps per macro, each either a fire-and-forget action or a
 * non-blocking "wait until predicate" gate, advanced by calling update()
 * once per opcontrol tick.
 */

#pragma once

#include <cstddef>
#include <functional>
#include <vector>

namespace lightspeed::driver {

struct MacroStep {
    // Called once, the instant this step becomes active.
    std::function<void()> action;

    // Polled every update() call while this step is active; the macro
    // advances to the next step once this returns true. A step with no
    // wait condition (the default, always true) advances immediately after
    // its action runs -- useful for a final step that's just "fire this
    // and finish" without gating on anything.
    std::function<bool()> isDone = [] { return true; };
};

// A single controller button's bound sequence of steps.
struct ButtonMacro {
    const char* name;
    std::vector<MacroStep> steps;
};

// Runs zero or more registered macros, advancing whichever one is active
// (if any) by at most one step per update() call -- never blocks. Only one
// macro runs at a time; triggering a new one while another is mid-sequence
// replaces it rather than queuing or interleaving.
class ButtonMacroRunner {
public:
    // Starts running `macro` from its first step, replacing whatever macro
    // (if any) was already in progress. Safe to call every tick on a
    // "new press" edge -- re-triggering the SAME macro that's already
    // running restarts it from step 0 rather than resuming, since a second
    // press generally means "run it again," not "continue."
    void trigger(const ButtonMacro& macro);

    // Advances the active macro (if any) by at most one step: runs the
    // current step's action if it just became active, then checks isDone()
    // and moves to the next step if it's satisfied. Call once per
    // opcontrol tick regardless of whether a macro is active -- a no-op
    // when nothing's running.
    void update();

    // True while a macro is actively running (i.e. between trigger() and
    // its last step completing) -- e.g. to suppress a conflicting direct
    // button binding while a macro owns a mechanism.
    [[nodiscard]] bool isRunning() const;

private:
    const ButtonMacro* activeMacro_ = nullptr;
    std::size_t currentStepIndex_ = 0;
    bool currentStepActionRun_ = false;
};

}  // namespace lightspeed::driver
