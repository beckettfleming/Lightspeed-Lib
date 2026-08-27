/**
 * \file lightspeed/auton/sequencer.hpp
 *
 * Lightweight procedural helpers for coordinating a Step 4 subsystem action
 * with Step 6 drivetrain motion inside a routine. Not a scheduler --
 * subsystem command methods (e.g. a subsystem's moveToPreset()) are
 * already non-blocking by construction (Step 4's scheduler runs their
 * update() asynchronously), so these two functions exist to make that
 * pattern explicit and readable in routine code, and to give routines a
 * generic join point that doesn't need to know a specific subsystem's
 * state enum.
 */

#pragma once

#include <cstdint>
#include <functional>

namespace lightspeed::auton {

// Documents intent: `action` (typically a subsystem command call) is
// expected to return immediately, running asynchronously via the Step 4
// scheduler. Pairs with waitUntilSettled() as the explicit join point.
template <typename Callable>
void fireAndForget(Callable&& action) {
    action();
}

// Blocks the calling task, polling `isSettled` at loopPeriodMs, until it
// returns true or timeoutSeconds elapses. Generic across subsystems --
// e.g. waitUntilSettled([&] { return exampleArm.getState() ==
// ExampleArmState::holding; }, 2.0).
void waitUntilSettled(const std::function<bool()>& isSettled, double timeoutSeconds, std::uint32_t loopPeriodMs = 20);

}  // namespace lightspeed::auton
