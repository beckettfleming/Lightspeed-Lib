/**
 * \file lightspeed/auton/sequencer.hpp
 *
 * Helpers for coordinating a subsystem action with drivetrain motion inside
 * a routine. Not a scheduler -- subsystem commands are already non-blocking
 * by construction; these exist to make that explicit in routine code and to
 * give routines a join point that doesn't need a specific state enum.
 *
 * waitUntilSettled() BLOCKS, so it is for the autonomous task only. Use
 * driver::ButtonMacroRunner during opcontrol().
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/autonomous/
 */

#pragma once

#include <cstdint>
#include <functional>

namespace lightspeed::auton {

// Documents intent: `action` is expected to return immediately, running
// asynchronously via the subsystem scheduler. Pairs with waitUntilSettled()
// as the explicit join point.
template <typename Callable>
void fireAndForget(Callable&& action) {
    action();
}

// Blocks the calling task, polling `isSettled` at loopPeriodMs, until it
// returns true or timeoutSeconds elapses. Returns silently on timeout, so a
// jammed mechanism can't hang the routine.
void waitUntilSettled(const std::function<bool()>& isSettled, double timeoutSeconds, std::uint32_t loopPeriodMs = 20);

}  // namespace lightspeed::auton
