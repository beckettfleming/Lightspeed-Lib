/**
 * \file lightspeed/driver/demo_macros.hpp
 *
 * PLACEHOLDER -- demonstrates the button-macro system against the demo
 * subsystem, since no real mechanism exists yet. Delete or replace once a
 * real subsystem needs a real macro.
 */

#pragma once

#include "lightspeed/driver/button_macro.hpp"
#include "lightspeed/subsystem/demo/example_arm.hpp"

namespace lightspeed::driver::demo {

// Raises the demo arm to HIGH, waits (non-blocking) for it to settle into
// HOLDING, then lowers it back to LOW -- a two-step scripted sequence
// bound to one button press instead of two direct presses.
[[nodiscard]] ButtonMacro makeDemoArmCycleMacro(subsystem::demo::ExampleArm& exampleArm);

}  // namespace lightspeed::driver::demo
