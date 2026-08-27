/**
 * \file lightspeed/auton/demo_routines.hpp
 *
 * ============================================================================
 * PLACEHOLDER ROUTINES -- NOT TIED TO ANY REAL SEASON STRATEGY.
 *
 * Tachyon's actual autonomous strategy isn't finalized (the field/game
 * aren't known yet). These exist purely to validate the registry/GUI/
 * sequencer framework end-to-end -- drive-straight-then-turn, and a short
 * pure-pursuit path, each using only the exact Steps 2/4/6 functions
 * driver control and bench-testing already use. Delete or replace once
 * real routines are designed for the actual season.
 * ============================================================================
 */

#pragma once

#include "lightspeed/auton/routine.hpp"

namespace lightspeed::auton::demo {

[[nodiscard]] Routine makeDemoStraightAndTurnRoutine();
[[nodiscard]] Routine makeDemoPursuitPathRoutine();

}  // namespace lightspeed::auton::demo
