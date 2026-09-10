/**
 * \file lightspeed/auton/demo_routines.hpp
 *
 * ============================================================================
 * PLACEHOLDER ROUTINES -- NOT TIED TO ANY REAL SEASON STRATEGY.
 *
 * These validate the registry/GUI/sequencer framework end-to-end. Delete or
 * replace once real routines are designed for the actual season.
 * ============================================================================
 */

#pragma once

#include "lightspeed/auton/routine.hpp"

namespace lightspeed::auton::demo {

[[nodiscard]] Routine makeDemoStraightAndTurnRoutine();
[[nodiscard]] Routine makeDemoPursuitPathRoutine();

}  // namespace lightspeed::auton::demo
