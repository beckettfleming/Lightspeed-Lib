/**
 * \file lightspeed/auton/field_renderer.hpp
 *
 * Schematic top-down field overview (simple shapes, no bitmap asset).
 * Reusable at full size (screen 1) and at a smaller scale (screen 2's
 * preview pane) -- callers just pass a smaller region and a correspondingly
 * scaled-down FieldToScreenConfig.pixelsPerInch.
 */

#pragma once

#include <vector>

#include "lightspeed/auton/field_coordinates.hpp"
#include "lightspeed/motion/path.hpp"

namespace lightspeed::auton {

// Draws a field border + center line filling `region`, using `config` to
// map field inches to screen pixels. Does not erase the screen first --
// callers control when to erase.
void renderField(const FieldToScreenConfig& config, const FieldDimensions& field, const ScreenRegion& region);

// Draws a route overlay: dots at each waypoint connected by lines, e.g. a
// routine's preview path over a field rendered with the same config/region.
void renderRoute(const FieldToScreenConfig& config, const ScreenRegion& region, const std::vector<motion::Waypoint>& points);

}  // namespace lightspeed::auton
