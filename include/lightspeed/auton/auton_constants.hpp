/**
 * \file lightspeed/auton/auton_constants.hpp
 *
 * Single source of truth for the field-to-screen mapping used by both
 * selector screens (see field_coordinates.hpp).
 *
 * TODO: kFieldDimensions assumes a standard 12ft x 12ft VRC field --
 * confirm against the actual season field.
 */

#pragma once

#include "lightspeed/auton/field_coordinates.hpp"

namespace lightspeed::auton {

inline constexpr FieldDimensions kFieldDimensions{.widthInches = 144.0, .lengthInches = 144.0};

// origin at the field's own (0,0) corner; pixelsPerInch chosen so the full
// 144in field fits within kScreen1FieldRegion (230x230px, see
// screen_layout.hpp) with a little margin: 230 / 144 ~= 1.6.
inline constexpr FieldToScreenConfig kFieldToScreenConfig{
    .originXInches = 0.0,
    .originYInches = 0.0,
    .pixelsPerInch = 1.6,
};

}  // namespace lightspeed::auton
