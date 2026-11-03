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

// pixelsPerInch chosen so a 144in field fits the 200x200px screen-1 region.
inline constexpr FieldToScreenConfig kFieldToScreenConfig{
    .originXInches = 0.0,
    .originYInches = 0.0,
    .pixelsPerInch = 200.0 / 144.0,
};

}  // namespace lightspeed::auton
