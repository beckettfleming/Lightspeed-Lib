/**
 * \file lightspeed/auton/routine.hpp
 *
 * Routine registry: each routine declares its name, which start location(s)
 * it's valid from, a preview point list for screen 2's route overlay, and
 * its run function (see autonomous_context.hpp).
 */

#pragma once

#include <cstddef>
#include <vector>

#include "lightspeed/auton/autonomous_context.hpp"
#include "lightspeed/motion/path.hpp"

namespace lightspeed::auton {

struct Routine {
    const char* name;
    std::vector<std::size_t> validStartLocationIndices;  // indices into kStartLocations; empty = valid from any start
    std::vector<motion::Waypoint> previewPoints;
    RoutineFunction run;
};

class RoutineRegistry {
public:
    void registerRoutine(const Routine& routine);

    [[nodiscard]] const std::vector<Routine>& getAll() const;

    // Indices into getAll() valid for startLocationIndex (routines with an
    // empty validStartLocationIndices are included for every start).
    [[nodiscard]] std::vector<std::size_t> getIndicesForStartLocation(std::size_t startLocationIndex) const;

private:
    std::vector<Routine> routines_;
};

}  // namespace lightspeed::auton
