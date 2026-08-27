#include "lightspeed/auton/routine.hpp"

#include <algorithm>

namespace lightspeed::auton {

void RoutineRegistry::registerRoutine(const Routine& routine) {
    routines_.push_back(routine);
}

const std::vector<Routine>& RoutineRegistry::getAll() const {
    return routines_;
}

std::vector<std::size_t> RoutineRegistry::getIndicesForStartLocation(std::size_t startLocationIndex) const {
    std::vector<std::size_t> indices;
    for (std::size_t i = 0; i < routines_.size(); ++i) {
        const std::vector<std::size_t>& valid = routines_[i].validStartLocationIndices;
        const bool matches = valid.empty() || std::find(valid.begin(), valid.end(), startLocationIndex) != valid.end();
        if (matches) {
            indices.push_back(i);
        }
    }
    return indices;
}

}  // namespace lightspeed::auton
