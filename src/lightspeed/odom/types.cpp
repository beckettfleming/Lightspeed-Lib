#include "lightspeed/odom/types.hpp"

namespace lightspeed::odom {

const char* toString(ConfidenceTier tier) {
    switch (tier) {
        case ConfidenceTier::fullPod:
            return "FULL_POD";
        case ConfidenceTier::partial:
            return "PARTIAL";
        case ConfidenceTier::imeOnly:
            return "IME_ONLY";
    }
    return "UNKNOWN";
}

}  // namespace lightspeed::odom
