#include "lightspeed/vision/vision_types.hpp"

namespace lightspeed::vision {

const char* toString(GateReason reason) {
    switch (reason) {
        case GateReason::accepted:
            return "ACCEPTED";
        case GateReason::noTagDetected:
            return "NO_TAG_DETECTED";
        case GateReason::unknownTagId:
            return "UNKNOWN_TAG_ID";
        case GateReason::notStableYet:
            return "NOT_STABLE_YET";
        case GateReason::skewTooHigh:
            return "SKEW_TOO_HIGH";
    }
    return "UNKNOWN";
}

}  // namespace lightspeed::vision
