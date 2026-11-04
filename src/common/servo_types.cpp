#include "t170/common/servo_types.h"
#include <chrono>

namespace wrist_servo {
double monotonic_seconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

const char *phase_name(Phase p) {
    switch (p) {
        case Phase::Acquiring: return "acquiring"; case Phase::Aligning: return "aligning";
        case Phase::Descending: return "descending"; case Phase::Hold: return "hold";
        case Phase::Commit: return "commit"; case Phase::Settling: return "settling"; case Phase::Done: return "done";
        case Phase::Failed: return "failed";
    }
    return "invalid";
}
}
