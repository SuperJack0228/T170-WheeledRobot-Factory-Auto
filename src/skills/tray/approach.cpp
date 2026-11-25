#include "../internal/execution.h"
#include "t170/skills/legacy/runtime.h"
#include "../internal/legacy_helpers.h"
#include <iostream>

namespace move_box {
ArmLineMoveResult run_head_approach_selective(
    Arm &, Arm &, HeadAssignState &st, bool move_r, bool move_l)
{
    t170::skills::detail::report(t170::skills::Stage::Approach, move_r, move_l);
    if (t170::skills::detail::active_execution && t170::skills::detail::active_execution->cancelled())
        return {move_r ? -4 : 0, move_l ? -4 : 0};
    drop_if_y_split_blocks(st, move_r, move_l, "hover下发前");
    // The descent planner owns the B-point approach and continuous RGB servo.
    // No separate stop, depth capture or wrist-to-base target replacement.
    return {};
}
} // namespace move_box
