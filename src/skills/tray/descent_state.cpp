#include "../internal/execution.h"
#include "t170/skills/legacy/runtime.h"
#include "../internal/legacy_helpers.h"
#include <cmath>
#include <iostream>

namespace move_box {
HandMoveState prepare_tray_descent(const HeadAssignState &st, bool enable_r, bool enable_l)
{
    t170::skills::detail::report(t170::skills::Stage::Observe, enable_r, enable_l);
    HandMoveState hs;
    hs.move_r = enable_r;
    hs.move_l = enable_l;
    // Preserve the existing head-height/reference-height choice. Wrist RGB
    // observation and XY corrections begin inside the continuous descent.
    hs.have_object_z_r = enable_r && st.have_r && std::isfinite(st.right_hand_pos[2]);
    hs.object_z_r = hs.have_object_z_r ? st.right_hand_pos[2] : 0.0;
    hs.have_object_z_l = enable_l && st.have_l && std::isfinite(st.left_hand_pos[2]);
    hs.object_z_l = hs.have_object_z_l ? st.left_hand_pos[2] : 0.0;
    return hs;
}

void log_arm_traj_plan_fail(const char *hand, int ret)
{
    if (ret == -31 || ret == -35 || ret == -36)
        std::cerr << "[arm] " << hand << "下降伺服中止 ret=" << ret
                  << "，原因见 logs/visual_servo\n";
    else
        std::cerr << "[arm] " << hand << "轨迹解析失败 ret=" << ret << "\n";
}
} // namespace move_box
