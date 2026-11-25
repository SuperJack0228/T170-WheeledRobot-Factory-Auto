#include "t170/skills/legacy/runtime.h"
#include <limits>
#include <stdexcept>
#include <iostream>

static void require(bool ok, const char *message)
{
    if (!ok) throw std::runtime_error(message);
}

int main()
{
    try {
        move_box::HeadAssignState target;
        target.have_r = true;
        target.have_l = true;
        target.right_hand_pos[2] = -.35;
        target.left_hand_pos[2] = -.36;
        target.goal_last_r.setConstant(.12);
        target.goal_last_l.setConstant(.13);
        const auto before_r = target.goal_last_r;
        const auto before_l = target.goal_last_l;
        auto both = move_box::prepare_tray_descent(target, true, true);
        require(both.move_r && both.move_l, "valid dual-arm target was lost");
        require(both.have_object_z_r && both.have_object_z_l &&
                both.object_z_r == -.35 && both.object_z_l == -.36, "head height was not preserved");
        auto left_only = move_box::prepare_tray_descent(target, false, true);
        require(!left_only.move_r && !left_only.have_object_z_r && left_only.object_z_r == 0 &&
                left_only.move_l, "inactive side leaked a stale target");
        target.left_hand_pos[2] = std::numeric_limits<double>::quiet_NaN();
        target.have_r = false;
        auto fallback = move_box::prepare_tray_descent(target, true, true);
        require(fallback.move_r && fallback.move_l && !fallback.have_object_z_l &&
                !fallback.have_object_z_r && fallback.object_z_l == 0 && fallback.object_z_r == 0,
                "unavailable height prevented the configured reference-height fallback");
        require(target.goal_last_r.isApprox(before_r) && target.goal_last_l.isApprox(before_l),
                "descent preparation changed the selected XY or wrist orientation");
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
