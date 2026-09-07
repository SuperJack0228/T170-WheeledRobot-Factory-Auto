#include "move_box_runtime.h"
#include "Ti5_socketcan.h"

#include <algorithm>
#include <iostream>
#include <unistd.h>

namespace move_box
{

namespace
{

bool aborting(const std::function<bool()> &should_abort)
{
    return hardware_abort_requested() || (should_abort && should_abort());
}

} // namespace

GraspToStandbyResult run_grasp_to_standby(
    gripper::Gripper &g,
    Robot_Arm &arm_r,
    Robot_Arm &arm_l,
    WaistRobot &waist,
    Eigen::Matrix<double, 1, 6> &posup_down_3_layer,
    RealSenseMultiCam &cameras,
    SegPoseBridge &bridge,
    const std::array<double, 16> &cam2robot,
    std::string &err,
    const std::function<bool()> &should_abort,
    SegEngineId engine_id)
{
    GraspToStandbyResult out;
    if (aborting(should_abort))
    {
        out.status = GraspToStandbyStatus::Aborted;
        return out;
    }

    if (engine_id == SegEngineId::Factory)
        log_phase_banner("工厂单类抓取：头相机识别分配(class0)");
    else if (engine_id == SegEngineId::Jindi)
        log_phase_banner("金帝四类抓取：头相机识别分配(毛胚 class0)");
    else
        log_phase_banner("抓取流程：头相机识别分配(class0)");
    g.openAndWait(gripper::Side::Right);
    g.openAndWait(gripper::Side::Left);
    if (aborting(should_abort))
    {
        out.status = GraspToStandbyStatus::Aborted;
        return out;
    }

    HeadAssignState st = detect_and_assign_head(cameras, bridge, cam2robot, true, engine_id);
    out.st = st;
    if (aborting(should_abort))
    {
        out.status = GraspToStandbyStatus::Aborted;
        return out;
    }

    {
        const auto &w = g_move_cfg.waist;
        auto lowest_object_z = [&](double &z) -> bool {
            bool any = false;
            z = 0.0;
            if (st.have_r)
            {
                z = st.right_hand_pos[2];
                any = true;
            }
            if (st.have_l)
            {
                if (!any || st.left_hand_pos[2] < z)
                    z = st.left_hand_pos[2];
                any = true;
            }
            return any;
        };
        double obj_z = 0.0;
        int lower_tries = 0;
        while (w.grasp_lower_z > 1e-6 && w.grasp_object_z_min < 0.0 && lower_tries < 2 &&
               lowest_object_z(obj_z) && obj_z < w.grasp_object_z_min)
        {
            const double need = w.grasp_object_z_min - obj_z;
            const double drop = std::min(w.grasp_lower_z, need);
            std::cout << std::fixed << std::setprecision(4)
                      << "[waist] 物体 z=" << obj_z << " < " << w.grasp_object_z_min
                      << "，腰下降靠近桌面（需要 " << need << " m）\n";
            if (waist_layer3_lower_z(waist, posup_down_3_layer, drop) <= 1e-9)
                break;
            ++lower_tries;
            if (aborting(should_abort))
            {
                out.status = GraspToStandbyStatus::Aborted;
                return out;
            }
            st = detect_and_assign_head(cameras, bridge, cam2robot, true, engine_id);
            out.st = st;
            if (aborting(should_abort))
            {
                out.status = GraspToStandbyStatus::Aborted;
                return out;
            }
        }
    }

    std::cout << "[head] goal_last_r: " << st.goal_last_r << std::endl;
    std::cout << "[head] goal_last_l: " << st.goal_last_l << std::endl;

    if (!st.have_r)
        std::cout << "[head] 未识别到右手，跳过右手手相机/抓取\n";
    if (!st.have_l)
        std::cout << "[head] 未识别到左手，跳过左手手相机/抓取\n";
    if (!st.move_r && st.have_r)
        log_arm_skip_right();
    if (!st.move_l && st.have_l)
        log_arm_skip_left();
    if (!st.move_r && !st.move_l)
    {
        out.status = GraspToStandbyStatus::NoTarget;
        return out;
    }

    const bool enable_r_hand = st.move_r;
    const bool enable_l_hand = st.move_l;

    bool stagger_r = st.move_r && head_x_needs_stagger(st.goal_last_r(0));
    bool stagger_l = st.move_l && head_x_needs_stagger(st.goal_last_l(0));

    bool waist_adjusted = false;
    GraspStaggerWaistState grasp_stagger;

    auto refresh_stagger_flags = [&]()
    {
        const int max_steps = std::max(1, g_move_cfg.waist.stagger_max_steps);
        if (grasp_stagger.steps_used >= max_steps)
        {
            stagger_r = false;
            stagger_l = false;
            return;
        }
        stagger_r = st.move_r && head_x_needs_stagger(st.goal_last_r(0));
        stagger_l = st.move_l && head_x_needs_stagger(st.goal_last_l(0));
    };

    bool grasped_r = false;
    bool grasped_l = false;
    bool lifted_r = false;
    bool lifted_l = false;

    auto finish_to_standby = [&](GraspToStandbyStatus if_empty) -> GraspToStandbyResult
    {
        GraspToStandbyResult r;
        r.st = st;
        r.grasped_r = grasped_r;
        r.grasped_l = grasped_l;
        r.waist_adjusted = waist_adjusted;
        if (!grasped_r && !grasped_l)
        {
            std::cout << "[arm] 本轮无抓取，回待命\n";
            r.status = if_empty;
            move_arms_to_standby(arm_r, arm_l);
            return r;
        }

        log_phase_banner("抓取完成 → 回待命");
        lift_grasped_after_pick(
            arm_r, arm_l, cameras, st, grasped_r, grasped_l, lifted_r, lifted_l);
        move_arms_to_standby(arm_r, arm_l);
        r.st = st;
        r.status = GraspToStandbyStatus::Ok;
        return r;
    };

    auto aborted_now = [&]() -> bool {
        if (!aborting(should_abort))
            return false;
        out.status = GraspToStandbyStatus::Aborted;
        out.st = st;
        out.grasped_r = grasped_r;
        out.grasped_l = grasped_l;
        out.waist_adjusted = waist_adjusted;
        std::cerr << "[grasp] STOP：退出整段抓取，停在原地\n";
        return true;
    };

    if (stagger_r && !stagger_l && st.move_l)
    {
        std::cout << "[stagger] 右手 x>" << g_move_cfg.stagger.head_x_threshold
                  << "，先执行左手\n";
        const ArmLineMoveResult head_tr_l =
            run_head_approach_selective(arm_r, arm_l, st, false, true);
        if (aborted_now())
            return out;
        HandMoveState hs_l = run_hand_detect_and_validate(
            arm_r, arm_l, cameras, bridge, st, false, enable_l_hand, err, engine_id);
        if (on_hand_steady_timeout_abort(hs_l, false, enable_l_hand, arm_r, arm_l) ||
            aborted_now())
            return out;
        if (!hs_l.move_l || head_tr_l.ret_l != 0)
        {
            log_arm_skip_left();
            move_arms_to_standby(arm_r, arm_l);
            out.status = GraspToStandbyStatus::GraspNone;
            return out;
        }
        const ArmLineMoveResult grasp_tr_l =
            run_hand_approach_and_grasp(g, arm_r, arm_l, st, hs_l);
        if (aborted_now())
            return out;
        if (grasp_tr_l.ret_l == 0)
        {
            grasped_l = true;
            lift_grasped_selective(arm_r, arm_l, cameras, st, false, true);
            lifted_l = true;
            if (aborted_now())
                return out;
        }
        move_arms_to_standby(arm_r, arm_l);
        if (aborted_now())
            return out;

        std::cout << "[stagger] 腰前进后抓右手(本轮腰进预算 "
                  << g_move_cfg.waist.stagger_max_steps << " 步)\n";
        waist_adjusted = true;
        if (!advance_waist_stagger_side(waist, posup_down_3_layer, true, st, grasp_stagger))
        {
            if (aborted_now())
                return out;
            std::cout << "[stagger] 腰前进后右手仍不可用，已抓左手，回待命\n";
            return finish_to_standby(GraspToStandbyStatus::GraspNone);
        }
        if (aborted_now())
            return out;
        refresh_stagger_flags();

        if (!st.move_r)
        {
            log_arm_skip_right();
            return finish_to_standby(GraspToStandbyStatus::GraspNone);
        }

        std::cout << "[stagger] 执行右手（左手已抓，goal_last_l 已保留）\n";
        const ArmLineMoveResult head_tr_r =
            run_head_approach_selective(arm_r, arm_l, st, true, false);
        if (aborted_now())
            return out;
        HandMoveState hs_r = run_hand_detect_and_validate(
            arm_r, arm_l, cameras, bridge, st, true, false, err, engine_id);
        if (on_hand_steady_timeout_abort(hs_r, true, false, arm_r, arm_l) || aborted_now())
            return out;
        if (hs_r.move_r && head_tr_r.ret_r == 0)
        {
            const ArmLineMoveResult grasp_tr_r =
                run_hand_approach_and_grasp(g, arm_r, arm_l, st, hs_r);
            if (aborted_now())
                return out;
            if (grasp_tr_r.ret_r == 0)
            {
                grasped_r = true;
                lift_grasped_selective(arm_r, arm_l, cameras, st, true, false);
                lifted_r = true;
                if (aborted_now())
                    return out;
            }
            move_arms_to_standby(arm_r, arm_l);
        }
        else
        {
            if (head_tr_r.ret_r != 0 || !hs_r.move_r)
                log_arm_skip_right();
        }
        if (aborted_now())
            return out;
        return finish_to_standby(GraspToStandbyStatus::GraspNone);
    }

    if (stagger_l && !stagger_r && st.move_r)
    {
        std::cout << "[stagger] 左手 x>" << g_move_cfg.stagger.head_x_threshold
                  << "，先执行右手\n";
        const ArmLineMoveResult head_tr_r =
            run_head_approach_selective(arm_r, arm_l, st, true, false);
        if (aborted_now())
            return out;
        HandMoveState hs_r = run_hand_detect_and_validate(
            arm_r, arm_l, cameras, bridge, st, enable_r_hand, false, err, engine_id);
        if (on_hand_steady_timeout_abort(hs_r, enable_r_hand, false, arm_r, arm_l) ||
            aborted_now())
            return out;
        if (!hs_r.move_r || head_tr_r.ret_r != 0)
        {
            log_arm_skip_right();
            move_arms_to_standby(arm_r, arm_l);
            out.status = GraspToStandbyStatus::GraspNone;
            return out;
        }
        const ArmLineMoveResult grasp_tr_r =
            run_hand_approach_and_grasp(g, arm_r, arm_l, st, hs_r);
        if (aborted_now())
            return out;
        if (grasp_tr_r.ret_r != 0)
        {
            move_arms_to_standby(arm_r, arm_l);
            out.status = GraspToStandbyStatus::GraspNone;
            return out;
        }
        grasped_r = true;
        lift_grasped_selective(arm_r, arm_l, cameras, st, true, false);
        lifted_r = true;
        if (aborted_now())
            return out;
        move_arms_to_standby(arm_r, arm_l);
        if (aborted_now())
            return out;

        std::cout << "[stagger] 腰前进后抓左手(本轮腰进预算 "
                  << g_move_cfg.waist.stagger_max_steps << " 步)\n";
        waist_adjusted = true;
        if (!advance_waist_stagger_side(waist, posup_down_3_layer, false, st, grasp_stagger))
        {
            if (aborted_now())
                return out;
            std::cout << "[stagger] 腰前进后左手仍不可用，已抓右手，回待命\n";
            return finish_to_standby(GraspToStandbyStatus::GraspNone);
        }
        if (aborted_now())
            return out;
        refresh_stagger_flags();

        if (!st.move_l)
        {
            log_arm_skip_left();
            return finish_to_standby(GraspToStandbyStatus::GraspNone);
        }

        std::cout << "[stagger] 执行左手（右手已抓，goal_last_r 已保留）\n";
        const ArmLineMoveResult head_tr_l =
            run_head_approach_selective(arm_r, arm_l, st, false, true);
        if (aborted_now())
            return out;
        HandMoveState hs_l = run_hand_detect_and_validate(
            arm_r, arm_l, cameras, bridge, st, false, true, err, engine_id);
        if (on_hand_steady_timeout_abort(hs_l, false, true, arm_r, arm_l) || aborted_now())
            return out;
        if (hs_l.move_l && head_tr_l.ret_l == 0)
        {
            const ArmLineMoveResult grasp_tr_l =
                run_hand_approach_and_grasp(g, arm_r, arm_l, st, hs_l);
            if (aborted_now())
                return out;
            if (grasp_tr_l.ret_l == 0)
            {
                grasped_l = true;
                lift_grasped_selective(arm_r, arm_l, cameras, st, false, true);
                lifted_l = true;
                if (aborted_now())
                    return out;
            }
            move_arms_to_standby(arm_r, arm_l);
        }
        else
        {
            if (head_tr_l.ret_l != 0 || !hs_l.move_l)
                log_arm_skip_left();
        }
        if (aborted_now())
            return out;
        return finish_to_standby(GraspToStandbyStatus::GraspNone);
    }

    if (stagger_r && stagger_l)
    {
        std::cout << "[stagger] 双手 x>" << g_move_cfg.stagger.head_x_threshold << "，腰前进\n";
        waist_adjusted = true;
        if (!advance_waist_stagger_dual_hands(waist, posup_down_3_layer, st, grasp_stagger))
        {
            if (aborted_now())
                return out;
            out.status = GraspToStandbyStatus::GraspNone;
            out.waist_adjusted = waist_adjusted;
            return out;
        }
        if (aborted_now())
            return out;
        refresh_stagger_flags();
    }

    if (stagger_r && !stagger_l && !st.move_l && st.move_r)
    {
        std::cout << "[stagger] 仅右手有效且 x>" << g_move_cfg.stagger.head_x_threshold
                  << "，腰前进后执行右手\n";
        waist_adjusted = true;
        if (!advance_waist_stagger_side(waist, posup_down_3_layer, true, st, grasp_stagger))
        {
            if (aborted_now())
                return out;
            out.status = GraspToStandbyStatus::GraspNone;
            out.waist_adjusted = waist_adjusted;
            return out;
        }
        if (aborted_now())
            return out;
        refresh_stagger_flags();
        st.move_l = false;
    }
    else if (stagger_l && !stagger_r && !st.move_r && st.move_l)
    {
        std::cout << "[stagger] 仅左手有效且 x>" << g_move_cfg.stagger.head_x_threshold
                  << "，腰前进后执行左手\n";
        waist_adjusted = true;
        if (!advance_waist_stagger_side(waist, posup_down_3_layer, false, st, grasp_stagger))
        {
            if (aborted_now())
                return out;
            out.status = GraspToStandbyStatus::GraspNone;
            out.waist_adjusted = waist_adjusted;
            return out;
        }
        if (aborted_now())
            return out;
        refresh_stagger_flags();
        st.move_r = false;
    }

    if (head_grasp_simultaneous(st))
    {
        std::cout << "[head] 左右侧区均有目标，双手同时抓取\n";
        const ArmLineMoveResult head_tr =
            run_head_approach_selective(arm_r, arm_l, st, st.move_r, st.move_l);
        if (aborted_now())
            return out;

        HandMoveState hs = run_hand_detect_and_validate(
            arm_r, arm_l, cameras, bridge, st, enable_r_hand, enable_l_hand, err, engine_id);

        if (on_hand_steady_timeout_abort(hs, enable_r_hand, enable_l_hand, arm_r, arm_l) ||
            aborted_now())
            return out;

        hs.move_r = hs.move_r && enable_r_hand && (!st.move_r || head_tr.ret_r == 0);
        hs.move_l = hs.move_l && enable_l_hand && (!st.move_l || head_tr.ret_l == 0);
        if (!hs.move_r && enable_r_hand)
            log_arm_skip_right();
        if (!hs.move_l && enable_l_hand)
            log_arm_skip_left();
        if (!hs.move_r && !hs.move_l)
        {
            out.status = GraspToStandbyStatus::GraspNone;
            out.waist_adjusted = waist_adjusted;
            return out;
        }

        const ArmLineMoveResult grasp_tr =
            run_hand_approach_and_grasp(g, arm_r, arm_l, st, hs);
        if (aborted_now())
            return out;
        grasped_r = hs.move_r && grasp_tr.ret_r == 0;
        grasped_l = hs.move_l && grasp_tr.ret_l == 0;
    }
    else
    {
        std::cout << "[head] 含中间区或侧区不全，依次：右手抓取→待机→左手抓取→待机\n";

        if (enable_r_hand && st.move_r)
        {
            const ArmLineMoveResult head_tr_r =
                run_head_approach_selective(arm_r, arm_l, st, true, false);
            if (aborted_now())
                return out;
            HandMoveState hs_r = run_hand_detect_and_validate(
                arm_r, arm_l, cameras, bridge, st, true, false, err, engine_id);
            if (on_hand_steady_timeout_abort(hs_r, true, false, arm_r, arm_l) || aborted_now())
                return out;
            if (hs_r.move_r && head_tr_r.ret_r == 0)
            {
                const ArmLineMoveResult grasp_tr_r =
                    run_hand_approach_and_grasp(g, arm_r, arm_l, st, hs_r);
                if (aborted_now())
                    return out;
                if (grasp_tr_r.ret_r == 0)
                {
                    grasped_r = true;
                    lift_grasped_selective(arm_r, arm_l, cameras, st, true, false);
                    lifted_r = true;
                    if (aborted_now())
                        return out;
                }
                move_arms_to_standby(arm_r, arm_l);
                if (aborted_now())
                    return out;
            }
            else
            {
                if (head_tr_r.ret_r != 0 || !hs_r.move_r)
                    log_arm_skip_right();
            }
        }

        if (enable_l_hand && st.move_l)
        {
            const ArmLineMoveResult head_tr_l =
                run_head_approach_selective(arm_r, arm_l, st, false, true);
            if (aborted_now())
                return out;
            HandMoveState hs_l = run_hand_detect_and_validate(
                arm_r, arm_l, cameras, bridge, st, false, true, err, engine_id);
            if (on_hand_steady_timeout_abort(hs_l, false, true, arm_r, arm_l) || aborted_now())
                return out;
            if (hs_l.move_l && head_tr_l.ret_l == 0)
            {
                const ArmLineMoveResult grasp_tr_l =
                    run_hand_approach_and_grasp(g, arm_r, arm_l, st, hs_l);
                if (aborted_now())
                    return out;
                if (grasp_tr_l.ret_l == 0)
                {
                    grasped_l = true;
                    lift_grasped_selective(arm_r, arm_l, cameras, st, false, true);
                    lifted_l = true;
                    if (aborted_now())
                        return out;
                }
                move_arms_to_standby(arm_r, arm_l);
                if (aborted_now())
                    return out;
            }
            else
            {
                if (head_tr_l.ret_l != 0 || !hs_l.move_l)
                    log_arm_skip_left();
            }
        }

        if (!grasped_r && !grasped_l)
        {
            out.status = GraspToStandbyStatus::GraspNone;
            out.waist_adjusted = waist_adjusted;
            return out;
        }
    }

    if (aborted_now())
        return out;
    return finish_to_standby(GraspToStandbyStatus::GraspNone);
}

GraspToStandbyResult run_hand_grasp_only(
    gripper::Gripper &g,
    Robot_Arm &arm_r,
    Robot_Arm &arm_l,
    RealSenseMultiCam &cameras,
    SegPoseBridge &bridge,
    std::string &err,
    const std::function<bool()> &should_abort,
    SegEngineId engine_id,
    bool enable_r,
    bool enable_l)
{
    GraspToStandbyResult out;
    if (aborting(should_abort))
    {
        out.status = GraspToStandbyStatus::Aborted;
        return out;
    }
    if (!enable_r && !enable_l)
    {
        std::cout << "[hand-only] 未指定手臂，跳过\n";
        out.status = GraspToStandbyStatus::NoTarget;
        return out;
    }

    if (engine_id == SegEngineId::Factory)
        log_phase_banner("手相机抓取(小毛胚)：识别 → 物体上方 → 下压夹取 → 抬起");
    else
        log_phase_banner("手相机抓取：识别 → 物体上方 → 下压夹取 → 抬起");

    if (enable_r)
        g.openAndWait(gripper::Side::Right);
    if (enable_l)
        g.openAndWait(gripper::Side::Left);
    if (aborting(should_abort))
    {
        out.status = GraspToStandbyStatus::Aborted;
        return out;
    }

    HeadAssignState st;
    st.goal_last_r = arm_get_tcp_pos(arm_r);
    st.goal_last_l = arm_get_tcp_pos(arm_l);
    st.have_r = enable_r;
    st.have_l = enable_l;
    st.move_r = enable_r;
    st.move_l = enable_l;
    out.st = st;

    std::cout << "[hand-only] 当前 TCP 右=" << st.goal_last_r << " 使能=" << enable_r << "\n";
    std::cout << "[hand-only] 当前 TCP 左=" << st.goal_last_l << " 使能=" << enable_l << "\n";

    HandMoveState hs = run_hand_detect_and_validate(
        arm_r, arm_l, cameras, bridge, st, enable_r, enable_l, err, engine_id, true);
    out.st = st;
    if (hs.steady_timed_out || aborting(should_abort))
    {
        std::cerr << "[hand-only] 停稳超时或中止，持料手保持原地\n";
        out.status = GraspToStandbyStatus::Aborted;
        out.st = st;
        return out;
    }
    if (!hs.move_r)
        log_arm_skip_right();
    if (!hs.move_l)
        log_arm_skip_left();
    if (!hs.move_r && !hs.move_l)
    {
        std::cout << "[hand-only] 手相机无有效目标，停在原处\n";
        out.status = GraspToStandbyStatus::NoTarget;
        return out;
    }

    const ArmLineMoveResult grasp_tr =
        run_hand_approach_and_grasp(g, arm_r, arm_l, st, hs);
    if (aborting(should_abort))
    {
        out.status = GraspToStandbyStatus::Aborted;
        out.st = st;
        return out;
    }

    const bool grasped_r = hs.move_r && grasp_tr.ret_r == 0;
    const bool grasped_l = hs.move_l && grasp_tr.ret_l == 0;
    out.grasped_r = grasped_r;
    out.grasped_l = grasped_l;
    out.st = st;
    if (!grasped_r && !grasped_l)
    {
        out.status = GraspToStandbyStatus::GraspNone;
        return out;
    }

    lift_grasped_selective(arm_r, arm_l, cameras, st, grasped_r, grasped_l);
    out.st = st;
    out.status = GraspToStandbyStatus::Ok;
    std::cout << "[hand-only] 完成 右=" << grasped_r << " 左=" << grasped_l << "（不回待命）\n";
    return out;
}

} // namespace move_box
