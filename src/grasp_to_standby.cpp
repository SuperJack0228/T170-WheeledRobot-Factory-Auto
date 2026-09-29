#include "move_box_runtime.h"
#include "head_control.h"
#include "Ti5_socketcan.h"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <string>
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
    SegEngineId engine_id,
    bool adaptive_approach_rpy,
    bool holding_r,
    bool holding_l,
    const std::function<void(bool did_r, bool did_l)> &on_place_retract)
{
    GraspToStandbyResult out;
    g_grasp_adaptive_rpy = adaptive_approach_rpy;
    struct AdaptiveRpyGuard
    {
        ~AdaptiveRpyGuard() { g_grasp_adaptive_rpy = false; }
    } adaptive_guard;

    if (aborting(should_abort))
    {
        out.status = GraspToStandbyStatus::Aborted;
        return out;
    }

    if (aborting(should_abort))
    {
        out.status = GraspToStandbyStatus::Aborted;
        return out;
    }

    {
        // 腰还停在远排前伸时，头必须是 far_pitch。补抓不能把头拉回标定 40°，
        // 否则近处两个码会切出画面。到位后再拍照，外参由 load_head_cam2robot 按编码器现算。
        const bool waist_at_far_row =
            posup_down_3_layer(0) > waist_ready_x() + 0.02;
        if (waist_at_far_row)
            std::cout << std::fixed << std::setprecision(4)
                      << "[head] 腰在远排前伸 x=" << posup_down_3_layer(0)
                      << " m，本轮头俯仰 "
                      << std::setprecision(1) << g_move_cfg.head.far_pitch_deg
                      << "°，按该角度现算相机外参\n";
        const int head_rc = waist_at_far_row ? enable_head_far_pitch()
                                             : enable_head_calib_pose();
        if (head_rc == -4 || aborting(should_abort))
        {
            out.status = GraspToStandbyStatus::Aborted;
            return out;
        }
        if (head_rc != 0)
        {
            std::cerr << "[grasp] 本轮头部使能/到位失败 code=" << head_rc << "，不拍照\n";
            out.status = GraspToStandbyStatus::CamFail;
            return out;
        }
    }

    if (tray_hole_task() == TrayHoleTask::PlaceEmpty)
        log_phase_banner("料盘2空孔放置：头相机识别分配(class3)");
    else if (adaptive_approach_rpy)
        log_phase_banner("抓取流程(自适应姿态)：头相机识别分配(class0)");
    else
        log_phase_banner("抓取流程：头相机识别分配(class0)");
    const bool skip_r = holding_r;
    const bool skip_l = holding_l;
    if (skip_r || skip_l)
        std::cout << (tray_hole_task() == TrayHoleTask::PlaceEmpty
                          ? "[tray2] 已放置 右="
                          : "[grasp] 已持件 右=")
                  << (skip_r ? 1 : 0)
                  << " 左=" << (skip_l ? 1 : 0)
                  << (tray_hole_task() == TrayHoleTask::PlaceEmpty
                          ? "，本轮只给未放侧补放\n"
                          : "，本轮只给空爪补抓，合爪保持\n");
    HeadAssignState st = detect_and_assign_head(
        cameras, bridge, cam2robot, true, engine_id, skip_r, skip_l);
    out.st = st;
    out.zone_count_ok = st.zone_count_ok;
    out.zone_right = st.zone_right;
    out.zone_left = st.zone_left;
    if (aborting(should_abort))
    {
        out.status = GraspToStandbyStatus::Aborted;
        return out;
    }

    {
        std::cout << std::fixed << std::setprecision(4)
                  << "[waist] 抓取腰高 ready_z=" << waist_ready_z()
                  << " m（home z=" << g_move_cfg.waist.layer3_home(2)
                  << "）盘面参考 z=" << tray_z_ref_now()
                  << " 物体顶面参考=" << expected_object_top_z() << " m\n";
    }

    std::cout << "[head] goal_last_r: " << st.goal_last_r << std::endl;
    std::cout << "[head] goal_last_l: " << st.goal_last_l << std::endl;

    if (!st.have_r)
        std::cout << (skip_r ? "[head] 右手已持件，本轮不抓右手\n"
                             : "[head] 未识别到右手，跳过右手手相机/抓取\n");
    if (!st.have_l)
        std::cout << (skip_l ? "[head] 左手已持件，本轮不抓左手\n"
                             : "[head] 未识别到左手，跳过左手手相机/抓取\n");
    if (!st.move_r && st.have_r)
        log_arm_skip_right();
    if (!st.move_l && st.have_l)
        log_arm_skip_left();
    if (!st.move_r && !st.move_l)
    {
        out.status = GraspToStandbyStatus::NoTarget;
        return out;
    }

    bool waist_adjusted = false;
    {
        const bool far_r = st.move_r && tray_row_is_far(st.row_r);
        const bool far_l = st.move_l && tray_row_is_far(st.row_l);
        if (far_r || far_l)
        {
            std::cout << "[waist] 目标在离机器人远的三排"
                      << (far_r ? " 右 from_robot=" + std::to_string(st.row_r) +
                                      " ArUco=" + std::to_string(st.row_aruco_r)
                                : "")
                      << (far_l ? " 左 from_robot=" + std::to_string(st.row_l) +
                                      " ArUco=" + std::to_string(st.row_aruco_l)
                                : "")
                      << "，腰前伸后抓取，本轮抓完不收回\n";
            const double actual =
                waist_layer3_ensure_far_row_forward(waist, posup_down_3_layer);
            if (actual > 1e-9)
                waist_adjusted = true;
            if (aborting(should_abort))
            {
                out.status = GraspToStandbyStatus::Aborted;
                out.waist_adjusted = waist_adjusted;
                return out;
            }
            std::cout << "[head] 远三排：低头到 " << std::fixed << std::setprecision(1)
                      << g_move_cfg.head.far_pitch_deg
                      << "°，free_calib 重算外参后重拍\n";
            const int head_rc = enable_head_far_pitch();
            if (head_rc == -4 || aborting(should_abort))
            {
                out.status = GraspToStandbyStatus::Aborted;
                out.waist_adjusted = waist_adjusted;
                return out;
            }
            if (head_rc != 0)
                std::cerr << "[head] 远排低头失败 code=" << head_rc << "，仍按当前头角重拍\n";
            HeadAssignState st_far = detect_and_assign_head(
                cameras, bridge, cam2robot, true, engine_id, skip_r, skip_l);
            if (aborting(should_abort))
            {
                out.status = GraspToStandbyStatus::Aborted;
                out.waist_adjusted = waist_adjusted;
                return out;
            }
            if (st_far.move_l || st_far.move_r)
            {
                st = st_far;
                std::cout << "[head] 低头后已用当前外参重分配目标\n";
            }
            else
            {
                if (actual > 1e-9)
                {
                    apply_grasp_stagger_waist_comp(st, actual);
                    std::cout << std::fixed << std::setprecision(4)
                              << "[head] 低头后未分到目标，回退腰进前孔位并补偿 x "
                              << actual << " m\n";
                }
                else
                    std::cerr << "[head] 低头后未分到目标，沿用低头前分配\n";
            }
        }
    }

    if (tray2_precision_test() && (st.move_r || st.move_l) &&
        !(st.move_r && tray_row_is_far(st.row_r)) &&
        !(st.move_l && tray_row_is_far(st.row_l)) &&
        posup_down_3_layer(0) > waist_ready_x() + 0.02)
    {
        std::cout << "[tray2test] 前三排结束，腰收回 ready（不站到 home），重拍后三排\n";
        if (!ensure_waist_ready_start(waist, posup_down_3_layer))
        {
            out.status = aborting(should_abort) ? GraspToStandbyStatus::Aborted
                                                : GraspToStandbyStatus::CamFail;
            return out;
        }
        const int head_rc = enable_head_calib_pose();
        if (head_rc == -4 || aborting(should_abort))
        {
            out.status = GraspToStandbyStatus::Aborted;
            return out;
        }
        HeadAssignState st_near = detect_and_assign_head(
            cameras, bridge, cam2robot, true, engine_id, skip_r, skip_l);
        if (aborting(should_abort))
        {
            out.status = GraspToStandbyStatus::Aborted;
            return out;
        }
        if (st_near.move_l || st_near.move_r)
            st = st_near;
    }

    const bool enable_r_hand = st.move_r;
    const bool enable_l_hand = st.move_l;

    bool stagger_r = st.move_r && head_x_needs_stagger(st.goal_last_r(0));
    bool stagger_l = st.move_l && head_x_needs_stagger(st.goal_last_l(0));

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

    auto retract_placed_nostop = [&](bool want_r, bool want_l) -> bool {
        if (!want_r && !want_l)
            return true;
        auto fill = [](Robot_Arm &arm, bool is_right,
                       Eigen::Matrix<double, 1, 6> &mid,
                       Eigen::Matrix<double, 1, 6> &goal) {
            mid = arm_get_tcp_pos(arm);
            mid(2) += g_move_cfg.head_grasp.lift_after_grasp_z;
            if (mid(2) > g_move_cfg.grasp_valid.z_max)
                mid(2) = g_move_cfg.grasp_valid.z_max;
            if (tray_hole_task() == TrayHoleTask::PlaceEmpty)
                goal = is_right ? make_home_tcp_right() : make_home_tcp_left();
            else
                goal = is_right ? g_move_cfg.conveyor.tcp.right : g_move_cfg.conveyor.tcp.left;
        };
        Eigen::Matrix<double, 1, 6> mid_r = Eigen::Matrix<double, 1, 6>::Zero();
        Eigen::Matrix<double, 1, 6> goal_r = Eigen::Matrix<double, 1, 6>::Zero();
        Eigen::Matrix<double, 1, 6> mid_l = Eigen::Matrix<double, 1, 6>::Zero();
        Eigen::Matrix<double, 1, 6> goal_l = Eigen::Matrix<double, 1, 6>::Zero();
        if (want_r)
            fill(arm_r, true, mid_r, goal_r);
        if (want_l)
            fill(arm_l, false, mid_l, goal_l);
        const auto &hg = g_move_cfg.head_grasp;
        const ArmLineMoveResult stitched = arm_dual_line_then_line_nostop(
            arm_r, mid_r, goal_r, want_r,
            arm_l, mid_l, goal_l, want_l,
            hg.lift_vel_m_s, hg.return_vel_m_s,
            [&]() {
                if (on_place_retract)
                    on_place_retract(grasped_r, grasped_l);
            },
            true);
        const bool ok_r = !want_r || stitched.ret_r == 0;
        const bool ok_l = !want_l || stitched.ret_l == 0;
        if (ok_r && ok_l)
            return true;
        std::cerr << "[arm] 收手衔接失败 右=" << stitched.ret_r
                  << " 左=" << stitched.ret_l << "，退回抬升后直线\n";
        lift_grasped_selective(arm_r, arm_l, cameras, st, want_r, want_l);
        if (on_place_retract)
            on_place_retract(grasped_r, grasped_l);
        return move_arms_tcp_to_home(arm_r, arm_l, want_r, want_l);
    };

    auto clear_after_place = [&](bool is_right) {
        if (!tray2_precision_test())
        {
            retract_placed_nostop(is_right, !is_right);
            if (is_right)
                lifted_r = true;
            else
                lifted_l = true;
            return;
        }
        lift_grasped_selective(arm_r, arm_l, cameras, st, is_right, !is_right);
        if (is_right)
            lifted_r = true;
        else
            lifted_l = true;
    };

    auto finish_to_standby = [&](GraspToStandbyStatus if_empty) -> GraspToStandbyResult
    {
        GraspToStandbyResult r;
        r.st = st;
        r.grasped_r = grasped_r;
        r.grasped_l = grasped_l;
        r.waist_adjusted = waist_adjusted;
        r.zone_count_ok = out.zone_count_ok;
        r.zone_right = out.zone_right;
        r.zone_left = out.zone_left;
        if (!grasped_r && !grasped_l)
        {
            std::cout << (tray_hole_task() == TrayHoleTask::PlaceEmpty
                              ? "[arm] 本轮无放置，停在当前 TCP\n"
                              : "[arm] 本轮无抓取，停在当前 TCP\n");
            r.status = if_empty;
            return r;
        }

        if (tray2_precision_test())
        {
            log_phase_banner("精度测试放置完成 → 轻抬后回蹲姿 standby，不起身");
            lift_grasped_after_pick(
                arm_r, arm_l, cameras, st, grasped_r, grasped_l, lifted_r, lifted_l);
            move_arms_to_standby(arm_r, arm_l, -1, true);
        }
        else
        {
            log_phase_banner(tray_hole_task() == TrayHoleTask::PlaceEmpty
                                 ? "放置完成 → 竖直抬升后不停，直接直线回 home_tcp"
                                 : "抓取完成 → 竖直抬升后不停，只把抓到的手去传送带准备 tcp");
            const bool need_r = grasped_r && !lifted_r;
            const bool need_l = grasped_l && !lifted_l;
            if (need_r || need_l)
                retract_placed_nostop(need_r, need_l);
        }
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
            clear_after_place(false);
            if (aborted_now())
                return out;
        }

        std::cout << "[stagger] 腰前进后抓右手(本轮腰进预算 "
                  << g_move_cfg.waist.stagger_max_steps << " 步)\n";
        waist_adjusted = true;
        if (!advance_waist_stagger_side(waist, posup_down_3_layer, true, st, grasp_stagger))
        {
            if (aborted_now())
                return out;
            std::cout << "[stagger] 腰前进后右手仍不可用，已抓左手，夹后抬升停在当前 TCP\n";
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
            clear_after_place(true);
                if (aborted_now())
                    return out;
            }
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
            out.status = GraspToStandbyStatus::GraspNone;
            return out;
        }
        const ArmLineMoveResult grasp_tr_r =
            run_hand_approach_and_grasp(g, arm_r, arm_l, st, hs_r);
        if (aborted_now())
            return out;
        if (grasp_tr_r.ret_r != 0)
        {
            out.status = GraspToStandbyStatus::GraspNone;
            return out;
        }
        grasped_r = true;
            clear_after_place(true);
        if (aborted_now())
            return out;

        std::cout << "[stagger] 腰前进后抓左手(本轮腰进预算 "
                  << g_move_cfg.waist.stagger_max_steps << " 步)\n";
        waist_adjusted = true;
        if (!advance_waist_stagger_side(waist, posup_down_3_layer, false, st, grasp_stagger))
        {
            if (aborted_now())
                return out;
            std::cout << "[stagger] 腰前进后左手仍不可用，已抓右手，夹后抬升停在当前 TCP\n";
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
                clear_after_place(false);
                if (aborted_now())
                    return out;
            }
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

    if (st.move_r && st.move_l)
    {
        const int cr = (st.col_r >= 1) ? st.col_r : tray_column_index_from_y(st.goal_last_r(1));
        const int cl = (st.col_l >= 1) ? st.col_l : tray_column_index_from_y(st.goal_last_l(1));
        std::cout << "[col] 本轮配对 右列=" << cr << " 左列=" << cl
                  << " 差=" << (cl - cr)
                  << " 同时门槛=" << g_move_cfg.grasp_zone.min_simultaneous_col_delta
                  << " (最小对 1-4/2-5/3-6)\n";
        (void)dual_grasp_targets_too_close(st);
    }

    if (head_grasp_simultaneous(st))
    {
        std::cout << "[head] 左右列分区均有目标且列间隔足够，双手同时抓取\n";
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
        std::cout << "[head] 列间隔不足或单侧无目标，依次：右手 TCP 抓取→左手 TCP 抓取（不回 standby）\n";

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
            clear_after_place(true);
                    if (aborted_now())
                        return out;
                }
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
                    clear_after_place(false);
                    if (aborted_now())
                        return out;
                }
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

} // namespace move_box
