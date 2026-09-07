#include "move_box_runtime.h"

#include "place_grid_correct.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <optional>
#include <random>
#include <sstream>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <unordered_set>

#include "head.h"
#include "Ti5_socketcan.h"
#include "wt_box_tcp.h"
#include "reals_tcp.h"
#include "wt_client.h"
#include "tts_http_client.h"

namespace move_box
{

    Eigen::Matrix<double, 1, 6> make_standby_pos_right()
    {
        return g_move_cfg.standby.right;
    }

    Eigen::Matrix<double, 1, 6> make_standby_pos_left()
    {
        return g_move_cfg.standby.left;
    }

    bool goal_z_in_range(double z)
    {
        return z >= g_move_cfg.grasp_valid.z_min && z <= g_move_cfg.grasp_valid.z_max;
    }

    bool goal_pos_valid_right(double x, double y, double z)
    {
        const auto &v = g_move_cfg.grasp_valid;
        return x >= v.x_min && x <= v.x_max && y >= v.right_y_min && y <= v.right_y_max && goal_z_in_range(z);
    }

    bool goal_pos_valid_left(double x, double y, double z)
    {
        const auto &v = g_move_cfg.grasp_valid;
        return x >= v.x_min && x <= v.x_max && y >= v.left_y_min && y <= v.left_y_max && goal_z_in_range(z);
    }

    /** 放货候选格点：按 place_zone.y_side_split 分左右（与侧区 left_idx/right_idx 规则一致） */
    bool place_slot_y_for_hand(double y, bool is_right, double y_split)
    {
        return is_right ? (y < y_split) : (y > y_split);
    }

    std::vector<size_t> filter_place_indices_for_hand(
        const std::vector<Eigen::Matrix<double, 1, 6>> &poses,
        const std::vector<size_t> &indices,
        bool is_right,
        double y_split)
    {
        std::vector<size_t> out;
        out.reserve(indices.size());
        for (size_t idx : indices)
        {
            if (idx >= poses.size())
                continue;
            if (place_slot_y_for_hand(poses[idx](1), is_right, y_split))
                out.push_back(idx);
        }
        return out;
    }

    bool head_hand_detected(const double hand_pos[3])
    {
        return hand_pos[0] >= 0.0;
    }

    bool head_x_needs_stagger(double x)
    {
        return x > g_move_cfg.stagger.head_x_threshold;
    }

    /** 抓取腰进：超出 threshold 的部分，1 步够→×1，2 步够→×2，否则×3（不超过剩余预算） */
    int stagger_waist_steps_for_excess(double grasp_x, int steps_remaining)
    {
        if (steps_remaining <= 0 || !head_x_needs_stagger(grasp_x))
            return 0;
        const double threshold = g_move_cfg.stagger.head_x_threshold;
        const double step = g_move_cfg.waist.stagger_step_x;
        const double excess = grasp_x - threshold;
        if (excess <= 1e-9)
            return 0;
        int need = 3;
        if (excess <= step + 1e-9)
            need = 1;
        else if (excess <= 2.0 * step + 1e-9)
            need = 2;
        return std::min(need, steps_remaining);
    }

    double hover_z_from_object(double object_z)
    {
        const auto &hg = g_move_cfg.head_grasp;
        const auto &v = g_move_cfg.grasp_valid;
        if (!(object_z >= v.z_min && object_z <= v.z_max))
            return hg.goal_z_base + hg.goal_z_extra;
        return object_z + hg.hover_above_m;
    }

    void fill_goal_last_from_head_assign(
        const double right_hand_pos[3],
        const double left_hand_pos[3],
        Eigen::Matrix<double, 1, 6> &goal_last_r,
        Eigen::Matrix<double, 1, 6> &goal_last_l)
    {
        const auto &hg = g_move_cfg.head_grasp;
        const double z_r = hover_z_from_object(right_hand_pos[2]);
        const double z_l = hover_z_from_object(left_hand_pos[2]);
        goal_last_r << right_hand_pos[0], right_hand_pos[1], z_r,
            hg.right_rx_deg * rad, hg.right_ry_deg * rad, hg.right_rz_deg * rad;
        goal_last_l << left_hand_pos[0], left_hand_pos[1], z_l,
            hg.left_rx_deg * rad, hg.left_ry_deg * rad, hg.left_rz_deg * rad;
        goal_last_r(0) += hg.goal_x_offset;
        goal_last_l(0) += hg.goal_x_offset;
        clamp_cart_goal_z(goal_last_r, "fill_goal_last_from_head_assign(右手, 物体上方)");
        clamp_cart_goal_z(goal_last_l, "fill_goal_last_from_head_assign(左手, 物体上方)");
        const auto &v = g_move_cfg.grasp_valid;
        auto log_hover = [&](const char *hand, const double pos[3], double hover_z)
        {
            if (!head_hand_detected(pos))
                return;
            std::cout << std::fixed << std::setprecision(4)
                      << "[head] " << hand << " 物体 xyz=(" << pos[0] << "," << pos[1] << ","
                      << pos[2] << ") → 上方 z=" << hover_z;
            if (!(pos[2] >= v.z_min && pos[2] <= v.z_max))
                std::cout << " (检测z超范围, 用备用高度)";
            std::cout << " hover=" << hg.hover_above_m << " m\n";
        };
        log_hover("右", right_hand_pos, goal_last_r(2));
        log_hover("左", left_hand_pos, goal_last_l(2));
    }

    bool head_move_allowed_right(
        const double right_hand_pos[3],
        const Eigen::Matrix<double, 1, 6> &goal_last_r)
    {
        return goal_pos_valid_right(goal_last_r(0), goal_last_r(1), right_hand_pos[2]);
    }

    bool head_move_allowed_left(
        const double left_hand_pos[3],
        const Eigen::Matrix<double, 1, 6> &goal_last_l)
    {
        return goal_pos_valid_left(goal_last_l(0), goal_last_l(1), left_hand_pos[2]);
    }

    void apply_hand_hover_z_if_valid(
        const Eigen::Matrix<double, 1, 6> &base_pos,
        Eigen::Matrix<double, 1, 6> &goal_last)
    {
        const auto &v = g_move_cfg.grasp_valid;
        if (!(base_pos(2) >= v.z_min && base_pos(2) <= v.z_max))
            return;
        goal_last(2) = hover_z_from_object(base_pos(2));
        clamp_cart_goal_z(goal_last, "手相机物体上方");
    }

    void apply_hand_base_to_goal(
        bool have_r,
        const Eigen::Matrix<double, 1, 6> &base_pos_r,
        bool have_l,
        const Eigen::Matrix<double, 1, 6> &base_pos_l,
        Eigen::Matrix<double, 1, 6> &goal_last_r,
        Eigen::Matrix<double, 1, 6> &goal_last_l,
        bool hover_z_from_hand)
    {
        const auto &hg = g_move_cfg.head_grasp;
        if (have_r)
        {
            goal_last_r(0) = base_pos_r(0) + hg.hand_right.offset_x;
            goal_last_r(1) = base_pos_r(1) + hg.hand_right.offset_y;
            if (hover_z_from_hand)
                apply_hand_hover_z_if_valid(base_pos_r, goal_last_r);
        }
        if (have_l)
        {
            goal_last_l(0) = base_pos_l(0) + hg.hand_left.offset_x;
            goal_last_l(1) = base_pos_l(1) + hg.hand_left.offset_y;
            if (hover_z_from_hand)
                apply_hand_hover_z_if_valid(base_pos_l, goal_last_l);
        }
    }

    void update_hand_goal_xy_from_base(
        const Eigen::Matrix<double, 1, 6> &base_pos,
        Eigen::Matrix<double, 1, 6> &goal_last,
        double offset_x,
        double offset_y)
    {
        goal_last(0) = base_pos(0) + offset_x;
        goal_last(1) = base_pos(1) + offset_y;
    }

    bool hand_move_allowed_right(
        const Eigen::Matrix<double, 1, 6> &goal_last_r,
        const Eigen::Matrix<double, 1, 6> &base_pos_r)
    {
        return goal_pos_valid_right(goal_last_r(0), goal_last_r(1), base_pos_r(2));
    }

    bool hand_move_allowed_left(
        const Eigen::Matrix<double, 1, 6> &goal_last_l,
        const Eigen::Matrix<double, 1, 6> &base_pos_l)
    {
        return goal_pos_valid_left(goal_last_l(0), goal_last_l(1), base_pos_l(2));
    }

    void log_arm_traj_plan_fail(const char *hand, int ret)
    {
        std::cerr << "[arm] " << hand << "轨迹解析失败 ret=" << ret << "\n";
        // 详细记录（goal/stage）由 arm_line_move 内 append_line_trajectory_fail_debug 写入
    }

    void log_arm_skip_right()
    {
        cout << "[arm] 右手目标超范围(x:0.1~0.8, y:-0.5~0.05, z:-0.50~-0.15)，本段不移动\n";
    }

    void log_arm_skip_left()
    {
        cout << "[arm] 左手目标超范围(x:0.1~0.8, y:-0.05~0.5, z:-0.50~-0.15)，本段不移动\n";
    }


    /** 已抓一侧的 goal_last 快照：腰进后重拍头时不得覆盖 */

    void grasp_snapshot_from_state(
        const HeadAssignState &st,
        bool grasped_r,
        bool grasped_l,
        GraspedGoalSnapshot &snap)
    {
        snap.grasped_r = grasped_r;
        snap.grasped_l = grasped_l;
        if (grasped_r)
            snap.goal_last_r = st.goal_last_r;
        if (grasped_l)
            snap.goal_last_l = st.goal_last_l;
    }

    void apply_head_assign_keep_grasped(HeadAssignState &st, const GraspedGoalSnapshot &snap)
    {
        if (snap.grasped_r)
        {
            st.goal_last_r = snap.goal_last_r;
            st.move_r = false;
        }
        if (snap.grasped_l)
        {
            st.goal_last_l = snap.goal_last_l;
            st.move_l = false;
        }
    }

    bool head_grasp_simultaneous(const HeadAssignState &st)
    {
        return st.move_r && st.move_l && st.zones.r_from_side && st.zones.l_from_side;
    }

    void move_arms_to_standby(Robot_Arm &arm_r, Robot_Arm &arm_l)
    {
        Eigen::Matrix<double, 1, 6> posr = make_standby_pos_right();
        Eigen::Matrix<double, 1, 6> posl = make_standby_pos_left();
        const Eigen::Matrix<double, 1, 6> cur_r = arm_get_tcp_pos(arm_r);
        const Eigen::Matrix<double, 1, 6> cur_l = arm_get_tcp_pos(arm_l);
        // 手先在料盘上方时，直线插到待机会斜着扫台面。先竖直抬到待机高度，再平移+改姿态。
        Eigen::Matrix<double, 1, 6> lift_r = cur_r;
        Eigen::Matrix<double, 1, 6> lift_l = cur_l;
        const bool need_r = cur_r(2) < posr(2) - 0.01;
        const bool need_l = cur_l(2) < posl(2) - 0.01;
        if (need_r)
            lift_r(2) = posr(2);
        if (need_l)
            lift_l(2) = posl(2);
        if (need_r || need_l)
        {
            cout << std::fixed << std::setprecision(4)
                 << "[arm] 回待机：先竖直抬起 右 z " << cur_r(2) << "→" << lift_r(2)
                 << " 左 z " << cur_l(2) << "→" << lift_l(2) << "\n";
            ArmLineMoveDebugStage stage("回待机先抬起");
            arm_dual_line_move_selective(arm_r, lift_r, need_r, arm_l, lift_l, need_l, 0.2);
            if (hardware_abort_requested())
                return;
        }
        cout << "[arm] 双臂回待机位\n";
        {
            ArmLineMoveDebugStage stage("回待机");
            arm_dual_line_move(arm_r, posr, arm_l, posl, 0.2);
        }
        sleep(1);
    }

    void refresh_head_move_flags(HeadAssignState &st)
    {
        st.have_r = head_hand_detected(st.right_hand_pos);
        st.have_l = head_hand_detected(st.left_hand_pos);
        st.move_r = st.have_r && head_move_allowed_right(st.right_hand_pos, st.goal_last_r);
        st.move_l = st.have_l && head_move_allowed_left(st.left_hand_pos, st.goal_last_l);
    }

    bool clamp_waist_layer3_xyz(Eigen::Matrix<double, 1, 6> &pos)
    {
        const auto &w = g_move_cfg.waist;
        bool ch = false;
        if (w.x_min < w.x_max)
        {
            if (pos(0) < w.x_min)
            {
                pos(0) = w.x_min;
                ch = true;
            }
            if (pos(0) > w.x_max)
            {
                pos(0) = w.x_max;
                ch = true;
            }
        }
        if (w.z_min < w.z_max)
        {
            if (pos(2) < w.z_min)
            {
                pos(2) = w.z_min;
                ch = true;
            }
            if (pos(2) > w.z_max)
            {
                pos(2) = w.z_max;
                ch = true;
            }
        }
        return ch;
    }

    double waist_layer3_lower_z(
        WaistRobot &waist,
        Eigen::Matrix<double, 1, 6> &layer3,
        double drop_m)
    {
        if (drop_m <= 1e-9)
            return 0.0;
        const double before = layer3(2);
        layer3(2) -= drop_m;
        clamp_waist_layer3_xyz(layer3);
        const double actual = before - layer3(2);
        if (actual <= 1e-9)
        {
            std::cout << "[waist] 抓取降腰跳过：已在 z_min=" << g_move_cfg.waist.z_min
                      << " 当前 z=" << before << "\n";
            return 0.0;
        }
        std::cout << std::fixed << std::setprecision(4)
                  << "[waist] 抓取降腰 " << actual << " m  layer3.z " << before << " → "
                  << layer3(2) << " (限位 z_min=" << g_move_cfg.waist.z_min << ")\n";
        waist.moveLToPos(layer3, 0.1);
        hardware_abort_sleep_ms(static_cast<int>(g_move_cfg.waist.move_settle_sec * 1000.0));
        return actual;
    }

    HeadAssignState detect_and_assign_head(
        RealSenseMultiCam &cameras,
        SegPoseBridge &bridge,
        const std::array<double, 16> &cam2robot,
        bool retry_if_incomplete,
        SegEngineId engine_id)
    {
        HeadAssignState st;
        if (kDebugVisualize)
            pose_vis_begin_phase(PoseVisLayout::Single, PoseVisPanel::Head, {PoseVisPanel::Head});
        PoseDetectionRecords head_records =
            detect_pose_at_slot(
                cameras, bridge, CameraSlot::Head, kDebugVisualize, kGraspDetectClassId, engine_id);
        assign_hand_targets_in_robot_frame(
            head_records, cam2robot, st.right_hand_pos, st.left_hand_pos, &st.zones);
        fill_goal_last_from_head_assign(
            st.right_hand_pos, st.left_hand_pos, st.goal_last_r, st.goal_last_l);
        refresh_head_move_flags(st);

        /** 至少一侧未分配到位则重拍一次（防图像质量漏检）；双手都已分配到则不再重拍 */
        if (retry_if_incomplete && (!st.have_r || !st.have_l))
        {
            cout << "[head] 左右未同时分配到目标(右="
                 << (st.have_r ? "有" : "无") << " 左=" << (st.have_l ? "有" : "无")
                 << ")，头相机重试一次\n";
            if (kDebugVisualize)
                pose_vis_clear_panels({PoseVisPanel::Head});
            head_records =
                detect_pose_at_slot(
                    cameras, bridge, CameraSlot::Head, kDebugVisualize, kGraspDetectClassId, engine_id);
            assign_hand_targets_in_robot_frame(
                head_records, cam2robot, st.right_hand_pos, st.left_hand_pos, &st.zones);
            fill_goal_last_from_head_assign(
                st.right_hand_pos, st.left_hand_pos, st.goal_last_r, st.goal_last_l);
            refresh_head_move_flags(st);
        }
        return st;
    }

    HeadAssignState detect_and_assign_head_keep_grasped(
        RealSenseMultiCam &cameras,
        SegPoseBridge &bridge,
        const std::array<double, 16> &cam2robot,
        bool retry_if_incomplete,
        const GraspedGoalSnapshot *keep_grasped,
        SegEngineId engine_id)
    {
        HeadAssignState st =
            detect_and_assign_head(cameras, bridge, cam2robot, retry_if_incomplete, engine_id);
        if (keep_grasped != nullptr)
            apply_head_assign_keep_grasped(st, *keep_grasped);
        return st;
    }

    ArmLineMoveResult run_head_approach_selective(
        Robot_Arm &arm_r,
        Robot_Arm &arm_l,
        HeadAssignState &st,
        bool move_r,
        bool move_l)
    {
        log_phase_banner("抓取流程：先到物体上方");
        ArmLineMoveResult out;
        Eigen::Matrix<double, 1, 6> go_r = st.goal_last_r;
        Eigen::Matrix<double, 1, 6> go_l = st.goal_last_l;
        if (move_r)
        {
            const Eigen::Matrix<double, 1, 6> cur = arm_get_tcp_pos(arm_r);
            go_r(3) = cur(3);
            go_r(4) = cur(4);
            go_r(5) = cur(5);
        }
        if (move_l)
        {
            const Eigen::Matrix<double, 1, 6> cur = arm_get_tcp_pos(arm_l);
            go_l(3) = cur(3);
            go_l(4) = cur(4);
            go_l(5) = cur(5);
        }
        {
            ArmLineMoveDebugStage stage("到物体上方");
            out = arm_dual_line_move_selective(
                arm_r, go_r, move_r, arm_l, go_l, move_l, 0.2);
        }
        if (move_r && out.ret_r != 0)
            log_arm_traj_plan_fail("右手", out.ret_r);
        if (move_l && out.ret_l != 0)
            log_arm_traj_plan_fail("左手", out.ret_l);
        if ((move_r && out.ret_r != 0) || (move_l && out.ret_l != 0))
            return out;

        const bool need_rpy_r = move_r && ((st.goal_last_r.tail<3>() - go_r.tail<3>()).cwiseAbs().maxCoeff() > 1e-3);
        const bool need_rpy_l = move_l && ((st.goal_last_l.tail<3>() - go_l.tail<3>()).cwiseAbs().maxCoeff() > 1e-3);
        if (need_rpy_r || need_rpy_l)
        {
            ArmLineMoveDebugStage stage("物体上方转抓取姿态");
            const ArmLineMoveResult rpy = arm_dual_line_move_selective(
                arm_r, st.goal_last_r, need_rpy_r, arm_l, st.goal_last_l, need_rpy_l, 0.2);
            if (need_rpy_r && rpy.ret_r != 0)
            {
                std::cerr << "[arm] 右手抓取姿态直线失败，保持当前姿态停在物体上方\n";
                const Eigen::Matrix<double, 1, 6> cur = arm_get_tcp_pos(arm_r);
                st.goal_last_r(3) = cur(3);
                st.goal_last_r(4) = cur(4);
                st.goal_last_r(5) = cur(5);
            }
            if (need_rpy_l && rpy.ret_l != 0)
            {
                std::cerr << "[arm] 左手抓取姿态直线失败，保持当前姿态停在物体上方\n";
                const Eigen::Matrix<double, 1, 6> cur = arm_get_tcp_pos(arm_l);
                st.goal_last_l(3) = cur(3);
                st.goal_last_l(4) = cur(4);
                st.goal_last_l(5) = cur(5);
            }
        }
        return out;
    }


    /** 停稳超时：回待机；若本次等待包含双手则返回 true（整轮 continue） */
    bool on_hand_steady_timeout_abort(
        const HandMoveState &hs,
        bool enable_r,
        bool enable_l,
        Robot_Arm &arm_r,
        Robot_Arm &arm_l)
    {
        if (hardware_abort_requested())
            return true;
        if (!hs.steady_timed_out)
            return false;

        cout << "[hand] 手相机前停稳超时";
        if (enable_r && enable_l)
            cout << "（双手）";
        else if (enable_r)
            cout << "（右手）";
        else if (enable_l)
            cout << "（左手）";
        cout << "，双臂回待机，本侧不抓取\n";
        move_arms_to_standby(arm_r, arm_l);
        return enable_r && enable_l;
    }

    HandMoveState run_hand_detect_and_validate(
        Robot_Arm &arm_r,
        Robot_Arm &arm_l,
        RealSenseMultiCam &cameras,
        SegPoseBridge &bridge,
        HeadAssignState &st,
        bool enable_r,
        bool enable_l,
        std::string &err,
        SegEngineId engine_id,
        bool hover_z_from_hand)
    {
        HandMoveState hs;
        if (!enable_r && !enable_l)
            return hs;

        log_phase_banner("抓取流程：手相机识别前停稳");

        if (kDebugVisualize)
        {
            pose_vis_begin_phase(
                PoseVisLayout::DualHand,
                PoseVisPanel::Head,
                {PoseVisPanel::LeftHand, PoseVisPanel::RightHand});
        }

        // 手相机前：编码器到位检查（速度停稳暂关闭），超时 15s
        auto wait_steady_for_hand_cam = [&]() -> bool
        {
            return wait_arms_motors_stopped(enable_r, enable_l, 15.0, 6);
        };

        if (!wait_steady_for_hand_cam())
        {
            if (hardware_abort_requested())
                std::cerr << "[hand] STOP：停稳等待已中止\n";
            else
                std::cerr << "[hand] 电机未停稳，跳过手相机拍照\n";
            hs.steady_timed_out = true;
            return hs;
        }
        if (hardware_abort_requested())
        {
            hs.steady_timed_out = true;
            return hs;
        }

        log_phase_banner("抓取流程：手相机识别(class0)");

        hardware_abort_sleep_ms(1000);
        if (hardware_abort_requested())
        {
            hs.steady_timed_out = true;
            return hs;
        }

        // 停稳后先排空手相机缓冲，避免移动过程中积压旧帧
        if (enable_r)
            cameras.flush(CameraSlot::RightHand);
        if (enable_l)
            cameras.flush(CameraSlot::LeftHand);
        hardware_abort_sleep_ms(1000);
        if (hardware_abort_requested())
        {
            hs.steady_timed_out = true;
            return hs;
        }

        if (enable_r)
            cout << "检测右手" << arm_get_tcp_pos(arm_r) << endl;
        if (enable_l)
            cout << "检测左手" << arm_get_tcp_pos(arm_l) << endl;

        PoseDetectionRecords right_hand_records;
        PoseDetectionRecords left_hand_records;
        if (enable_r)
            right_hand_records =
                detect_pose_at_slot(
                    cameras, bridge, CameraSlot::RightHand, kDebugVisualize, kGraspDetectClassId, engine_id);
        if (enable_l)
        {
            // 拍右手期间左手 pipeline 可能又积帧，拍左前再 flush 一次
            cameras.flush(CameraSlot::LeftHand);
            left_hand_records =
                detect_pose_at_slot(
                    cameras, bridge, CameraSlot::LeftHand, kDebugVisualize, kGraspDetectClassId, engine_id);
        }

        std::array<double, 16> cam2robot_r{};
        std::array<double, 16> cam2robot_l{};
        Eigen::Matrix<double, 1, 6> hand_cam_r, hand_cam_l;
        hand_cam_r << -1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        hand_cam_l << -1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        Eigen::Matrix<double, 1, 6> base_pos_r, base_pos_l;

        const bool have_cam_r =
            !enable_r ||
            load_cam2robot_matrix(default_right_hand_to_robot_yaml_path(), cam2robot_r, err);
        if (enable_r && !have_cam_r)
            std::cerr << "读取右手 cam2robot 失败: " << err << std::endl;
        else if (enable_r)
        {
            assign_nearest_hand_cam_target_in_robot_frame(
                right_hand_records, cam2robot_r, hand_cam_r);
            base_pos_r = arm_get_base_visual_pos(arm_r, hand_cam_r);
        }

        const bool have_cam_l =
            !enable_l ||
            load_cam2robot_matrix(default_left_hand_to_robot_yaml_path(), cam2robot_l, err);
        if (enable_l && !have_cam_l)
            std::cerr << "读取左手 cam2robot 失败: " << err << std::endl;
        else if (enable_l)
        {
            assign_nearest_hand_cam_target_in_robot_frame(
                left_hand_records, cam2robot_l, hand_cam_l);
            base_pos_l = arm_get_base_visual_pos(arm_l, hand_cam_l);
        }

        apply_hand_base_to_goal(
            enable_r && have_cam_r,
            base_pos_r,
            enable_l && have_cam_l,
            base_pos_l,
            st.goal_last_r,
            st.goal_last_l,
            hover_z_from_hand);

        hs.move_r = enable_r && have_cam_r && hand_move_allowed_right(st.goal_last_r, base_pos_r);
        hs.move_l = enable_l && have_cam_l && hand_move_allowed_left(st.goal_last_l, base_pos_l);

        const int hand_redo_max = g_move_cfg.head_grasp.hand_detect_invalid_redo_max;
        for (int redo = 0; redo < hand_redo_max && !hs.move_r && enable_r && have_cam_r; ++redo)
        {
            if (hardware_abort_requested())
            {
                hs.steady_timed_out = true;
                return hs;
            }
            if (!wait_steady_for_hand_cam())
            {
                std::cerr << "[hand] 重拍前电机未停稳，跳过右手手相机\n";
                hs.steady_timed_out = true;
                return hs;
            }
            hardware_abort_sleep_ms(1000);
            if (hardware_abort_requested())
            {
                hs.steady_timed_out = true;
                return hs;
            }
            cout << "[hand] 右手无效，手相机重拍 " << (redo + 1) << "/" << hand_redo_max << endl;
            if (kDebugVisualize)
                pose_vis_clear_panels({PoseVisPanel::RightHand});
            right_hand_records =
                detect_pose_at_slot(
                    cameras, bridge, CameraSlot::RightHand, kDebugVisualize, kGraspDetectClassId, engine_id);
            assign_nearest_hand_cam_target_in_robot_frame(
                right_hand_records, cam2robot_r, hand_cam_r);
            base_pos_r = arm_get_base_visual_pos(arm_r, hand_cam_r);
            update_hand_goal_xy_from_base(
                base_pos_r,
                st.goal_last_r,
                g_move_cfg.head_grasp.hand_right.offset_x,
                g_move_cfg.head_grasp.hand_right.offset_y);
            if (hover_z_from_hand)
                apply_hand_hover_z_if_valid(base_pos_r, st.goal_last_r);
            hs.move_r = hand_move_allowed_right(st.goal_last_r, base_pos_r);
        }

        for (int redo = 0; redo < hand_redo_max && !hs.move_l && enable_l && have_cam_l; ++redo)
        {
            if (hardware_abort_requested())
            {
                hs.steady_timed_out = true;
                return hs;
            }
            if (!wait_steady_for_hand_cam())
            {
                std::cerr << "[hand] 重拍前电机未停稳，跳过左手手相机\n";
                hs.steady_timed_out = true;
                return hs;
            }
            hardware_abort_sleep_ms(1000);
            if (hardware_abort_requested())
            {
                hs.steady_timed_out = true;
                return hs;
            }
            cout << "[hand] 左手无效，手相机重拍 " << (redo + 1) << "/" << hand_redo_max << endl;
            if (kDebugVisualize)
                pose_vis_clear_panels({PoseVisPanel::LeftHand});
            left_hand_records =
                detect_pose_at_slot(
                    cameras, bridge, CameraSlot::LeftHand, kDebugVisualize, kGraspDetectClassId, engine_id);
            assign_nearest_hand_cam_target_in_robot_frame(
                left_hand_records, cam2robot_l, hand_cam_l);
            base_pos_l = arm_get_base_visual_pos(arm_l, hand_cam_l);
            update_hand_goal_xy_from_base(
                base_pos_l,
                st.goal_last_l,
                g_move_cfg.head_grasp.hand_left.offset_x,
                g_move_cfg.head_grasp.hand_left.offset_y);
            if (hover_z_from_hand)
                apply_hand_hover_z_if_valid(base_pos_l, st.goal_last_l);
            hs.move_l = hand_move_allowed_left(st.goal_last_l, base_pos_l);
        }

        if (enable_r)
        {
            cout << "hand_cam_r: " << hand_cam_r << endl;
            cout << "base_pos_r: " << base_pos_r << endl;
        }
        if (enable_l)
        {
            cout << "hand_cam_l: " << hand_cam_l << endl;
            cout << "base_pos_l: " << base_pos_l << endl;
        }

        return hs;
    }

    ArmLineMoveResult run_hand_approach_and_grasp(
        gripper::Gripper &g,
        Robot_Arm &arm_r,
        Robot_Arm &arm_l,
        HeadAssignState &st,
        const HandMoveState &hs)
    {
        log_phase_banner("抓取流程：手部下压夹取");
        ArmLineMoveResult out;
        // 1) 先到物体上方（手相机 xy，高度已是 hover 或头部分配 hover）
        {
            ArmLineMoveDebugStage stage("到物体上方(手相机)");
            out = arm_dual_line_move_selective(
                arm_r, st.goal_last_r, hs.move_r, arm_l, st.goal_last_l, hs.move_l, 0.2);
        }
        if (hs.move_r && out.ret_r != 0)
        {
            log_arm_traj_plan_fail("右手", out.ret_r);
            return out;
        }
        if (hs.move_l && out.ret_l != 0)
        {
            log_arm_traj_plan_fail("左手", out.ret_l);
            return out;
        }
        sleep(1);

        // 2) 再下压 z，然后夹取
        const double hand_desc = g_move_cfg.head_grasp.hand_descend_z;
        if (hs.move_r)
        {
            st.goal_last_r(2) -= hand_desc;
            clamp_cart_goal_z(st.goal_last_r, "run_hand_approach_and_grasp(右手, 手相机后下压)");
        }
        if (hs.move_l)
        {
            st.goal_last_l(2) -= hand_desc;
            clamp_cart_goal_z(st.goal_last_l, "run_hand_approach_and_grasp(左手, 手相机后下压)");
        }
        const ArmLineMoveResult out2 = [&]()
        {
            ArmLineMoveDebugStage stage("手部下压");
            return arm_dual_line_move_selective(
                arm_r, st.goal_last_r, hs.move_r, arm_l, st.goal_last_l, hs.move_l, 0.2);
        }();
        if (hs.move_r)
            out.ret_r = out2.ret_r;
        if (hs.move_l)
            out.ret_l = out2.ret_l;
        if (hs.move_r && out.ret_r != 0)
        {
            log_arm_traj_plan_fail("右手", out.ret_r);
            return out;
        }
        if (hs.move_l && out.ret_l != 0)
        {
            log_arm_traj_plan_fail("左手", out.ret_l);
            return out;
        }
        sleep(1);

        if (hs.move_r)
        {
            const auto grasp_r = g.graspAndWait(gripper::Side::Right);
            if (grasp_r.result != gripper::GraspResult::TorqueLimit &&
                grasp_r.result != gripper::GraspResult::PositionReached)
            {
                std::cerr << "[gripper] 右手夹取异常 result=" << static_cast<int>(grasp_r.result)
                          << " q=" << grasp_r.position_rad << "\n";
            }
        }
        if (hs.move_l)
        {
            const auto grasp_l = g.graspAndWait(gripper::Side::Left);
            if (grasp_l.result != gripper::GraspResult::TorqueLimit &&
                grasp_l.result != gripper::GraspResult::PositionReached)
            {
                std::cerr << "[gripper] 左手夹取异常 result=" << static_cast<int>(grasp_l.result)
                          << " q=" << grasp_l.position_rad << "\n";
            }
        }
        return out;
    }

    void lift_grasped_selective(
        Robot_Arm &arm_r,
        Robot_Arm &arm_l,
        RealSenseMultiCam &cameras,
        HeadAssignState &st,
        bool lift_r,
        bool lift_l)
    {
        if (!lift_r && !lift_l)
            return;
        const double lift_z = g_move_cfg.head_grasp.lift_after_grasp_z;
        if (lift_r)
            st.goal_last_r(2) += lift_z;
        if (lift_l)
            st.goal_last_l(2) += lift_z;
        {
            ArmLineMoveDebugStage stage("夹取后抬起");
            arm_dual_line_move_selective(
                arm_r, st.goal_last_r, lift_r, arm_l, st.goal_last_l, lift_l, 0.2);
        }
        sleep(1);
        cameras.save_grasp_hand_camera_snapshots(lift_r, lift_l);
    }

    /** 放货时腰 x 下限改走 yaml waist.x_min */

    double waist_place_clamp_x(double x)
    {
        const auto &w = g_move_cfg.waist;
        if (w.x_min >= w.x_max)
            return x;
        return std::min(std::max(x, w.x_min), w.x_max);
    }

    void waist_layer3_apply_x_delta(
        WaistRobot &waist,
        Eigen::Matrix<double, 1, 6> &posup_down_3_layer,
        double delta_x,
        const char *log_tag)
    {
        if (std::abs(delta_x) <= 1e-9)
            return;
        posup_down_3_layer(0) -= delta_x;
        if (clamp_waist_layer3_xyz(posup_down_3_layer))
            cout << log_tag << " 行程限位后 x=" << posup_down_3_layer(0)
                 << " z=" << posup_down_3_layer(2) << endl;
        else
            cout << log_tag << " delta=" << delta_x << " pos_x=" << posup_down_3_layer(0) << endl;
        waist.moveLToPos(posup_down_3_layer, 0.1);
        hardware_abort_sleep_ms(static_cast<int>(g_move_cfg.waist.move_settle_sec * 1000.0));
    }

    /** 放货排腰进：目标 x 夹到 yaml waist.x_min/x_max；返回实际 delta（供手臂 x 补偿） */
    double waist_layer3_apply_place_x_delta(
        WaistRobot &waist,
        Eigen::Matrix<double, 1, 6> &posup_down_3_layer,
        double delta_x,
        const char *log_tag)
    {
        if (std::abs(delta_x) <= 1e-9)
            return 0.0;
        const double cur_x = posup_down_3_layer(0);
        double new_x = waist_place_clamp_x(cur_x - delta_x);
        if (std::abs(new_x - cur_x) <= 1e-9)
            return 0.0;
        const double actual_delta = cur_x - new_x;
        posup_down_3_layer(0) = new_x;
        cout << log_tag << " req_delta=" << delta_x << " actual_delta=" << actual_delta
             << " pos_x=" << new_x;
        if (new_x <= g_move_cfg.waist.x_min + 1e-9 && cur_x - delta_x < g_move_cfg.waist.x_min - 1e-9)
            cout << " (限幅@x_min=" << g_move_cfg.waist.x_min << ")";
        cout << endl;
        waist.moveLToPos(posup_down_3_layer, 0.1);
        hardware_abort_sleep_ms(static_cast<int>(g_move_cfg.waist.move_settle_sec * 1000.0));
        return actual_delta;
    }

    /** 腰 x 动 delta 后未重拍时，手臂放货目标 x 补偿：goal_x += -delta */
    double waist_x_arm_goal_comp(double waist_delta_x)
    {
        return -waist_delta_x;
    }

    double waist_layer3_stagger_forward(
        WaistRobot &waist,
        Eigen::Matrix<double, 1, 6> &posup_down_3_layer,
        int multiplier,
        const char *log_tag)
    {
        if (multiplier <= 0)
            return 0.0;
        const double cur_x = posup_down_3_layer(0);
        const double delta = static_cast<double>(multiplier) * g_move_cfg.waist.stagger_step_x;
        std::ostringstream oss;
        oss << log_tag << " x×" << multiplier << " step=" << g_move_cfg.waist.stagger_step_x
            << " delta=" << delta;
        waist_layer3_apply_x_delta(waist, posup_down_3_layer, delta, oss.str().c_str());
        return cur_x - posup_down_3_layer(0);
    }

    void apply_grasp_stagger_waist_comp(HeadAssignState &st, double waist_actual_delta)
    {
        if (std::abs(waist_actual_delta) <= 1e-9)
            return;
        const double comp = waist_x_arm_goal_comp(waist_actual_delta);
        if (st.have_r)
            st.goal_last_r(0) += comp;
        if (st.have_l)
            st.goal_last_l(0) += comp;
        refresh_head_move_flags(st);
    }

    void waist_layer3_step_forward(WaistRobot &waist, Eigen::Matrix<double, 1, 6> &posup_down_3_layer, int step_no)
    {
        (void)step_no;
        waist_layer3_stagger_forward(waist, posup_down_3_layer, 1, "[waist] x 前进");
    }

    void restore_waist_layer3_x(WaistRobot &waist, Eigen::Matrix<double, 1, 6> &posup_down_3_layer)
    {
        const double home_x = g_move_cfg.waist.layer3_home(0);
        posup_down_3_layer(0) = home_x;
        cout << "[waist] 恢复 layer3 x=" << home_x << endl;
        waist.moveLToPos(posup_down_3_layer, 0.1);
        hardware_abort_sleep_ms(static_cast<int>(g_move_cfg.waist.move_settle_sec * 1000.0));
    }

    /** 放货预备 x：layer3_home.x 与 place_advance_x 合成目标（pos(0) -= place_advance_x） */
    double waist_layer3_place_ready_x()
    {
        return g_move_cfg.waist.layer3_home(0) - g_move_cfg.waist.place_advance_x;
    }

    /** 放货预备 x（含 yaml waist x 行程） */
    double waist_layer3_place_ready_x_effective()
    {
        return waist_place_clamp_x(waist_layer3_place_ready_x());
    }

    bool waist_layer3_at_place_ready_x(const Eigen::Matrix<double, 1, 6> &posup_down_3_layer)
    {
        return std::abs(posup_down_3_layer(0) - waist_layer3_place_ready_x_effective()) <= 1e-5;
    }

    void move_waist_to_place_ready_x(WaistRobot &waist, Eigen::Matrix<double, 1, 6> &posup_down_3_layer)
    {
        const double cfg_x = waist_layer3_place_ready_x();
        const double target_x = waist_layer3_place_ready_x_effective();
        if (waist_layer3_at_place_ready_x(posup_down_3_layer))
            return;
        posup_down_3_layer(0) = target_x;
        cout << "[waist] 放货预备 x=" << target_x
             << " (home_x=" << g_move_cfg.waist.layer3_home(0)
             << " place_advance_x=" << g_move_cfg.waist.place_advance_x << ")";
        if (std::abs(target_x - cfg_x) > 1e-9)
            cout << " cfg_x=" << cfg_x << " 限幅@x=[" << g_move_cfg.waist.x_min << ","
                 << g_move_cfg.waist.x_max << "]";
        cout << "\n";
        waist.moveLToPos(posup_down_3_layer, 0.1);
        hardware_abort_sleep_ms(static_cast<int>(g_move_cfg.waist.move_settle_sec * 1000.0));
    }

    void ensure_waist_layer3_x_home(WaistRobot &waist, Eigen::Matrix<double, 1, 6> &posup_down_3_layer)
    {
        if (std::abs(posup_down_3_layer(0) - g_move_cfg.waist.layer3_home(0)) <= 1e-5)
            return;
        cout << "[waist] layer3 x 不在 home，先恢复\n";
        restore_waist_layer3_x(waist, posup_down_3_layer);
    }

    bool stagger_target_side_graspable(const HeadAssignState &st, bool for_right)
    {
        return for_right ? (st.have_r && st.move_r) : (st.have_l && st.move_l);
    }

    bool stagger_target_side_x_strict_ok(const HeadAssignState &st, bool for_right)
    {
        if (!stagger_target_side_graspable(st, for_right))
            return false;
        const double x = for_right ? st.goal_last_r(0) : st.goal_last_l(0);
        return !head_x_needs_stagger(x);
    }

    bool stagger_waist_budget_exhausted(const GraspStaggerWaistState &sw)
    {
        return sw.steps_used >= std::max(1, g_move_cfg.waist.stagger_max_steps);
    }

    bool advance_waist_stagger_side(
        WaistRobot &waist,
        Eigen::Matrix<double, 1, 6> &posup_down_3_layer,
        bool for_right,
        HeadAssignState &st,
        GraspStaggerWaistState &stagger_waist)
    {
        const int max_steps = std::max(1, g_move_cfg.waist.stagger_max_steps);
        const char *hand = for_right ? "右手" : "左手";

        if (stagger_target_side_x_strict_ok(st, for_right))
            return true;
        if (!stagger_target_side_graspable(st, for_right))
            return false;

        if (stagger_waist_budget_exhausted(stagger_waist))
        {
            const double x = for_right ? st.goal_last_r(0) : st.goal_last_l(0);
            cout << "[stagger] " << hand << " 本轮腰进已用满 " << max_steps << " 步，x=" << x
                 << " 仍>" << g_move_cfg.stagger.head_x_threshold
                 << "，grasp_valid 合格，继续抓取\n";
            return true;
        }

        const int remaining = max_steps - stagger_waist.steps_used;
        const double x = for_right ? st.goal_last_r(0) : st.goal_last_l(0);
        const int mult = stagger_waist_steps_for_excess(x, remaining);
        if (mult <= 0)
            return true;

        cout << "[stagger] " << hand << " 目标x=" << x << " 一次性腰进×" << mult << " (本轮 "
             << stagger_waist.steps_used << "+" << mult << "/" << max_steps
             << "，头相机不重拍)\n";
        const double actual = waist_layer3_stagger_forward(
            waist, posup_down_3_layer, mult, "[waist] stagger抓取腰进");
        apply_grasp_stagger_waist_comp(st, actual);
        stagger_waist.steps_used += mult;

        if (stagger_target_side_x_strict_ok(st, for_right))
            return true;
        if (stagger_waist_budget_exhausted(stagger_waist))
        {
            cout << "[stagger] " << hand << " 腰进后本轮已达 " << max_steps
                 << " 步，grasp_valid 合格，继续抓取\n";
            return stagger_target_side_graspable(st, for_right);
        }
        return stagger_target_side_graspable(st, for_right);
    }

    bool advance_waist_stagger_dual_hands(
        WaistRobot &waist,
        Eigen::Matrix<double, 1, 6> &posup_down_3_layer,
        HeadAssignState &st,
        GraspStaggerWaistState &stagger_waist)
    {
        const int max_steps = std::max(1, g_move_cfg.waist.stagger_max_steps);

        auto dual_x_strict_ok = [&]() -> bool
        {
            if (st.move_r && head_x_needs_stagger(st.goal_last_r(0)))
                return false;
            if (st.move_l && head_x_needs_stagger(st.goal_last_l(0)))
                return false;
            return st.move_r || st.move_l;
        };
        auto dual_graspable = [&]() -> bool { return st.move_r || st.move_l; };

        if (dual_x_strict_ok())
            return true;
        if (!dual_graspable())
            return false;

        if (stagger_waist_budget_exhausted(stagger_waist))
        {
            cout << "[stagger] 双手本轮腰进已用满 " << max_steps
                 << " 步，grasp_valid 合格，继续抓取\n";
            return true;
        }

        double target_x = 0.0;
        if (st.move_r && head_x_needs_stagger(st.goal_last_r(0)))
            target_x = std::max(target_x, st.goal_last_r(0));
        if (st.move_l && head_x_needs_stagger(st.goal_last_l(0)))
            target_x = std::max(target_x, st.goal_last_l(0));

        const int remaining = max_steps - stagger_waist.steps_used;
        const int mult = stagger_waist_steps_for_excess(target_x, remaining);
        if (mult <= 0)
            return true;

        cout << "[stagger] 双手最远x=" << target_x << " 一次性腰进×" << mult << " (本轮 "
             << stagger_waist.steps_used << "+" << mult << "/" << max_steps
             << "，头相机不重拍)\n";
        const double actual = waist_layer3_stagger_forward(
            waist, posup_down_3_layer, mult, "[waist] stagger双手腰进");
        apply_grasp_stagger_waist_comp(st, actual);
        stagger_waist.steps_used += mult;

        if (dual_x_strict_ok())
            return true;
        if (stagger_waist_budget_exhausted(stagger_waist))
        {
            cout << "[stagger] 双手腰进后本轮已达 " << max_steps << " 步，继续抓取\n";
            return dual_graspable();
        }
        return dual_graspable();
    }

    void apply_place_goal_xy_left(Eigen::Matrix<double, 1, 6> &goal, double x, double y, int row_0)
    {
        const double base_z = goal(2);
        const auto &p = place_hand_xy_offset(row_0, false);
        goal(0) = x + p.offset_x;
        goal(1) = y + p.offset_y;
        goal(2) = base_z + g_move_cfg.place.z_raise;
    }

    void apply_place_goal_xy_right(Eigen::Matrix<double, 1, 6> &goal, double x, double y, int row_0)
    {
        const double base_z = goal(2);
        const auto &p = place_hand_xy_offset(row_0, true);
        goal(0) = x + p.offset_x;
        goal(1) = y + p.offset_y;
        goal(2) = base_z + g_move_cfg.place.z_raise;
    }

    std::string pose6_to_csv(const Eigen::Matrix<double, 1, 6> &p)
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6);
        for (int i = 0; i < 6; ++i)
        {
            if (i > 0)
                oss << ',';
            oss << p(i);
        }
        return oss.str();
    }

    constexpr const char *kPlacePoseDebugSep =
        "================================================================================\n";

    std::string place_pose_debug_file_path()
    {
        return project_root_dir() + "/picture_debug/place_pose.txt";
    }

    /** 松爪前：仅记录本轮实际要放的手（do && grasped），写入 picture_debug/place_pose.txt */
    void append_place_pose_before_release_debug(
        Robot_Arm &arm_r,
        Robot_Arm &arm_l,
        const HeadAssignState &st,
        bool do_r,
        bool do_l,
        bool grasped_r,
        bool grasped_l,
        const char *tag,
        const std::vector<Eigen::Matrix<double, 1, 6>> *vision_grid,
        std::optional<size_t> slot_r,
        std::optional<size_t> slot_l)
    {
        if (!g_move_cfg.place.place_pose_debug)
            return;

        const bool save_r = do_r && grasped_r;
        const bool save_l = do_l && grasped_l;
        if (!save_r && !save_l)
            return;

        auto vision_pose_for_slot = [&](std::optional<size_t> slot) -> std::optional<Eigen::Matrix<double, 1, 6>>
        {
            if (!vision_grid || !slot || *slot >= vision_grid->size())
                return std::nullopt;
            return (*vision_grid)[*slot];
        };

        const std::filesystem::path file_path = place_pose_debug_file_path();
        std::error_code ec;
        std::filesystem::create_directories(file_path.parent_path(), ec);
        if (ec)
        {
            std::cerr << "[place_pose_debug] 创建目录失败: " << ec.message() << std::endl;
            return;
        }

        std::ofstream out(file_path, std::ios::app);
        if (!out)
        {
            std::cerr << "[place_pose_debug] 无法写入 " << file_path.string() << std::endl;
            return;
        }

        const auto now = std::chrono::system_clock::now();
        const std::time_t sec = std::chrono::system_clock::to_time_t(now);
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) %
                        1000;
        std::tm tm_local{};
        localtime_r(&sec, &tm_local);

        out << kPlacePoseDebugSep;
        out << "[place_pose_debug] step=" << (tag ? tag : "") << "\n";
        out << "time=" << std::put_time(&tm_local, "%Y-%m-%d %H:%M:%S") << '.'
            << std::setw(3) << std::setfill('0') << ms.count() << "\n";
        out << "format: x,y,z(m) rx,ry,rz(rad)\n";
        out << "vision_base=头相机 class1 检测位姿 cam2robot 转基座系(未加 place offset/z_raise/腰 x 补偿)\n";

        if (save_r)
        {
            const Eigen::Matrix<double, 1, 6> actual_r = arm_get_tcp_pos(arm_r);
            const Eigen::Matrix<double, 1, 6> err_r = actual_r - st.goal_last_r;
            if (const auto vis = vision_pose_for_slot(slot_r))
                out << "right vision_base=" << pose6_to_csv(*vis) << "\n";
            out << "right cmd=" << pose6_to_csv(st.goal_last_r) << "\n";
            out << "right act=" << pose6_to_csv(actual_r) << "\n";
            out << "right err=" << pose6_to_csv(err_r) << "\n";
        }
        if (save_l)
        {
            const Eigen::Matrix<double, 1, 6> actual_l = arm_get_tcp_pos(arm_l);
            const Eigen::Matrix<double, 1, 6> err_l = actual_l - st.goal_last_l;
            if (const auto vis = vision_pose_for_slot(slot_l))
                out << "left vision_base=" << pose6_to_csv(*vis) << "\n";
            out << "left cmd=" << pose6_to_csv(st.goal_last_l) << "\n";
            out << "left act=" << pose6_to_csv(actual_l) << "\n";
            out << "left err=" << pose6_to_csv(err_l) << "\n";
        }
        out << kPlacePoseDebugSep << "\n";
        out.flush();

        std::cout << "[place_pose_debug] 已写入 " << file_path.string() << std::endl;
    }

    constexpr const char kPlaceCmdYellow[] = "\033[33m";
    constexpr const char kPlaceCmdReset[] = "\033[0m";

    /** 放货实际下发 1×6（黄字）：x,y,z(m) rx,ry,rz(rad) */
    void log_place_motion_cmd(
        const char *step,
        bool move_r,
        bool move_l,
        const Eigen::Matrix<double, 1, 6> &goal_r,
        const Eigen::Matrix<double, 1, 6> &goal_l)
    {
        const char *label = (step != nullptr && step[0] != '\0') ? step : "放货";
        cout << kLogPhaseSep << '\n';
        cout << ">>> [放货下发] " << label << " <<<\n";
        if (move_r)
        {
            cout << kPlaceCmdYellow << "[place][下发] 右手 x,y,z,rx,ry,rz="
                 << pose6_to_csv(goal_r) << kPlaceCmdReset << '\n';
        }
        if (move_l)
        {
            cout << kPlaceCmdYellow << "[place][下发] 左手 x,y,z,rx,ry,rz="
                 << pose6_to_csv(goal_l) << kPlaceCmdReset << '\n';
        }
        cout << kLogPhaseSep << '\n';
    }

    void apply_place_fallback_goal_right(Eigen::Matrix<double, 1, 6> &goal)
    {
        goal = g_move_cfg.place.fallback_right;
    }

    void apply_place_fallback_goal_left(Eigen::Matrix<double, 1, 6> &goal)
    {
        goal = g_move_cfg.place.fallback_left;
    }

    /** 单侧直线失败则改 fallback 位再试；仍失败则 move 侧置 false */
    void place_retry_side_with_fallback(
        Robot_Arm &arm_r,
        Robot_Arm &arm_l,
        HeadAssignState &st,
        bool &move_r,
        bool &move_l,
        bool &used_fallback_r,
        bool &used_fallback_l,
        int failed_ret_r,
        int failed_ret_l,
        double cart_vel = 0.2)
    {
        if (move_r && failed_ret_r != 0)
        {
            log_arm_traj_plan_fail("右手", failed_ret_r);
            cout << "[place] 右手改放 fallback 位\n";
            apply_place_fallback_goal_right(st.goal_last_r);
            log_place_motion_cmd("放货fallback(右手)", true, false, st.goal_last_r, st.goal_last_l);
            const int ret = [&]()
            {
                ArmLineMoveDebugStage stage("放货fallback(右手)");
                return arm_line_move(arm_r, st.goal_last_r, cart_vel);
            }();
            if (ret != 0)
            {
                log_arm_traj_plan_fail("右手(fallback)", ret);
                move_r = false;
            }
            else
            {
                used_fallback_r = true;
            }
        }
        if (move_l && failed_ret_l != 0)
        {
            log_arm_traj_plan_fail("左手", failed_ret_l);
            cout << "[place] 左手改放 fallback 位\n";
            apply_place_fallback_goal_left(st.goal_last_l);
            log_place_motion_cmd("放货fallback(左手)", false, true, st.goal_last_r, st.goal_last_l);
            const int ret = [&]()
            {
                ArmLineMoveDebugStage stage("放货fallback(左手)");
                return arm_line_move(arm_l, st.goal_last_l, cart_vel);
            }();
            if (ret != 0)
            {
                log_arm_traj_plan_fail("左手(fallback)", ret);
                move_l = false;
            }
            else
            {
                used_fallback_l = true;
            }
        }
    }

    ArmLineMoveResult place_line_move_logged(
        Robot_Arm &arm_r,
        Robot_Arm &arm_l,
        HeadAssignState &st,
        bool move_r,
        bool move_l,
        const char *step,
        double cart_vel = 0.2)
    {
        if (!move_r && !move_l)
            return {};
        std::string step_label = "放货-抬高位";
        if (step != nullptr && step[0] != '\0')
            step_label += std::string(" | ") + step;
        log_place_motion_cmd(step_label.c_str(), move_r, move_l, st.goal_last_r, st.goal_last_l);
        ArmLineMoveDebugStage stage(step);
        ArmLineMoveResult out = arm_dual_line_move_selective(
            arm_r, st.goal_last_r, move_r, arm_l, st.goal_last_l, move_l, cart_vel);
        sleep(1);
        return out;
    }

    /** 放货直线段：失败则 fallback；返回仍有效的 move 侧 */
    void place_line_move_with_fallback(
        Robot_Arm &arm_r,
        Robot_Arm &arm_l,
        HeadAssignState &st,
        bool &move_r,
        bool &move_l,
        bool &used_fallback_r,
        bool &used_fallback_l,
        const char *step,
        double cart_vel = 0.2)
    {
        const ArmLineMoveResult r =
            place_line_move_logged(arm_r, arm_l, st, move_r, move_l, step, cart_vel);
        place_retry_side_with_fallback(
            arm_r,
            arm_l,
            st,
            move_r,
            move_l,
            used_fallback_r,
            used_fallback_l,
            move_r ? r.ret_r : 0,
            move_l ? r.ret_l : 0,
            cart_vel);
    }

    /** 到放货抬高位后，再 z_descend（如 -0.02 下降 2cm）；为 0 则跳过 */
    void place_descend_before_release(
        Robot_Arm &arm_r,
        Robot_Arm &arm_l,
        HeadAssignState &st,
        bool &move_r,
        bool &move_l,
        bool &used_fallback_r,
        bool &used_fallback_l)
    {
        const double dz = g_move_cfg.place.z_descend;
        if (std::abs(dz) <= 1e-9 || (!move_r && !move_l))
            return;
        if (move_r)
            st.goal_last_r(2) += dz;
        if (move_l)
            st.goal_last_l(2) += dz;
        log_place_motion_cmd("放货-下压(松爪前)", move_r, move_l, st.goal_last_r, st.goal_last_l);
        const ArmLineMoveResult r = [&]()
        {
            ArmLineMoveDebugStage stage("放货z下压");
            return arm_dual_line_move_selective(
                arm_r, st.goal_last_r, move_r, arm_l, st.goal_last_l, move_l, 0.2);
        }();
        place_retry_side_with_fallback(
            arm_r,
            arm_l,
            st,
            move_r,
            move_l,
            used_fallback_r,
            used_fallback_l,
            move_r ? r.ret_r : 0,
            move_l ? r.ret_l : 0,
            0.2);
        sleep(1);
    }

    /** 松爪后沿 z 升回下压前高度（撤销 z_descend） */
    void place_ascend_after_release(
        Robot_Arm &arm_r,
        Robot_Arm &arm_l,
        HeadAssignState &st,
        bool move_r,
        bool move_l)
    {
        const double dz = g_move_cfg.place.z_descend;
        if (std::abs(dz) <= 1e-9 || (!move_r && !move_l))
            return;
        if (move_r)
            st.goal_last_r(2) -= dz;
        if (move_l)
            st.goal_last_l(2) -= dz;
        log_place_motion_cmd("放货z回升", move_r, move_l, st.goal_last_r, st.goal_last_l);
        const ArmLineMoveResult r = [&]()
        {
            ArmLineMoveDebugStage stage("放货z回升");
            return arm_dual_line_move_selective(
                arm_r, st.goal_last_r, move_r, arm_l, st.goal_last_l, move_l, 0.2);
        }();
        if (move_r && r.ret_r != 0)
            log_arm_traj_plan_fail("右手", r.ret_r);
        if (move_l && r.ret_l != 0)
            log_arm_traj_plan_fail("左手", r.ret_l);
        sleep(1);
    }

    double configured_row_waist_x(int row_0)
    {
        if (row_0 < 0)
            return 0.0;
        const auto &rows = g_move_cfg.place.row_waist_x;
        if (row_0 >= static_cast<int>(rows.size()))
            return 0.0;
        return rows[static_cast<size_t>(row_0)];
    }

    /** 在 indices 中选距参考点 (ref_x, ref_y) xy 平面最近的格点 */
    std::optional<size_t> pick_nearest_pose_index(
        const std::vector<Eigen::Matrix<double, 1, 6>> &poses,
        const std::vector<size_t> &indices,
        double ref_x,
        double ref_y)
    {
        if (indices.empty())
            return std::nullopt;
        size_t best = indices.front();
        double best_d2 = std::numeric_limits<double>::max();
        for (size_t idx : indices)
        {
            if (idx >= poses.size())
                continue;
            const double dx = poses[idx](0) - ref_x;
            const double dy = poses[idx](1) - ref_y;
            const double d2 = dx * dx + dy * dy;
            if (d2 < best_d2)
            {
                best_d2 = d2;
                best = idx;
            }
        }
        return best;
    }

    /** 放货选点：距机器人基座原点 (0,0) xy 最近 */
    std::optional<size_t> pick_nearest_pose_to_base_index(
        const std::vector<Eigen::Matrix<double, 1, 6>> &poses,
        const std::vector<size_t> &indices)
    {
        return pick_nearest_pose_index(poses, indices, 0.0, 0.0);
    }

    bool is_row_enabled_for_place(int row_0)
    {
        if (row_0 < 0)
            return false;
        const auto &enabled = g_move_cfg.place.row_enabled;
        if (row_0 >= static_cast<int>(enabled.size()))
            return true;
        return enabled[static_cast<size_t>(row_0)] != 0;
    }

    /** 放货选点：按 row_enabled / row_place_from_front，同排内距 base 最近；排号来自棋盘或 row_x_bounds 回退 */
    std::optional<size_t> pick_place_slot_row_ordered(
        const std::vector<Eigen::Matrix<double, 1, 6>> &poses,
        const std::vector<size_t> &indices,
        const std::vector<int> *row_for_pose_index = nullptr)
    {
        if (indices.empty())
            return std::nullopt;

        std::vector<size_t> row_order;
        row_order.reserve(g_move_cfg.place.row_x_bounds.size());
        for (size_t r = 0; r < g_move_cfg.place.row_x_bounds.size(); ++r)
        {
            if (is_row_enabled_for_place(static_cast<int>(r)))
                row_order.push_back(r);
        }
        if (row_order.empty())
            return std::nullopt;

        if (g_move_cfg.place.row_place_from_front == 0)
            std::reverse(row_order.begin(), row_order.end());

        for (size_t r : row_order)
        {
            std::vector<size_t> row_candidates;
            row_candidates.reserve(indices.size());
            for (size_t idx : indices)
            {
                if (idx >= poses.size())
                    continue;
                int row_0 = -1;
                if (row_for_pose_index && idx < row_for_pose_index->size())
                    row_0 = (*row_for_pose_index)[idx];
                else
                    row_0 = place_pose_row_from_x(poses[idx](0));
                if (row_0 == static_cast<int>(r))
                    row_candidates.push_back(idx);
            }
            if (!row_candidates.empty())
                return pick_nearest_pose_index(poses, row_candidates, 0.0, 0.0);
        }
        return std::nullopt;
    }

    void erase_pose_index(std::vector<size_t> &indices, size_t idx)
    {
        indices.erase(std::remove(indices.begin(), indices.end(), idx), indices.end());
    }

    void consume_place_slot_from_lists(
        std::vector<size_t> &left_idx,
        std::vector<size_t> &right_idx,
        std::vector<size_t> &middle_idx,
        std::optional<size_t> slot_l,
        std::optional<size_t> slot_r)
    {
        if (slot_l)
        {
            erase_pose_index(left_idx, *slot_l);
            erase_pose_index(middle_idx, *slot_l);
        }
        if (slot_r)
        {
            erase_pose_index(right_idx, *slot_r);
            erase_pose_index(middle_idx, *slot_r);
        }
    }

    void lift_grasped_after_pick(
        Robot_Arm &arm_r,
        Robot_Arm &arm_l,
        RealSenseMultiCam &cameras,
        HeadAssignState &st,
        bool grasped_r,
        bool grasped_l,
        bool lifted_r,
        bool lifted_l)
    {
        const bool move_r = grasped_r && !lifted_r;
        const bool move_l = grasped_l && !lifted_l;
        if (!move_r && !move_l)
            return;
        const double lift_z = g_move_cfg.head_grasp.lift_after_grasp_z;
        if (move_r)
            st.goal_last_r(2) += lift_z;
        if (move_l)
            st.goal_last_l(2) += lift_z;
        {
            ArmLineMoveDebugStage stage("夹取后抬起");
            arm_dual_line_move_selective(
                arm_r, st.goal_last_r, move_r, arm_l, st.goal_last_l, move_l, 0.2);
        }
        sleep(1);
        cameras.save_grasp_hand_camera_snapshots(move_r, move_l);
    }

    PlaceToTrayResult run_place_holding_to_tray(
        gripper::Gripper &g,
        Robot_Arm &arm_r,
        Robot_Arm &arm_l,
        RealSenseMultiCam &cameras,
        SegPoseBridge &bridge,
        const std::array<double, 16> &cam2robot,
        bool holding_r,
        bool holding_l,
        std::string &err,
        const std::function<bool()> &should_abort)
    {
        PlaceToTrayResult out;
        auto aborted = [&]() {
            return should_abort && should_abort();
        };
        if (aborted())
        {
            out.status = PlaceToTrayStatus::Aborted;
            err = "已中止";
            return out;
        }
        if (!holding_r && !holding_l)
        {
            err = "没有持料手";
            std::cerr << "[place] " << err << "\n";
            return out;
        }

        log_phase_banner("放货(编排台)：头相机空位识别(class1，4类模型)");
        const PlaceHoleDetectResult holes = detect_place_holes_with_consensus(
            bridge,
            cameras,
            cam2robot,
            g_move_cfg.place.detect_trials,
            0.02,
            0.05,
            kDebugVisualize);
        if (aborted())
        {
            out.status = PlaceToTrayStatus::Aborted;
            err = "已中止";
            return out;
        }
        if (!holes.ok || holes.poses_robot.empty())
        {
            err = holes.message.empty() ? "未检测到料盘空位(class1)" : holes.message;
            std::cerr << "[place] " << err << "\n";
            return out;
        }
        out.holes_found = static_cast<int>(holes.poses_robot.size());
        cout << "[place] " << holes.message << "\n";

        const std::vector<Eigen::Matrix<double, 1, 6>> &grid = holes.poses_robot;
        std::vector<size_t> left_idx;
        std::vector<size_t> right_idx;
        std::vector<size_t> middle_idx;
        left_idx.reserve(grid.size());
        right_idx.reserve(grid.size());
        middle_idx.reserve(grid.size());
        const double y_split = g_move_cfg.place_zone.y_side_split;
        const double y_mid_lo = g_move_cfg.place_zone.y_right;
        const double y_mid_hi = g_move_cfg.place_zone.y_left;
        const bool skip_row6 = g_move_cfg.place.row6_bend.enabled;
        int skipped_row6 = 0;
        for (size_t i = 0; i < grid.size(); ++i)
        {
            const int row = place_pose_row_from_x(grid[i](0));
            if (row < 0 || !is_row_enabled_for_place(row))
                continue;
            if (skip_row6 && row == 5)
            {
                ++skipped_row6;
                continue;
            }
            const double y = grid[i](1);
            if (y > y_split)
                left_idx.push_back(i);
            if (y < y_split)
                right_idx.push_back(i);
            if (y >= y_mid_lo && y <= y_mid_hi)
                middle_idx.push_back(i);
        }
        if (skipped_row6 > 0)
            cout << "[place] 编排台跳过第6排空位 " << skipped_row6 << " 个（无弯腰路径）\n";
        cout << "[place] 空位分区: 左=" << left_idx.size()
             << " 右=" << right_idx.size() << " 中=" << middle_idx.size()
             << " 持料 R=" << holding_r << " L=" << holding_l << "\n";

        std::optional<size_t> pick_l;
        std::optional<size_t> pick_r;
        if (holding_l)
            pick_l = pick_place_slot_row_ordered(grid, left_idx, nullptr);
        if (holding_r)
            pick_r = pick_place_slot_row_ordered(grid, right_idx, nullptr);
        if (holding_l && !pick_l)
        {
            auto mid = middle_idx;
            if (pick_r)
                erase_pose_index(mid, *pick_r);
            pick_l = pick_place_slot_row_ordered(grid, mid, nullptr);
            if (pick_l)
                cout << "[place] 左手改用中间带空位 idx=" << *pick_l << "\n";
        }
        if (holding_r && !pick_r)
        {
            auto mid = middle_idx;
            if (pick_l)
                erase_pose_index(mid, *pick_l);
            pick_r = pick_place_slot_row_ordered(grid, mid, nullptr);
            if (pick_r)
                cout << "[place] 右手改用中间带空位 idx=" << *pick_r << "\n";
        }
        if (!pick_l && !pick_r)
        {
            err = "持料手对应分区没有可用空位";
            std::cerr << "[place] " << err << "\n";
            return out;
        }

        HeadAssignState st;
        st.goal_last_r = arm_get_tcp_pos(arm_r);
        st.goal_last_l = arm_get_tcp_pos(arm_l);
        st.have_r = holding_r;
        st.have_l = holding_l;
        bool move_r = false;
        bool move_l = false;
        if (pick_l)
        {
            const auto &p = grid[*pick_l];
            const int row = place_pose_row_from_x(p(0));
            st.goal_last_l(2) = p(2);
            apply_place_goal_xy_left(st.goal_last_l, p(0), p(1), row);
            move_l = true;
            cout << "[place] 左手空位 idx=" << *pick_l << " xy=(" << p(0) << "," << p(1)
                 << ") z=" << p(2) << " 排=" << (row + 1) << "\n";
        }
        else if (holding_l)
            cout << "[place] 左手无空位，本轮不放\n";
        if (pick_r)
        {
            const auto &p = grid[*pick_r];
            const int row = place_pose_row_from_x(p(0));
            st.goal_last_r(2) = p(2);
            apply_place_goal_xy_right(st.goal_last_r, p(0), p(1), row);
            move_r = true;
            cout << "[place] 右手空位 idx=" << *pick_r << " xy=(" << p(0) << "," << p(1)
                 << ") z=" << p(2) << " 排=" << (row + 1) << "\n";
        }
        else if (holding_r)
            cout << "[place] 右手无空位，本轮不放\n";

        if (aborted())
        {
            out.status = PlaceToTrayStatus::Aborted;
            err = "已中止";
            return out;
        }

        log_phase_banner("放货(编排台)：抬高 → 下压 → 松爪 → 回升");
        bool used_fallback_r = false;
        bool used_fallback_l = false;
        place_line_move_with_fallback(
            arm_r, arm_l, st, move_r, move_l, used_fallback_r, used_fallback_l, "料盘空位");
        if (!move_r && !move_l)
        {
            err = "放货直线失败";
            out.status = PlaceToTrayStatus::MoveFail;
            return out;
        }
        place_descend_before_release(
            arm_r, arm_l, st, move_r, move_l, used_fallback_r, used_fallback_l);
        if (!move_r && !move_l)
        {
            err = "放货下压失败";
            out.status = PlaceToTrayStatus::MoveFail;
            return out;
        }
        if (aborted())
        {
            out.status = PlaceToTrayStatus::Aborted;
            err = "已中止";
            return out;
        }

        const bool open_r = move_r && holding_r;
        const bool open_l = move_l && holding_l;
        if (!wait_arms_motors_stopped(open_r, open_l, 15.0, 6, "放货松爪前"))
        {
            err = "松爪前编码器未到位";
            out.status = PlaceToTrayStatus::MoveFail;
            return out;
        }
        append_place_pose_before_release_debug(
            arm_r, arm_l, st, move_r, move_l, holding_r, holding_l, "编排台料盘放货",
            &grid, pick_r, pick_l);
        if (open_l)
        {
            g.openAndWait(gripper::Side::Left);
            out.released_l = true;
        }
        if (open_r)
        {
            g.openAndWait(gripper::Side::Right);
            out.released_r = true;
        }
        place_ascend_after_release(arm_r, arm_l, st, move_r, move_l);
        cout << "[place] 放完回待机\n";
        move_arms_to_standby(arm_r, arm_l);
        out.status = PlaceToTrayStatus::Ok;
        return out;
    }

#ifndef ORCH_NO_CHASSIS
    /** 去 fang1 执行两次拓扑，补偿偶发走不准 */
    void move_topo_fang1_twice(LanxinControl &ctrl)
    {
        move_topo_until_ok(ctrl, "fang1", 0);
        move_topo_until_ok(ctrl, "fang1", 0);
    }

    /** 底盘旋转：失败重试最多 10 次，仍失败则退出程序 */
    void chassis_rotate_or_exit(LanxinControl &ctrl, double deg, const char *ctx)
    {
        constexpr int kMaxRotateAttempts = 10;
        if (ctrl.RotateInPlaceRetry(deg, kMaxRotateAttempts, 2))
            return;
        std::cerr << "[chassis] " << ctx << " RotateInPlace(" << deg << ") 连续 "
                  << kMaxRotateAttempts << " 次失败，退出程序\n";
        aruco_test_exit(1);
    }

    /** 底盘旋转时并行将腰移到放货预备位（一次到位，不再先回 home 再 advance） */
    void chassis_rotate_with_waist_place_ready(
        LanxinControl &ctrl,
        WaistRobot &waist,
        Eigen::Matrix<double, 1, 6> &posup_down_3_layer,
        double deg,
        const char *ctx)
    {
        const bool need_waist = !waist_layer3_at_place_ready_x(posup_down_3_layer);
        const int waist_delay_sec = std::max(0, g_move_cfg.waist.place_ready_waist_delay_sec);
        const int chassis_delay_sec = std::max(0, g_move_cfg.waist.place_ready_chassis_delay_sec);
        std::thread waist_th;
        if (need_waist)
        {
            cout << "[place] 底盘旋转与腰放货预备并行"
                 << " (腰延迟" << waist_delay_sec << "s, 底盘延迟" << chassis_delay_sec << "s)\n";
            waist_th = std::thread([&]()
                                   {
                                       if (waist_delay_sec > 0)
                                       {
                                           cout << "[place] 腰放货预备 " << waist_delay_sec << "s 后启动\n";
                                           sleep(static_cast<unsigned>(waist_delay_sec));
                                       }
                                       move_waist_to_place_ready_x(waist, posup_down_3_layer); });
        }
        if (chassis_delay_sec > 0)
        {
            cout << "[place] 底盘旋转 " << chassis_delay_sec << "s 后启动\n";
            sleep(static_cast<unsigned>(chassis_delay_sec));
        }
        chassis_rotate_or_exit(ctrl, deg, ctx);
        if (need_waist)
            waist_th.join();
        else
            sleep(1);
    }

    /** 抓取前底盘：仅首次/充电回场时拓扑 fang1；已在工位则只依赖放货后的 -90°，不再重复导航 */
    bool prepare_chassis_for_grasp(
        LanxinControl &ctrl,
        WaistRobot &waist,
        Eigen::Matrix<double, 1, 6> &posup_down_3_layer,
        bool &need_topo_fang1)
    {
        if (!need_topo_fang1)
            return true;

        std::cout << "[chassis] 拓扑去 fang1（首次到位或充电返回）\n";
        move_topo_fang1_twice(ctrl);
        need_topo_fang1 = false;

        chassis_rotate_or_exit(ctrl, -90, "抓取前");
        sleep(1);
        // 首次/充电回场：软件 pos 已是 home 但拓扑+转底盘后需再发一次腰到基准位
        cout << "[waist] 到达 fang1 并转底盘后，腰回到 layer3 home\n";
        restore_waist_layer3_x(waist, posup_down_3_layer);
        return true;
    }

    /** 双手空闲时查电：低于配置阈值去充电，充至高于 full 停充；充电后标记需重新拓扑 fang1 */
    bool ensure_charged_and_at_work_site(LanxinControl &ctrl, bool &need_topo_fang1)
    {
        const double low_th = g_move_cfg.battery.low_threshold;
        const double full_th = g_move_cfg.battery.full_threshold;

        double charge_num = getBatteryLevel(ctrl);
        if (charge_num < 0)
        {
            std::cerr << "[battery] 读取电量失败\n";
            return false;
        }
        if (charge_num >= low_th)
            return true;

        std::cout << "[battery] 电量 " << charge_num << " < " << low_th << "，去充电\n";
        if (chargeInMobileZone(ctrl) != 0)
        {
            std::cerr << "[battery] 启动充电失败\n";
            return false;
        }

        while (true)
        {
            charge_num = getBatteryLevel(ctrl);
            if (charge_num < 0)
                continue;
            std::cout << "[battery] 充电中 电量=" << charge_num << std::endl;
            if (charge_num > full_th)
            {
                if (cancelCharging(ctrl) != 0)
                {
                    std::cerr << "[battery] 退出充电失败\n";
                    return false;
                }
                break;
            }
        }

        std::cout << "[battery] 充电完成(>" << full_th << ")，已停充回 c01，下轮重新拓扑 fang1\n";
        need_topo_fang1 = true;
        return true;
    }

    // ---------- 第6排弯腰放货（生产流程，参数见 place.row6_bend）----------

    /** 第六排腰是否已弯：true=已弯不再叠弯，回正后置 false */
    bool g_row6_waist_bent = false;

    const MoveBoxRow6BendConfig &row6_bend_cfg()
    {
        return g_move_cfg.place.row6_bend;
    }

    bool row6_bend_enabled_for_row(int row_0)
    {
        const auto &c = row6_bend_cfg();
        return c.enabled && row_0 == c.row_index_0 && is_row_enabled_for_place(row_0);
    }

    double row6_quintic_interp(double t, double total_time, double start, double target)
    {
        if (total_time <= 1e-6)
            return target;
        const double ratio = std::clamp(t / total_time, 0.0, 1.0);
        const double r2 = ratio * ratio;
        const double r3 = r2 * ratio;
        const double r4 = r3 * ratio;
        const double r5 = r4 * ratio;
        return start + (target - start) * (10.0 * r3 - 15.0 * r4 + 6.0 * r5);
    }

    bool row6_read_waist_pitch_cnt(int &out_cnt)
    {
        int data[1] = {0};
        uint32_t id = row6_bend_cfg().waist_pitch_motor_can_id;
        if (socketcan_sendsimplecommand(waist_id, 1, &id, 8, data) != 1)
            return false;
        out_cnt = data[0];
        return true;
    }

    void row6_send_waist_pitch_cnt(int cnt)
    {
        int data[1] = {cnt};
        uint32_t id = row6_bend_cfg().waist_pitch_motor_can_id;
        socketcan_sendcommand(waist_id, 1, &id, 30, data);
    }

    bool row6_wait_waist_pitch_reached(int target_cnt, double timeout_sec, const char *tag)
    {
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                  std::chrono::duration<double>(timeout_sec));
        while (std::chrono::steady_clock::now() < deadline)
        {
            int actual = 0;
            if (!row6_read_waist_pitch_cnt(actual))
                return false;
            if (std::abs(actual - target_cnt) <= 550)
                return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        std::cerr << "[row6] " << tag << " 腰pitch超时 target=" << target_cnt << "\n";
        return false;
    }

    bool row6_smooth_shoulder_only(double delta_deg)
    {
        const auto &c = row6_bend_cfg();
        if (std::abs(delta_deg) < 0.01)
            return true;

        int pos_r[7] = {0};
        int pos_l[7] = {0};
        get_motor_position(pos_r, 1);
        get_motor_position(pos_l, 0);

        const double r0 = motor_cnt_to_angle_deg(pos_r[c.right_shoulder_joint_index]);
        const double r1 = r0 + delta_deg;
        const double l0 = motor_cnt_to_angle_deg(pos_l[c.left_shoulder_joint_index]);
        const double l1 = l0 - delta_deg;

        std::cout << "[row6] 肩 右" << r0 << "°→" << r1 << "° 左" << l0 << "°→" << l1 << "° Δ=" << delta_deg
                  << "°\n";

        pos_r[c.right_shoulder_joint_index] = motor_angle_deg_to_cnt(r1);
        pos_l[c.left_shoulder_joint_index] = motor_angle_deg_to_cnt(l1);

        const double total = std::abs(delta_deg) / c.speed_deg_per_s;
        const double dt_s = static_cast<double>(c.smooth_dt_ms) / 1000.0;
        double t = 0.0;
        while (t < total)
        {
            const double rd = row6_quintic_interp(t, total, r0, r1);
            const double ld = row6_quintic_interp(t, total, l0, l1);
            int cmd_r[7], cmd_l[7];
            for (int i = 0; i < 7; ++i)
            {
                cmd_r[i] = pos_r[i];
                cmd_l[i] = pos_l[i];
            }
            cmd_r[c.right_shoulder_joint_index] = motor_angle_deg_to_cnt(rd);
            cmd_l[c.left_shoulder_joint_index] = motor_angle_deg_to_cnt(ld);
            set_motor_position(cmd_r, 1);
            set_motor_position(cmd_l, 0);
            usleep(static_cast<useconds_t>(c.smooth_dt_ms * 1000));
            t += dt_s;
        }
        set_motor_position(pos_r, 1);
        set_motor_position(pos_l, 0);
        save_arm_last_commanded_position(1, pos_r);
        save_arm_last_commanded_position(0, pos_l);
        return wait_arms_motors_stopped(true, true, 15.0, 6, "row6肩");
    }

    /** 仅抬一侧肩（空闲手防碰撞）：右手 +Δ，左手 -Δ */
    bool row6_smooth_shoulder_one_arm(bool is_right, double delta_deg)
    {
        const auto &c = row6_bend_cfg();
        if (std::abs(delta_deg) < 0.01)
            return true;

        const int hand = is_right ? 1 : 0;
        int pos[7] = {0};
        get_motor_position(pos, hand);
        const int ji = is_right ? c.right_shoulder_joint_index : c.left_shoulder_joint_index;
        const double s0 = motor_cnt_to_angle_deg(pos[ji]);
        const double s1 = is_right ? s0 + delta_deg : s0 - delta_deg;

        std::cout << "[row6] 单肩 " << (is_right ? "右" : "左") << " " << s0 << "°→" << s1
                  << "° Δ=" << (is_right ? delta_deg : -delta_deg) << "°\n";

        pos[ji] = motor_angle_deg_to_cnt(s1);

        const double total = std::abs(delta_deg) / c.speed_deg_per_s;
        const double dt_s = static_cast<double>(c.smooth_dt_ms) / 1000.0;
        double t = 0.0;
        while (t < total)
        {
            const double sd = row6_quintic_interp(t, total, s0, s1);
            int cmd[7];
            get_motor_position(cmd, hand);
            cmd[ji] = motor_angle_deg_to_cnt(sd);
            set_motor_position(cmd, hand);
            usleep(static_cast<useconds_t>(c.smooth_dt_ms * 1000));
            t += dt_s;
        }
        set_motor_position(pos, hand);
        save_arm_last_commanded_position(hand, pos);
        return wait_arms_motors_stopped(is_right, !is_right, 15.0, 6, "row6单肩");
    }

    enum class Row6ShoulderPrep
    {
        Auto,
        Skip,
        BothArms,
        /** @deprecated 生产流程已改为弯腰前双手均抬肩；保留枚举值兼容旧调用 */
        IdleOppositeOnly,
    };

    enum class Row6AfterRelease
    {
        Auto,
        FullStandby,
        /** 中间带双手第六排：放完一手后腰保持弯，已放手丝滑回记录的 7 关节 */
        KeepBentWaistForNextRow6,
    };

    bool row6_apply_shoulder_prep(Row6ShoulderPrep mode, bool move_r, bool move_l)
    {
        const auto &c = row6_bend_cfg();
        if (mode == Row6ShoulderPrep::Skip)
            return true;
        if (mode == Row6ShoulderPrep::BothArms)
            return row6_smooth_shoulder_only(c.shoulder_lift_deg);
        if (mode == Row6ShoulderPrep::IdleOppositeOnly)
            return row6_smooth_shoulder_only(c.shoulder_lift_deg);
        // Auto：只要弯腰前抬肩，双手肩均 +Δ（单手抓货时也抬抓取侧肩）
        return row6_smooth_shoulder_only(c.shoulder_lift_deg);
    }

    /** 直线回 yaml standby，但保留当前肩电机角（+36° 预备位） */
    bool row6_move_arm_to_standby_keep_shoulder(Robot_Arm &arm, bool is_right)
    {
        const auto &c = row6_bend_cfg();
        const int hand = is_right ? 1 : 0;
        int pos[7] = {0};
        get_motor_position(pos, hand);
        const int ji = is_right ? c.right_shoulder_joint_index : c.left_shoulder_joint_index;
        const int saved_shoulder = pos[ji];

        Eigen::Matrix<double, 1, 6> goal =
            is_right ? make_standby_pos_right() : make_standby_pos_left();
        std::cout << "[row6] " << (is_right ? "右" : "左") << "手回待机(TCP)+保留肩角\n";
        {
            ArmLineMoveDebugStage stage("第6排回standby保肩");
            const int ret = arm_line_move(arm, goal, 0.2);
            if (ret != 0)
            {
                log_arm_traj_plan_fail(is_right ? "右手" : "左手", ret);
                return false;
            }
        }
        if (!wait_arms_motors_stopped(is_right, !is_right, 15.0, 6, "第6排回standby保肩"))
            return false;

        get_motor_position(pos, hand);
        pos[ji] = saved_shoulder;
        set_motor_position(pos, hand);
        save_arm_last_commanded_position(hand, pos);
        return wait_arms_motors_stopped(is_right, !is_right, 15.0, 6, "第6排肩角恢复");
    }

    bool row6_smooth_waist_pitch_only(double bend_deg)
    {
        const auto &c = row6_bend_cfg();
        if (std::abs(bend_deg) < 0.01)
            return true;

        int cur = 0;
        if (!row6_read_waist_pitch_cnt(cur))
            return false;
        const double start = motor_cnt_to_angle_deg(cur);
        const double tgt = start + bend_deg;
        const int tgt_cnt = motor_angle_deg_to_cnt(tgt);

        std::cout << "[row6] 腰2 pitch " << start << "°→" << tgt << "°\n";

        const double total = std::abs(bend_deg) / c.speed_deg_per_s;
        const double dt_s = static_cast<double>(c.smooth_dt_ms) / 1000.0;
        double t = 0.0;
        while (t < total)
        {
            const double w = row6_quintic_interp(t, total, start, tgt);
            row6_send_waist_pitch_cnt(motor_angle_deg_to_cnt(w));
            usleep(static_cast<useconds_t>(c.smooth_dt_ms * 1000));
            t += dt_s;
        }
        row6_send_waist_pitch_cnt(tgt_cnt);
        return row6_wait_waist_pitch_reached(tgt_cnt, 15.0, bend_deg >= 0 ? "弯腰" : "回正");
    }

    bool row6_bend_out_waist_only()
    {
        if (!g_row6_waist_bent)
            return true;
        const bool ok = row6_smooth_waist_pitch_only(-row6_bend_cfg().waist_pitch_deg);
        if (ok)
            g_row6_waist_bent = false;
        return ok;
    }

    bool row6_bend_waist_in()
    {
        if (g_row6_waist_bent)
            return true;
        const bool ok = row6_smooth_waist_pitch_only(row6_bend_cfg().waist_pitch_deg);
        if (ok)
            g_row6_waist_bent = true;
        return ok;
    }

    bool row6_bend_in()
    {
        const auto &c = row6_bend_cfg();
        if (!row6_smooth_shoulder_only(c.shoulder_lift_deg))
            return false;
        if (g_row6_waist_bent)
            return true;
        const bool ok = row6_smooth_waist_pitch_only(c.waist_pitch_deg);
        if (ok)
            g_row6_waist_bent = true;
        return ok;
    }

    Eigen::Matrix<double, 1, 6> row6_compensate_pitch_xyz(
        const Eigen::Matrix<double, 1, 6> &upright,
        double pitch_deg,
        double pivot_z)
    {
        const double th = pitch_deg * M_PI / 180.0;
        const double c = std::cos(th);
        const double s = std::sin(th);
        const double x = upright(0);
        const double zr = upright(2) - (-pivot_z);
        Eigen::Matrix<double, 1, 6> out = upright;
        out(0) = x * c - zr * s;
        out(2) = -pivot_z + x * s + zr * c;
        return out;
    }

    void row6_apply_bent_orientation(Eigen::Matrix<double, 1, 6> &goal, bool is_right)
    {
        const auto &c = row6_bend_cfg();
        if (is_right)
        {
            goal(3) = c.right_rx_deg * rad;
            goal(4) = c.right_ry_deg * rad;
            goal(5) = c.right_rz_deg * rad;
        }
        else
        {
            goal(3) = c.left_rx_deg * rad;
            goal(4) = c.left_ry_deg * rad;
            goal(5) = c.left_rz_deg * rad;
        }
    }

    void row6_apply_world_vertical(Eigen::Matrix<double, 1, 6> &goal, double vertical_h_m, double pitch_deg)
    {
        const double th = pitch_deg * M_PI / 180.0;
        goal(0) -= vertical_h_m * std::sin(th);
        goal(2) += vertical_h_m * std::cos(th);
    }

    void row6_build_bent_raise_descend(
        Eigen::Matrix<double, 1, 6> &goal_inout,
        Eigen::Matrix<double, 1, 6> &descend_out,
        bool is_right)
    {
        const auto &c = row6_bend_cfg();
        goal_inout(2) -= c.z_raise;
        goal_inout = row6_compensate_pitch_xyz(goal_inout, c.waist_pitch_deg, c.pivot_z_below_base_m);
        row6_apply_bent_orientation(goal_inout, is_right);
        row6_apply_world_vertical(goal_inout, c.z_raise, c.waist_pitch_deg);
        descend_out = goal_inout;
        row6_apply_world_vertical(descend_out, c.z_descend, c.waist_pitch_deg);
    }

    bool row6_bent_dual_move_raise_descend(
        Robot_Arm &arm_r,
        Robot_Arm &arm_l,
        Eigen::Matrix<double, 1, 6> raise_r,
        Eigen::Matrix<double, 1, 6> raise_l,
        Eigen::Matrix<double, 1, 6> desc_r,
        Eigen::Matrix<double, 1, 6> desc_l,
        bool move_r,
        bool move_l,
        const char *tag)
    {
        log_place_motion_cmd("第6排-弯放抬高位", move_r, move_l, raise_r, raise_l);
        const ArmLineMoveResult r1 = [&]()
        {
            ArmLineMoveDebugStage stage("第6排弯放抬高位");
            return arm_dual_line_move_selective(arm_r, raise_r, move_r, arm_l, raise_l, move_l, 0.2);
        }();
        if ((move_r && r1.ret_r != 0) || (move_l && r1.ret_l != 0))
        {
            if (move_r && r1.ret_r != 0)
                log_arm_traj_plan_fail("右手(第6排弯放)", r1.ret_r);
            if (move_l && r1.ret_l != 0)
                log_arm_traj_plan_fail("左手(第6排弯放)", r1.ret_l);
            std::cerr << "[row6] " << tag << " 抬高位轨迹失败\n";
            return false;
        }
        if (!wait_arms_motors_stopped(move_r, move_l, 15.0, 6, "第6排弯放抬高位"))
            return false;

        if (std::abs(row6_bend_cfg().z_descend) <= 1e-9)
            return true;

        log_place_motion_cmd("第6排-弯放下压", move_r, move_l, desc_r, desc_l);
        const ArmLineMoveResult r2 = [&]()
        {
            ArmLineMoveDebugStage stage("第6排弯放下压");
            return arm_dual_line_move_selective(arm_r, desc_r, move_r, arm_l, desc_l, move_l, 0.2);
        }();
        if ((move_r && r2.ret_r != 0) || (move_l && r2.ret_l != 0))
        {
            std::cerr << "[row6] " << tag << " 下压轨迹失败\n";
            return false;
        }
        return wait_arms_motors_stopped(move_r, move_l, 15.0, 6, "第6排弯放下压");
    }

    bool row6_waist_pitch_angle_deg(double &out_deg)
    {
        int cnt = 0;
        if (!row6_read_waist_pitch_cnt(cnt))
            return false;
        out_deg = motor_cnt_to_angle_deg(cnt);
        return true;
    }

    bool row6_waist_is_bent()
    {
        return g_row6_waist_bent;
    }

    bool row6_bend_waist_in_if_needed()
    {
        if (g_row6_waist_bent)
        {
            std::cout << "[row6] 腰已弯，跳过重复弯腰\n";
            return true;
        }
        const bool ok = row6_smooth_waist_pitch_only(row6_bend_cfg().waist_pitch_deg);
        if (ok)
            g_row6_waist_bent = true;
        return ok;
    }

    bool row6_bend_waist_out_if_needed()
    {
        if (!g_row6_waist_bent)
        {
            std::cout << "[row6] 腰已直，跳过重复回正\n";
            return true;
        }
        return row6_bend_out_waist_only();
    }

    void row6_apply_retreat_orientation(Eigen::Matrix<double, 1, 6> &goal, bool is_right)
    {
        const auto &c = row6_bend_cfg();
        if (is_right)
        {
            goal(3) = c.retreat_right_rx_deg * rad;
            goal(4) = c.retreat_right_ry_deg * rad;
            goal(5) = c.retreat_right_rz_deg * rad;
        }
        else
        {
            goal(3) = c.retreat_left_rx_deg * rad;
            goal(4) = c.retreat_left_ry_deg * rad;
            goal(5) = c.retreat_left_rz_deg * rad;
        }
    }

    /** 松爪后：先沿世界竖直撤销 row6 z_descend 回到 z_raise 高度，再 x/y/z/姿态让位 */
    bool row6_bent_retreat_after_release(
        Robot_Arm &arm_r,
        Robot_Arm &arm_l,
        const Eigen::Matrix<double, 1, 6> &desc_r,
        const Eigen::Matrix<double, 1, 6> &desc_l,
        bool move_r,
        bool move_l,
        const char *tag)
    {
        const auto &c = row6_bend_cfg();
        const double pitch = c.waist_pitch_deg;
        const double z_ascend = -c.z_descend;

        Eigen::Matrix<double, 1, 6> ascend_r = desc_r;
        Eigen::Matrix<double, 1, 6> ascend_l = desc_l;
        if (move_r && std::abs(z_ascend) > 1e-9)
            row6_apply_world_vertical(ascend_r, z_ascend, pitch);
        if (move_l && std::abs(z_ascend) > 1e-9)
            row6_apply_world_vertical(ascend_l, z_ascend, pitch);

        if (std::abs(z_ascend) > 1e-9 && (move_r || move_l))
        {
            log_place_motion_cmd("第6排弯放抬升", move_r, move_l, ascend_r, ascend_l);
            const ArmLineMoveResult r1 = [&]()
            {
                ArmLineMoveDebugStage stage("第6排弯放抬升");
                return arm_dual_line_move_selective(
                    arm_r, ascend_r, move_r, arm_l, ascend_l, move_l, 0.2);
            }();
            if ((move_r && r1.ret_r != 0) || (move_l && r1.ret_l != 0))
            {
                std::cerr << "[row6] " << tag << " 松爪后抬升失败\n";
                return false;
            }
            if (!wait_arms_motors_stopped(move_r, move_l, 15.0, 6, "第6排弯放抬升"))
                return false;
        }

        Eigen::Matrix<double, 1, 6> retreat_r = ascend_r;
        Eigen::Matrix<double, 1, 6> retreat_l = ascend_l;
        if (move_r)
        {
            retreat_r(0) += c.retreat_x_delta_m;
            retreat_r(1) = c.retreat_y_right;
            row6_apply_retreat_orientation(retreat_r, true);
            if (std::abs(c.retreat_z_delta_m) > 1e-9)
                row6_apply_world_vertical(retreat_r, c.retreat_z_delta_m, pitch);
        }
        if (move_l)
        {
            retreat_l(0) += c.retreat_x_delta_m;
            retreat_l(1) = c.retreat_y_left;
            row6_apply_retreat_orientation(retreat_l, false);
            if (std::abs(c.retreat_z_delta_m) > 1e-9)
                row6_apply_world_vertical(retreat_l, c.retreat_z_delta_m, pitch);
        }

        log_place_motion_cmd("第6排弯放让位", move_r, move_l, retreat_r, retreat_l);
        const ArmLineMoveResult r2 = [&]()
        {
            ArmLineMoveDebugStage stage("第6排弯放让位");
            return arm_dual_line_move_selective(
                arm_r, retreat_r, move_r, arm_l, retreat_l, move_l, 0.2);
        }();
        if ((move_r && r2.ret_r != 0) || (move_l && r2.ret_l != 0))
        {
            std::cerr << "[row6] " << tag << " 松爪后让位失败\n";
            return false;
        }
        return wait_arms_motors_stopped(move_r, move_l, 15.0, 6, "第6排弯放让位");
    }

    bool run_post_grasp_place_flow(
        gripper::Gripper &g,
        Robot_Arm &arm_r,
        Robot_Arm &arm_l,
        HeadAssignState &st,
        bool grasped_r,
        bool grasped_l,
        WaistRobot &waist,
        Eigen::Matrix<double, 1, 6> &posup_down_3_layer,
        LanxinControl &ctrl,
        RealSenseMultiCam &cameras,
        SegPoseBridge &bridge,
        const std::array<double, 16> &cam2robot,
        bool &waist_adjusted)
    {
        log_phase_banner("放货流程：底盘旋转 + 腰到放货预备位");

        chassis_rotate_with_waist_place_ready(ctrl, waist, posup_down_3_layer, 90, "放货前");

        log_phase_banner("放货流程：头相机空位识别(class1)");
        const PlaceHoleDetectResult holes = detect_place_holes_with_consensus(
            bridge,
            cameras,
            cam2robot,
            g_move_cfg.place.detect_trials,
            0.02,
            0.05,
            kDebugVisualize);
        if (!holes.ok)
            std::cerr << "[place] 空位识别失败: " << holes.message << "，无格点时改放 config fallback\n";
        else
            cout << "[place] " << holes.message << endl;

        const std::vector<Eigen::Matrix<double, 1, 6>> raw_holes =
            holes.ok ? holes.poses_robot : std::vector<Eigen::Matrix<double, 1, 6>>{};
        PlaceSlotCatalog slot_cat;
        const PlaceGridSession &pg_session = place_grid_session();
        const bool use_corr_grid =
            pg_session.active &&
            place_grid_build_slot_catalog(pg_session, raw_holes, slot_cat);

        std::vector<Eigen::Matrix<double, 1, 6>> grid;
        std::vector<int> grid_row_for_index;
        std::vector<int> grid_flat_cell;
        if (use_corr_grid)
        {
            grid = slot_cat.poses;
            grid_row_for_index = slot_cat.row_for_index;
            grid_flat_cell = slot_cat.flat_cell;
            cout << "[place] 使用48格corr放货 候选=" << grid.size()
                 << " 顺序=" << (g_move_cfg.place.place_non_anchor_first != 0 ? "先44" : "先4")
                 << " 阶段=" << place_grid_place_phase_label(place_grid_place_phase(pg_session))
                 << " 已放非基准=" << pg_session.placed_non_anchor << "/"
                 << kPlaceGridNonAnchorCells
                 << " 基准=" << pg_session.placed_anchor << "/4\n";
        }
        else
        {
            grid = raw_holes;
            if (pg_session.active)
                cout << "[place] 棋盘无可用corr格，回退 raw 空位列表 size=" << grid.size() << "\n";
        }

        const std::vector<int> *row_lookup =
            use_corr_grid ? &grid_row_for_index : nullptr;
        const std::vector<int> *flat_lookup =
            use_corr_grid ? &grid_flat_cell : nullptr;

        auto grid_row_0 = [&](size_t idx) -> int
        {
            if (idx >= grid.size())
                return -1;
            if (row_lookup && idx < row_lookup->size())
                return (*row_lookup)[idx];
            return place_pose_row_from_x(grid[idx](0));
        };

        auto mark_grid_placed = [&](std::optional<size_t> slot_idx)
        {
            if (!slot_idx || !flat_lookup || *slot_idx >= flat_lookup->size())
                return;
            place_grid_mark_cell_placed((*flat_lookup)[*slot_idx]);
        };
        bool placed_any = false;
        bool released_r = false;
        bool released_l = false;

        std::vector<size_t> left_idx;
        std::vector<size_t> right_idx;
        std::vector<size_t> middle_idx;
        left_idx.reserve(grid.size());
        right_idx.reserve(grid.size());
        middle_idx.reserve(grid.size());
        const double y_split = g_move_cfg.place_zone.y_side_split;
        const double y_mid_lo = g_move_cfg.place_zone.y_right;
        const double y_mid_hi = g_move_cfg.place_zone.y_left;
        for (size_t i = 0; i < grid.size(); ++i)
        {
            const int row = grid_row_0(i);
            if (row < 0 || !is_row_enabled_for_place(row))
                continue;
            const double y = grid[i](1);
            if (y > y_split)
                left_idx.push_back(i);
            if (y < y_split)
                right_idx.push_back(i);
            if (y >= y_mid_lo && y <= y_mid_hi)
                middle_idx.push_back(i);
        }
        cout << "[place] 空位分区(侧区 y>" << y_split << " / y<" << y_split << ", 中间 ["
             << y_mid_lo << "," << y_mid_hi << "]): 左=" << left_idx.size()
             << " 右=" << right_idx.size() << " 中=" << middle_idx.size() << endl;
        cout << "[place] 放货顺序=" << (g_move_cfg.place.row_place_from_front != 0 ? "前→后" : "后→前");
        cout << " row_enabled:";
        for (size_t i = 0; i < g_move_cfg.place.row_enabled.size(); ++i)
            cout << " r" << (i + 1) << "=" << g_move_cfg.place.row_enabled[i];
        cout << "\n";

        log_phase_banner("放货流程：选点与侧区/中间带放货");

        bool move_side_l = false;
        bool move_side_r = false;
        std::optional<size_t> side_pick_l;
        std::optional<size_t> side_pick_r;
        if (grasped_l)
        {
            if (const auto pick = pick_place_slot_row_ordered(grid, left_idx, row_lookup))
            {
                side_pick_l = *pick;
                const auto &p = grid[*pick];
                apply_place_goal_xy_left(st.goal_last_l, p(0), p(1), grid_row_0(*pick));
                move_side_l = true;
                cout << "[place] 左区(y>" << y_split << ")按排优先 idx=" << *pick << " xy=(" << p(0)
                     << "," << p(1) << ") 排=" << (grid_row_0(*pick) + 1) << "\n";
            }
        }
        if (grasped_r)
        {
            if (const auto pick = pick_place_slot_row_ordered(grid, right_idx, row_lookup))
            {
                side_pick_r = *pick;
                const auto &p = grid[*pick];
                apply_place_goal_xy_right(st.goal_last_r, p(0), p(1), grid_row_0(*pick));
                move_side_r = true;
                cout << "[place] 右区(y<" << y_split << ")按排优先 idx=" << *pick << " xy=(" << p(0)
                     << "," << p(1) << ") 排=" << (grid_row_0(*pick) + 1) << "\n";
            }
        }

        auto pose_in_middle_band = [&](size_t idx) -> bool
        {
            if (idx >= grid.size())
                return false;
            const double y = grid[idx](1);
            return y >= y_mid_lo && y <= y_mid_hi;
        };

        double waist_row_extra_applied = 0.0;
        auto hand_still_pending_place = [&](bool is_right) -> bool
        {
            if (is_right)
                return grasped_r && side_pick_r.has_value();
            return grasped_l && side_pick_l.has_value();
        };

        auto ensure_waist_for_grid_row = [&](int row_0) -> double
        {
            if (row_0 < 0)
                return 0.0;
            const double target = configured_row_waist_x(row_0);
            const double delta = target - waist_row_extra_applied;
            if (std::abs(delta) <= 1e-9)
                return 0.0;
            std::ostringstream oss;
            oss << "[place] 第" << (row_0 + 1) << "排腰移动";
            const double actual_delta = waist_layer3_apply_place_x_delta(
                waist, posup_down_3_layer, delta, oss.str().c_str());
            if (std::abs(actual_delta) <= 1e-9)
                return 0.0;
            waist_row_extra_applied += actual_delta;
            return waist_x_arm_goal_comp(actual_delta);
        };

        /** 腰动后：本次放货的手 + 仍抓着待放的手，都要加 x 补偿 */
        auto apply_waist_dx_to_pending_hands = [&](double dx, bool do_r, bool do_l)
        {
            if (std::abs(dx) <= 1e-9)
                return;
            const bool comp_r = grasped_r && (do_r || hand_still_pending_place(true));
            const bool comp_l = grasped_l && (do_l || hand_still_pending_place(false));
            if (comp_r)
                st.goal_last_r(0) += dx;
            if (comp_l)
                st.goal_last_l(0) += dx;
        };

        auto apply_row_waist_goal_comp = [&](bool do_r, bool do_l, int row_r, int row_l)
        {
            if (do_r && do_l && row_r == row_l)
            {
                apply_waist_dx_to_pending_hands(ensure_waist_for_grid_row(row_r), do_r, do_l);
                return;
            }
            if (do_r)
                apply_waist_dx_to_pending_hands(ensure_waist_for_grid_row(row_r), do_r, do_l);
            if (do_l)
                apply_waist_dx_to_pending_hands(ensure_waist_for_grid_row(row_l), do_r, do_l);
        };

        auto try_place_open_after_encoder = [&](bool move_r,
                                                bool move_l,
                                                std::optional<size_t> mark_slot_l,
                                                std::optional<size_t> mark_slot_r,
                                                bool used_fallback_r,
                                                bool used_fallback_l,
                                                const char *tag) -> bool
        {
            const bool open_r = move_r && grasped_r;
            const bool open_l = move_l && grasped_l;
            if (!open_r && !open_l)
                return false;
            if (!wait_arms_motors_stopped(open_r, open_l, 15.0, 6, "放货松爪前"))
            {
                std::cerr << "[place] " << tag << " 松爪前编码器未到位，跳过松爪\n";
                return false;
            }
            if (open_l)
            {
                g.openAndWait(gripper::Side::Left);
                released_l = true;
                if (mark_slot_l && !used_fallback_l)
                {
                    consume_place_slot_from_lists(
                        left_idx, right_idx, middle_idx, mark_slot_l, std::nullopt);
                    mark_grid_placed(mark_slot_l);
                }
                placed_any = true;
            }
            if (open_r)
            {
                g.openAndWait(gripper::Side::Right);
                released_r = true;
                if (mark_slot_r && !used_fallback_r)
                {
                    consume_place_slot_from_lists(
                        left_idx, right_idx, middle_idx, std::nullopt, mark_slot_r);
                    mark_grid_placed(mark_slot_r);
                }
                placed_any = true;
            }
            return true;
        };

        auto place_release_config_fallback = [&](bool do_r, bool do_l, const char *tag)
        {
            if (!do_r && !do_l)
                return;
            bool move_r = do_r && grasped_r && !released_r;
            bool move_l = do_l && grasped_l && !released_l;
            if (!move_r && !move_l)
                return;
            if (move_r)
                apply_place_fallback_goal_right(st.goal_last_r);
            if (move_l)
                apply_place_fallback_goal_left(st.goal_last_l);
            bool used_fallback_r = move_r;
            bool used_fallback_l = move_l;
            place_line_move_with_fallback(
                arm_r, arm_l, st, move_r, move_l, used_fallback_r, used_fallback_l, tag);
            if (!move_r && !move_l)
            {
                cout << tag << "，config fallback 轨迹失败，跳过松爪\n";
                return;
            }
            place_descend_before_release(
                arm_r, arm_l, st, move_r, move_l, used_fallback_r, used_fallback_l);
            if (!move_r && !move_l)
            {
                cout << tag << "，config fallback 下压失败，跳过松爪\n";
                return;
            }
            append_place_pose_before_release_debug(
                arm_r,
                arm_l,
                st,
                move_r,
                move_l,
                grasped_r,
                grasped_l,
                tag,
                holes.ok ? &grid : nullptr,
                std::nullopt,
                std::nullopt);
            try_place_open_after_encoder(
                move_r, move_l, std::nullopt, std::nullopt, used_fallback_r, used_fallback_l, tag);
            place_ascend_after_release(arm_r, arm_l, st, move_r, move_l);
            cout << tag << "，双臂回待机位\n";
            move_arms_to_standby(arm_r, arm_l);
        };

        bool row6_session_bent = false;
        bool row6_shoulder_prepared = false;
        bool row6_force_both_shoulders_next = false;
        bool row6_pending_dual_middle = false;
        bool row6_layer3_restored_in_session = false;

        /** 仍抓着待放的那只手对应排号（侧区/中间带/goal x 推断） */
        auto pending_hand_row_0 = [&](bool is_right) -> int
        {
            if (is_right)
            {
                if (!(grasped_r && !released_r))
                    return -1;
                if (side_pick_r)
                    return grid_row_0(*side_pick_r);
            }
            else
            {
                if (!(grasped_l && !released_l))
                    return -1;
                if (side_pick_l)
                    return grid_row_0(*side_pick_l);
            }
            if (!middle_idx.empty())
            {
                const auto hand_indices =
                    filter_place_indices_for_hand(grid, middle_idx, is_right, y_split);
                if (const auto pick = pick_place_slot_row_ordered(grid, hand_indices, row_lookup))
                    return grid_row_0(*pick);
            }
            const double x = is_right ? st.goal_last_r(0) : st.goal_last_l(0);
            if (x > 0.05 && !use_corr_grid)
                return place_pose_row_from_x(x);
            return -1;
        };

        auto row6_other_hand_row6_pending = [&](bool this_move_r, bool this_move_l) -> bool
        {
            if (this_move_r && !this_move_l)
            {
                const int other_row = pending_hand_row_0(false);
                return grasped_l && !released_l && other_row >= 0 &&
                       row6_bend_enabled_for_row(other_row);
            }
            if (this_move_l && !this_move_r)
            {
                const int other_row = pending_hand_row_0(true);
                return grasped_r && !released_r && other_row >= 0 &&
                       row6_bend_enabled_for_row(other_row);
            }
            return false;
        };

        auto row6_finish_place_session = [&](const char *tag)
        {
            cout << tag << "，第6排结束：腰回正→layer3后退→双臂待机\n";
            row6_bend_waist_out_if_needed();
            row6_session_bent = false;
            row6_pending_dual_middle = false;
            row6_shoulder_prepared = false;
            waist_row_extra_applied = 0.0;
            restore_waist_layer3_x(waist, posup_down_3_layer);
            row6_layer3_restored_in_session = true;
            waist_adjusted = false;
            move_arms_to_standby(arm_r, arm_l);
        };

        const bool pick_l = move_side_l && side_pick_l.has_value();
        const bool pick_r = move_side_r && side_pick_r.has_value();
        const bool act_l = pick_l && grasped_l;
        const bool act_r = pick_r && grasped_r;
        const bool both_in_middle =
            act_l && act_r && side_pick_l && side_pick_r &&
            pose_in_middle_band(*side_pick_l) && pose_in_middle_band(*side_pick_r);

        auto place_release_one = [&](bool do_r,
                                     bool do_l,
                                     int row_r,
                                     int row_l,
                                     std::optional<size_t> mark_slot_l,
                                     std::optional<size_t> mark_slot_r,
                                     const char *tag,
                                     Row6ShoulderPrep row6_shoulder = Row6ShoulderPrep::Auto,
                                     Row6AfterRelease row6_after = Row6AfterRelease::Auto)
        {
            if (!do_r && !do_l)
                return;
            bool move_r = do_r && grasped_r;
            bool move_l = do_l && grasped_l;
            bool used_fallback_r = false;
            bool used_fallback_l = false;
            if (!row6_pending_dual_middle)
                apply_row_waist_goal_comp(move_r, move_l, row_r, row_l);

            const bool bend_r = move_r && row_r >= 0 && row6_bend_enabled_for_row(row_r);
            const bool bend_l = move_l && row_l >= 0 && row6_bend_enabled_for_row(row_l);
            if (bend_r || bend_l)
            {
                Row6ShoulderPrep shoulder_prep = row6_shoulder;
                if (shoulder_prep == Row6ShoulderPrep::Auto)
                {
                    if (row6_shoulder_prepared)
                        shoulder_prep = Row6ShoulderPrep::Skip;
                    else
                        shoulder_prep = Row6ShoulderPrep::BothArms;
                }
                else if (row6_shoulder_prepared && shoulder_prep != Row6ShoulderPrep::BothArms)
                {
                    shoulder_prep = Row6ShoulderPrep::Skip;
                }
                row6_force_both_shoulders_next = false;

                if (shoulder_prep != Row6ShoulderPrep::Skip)
                {
                    if (!row6_apply_shoulder_prep(shoulder_prep, move_r, move_l))
                    {
                        std::cerr << tag << "，第6排抬肩失败，跳过本段放货\n";
                        return;
                    }
                    if (!wait_arms_motors_stopped(true, true, 15.0, 6, "row6肩到位"))
                    {
                        std::cerr << tag << "，第6排抬肩编码器未到位\n";
                        return;
                    }
                    row6_shoulder_prepared = true;
                }

                Row6AfterRelease after = row6_after;
                if (after == Row6AfterRelease::Auto)
                {
                    if (row6_pending_dual_middle)
                        after = Row6AfterRelease::FullStandby;
                    else if (row6_other_hand_row6_pending(move_r, move_l))
                        after = Row6AfterRelease::KeepBentWaistForNextRow6;
                    else
                        after = Row6AfterRelease::FullStandby;
                }

                if (!row6_bend_waist_in_if_needed())
                {
                    std::cerr << tag << "，第6排弯腰失败，跳过本段放货\n";
                    return;
                }
                row6_session_bent = true;

                Eigen::Matrix<double, 1, 6> raise_r = st.goal_last_r;
                Eigen::Matrix<double, 1, 6> desc_r = st.goal_last_r;
                Eigen::Matrix<double, 1, 6> raise_l = st.goal_last_l;
                Eigen::Matrix<double, 1, 6> desc_l = st.goal_last_l;
                if (bend_r)
                    row6_build_bent_raise_descend(raise_r, desc_r, true);
                if (bend_l)
                    row6_build_bent_raise_descend(raise_l, desc_l, false);

                const bool traj_r = move_r && bend_r;
                const bool traj_l = move_l && bend_l;
                if (!row6_bent_dual_move_raise_descend(
                        arm_r, arm_l, raise_r, raise_l, desc_r, desc_l, traj_r, traj_l, tag))
                {
                    std::cerr << tag << "，第6排弯放轨迹失败，跳过松爪\n";
                    return;
                }

                append_place_pose_before_release_debug(
                    arm_r,
                    arm_l,
                    st,
                    move_r,
                    move_l,
                    grasped_r,
                    grasped_l,
                    tag,
                    &grid,
                    mark_slot_r,
                    mark_slot_l);
                try_place_open_after_encoder(
                    move_r,
                    move_l,
                    mark_slot_l,
                    mark_slot_r,
                    false,
                    false,
                    tag);

                const bool retreat_r = move_r && bend_r;
                const bool retreat_l = move_l && bend_l;
                if (!row6_bent_retreat_after_release(
                        arm_r, arm_l, desc_r, desc_l, retreat_r, retreat_l, tag))
                {
                    std::cerr << tag << "，第6排松爪后抬升/让位失败\n";
                }

                if (after == Row6AfterRelease::KeepBentWaistForNextRow6)
                {
                    row6_pending_dual_middle = true;
                    const int other_row =
                        move_r ? pending_hand_row_0(false) : pending_hand_row_0(true);
                    cout << tag << "，已放一手，腰保持弯(待放另一手第" << (other_row + 1)
                         << "排)，layer3 不回退\n";
                    return;
                }

                row6_finish_place_session(tag);
                return;
            }

            place_line_move_with_fallback(
                arm_r, arm_l, st, move_r, move_l, used_fallback_r, used_fallback_l, tag);
            if (!move_r && !move_l)
            {
                cout << tag << "，轨迹解析失败且 fallback 不可用，跳过本段放货\n";
                return;
            }
            place_descend_before_release(
                arm_r, arm_l, st, move_r, move_l, used_fallback_r, used_fallback_l);
            if (!move_r && !move_l)
            {
                cout << tag << "，下压轨迹失败，跳过松爪\n";
                return;
            }
            append_place_pose_before_release_debug(
                arm_r,
                arm_l,
                st,
                move_r,
                move_l,
                grasped_r,
                grasped_l,
                tag,
                &grid,
                used_fallback_r ? std::nullopt : mark_slot_r,
                used_fallback_l ? std::nullopt : mark_slot_l);
            try_place_open_after_encoder(
                move_r,
                move_l,
                mark_slot_l,
                mark_slot_r,
                used_fallback_r,
                used_fallback_l,
                tag);
            place_ascend_after_release(arm_r, arm_l, st, move_r, move_l);
            cout << tag << "，双臂回待机位\n";
            move_arms_to_standby(arm_r, arm_l);
            row6_shoulder_prepared = false;
        };

        auto run_row6_dual_place_sequential = [&](int row_r,
                                                  int row_l,
                                                  std::optional<size_t> slot_l,
                                                  std::optional<size_t> slot_r,
                                                  const char *tag_prefix)
        {
            cout << tag_prefix << "，第六排双手：先右后左，腰只弯一次\n";
            place_release_one(
                true,
                false,
                row_r,
                row_l,
                std::nullopt,
                slot_r,
                "[place] 第六排右手放完",
                Row6ShoulderPrep::BothArms,
                Row6AfterRelease::KeepBentWaistForNextRow6);
            place_release_one(
                false,
                true,
                row_r,
                row_l,
                slot_l,
                std::nullopt,
                "[place] 第六排左手放完",
                Row6ShoulderPrep::Skip,
                Row6AfterRelease::FullStandby);
        };

        auto run_side_place_phase = [&](bool want_r, bool want_l, int row_r, int row_l, const char *tag)
        {
            if (!want_r && !want_l)
                return;
            if (both_in_middle && want_r && want_l)
            {
                cout << "[place] 中间带 [" << y_mid_lo << "," << y_mid_hi
                     << "]，依次：右手→左手\n";
                const bool both_row6 =
                    row6_bend_enabled_for_row(row_r) && row6_bend_enabled_for_row(row_l);
                if (both_row6)
                {
                    run_row6_dual_place_sequential(
                        row_r, row_l, side_pick_l, side_pick_r, "[place] 中间带");
                }
                else
                {
                    place_release_one(
                        true,
                        false,
                        row_r,
                        row_l,
                        std::nullopt,
                        side_pick_r,
                        "[place] 中间带右手放完");
                    place_release_one(
                        false,
                        true,
                        row_r,
                        row_l,
                        side_pick_l,
                        std::nullopt,
                        "[place] 中间带左手放完");
                }
            }
            else
            {
                place_release_one(
                    want_r,
                    want_l,
                    row_r,
                    row_l,
                    want_l ? side_pick_l : std::nullopt,
                    want_r ? side_pick_r : std::nullopt,
                    tag);
            }
        };

        if (act_l || act_r)
        {
            const int row_l = side_pick_l ? grid_row_0(*side_pick_l) : -1;
            const int row_r = side_pick_r ? grid_row_0(*side_pick_r) : -1;

            if (act_l && act_r && row_l >= 0 && row_r >= 0 && row_l != row_r)
            {
                const bool r6_l = row6_bend_enabled_for_row(row_l);
                const bool r6_r = row6_bend_enabled_for_row(row_r);
                if (r6_l != r6_r)
                {
                    cout << "[place] 第六排与非第六排：先放非第六排，再双手抬肩+第6排弯放\n";
                    if (r6_r)
                    {
                        run_side_place_phase(false, true, row_r, row_l, "[place] 非第六排侧区左手");
                        row6_force_both_shoulders_next = true;
                        run_side_place_phase(
                            true,
                            false,
                            row_r,
                            row_l,
                            "[place] 第六排侧区右手");
                    }
                    else
                    {
                        run_side_place_phase(true, false, row_r, row_l, "[place] 非第六排侧区右手");
                        row6_force_both_shoulders_next = true;
                        run_side_place_phase(
                            false,
                            true,
                            row_r,
                            row_l,
                            "[place] 第六排侧区左手");
                    }
                }
                else
                {
                    cout << "[place] 侧区双手不同排：先第" << std::max(row_l, row_r) + 1
                         << "排，后第" << std::min(row_l, row_r) + 1 << "排\n";
                    if (row_r > row_l)
                    {
                        run_side_place_phase(true, false, row_r, row_l, "[place] 远排侧区右手放完");
                        run_side_place_phase(false, true, row_r, row_l, "[place] 近排侧区左手放完");
                    }
                    else
                    {
                        run_side_place_phase(false, true, row_r, row_l, "[place] 远排侧区左手放完");
                        run_side_place_phase(true, false, row_r, row_l, "[place] 近排侧区右手放完");
                    }
                }
            }
            else
            {
                const bool both_row6 =
                    act_l && act_r && row_l >= 0 && row_r >= 0 &&
                    row6_bend_enabled_for_row(row_l) && row6_bend_enabled_for_row(row_r);
                if (both_row6)
                {
                    run_row6_dual_place_sequential(
                        row_r, row_l, side_pick_l, side_pick_r, "[place] 侧区");
                }
                else
                {
                    run_side_place_phase(act_r, act_l, row_r, row_l, "[place] 侧区放完");
                }
            }
        }

        bool hold_l = grasped_l && !move_side_l;
        bool hold_r = grasped_r && !move_side_r;

        auto place_middle_fallback_one = [&](bool is_right) -> bool
        {
            if (middle_idx.empty())
                return false;
            const auto hand_indices = filter_place_indices_for_hand(grid, middle_idx, is_right, y_split);
            if (hand_indices.empty())
            {
                cout << "[place] 中间带无" << (is_right ? "右手" : "左手")
                     << "可用格(y" << (is_right ? "<" : ">") << y_split << ")\n";
                return false;
            }
            const auto pick = pick_place_slot_row_ordered(grid, hand_indices, row_lookup);
            if (!pick)
                return false;
            const int pick_row = grid_row_0(*pick);
            const auto &p = grid[*pick];
            const char *hand = is_right ? "右手" : "左手";
            if (is_right)
            {
                apply_place_goal_xy_right(st.goal_last_r, p(0), p(1), pick_row);
                place_release_one(
                    true,
                    false,
                    pick_row,
                    pick_row,
                    std::nullopt,
                    *pick,
                    "[place] 中间带兜底右手");
            }
            else
            {
                apply_place_goal_xy_left(st.goal_last_l, p(0), p(1), pick_row);
                place_release_one(
                    false,
                    true,
                    pick_row,
                    pick_row,
                    *pick,
                    std::nullopt,
                    "[place] 中间带兜底左手");
            }
            cout << "[place] 侧区无格，中间带兜底" << hand << " idx=" << *pick << " xy=(" << p(0)
                 << "," << p(1) << ") 排=" << pick_row << "\n";
            return true;
        };

        while (!middle_idx.empty() && (hold_l || hold_r))
        {
            if (hold_r && hold_l)
            {
                const auto mid_r = filter_place_indices_for_hand(grid, middle_idx, true, y_split);
                const auto mid_l = filter_place_indices_for_hand(grid, middle_idx, false, y_split);
                const auto peek_r = pick_place_slot_row_ordered(grid, mid_r, row_lookup);
                const auto peek_l = pick_place_slot_row_ordered(grid, mid_l, row_lookup);
                const int r_row = peek_r ? grid_row_0(*peek_r) : -1;
                const int l_row = peek_l ? grid_row_0(*peek_l) : -1;

                if (peek_r && peek_l && r_row >= 0 && l_row >= 0 &&
                    row6_bend_enabled_for_row(r_row) && row6_bend_enabled_for_row(l_row))
                {
                    apply_place_goal_xy_right(
                        st.goal_last_r, grid[*peek_r](0), grid[*peek_r](1), r_row);
                    apply_place_goal_xy_left(
                        st.goal_last_l, grid[*peek_l](0), grid[*peek_l](1), l_row);
                    cout << "[place] 中间带兜底第六排双手 idx_r=" << *peek_r << " idx_l=" << *peek_l
                         << "\n";
                    run_row6_dual_place_sequential(
                        r_row, l_row, *peek_l, *peek_r, "[place] 中间带兜底");
                    hold_r = false;
                    hold_l = false;
                }
                else if (peek_r && peek_l && r_row >= 0 && l_row >= 0 && r_row != l_row &&
                         l_row > r_row)
                {
                    if (!place_middle_fallback_one(false))
                        hold_l = false;
                    if (hold_r && !middle_idx.empty())
                    {
                        if (!place_middle_fallback_one(true))
                            hold_r = false;
                    }
                }
                else
                {
                    if (!place_middle_fallback_one(true))
                        hold_r = false;
                    if (hold_l && !middle_idx.empty())
                    {
                        if (!place_middle_fallback_one(false))
                            hold_l = false;
                    }
                }
            }
            else if (hold_r)
            {
                if (!place_middle_fallback_one(true))
                    hold_r = false;
            }
            else if (hold_l)
            {
                if (!place_middle_fallback_one(false))
                    hold_l = false;
            }
        }

        if (grasped_r && !released_r)
        {
            cout << "[place] 右手无合适空位或未识别到格，放 config fallback\n";
            place_release_config_fallback(true, false, "[place] 右手config fallback");
        }
        if (grasped_l && !released_l)
        {
            cout << "[place] 左手无合适空位或未识别到格，放 config fallback\n";
            place_release_config_fallback(false, true, "[place] 左手config fallback");
        }

        if ((grasped_l || grasped_r) && !placed_any)
        {
            std::cerr << "[place] 有抓取但未成功放置任何货物（含 config fallback）\n";
            if (row6_session_bent)
            {
                row6_bend_waist_out_if_needed();
                row6_session_bent = false;
            }
            row6_shoulder_prepared = false;
            row6_pending_dual_middle = false;
            if (!row6_layer3_restored_in_session)
                restore_waist_layer3_x(waist, posup_down_3_layer);
            waist_adjusted = false;
            move_arms_to_standby(arm_r, arm_l);
            return false;
        }

        if (row6_session_bent)
        {
            row6_bend_waist_out_if_needed();
            row6_session_bent = false;
        }
        row6_shoulder_prepared = false;
        row6_pending_dual_middle = false;

        if (!row6_layer3_restored_in_session)
        {
            restore_waist_layer3_x(waist, posup_down_3_layer);
            waist_adjusted = false;
        }
        cout << "[place] 放货流程结束，腰 layer3 已恢复\n";

        chassis_rotate_or_exit(ctrl, -90, "放货后");
        sleep(1);
        return true;
    }
#endif

    /** ArUco 试验退出：Py_Finalize 后 librealsense2 全局析构会在 exit() 里 segfault，用 _exit 跳过 */
    void aruco_test_exit(int code)
    {
        std::cout.flush();
        std::cerr.flush();
        _exit(code);
    }


} // namespace move_box

namespace row6_experiment
{

    void row6_wait_stage(const char *msg)
    {
        std::cout << "[row6] " << msg << "，按回车继续..." << std::endl;
        std::string line;
        std::getline(std::cin, line);
    }

    constexpr double kWaistPivotZBelowBaseM = 0.40;
    constexpr uint32_t kWaistPitchMotorCanId = 2;
    constexpr int kRightShoulderJointIndex = 0; // CAN 16
    constexpr int kLeftShoulderJointIndex = 0;  // CAN 23
    constexpr int kRow6BendSmoothDtMs = 20;

    double quintic_scalar_interp(double t, double total_time, double start, double target)
    {
        if (total_time <= 1e-6)
            return target;
        const double ratio = std::clamp(t / total_time, 0.0, 1.0);
        const double r2 = ratio * ratio;
        const double r3 = r2 * ratio;
        const double r4 = r3 * ratio;
        const double r5 = r4 * ratio;
        const double pos_term = 10.0 * r3 - 15.0 * r4 + 6.0 * r5;
        return start + (target - start) * pos_term;
    }

    bool read_motor_encoder_cnt(int channel, uint32_t can_id, int &out_cnt)
    {
        int data[1] = {0};
        uint32_t id = can_id;
        if (socketcan_sendsimplecommand(channel, 1, &id, 8, data) != 1)
            return false;
        out_cnt = data[0];
        return true;
    }

    void send_motor_encoder_cnt(int channel, uint32_t can_id, int cnt)
    {
        int data[1] = {cnt};
        uint32_t id = can_id;
        socketcan_sendcommand(channel, 1, &id, 30, data);
    }

    bool wait_waist_motor_reached(
        uint32_t can_id,
        int target_cnt,
        double timeout_sec,
        const char *log_tag)
    {
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                  std::chrono::duration<double>(timeout_sec));
        while (std::chrono::steady_clock::now() < deadline)
        {
            int actual = 0;
            if (!read_motor_encoder_cnt(waist_id, can_id, actual))
                return false;
            const int err = actual - target_cnt;
            if (std::abs(err) <= 550)
            {
                std::cout << "[waist] " << log_tag << " 到位 can=" << can_id << " err=" << err << "\n";
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        std::cerr << "[waist] " << log_tag << " 超时 can=" << can_id << " target=" << target_cnt << "\n";
        return false;
    }

    /** 直立 base 目标 → 弯腰后 base 系：枢轴 (0,-0.40)，前倒 pitch 时前方点 x↓(更近) z↑ */
    Eigen::Matrix<double, 1, 6> compensate_tcp_for_waist_pitch_forward(
        const Eigen::Matrix<double, 1, 6> &goal_upright,
        double pitch_deg)
    {
        const double th = pitch_deg * M_PI / 180.0;
        const double c = std::cos(th);
        const double s = std::sin(th);
        const double px = 0.0;
        const double pz = -kWaistPivotZBelowBaseM;
        const double x = goal_upright(0) - px;
        const double z = goal_upright(2) - pz;
        Eigen::Matrix<double, 1, 6> out = goal_upright;
        // 固定空间点：腰 base 前倒 +θ → 点在弯后 base 里 x 更近、z 更高
        out(0) = px + x * c - z * s;
        out(2) = pz + x * s + z * c;
        return out;
    }

    Eigen::Matrix<double, 1, 6> make_row6_test_goal_right()
    {
        Eigen::Matrix<double, 1, 6> g = g_move_cfg.place.fallback_right;
        g(0) = 0.8;
        g(1) = -0.18;
        return g;
    }

    Eigen::Matrix<double, 1, 6> make_row6_test_goal_left()
    {
        Eigen::Matrix<double, 1, 6> g = g_move_cfg.place.fallback_left;
        g(0) = 0.8;
        g(1) = 0.18;
        return g;
    }

    void apply_row6_bent_orientation(Eigen::Matrix<double, 1, 6> &goal, bool is_right)
    {
        goal(3) = (is_right ? -90.0 : 90.0) * rad;
        goal(4) = 25.0 * rad;
        goal(5) = 0.0;
    }

    void apply_row6_place_xy_offsets(Eigen::Matrix<double, 1, 6> &goal, bool is_right)
    {
        const auto &p = place_hand_xy_offset(5, is_right);
        goal(0) += p.offset_x;
        goal(1) += p.offset_y;
    }

    /** 沿世界竖直位移 h(m)：弯后 base 下 Δx=-h·sin(pitch)，Δz=+h·cos(pitch) */
    void apply_row6_world_vertical_in_bent_base(
        Eigen::Matrix<double, 1, 6> &goal,
        double vertical_h_m,
        double pitch_deg)
    {
        const double th = pitch_deg * M_PI / 180.0;
        goal(0) -= vertical_h_m * std::sin(th);
        goal(2) += vertical_h_m * std::cos(th);
    }

    void row6_trace_sep()
    {
        std::cout << "=====\n=====\n=====\n";
    }

    void row6_trace_pose6(const char *label, const Eigen::Matrix<double, 1, 6> &p)
    {
        std::cout << "  " << label << " xyz=(" << p(0) << ", " << p(1) << ", " << p(2) << ")"
                  << " rxryrz(rad)=(" << p(3) << ", " << p(4) << ", " << p(5) << ")\n";
    }

    void row6_trace_pose_change(
        const char *hand,
        const char *step,
        const Eigen::Matrix<double, 1, 6> &before,
        const Eigen::Matrix<double, 1, 6> &after,
        const char *formula)
    {
        std::cout << "  [" << hand << "] " << step << "\n";
        std::cout << "    变化前 xyz=(" << before(0) << ", " << before(1) << ", " << before(2) << ")\n";
        std::cout << "    变化后 xyz=(" << after(0) << ", " << after(1) << ", " << after(2) << ")\n";
        std::cout << "    Δx=" << (after(0) - before(0)) << " Δy=" << (after(1) - before(1))
                  << " Δz=" << (after(2) - before(2)) << "\n";
        std::cout << "    计算: " << formula << "\n";
    }

    void row6_trace_section(const char *title)
    {
        row6_trace_sep();
        std::cout << "[row6/trace] " << title << "\n";
    }

    /** 仅肩 16/23 quintic（右+Δ，左-Δ） */
    bool smooth_row6_shoulder_only(double delta_deg, double speed_deg_per_s, int dt_ms)
    {
        if (speed_deg_per_s <= 1e-6)
            speed_deg_per_s = kRow6BendSpeedDegPerS;
        if (dt_ms <= 0)
            dt_ms = kRow6BendSmoothDtMs;

        int pos_r[7] = {0};
        int pos_l[7] = {0};
        get_motor_position(pos_r, 1);
        get_motor_position(pos_l, 0);

        const double r_sh_start = motor_cnt_to_angle_deg(pos_r[kRightShoulderJointIndex]);
        const double r_sh_tgt = r_sh_start + delta_deg;
        const double l_sh_start = motor_cnt_to_angle_deg(pos_l[kLeftShoulderJointIndex]);
        const double l_sh_tgt = l_sh_start - delta_deg;

        std::cout << "[row6] 肩先动 右16: " << r_sh_start << "°→" << r_sh_tgt << "°"
                  << " 左23: " << l_sh_start << "°→" << l_sh_tgt << "°"
                  << " Δ=" << delta_deg << "°\n";

        pos_r[kRightShoulderJointIndex] = motor_angle_deg_to_cnt(r_sh_tgt);
        pos_l[kLeftShoulderJointIndex] = motor_angle_deg_to_cnt(l_sh_tgt);

        const double abs_delta = std::abs(delta_deg);
        if (abs_delta < 0.01)
        {
            set_motor_position(pos_r, 1);
            set_motor_position(pos_l, 0);
        }
        else
        {
            const double total_time_s = abs_delta / speed_deg_per_s;
            const double dt_s = static_cast<double>(dt_ms) / 1000.0;
            double t = 0.0;
            while (t < total_time_s)
            {
                const double r_deg = quintic_scalar_interp(t, total_time_s, r_sh_start, r_sh_tgt);
                const double l_deg = quintic_scalar_interp(t, total_time_s, l_sh_start, l_sh_tgt);
                int cmd_r[7];
                int cmd_l[7];
                for (int i = 0; i < 7; ++i)
                {
                    cmd_r[i] = pos_r[i];
                    cmd_l[i] = pos_l[i];
                }
                cmd_r[kRightShoulderJointIndex] = motor_angle_deg_to_cnt(r_deg);
                cmd_l[kLeftShoulderJointIndex] = motor_angle_deg_to_cnt(l_deg);
                set_motor_position(cmd_r, 1);
                set_motor_position(cmd_l, 0);
                usleep(static_cast<useconds_t>(dt_ms * 1000));
                t += dt_s;
            }
            set_motor_position(pos_r, 1);
            set_motor_position(pos_l, 0);
        }

        save_arm_last_commanded_position(1, pos_r);
        save_arm_last_commanded_position(0, pos_l);
        return wait_arms_motors_stopped(true, true, 15.0, 6, "肩补偿");
    }

    /** 仅腰 2 号 pitch quintic */
    bool smooth_row6_waist_pitch_only(double bend_deg, double speed_deg_per_s, int dt_ms)
    {
        if (speed_deg_per_s <= 1e-6)
            speed_deg_per_s = kRow6BendSpeedDegPerS;
        if (dt_ms <= 0)
            dt_ms = kRow6BendSmoothDtMs;

        int waist_cur = 0;
        if (!read_motor_encoder_cnt(waist_id, kWaistPitchMotorCanId, waist_cur))
        {
            std::cerr << "[row6] 读腰 2 号电机失败\n";
            return false;
        }

        const double waist_start = motor_cnt_to_angle_deg(waist_cur);
        const double waist_tgt = waist_start + bend_deg;
        const int waist_tgt_cnt = motor_angle_deg_to_cnt(waist_tgt);

        std::cout << "[row6] 腰再弯 腰2: " << waist_start << "°→" << waist_tgt << "°"
                  << " Δ=" << bend_deg << "°\n";

        const double abs_bend = std::abs(bend_deg);
        if (abs_bend < 0.01)
        {
            send_motor_encoder_cnt(waist_id, kWaistPitchMotorCanId, waist_tgt_cnt);
        }
        else
        {
            const double total_time_s = abs_bend / speed_deg_per_s;
            const double dt_s = static_cast<double>(dt_ms) / 1000.0;
            double t = 0.0;
            while (t < total_time_s)
            {
                const double w_deg = quintic_scalar_interp(t, total_time_s, waist_start, waist_tgt);
                send_motor_encoder_cnt(waist_id, kWaistPitchMotorCanId, motor_angle_deg_to_cnt(w_deg));
                usleep(static_cast<useconds_t>(dt_ms * 1000));
                t += dt_s;
            }
            send_motor_encoder_cnt(waist_id, kWaistPitchMotorCanId, waist_tgt_cnt);
        }

        const char *tag = bend_deg >= 0 ? "腰弯腰" : "腰回正";
        return wait_waist_motor_reached(kWaistPitchMotorCanId, waist_tgt_cnt, 15.0, tag);
    }

    /** 肩先动 shoulder_Δ，再腰弯 waist_Δ；回正时先腰后肩（肩撤销 -shoulder_Δ） */
    bool perform_row6_waist_bend_with_shoulder_lift(
        double waist_bend_deg,
        double shoulder_delta_deg,
        double speed_deg_per_s,
        int dt_ms)
    {
        if (waist_bend_deg >= 0)
        {
            std::cout << "[row6] 弯腰: 肩 +" << shoulder_delta_deg << "°，腰 +" << waist_bend_deg << "°\n";
            if (!smooth_row6_shoulder_only(shoulder_delta_deg, speed_deg_per_s, dt_ms))
                return false;
            return smooth_row6_waist_pitch_only(waist_bend_deg, speed_deg_per_s, dt_ms);
        }
        std::cout << "[row6] 回正: 腰 " << waist_bend_deg << "°，肩 -" << shoulder_delta_deg << "°\n";
        if (!smooth_row6_waist_pitch_only(waist_bend_deg, speed_deg_per_s, dt_ms))
            return false;
        return smooth_row6_shoulder_only(-shoulder_delta_deg, speed_deg_per_s, dt_ms);
    }

    bool run_row6_place_experiment(
        gripper::Gripper &g,
        Robot_Arm &arm_r,
        Robot_Arm &arm_l,
        WaistRobot &waist,
        Eigen::Matrix<double, 1, 6> &pos_layer3)
    {
        using move_box::configured_row_waist_x;
        using move_box::move_waist_to_place_ready_x;
        using move_box::move_arms_to_standby;
        using move_box::restore_waist_layer3_x;
        using move_box::waist_layer3_apply_place_x_delta;
        using move_box::waist_x_arm_goal_comp;

        constexpr int kRow6Index = 5;
        constexpr double kRow6SlotX = 0.8;
        constexpr double kRow6SlotYRight = -0.18;
        constexpr double kRow6SlotYLeft = 0.18;

        std::cout << "[row6] 模拟夹持：双手夹爪闭合\n";
        g.graspAndWait(gripper::Side::Right);
        g.graspAndWait(gripper::Side::Left);
        row6_wait_stage("阶段4: 夹爪已夹住");

        Eigen::Matrix<double, 1, 6> trace_r = make_row6_test_goal_right();
        Eigen::Matrix<double, 1, 6> trace_l = make_row6_test_goal_left();
        trace_r(1) = kRow6SlotYRight;
        trace_l(1) = kRow6SlotYLeft;
        row6_trace_section("0) 名义第六排格点 (place.fallback 姿态 + x/y)");
        row6_trace_pose6("右手", trace_r);
        row6_trace_pose6("左手", trace_l);
        std::cout << "  来源: rx,ry,rz=yaml place.fallback; x=" << kRow6SlotX
                  << " y=±" << kRow6SlotYRight << "/" << kRow6SlotYLeft << "\n";

        std::cout << "[row6] 腰到放货预备位 (place_advance)\n";
        const double layer3_x_before_ready = pos_layer3(0);
        move_waist_to_place_ready_x(waist, pos_layer3);
        row6_trace_section("1) 放货预备位 place_advance");
        std::cout << "  腰 layer3 x: " << layer3_x_before_ready << " → " << pos_layer3(0)
                  << " (home.x - place_advance_x = " << g_move_cfg.waist.layer3_home(0) << " - ("
                  << g_move_cfg.waist.place_advance_x << "))\n";
        std::cout << "  说明: 仅腰 x 动，手臂目标 xyz 此步不变\n";
        row6_wait_stage("阶段5: 腰已到放货预备位");

        const double row_waist_x = configured_row_waist_x(kRow6Index);
        const Eigen::Matrix<double, 1, 6> before_waist_r = trace_r;
        const Eigen::Matrix<double, 1, 6> before_waist_l = trace_l;
        const double layer3_x_before_row6 = pos_layer3(0);
        std::cout << "[row6] 第6排 x=" << kRow6SlotX << " 触发 row_waist_x=" << row_waist_x << "\n";
        const double waist_actual =
            waist_layer3_apply_place_x_delta(waist, pos_layer3, row_waist_x, "[row6] 第6排腰进");
        const double arm_x_comp = waist_x_arm_goal_comp(waist_actual);
        const double slot_x = kRow6SlotX + arm_x_comp;
        trace_r(0) = slot_x;
        trace_l(0) = slot_x;
        {
            std::ostringstream f;
            f << "slot_x = 0.8 + waist_x_arm_goal_comp(" << waist_actual << ") = 0.8 + ("
              << arm_x_comp << ") = " << slot_x;
            row6_trace_section("2) 第6排腰进 row_waist_x.row6");
            std::cout << "  腰 layer3 x: " << layer3_x_before_row6 << " → " << pos_layer3(0)
                      << " (pos(0) -= row_waist_x, actual_delta=" << waist_actual << ")\n";
            row6_trace_pose_change("右手", "腰进后手臂 x 补偿", before_waist_r, trace_r, f.str().c_str());
            row6_trace_pose_change("左手", "腰进后手臂 x 补偿", before_waist_l, trace_l, f.str().c_str());
        }
        row6_wait_stage("阶段6: 第6排腰进完成，手臂 x 已补偿");

        std::cout << "[row6] 肩先 +" << kRow6ShoulderLiftDeg << "°，腰再弯 +" << kRow6WaistBendDeg << "°\n";
        if (!perform_row6_waist_bend_with_shoulder_lift(kRow6WaistBendDeg, kRow6ShoulderLiftDeg))
        {
            std::cerr << "[row6] 弯腰失败\n";
            return false;
        }
        row6_trace_section("3) 肩+腰弯 (肩36° 腰20°)");
        std::cout << "  顺序: 肩16/23 +" << kRow6ShoulderLiftDeg << "°，腰2 +" << kRow6WaistBendDeg << "°\n";
        std::cout << "  说明: 下面 pitch 补偿把直立 base 系 xyz 换算到弯腰 base 系\n";
        row6_wait_stage("阶段7: 肩+腰弯完成");

        Eigen::Matrix<double, 1, 6> upright_r = trace_r;
        Eigen::Matrix<double, 1, 6> upright_l = trace_l;
        const Eigen::Matrix<double, 1, 6> before_pitch_r = upright_r;
        const Eigen::Matrix<double, 1, 6> before_pitch_l = upright_l;
        Eigen::Matrix<double, 1, 6> goal_r = compensate_tcp_for_waist_pitch_forward(upright_r, kRow6WaistBendDeg);
        Eigen::Matrix<double, 1, 6> goal_l = compensate_tcp_for_waist_pitch_forward(upright_l, kRow6WaistBendDeg);
        row6_trace_section("4) 弯腰 pitch 补偿 xyz");
        std::cout << "  枢轴: base 下方 z=-0.40m，前倒 pitch=" << kRow6WaistBendDeg << "°\n";
        row6_trace_pose_change(
            "右手",
            "pitch 补偿",
            before_pitch_r,
            goal_r,
            "compensate_tcp_for_waist_pitch_forward(直立xyz, pitch=20°): 弯后 base 里 x↓(更近) z↑，y 不变");
        row6_trace_pose_change(
            "左手",
            "pitch 补偿",
            before_pitch_l,
            goal_l,
            "同上");

        apply_row6_bent_orientation(goal_r, true);
        apply_row6_bent_orientation(goal_l, false);
        row6_trace_section("4b) 弯腰后下发姿态 (非 fallback ry=45)");
        std::cout << "  右: rx,ry,rz = -90°, 25°, 0°\n";
        std::cout << "  左: rx,ry,rz =  90°, 25°, 0°\n";
        row6_trace_pose6("右手(弯后姿态)", goal_r);
        row6_trace_pose6("左手(弯后姿态)", goal_l);

        const Eigen::Matrix<double, 1, 6> before_xy_r = goal_r;
        const Eigen::Matrix<double, 1, 6> before_xy_l = goal_l;
        apply_row6_place_xy_offsets(goal_r, true);
        apply_row6_place_xy_offsets(goal_l, false);
        row6_trace_section("5) place xy 偏移");
        {
            std::ostringstream f_r, f_l;
            const auto &p_r = place_hand_xy_offset(5, true);
            const auto &p_l = place_hand_xy_offset(5, false);
            f_r << "x+=" << p_r.offset_x << " y+=" << p_r.offset_y;
            f_l << "x+=" << p_l.offset_x << " y+=" << p_l.offset_y;
            row6_trace_pose_change("右手", "place xy", before_xy_r, goal_r, f_r.str().c_str());
            row6_trace_pose_change("左手", "place xy", before_xy_l, goal_l, f_l.str().c_str());
        }

        const Eigen::Matrix<double, 1, 6> before_raise_r = goal_r;
        const Eigen::Matrix<double, 1, 6> before_raise_l = goal_l;
        const double z_raise_h = g_move_cfg.place.z_raise;
        apply_row6_world_vertical_in_bent_base(goal_r, z_raise_h, kRow6WaistBendDeg);
        apply_row6_world_vertical_in_bent_base(goal_l, z_raise_h, kRow6WaistBendDeg);
        {
            const double th = kRow6WaistBendDeg * M_PI / 180.0;
            std::ostringstream f;
            f << "世界竖直抬高 h=" << z_raise_h << "m → Δx=-h·sin(" << kRow6WaistBendDeg
              << "°)=" << (-z_raise_h * std::sin(th)) << " Δz=+h·cos=" << (z_raise_h * std::cos(th));
            row6_trace_section("6) z_raise 世界竖直抬高 (弯后 base 联动 x/z)");
            row6_trace_pose_change("右手", "z_raise", before_raise_r, goal_r, f.str().c_str());
            row6_trace_pose_change("左手", "z_raise", before_raise_l, goal_l, f.str().c_str());
        }

        Eigen::Matrix<double, 1, 6> descend_r = goal_r;
        Eigen::Matrix<double, 1, 6> descend_l = goal_l;
        const double z_descend_h = g_move_cfg.place.z_descend;
        apply_row6_world_vertical_in_bent_base(descend_r, z_descend_h, kRow6WaistBendDeg);
        apply_row6_world_vertical_in_bent_base(descend_l, z_descend_h, kRow6WaistBendDeg);
        {
            const double th = kRow6WaistBendDeg * M_PI / 180.0;
            const double h = z_descend_h;
            std::ostringstream f;
            f << "世界竖直 h=" << h << "m (yaml z_descend) → Δx=-h·sin=" << (-h * std::sin(th))
              << " Δz=+h·cos=" << (h * std::cos(th)) << " (h<0 即下降: x↑ z↓)";
            row6_trace_section("7) z_descend 世界竖直下压 (松爪前)");
            row6_trace_pose_change("右手", "z_descend", goal_r, descend_r, f.str().c_str());
            row6_trace_pose_change("左手", "z_descend", goal_l, descend_l, f.str().c_str());
        }
        row6_trace_sep();

        std::cout << "[row6] 执行: 先到抬高位 → z_descend → 松爪\n";
        row6_wait_stage("阶段8: 即将直线到放货抬高位");

        {
            ArmLineMoveDebugStage stage("第六排放货抬高位");
            const int ret_r = arm_line_move(arm_r, goal_r, 0.2);
            const int ret_l = arm_line_move(arm_l, goal_l, 0.2);
            if (ret_r != 0 || ret_l != 0)
            {
                std::cerr << "[row6] 抬高位轨迹失败 ret_r=" << ret_r << " ret_l=" << ret_l << "\n";
                return false;
            }
        }
        if (!wait_arms_motors_stopped(true, true, 15.0, 6, "放货抬高位"))
            return false;
        row6_wait_stage("阶段9: 已到抬高位，即将 z_descend");

        {
            ArmLineMoveDebugStage stage("第六排放货z下压");
            const int ret_r = arm_line_move(arm_r, descend_r, 0.2);
            const int ret_l = arm_line_move(arm_l, descend_l, 0.2);
            if (ret_r != 0 || ret_l != 0)
            {
                std::cerr << "[row6] z下压轨迹失败 ret_r=" << ret_r << " ret_l=" << ret_l << "\n";
                return false;
            }
        }
        if (!wait_arms_motors_stopped(true, true, 15.0, 6, "放货z下压"))
            return false;
        row6_wait_stage("阶段10: z下压到位，即将松爪");

        g.openAndWait(gripper::Side::Left);
        g.openAndWait(gripper::Side::Right);
        std::cout << "[row6] 夹爪已松开\n";
        row6_wait_stage("阶段11: 已松爪，腰回正后双手回 standby");

        std::cout << "[row6] 仅腰2回正 (肩不单独回正，由回 standby 一并带动)\n";
        if (!smooth_row6_waist_pitch_only(-kRow6WaistBendDeg, kRow6BendSpeedDegPerS, kRow6BendSmoothDtMs))
        {
            std::cerr << "[row6] 腰回正失败\n";
            return false;
        }
        row6_wait_stage("阶段12: 腰已回正，layer3 home + 双臂 standby");

        Eigen::Matrix<double, 1, 6> standby_r = move_box::make_standby_pos_right();
        Eigen::Matrix<double, 1, 6> standby_l = move_box::make_standby_pos_left();
        std::cout << "[row6] 双臂直线回 yaml standby 右=" << standby_r << "\n";
        std::cout << "[row6] 双臂直线回 yaml standby 左=" << standby_l << "\n";

        std::thread t_waist_home([&]() { restore_waist_layer3_x(waist, pos_layer3); });
        {
            ArmLineMoveDebugStage stage("放货后回standby");
            arm_dual_line_move(arm_r, standby_r, arm_l, standby_l, 0.2);
        }
        t_waist_home.join();

        if (!wait_arms_motors_stopped(true, true, 15.0, 6, "回待机"))
            return false;

        std::cout << "[row6] TCP 右=" << arm_get_tcp_pos(arm_r) << "\n";
        std::cout << "[row6] TCP 左=" << arm_get_tcp_pos(arm_l) << "\n";
        std::cout << "[row6] layer3 x=" << pos_layer3(0) << " (home="
                  << g_move_cfg.waist.layer3_home(0) << ")\n";
        row6_wait_stage("阶段13: 全流程完成");
        return true;
    }



} // namespace row6_experiment
