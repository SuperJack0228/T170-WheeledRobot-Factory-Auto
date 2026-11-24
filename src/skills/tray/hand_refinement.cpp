#include "../internal/execution.h"
#include "t170/vision/tray_localizer.h"
#include "../internal/legacy_helpers.h"
#include "t170/motion/servo.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <numeric>
#include <optional>
#include <random>
#include <sstream>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "t170/vision/wrist_observer.h"
#include "t170/motion/arm.h"
#include "t170/common/units.h"
#include "t170/motion/cartesian_tracking.h"
#include "t170/motion/feedback.h"
#include "t170/motion/kinematics_adapter.h"
#include "t170/motion/retreat.h"
#include "t170/motion/servo_handoff.h"
#include <array>
#include <cstring>
#include "t170/vision/head_transform.h"
#include "t170/hardware/device_bus.h"

using std::vector;
using std::cout;
using std::endl;
using std::thread;


namespace move_box {
double clamp_signed_abs(double v, double max_abs)
    {
        if (v > max_abs)
            return max_abs;
        if (v < -max_abs)
            return -max_abs;
        return v;
    }

void apply_hand_xy_refine_keep_hole(
        const char *hand,
        bool is_right,
        int from_robot_row,
        const Eigen::Matrix<double, 1, 6> &base_pos,
        Eigen::Matrix<double, 1, 6> &goal_last)
    {
        const auto &hg = g_move_cfg.head_grasp;
        const double maxd = hg.hand_xy_refine_max_m;

        const double hole_x = goal_last(0) - active_goal_x_offset(is_right);
        const double hole_y = goal_last(1) - active_goal_y_offset(is_right);
        if (!(maxd > 0.0))
        {
            std::cout << std::fixed << std::setprecision(4)
                      << "[hand_xy] " << hand << " 微调关闭，XY 锁定孔位 ("
                      << hole_x << "," << hole_y << ")\n";
            return;
        }
        const double raw_dx = base_pos(0) - hole_x;
        const double raw_dy = base_pos(1) - hole_y;
        const double dx = clamp_signed_abs(raw_dx, maxd);
        const double dy = clamp_signed_abs(raw_dy, maxd);
        goal_last(0) += dx;
        goal_last(1) += dy;
        std::cout << std::fixed << std::setprecision(4)
                  << "[hand_xy] " << hand
                  << " 孔=(" << hole_x << "," << hole_y
                  << ") 手=(" << base_pos(0) << "," << base_pos(1)
                  << ") 原差=(" << raw_dx << "," << raw_dy
                  << ") 采用=(" << dx << "," << dy
                  << ") → goal=(" << goal_last(0) << "," << goal_last(1) << ")\n";
    }

void apply_hand_base_to_goal(
        bool have_r,
        const Eigen::Matrix<double, 1, 6> &base_pos_r,
        bool have_l,
        const Eigen::Matrix<double, 1, 6> &base_pos_l,
        Eigen::Matrix<double, 1, 6> &goal_last_r,
        Eigen::Matrix<double, 1, 6> &goal_last_l,
        bool hover_z_from_hand,
        bool keep_xy,
        int row_r,
        int row_l)
    {
        (void)hover_z_from_hand;
        const auto &hg = g_move_cfg.head_grasp;
        if (have_r)
        {
            if (!keep_xy)
            {
                goal_last_r(0) = base_pos_r(0) + hg.hand_right.offset_x;
                goal_last_r(1) = base_pos_r(1) + hg.hand_right.offset_y;
            }
            else
            {
                apply_hand_xy_refine_keep_hole("右", true, row_r, base_pos_r, goal_last_r);
            }
            apply_hand_hover_z_if_valid(base_pos_r, goal_last_r);
            std::cout << std::fixed << std::setprecision(4)
                      << "[hand_z] 右 hand_object_z=" << base_pos_r(2)
                      << " hover_z=" << goal_last_r(2)
                      << (keep_xy ? " (XY 孔位+手相机限幅)\n" : " m\n");
        }
        if (have_l)
        {
            if (!keep_xy)
            {
                goal_last_l(0) = base_pos_l(0) + hg.hand_left.offset_x;
                goal_last_l(1) = base_pos_l(1) + hg.hand_left.offset_y;
            }
            else
            {
                apply_hand_xy_refine_keep_hole("左", false, row_l, base_pos_l, goal_last_l);
            }
            apply_hand_hover_z_if_valid(base_pos_l, goal_last_l);
            std::cout << std::fixed << std::setprecision(4)
                      << "[hand_z] 左 hand_object_z=" << base_pos_l(2)
                      << " hover_z=" << goal_last_l(2)
                      << (keep_xy ? " (XY 孔位+手相机限幅)\n" : " m\n");
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

bool select_hand_target_matching_head(
        const PoseDetectionRecords &records,
        const std::array<double, 16> &cam2robot,
        Arm &arm,
        const Eigen::Vector2d &expected_head_xy,
        const char *hand_label,
        bool is_right,
        Eigen::Matrix<double, 1, 6> &out_hand_pose,
        Eigen::Matrix<double, 1, 6> &out_base_pose)
    {
        out_hand_pose << -1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        out_base_pose << -1.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        bool found = false;
        double best_xy = std::numeric_limits<double>::infinity();
        Eigen::Matrix<double, 1, 6> best_hand;
        Eigen::Matrix<double, 1, 6> best_base;

        for (size_t i = 0; i < records.size(); ++i)
        {
            Eigen::Matrix<double, 1, 6> hand_pose;
            if (!transform_grasp_detection_to_pose(records[i], cam2robot, hand_pose))
                continue;
            const Eigen::Matrix<double, 1, 6> base_pose =
                arm_get_base_visual_pos(arm, hand_pose);
            if (!std::isfinite(base_pose(0)) || !std::isfinite(base_pose(1)) ||
                !std::isfinite(base_pose(2)))
                continue;

            const double dx = base_pose(0) - expected_head_xy.x();
            const double dy = base_pose(1) - expected_head_xy.y();
            const double xy = std::hypot(dx, dy);
            const bool zone_ok = is_right
                ? goal_pos_valid_right(base_pose(0), base_pose(1), base_pose(2))
                : goal_pos_valid_left(base_pose(0), base_pose(1), base_pose(2));
            std::cout << std::fixed << std::setprecision(4)
                      << "[hand_match] " << hand_label << " cand=" << i
                      << " base_xyz=(" << base_pose(0) << ',' << base_pose(1) << ','
                      << base_pose(2) << ") col=" << tray_column_index_from_y(base_pose(1))
                      << " expected_xy=(" << expected_head_xy.x() << ','
                      << expected_head_xy.y() << ") dxy=" << xy << "m";
            if (!zone_ok)
            {
                std::cout << " [跳过:超工作包络 y=" << base_pose(1) << "]\n";
                continue;
            }
            std::cout << "\n";
            if (!found || xy < best_xy)
            {
                found = true;
                best_xy = xy;
                best_hand = hand_pose;
                best_base = base_pose;
            }
        }

        if (!found)
        {
            std::cout << "[hand_match] " << hand_label << " 无本侧列分区内抓取候选\n";
            return false;
        }

        const double max_xy = g_move_cfg.head_grasp.hand_match_xy_max_m;
        if (best_xy > max_xy)
        {
            std::cout << std::fixed << std::setprecision(4)
                      << "[hand_match] " << hand_label << " 拒绝：最近候选 dxy=" << best_xy
                      << "m > " << max_xy << "m，避免切换到另一圆柱\n";
            return false;
        }

        out_hand_pose = best_hand;
        out_base_pose = best_base;
        std::cout << std::fixed << std::setprecision(4)
                  << "[hand_match] " << hand_label << " 接受同一目标 dxy=" << best_xy
                  << "m base_xyz=(" << best_base(0) << ',' << best_base(1) << ','
                  << best_base(2) << ")\n";
        return true;
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
        if (ret == -31 || ret == -35 || ret == -36)
            std::cerr << "[arm] " << hand << "下降伺服中止 ret=" << ret
                      << "，原因见 logs/visual_servo\n";
        else
            std::cerr << "[arm] " << hand << "轨迹解析失败 ret=" << ret << "\n";
        // 详细记录（goal/stage）由 arm_line_move 内 append_line_trajectory_fail_debug 写入
    }

HandMoveState run_hand_detect_and_validate(
        Arm &arm_r,
        Arm &arm_l,
        RealSenseMultiCam &cameras,
        SegPoseBridge &bridge,
        HeadAssignState &st,
        bool enable_r,
        bool enable_l,
        std::string &err,
        SegEngineId engine_id,
        bool hover_z_from_hand)
    {
        t170::skills::detail::report(t170::skills::Stage::Observe,enable_r,enable_l);
        HandMoveState hs;
        (void)hover_z_from_hand;
        if (!enable_r && !enable_l)
            return hs;

        if (!g_move_cfg.head_grasp.use_hand_camera)
        {
            std::cout << "[hand] 旧版停点复检未启用；连续下降视觉伺服="
                      << (g_move_cfg.cameras.hand_descent_correct?"开启":"关闭")
                      << "（cameras.hand_descent_correct）\n";
            hs.move_r = enable_r;
            hs.move_l = enable_l;
            hs.have_object_z_r = enable_r && st.have_r && std::isfinite(st.right_hand_pos[2]);
            hs.object_z_r = hs.have_object_z_r ? st.right_hand_pos[2] : 0.0;
            hs.have_object_z_l = enable_l && st.have_l && std::isfinite(st.left_hand_pos[2]);
            hs.object_z_l = hs.have_object_z_l ? st.left_hand_pos[2] : 0.0;
            return hs;
        }

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

        const Eigen::Vector2d expected_head_xy_r(
            st.goal_last_r(0) - active_goal_x_offset(true),
            st.goal_last_r(1) - active_goal_y_offset(true));
        const Eigen::Vector2d expected_head_xy_l(
            st.goal_last_l(0) - active_goal_x_offset(false),
            st.goal_last_l(1) - active_goal_y_offset(false));
        Eigen::Matrix<double, 1, 6> hand_cam_r, hand_cam_l;
        hand_cam_r << -1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        hand_cam_l << -1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        Eigen::Matrix<double, 1, 6> base_pos_r, base_pos_l;
        base_pos_r << -1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        base_pos_l << -1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        bool matched_r = false;
        bool matched_l = false;

        const bool have_cam_r =
            !enable_r ||
            load_cam2robot_matrix(default_right_hand_to_robot_yaml_path(), cam2robot_r, err);
        if (enable_r && !have_cam_r)
            std::cerr << "读取右手 cam2robot 失败: " << err << std::endl;
        else if (enable_r)
        {
            matched_r = select_hand_target_matching_head(
                right_hand_records, cam2robot_r, arm_r, expected_head_xy_r,
                "右", true, hand_cam_r, base_pos_r);
        }

        const bool have_cam_l =
            !enable_l ||
            load_cam2robot_matrix(default_left_hand_to_robot_yaml_path(), cam2robot_l, err);
        if (enable_l && !have_cam_l)
            std::cerr << "读取左手 cam2robot 失败: " << err << std::endl;
        else if (enable_l)
        {
            matched_l = select_hand_target_matching_head(
                left_hand_records, cam2robot_l, arm_l, expected_head_xy_l,
                "左", false, hand_cam_l, base_pos_l);
        }

        apply_hand_base_to_goal(
            enable_r && have_cam_r && matched_r,
            base_pos_r,
            enable_l && have_cam_l && matched_l,
            base_pos_l,
            st.goal_last_r,
            st.goal_last_l,
            true,
            st.xy_from_tray,
            st.row_r,
            st.row_l);
        record_hand_object_z(hs, true, enable_r && matched_r, base_pos_r);
        record_hand_object_z(hs, false, enable_l && matched_l, base_pos_l);
        if (enable_r && matched_r &&
            grasp_goal_side_blocked(true, st.goal_last_r(1), st.xy_from_tray))
            log_arm_wall_reject("手相机offset后", true, st.goal_last_r(1));
        if (enable_l && matched_l &&
            grasp_goal_side_blocked(false, st.goal_last_l(1), st.xy_from_tray))
            log_arm_wall_reject("手相机offset后", false, st.goal_last_l(1));

        hs.move_r = enable_r && have_cam_r && matched_r &&
                    hand_move_allowed_right(st.goal_last_r, base_pos_r);
        hs.move_l = enable_l && have_cam_l && matched_l &&
                    hand_move_allowed_left(st.goal_last_l, base_pos_l);

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
            matched_r = select_hand_target_matching_head(
                right_hand_records, cam2robot_r, arm_r, expected_head_xy_r,
                "右", true, hand_cam_r, base_pos_r);
            if (matched_r)
            {
                if (!st.xy_from_tray)
                {
                    update_hand_goal_xy_from_base(
                        base_pos_r,
                        st.goal_last_r,
                        g_move_cfg.head_grasp.hand_right.offset_x,
                        g_move_cfg.head_grasp.hand_right.offset_y);
                }
                else
                {
                    apply_hand_xy_refine_keep_hole("右", true, st.row_r, base_pos_r, st.goal_last_r);
                }
                apply_hand_hover_z_if_valid(base_pos_r, st.goal_last_r);
                record_hand_object_z(hs, true, true, base_pos_r);
                std::cout << std::fixed << std::setprecision(4)
                          << "[hand_z] 右 hand_object_z=" << base_pos_r(2)
                          << " hover_z=" << st.goal_last_r(2)
                          << (st.xy_from_tray ? " (XY 孔位+手相机限幅)\n" : " m\n");
                if (grasp_goal_side_blocked(true, st.goal_last_r(1), st.xy_from_tray))
                    log_arm_wall_reject("手相机offset后", true, st.goal_last_r(1));
            }
            hs.move_r = matched_r && hand_move_allowed_right(st.goal_last_r, base_pos_r);
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
            matched_l = select_hand_target_matching_head(
                left_hand_records, cam2robot_l, arm_l, expected_head_xy_l,
                "左", false, hand_cam_l, base_pos_l);
            if (matched_l)
            {
                if (!st.xy_from_tray)
                {
                    update_hand_goal_xy_from_base(
                        base_pos_l,
                        st.goal_last_l,
                        g_move_cfg.head_grasp.hand_left.offset_x,
                        g_move_cfg.head_grasp.hand_left.offset_y);
                }
                else
                {
                    apply_hand_xy_refine_keep_hole("左", false, st.row_l, base_pos_l, st.goal_last_l);
                }
                apply_hand_hover_z_if_valid(base_pos_l, st.goal_last_l);
                record_hand_object_z(hs, false, true, base_pos_l);
                std::cout << std::fixed << std::setprecision(4)
                          << "[hand_z] 左 hand_object_z=" << base_pos_l(2)
                          << " hover_z=" << st.goal_last_l(2)
                          << (st.xy_from_tray ? " (XY 孔位+手相机限幅)\n" : " m\n");
                if (grasp_goal_side_blocked(false, st.goal_last_l(1), st.xy_from_tray))
                    log_arm_wall_reject("手相机offset后", false, st.goal_last_l(1));
            }
            hs.move_l = matched_l && hand_move_allowed_left(st.goal_last_l, base_pos_l);
        }

        if (st.xy_from_tray)
        {
            if (enable_r && st.move_r && !hs.move_r)
            {
                hs.move_r = true;
                std::cout << "[tray_xy] 右手手相机未匹配，仍按孔位 XY + 盘面参考 Z 抓取\n";
            }
            if (enable_l && st.move_l && !hs.move_l)
            {
                hs.move_l = true;
                std::cout << "[tray_xy] 左手手相机未匹配，仍按孔位 XY + 盘面参考 Z 抓取\n";
            }
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
}
