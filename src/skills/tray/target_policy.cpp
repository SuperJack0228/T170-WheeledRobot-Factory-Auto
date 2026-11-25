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
#include "t170/motion/kinematics_adapter.h"
#include "t170/motion/planning.h"
#include "t170/common/pose_math.h"
#include "t170/motion/retreat.h"
#include "t170/motion/servo_handoff.h"
#include <array>
#include <cstring>
#include "t170/vision/head_transform.h"
#include "t170/hardware/device_bus.h"

using std::vector;
using std::thread;


namespace move_box {


void set_tray_hole_task(TrayHoleTask task)
    {
        g_tray_hole_task = task;
    }











TrayHoleTask tray_hole_task()
    {
        return g_tray_hole_task;
    }

bool tray1_far_band_active()
    {
        return g_tray1_far_band_memory;
    }

void tray1_far_band_set(bool on)
    {
        if (on == g_tray1_far_band_memory)
            return;
        g_tray1_far_band_memory = on;
        std::cout << "[tray_xy] 料盘一远三排记忆 " << (on ? "开启" : "关闭")
                  << (on ? "：后续直接远排姿态拍照\n" : "\n");
    }

void tray1_far_band_clear()
    {
        tray1_far_band_set(false);
    }

int tray_assign_class_id()
    {
        if (g_assign_class_override >= 0)
            return g_assign_class_override;
        return g_tray_hole_task == TrayHoleTask::PlaceEmpty ? kEmptyHoleClassId
                                                           : kGraspDetectClassId;
    }

Eigen::Matrix4d placement_calibration_frame() {
        Eigen::Matrix<double,1,6> zero=Eigen::Matrix<double,1,6>::Zero();
        return TR(arm_goal_in_real_waist_frame(zero));
    }

bool tray2_place_active()
    {
        return g_tray_hole_task == TrayHoleTask::PlaceEmpty;
    }

const wrist_servo::PlacementCalibrationProfile *active_tray_calibration(bool is_right) {
        const int hand=is_right?1:0;
        if(!tray2_place_active()) return wrist_servo::calibrated_tray_grasp_for(g_move_cfg.visual_servo,hand);
        const auto &rpy=tray2_place_rpy_deg_for_row(5,is_right);
        return wrist_servo::calibrated_placement_for(g_move_cfg.visual_servo,hand,rpy.rx*rad,rpy.ry*rad);
    }

double active_goal_x_offset(bool is_right) {
        if(const auto *c=active_tray_calibration(is_right)) return c->head_xy_bias.x();
        if(tray2_place_active()) return is_right?g_move_cfg.tray2_place.right_goal_x_offset:g_move_cfg.tray2_place.goal_x_offset;
        return grasp_goal_x_offset(is_right);
    }

double active_goal_y_offset(bool is_right) {
        if(const auto *c=active_tray_calibration(is_right)) return c->head_xy_bias.y();
        if(tray2_place_active()) return is_right?g_move_cfg.tray2_place.right_goal_y_offset:g_move_cfg.tray2_place.goal_y_offset;
        return grasp_goal_y_offset(is_right);
    }





double active_goal_z_offset(bool is_right, int from_robot_row)
    {
        double z = 0.0;
        if (tray2_place_active())
        {
            const auto &t = g_move_cfg.tray2_place;
            z = is_right ? t.right_grasp_z_offset_m : t.grasp_z_offset_m;
        }
        else
        {
            z = is_right ? g_move_cfg.head_grasp.right_grasp_z_offset_m
                         : g_move_cfg.head_grasp.grasp_z_offset_m;
        }
        (void)from_robot_row;
        return z;
    }

bool goal_z_in_range(double z)
    {
        return z >= g_move_cfg.grasp_valid.z_min && z <= g_move_cfg.grasp_valid.z_max;
    }

bool goal_pos_valid_right(double x, double y, double z)
    {
        const auto &v = g_move_cfg.grasp_valid;
        return x >= v.x_min && x <= v.x_max && y >= v.right_y_min && y <= v.right_y_max &&
               goal_z_in_range(z);
    }

bool goal_pos_valid_left(double x, double y, double z)
    {
        const auto &v = g_move_cfg.grasp_valid;
        return x >= v.x_min && x <= v.x_max && y >= v.left_y_min && y <= v.left_y_max &&
               goal_z_in_range(z);
    }

bool grasp_goal_side_blocked(bool is_right, double y, bool from_tray)
    {
        if (from_tray)
            return false;
        return is_right ? !arm_y_allowed_right(y) : !arm_y_allowed_left(y);
    }

void drop_if_y_split_blocks(HeadAssignState &st, bool &move_r, bool &move_l, const char *stage)
    {
        if (move_r && grasp_goal_side_blocked(true, st.goal_last_r(1), st.xy_from_tray))
        {
            log_arm_wall_reject(stage, true, st.goal_last_r(1));
            move_r = false;
            st.move_r = false;
        }
        if (move_l && grasp_goal_side_blocked(false, st.goal_last_l(1), st.xy_from_tray))
        {
            log_arm_wall_reject(stage, false, st.goal_last_l(1));
            move_l = false;
            st.move_l = false;
        }
    }

void drop_if_y_split_blocks(HeadAssignState &st, HandMoveState &hs, const char *stage)
    {
        if (hs.move_r && grasp_goal_side_blocked(true, st.goal_last_r(1), st.xy_from_tray))
        {
            log_arm_wall_reject(stage, true, st.goal_last_r(1));
            hs.move_r = false;
        }
        if (hs.move_l && grasp_goal_side_blocked(false, st.goal_last_l(1), st.xy_from_tray))
        {
            log_arm_wall_reject(stage, false, st.goal_last_l(1));
            hs.move_l = false;
        }
    }

bool head_hand_detected(const double hand_pos[3])
    {
        return hand_pos[0] >= 0.0;
    }

bool head_x_needs_stagger(double x)
    {
        return g_move_cfg.waist.stagger_max_steps > 0 &&
               x > g_move_cfg.stagger.head_x_threshold;
    }

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

double tray_z_ref_now()
    {
        const auto &tr = g_move_cfg.tray;
        if (!waist_layer3_is_synced() || !std::isfinite(g_current_waist_layer3_z))
            return tr.z_ref_m;
        return tr.z_ref_m - (g_current_waist_layer3_z - tr.z_ref_waist_z_m);
    }

double expected_object_top_z()
    {
        return tray_z_ref_now() + g_move_cfg.tray.part_above_tray_m;
    }

double refined_object_top_z(double camera_z, bool have_camera_z, const char *source)
    {
        const double expected = expected_object_top_z();
        const double maxd = g_move_cfg.tray.z_refine_max_m;
        if (!(have_camera_z && std::isfinite(camera_z)))
        {
            std::cout << std::fixed << std::setprecision(4)
                      << "[z_ref] " << source << " 无相机顶面，采用参考高度 " << expected << "\n";
            return expected;
        }
        const double d = camera_z - expected;
        if (std::abs(d) <= maxd)
        {
            std::cout << std::fixed << std::setprecision(4)
                      << "[z_ref] " << source << " 采用相机顶面 " << camera_z
                      << "（参考 " << expected << "，差 " << d * 1000.0
                      << " mm ≤ " << maxd * 1000.0 << " mm）\n";
            return camera_z;
        }
        std::cout << std::fixed << std::setprecision(4)
                  << "[z_ref] " << source << " 相机顶面 " << camera_z
                  << " 与参考 " << expected << " 差 " << d * 1000.0
                  << " mm，超过 " << maxd * 1000.0 << " mm，改用参考高度\n";
        return expected;
    }

void fill_goal_last_from_head_assign(
        const double right_hand_pos[3],
        const double left_hand_pos[3],
        int row_r,
        int row_l,
        Eigen::Matrix<double, 1, 6> &goal_last_r,
        Eigen::Matrix<double, 1, 6> &goal_last_l)
    {
        const auto &hg = g_move_cfg.head_grasp;
        const bool have_r = head_hand_detected(right_hand_pos);
        const bool have_l = head_hand_detected(left_hand_pos);
        const double near_z = 0.0;
        const double extra_z_r = (row_r == 6) ? near_z : 0.0;
        const double extra_z_l = (row_l == 6) ? near_z : 0.0;
        const double z_r =
            refined_object_top_z(right_hand_pos[2], have_r, "头右") + hg.hover_above_m + extra_z_r;
        const double z_l =
            refined_object_top_z(left_hand_pos[2], have_l, "头左") + hg.hover_above_m + extra_z_l;
        const bool place = (g_tray_hole_task == TrayHoleTask::PlaceEmpty);
        const GraspRpyDeg &rpy_r = place ? tray2_place_rpy_deg_for_row(row_r, true)
                                         : grasp_rpy_deg_for_row(row_r, true);
        const GraspRpyDeg &rpy_l = place ? tray2_place_rpy_deg_for_row(row_l, false)
                                         : grasp_rpy_deg_for_row(row_l, false);
        goal_last_r << right_hand_pos[0], right_hand_pos[1], z_r,
            rpy_r.rx * rad, rpy_r.ry * rad, rpy_r.rz * rad;
        goal_last_l << left_hand_pos[0], left_hand_pos[1], z_l,
            rpy_l.rx * rad, rpy_l.ry * rad, rpy_l.rz * rad;

        goal_last_r(0) += active_goal_x_offset(true)+0.0;
        goal_last_l(0) += active_goal_x_offset(false)+0.0;
        goal_last_r(1) += active_goal_y_offset(true)+0.0;
        goal_last_l(1) += active_goal_y_offset(false)+0.0;
        goal_last_r(2) += 0.0;
        goal_last_l(2) += 0.0;
        clamp_cart_goal_z(goal_last_r, "fill_goal_last_from_head_assign(右手, 物体上方)");
        clamp_cart_goal_z(goal_last_l, "fill_goal_last_from_head_assign(左手, 物体上方)");
        auto log_hover = [&](const char *hand, bool is_right, const double pos[3], double hover_z,
                             int row, const GraspRpyDeg &rpy)
        {
            if (!head_hand_detected(pos))
                return;
            std::cout << std::fixed << std::setprecision(4)
                      << "[head] " << hand << " 物体 xyz=(" << pos[0] << "," << pos[1] << ","
                      << pos[2] << ") → 上方 z=" << hover_z << " (Z相机/参考比对)";
            std::cout << " hover=" << hg.hover_above_m << " m"
                      << " offset_xyz=(" << active_goal_x_offset(is_right) << ","
                      << active_goal_y_offset(is_right) << ","
                      << active_goal_z_offset(is_right, row) << ")"
                      << " row_xyz=(" << XyzOffsetM{}.x << ","
                      << XyzOffsetM{}.y << ","
                      << XyzOffsetM{}.z << ")"
                      << (tray2_place_active() ? " [tray2]" : "")
                      << " from_robot=" << row;
            if (row == 1)
                std::cout << " pose=row1最远专用";
            else
                std::cout << " ready" << (tray_row_pose_group(row) + 1);
            if (row == 6 && std::abs(near_z) > 1e-9)
                std::cout << " 近排额外z=" << near_z;
            std::cout << std::setprecision(1)
                      << " rpy_deg=(" << rpy.rx << "," << rpy.ry << "," << rpy.rz << ")\n";
        };
        log_hover("右", true, right_hand_pos, goal_last_r(2), row_r, rpy_r);
        log_hover("左", false, left_hand_pos, goal_last_l(2), row_l, rpy_l);
        if (have_r)
        {
            std::cout << std::fixed << std::setprecision(1)
                      << "[rpy] 右 from_robot=" << row_r
                      << (row_r == 1 ? " 使用 yaml row1_rpy_deg(离机器人最远)" : " 使用 ready 组")
                      << " rpy_deg=(" << rpy_r.rx << "," << rpy_r.ry << "," << rpy_r.rz << ")\n";
        }
        if (have_l)
        {
            std::cout << std::fixed << std::setprecision(1)
                      << "[rpy] 左 from_robot=" << row_l
                      << (row_l == 1 ? " 使用 yaml row1_rpy_deg(离机器人最远)" : " 使用 ready 组")
                      << " rpy_deg=(" << rpy_l.rx << "," << rpy_l.ry << "," << rpy_l.rz << ")\n";
        }
        if (head_hand_detected(right_hand_pos) &&
            !goal_pos_valid_right(goal_last_r(0), goal_last_r(1), right_hand_pos[2]))
            log_arm_skip_right();
        if (head_hand_detected(left_hand_pos) &&
            !goal_pos_valid_left(goal_last_l(0), goal_last_l(1), left_hand_pos[2]))
            log_arm_skip_left();
    }

bool apply_adaptive_grasp_rpy_one(
        const Eigen::Matrix<double, 1, 6> &tcp,
        Eigen::Matrix<double, 1, 6> &goal,
        bool is_right)
    {
        const auto &hg = g_move_cfg.head_grasp;
        const double dx = goal(0) - tcp(0);
        const double dy = goal(1) - tcp(1);
        const double horiz = std::hypot(dx, dy);
        if (horiz < hg.adaptive_min_horiz_m)
        {
            std::cout << std::fixed << std::setprecision(3)
                      << "[adapt-rpy] " << (is_right ? "右" : "左")
                      << " 水平距=" << horiz << " < " << hg.adaptive_min_horiz_m
                      << " m，保持固定姿态\n";
            return false;
        }

        double yaw = std::atan2(dy, dx);
        const double yaw_max = std::abs(hg.adaptive_yaw_max_deg) * rad;
        if (yaw > yaw_max)
            yaw = yaw_max;
        else if (yaw < -yaw_max)
            yaw = -yaw_max;

        const Eigen::Vector3d h(std::cos(yaw), std::sin(yaw), 0.0);
        double pitch = hg.adaptive_pitch_down_deg * rad;
        if (pitch < 5.0 * rad)
            pitch = 5.0 * rad;
        if (pitch > 80.0 * rad)
            pitch = 80.0 * rad;

        Eigen::Vector3d tool_x = std::cos(pitch) * h + std::sin(pitch) * Eigen::Vector3d(0.0, 0.0, -1.0);
        Eigen::Vector3d tool_z =
            is_right ? Eigen::Vector3d(-h.y(), h.x(), 0.0) : Eigen::Vector3d(h.y(), -h.x(), 0.0);
        tool_z.normalize();
        tool_x.normalize();
        Eigen::Vector3d tool_y = tool_z.cross(tool_x);
        const double y_n = tool_y.norm();
        if (y_n < 1e-8)
        {
            std::cerr << "[adapt-rpy] " << (is_right ? "右" : "左") << " 姿态奇异，保持固定\n";
            return false;
        }
        tool_y /= y_n;
        tool_x = tool_y.cross(tool_z).normalized();

        Eigen::Matrix3d R;
        R.col(0) = tool_x;
        R.col(1) = tool_y;
        R.col(2) = tool_z;
        const Eigen::Matrix<double, 1, 3> eul = rotationMatrixToEulerAngles(R);
        goal(3) = eul(0);
        goal(4) = eul(1);
        goal(5) = eul(2);

        std::cout << std::fixed << std::setprecision(1)
                  << "[adapt-rpy] " << (is_right ? "右" : "左")
                  << " TCP→目标 yaw=" << (yaw / rad) << "° pitch_down=" << (pitch / rad)
                  << "° → rpy=(" << (goal(3) / rad) << "," << (goal(4) / rad) << ","
                  << (goal(5) / rad) << ")°\n";
        return true;
    }

void apply_adaptive_grasp_rpy_selective(
        Arm &arm_r,
        Arm &arm_l,
        HeadAssignState &st,
        bool move_r,
        bool move_l)
    {
        if (!g_grasp_adaptive_rpy || tray2_place_active())
            return;
        if (move_r)
            apply_adaptive_grasp_rpy_one(arm_get_tcp_pos(arm_r), st.goal_last_r, true);
        if (move_l)
            apply_adaptive_grasp_rpy_one(arm_get_tcp_pos(arm_l), st.goal_last_l, false);
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

void log_hand_grasp_heights(const char *hand, bool is_right, int row, double object_z,
                                double hover_z, double final_z)
    {
        const auto &hg = g_move_cfg.head_grasp;
        const auto &tr = g_move_cfg.tray;
        std::cout << std::fixed << std::setprecision(4)
                  << "[hand_z] " << hand
                  << " z_ref=" << tray_z_ref_now()
                  << " (yaml " << tr.z_ref_m << " @waist " << tr.z_ref_waist_z_m
                  << " now " << g_current_waist_layer3_z << ")"
                  << " part_above=" << tr.part_above_tray_m
                  << " hand_object_z=" << object_z
                  << " hover_z=" << hover_z
                  << " final_grasp_z=" << final_z
                  << " (hover_above=" << hg.hover_above_m
                  << " z_offset=" << active_goal_z_offset(is_right, row) << " m";
        if (row == 6 && std::abs(0.0) > 1e-9)
            std::cout << " 含近排额外 " << 0.0;
        if (tray2_place_active())
            std::cout << " [tray2 offset]";
        std::cout << ")\n";
    }

bool apply_final_grasp_z_from_hand(
        const char *hand,
        bool is_right,
        int from_robot_row,
        bool have_object_z,
        double object_z,
        Eigen::Matrix<double, 1, 6> &goal_last)
    {
        const auto &hg = g_move_cfg.head_grasp;
        const auto &v = g_move_cfg.grasp_valid;
        const double top_z = refined_object_top_z(
            object_z, have_object_z, hand);
        const double extra_z = (from_robot_row == 6) ? 0.0 : 0.0;

        const double z_off = active_goal_z_offset(is_right, from_robot_row);
        const double hover_z = top_z + hg.hover_above_m + extra_z;
        const double final_z = top_z + z_off;
        log_hand_grasp_heights(hand, is_right, from_robot_row,
                               have_object_z ? object_z : expected_object_top_z(),
                               hover_z, final_z);
        const bool z_ok = final_z >= v.z_min && final_z <= v.z_max;
        const bool pos_ok = is_right
                                ? goal_pos_valid_right(goal_last(0), goal_last(1), final_z)
                                : goal_pos_valid_left(goal_last(0), goal_last(1), final_z);
        if (!z_ok || !pos_ok)
        {
            std::cerr << std::fixed << std::setprecision(4)
                      << "[hand_z] " << hand << " final_grasp_z=" << final_z
                      << " 超出抓取安全范围 z[" << v.z_min << ',' << v.z_max
                      << "]，拒绝本侧\n";
            return false;
        }
        goal_last(2) = final_z;
        return true;
    }
}
