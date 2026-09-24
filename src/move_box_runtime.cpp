#include "move_box_runtime.h"

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
#include <map>
#include <numeric>
#include <optional>
#include <random>
#include <sstream>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "head.h"
#include "head_cam2robot.h"
#include "Ti5_socketcan.h"

namespace move_box
{

    bool g_grasp_adaptive_rpy = false;
    double g_current_waist_layer3_z = std::numeric_limits<double>::quiet_NaN();
    TrayHoleTask g_tray_hole_task = TrayHoleTask::GraspParts;

    bool g_tray2_precision_test = false;

    void set_tray_hole_task(TrayHoleTask task)
    {
        g_tray_hole_task = task;
    }

    void set_tray2_precision_test(bool on)
    {
        g_tray2_precision_test = on;
    }

    bool tray2_precision_test()
    {
        return g_tray2_precision_test;
    }

    TrayHoleTask tray_hole_task()
    {
        return g_tray_hole_task;
    }

    int tray_assign_class_id()
    {
        return g_tray_hole_task == TrayHoleTask::PlaceEmpty ? kEmptyHoleClassId
                                                           : kGraspDetectClassId;
    }

    bool tray2_place_active()
    {
        return g_tray_hole_task == TrayHoleTask::PlaceEmpty;
    }

    double active_goal_x_offset(bool is_right)
    {
        if (tray2_place_active())
        {
            const auto &t = g_move_cfg.tray2_place;
            return is_right ? t.right_goal_x_offset : t.goal_x_offset;
        }
        return grasp_goal_x_offset(is_right);
    }

    double active_goal_y_offset(bool is_right)
    {
        if (tray2_place_active())
        {
            const auto &t = g_move_cfg.tray2_place;
            return is_right ? t.right_goal_y_offset : t.goal_y_offset;
        }
        return grasp_goal_y_offset(is_right);
    }

    double active_nearest_row_z()
    {
        return tray2_place_active() ? g_move_cfg.tray2_place.nearest_row_z_offset_m
                                    : g_move_cfg.head_grasp.nearest_row_grasp_z_offset_m;
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
        if (from_robot_row == 6)
            z += active_nearest_row_z();
        return z;
    }

    Eigen::Matrix<double, 1, 6> make_standby_pos_right()
    {
        return g_move_cfg.standby.right;
    }

    Eigen::Matrix<double, 1, 6> make_standby_pos_left()
    {
        return g_move_cfg.standby.left;
    }

    Eigen::Matrix<double, 1, 6> make_home_tcp_right()
    {
        return g_move_cfg.home_tcp.right;
    }

    Eigen::Matrix<double, 1, 6> make_home_tcp_left()
    {
        return g_move_cfg.home_tcp.left;
    }

    static Eigen::Matrix<double, 1, 6> apply_belt_place_offset(
        bool is_right,
        double origin_x,
        double origin_y,
        double origin_z,
        const ConveyorStationConfig &c,
        const BeltDetectResult *det)
    {
        const auto &off = is_right ? c.offset_right : c.offset_left;
        Eigen::Matrix<double, 1, 6> p = is_right ? c.place_tcp.right : c.place_tcp.left;
        if (det != nullptr && det->have_axes)
        {
            p(0) = origin_x + off.x * det->ax_x + off.y * det->ay_x + off.z * det->az_x;
            p(1) = origin_y + off.x * det->ax_y + off.y * det->ay_y + off.z * det->az_y;
            p(2) = origin_z + off.x * det->ax_z + off.y * det->ay_z + off.z * det->az_z;
        }
        else
        {
            p(0) = origin_x + off.x;
            p(1) = origin_y + off.y;
            p(2) = origin_z + off.z;
        }
        return p;
    }

    Eigen::Matrix<double, 1, 6> make_belt_goal_right(
        const BeltDetectResult &det, const ConveyorStationConfig &c)
    {
        return apply_belt_place_offset(
            true, det.origin_x, det.origin_y, det.origin_z, c, &det);
    }

    Eigen::Matrix<double, 1, 6> make_belt_goal_left(
        const BeltDetectResult &det, const ConveyorStationConfig &c)
    {
        return apply_belt_place_offset(
            false, det.origin_x, det.origin_y, det.origin_z, c, &det);
    }

    Eigen::Matrix<double, 1, 6> make_belt_goal_right(
        double origin_x, double origin_y, double origin_z, const ConveyorStationConfig &c)
    {
        return apply_belt_place_offset(true, origin_x, origin_y, origin_z, c, nullptr);
    }

    Eigen::Matrix<double, 1, 6> make_belt_goal_left(
        double origin_x, double origin_y, double origin_z, const ConveyorStationConfig &c)
    {
        return apply_belt_place_offset(false, origin_x, origin_y, origin_z, c, nullptr);
    }

    Eigen::Matrix<double, 1, 6> make_belt_goal_right(double origin_x, double origin_y, double origin_z)
    {
        return make_belt_goal_right(origin_x, origin_y, origin_z, g_move_cfg.conveyor);
    }

    Eigen::Matrix<double, 1, 6> make_belt_goal_left(double origin_x, double origin_y, double origin_z)
    {
        return make_belt_goal_left(origin_x, origin_y, origin_z, g_move_cfg.conveyor);
    }

    Eigen::Matrix<double, 1, 6> make_belt_grasp_goal(
        bool is_right, double grasp_x, double grasp_y, double grasp_z, const ConveyorStationConfig &c)
    {
        const GraspRpyDeg &rpy = is_right ? c.grasp_rpy_right : c.grasp_rpy_left;
        Eigen::Matrix<double, 1, 6> p = Eigen::Matrix<double, 1, 6>::Zero();
        p(0) = grasp_x;
        p(1) = grasp_y;
        p(2) = grasp_z;
        p(3) = rpy.rx * rad;
        p(4) = rpy.ry * rad;
        p(5) = rpy.rz * rad;
        return p;
    }

    Eigen::Matrix<double, 1, 6> make_belt_grasp_goal(
        bool is_right, double grasp_x, double grasp_y, double grasp_z)
    {
        return make_belt_grasp_goal(is_right, grasp_x, grasp_y, grasp_z, g_move_cfg.conveyor);
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

    /** 料盘已按 ArUco 列分左右时，不再用盘心 Y（含 offset 后）拒掉。无料盘才用 Y 分界。 */
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

    /** 优先相机顶面；与参考偏差超过 z_refine_max_m 则改用参考。无相机用参考。 */
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

    double hover_z_from_object(double object_z)
    {
        const auto &hg = g_move_cfg.head_grasp;
        return refined_object_top_z(object_z, std::isfinite(object_z), "hover") +
               hg.hover_above_m;
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
        const double near_z = active_nearest_row_z();
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
        goal_last_r(0) += active_goal_x_offset(true);
        goal_last_l(0) += active_goal_x_offset(false);
        goal_last_r(1) += active_goal_y_offset(true);
        goal_last_l(1) += active_goal_y_offset(false);
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

    /**
     * 用当前 TCP 指向目标 xy（hover 点）的水平方向定 yaw，再绕水平轴下斜 pitch_down。
     * 几何与固定 (rx=±90, ry=45, rz=0) 一致：tool+X = cos(p)·h + sin(p)·(0,0,-1)。
     * 水平距过近则保持 yaml 固定姿态。返回 true=已改写 goal 的 rpy。
     */
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
        Robot_Arm &arm_r,
        Robot_Arm &arm_l,
        HeadAssignState &st,
        bool move_r,
        bool move_l)
    {
        if (!g_grasp_adaptive_rpy)
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

    void apply_hand_hover_z_if_valid(
        const Eigen::Matrix<double, 1, 6> &base_pos,
        Eigen::Matrix<double, 1, 6> &goal_last)
    {
        goal_last(2) = hover_z_from_object(base_pos(2));
        clamp_cart_goal_z(goal_last, "手相机物体上方");
    }

    void record_hand_object_z(HandMoveState &hs, bool is_right, bool matched,
                              const Eigen::Matrix<double, 1, 6> &base_pos)
    {
        if (is_right)
        {
            hs.have_object_z_r = matched && std::isfinite(base_pos(2));
            hs.object_z_r = hs.have_object_z_r ? base_pos(2) : 0.0;
        }
        else
        {
            hs.have_object_z_l = matched && std::isfinite(base_pos(2));
            hs.object_z_l = hs.have_object_z_l ? base_pos(2) : 0.0;
        }
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
        if (row == 6 && std::abs(active_nearest_row_z()) > 1e-9)
            std::cout << " 含近排额外 " << active_nearest_row_z();
        if (tray2_place_active())
            std::cout << " [tray2 offset]";
        std::cout << ")\n";
    }

    /** 最终夹取高度 = 比对后的顶面 + grasp_z_offset。 */
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
        const double extra_z = (from_robot_row == 6) ? active_nearest_row_z() : 0.0;
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

    double clamp_signed_abs(double v, double max_abs)
    {
        if (v > max_abs)
            return max_abs;
        if (v < -max_abs)
            return -max_abs;
        return v;
    }

    /** 孔位 XY 为基准，用手相机观测做限幅微调；goal_x_offset 保留。 */
    void apply_hand_xy_refine_keep_hole(
        const char *hand,
        bool is_right,
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
        bool keep_xy)
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
                apply_hand_xy_refine_keep_hole("右", true, base_pos_r, goal_last_r);
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
                apply_hand_xy_refine_keep_hole("左", false, base_pos_l, goal_last_l);
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

    /**
     * 手相机只允许继续跟踪头相机已经选中的同一目标。
     * 每个候选先转到基座系，再以 XY 距离匹配；距离过大则拒绝，禁止跳到另一排物料。
     */
    bool select_hand_target_matching_head(
        const PoseDetectionRecords &records,
        const std::array<double, 16> &cam2robot,
        Robot_Arm &arm,
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
        std::cerr << "[arm] " << hand << "轨迹解析失败 ret=" << ret << "\n";
        // 详细记录（goal/stage）由 arm_line_move 内 append_line_trajectory_fail_debug 写入
    }

    void log_arm_skip_right()
    {
        const auto &v = g_move_cfg.grasp_valid;
        cout << std::fixed << std::setprecision(3)
             << "[arm] 右手目标超工作包络(x:" << v.x_min << "~" << v.x_max
             << ", y:" << v.right_y_min << "~" << v.right_y_max << ")，本段不移动\n";
    }

    void log_arm_skip_left()
    {
        const auto &v = g_move_cfg.grasp_valid;
        cout << std::fixed << std::setprecision(3)
             << "[arm] 左手目标超工作包络(x:" << v.x_min << "~" << v.x_max
             << ", y:" << v.left_y_min << "~" << v.left_y_max << ")，本段不移动\n";
    }

    bool dual_grasp_targets_too_close(const HeadAssignState &st)
    {
        if (!st.move_r || !st.move_l)
            return false;
        const int cr = (st.col_r >= 1) ? st.col_r : tray_column_index_from_y(st.goal_last_r(1));
        const int cl = (st.col_l >= 1) ? st.col_l : tray_column_index_from_y(st.goal_last_l(1));
        if (simultaneous_assign_columns_ok(cr, cl))
            return false;
        std::cout << std::fixed << std::setprecision(4)
                  << "[col] 双臂列号 右=" << cr << " 左=" << cl
                  << " 差=" << (cl - cr)
                  << " < 最小同时列差 " << g_move_cfg.grasp_zone.min_simultaneous_col_delta
                  << " (允许 1-4/2-5/3-6 或更远)，禁止同时运动\n";
        return true;
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
        if (!(st.move_r && st.move_l))
            return false;
        if (st.col_r >= 1 && st.col_l >= 1)
            return simultaneous_assign_columns_ok(st.col_r, st.col_l);
        return simultaneous_columns_ok(st.goal_last_r(1), st.goal_last_l(1));
    }

    constexpr double kRetractMaxJointJumpRad = 2.0;
    constexpr double kRetractSkipXyzM = 0.012;
    constexpr double kRetractSkipRpyRad = 5.0 * rad;
    constexpr double kRetractOwnSideMarginM = 0.01;
    constexpr double kFoldedNearBodyXMaxM = 0.20;
    constexpr double kFoldedLowZMaxM = -0.45;
    constexpr double kFoldedElbowAbsRad = 70.0 * rad;
    constexpr double kJointHomeArriveXyzM = 0.05;

    bool joints_to_matrix7(const Eigen::RowVectorXd &q, Eigen::Matrix<double, 1, 7> &out)
    {
        if (q.size() < 7)
            return false;
        for (int i = 0; i < 7; ++i)
            out(i) = q(i);
        return true;
    }

    bool try_standby_ik(Robot_Arm &arm, Eigen::Matrix<double, 1, 6> goal,
                        const Eigen::RowVectorXd &seed, double analytic_j2,
                        Eigen::RowVectorXd &q_out, int &ik_out)
    {
        Eigen::RowVectorXd q = seed;
        Eigen::Matrix<double, 1, 6> g = goal;
        int ik = arm.Inverse_Kinematics(g, q);
        if (ik != 0)
        {
            q = seed;
            g = goal;
            ik = arm.Inverse_Kinematics_Numeric(g, q);
        }
        if (ik != 0)
        {
            q = seed;
            ik = arm.Inverse_Kinematics_Analytic(goal, q, analytic_j2);
        }
        ik_out = ik;
        if (ik != 0)
            return false;
        q_out = q;
        return true;
    }

    bool solve_home_joints(Robot_Arm &arm, bool is_right, Eigen::Matrix<double, 1, 6> goal,
                           Eigen::Matrix<double, 1, 7> &qd)
    {
        const Eigen::RowVectorXd q_now = arm_get_joint_pos(arm);
        if (q_now.size() < 7)
        {
            std::cerr << "[home] " << (is_right ? "右" : "左") << " 逆解维数不足\n";
            return false;
        }

        // 垂臂当前关节离 standby 太远，不能当唯一种子。换零位/待机构型种子再解。
        Eigen::RowVectorXd q_zero = Eigen::RowVectorXd::Zero(q_now.size());
        Eigen::RowVectorXd q_standby_seed = q_zero;
        if (is_right)
        {
            q_standby_seed << 20.9 * rad, 45.7 * rad, -13.6 * rad, 101.3 * rad, -44.5 * rad,
                -18.8 * rad, 65.2 * rad;
        }
        else
        {
            q_standby_seed << -28.0 * rad, -30.9 * rad, 21.4 * rad, -88.1 * rad, -39.8 * rad,
                49.1 * rad, -32.4 * rad;
        }

        const Eigen::RowVectorXd seeds[3] = {q_now, q_zero, q_standby_seed};
        const char *seed_names[3] = {"当前关节", "零位", "待机构型"};
        const double j2_try[8] = {
            q_now(1),
            -45.0 * rad, -30.0 * rad, -20.0 * rad, 0.0,
            20.0 * rad, 30.0 * rad, 45.0 * rad};

        Eigen::RowVectorXd q_sol;
        int last_ik = -1;
        const char *used = nullptr;
        for (int s = 0; s < 3; ++s)
        {
            for (double j2 : j2_try)
            {
                Eigen::RowVectorXd q;
                if (try_standby_ik(arm, goal, seeds[s], j2, q, last_ik))
                {
                    q_sol = q;
                    used = seed_names[s];
                    break;
                }
            }
            if (used != nullptr)
                break;
        }

        if (used == nullptr)
        {
            const Matrix<double, 1, 6> fk =
                T2PosEulerAngles(arm.Forward_Kinematics(q_standby_seed));
            if ((fk.head<3>() - goal.head<3>()).norm() <= 0.05)
            {
                q_sol = q_standby_seed;
                used = "待机构型直达";
                last_ik = 0;
                std::cout << "[home] " << (is_right ? "右" : "左")
                          << " 笛卡尔逆解失败，改用已知待机关节直达 yaml standby\n";
            }
        }

        if (used == nullptr || !joints_to_matrix7(q_sol, qd))
        {
            std::cerr << "[home] " << (is_right ? "右" : "左")
                      << " standby 逆解失败 code=" << last_ik
                      << "（垂臂种子不可用时已换种子仍失败）\n";
            return false;
        }

        const Eigen::Matrix<double, 1, 7> q0 = arm_get_joint_pos(arm);
        std::cout << std::fixed << std::setprecision(1)
                  << "[home] " << (is_right ? "右" : "左") << " 关节回 standby seed=" << used
                  << " Δq_deg=[";
        double max_abs = 0.0;
        for (int i = 0; i < 7; ++i)
        {
            const double d = std::abs(qd(i) - q0(i));
            if (d > max_abs)
                max_abs = d;
            std::cout << (d / rad);
            if (i + 1 < 7)
                std::cout << ",";
        }
        std::cout << "] max=" << (max_abs / rad) << "°\n";
        return true;
    }

    void log_retract_arm_state(const char *tag, bool is_right, Robot_Arm &arm, bool enforce_zone = true);

    bool arm_pose_needs_joint_home(Robot_Arm &arm)
    {
        const Eigen::Matrix<double, 1, 6> tcp = arm_get_tcp_pos(arm);
        const Eigen::Matrix<double, 1, 7> q = arm_get_joint_pos(arm);
        const bool near_body = tcp(0) < kFoldedNearBodyXMaxM;
        const bool very_low = tcp(2) < kFoldedLowZMaxM;
        const bool elbow_folded = std::abs(q(1)) > kFoldedElbowAbsRad;
        return (near_body && very_low) || (very_low && elbow_folded);
    }

    bool retract_joint_to_standby(
        Robot_Arm &arm_r,
        Robot_Arm &arm_l,
        Eigen::Matrix<double, 1, 6> standby_r,
        Eigen::Matrix<double, 1, 6> standby_l,
        bool check_zone)
    {
        Eigen::Matrix<double, 1, 7> qd_r = Eigen::Matrix<double, 1, 7>::Zero();
        Eigen::Matrix<double, 1, 7> qd_l = Eigen::Matrix<double, 1, 7>::Zero();
        const bool move_r = !right_arm_motors_locked();
        if (move_r && !solve_home_joints(arm_r, true, standby_r, qd_r))
            return false;
        if (!solve_home_joints(arm_l, false, standby_l, qd_l))
            return false;
        if (hardware_abort_requested())
            return false;

        int ret_r = 0;
        int ret_l = 0;
        std::thread tr;
        std::thread tl([&]() { ret_l = arm_joint_move(arm_l, qd_l); });
        if (move_r)
            tr = std::thread([&]() { ret_r = arm_joint_move(arm_r, qd_r); });
        if (tr.joinable())
            tr.join();
        tl.join();

        bool ok = true;
        if (move_r && ret_r != 0)
        {
            log_arm_traj_plan_fail("右手(home关节)", ret_r);
            ok = false;
        }
        if (ret_l != 0)
        {
            log_arm_traj_plan_fail("左手(home关节)", ret_l);
            ok = false;
        }
        if (move_r && ret_r == 0)
            record_arm_cartesian_arrival(arm_r, standby_r, "关节回standby");
        if (ret_l == 0)
            record_arm_cartesian_arrival(arm_l, standby_l, "关节回standby");
        if (move_r)
            log_retract_arm_state("关节回standby后", true, arm_r);
        log_retract_arm_state("关节回standby后", false, arm_l);
        const Eigen::Matrix<double, 1, 6> cur_r = arm_get_tcp_pos(arm_r);
        const Eigen::Matrix<double, 1, 6> cur_l = arm_get_tcp_pos(arm_l);
        if (move_r && (cur_r.head<3>() - standby_r.head<3>()).norm() > kJointHomeArriveXyzM)
        {
            std::cerr << "[home] 右手关节复位后未到 standby XYZ\n";
            ok = false;
        }
        if ((cur_l.head<3>() - standby_l.head<3>()).norm() > kJointHomeArriveXyzM)
        {
            std::cerr << "[home] 左手关节复位后未到 standby XYZ\n";
            ok = false;
        }
        if (check_zone && move_r && !arm_y_allowed_right(cur_r(1)))
        {
            std::cerr << "[home] 右手关节复位后越左区边界\n";
            ok = false;
        }
        if (check_zone && !arm_y_allowed_left(cur_l(1)))
        {
            std::cerr << "[home] 左手关节复位后越右区边界\n";
            ok = false;
        }
        return ok && !hardware_abort_requested();
    }

    void log_retract_arm_state(const char *tag, bool is_right, Robot_Arm &arm, bool enforce_zone)
    {
        const Eigen::Matrix<double, 1, 6> tcp = arm_get_tcp_pos(arm);
        const Eigen::Matrix<double, 1, 7> q = arm_get_joint_pos(arm);
        const int col = tray_column_index_from_y(tcp(1));
        const bool zone_ok = is_right ? arm_y_allowed_right(tcp(1)) : arm_y_allowed_left(tcp(1));
        const char *zone_txt = !enforce_zone ? " 传送带不卡中缝"
                                             : (zone_ok ? " 本侧OK" : " 越分区");
        std::cout << std::fixed << std::setprecision(4)
                  << "[home] " << (is_right ? "右" : "左") << " " << tag
                  << " xyz=(" << tcp(0) << "," << tcp(1) << "," << tcp(2)
                  << ") rpy_deg=(" << (tcp(3) / rad) << "," << (tcp(4) / rad) << ","
                  << (tcp(5) / rad) << ") col=" << col
                  << zone_txt
                  << std::setprecision(1) << " q_deg=[";
        for (int i = 0; i < 7; ++i)
        {
            std::cout << (q(i) / rad);
            if (i + 1 < 7)
                std::cout << ",";
        }
        std::cout << "]\n";
    }

    void copy_current_rpy(const Eigen::Matrix<double, 1, 6> &cur, Eigen::Matrix<double, 1, 6> &goal)
    {
        goal(3) = cur(3);
        goal(4) = cur(4);
        goal(5) = cur(5);
    }

    bool retract_xyz_close(const Eigen::Matrix<double, 1, 6> &a, const Eigen::Matrix<double, 1, 6> &b)
    {
        return (a.head<3>() - b.head<3>()).norm() <= kRetractSkipXyzM;
    }

    bool retract_rpy_close(const Eigen::Matrix<double, 1, 6> &a, const Eigen::Matrix<double, 1, 6> &b)
    {
        return (a.tail<3>() - b.tail<3>()).cwiseAbs().maxCoeff() <= kRetractSkipRpyRad;
    }

    void clamp_retract_goal_y(bool is_right, Eigen::Matrix<double, 1, 6> &goal)
    {
        const double split = column_split_y();
        const double y0 = goal(1);
        if (is_right)
        {
            if (goal(1) > split)
                goal(1) = split;
        }
        else if (goal(1) <= split)
            goal(1) = split + kRetractOwnSideMarginM;
        if (std::abs(goal(1) - y0) > 1e-6)
        {
            std::cout << std::fixed << std::setprecision(4)
                      << "[home] " << (is_right ? "右" : "左")
                      << " 目标 y=" << y0 << " 越分区，收到 y=" << goal(1)
                      << " (split=" << split << "，不穿过对侧)\n";
        }
    }

    void log_retract_ik_preview(
        Robot_Arm &arm,
        bool is_right,
        Eigen::Matrix<double, 1, 6> goal,
        const char *stage)
    {
        Eigen::RowVectorXd q = arm_get_joint_pos(arm);
        Eigen::RowVectorXd q_ik = q;
        Eigen::Matrix<double, 1, 6> g = goal;
        const int ik = arm.Inverse_Kinematics(g, q_ik);
        if (ik != 0)
        {
            std::cout << "[home] " << (is_right ? "右" : "左") << " " << stage
                      << " 混合逆解预览失败 code=" << ik << "，仍下发解锁J2直线\n";
            return;
        }
        const int n = static_cast<int>(std::min(q.size(), q_ik.size()));
        double max_abs = 0.0;
        std::cout << std::fixed << std::setprecision(1)
                  << "[home] " << (is_right ? "右" : "左") << " " << stage << " Δq_deg=[";
        for (int i = 0; i < n; ++i)
        {
            const double d = std::abs(q_ik(i) - q(i));
            if (d > max_abs)
                max_abs = d;
            std::cout << (d / rad);
            if (i + 1 < n)
                std::cout << ",";
        }
        std::cout << "] max=" << (max_abs / rad) << "°";
        if (max_abs > kRetractMaxJointJumpRad)
            std::cout << "（关节变化较大，允许J2随路径调整）";
        std::cout << "\n";
    }

    bool retract_phase_move(
        Robot_Arm &arm_r,
        Robot_Arm &arm_l,
        Eigen::Matrix<double, 1, 6> goal_r,
        bool want_r,
        Eigen::Matrix<double, 1, 6> goal_l,
        bool want_l,
        const char *stage,
        bool check_zone)
    {
        if (hardware_abort_requested())
            return false;
        if (right_arm_motors_locked())
            want_r = false;
        if (check_zone && want_r)
            clamp_retract_goal_y(true, goal_r);
        if (check_zone && want_l)
            clamp_retract_goal_y(false, goal_l);

        const Eigen::Matrix<double, 1, 6> cur_r = arm_get_tcp_pos(arm_r);
        const Eigen::Matrix<double, 1, 6> cur_l = arm_get_tcp_pos(arm_l);
        bool move_r = want_r && (!retract_xyz_close(cur_r, goal_r) || !retract_rpy_close(cur_r, goal_r));
        bool move_l = want_l && (!retract_xyz_close(cur_l, goal_l) || !retract_rpy_close(cur_l, goal_l));
        if (move_r)
            log_retract_ik_preview(arm_r, true, goal_r, stage);
        if (move_l)
            log_retract_ik_preview(arm_l, false, goal_l, stage);
        if (!move_r && !move_l)
        {
            const bool r_ok = !want_r ||
                              (retract_xyz_close(cur_r, goal_r) && retract_rpy_close(cur_r, goal_r));
            const bool l_ok = !want_l ||
                              (retract_xyz_close(cur_l, goal_l) && retract_rpy_close(cur_l, goal_l));
            return r_ok && l_ok;
        }

        ArmLineMoveResult out;
        {
            ArmLineMoveDebugStage dbg(stage);
            out = arm_dual_line_move_selective(
                arm_r, goal_r, move_r, arm_l, goal_l, move_l, 0.2);
        }
        bool ok = true;
        if (move_r && out.ret_r != 0)
        {
            log_arm_traj_plan_fail("右手(home)", out.ret_r);
            ok = false;
        }
        if (move_l && out.ret_l != 0)
        {
            log_arm_traj_plan_fail("左手(home)", out.ret_l);
            ok = false;
        }
        if (want_r)
            log_retract_arm_state(stage, true, arm_r);
        if (want_l)
            log_retract_arm_state(stage, false, arm_l);
        if (check_zone && want_r && !arm_y_allowed_right(arm_get_tcp_pos(arm_r)(1)))
        {
            std::cerr << "[home] 右手结束后仍越左区边界\n";
            ok = false;
        }
        if (check_zone && want_l && !arm_y_allowed_left(arm_get_tcp_pos(arm_l)(1)))
        {
            std::cerr << "[home] 左手结束后仍越右区边界\n";
            ok = false;
        }
        return ok && !hardware_abort_requested();
    }

    bool move_arms_retract_to(
        Robot_Arm &arm_r,
        Robot_Arm &arm_l,
        Eigen::Matrix<double, 1, 6> standby_r,
        Eigen::Matrix<double, 1, 6> standby_l,
        bool check_zone,
        const char *pose_name)
    {
        Eigen::Matrix<double, 1, 6> cur_r = arm_get_tcp_pos(arm_r);
        Eigen::Matrix<double, 1, 6> cur_l = arm_get_tcp_pos(arm_l);

        std::cout << "[home] 安全抬升/退出料盘 → " << pose_name << "\n";
        if (right_arm_motors_locked())
            std::cout << "[home] 右臂电机已锁定，本段只动左臂\n";
        log_retract_arm_state("起点", true, arm_r);
        log_retract_arm_state("起点", false, arm_l);

        if ((!right_arm_motors_locked() && arm_pose_needs_joint_home(arm_r)) ||
            arm_pose_needs_joint_home(arm_l))
        {
            std::cout << "[home] 垂臂直接关节回 yaml standby，不限制起点关节限位\n";
            const bool joint_ok =
                retract_joint_to_standby(arm_r, arm_l, standby_r, standby_l, check_zone);
            log_retract_arm_state("完成", true, arm_r);
            log_retract_arm_state("完成", false, arm_l);
            if (hardware_abort_requested())
                return false;
            return joint_ok;
        }

        const double clear_z_r = std::max(cur_r(2), standby_r(2));
        const double clear_z_l = std::max(cur_l(2), standby_l(2));

        auto hold_rpy_goal = [](const Eigen::Matrix<double, 1, 6> &cur,
                                double x, double y, double z) {
            Eigen::Matrix<double, 1, 6> g = cur;
            g(0) = x;
            g(1) = y;
            g(2) = z;
            copy_current_rpy(cur, g);
            return g;
        };

        bool ok = true;

        // 1) 竖直抬离料盘：XY/RPY 保持当前，避免斜插扫台面、避免无谓转腕。
        {
            cur_r = arm_get_tcp_pos(arm_r);
            cur_l = arm_get_tcp_pos(arm_l);
            const bool lift_r = cur_r(2) < standby_r(2) - 0.01;
            const bool lift_l = cur_l(2) < standby_l(2) - 0.01;
            if (lift_r || lift_l)
            {
                Eigen::Matrix<double, 1, 6> lift_r_g =
                    hold_rpy_goal(cur_r, cur_r(0), cur_r(1), clear_z_r);
                Eigen::Matrix<double, 1, 6> lift_l_g =
                    hold_rpy_goal(cur_l, cur_l(0), cur_l(1), clear_z_l);
                std::cout << std::fixed << std::setprecision(4)
                          << "[home] 竖直抬升 右 z " << cur_r(2) << "→" << lift_r_g(2)
                          << " 左 z " << cur_l(2) << "→" << lift_l_g(2) << "\n";
                if (!retract_phase_move(
                        arm_r, arm_l, lift_r_g, lift_r, lift_l_g, lift_l, "home竖直抬升", false))
                    ok = false;
                if (hardware_abort_requested())
                    return false;
            }
        }

        // 2) 若已越过左右中缝，先沿 Y 退回本侧边界（保持当前 X/Z/RPY）。home_tcp 不卡分区。
        if (check_zone)
        {
            cur_r = arm_get_tcp_pos(arm_r);
            cur_l = arm_get_tcp_pos(arm_l);
            const bool pull_r = !right_arm_motors_locked() && !arm_y_allowed_right(cur_r(1));
            const bool pull_l = !arm_y_allowed_left(cur_l(1));
            if (pull_r || pull_l)
            {
                Eigen::Matrix<double, 1, 6> pull_r_g =
                    hold_rpy_goal(cur_r, cur_r(0), column_split_y(), cur_r(2));
                Eigen::Matrix<double, 1, 6> pull_l_g =
                    hold_rpy_goal(cur_l, cur_l(0),
                                  column_split_y() + kRetractOwnSideMarginM, cur_l(2));
                std::cout << "[home] 越分区，先沿Y退回本侧边界（不改姿态）\n";
                if (!retract_phase_move(
                        arm_r, arm_l, pull_r_g, pull_r, pull_l_g, pull_l, "home退回本侧Y", true))
                    ok = false;
                if (hardware_abort_requested())
                    return false;
            }
        }

        // 3) 本侧分区内平移到 standby XY，Z 保持抬升高度，RPY 仍用当前值。
        {
            cur_r = arm_get_tcp_pos(arm_r);
            cur_l = arm_get_tcp_pos(arm_l);
            bool xy_r = !right_arm_motors_locked() &&
                        (!check_zone || arm_y_allowed_right(cur_r(1)));
            bool xy_l = !check_zone || arm_y_allowed_left(cur_l(1));
            if (!right_arm_motors_locked() && !xy_r)
            {
                std::cerr << "[home] 右手仍越区，跳过平移，避免扫过左区\n";
                ok = false;
            }
            if (!xy_l)
            {
                std::cerr << "[home] 左手仍越区，跳过平移，避免扫过右区\n";
                ok = false;
            }
            Eigen::Matrix<double, 1, 6> xy_r_g =
                hold_rpy_goal(cur_r, standby_r(0), standby_r(1), cur_r(2));
            Eigen::Matrix<double, 1, 6> xy_l_g =
                hold_rpy_goal(cur_l, standby_l(0), standby_l(1), cur_l(2));
            std::cout << "[home] 保持当前姿态，平移到准备位 XY"
                      << (check_zone ? "（卡本侧分区）\n" : "（不卡中缝）\n");
            if (!retract_phase_move(
                    arm_r, arm_l, xy_r_g, xy_r, xy_l_g, xy_l, "home平移到准备XY", check_zone))
                ok = false;
            if (hardware_abort_requested())
                return false;
        }

        auto near_standby_xy = [](const Eigen::Matrix<double, 1, 6> &cur,
                                  const Eigen::Matrix<double, 1, 6> &sb) {
            // 左臂 XY 常停在 4cm 级误差；3cm 会跳过落 Z，左右高度差一整段。
            return std::hypot(cur(0) - sb(0), cur(1) - sb(1)) <= 0.08;
        };

        // 4) 已在准备位上方，再竖直落到 standby Z（仍不转腕）。
        {
            cur_r = arm_get_tcp_pos(arm_r);
            cur_l = arm_get_tcp_pos(arm_l);
            const bool down_r = near_standby_xy(cur_r, standby_r) &&
                                (!check_zone || arm_y_allowed_right(cur_r(1))) &&
                                std::abs(cur_r(2) - standby_r(2)) > 0.01;
            const bool down_l = near_standby_xy(cur_l, standby_l) &&
                                (!check_zone || arm_y_allowed_left(cur_l(1))) &&
                                std::abs(cur_l(2) - standby_l(2)) > 0.01;
            if (down_r || down_l)
            {
                Eigen::Matrix<double, 1, 6> z_r =
                    hold_rpy_goal(cur_r, standby_r(0), standby_r(1), standby_r(2));
                Eigen::Matrix<double, 1, 6> z_l =
                    hold_rpy_goal(cur_l, standby_l(0), standby_l(1), standby_l(2));
                std::cout << std::fixed << std::setprecision(4)
                          << "[home] 竖直校正准备Z 右 " << cur_r(2) << "→" << standby_r(2)
                          << (down_r ? "" : " 跳过")
                          << " 左 " << cur_l(2) << "→" << standby_l(2)
                          << (down_l ? "" : " 跳过") << "\n";
                if (!retract_phase_move(
                        arm_r, arm_l, z_r, down_r, z_l, down_l, "home落到准备Z", check_zone))
                    ok = false;
                if (hardware_abort_requested())
                    return false;
            }
        }

        // 5) 已在准备位 XYZ，仅当姿态差够大才转腕；否则跳过，避免无谓旋转。
        {
            cur_r = arm_get_tcp_pos(arm_r);
            cur_l = arm_get_tcp_pos(arm_l);
            const bool rpy_r = near_standby_xy(cur_r, standby_r) &&
                               (!check_zone || arm_y_allowed_right(cur_r(1))) &&
                               !retract_rpy_close(cur_r, standby_r);
            const bool rpy_l = near_standby_xy(cur_l, standby_l) &&
                               (!check_zone || arm_y_allowed_left(cur_l(1))) &&
                               !retract_rpy_close(cur_l, standby_l);
            if (rpy_r || rpy_l)
            {
                std::cout << "[home] 准备位原地转到 standby 姿态\n";
                Eigen::Matrix<double, 1, 6> rpy_goal_r = standby_r;
                Eigen::Matrix<double, 1, 6> rpy_goal_l = standby_l;
                rpy_goal_r(0) = cur_r(0);
                rpy_goal_r(1) = cur_r(1);
                rpy_goal_r(2) = standby_r(2);
                rpy_goal_l(0) = cur_l(0);
                rpy_goal_l(1) = cur_l(1);
                rpy_goal_l(2) = standby_l(2);
                if (!retract_phase_move(
                        arm_r, arm_l, rpy_goal_r, rpy_r, rpy_goal_l, rpy_l, "home准备姿态", check_zone))
                    ok = false;
            }
            else
                std::cout << "[home] 当前腕角已接近 standby，或尚未到准备XY，跳过转腕\n";
        }

        // 6) 转腕后若 Z 仍偏，再竖直对齐，避免左右手高度不一致。
        {
            cur_r = arm_get_tcp_pos(arm_r);
            cur_l = arm_get_tcp_pos(arm_l);
            const bool z_r = !right_arm_motors_locked() &&
                             (!check_zone || arm_y_allowed_right(cur_r(1))) &&
                             std::abs(cur_r(2) - standby_r(2)) > 0.015;
            const bool z_l = (!check_zone || arm_y_allowed_left(cur_l(1))) &&
                             std::abs(cur_l(2) - standby_l(2)) > 0.015;
            if (z_r || z_l)
            {
                Eigen::Matrix<double, 1, 6> z_r_g =
                    hold_rpy_goal(cur_r, cur_r(0), cur_r(1), standby_r(2));
                Eigen::Matrix<double, 1, 6> z_l_g =
                    hold_rpy_goal(cur_l, cur_l(0), cur_l(1), standby_l(2));
                std::cout << std::fixed << std::setprecision(4)
                          << "[home] 转腕后Z仍偏，再竖直对齐 右 " << cur_r(2) << "→"
                          << standby_r(2) << " 左 " << cur_l(2) << "→" << standby_l(2) << "\n";
                if (!retract_phase_move(
                        arm_r, arm_l, z_r_g, z_r, z_l_g, z_l, "home转腕后对齐Z", check_zone))
                    ok = false;
            }
        }

        log_retract_arm_state("完成", true, arm_r);
        log_retract_arm_state("完成", false, arm_l);
        if (hardware_abort_requested())
            return false;
        return ok;
    }

    bool move_arms_to_standby(
        Robot_Arm &arm_r,
        Robot_Arm &arm_l,
        int row_pose_group,
        bool tray2_place_rpy)
    {
        Eigen::Matrix<double, 1, 6> standby_r = make_standby_pos_right();
        Eigen::Matrix<double, 1, 6> standby_l = make_standby_pos_left();
        if (row_pose_group == 1 || row_pose_group == 2 || row_pose_group == 3 ||
            row_pose_group == 6)
        {
            const GraspRpyDeg &rpy_r = tray2_place_rpy
                                           ? tray2_place_rpy_deg_for_ready(row_pose_group, true)
                                           : grasp_rpy_deg_for_ready(row_pose_group, true);
            const GraspRpyDeg &rpy_l = tray2_place_rpy
                                           ? tray2_place_rpy_deg_for_ready(row_pose_group, false)
                                           : grasp_rpy_deg_for_ready(row_pose_group, false);
            standby_r(3) = rpy_r.rx * rad;
            standby_r(4) = rpy_r.ry * rad;
            standby_r(5) = rpy_r.rz * rad;
            standby_l(3) = rpy_l.rx * rad;
            standby_l(4) = rpy_l.ry * rad;
            standby_l(5) = rpy_l.rz * rad;
            std::cout << std::fixed << std::setprecision(1)
                      << (tray2_place_rpy ? "[home] 料盘2放置准备 tray2ready" : "[home] 准备姿态用 ready")
                      << row_pose_group
                      << " 左=(" << rpy_l.rx << "," << rpy_l.ry << "," << rpy_l.rz << ")deg"
                      << " 右=(" << rpy_r.rx << "," << rpy_r.ry << "," << rpy_r.rz << ")deg"
                      << " （" << ready_rows_label(row_pose_group) << "）\n";
        }
        return move_arms_retract_to(
            arm_r, arm_l, standby_r, standby_l, true,
            tray2_place_rpy ? "yaml standby（料盘2放置准备）" : "yaml standby（抓取准备）");
    }

    bool move_arms_to_home(Robot_Arm &arm_r, Robot_Arm &arm_l)
    {
        const Eigen::Matrix<double, 1, 6> home_r = make_home_tcp_right();
        const Eigen::Matrix<double, 1, 6> home_l = make_home_tcp_left();
        std::cout << std::fixed << std::setprecision(4)
                  << "[home] 使用专用 home_tcp 右=(" << home_r(0) << "," << home_r(1) << ","
                  << home_r(2) << ") 左=(" << home_l(0) << "," << home_l(1) << "," << home_l(2)
                  << ") rpy_deg 右=(" << (home_r(3) / rad) << "," << (home_r(4) / rad) << ","
                  << (home_r(5) / rad) << ") 左=(" << (home_l(3) / rad) << ","
                  << (home_l(4) / rad) << "," << (home_l(5) / rad) << ")\n";
        const Eigen::Matrix<double, 1, 6> cur_r = arm_get_tcp_pos(arm_r);
        const Eigen::Matrix<double, 1, 6> cur_l = arm_get_tcp_pos(arm_l);
        const bool r_ok = right_arm_motors_locked() ||
                          (retract_xyz_close(cur_r, home_r) && retract_rpy_close(cur_r, home_r));
        const bool l_ok = retract_xyz_close(cur_l, home_l) && retract_rpy_close(cur_l, home_l);
        if (r_ok && l_ok)
        {
            std::cout << "[home] 手臂已在 home_tcp，跳过分段回程\n";
            return !hardware_abort_requested();
        }
        return move_arms_retract_to(
            arm_r, arm_l, home_r, home_l, false, "yaml home_tcp");
    }

    bool move_arms_to_belt_ready(
        Robot_Arm &arm_r, Robot_Arm &arm_l, const ConveyorStationConfig &c)
    {
        const Eigen::Matrix<double, 1, 6> ready_r = c.tcp.right;
        const Eigen::Matrix<double, 1, 6> ready_l = c.tcp.left;
        std::cout << std::fixed << std::setprecision(4)
                  << "[belt] 准备 tcp 右=(" << ready_r(0) << "," << ready_r(1) << "," << ready_r(2)
                  << ") 左=(" << ready_l(0) << "," << ready_l(1) << "," << ready_l(2)
                  << ") rpy_deg 右=(" << (ready_r(3) / rad) << "," << (ready_r(4) / rad) << ","
                  << (ready_r(5) / rad) << ") 左=(" << (ready_l(3) / rad) << ","
                  << (ready_l(4) / rad) << "," << (ready_l(5) / rad) << ")\n";
        return move_arms_retract_to(
            arm_r, arm_l, ready_r, ready_l, false, "yaml conveyor.tcp 准备");
    }

    bool move_arms_to_belt_ready(Robot_Arm &arm_r, Robot_Arm &arm_l)
    {
        return move_arms_to_belt_ready(arm_r, arm_l, g_move_cfg.conveyor);
    }

    bool move_arms_to_belt_grasp_rpy(
        Robot_Arm &arm_r, Robot_Arm &arm_l, const ConveyorStationConfig &c)
    {
        Eigen::Matrix<double, 1, 6> pose_r = c.tcp.right;
        Eigen::Matrix<double, 1, 6> pose_l = c.tcp.left;
        pose_r(3) = c.grasp_rpy_right.rx * rad;
        pose_r(4) = c.grasp_rpy_right.ry * rad;
        pose_r(5) = c.grasp_rpy_right.rz * rad;
        pose_l(3) = c.grasp_rpy_left.rx * rad;
        pose_l(4) = c.grasp_rpy_left.ry * rad;
        pose_l(5) = c.grasp_rpy_left.rz * rad;
        std::cout << std::fixed << std::setprecision(4)
                  << "[belt] grasp_rpy 停在 tcp XYZ 右=(" << pose_r(0) << "," << pose_r(1) << ","
                  << pose_r(2) << ") 左=(" << pose_l(0) << "," << pose_l(1) << "," << pose_l(2)
                  << ") rpy_deg 右=(" << c.grasp_rpy_right.rx << "," << c.grasp_rpy_right.ry
                  << "," << c.grasp_rpy_right.rz << ") 左=(" << c.grasp_rpy_left.rx << ","
                  << c.grasp_rpy_left.ry << "," << c.grasp_rpy_left.rz << ")\n";
        return move_arms_retract_to(
            arm_r, arm_l, pose_r, pose_l, false, "yaml grasp_rpy_deg");
    }

    bool move_arms_to_belt_place(
        Robot_Arm &arm_r, Robot_Arm &arm_l, const ConveyorStationConfig &c)
    {
        const Eigen::Matrix<double, 1, 6> place_r = c.place_tcp.right;
        const Eigen::Matrix<double, 1, 6> place_l = c.place_tcp.left;
        std::cout << std::fixed << std::setprecision(4)
                  << "[belt] 放置 place_tcp 右=(" << place_r(0) << "," << place_r(1) << ","
                  << place_r(2) << ") 左=(" << place_l(0) << "," << place_l(1) << ","
                  << place_l(2) << ") rpy_deg 右=(" << (place_r(3) / rad) << ","
                  << (place_r(4) / rad) << "," << (place_r(5) / rad) << ") 左=("
                  << (place_l(3) / rad) << "," << (place_l(4) / rad) << ","
                  << (place_l(5) / rad) << ")\n";
        return move_arms_retract_to(
            arm_r, arm_l, place_r, place_l, false, "yaml conveyor.place_tcp 放置");
    }

    bool move_arms_to_belt_place(Robot_Arm &arm_r, Robot_Arm &arm_l)
    {
        return move_arms_to_belt_place(arm_r, arm_l, g_move_cfg.conveyor);
    }

    bool move_one_arm_line_to(
        Robot_Arm &arm,
        bool is_right,
        Eigen::Matrix<double, 1, 6> goal,
        const char *tag,
        bool hold_j2)
    {
        if (is_right && right_arm_motors_locked())
        {
            std::cout << "[belt] 右臂锁定，跳过 " << tag << "\n";
            return true;
        }
        if (hardware_abort_requested())
            return false;
        std::cout << std::fixed << std::setprecision(4)
                  << "[belt] " << tag << " " << (is_right ? "右" : "左")
                  << " xyz=(" << goal(0) << "," << goal(1) << "," << goal(2)
                  << ") rpy_deg=(" << (goal(3) / rad) << "," << (goal(4) / rad) << ","
                  << (goal(5) / rad) << ")";
        if (!is_right && !arm_y_allowed_left(goal(1)))
            std::cout << " 左手过中缝 y=" << goal(1) << "（传送带允许）";
        if (is_right && !arm_y_allowed_right(goal(1)))
            std::cout << " 右手过中缝 y=" << goal(1) << "（传送带允许）";
        if (hold_j2)
            std::cout << " 锁J2";
        std::cout << "\n";
        ArmLineMoveDebugStage dbg(tag);
        const int rc = hold_j2
                          ? arm_line_move_hold_redundant(
                                arm, goal, g_move_cfg.head_grasp.approach_vel_m_s)
                          : arm_line_move(arm, goal, g_move_cfg.head_grasp.approach_vel_m_s);
        if (rc != 0)
        {
            log_arm_traj_plan_fail(is_right ? "右手(belt)" : "左手(belt)", rc);
            return false;
        }
        const bool stopped = wait_arms_motors_stopped(
            is_right, !is_right, 15.0, 6, tag);
        if (!stopped)
            std::cerr << "[belt] " << tag << " 手臂未在超时内停稳，仍读当前位置\n";
        log_retract_arm_state(tag, is_right, arm, false);
        std::cout.flush();
        return stopped && !hardware_abort_requested();
    }

    bool move_arms_tcp_to_standby(Robot_Arm &arm_r, Robot_Arm &arm_l)
    {
        Eigen::Matrix<double, 1, 6> standby_r = make_standby_pos_right();
        Eigen::Matrix<double, 1, 6> standby_l = make_standby_pos_left();
        const Eigen::Matrix<double, 1, 6> cur_r = arm_get_tcp_pos(arm_r);
        const Eigen::Matrix<double, 1, 6> cur_l = arm_get_tcp_pos(arm_l);
        const bool move_r = !right_arm_motors_locked() &&
                            (!retract_xyz_close(cur_r, standby_r) || !retract_rpy_close(cur_r, standby_r));
        const bool move_l =
            !retract_xyz_close(cur_l, standby_l) || !retract_rpy_close(cur_l, standby_l);

        std::cout << "[arm] 夹后一次 TCP 直线回 yaml standby（XYZ+姿态）"
                  << (right_arm_motors_locked() ? "；右臂锁定只动左手" : "")
                  << (move_r || move_l ? "\n" : "，已到位跳过\n");
        if (!move_r && !move_l)
            return !hardware_abort_requested();

        ArmLineMoveDebugStage stage("夹后TCP直线回standby");
        const ArmLineMoveResult out = arm_dual_line_move_selective(
            arm_r, standby_r, move_r, arm_l, standby_l, move_l,
            g_move_cfg.head_grasp.return_vel_m_s);
        if (move_r && out.ret_r != 0)
        {
            log_arm_traj_plan_fail("右手(夹后回standby)", out.ret_r);
            return false;
        }
        if (move_l && out.ret_l != 0)
        {
            log_arm_traj_plan_fail("左手(夹后回standby)", out.ret_l);
            return false;
        }
        log_retract_arm_state("夹后TCP回standby", true, arm_r);
        log_retract_arm_state("夹后TCP回standby", false, arm_l);
        return !hardware_abort_requested();
    }

    bool move_arms_tcp_to_home(Robot_Arm &arm_r, Robot_Arm &arm_l)
    {
        Eigen::Matrix<double, 1, 6> home_r = make_home_tcp_right();
        Eigen::Matrix<double, 1, 6> home_l = make_home_tcp_left();
        const Eigen::Matrix<double, 1, 6> cur_r = arm_get_tcp_pos(arm_r);
        const Eigen::Matrix<double, 1, 6> cur_l = arm_get_tcp_pos(arm_l);
        const bool move_r = !right_arm_motors_locked() &&
                            (!retract_xyz_close(cur_r, home_r) || !retract_rpy_close(cur_r, home_r));
        const bool move_l =
            !retract_xyz_close(cur_l, home_l) || !retract_rpy_close(cur_l, home_l);

        std::cout << "[arm] 夹后一次 TCP 直线回 yaml home_tcp（XYZ+姿态）"
                  << (right_arm_motors_locked() ? "；右臂锁定只动左手" : "")
                  << (move_r || move_l ? "\n" : "，已到位跳过\n");
        if (!move_r && !move_l)
            return !hardware_abort_requested();

        ArmLineMoveDebugStage stage("夹后TCP直线回home");
        const ArmLineMoveResult out = arm_dual_line_move_selective(
            arm_r, home_r, move_r, arm_l, home_l, move_l,
            g_move_cfg.head_grasp.return_vel_m_s);
        if (move_r && out.ret_r != 0)
        {
            log_arm_traj_plan_fail("右手(夹后回home)", out.ret_r);
            return false;
        }
        if (move_l && out.ret_l != 0)
        {
            log_arm_traj_plan_fail("左手(夹后回home)", out.ret_l);
            return false;
        }
        log_retract_arm_state("夹后TCP回home", true, arm_r);
        log_retract_arm_state("夹后TCP回home", false, arm_l);
        return !hardware_abort_requested();
    }

    bool rotate_arms_in_place_to_standby_rpy(Robot_Arm &arm_r, Robot_Arm &arm_l)
    {
        const Eigen::Matrix<double, 1, 6> standby_r = make_standby_pos_right();
        const Eigen::Matrix<double, 1, 6> standby_l = make_standby_pos_left();
        Eigen::Matrix<double, 1, 6> cur_r = arm_get_tcp_pos(arm_r);
        Eigen::Matrix<double, 1, 6> cur_l = arm_get_tcp_pos(arm_l);
        Eigen::Matrix<double, 1, 6> goal_r = cur_r;
        Eigen::Matrix<double, 1, 6> goal_l = cur_l;
        goal_r(3) = standby_r(3);
        goal_r(4) = standby_r(4);
        goal_r(5) = standby_r(5);
        goal_l(3) = standby_l(3);
        goal_l(4) = standby_l(4);
        goal_l(5) = standby_l(5);

        constexpr double kSkipDeg = 1.0;
        const bool rot_r = !right_arm_motors_locked() &&
                           (cur_r.tail<3>() - goal_r.tail<3>()).cwiseAbs().maxCoeff() > kSkipDeg * rad;
        const bool rot_l =
            (cur_l.tail<3>() - goal_l.tail<3>()).cwiseAbs().maxCoeff() > kSkipDeg * rad;

        std::cout << std::fixed << std::setprecision(1)
                  << "[grasp] 入口原地转 yaml standby 姿态"
                  << " 左当前=("
                  << (cur_l(3) / rad) << "," << (cur_l(4) / rad) << "," << (cur_l(5) / rad)
                  << ") → (" << (goal_l(3) / rad) << "," << (goal_l(4) / rad) << ","
                  << (goal_l(5) / rad) << ") deg"
                  << (rot_l ? "" : " 已接近，跳过")
                  << " 右当前=("
                  << (cur_r(3) / rad) << "," << (cur_r(4) / rad) << "," << (cur_r(5) / rad)
                  << ") → (" << (goal_r(3) / rad) << "," << (goal_r(4) / rad) << ","
                  << (goal_r(5) / rad) << ") deg"
                  << (rot_r ? "\n" : " 已接近，跳过\n");
        if (!rot_r && !rot_l)
            return !hardware_abort_requested();

        ArmLineMoveDebugStage stage("grasp入口原地转standby姿态");
        const ArmLineMoveResult out = arm_dual_line_move_selective(
            arm_r, goal_r, rot_r, arm_l, goal_l, rot_l,
            g_move_cfg.head_grasp.approach_vel_m_s);
        if (rot_r && out.ret_r != 0)
        {
            log_arm_traj_plan_fail("右手(入口standby姿态)", out.ret_r);
            return false;
        }
        if (rot_l && out.ret_l != 0)
        {
            log_arm_traj_plan_fail("左手(入口standby姿态)", out.ret_l);
            return false;
        }
        const Eigen::Matrix<double, 1, 6> after_l = arm_get_tcp_pos(arm_l);
        const Eigen::Matrix<double, 1, 6> after_r = arm_get_tcp_pos(arm_r);
        std::cout << std::fixed << std::setprecision(1)
                  << "[grasp] 入口姿态到位 左=(" << (after_l(3) / rad) << ","
                  << (after_l(4) / rad) << "," << (after_l(5) / rad)
                  << ") 右=(" << (after_r(3) / rad) << ","
                  << (after_r(4) / rad) << "," << (after_r(5) / rad) << ") deg\n";
        return !hardware_abort_requested();
    }

    void refresh_head_move_flags(HeadAssignState &st)
    {
        st.have_r = head_hand_detected(st.right_hand_pos);
        st.have_l = head_hand_detected(st.left_hand_pos);
        st.move_r = st.have_r && head_move_allowed_right(st.right_hand_pos, st.goal_last_r);
        st.move_l = st.have_l && head_move_allowed_left(st.left_hand_pos, st.goal_last_l);
        if (right_arm_motors_locked())
        {
            st.have_r = false;
            st.move_r = false;
        }
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

    bool g_waist_layer3_synced = false;

    bool waist_layer3_is_synced()
    {
        return g_waist_layer3_synced;
    }

    void waist_layer3_set_synced(bool synced)
    {
        g_waist_layer3_synced = synced;
    }

    bool sync_waist_layer3_from_encoders(
        WaistRobot &waist,
        Eigen::Matrix<double, 1, 6> &layer3,
        const char *tag)
    {
        const auto &w = g_move_cfg.waist;
        Eigen::MatrixXd measured;
        try
        {
            measured = waist.getTcpPos();
        }
        catch (const std::exception &ex)
        {
            std::cerr << "[waist] " << tag << " 读取腰部编码器异常: " << ex.what() << "\n";
            g_waist_layer3_synced = false;
            return false;
        }

        if (measured.rows() != 1 || measured.cols() != 6 || !measured.allFinite())
        {
            std::cerr << "[waist] " << tag << " 腰部编码器回读无效，layer3 仍未同步\n";
            g_waist_layer3_synced = false;
            return false;
        }

        // 回读值落在行程外，多半是 CAN 没读到而不是腰真的在那里，采信它比不采信更危险。
        constexpr double kReadMarginM = 0.15;
        if (measured(0) < w.x_min - kReadMarginM || measured(0) > w.x_max + kReadMarginM ||
            measured(2) < w.z_min - kReadMarginM || measured(2) > w.z_max + kReadMarginM)
        {
            std::cerr << std::fixed << std::setprecision(4)
                      << "[waist] " << tag << " 腰部回读 x=" << measured(0)
                      << " z=" << measured(2) << " 超出行程 x[" << w.x_min << ',' << w.x_max
                      << "] z[" << w.z_min << ',' << w.z_max << "] ±" << kReadMarginM
                      << " m，拒绝采信，layer3 仍未同步\n";
            g_waist_layer3_synced = false;
            return false;
        }

        layer3 = measured;
        g_waist_layer3_synced = true;
        g_current_waist_layer3_z = layer3(2);
        std::cout << std::fixed << std::setprecision(4)
                  << "[waist] " << tag << " layer3 已按编码器同步 x=" << layer3(0)
                  << " y=" << layer3(1) << " z=" << layer3(2) << "（home z=" << w.layer3_home(2)
                  << " 偏差=" << layer3(2) - w.layer3_home(2) << " m）\n";
        const Eigen::MatrixXd qj = waist.getJointPos();
        if (qj.rows() == 1 && qj.cols() >= 5)
        {
            std::cout << std::fixed << std::setprecision(1)
                      << "[waist] " << tag << " 几何deg 踝/膝/髋/滚/回=["
                      << (qj(0) / rad) << "," << (qj(1) / rad) << "," << (qj(2) / rad) << ","
                      << (qj(3) / rad) << "," << (qj(4) / rad) << "]\n";
        }
        return true;
    }

    bool waist_layer3_move_z_only(
        WaistRobot &waist,
        Eigen::Matrix<double, 1, 6> &layer3,
        double z_target,
        bool settle)
    {
        if (!sync_waist_layer3_from_encoders(waist, layer3, "竖直升降前"))
            return false;
        const double z0 = layer3(2);
        layer3(2) = z_target;
        clamp_waist_layer3_xyz(layer3);
        if (std::abs(layer3(2) - z0) <= 1e-3)
        {
            std::cout << std::fixed << std::setprecision(4)
                      << "[waist] 竖直 Z 已在 " << z0 << " m，跳过\n";
            g_current_waist_layer3_z = layer3(2);
            return true;
        }
        std::cout << std::fixed << std::setprecision(4)
                  << "[waist] 竖直 Z " << z0 << " → " << layer3(2)
                  << " 锁 x=" << layer3(0) << " y=" << layer3(1) << " m\n";
        const int rc = waist.moveLToPosZ(layer3(2), 0.1);
        if (rc != 0)
        {
            std::cerr << "[waist] 竖直升降失败 code=" << rc << "\n";
            sync_waist_layer3_from_encoders(waist, layer3, "竖直升降失败后");
            return false;
        }
        if (settle)
            hardware_abort_sleep_ms(static_cast<int>(g_move_cfg.waist.move_settle_sec * 1000.0));
        if (!sync_waist_layer3_from_encoders(waist, layer3, "竖直升降后"))
            g_current_waist_layer3_z = layer3(2);
        return true;
    }

    bool waist_layer3_move_x_only(
        WaistRobot &waist,
        Eigen::Matrix<double, 1, 6> &layer3,
        double x_target,
        bool settle)
    {
        if (!sync_waist_layer3_from_encoders(waist, layer3, "腰平移前"))
            return false;
        const double x0 = layer3(0);
        layer3(0) = x_target;
        clamp_waist_layer3_xyz(layer3);
        if (std::abs(layer3(0) - x0) <= 0.01)
        {
            std::cout << std::fixed << std::setprecision(4)
                      << "[waist] 平移 X 已在 " << x0 << " m，跳过\n";
            layer3(0) = x0;
            return true;
        }
        std::cout << std::fixed << std::setprecision(4)
                  << "[waist] 平移 X " << x0 << " → " << layer3(0)
                  << " 锁 y=" << layer3(1) << " z=" << layer3(2) << " m\n";
        const int rc = waist.moveLToPos(layer3, 0.1);
        if (rc != 0)
        {
            std::cerr << "[waist] 平移 X 失败 code=" << rc << "\n";
            sync_waist_layer3_from_encoders(waist, layer3, "腰平移失败后");
            return false;
        }
        if (settle)
            hardware_abort_sleep_ms(static_cast<int>(g_move_cfg.waist.move_settle_sec * 1000.0));
        if (!sync_waist_layer3_from_encoders(waist, layer3, "腰平移后"))
            g_current_waist_layer3_z = layer3(2);
        return true;
    }

    double waist_layer3_lower_z(
        WaistRobot &waist,
        Eigen::Matrix<double, 1, 6> &layer3,
        double drop_m)
    {
        if (drop_m <= 1e-9)
            return 0.0;
        if (!waist_layer3_is_synced())
        {
            std::cerr << "[waist] layer3 未与编码器同步，拒绝降腰（先执行 home）\n";
            return 0.0;
        }
        if (!sync_waist_layer3_from_encoders(waist, layer3, "降腰前"))
            return 0.0;
        const double before = layer3(2);
        if (!waist_layer3_move_z_only(waist, layer3, before - drop_m, true))
            return 0.0;
        const double actual = before - layer3(2);
        if (actual <= 1e-9)
        {
            std::cout << "[waist] 抓取降腰跳过：已在 z_min=" << g_move_cfg.waist.z_min
                      << " 当前 z=" << before << "\n";
        }
        return std::max(0.0, actual);
    }

    bool assign_from_tray_holes(
        const TrayDetectResult &tray,
        HeadAssignState &st,
        bool skip_right,
        bool skip_left)
    {
        auto set_invalid = [](double pos[3]) {
            pos[0] = -1.0;
            pos[1] = 0.0;
            pos[2] = 0.0;
        };
        set_invalid(st.right_hand_pos);
        set_invalid(st.left_hand_pos);
        st.xy_from_tray = false;
        st.row_r = 0;
        st.row_l = 0;
        st.row_aruco_r = 0;
        st.row_aruco_l = 0;
        st.col_r = 0;
        st.col_l = 0;
        st.col_aruco_r = 0;
        st.col_aruco_l = 0;
        st.zones = {};
        if (!tray.ok)
            return false;

        struct Cand
        {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            int id = 0;
            int col = 0;
            int row = 0;
            int row_aruco = 0;
            int col_aruco = 0;
            double conf = 0.0;
        };

        const auto &valid = g_move_cfg.grasp_valid;
        const int right_max_col = std::max(1, g_move_cfg.grasp_zone.column_count / 2);

        double origin_y = tray.origin_y;
        if (!std::isfinite(origin_y))
        {
            double acc = 0.0;
            int n = 0;
            for (const TrayHoleResult &h : tray.holes)
            {
                const double y = std::isfinite(h.y_level) ? h.y_level : h.y;
                if (!h.in_robot || !std::isfinite(y))
                    continue;
                acc += y;
                ++n;
            }
            if (n > 0)
                origin_y = acc / static_cast<double>(n);
        }
        if (std::isfinite(origin_y))
            set_live_tray_origin_y(origin_y);

        std::map<int, std::pair<double, int>> col_y_acc;
        for (const TrayHoleResult &h : tray.holes)
        {
            const double y = std::isfinite(h.y_level) ? h.y_level : h.y;
            if (!h.in_robot || !std::isfinite(y) || h.col < 1)
                continue;
            col_y_acc[h.col].first += y;
            col_y_acc[h.col].second += 1;
        }
        bool cols_flipped = false;
        if (col_y_acc.size() >= 2)
        {
            const int c_lo = col_y_acc.begin()->first;
            const int c_hi = col_y_acc.rbegin()->first;
            if (c_hi > c_lo && col_y_acc[c_lo].second > 0 && col_y_acc[c_hi].second > 0)
            {
                const double y_lo = col_y_acc[c_lo].first / static_cast<double>(col_y_acc[c_lo].second);
                const double y_hi = col_y_acc[c_hi].first / static_cast<double>(col_y_acc[c_hi].second);
                // ArUco col1 在盘系 -Y（机器人右侧）。若小号列的基座 Y 反而更大，说明盘转了 180°。
                cols_flipped = y_lo > y_hi;
            }
        }

        std::map<int, std::pair<double, int>> row_x_acc;
        for (const TrayHoleResult &h : tray.holes)
        {
            const double x = std::isfinite(h.x_level) ? h.x_level : h.x;
            if (!h.in_robot || !std::isfinite(x) || h.row < 1)
                continue;
            row_x_acc[h.row].first += x;
            row_x_acc[h.row].second += 1;
        }
        std::vector<std::pair<double, int>> ranked;
        ranked.reserve(row_x_acc.size());
        for (const auto &kv : row_x_acc)
        {
            if (kv.second.second <= 0)
                continue;
            ranked.push_back({kv.second.first / static_cast<double>(kv.second.second), kv.first});
        }
        std::sort(ranked.begin(), ranked.end(),
                  [](const std::pair<double, int> &a, const std::pair<double, int> &b) {
                      return a.first > b.first;
                  });
        std::map<int, int> tray_to_from_robot;
        for (size_t i = 0; i < ranked.size(); ++i)
            tray_to_from_robot[ranked[i].second] = static_cast<int>(i) + 1;

        const int want_class = tray_assign_class_id();
        std::cout << "\n--- 料盘孔位分配 (ArUco XY + YOLO class" << want_class
                  << (want_class == kEmptyHoleClassId ? " 空孔放置" : "") << ") ---\n";
        std::cout << "[tray] 行列绑定料盘系：+X前=row1  -X后=row6  -Y右=col1  +Y左=col6\n";
        std::cout << std::fixed << std::setprecision(4)
                  << "[tray] 左右分列按 ArUco 列号，不用机器人 Y=0"
                  << "；盘心 y=" << (std::isfinite(origin_y) ? origin_y : 0.0)
                  << (std::isfinite(origin_y) ? "（基座）" : "（未知）")
                  << (cols_flipped ? "；列号已对调(盘约转180°)\n" : "\n");
        if (!ranked.empty())
        {
            const int r1 = tray_to_from_robot.count(1) ? tray_to_from_robot[1] : 0;
            if (r1 == static_cast<int>(ranked.size()) && ranked.size() >= 2)
                std::cout << "[tray] 料盘相对机器人约转了 180°：ArUco row1 现在离机器人最近，"
                             "抓取姿态改按基座X最远/最近匹配\n";
            else if (r1 == 1)
                std::cout << "[tray] 料盘朝向正常：ArUco row1 离机器人最远\n";
            for (const auto &p : ranked)
            {
                const int fr = tray_to_from_robot[p.second];
                const char *tag = (fr == 1) ? "最远" :
                    (fr == static_cast<int>(ranked.size()) ? "最近" : "中间");
                std::cout << std::fixed << std::setprecision(4)
                          << "  ArUco row" << p.second << " 基座x=" << p.first
                          << " → " << tag << " from_robot=" << fr << "\n";
            }
        }

        auto from_robot_row = [&](int tray_row) -> int {
            const auto it = tray_to_from_robot.find(tray_row);
            return it != tray_to_from_robot.end() ? it->second : tray_row;
        };

        std::vector<Cand> right_zone;
        std::vector<Cand> left_zone;
        for (const TrayHoleResult &h : tray.holes)
        {
            if (h.class_id != want_class)
                continue;
            const double x = std::isfinite(h.x_level) ? h.x_level : h.x;
            const double y = std::isfinite(h.y_level) ? h.y_level : h.y;
            if (!h.in_robot || !std::isfinite(x) || !std::isfinite(y))
                continue;
            if (x < valid.x_min || x > valid.x_max)
            {
                std::cout << "  hole " << h.id << " 过滤(工作包络x) x=" << x << " y=" << y << "\n";
                continue;
            }
            const int col = tray_assign_col_from_aruco(h.col, cols_flipped);
            if (col < 1)
            {
                std::cout << "  hole " << h.id << " 过滤(无 ArUco 列) col=" << h.col << "\n";
                continue;
            }
            const bool is_right_col = col <= right_max_col;
            if (is_right_col
                    ? (y < valid.right_y_min || y > valid.right_y_max)
                    : (y < valid.left_y_min || y > valid.left_y_max))
            {
                std::cout << "  hole " << h.id << " 过滤(工作包络y) x=" << x << " y=" << y
                          << (is_right_col ? " 右\n" : " 左\n");
                continue;
            }
            const double z = std::isfinite(h.top_z_level)
                                 ? h.top_z_level
                                 : (std::isfinite(h.top_z) ? h.top_z : expected_object_top_z());
            Cand c;
            c.x = x;
            c.y = y;
            c.z = z;
            c.id = h.id;
            c.col = col;
            c.row = from_robot_row(h.row);
            c.row_aruco = h.row;
            c.col_aruco = h.col;
            c.conf = h.conf;
            if (col <= right_max_col)
                right_zone.push_back(c);
            else
                left_zone.push_back(c);
            std::cout << std::fixed << std::setprecision(4)
                      << "  hole " << h.id << " ArUco row=" << h.row << " col=" << h.col
                      << " from_robot=" << c.row << " 分配col=" << col
                      << (col <= right_max_col ? " 右" : " 左")
                      << " xy=(" << x << "," << y << ") top_z=" << z
                      << " conf=" << std::setprecision(3) << h.conf << "\n";
        }

        auto zone_has_near = [](const std::vector<Cand> &zone) {
            for (const Cand &c : zone)
            {
                if (!tray_row_is_far(c.row))
                    return true;
            }
            return false;
        };
        const bool skip_r = skip_right || right_arm_motors_locked();
        const bool skip_l = skip_left;
        if (skip_r && skip_l)
        {
            std::cout << "[tray_xy] 左右手都已持件，不再分配\n";
            return false;
        }
        if (skip_r)
            std::cout << "[tray_xy] 右手已持件或锁定，本轮只给左手补抓\n";
        if (skip_l)
            std::cout << "[tray_xy] 左手已持件，本轮只给右手补抓\n";

        auto zone_has_far = [](const std::vector<Cand> &zone) {
            for (const Cand &c : zone)
            {
                if (tray_row_is_far(c.row))
                    return true;
            }
            return false;
        };
        const bool precision = tray2_precision_test();
        const bool prefer_front =
            precision &&
            ((!skip_l && zone_has_far(left_zone)) || (!skip_r && zone_has_far(right_zone)));
        const bool prefer_near =
            !precision &&
            ((!skip_l && zone_has_near(left_zone)) ||
             (!skip_r && zone_has_near(right_zone)));
        if (precision)
        {
            if (prefer_front)
                std::cout << "[tray_xy] 精度测试：先前三排，从离机器人近的一行开始"
                          << "（from_robot " << g_move_cfg.grasp_zone.far_row_max
                          << "→1），不跳到最后一排\n";
            else
                std::cout << "[tray_xy] 精度测试：前三排已空，后三排 from_robot "
                          << (g_move_cfg.grasp_zone.far_row_max + 1) << "-6\n";
        }
        else if (g_move_cfg.grasp_zone.far_row_max > 0)
        {
            if (prefer_near)
                std::cout << "[tray_xy] 3+3 近区优先：先抓离机器人近的三排"
                          << "（from_robot " << (g_move_cfg.grasp_zone.far_row_max + 1)
                          << "-6），远三排暂缓\n";
            else
                std::cout << "[tray_xy] 3+3 近区已空，开始离机器人远的三排 from_robot 1-"
                          << g_move_cfg.grasp_zone.far_row_max << "\n";
        }

        auto in_prefer_band = [&](const Cand &c) {
            if (prefer_front)
                return tray_row_is_far(c.row);
            return !(prefer_near && tray_row_is_far(c.row));
        };
        auto leftmost_of = [](const std::vector<const Cand *> &zs) -> const Cand * {
            const Cand *best = nullptr;
            for (const Cand *c : zs)
            {
                if (c == nullptr)
                    continue;
                if (best == nullptr || c->col > best->col ||
                    (c->col == best->col && c->y > best->y))
                    best = c;
            }
            return best;
        };

        // 左右锁在同一行：先清空离机器人最近的一行。满排配对 (3,6)→(2,5)→(1,4)。
        // 一侧本行没料就等，禁止跳去更远行把配对手抽走。
        int shared_row = 0;
        if (precision)
        {
            int best = 0;
            bool any = false;
            auto consider = [&](const Cand &c, bool skip) {
                if (skip || !in_prefer_band(c))
                    return;
                // 前三排从靠近机器人的一行开始（3→2→1），不要一上来跳到最远的最后一排。
                if (!any || c.row > best)
                    best = c.row;
                any = true;
            };
            for (const Cand &c : right_zone)
                consider(c, skip_r);
            for (const Cand &c : left_zone)
                consider(c, skip_l);
            shared_row = any ? best : 0;
        }
        else
        {
            for (const Cand &c : right_zone)
            {
                if (!skip_r && in_prefer_band(c))
                    shared_row = std::max(shared_row, c.row);
            }
            for (const Cand &c : left_zone)
            {
                if (!skip_l && in_prefer_band(c))
                    shared_row = std::max(shared_row, c.row);
            }
        }

        std::vector<const Cand *> rights;
        std::vector<const Cand *> lefts;
        if (shared_row >= 1)
        {
            for (const Cand &c : right_zone)
            {
                if (!skip_r && in_prefer_band(c) && c.row == shared_row)
                    rights.push_back(&c);
            }
            for (const Cand &c : left_zone)
            {
                if (!skip_l && in_prefer_band(c) && c.row == shared_row)
                    lefts.push_back(&c);
            }
        }

        const Cand *pr = nullptr;
        const Cand *pl = nullptr;
        const int min_delta = g_move_cfg.grasp_zone.min_simultaneous_col_delta;
        if (skip_r)
        {
            pl = leftmost_of(lefts);
        }
        else if (skip_l)
        {
            pr = leftmost_of(rights);
        }
        else if (lefts.empty())
        {
            pr = leftmost_of(rights);
        }
        else if (rights.empty())
        {
            pl = leftmost_of(lefts);
        }
        else
        {
            for (const Cand *l : lefts)
            {
                for (const Cand *r : rights)
                {
                    if ((l->col - r->col) < min_delta)
                        continue;
                    const bool better =
                        pl == nullptr || l->col > pl->col ||
                        (l->col == pl->col && r->col > pr->col);
                    if (better)
                    {
                        pl = l;
                        pr = r;
                    }
                }
            }
            if (pl == nullptr)
            {
                pl = leftmost_of(lefts);
                std::cout << "[tray_xy] from_robot=" << shared_row
                          << " 本行列差不足同时，同行从左往右先抓左\n";
            }
        }
        if (shared_row >= 1)
        {
            std::cout << "[tray_xy] 选孔：同行 from_robot=" << shared_row
                      << " 从左往右；满排同时对 (3,6)/(2,5)/(1,4)"
                      << " 本行右" << rights.size() << "孔 左" << lefts.size() << "孔\n";
            if (pr != nullptr && pl == nullptr && !skip_l)
                std::cout << "[tray_xy] 本行仅右侧有料，左手等本行清空，不抢下一行\n";
            if (pl != nullptr && pr == nullptr && !skip_r && !rights.empty())
                std::cout << "[tray_xy] 本行先抓左，右手本轮停\n";
        }
        if (pr != nullptr)
        {
            st.right_hand_pos[0] = pr->x;
            st.right_hand_pos[1] = pr->y;
            st.right_hand_pos[2] = pr->z;
            st.row_r = pr->row;
            st.row_aruco_r = pr->row_aruco;
            st.col_r = pr->col;
            st.col_aruco_r = pr->col_aruco;
            st.zones.r_from_side = true;
            std::cout << std::fixed << std::setprecision(4)
                      << "[tray_xy] 右手 hole " << pr->id
                      << " ArUco row=" << pr->row_aruco << " col=" << pr->col_aruco
                      << " from_robot=" << pr->row << " 分配col=" << pr->col
                      << " xy=(" << pr->x << "," << pr->y << ")\n";
        }
        if (pl != nullptr)
        {
            st.left_hand_pos[0] = pl->x;
            st.left_hand_pos[1] = pl->y;
            st.left_hand_pos[2] = pl->z;
            st.row_l = pl->row;
            st.row_aruco_l = pl->row_aruco;
            st.col_l = pl->col;
            st.col_aruco_l = pl->col_aruco;
            st.zones.l_from_side = true;
            std::cout << std::fixed << std::setprecision(4)
                      << "[tray_xy] 左手 hole " << pl->id
                      << " ArUco row=" << pl->row_aruco << " col=" << pl->col_aruco
                      << " from_robot=" << pl->row << " 分配col=" << pl->col
                      << " xy=(" << pl->x << "," << pl->y << ")\n";
        }
        st.xy_from_tray = (pr != nullptr) || (pl != nullptr);
        return st.xy_from_tray;
    }

    void fill_head_assign_from_frame(
        RealSenseMultiCam &cameras,
        SegPoseBridge &bridge,
        const std::array<double, 16> &cam2robot,
        SegEngineId engine_id,
        HeadAssignState &st,
        bool skip_right,
        bool skip_left)
    {
        st = {};
        if (app_stop_requested() || hardware_abort_requested())
            return;

        std::array<double, 16> T = cam2robot;
        std::string ext_err;
        if (!load_head_cam2robot(T, ext_err))
        {
            std::cerr << "[head_ext] 现算失败: " << ext_err << "，用传入外参\n";
            T = cam2robot;
        }

        const int fuse_n = std::max(1, g_move_cfg.tray.fuse_frames);
        cameras.flush(CameraSlot::Head);
        std::vector<CameraFrameData> frames;
        frames.reserve(static_cast<size_t>(fuse_n));
        for (int i = 0; i < fuse_n; ++i)
        {
            CameraFrameData one = cameras.grab_wait(CameraSlot::Head);
            one = RealSenseMultiCam::prepare_frame_for_slot(std::move(one), CameraSlot::Head);
            if (!one.ok)
            {
                std::cerr << "[head] 取帧失败[" << i << "]: " << one.message << std::endl;
                continue;
            }
            frames.push_back(std::move(one));
        }
        if (frames.empty())
        {
            std::cerr << "[head] 取帧失败: 多帧融合没有有效帧\n";
            return;
        }
        const CameraFrameData &frame = frames.back();

        const int algorithm_id = algorithm_id_for_slot(CameraSlot::Head);
        const bool tray_ready = bridge.tray_engine_ready();
        const PoseRunResult yolo = bridge.run(
            frame, algorithm_id, !tray_ready && kDebugVisualize, CameraSlot::Head, -1, engine_id);
        if (!yolo.ok)
            std::cerr << "[head] YOLO 失败: " << yolo.message << std::endl;

        const PoseDetectionRecords yolo_all =
            pose_records_from_run(yolo, CameraSlot::Head, frame, -1, false);

        TrayDetectResult tray;
        if (tray_ready)
        {
            tray = bridge.run_tray_annotate_multiframe(
                frames, yolo_all, kDebugVisualize, CameraSlot::Head, tray2_place_active());
            if (!tray.ok)
                std::cerr << "[tray_xy] 料盘解算失败: " << tray.message
                          << "，不回退 YOLO，本轮不抓\n";
        }
        else
            std::cerr << "[tray_xy] 料盘引擎未就绪，不回退 YOLO，本轮不抓\n";

        if (!assign_from_tray_holes(tray, st, skip_right, skip_left))
        {
            if (tray.ok)
            {
                std::cout << "[tray_xy] 料盘已解算，"
                          << (right_arm_motors_locked() ? "左三列" : "本侧列")
                          << (g_tray_hole_task == TrayHoleTask::PlaceEmpty
                                  ? "没有空孔，不放\n"
                                  : "没有毛坯，不抓\n");
            }
            st.xy_from_tray = false;
        }
        fill_goal_last_from_head_assign(
            st.right_hand_pos, st.left_hand_pos, st.row_r, st.row_l,
            st.goal_last_r, st.goal_last_l);
        refresh_head_move_flags(st);
    }

    HeadAssignState detect_and_assign_head(
        RealSenseMultiCam &cameras,
        SegPoseBridge &bridge,
        const std::array<double, 16> &cam2robot,
        bool retry_if_incomplete,
        SegEngineId engine_id,
        bool skip_right,
        bool skip_left)
    {
        HeadAssignState st;
        if (kDebugVisualize)
            pose_vis_begin_phase(PoseVisLayout::Single, PoseVisPanel::Head, {PoseVisPanel::Head});
        fill_head_assign_from_frame(
            cameras, bridge, cam2robot, engine_id, st, skip_right, skip_left);

        const bool need_r = !skip_right && !right_arm_motors_locked();
        const bool need_l = !skip_left;
        const bool missing = (need_r && !st.have_r) || (need_l && !st.have_l);
        if (retry_if_incomplete && missing)
        {
            cout << "[head] 未分配到可动目标(右="
                 << (st.have_r ? "有" : "无") << " 左=" << (st.have_l ? "有" : "无")
                 << (skip_right ? " 右手已持件" : "")
                 << (skip_left ? " 左手已持件" : "")
                 << (right_arm_motors_locked() ? " 右臂锁定" : "")
                 << ")，头相机重试一次\n";
            if (kDebugVisualize)
                pose_vis_clear_panels({PoseVisPanel::Head});
            fill_head_assign_from_frame(
                cameras, bridge, cam2robot, engine_id, st, skip_right, skip_left);
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
        HeadAssignState st = detect_and_assign_head(
            cameras, bridge, cam2robot, retry_if_incomplete, engine_id,
            keep_grasped != nullptr && keep_grasped->grasped_r,
            keep_grasped != nullptr && keep_grasped->grasped_l);
        if (keep_grasped != nullptr)
            apply_head_assign_keep_grasped(st, *keep_grasped);
        return st;
    }

    bool tcp_matches_goal_pose(
        const Eigen::Matrix<double, 1, 6> &cur,
        const Eigen::Matrix<double, 1, 6> &goal,
        double xyz_tol_m,
        double rpy_tol_rad,
        double &xyz_err_m,
        double &rpy_err_rad)
    {
        xyz_err_m = (cur.head<3>() - goal.head<3>()).norm();
        rpy_err_rad = (cur.tail<3>() - goal.tail<3>()).cwiseAbs().maxCoeff();
        return xyz_err_m <= xyz_tol_m && rpy_err_rad <= rpy_tol_rad;
    }

    /**
     * 手相机启用时的旧 hover 接近流程。无手相机模式在此不移动，
     * 由 run_hand_approach_and_grasp 一次执行完整 A-B-C Bezier。
     * 自适应接近（安全优先）：
     * 1) 先在当前 xyz 转到目标姿态（避免用待机腕角直线接近）
     * 2) 再带着正确姿态直线到 hover
     * 3) 实测 TCP 超差则本臂失败，不继续转/下压
     * 固定姿态：一条直线到上方，同时转到 yaml 抓取 RPY（左 0/45/−10）。
     */
    ArmLineMoveResult run_head_approach_selective(
        Robot_Arm &arm_r,
        Robot_Arm &arm_l,
        HeadAssignState &st,
        bool move_r,
        bool move_l)
    {
        ArmLineMoveResult out;
        drop_if_y_split_blocks(st, move_r, move_l, "hover下发前");

        // 无手相机 + Bezier：本段不移动，最终 C 在 run_hand_approach_and_grasp 一次规划。
        // 无手相机 + 直线：A→B 到物体上方，同时转到该行姿态。
        if (!g_move_cfg.head_grasp.use_hand_camera)
        {
            if (g_move_cfg.head_grasp.use_bezier_grasp)
            {
                log_phase_banner("抓取流程：等待最终C，随后一次执行A-B-C Bezier");
                std::cout << "[bezier] 已废除旧的先到hover再直线下压流程；本阶段不移动机械臂\n";
                return out;
            }
            log_phase_banner("抓取流程：直线 A→B 到物体上方并转该行姿态");
            {
                ArmLineMoveDebugStage stage("A→B 到物体上方同时转抓取姿态");
                out = arm_dual_line_move_selective(
                    arm_r, st.goal_last_r, move_r, arm_l, st.goal_last_l, move_l,
                    g_move_cfg.head_grasp.approach_vel_m_s);
            }
            if (move_r && out.ret_r != 0)
                log_arm_traj_plan_fail("右手(A→B)", out.ret_r);
            if (move_l && out.ret_l != 0)
                log_arm_traj_plan_fail("左手(A→B)", out.ret_l);
            if (move_l && out.ret_l == 0)
            {
                const Eigen::Matrix<double, 1, 6> tcp = arm_get_tcp_pos(arm_l);
                std::cout << std::fixed << std::setprecision(1)
                          << "[arm] A→B 后左实际 rpy_deg=(" << (tcp(3) / rad) << ","
                          << (tcp(4) / rad) << "," << (tcp(5) / rad)
                          << ") 目标=(" << (st.goal_last_l(3) / rad) << ","
                          << (st.goal_last_l(4) / rad) << "," << (st.goal_last_l(5) / rad)
                          << ")\n";
            }
            return out;
        }

        log_phase_banner("抓取流程：先到物体上方");
        apply_adaptive_grasp_rpy_selective(arm_r, arm_l, st, move_r, move_l);

        if (g_grasp_adaptive_rpy)
        {
            const auto &hg = g_move_cfg.head_grasp;
            const double xyz_tol = std::max(0.005, hg.adaptive_approach_xyz_tol_m);
            const double rpy_tol = std::max(1.0, hg.adaptive_approach_rpy_tol_deg) * rad;

            auto need_orient = [](const Eigen::Matrix<double, 1, 6> &cur,
                                  const Eigen::Matrix<double, 1, 6> &goal) -> bool {
                return (cur.tail<3>() - goal.tail<3>()).cwiseAbs().maxCoeff() > 1e-3;
            };

            bool do_orient_r = false;
            bool do_orient_l = false;
            Eigen::Matrix<double, 1, 6> orient_r = st.goal_last_r;
            Eigen::Matrix<double, 1, 6> orient_l = st.goal_last_l;
            if (move_r)
            {
                const Eigen::Matrix<double, 1, 6> cur = arm_get_tcp_pos(arm_r);
                orient_r = cur;
                orient_r(3) = st.goal_last_r(3);
                orient_r(4) = st.goal_last_r(4);
                orient_r(5) = st.goal_last_r(5);
                do_orient_r = need_orient(cur, st.goal_last_r);
            }
            if (move_l)
            {
                const Eigen::Matrix<double, 1, 6> cur = arm_get_tcp_pos(arm_l);
                orient_l = cur;
                orient_l(3) = st.goal_last_l(3);
                orient_l(4) = st.goal_last_l(4);
                orient_l(5) = st.goal_last_l(5);
                do_orient_l = need_orient(cur, st.goal_last_l);
            }

            if (do_orient_r || do_orient_l)
            {
                ArmLineMoveDebugStage stage("自适应：原地转抓取姿态");
                const ArmLineMoveResult rpy = arm_dual_line_move_selective(
                    arm_r, orient_r, do_orient_r, arm_l, orient_l, do_orient_l, 0.15);
                if (do_orient_r)
                    out.ret_r = rpy.ret_r;
                if (do_orient_l)
                    out.ret_l = rpy.ret_l;
                if (do_orient_r && out.ret_r != 0)
                    log_arm_traj_plan_fail("右手(自适应原地转姿态)", out.ret_r);
                if (do_orient_l && out.ret_l != 0)
                    log_arm_traj_plan_fail("左手(自适应原地转姿态)", out.ret_l);
                if ((do_orient_r && out.ret_r != 0) || (do_orient_l && out.ret_l != 0))
                {
                    std::cerr << "[adapt-rpy] 原地转姿态失败，停止本侧接近（不下压）\n";
                    return out;
                }
            }

            {
                ArmLineMoveDebugStage stage("自适应：带姿态到物体上方");
                out = arm_dual_line_move_selective(
                    arm_r, st.goal_last_r, move_r, arm_l, st.goal_last_l, move_l,
                    g_move_cfg.head_grasp.approach_vel_m_s);
            }
            if (move_r && out.ret_r != 0)
                log_arm_traj_plan_fail("右手(自适应平移)", out.ret_r);
            if (move_l && out.ret_l != 0)
                log_arm_traj_plan_fail("左手(自适应平移)", out.ret_l);
            if ((move_r && out.ret_r != 0) || (move_l && out.ret_l != 0))
            {
                std::cerr << "[adapt-rpy] 带姿态平移失败，停止本侧接近（不下压）\n";
                return out;
            }

            auto verify_one = [&](Robot_Arm &arm,
                                  const Eigen::Matrix<double, 1, 6> &goal,
                                  const char *hand,
                                  int &ret_io) {
                double xyz_err = 0.0;
                double rpy_err = 0.0;
                const Eigen::Matrix<double, 1, 6> cur = arm_get_tcp_pos(arm);
                if (tcp_matches_goal_pose(cur, goal, xyz_tol, rpy_tol, xyz_err, rpy_err))
                {
                    std::cout << std::fixed << std::setprecision(3)
                              << "[adapt-rpy] " << hand << " 到位校验通过 xyz_err=" << xyz_err
                              << " m rpy_err=" << (rpy_err / rad) << "°\n";
                    return;
                }
                std::cerr << std::fixed << std::setprecision(3)
                          << "[adapt-rpy] " << hand << " 到位超差 xyz_err=" << xyz_err
                          << " m (tol=" << xyz_tol << ") rpy_err=" << (rpy_err / rad)
                          << "° (tol=" << (rpy_tol / rad) << ")，本臂失败不下压\n";
                ret_io = -7;
            };
            if (move_r)
                verify_one(arm_r, st.goal_last_r, "右", out.ret_r);
            if (move_l)
                verify_one(arm_l, st.goal_last_l, "左", out.ret_l);
            return out;
        }

        // 固定 yaml 姿态：一条 Line_Trajectory 到上方，同时转到 0/45/−10
        {
            ArmLineMoveDebugStage stage("到物体上方同时转抓取姿态");
            out = arm_dual_line_move_selective(
                arm_r, st.goal_last_r, move_r, arm_l, st.goal_last_l, move_l,
                g_move_cfg.head_grasp.approach_vel_m_s);
        }
        if (move_r && out.ret_r != 0)
            log_arm_traj_plan_fail("右手", out.ret_r);
        if (move_l && out.ret_l != 0)
            log_arm_traj_plan_fail("左手", out.ret_l);
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
        cout << "，本侧不抓取，停在当前 TCP\n";
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
        (void)hover_z_from_hand;
        if (!enable_r && !enable_l)
            return hs;

        if (!g_move_cfg.head_grasp.use_hand_camera)
        {
            std::cout << "[hand] 手相机已关闭，XY 用孔位+goal_x/y_offset，Z 用头相机顶面（可抬不可再往下探）\n";
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
            st.xy_from_tray);
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
                    apply_hand_xy_refine_keep_hole("右", true, base_pos_r, st.goal_last_r);
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
                    apply_hand_xy_refine_keep_hole("左", false, base_pos_l, st.goal_last_l);
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

    ArmLineMoveResult run_hand_approach_and_grasp(
        gripper::Gripper &g,
        Robot_Arm &arm_r,
        Robot_Arm &arm_l,
        HeadAssignState &st,
        HandMoveState &hs)
    {
        log_phase_banner(g_move_cfg.head_grasp.use_hand_camera
                             ? "抓取流程：从上方下压夹取"
                             : (g_move_cfg.head_grasp.use_bezier_grasp
                                    ? "抓取流程：二次Bezier A-B-C到最终抓取点"
                                    : "抓取流程：直线 B→C 下压到最终抓取点"));
        drop_if_y_split_blocks(st, hs, "hover/final下发前");
        if (hs.move_r && hs.move_l && dual_grasp_targets_too_close(st))
        {
            std::cout << "[col] 下压前列间隔不足，本轮只保留右手\n";
            hs.move_l = false;
        }

        if (hs.move_r &&
            !apply_final_grasp_z_from_hand(
                "右", true, st.row_r, hs.have_object_z_r, hs.object_z_r, st.goal_last_r))
        {
            hs.move_r = false;
        }
        if (hs.move_l &&
            !apply_final_grasp_z_from_hand(
                "左", false, st.row_l, hs.have_object_z_l, hs.object_z_l, st.goal_last_l))
        {
            hs.move_l = false;
        }
        if (!hs.move_r && !hs.move_l)
            return {};

        drop_if_y_split_blocks(st, hs, "final下发前");
        if (!hs.move_r && !hs.move_l)
            return {};

        // 自适应姿态也必须在完整曲线规划前确定，曲线执行中不再改写 C。
        if (!g_move_cfg.head_grasp.use_hand_camera)
            apply_adaptive_grasp_rpy_selective(arm_r, arm_l, st, hs.move_r, hs.move_l);

        ArmLineMoveResult out;
        if (!g_move_cfg.head_grasp.use_hand_camera &&
            g_move_cfg.head_grasp.use_bezier_grasp)
        {
            const auto &hg = g_move_cfg.head_grasp;
            const double gap = std::max(0.03, hg.bezier_ab_gap_m);
            auto make_lowered_a = [&](Robot_Arm &arm, const Eigen::Matrix<double, 1, 6> &c,
                                      bool do_move, Eigen::Matrix<double, 1, 6> &drop_pose)
                -> bool {
                if (!do_move)
                    return false;
                const Eigen::Matrix<double, 1, 6> cur = arm_get_tcp_pos(arm);
                const double b_z = c(2) + hg.bezier_guide_height_m;
                const double min_z = c(2) + 0.04;
                double a_z = std::min(cur(2), b_z - gap);
                a_z = std::max(a_z, min_z);
                if (cur(2) - a_z < 0.015)
                    return false;
                drop_pose = cur;
                drop_pose(2) = a_z;
                copy_current_rpy(cur, drop_pose);
                std::cout << std::fixed << std::setprecision(4)
                          << "[bezier] 降低起点 A z " << cur(2) << " → " << a_z
                          << " 使低于 B.z=" << b_z << "（不抬高B）\n";
                return true;
            };
            Eigen::Matrix<double, 1, 6> drop_r = st.goal_last_r;
            Eigen::Matrix<double, 1, 6> drop_l = st.goal_last_l;
            const bool drop_need_r = make_lowered_a(arm_r, st.goal_last_r, hs.move_r, drop_r);
            const bool drop_need_l = make_lowered_a(arm_l, st.goal_last_l, hs.move_l, drop_l);
            if (drop_need_r || drop_need_l)
            {
                ArmLineMoveDebugStage stage("Bezier前降低A拉开与B的高度");
                out = arm_dual_line_move_hold_redundant_selective(
                    arm_r, drop_r, drop_need_r, arm_l, drop_l, drop_need_l, hg.descend_vel_m_s);
                if ((drop_need_r && out.ret_r != 0) || (drop_need_l && out.ret_l != 0))
                {
                    if (drop_need_r && out.ret_r != 0)
                        log_arm_traj_plan_fail("右手(降低A)", out.ret_r);
                    if (drop_need_l && out.ret_l != 0)
                        log_arm_traj_plan_fail("左手(降低A)", out.ret_l);
                    return out;
                }
            }
            {
                ArmLineMoveDebugStage stage("二次Bezier A-B-C完整接近");
                out = arm_dual_quadratic_bezier_move_selective(
                    arm_r, st.goal_last_r, hs.move_r,
                    arm_l, st.goal_last_l, hs.move_l,
                    hg.bezier_guide_height_m,
                    hg.bezier_vel_m_s,
                    hg.bezier_orient_finish_ratio);
            }
        }
        else
        {
            ArmLineMoveDebugStage stage(
                g_move_cfg.head_grasp.use_hand_camera ? "物体上方下压"
                                                     : "B→C 下压到最终抓取点");
            out = arm_dual_line_move_selective(
                arm_r, st.goal_last_r, hs.move_r, arm_l, st.goal_last_l, hs.move_l,
                g_move_cfg.head_grasp.descend_vel_m_s);
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

        if (!wait_arms_motors_stopped(hs.move_r, hs.move_l, 5.0, 6, "最终C到位"))
        {
            std::cerr << "[grasp] 最终C电机到位等待超时，禁止合爪\n";
            if (hs.move_r)
                out.ret_r = -8;
            if (hs.move_l)
                out.ret_l = -8;
            return out;
        }

        // 位置偏差必须以最终 C 为基准报告；曲线控制点 B 不参与 offset 标定。
        auto verify_final_c = [&](Robot_Arm &arm,
                                  const Eigen::Matrix<double, 1, 6> &goal,
                                  const char *hand,
                                  int &ret_io) {
            const auto &hg = g_move_cfg.head_grasp;
            Eigen::Matrix<double, 1, 6> cur = arm_get_tcp_pos(arm);
            double xyz_err = 0.0;
            double rpy_err = 0.0;
            const double xyz_tol = std::max(0.001, hg.bezier_endpoint_xyz_tol_m);
            const double rpy_tol = std::max(0.5, hg.bezier_endpoint_rpy_tol_deg) * rad;
            bool ok = false;
            int stable_samples = 0;
            // 编码器停稳后复核 1 次即可；超差最多再看 0.3s。
            for (int sample = 0; sample < 6; ++sample)
            {
                cur = arm_get_tcp_pos(arm);
                xyz_err = (cur.head<3>() - goal.head<3>()).norm();
                rpy_err = 0.0;
                for (int axis = 3; axis < 6; ++axis)
                {
                    const double d = std::atan2(
                        std::sin(cur(axis) - goal(axis)),
                        std::cos(cur(axis) - goal(axis)));
                    rpy_err = std::max(rpy_err, std::abs(d));
                }
                const bool sample_ok = xyz_err <= xyz_tol && rpy_err <= rpy_tol;
                stable_samples = sample_ok ? stable_samples + 1 : 0;
                if (stable_samples >= 1)
                {
                    ok = true;
                    break;
                }
                if (hardware_abort_requested())
                    break;
                hardware_abort_sleep_ms(50);
            }
            std::cout << std::fixed << std::setprecision(4)
                      << "[bezier_C] " << hand
                      << " target=(" << goal(0) << ',' << goal(1) << ',' << goal(2) << ')'
                      << " actual=(" << cur(0) << ',' << cur(1) << ',' << cur(2) << ')'
                      << " xyz_err=" << xyz_err << "m rpy_err=" << (rpy_err / rad) << "deg\n";
            if (!ok)
            {
                std::cerr << "[bezier_C] " << hand << " 最终C超差，禁止"
                          << (g_tray_hole_task == TrayHoleTask::PlaceEmpty ? "张爪" : "合爪")
                          << "；请勿把轨迹误差混入视觉offset\n";
                ret_io = -7;
            }
        };
        if (hs.move_r)
            verify_final_c(arm_r, st.goal_last_r, "右", out.ret_r);
        if (hs.move_l)
            verify_final_c(arm_l, st.goal_last_l, "左", out.ret_l);
        if ((hs.move_r && out.ret_r != 0) || (hs.move_l && out.ret_l != 0))
            return out;

        const int settle_ms = static_cast<int>(
            std::max(0.0, g_move_cfg.head_grasp.pre_grasp_settle_sec) * 1000.0);
        if (settle_ms > 0)
            hardware_abort_sleep_ms(settle_ms);
        if (hardware_abort_requested())
        {
            out.ret_r = hs.move_r ? -4 : out.ret_r;
            out.ret_l = hs.move_l ? -4 : out.ret_l;
            return out;
        }

        if (hs.move_r)
        {
            if (g_tray_hole_task == TrayHoleTask::PlaceEmpty && tray2_precision_test())
                std::cout << "[gripper] 精度测试右手到位，不松爪\n";
            else if (g_tray_hole_task == TrayHoleTask::PlaceEmpty)
            {
                if (!g.openAndWait(gripper::Side::Right, 0.2, std::chrono::milliseconds(1500)))
                    std::cerr << "[gripper] 右手张爪失败\n";
            }
            else
            {
                const auto grasp_r = g.graspAndWait(gripper::Side::Right);
                if (grasp_r.result != gripper::GraspResult::TorqueLimit &&
                    grasp_r.result != gripper::GraspResult::PositionReached)
                {
                    std::cerr << "[gripper] 右手夹取异常 result=" << static_cast<int>(grasp_r.result)
                              << " q=" << grasp_r.position_rad << "\n";
                }
            }
        }
        if (hs.move_l)
        {
            if (g_tray_hole_task == TrayHoleTask::PlaceEmpty && tray2_precision_test())
                std::cout << "[gripper] 精度测试左手到位，不松爪\n";
            else if (g_tray_hole_task == TrayHoleTask::PlaceEmpty)
            {
                if (!g.openAndWait(gripper::Side::Left, 0.2, std::chrono::milliseconds(1500)))
                    std::cerr << "[gripper] 左手张爪失败\n";
            }
            else
            {
                const auto grasp_l = g.graspAndWait(gripper::Side::Left);
                if (grasp_l.result != gripper::GraspResult::TorqueLimit &&
                    grasp_l.result != gripper::GraspResult::PositionReached)
                {
                    std::cerr << "[gripper] 左手夹取异常 result=" << static_cast<int>(grasp_l.result)
                              << " q=" << grasp_l.position_rad << "\n";
                }
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

        if (!g_move_cfg.head_grasp.use_hand_camera &&
            g_move_cfg.head_grasp.use_bezier_grasp)
        {
            log_phase_banner("抓取完成：倒放Bezier关节轨迹，沿原路径返回A");
            const ArmLineMoveResult reverse = arm_dual_reverse_last_quadratic_bezier_selective(
                arm_r, lift_r, arm_l, lift_l);
            if ((lift_r && reverse.ret_r != 0) || (lift_l && reverse.ret_l != 0))
            {
                std::cerr << "[bezier] 倒放回程失败，保持当前位置，不启用旧直线回程。右="
                          << reverse.ret_r << " 左=" << reverse.ret_l << '\n';
                return;
            }
            if (lift_r)
                st.goal_last_r = make_standby_pos_right();
            if (lift_l)
                st.goal_last_l = make_standby_pos_left();
            return;
        }

        const double lift_z = g_move_cfg.head_grasp.lift_after_grasp_z;
        Eigen::Matrix<double, 1, 6> lift_pos_r = Eigen::Matrix<double, 1, 6>::Zero();
        Eigen::Matrix<double, 1, 6> lift_pos_l = Eigen::Matrix<double, 1, 6>::Zero();

        auto build_lift = [&](bool is_right, Eigen::Matrix<double, 1, 6> &goal,
                              Eigen::Matrix<double, 1, 6> &lift_pos) {
            const Eigen::Matrix<double, 1, 6> cur =
                arm_get_tcp_pos(is_right ? arm_r : arm_l);
            lift_pos = cur;
            lift_pos(2) = cur(2) + lift_z;
            if (lift_pos(2) > g_move_cfg.grasp_valid.z_max)
                lift_pos(2) = g_move_cfg.grasp_valid.z_max;
            goal = lift_pos;
        };
        if (lift_r)
            build_lift(true, st.goal_last_r, lift_pos_r);
        if (lift_l)
            build_lift(false, st.goal_last_l, lift_pos_l);

        log_phase_banner("抓取完成：夹后 TCP 抬升");
        {
            ArmLineMoveDebugStage stage("夹后抬升保持抓取姿态");
            arm_dual_line_move_selective(
                arm_r, lift_pos_r, lift_r, arm_l, lift_pos_l, lift_l,
                g_move_cfg.head_grasp.lift_vel_m_s);
        }
        if (g_move_cfg.head_grasp.use_hand_camera)
            cameras.save_grasp_hand_camera_snapshots(lift_r, lift_l);
    }

    /** 放货时腰 x 下限改走 yaml waist.x_min */


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
        if (!waist_layer3_is_synced())
        {
            std::cerr << "[waist] layer3 未与编码器同步，拒绝腰进（先执行 home）\n";
            return 0.0;
        }
        const double cur_x = posup_down_3_layer(0);
        const double delta = static_cast<double>(multiplier) * g_move_cfg.waist.stagger_step_x;
        std::ostringstream oss;
        oss << log_tag << " x×" << multiplier << " step=" << g_move_cfg.waist.stagger_step_x
            << " delta=" << delta;
        posup_down_3_layer(0) += delta;
        clamp_waist_layer3_xyz(posup_down_3_layer);
        const double actual = posup_down_3_layer(0) - cur_x;
        if (actual <= 1e-9)
        {
            std::cout << oss.str() << "，已到 x 行程上限，跳过\n";
            return 0.0;
        }
        std::cout << oss.str() << "，实际前进=" << actual << " m\n";
        const int rc = waist.moveLToPos(posup_down_3_layer, 0.1);
        if (rc != 0)
        {
            std::cerr << "[waist] stagger 运动失败 code=" << rc << "\n";
            sync_waist_layer3_from_encoders(waist, posup_down_3_layer, "腰进失败后");
            return 0.0;
        }
        hardware_abort_sleep_ms(
            static_cast<int>(g_move_cfg.waist.move_settle_sec * 1000.0));
        g_current_waist_layer3_z = posup_down_3_layer(2);
        return posup_down_3_layer(0) - cur_x;
    }

    double waist_layer3_ensure_far_row_forward(
        WaistRobot &waist,
        Eigen::Matrix<double, 1, 6> &layer3)
    {
        const double want = g_move_cfg.waist.far_row_forward_m;
        if (want <= 1e-9)
            return 0.0;
        if (!waist_layer3_is_synced())
        {
            std::cerr << "[waist] layer3 未与编码器同步，拒绝远三排腰进（先执行 home）\n";
            return 0.0;
        }
        if (!sync_waist_layer3_from_encoders(waist, layer3, "远三排腰进前"))
            return 0.0;
        const double x_fwd = waist_far_row_x();
        const double cur_x = layer3(0);
        layer3(0) = x_fwd;
        clamp_waist_layer3_xyz(layer3);
        const double actual = layer3(0) - cur_x;
        if (actual <= 0.01)
        {
            std::cout << std::fixed << std::setprecision(4)
                      << "[waist] 远三排已在前伸位 x=" << cur_x
                      << " 目标=" << x_fwd << " m，跳过\n";
            layer3(0) = cur_x;
            return 0.0;
        }
        std::cout << std::fixed << std::setprecision(4)
                  << "[waist] 远三排腰进 x " << cur_x << " → " << layer3(0)
                  << " (home " << g_move_cfg.waist.layer3_home(0) << " 前伸 " << want
                  << " m，锁 y/z/姿态)\n";
        const int rc = waist.moveLToPos(layer3, 0.1);
        if (rc != 0)
        {
            std::cerr << "[waist] 远三排腰进失败 code=" << rc << "\n";
            sync_waist_layer3_from_encoders(waist, layer3, "远三排腰进失败后");
            return 0.0;
        }
        hardware_abort_sleep_ms(
            static_cast<int>(g_move_cfg.waist.move_settle_sec * 1000.0));
        if (!sync_waist_layer3_from_encoders(waist, layer3, "远三排腰进后"))
            g_current_waist_layer3_z = layer3(2);
        const double moved = layer3(0) - cur_x;
        return moved > 1e-9 ? moved : 0.0;
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

    bool restore_waist_layer3_x(WaistRobot &waist, Eigen::Matrix<double, 1, 6> &posup_down_3_layer)
    {
        if (!sync_waist_layer3_from_encoders(waist, posup_down_3_layer, "恢复x前") &&
            !waist_layer3_is_synced())
        {
            std::cerr << "[waist] layer3 未与编码器同步，拒绝恢复 x\n";
            return false;
        }
        const double home_x = g_move_cfg.waist.layer3_home(0);
        const double cur_x = posup_down_3_layer(0);
        posup_down_3_layer(0) = home_x;
        clamp_waist_layer3_xyz(posup_down_3_layer);
        if (std::abs(cur_x - posup_down_3_layer(0)) <= 0.01)
        {
            posup_down_3_layer(0) = cur_x;
            return true;
        }
        std::cout << std::fixed << std::setprecision(4)
                  << "[waist] 恢复 layer3 x " << cur_x << " → " << posup_down_3_layer(0)
                  << "（锁 y/z/姿态）\n";
        const int rc = waist.moveLToPos(posup_down_3_layer, 0.1);
        if (rc != 0)
        {
            std::cerr << "[waist] 恢复 layer3 x 失败 code=" << rc << "\n";
            sync_waist_layer3_from_encoders(waist, posup_down_3_layer, "恢复x失败后");
            return false;
        }
        hardware_abort_sleep_ms(static_cast<int>(g_move_cfg.waist.move_settle_sec * 1000.0));
        if (!sync_waist_layer3_from_encoders(waist, posup_down_3_layer, "恢复x后"))
            g_current_waist_layer3_z = posup_down_3_layer(2);
        return true;
    }

    bool ensure_waist_layer3_x_home(WaistRobot &waist, Eigen::Matrix<double, 1, 6> &posup_down_3_layer)
    {
        if (!sync_waist_layer3_from_encoders(waist, posup_down_3_layer, "回腰前"))
            return false;
        const double home_x = g_move_cfg.waist.layer3_home(0);
        if (std::abs(posup_down_3_layer(0) - home_x) <= 0.01)
        {
            std::cout << std::fixed << std::setprecision(4)
                      << "[waist] 已在 home x=" << posup_down_3_layer(0) << " m，不回退\n";
            return true;
        }
        std::cout << std::fixed << std::setprecision(4)
                  << "[waist] 当前 x=" << posup_down_3_layer(0)
                  << " 不在 home " << home_x << "，先平移回去再升降\n";
        return restore_waist_layer3_x(waist, posup_down_3_layer);
    }

    bool ensure_waist_ready_start(WaistRobot &waist, Eigen::Matrix<double, 1, 6> &posup_down_3_layer)
    {
        const auto &w = g_move_cfg.waist;
        const double x_ready = waist_ready_x();
        const double z_ready = waist_ready_z();
        std::cout << std::fixed << std::setprecision(4)
                  << "[waist] ready/grasp 起始 x=" << x_ready
                  << "（home_x=" << w.layer3_home(0) << " 再向前 "
                  << w.ready_forward_m << " m）z=" << z_ready
                  << "（home z=" << w.layer3_home(2) << "）\n";
        if (!waist_layer3_move_x_only(waist, posup_down_3_layer, x_ready, false))
            return false;
        if (!waist_layer3_move_z_only(waist, posup_down_3_layer, z_ready, false))
            return false;
        if (!sync_waist_layer3_from_encoders(waist, posup_down_3_layer, "ready起始到位检查"))
            return false;
        const double dx = std::abs(posup_down_3_layer(0) - x_ready);
        const double dz = std::abs(posup_down_3_layer(2) - z_ready);
        if (dx > 0.01 || dz > 0.01)
        {
            std::cerr << std::fixed << std::setprecision(4)
                      << "[waist] ready 起始未到位 当前 x=" << posup_down_3_layer(0)
                      << " z=" << posup_down_3_layer(2)
                      << " 目标 x=" << x_ready << " z=" << z_ready
                      << " （|dx|=" << dx << " |dz|=" << dz << " > 0.01 m）\n";
            return false;
        }
        std::cout << std::fixed << std::setprecision(4)
                  << "[waist] ready 起始已到位 x=" << posup_down_3_layer(0)
                  << " z=" << posup_down_3_layer(2) << " m\n";
        return true;
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
        lift_grasped_selective(
            arm_r, arm_l, cameras, st,
            grasped_r && !lifted_r, grasped_l && !lifted_l);
    }

} // namespace move_box
