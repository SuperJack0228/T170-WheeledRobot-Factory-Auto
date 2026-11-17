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
#include "t170/motion/planning.h"
#include "t170/common/pose_math.h"
#include "t170/motion/retreat.h"
#include "t170/motion/servo_handoff.h"
#include <array>
#include <cstring>
#include "t170/vision/head_transform.h"
#include "t170/hardware/device_bus.h"

using Eigen::Matrix;
using std::vector;
using std::thread;


namespace move_box {
void log_retract_arm_state(const char *tag, bool is_right, Arm &arm, bool enforce_zone = true);
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

bool try_standby_ik(Arm &arm, Eigen::Matrix<double, 1, 6> goal,
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

bool solve_home_joints(Arm &arm, bool is_right, Eigen::Matrix<double, 1, 6> goal,
                           Eigen::Matrix<double, 1, 7> &qd)
    {
        const Eigen::RowVectorXd q_now = arm_get_joint_pos(arm);
        if (q_now.size() < 7)
        {
            std::cerr << "[home] " << (is_right ? "右" : "左") << " 逆解维数不足\n";
            return false;
        }

        // 垂臂当前关节离 standby 太远，不能当唯一种子。换零位/待机构型种子再解。
        Eigen::RowVectorXd q_zero = arm.to_model(Eigen::Matrix<double,1,7>::Zero());
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

        q_standby_seed = arm.to_model(q_standby_seed);
        const Eigen::RowVectorXd seeds[3] = {q_now, q_zero, q_standby_seed};
        const char *seed_names[3] = {"当前关节", "零位", "待机构型"};
        const double j2_try[8] = {
            q_now(1),
            q_zero(1)-45.0 * rad, q_zero(1)-30.0 * rad, q_zero(1)-20.0 * rad, q_zero(1),
            q_zero(1)+20.0 * rad, q_zero(1)+30.0 * rad, q_zero(1)+45.0 * rad};

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

bool arm_pose_needs_joint_home(Arm &arm)
    {
        const Eigen::Matrix<double, 1, 6> tcp = arm_get_tcp_pos(arm);
        const Eigen::Matrix<double, 1, 7> q = arm_get_joint_pos(arm);
        const bool near_body = tcp(0) < kFoldedNearBodyXMaxM;
        const bool very_low = tcp(2) < kFoldedLowZMaxM;
        // Preserve the old posture heuristic's device-zero reference explicitly.
        const bool elbow_folded = std::abs(arm.to_device(q)(1)) > kFoldedElbowAbsRad;
        return (near_body && very_low) || (very_low && elbow_folded);
    }

bool retract_joint_to_standby(
        Arm &arm_r,
        Arm &arm_l,
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

void log_retract_arm_state(const char *tag, bool is_right, Arm &arm, bool enforce_zone)
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
        Arm &arm,
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
        Arm &arm_r,
        Arm &arm_l,
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
        Arm &arm_r,
        Arm &arm_l,
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
        Arm &arm_r,
        Arm &arm_l,
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

bool move_arms_to_home(Arm &arm_r, Arm &arm_l)
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
        Arm &arm_r, Arm &arm_l, const ConveyorStationConfig &c)
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

bool move_arms_to_belt_ready(Arm &arm_r, Arm &arm_l)
    {
        return move_arms_to_belt_ready(arm_r, arm_l, g_move_cfg.conveyor);
    }

bool move_arms_to_belt_grasp_rpy(
        Arm &arm_r, Arm &arm_l, const ConveyorStationConfig &c)
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
        Arm &arm_r, Arm &arm_l, const ConveyorStationConfig &c)
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

bool move_arms_to_belt_place(Arm &arm_r, Arm &arm_l)
    {
        return move_arms_to_belt_place(arm_r, arm_l, g_move_cfg.conveyor);
    }








}
