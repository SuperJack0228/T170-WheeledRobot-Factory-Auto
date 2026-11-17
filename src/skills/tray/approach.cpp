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
#include "Ti5_Arm.h"
#include "t170/common/units.h"
#include "t170/motion/feedback.h"
#include "t170/motion/kinematics_adapter.h"
#include "t170/motion/planning.h"
#include <array>
#include <cstring>
#include "t170/vision/head_transform.h"
#include "t170/hardware/device_bus.h"

using std::vector;
using std::cout;
using std::thread;


namespace move_box {
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

ArmLineMoveResult run_head_approach_selective(
        Robot_Arm &arm_r,
        Robot_Arm &arm_l,
        HeadAssignState &st,
        bool move_r,
        bool move_l)
    {
        t170::skills::detail::report(t170::skills::Stage::Approach,move_r,move_l);
        if(t170::skills::detail::active_execution && t170::skills::detail::active_execution->cancelled()) return {move_r?-4:0,move_l?-4:0};
        ArmLineMoveResult out;
        drop_if_y_split_blocks(st, move_r, move_l, "hover下发前");

        // 不用手相机停稳拍照：本段不移动。上方和竖直下降在下一段一次走完。
        if (!g_move_cfg.head_grasp.use_hand_camera)
        {
            log_phase_banner(g_tray_hole_task == TrayHoleTask::PlaceEmpty
                                 ? "料盘2放置：先到孔上方再下降，本阶段不单独停车"
                                 : "抓取流程：先到孔上方再下降，本阶段不单独停车");
            std::cout << "[descent] 上方和下降接成一次直线，这里不移动\n";
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
            Eigen::Matrix<double, 1, 6> goal_r = arm_goal_in_real_waist_frame(st.goal_last_r);
            Eigen::Matrix<double, 1, 6> goal_l = arm_goal_in_real_waist_frame(st.goal_last_l);
            Eigen::Matrix<double, 1, 6> orient_r = goal_r;
            Eigen::Matrix<double, 1, 6> orient_l = goal_l;
            if (move_r)
            {
                const Eigen::Matrix<double, 1, 6> cur = arm_get_tcp_pos(arm_r);
                orient_r = cur;
                orient_r(3) = goal_r(3);
                orient_r(4) = goal_r(4);
                orient_r(5) = goal_r(5);
                do_orient_r = need_orient(cur, goal_r);
            }
            if (move_l)
            {
                const Eigen::Matrix<double, 1, 6> cur = arm_get_tcp_pos(arm_l);
                orient_l = cur;
                orient_l(3) = goal_l(3);
                orient_l(4) = goal_l(4);
                orient_l(5) = goal_l(5);
                do_orient_l = need_orient(cur, goal_l);
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
                    arm_r, goal_r, move_r, arm_l, goal_l, move_l,
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
                verify_one(arm_r, goal_r, "右", out.ret_r);
            if (move_l)
                verify_one(arm_l, goal_l, "左", out.ret_l);
            return out;
        }

        // 固定 yaml 姿态：一条 Line_Trajectory 到上方，同时转到 0/45/−10
        Eigen::Matrix<double, 1, 6> goal_r = arm_goal_in_real_waist_frame(st.goal_last_r);
        Eigen::Matrix<double, 1, 6> goal_l = arm_goal_in_real_waist_frame(st.goal_last_l);
        {
            ArmLineMoveDebugStage stage("到物体上方同时转抓取姿态");
            out = arm_dual_line_move_selective(
                arm_r, goal_r, move_r, arm_l, goal_l, move_l,
                g_move_cfg.head_grasp.approach_vel_m_s);
        }
        if (move_r && out.ret_r != 0)
            log_arm_traj_plan_fail("右手", out.ret_r);
        if (move_l && out.ret_l != 0)
            log_arm_traj_plan_fail("左手", out.ret_l);
        return out;
    }

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
}
