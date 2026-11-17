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
#include "t170/motion/retreat.h"
#include "t170/motion/servo_handoff.h"
#include <array>
#include <cstring>
#include "t170/vision/head_transform.h"
#include "t170/hardware/device_bus.h"

using std::vector;
using std::thread;


namespace move_box {
void lift_grasped_selective(
        Arm &arm_r,
        Arm &arm_l,
        RealSenseMultiCam &cameras,
        HeadAssignState &st,
        bool lift_r,
        bool lift_l)
    {
        if (!lift_r && !lift_l)
            return;

        const double lift_z = g_move_cfg.head_grasp.lift_after_grasp_z;
        const double lift_mps = g_move_cfg.head_grasp.lift_vel_m_s;
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
                lift_mps);
        }
        if (g_move_cfg.head_grasp.use_hand_camera)
            cameras.save_grasp_hand_camera_snapshots(lift_r, lift_l);
    }

void lift_grasped_after_pick(
        Arm &arm_r,
        Arm &arm_l,
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
}
