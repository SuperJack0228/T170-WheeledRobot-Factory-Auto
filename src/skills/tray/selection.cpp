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
#include "t170/motion/retreat.h"
#include "t170/motion/servo_handoff.h"
#include <array>
#include <cstring>
#include "t170/vision/head_transform.h"
#include "t170/hardware/device_bus.h"

using std::vector;
using std::cout;
using std::thread;


namespace move_box {
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
}
