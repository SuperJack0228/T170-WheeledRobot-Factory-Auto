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
#include <array>
#include <cstring>
#include "t170/vision/head_transform.h"
#include "t170/hardware/device_bus.h"

using std::vector;
using std::thread;


namespace move_box {
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
        return arm_goal_in_real_waist_frame(p);
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
        return arm_goal_in_real_waist_frame(p);
    }

Eigen::Matrix<double, 1, 6> make_belt_grasp_goal(
        bool is_right, double grasp_x, double grasp_y, double grasp_z)
    {
        return make_belt_grasp_goal(is_right, grasp_x, grasp_y, grasp_z, g_move_cfg.conveyor);
    }
}
