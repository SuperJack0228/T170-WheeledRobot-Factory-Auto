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
double wrap_pi(double a)
    {
        a = std::fmod(a + M_PI, 2.0 * M_PI);
        if (a < 0.0)
            a += 2.0 * M_PI;
        return a - M_PI;
    }

Eigen::Matrix3d rot_x(double a)
    {
        const double c = std::cos(a), s = std::sin(a);
        Eigen::Matrix3d R;
        R << 1, 0, 0, 0, c, -s, 0, s, c;
        return R;
    }

Eigen::Matrix3d rot_y(double a)
    {
        const double c = std::cos(a), s = std::sin(a);
        Eigen::Matrix3d R;
        R << c, 0, s, 0, 1, 0, -s, 0, c;
        return R;
    }

Eigen::Matrix3d rot_z(double a)
    {
        const double c = std::cos(a), s = std::sin(a);
        Eigen::Matrix3d R;
        R << c, -s, 0, s, c, 0, 0, 0, 1;
        return R;
    }

void remember_waist_attitude(const Eigen::MatrixXd &qj)
    {
        auto at = [&](int i) -> double {
            if (qj.rows() == 1 && qj.cols() > i)
                return qj(0, i);
            if (qj.cols() == 1 && qj.rows() > i)
                return qj(i, 0);
            return std::numeric_limits<double>::quiet_NaN();
        };
        const double ankle = at(0);
        const double knee = at(1);
        const double hip = at(2);
        const double roll = at(3);
        const double yaw = at(4);
        if (!std::isfinite(ankle) || !std::isfinite(knee) || !std::isfinite(hip) ||
            !std::isfinite(roll) || !std::isfinite(yaw))
            return;
        const double leg_hip = wrap_pi(-ankle - knee);
        g_waist_pitch_rad = wrap_pi(leg_hip - hip);
        g_waist_roll_rad = roll;
        g_waist_yaw_rad = yaw;
    }

Eigen::Matrix<double, 1, 6> arm_goal_in_real_waist_frame(const Eigen::Matrix<double, 1, 6> &upright)
    {
        const double pitch = g_waist_pitch_rad;
        const double roll = g_waist_roll_rad;
        const double yaw = g_waist_yaw_rad;
        if (std::abs(pitch) < 0.05 * rad && std::abs(roll) < 0.05 * rad && std::abs(yaw) < 0.05 * rad)
            return upright;
        const Eigen::Matrix3d R = rot_y(pitch) * rot_x(roll) * rot_z(yaw);
        const double cx = kWaistPitchLinkM * std::sin(pitch);
        const double cz = kWaistPitchLinkM * (std::cos(pitch) - 1.0);
        const Eigen::Vector3d xyz =
            R.transpose() * (Eigen::Vector3d(upright(0), upright(1), upright(2)) -
                             Eigen::Vector3d(cx, 0.0, cz));
        Eigen::Matrix4d T = TR(upright);
        const Eigen::Matrix3d Rout = R.transpose() * T.block<3, 3>(0, 0);
        const Eigen::Matrix<double, 1, 3> eul = rotationMatrixToEulerAngles(Rout);
        Eigen::Matrix<double, 1, 6> out;
        out << xyz.transpose(), eul;
        std::cout << std::fixed << std::setprecision(2)
                  << "[waist_frame] 姿态误差 pitch=" << (pitch / rad)
                  << "° roll=" << (roll / rad) << "° yaw=" << (yaw / rad)
                  << "°，基座修正 Δxyz_mm=(" << std::setprecision(1)
                  << ((xyz(0) - upright(0)) * 1000.0) << ","
                  << ((xyz(1) - upright(1)) * 1000.0) << ","
                  << ((xyz(2) - upright(2)) * 1000.0) << ")\n";
        return out;
    }
}
