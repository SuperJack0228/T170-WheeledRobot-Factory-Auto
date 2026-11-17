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
#include "t170/motion/cartesian_tracking.h"
#include "t170/motion/retreat.h"
#include "t170/motion/servo_handoff.h"
#include <array>
#include <cstring>
#include "t170/vision/head_transform.h"
#include "t170/hardware/device_bus.h"

using std::vector;
using std::thread;


namespace move_box {
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
            remember_waist_attitude(qj);
            std::cout << std::fixed << std::setprecision(2)
                      << "[waist] " << tag << " 姿态 pitch=" << (g_waist_pitch_rad / rad)
                      << "° roll=" << (g_waist_roll_rad / rad)
                      << "° yaw=" << (g_waist_yaw_rad / rad) << "°\n";
        }
        return true;
    }

void read_arrived_pose(WaistRobot &waist, Eigen::Matrix<double, 1, 6> &layer3)
    {
        if (!sync_waist_layer3_from_encoders(waist, layer3, "到位读角度"))
            return;
        std::cout << std::fixed << std::setprecision(4)
                  << "[waist] 实时位姿用于基座 xyz=(" << layer3(0) << ","
                  << layer3(1) << "," << layer3(2) << ") m"
                  << std::setprecision(2)
                  << " pitch=" << (g_waist_pitch_rad / rad)
                  << "° roll=" << (g_waist_roll_rad / rad)
                  << "° yaw=" << (g_waist_yaw_rad / rad) << "°\n";
    }

bool waist_layer3_move_z_only(
        WaistRobot &waist,
        Eigen::Matrix<double, 1, 6> &layer3,
        double z_target,
        bool settle,
        bool read_pose_when_done)
    {
        enable_waist_motors();
        if (!sync_waist_layer3_from_encoders(waist, layer3, "竖直升降前"))
            return false;
        const double z0 = layer3(2);
        layer3(2) = z_target;
        clamp_waist_layer3_xyz(layer3);
        if (std::abs(layer3(2) - z0) <= 1e-3)
        {
            std::cout << std::fixed << std::setprecision(4)
                      << "[waist] 竖直 Z 已在 " << z0 << " m，跳过\n";
            layer3(2) = z0;
            g_current_waist_layer3_z = layer3(2);
            if (read_pose_when_done)
                read_arrived_pose(waist, layer3);
            return true;
        }
        std::cout << std::fixed << std::setprecision(4)
                  << "[waist] 竖直 Z " << z0 << " → " << layer3(2)
                  << " 锁 x=" << layer3(0) << " y=" << layer3(1) << " m\n";
        const double z_cmd = layer3(2);
        const int rc = waist.moveLToPosZ(z_cmd, 0.1);
        if (rc != 0)
        {
            std::cerr << "[waist] 竖直升降失败 code=" << rc << "\n";
            sync_waist_layer3_from_encoders(waist, layer3, "竖直升降失败后");
            return false;
        }
        if (settle)
            hardware_abort_sleep_ms(static_cast<int>(g_move_cfg.waist.move_settle_sec * 1000.0));
        if (read_pose_when_done)
            read_arrived_pose(waist, layer3);
        else if (!sync_waist_layer3_from_encoders(waist, layer3, "竖直升降后"))
            g_current_waist_layer3_z = z_cmd;
        return true;
    }

bool waist_layer3_move_x_only(
        WaistRobot &waist,
        Eigen::Matrix<double, 1, 6> &layer3,
        double x_target,
        bool settle,
        bool read_pose_when_done)
    {
        enable_waist_motors();
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
            if (read_pose_when_done)
                read_arrived_pose(waist, layer3);
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
        if (read_pose_when_done)
            read_arrived_pose(waist, layer3);
        else if (!sync_waist_layer3_from_encoders(waist, layer3, "腰平移后"))
            g_current_waist_layer3_z = layer3(2);
        return true;
    }


}
