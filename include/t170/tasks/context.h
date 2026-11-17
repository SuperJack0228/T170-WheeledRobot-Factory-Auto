#pragma once
#include <Eigen/Core>
#include "t170/vision/perception_types.h"
#include "t170/hardware/chassis.h"
#include "t170/integration/dispatch/client.h"
#include <array>
#include <atomic>
#include <limits>
#include <string>
#include <vector>
class Arm;
class WaistRobot;
class PosePipeline;
namespace gripper {class Gripper;}
namespace orch {
struct ArucoHit
{
    std::string slot;
    int id = -1;
    double side_m = 0.0;
    double reproj_px = 0.0;
    double cam_x = 0.0, cam_y = 0.0, cam_z = 0.0;
    double cam_yaw = 0.0, cam_pitch = 0.0, cam_roll = 0.0;
    bool robot_ok = false;
    double robot_x = 0.0, robot_y = 0.0, robot_z = 0.0;
    double robot_rx_deg = 0.0, robot_ry_deg = 0.0, robot_rz_deg = 0.0;
    std::array<double, 16> pose_robot_4x4{};
};
struct HwSession
{
    gripper::Gripper *g = nullptr;
    Arm *arm_r = nullptr;
    Arm *arm_l = nullptr;
    WaistRobot *waist = nullptr;
    Eigen::Matrix<double, 1, 6> *layer3 = nullptr;
    PosePipeline *pipeline = nullptr;
    std::atomic<bool> *abort = nullptr;
    bool last_grasped_r = false;
    bool last_grasped_l = false;
    int last_grasp_status = 0;
    int last_grasp_count = 0;
    bool last_placed_r=false,last_placed_l=false;
    int last_aruco_head = 0;
    int last_aruco_status = 0;
    std::vector<ArucoHit> last_aruco_hits;
    bool last_tray_ok = false;
    std::string last_tray_message;
    std::string last_tray_save_path;
    double last_tray_reproj_px = -1.0;
    double last_tray_tilt_deg = std::numeric_limits<double>::quiet_NaN();
    std::vector<int> last_tray_used_ids;
    std::vector<TrayHoleResult> last_tray_holes;
    std::string last_debug_photo;
    bool last_belt_ok = false;
    std::string last_belt_name;
    std::string last_belt_message;
    double last_belt_x = std::numeric_limits<double>::quiet_NaN();
    double last_belt_y = std::numeric_limits<double>::quiet_NaN();
    double last_belt_z = std::numeric_limits<double>::quiet_NaN();
    bool last_chassis_ok = false;
    std::string last_chassis_station;
    std::string last_chassis_message;
};
extern HwSession *g_hw;
}
namespace t170::tasks {
struct Services {
    hardware::Chassis &chassis;
    integration::dispatch::Gateway &dispatch;
};
// Non-owning binding, set once by the application before task threads start.
void bind_services(Services *);
Services &services();
}
