#pragma once
#include "t170/app/application.h"
#include "t170/app/options.h"
#include "t170/app/commands.h"
#include "t170/tasks/context.h"
#include "t170/tasks/operations.h"
#include "t170/tasks/factory.h"
#include "t170/app/maintenance.h"
#include "t170/common/announcements.h"
#include "t170/telemetry/factory_status.h"
#include "t170/calibration/placement_session.h"
#include "t170/hardware/gripper.hpp"
#include "t170/hardware/waist.h"
#include "t170/vision/python_bridge.h"
#include "t170/motion/arm.h"
#include "t170/common/units.h"
#include "t170/motion/cartesian_tracking.h"
#include "t170/motion/retreat.h"
#include "t170/motion/servo_handoff.h"
#include "t170/skills/legacy/runtime.h"
#include "t170/skills/legacy/profiles.h"
#include <json/json.h>
#include <optional>
#include <memory>
#include <mutex>
#include <atomic>
namespace t170::app::detail {
struct State {
std::atomic<bool> abort{false};
std::mutex hw_mu;
t170::calibration::PlacementSession placement_calibration;
orch::HwSession session;
std::unique_ptr<gripper::Gripper> grip;
std::unique_ptr<Arm> arm_r;
std::unique_ptr<Arm> arm_l;
std::unique_ptr<WaistRobot> waist;
std::unique_ptr<PosePipeline> pipeline;
Eigen::Matrix<double, 1, 6> layer3;
    Mode mode=Mode::Maintenance;
    std::unique_ptr<t170::hardware::Chassis> chassis;
    std::unique_ptr<t170::integration::dispatch::Gateway> gateway;
    std::unique_ptr<t170::tasks::Services> services;
};
State &state();
extern std::atomic<bool> &g_abort;
extern std::mutex &g_hw_mu;
extern std::mutex &g_config_mu;
extern t170::calibration::PlacementSession &g_placement_calibration;
extern orch::HwSession &g_session;
extern std::unique_ptr<gripper::Gripper> &g_grip;
extern std::unique_ptr<Arm> &g_arm_r;
extern std::unique_ptr<Arm> &g_arm_l;
extern std::unique_ptr<WaistRobot> &g_waist;
extern std::unique_ptr<PosePipeline> &g_pipeline;
extern Eigen::Matrix<double, 1, 6> &g_layer3;
Json::Value reply(bool ok,const std::string &error={});
Json::Value dispatch(const Json::Value &request);
Json::Value dispatch_status_json();
Json::Value with_arm_arrivals(Json::Value);
Json::Value pose_json(const Eigen::Matrix<double,1,6>&);
Json::Value markers_json();
Json::Value tray_holes_json();
Json::Value xyz_json(double,double,double);
Json::Value json_finite_or_null(double);
bool init_hardware(std::string &);
[[noreturn]] void shutdown_process(int);
void handle_client(int);
void start_boot_dispatch();
Json::Value capabilities_json();
std::optional<Json::Value> handle_runtime_commands(const Json::Value &request);
std::optional<Json::Value> handle_status_commands(const Json::Value &request);
std::optional<Json::Value> handle_maintenance_commands(const Json::Value &request);
std::optional<Json::Value> handle_manual_tasks_commands(const Json::Value &request);
std::optional<Json::Value> handle_inspection_commands(const Json::Value &request);
std::optional<Json::Value> handle_operation_commands(const Json::Value &request);
}
