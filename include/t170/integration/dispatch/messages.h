#pragma once
namespace orch::dispatch_msg {
inline constexpr const char *TASK_LOAD = "TASK_LOAD";
inline constexpr const char *TASK_TRANSFER = "TASK_TRANSFER";
inline constexpr const char *TASK_UNLOAD = "TASK_UNLOAD";
inline constexpr const char *TASK_CHARGE = "CHARGE";
inline constexpr const char *APPLY_PLACE_RAW = "APPLY_PLACE_RAW";
inline constexpr const char *APPLY_PLACE_HALF = "APPLY_PLACE_HALF";
inline constexpr const char *APPLY_PICK_HALF = "APPLY_PICK_HALF";
inline constexpr const char *APPLY_PICK_WELL = "APPLY_PICK_WELL";
inline constexpr const char *PLACE = "PLACE";
inline constexpr const char *PICK_HALF = "PICK_HALF";
inline constexpr const char *PICK_WELL = "PICK_WELL";
inline constexpr const char *PLACE_DONE = "PLACE_DONE";
inline constexpr const char *PICK_UP_DONE = "PICK_UP_DONE";
inline constexpr const char *DONE = "DONE";
inline constexpr const char *STANDBY = "STANDBY";
inline constexpr const char *WORKING = "WORKING";
inline constexpr const char *GO_CHARGE = "GO_CHARGE";
inline constexpr const char *COMMUTING = "commuting";
inline constexpr const char *CHARGING = "CHARGING";
inline constexpr const char *ERR_MOTOR_LOST = "ERR_MOTOR_LOST";
inline constexpr const char *ERR_GRIPPER_LOST = "ERR_GRIPPER_LOST";
inline constexpr const char *ERR_CAMERA = "ERR_CAMERA";
inline constexpr const char *ERR_MOTOR_DISABLE = "ERR_MOTOR_DISABLE";
inline constexpr const char *ERR_AGV_OBSTACLE = "ERR_AGV_OBSTACLE";
inline constexpr const char *RAW_MATERIAL_EMPTY = "RAW_MATERIAL_EMPTY";
inline constexpr const char *WELL_DONE_MATERIAL_FULL = "WELL_DONE_MATERIAL_FULL";
inline constexpr const char *kDefaultHost = "192.168.122.120";
inline constexpr int kDefaultPort = 8080;
}
