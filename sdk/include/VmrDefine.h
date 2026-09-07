#ifndef VMR_DEFINE_H
#define VMR_DEFINE_H
#ifndef __cplusplus
#error "This header file is for C++ only. For C projects, please use VmrSDK_C.h."
#endif
#include <cstdint>
#include <string>
#include <vector>
typedef struct {
  uint64_t timestamp_ns; // Timestamp (nanoseconds)
  float x;               // Odometry coordinate x (meters)
  float y;               // Odometry coordinate y (meters)
  float theta;           // Odometry angle (radians)
  float vx;              // Linear velocity in x direction (m/s)
  float vy;              // Linear velocity in y direction (m/s)
  float w;               // Angular velocity (rad/s)
} VmrOdomInfo;
// Position Information - Cartesian Coordinate System
typedef struct {
  float x = 0;    // meters
  float y = 0;    // meters
  float z = 0.0f; // meters, default to 2D
} VmrPoint;
typedef struct {
  uint64_t timestamp_ns;
  std::string frame_id;
  // Scanning Parameters
  float angle_min;       // Start radian (-π~π)
  float angle_max;       // End radian (-π~π)
  float angle_increment; // Angular resolution (radians/sample)
  float time_increment;  // Time interval (seconds/sample)
  float scan_time;       // Total scanning time (seconds)
  float range_min;       // Minimum effective distance (meters)
  float range_max;       // Maximum effective distance (meters)
  // Scanning Data
  std::vector<float> ranges;      // Distance data (meters)
  std::vector<float> intensities; // Intensity data (optional)
} VmrScan;
typedef struct {
  uint64_t timestamp_ns;            // Timestamp (nanoseconds)
  std::string frame_id;             // Coordinate system, e.g., "laser_1"
  std::vector<VmrPoint> laser_scan; // Laser point cloud position based on the
                                    // robot's coordinate system
  std::vector<float> intensities;   // Intensity
} VmrLaserScan;
// Coordinate System (Laser)
//  frame_id      Meaning
//  laser_1       Front laser
//  laser_2       Rear laser
//  laser_-1      Front 3D laser
//  laser_-2      Rear 3D laser
// Localization Information
typedef struct {
  uint64_t timestamp_ns; // Timestamp (nanoseconds)
  float x;               // Localization coordinate x (meters)
  float y;               // Localization coordinate y (meters)
  float theta;           // Localization angle (radians)
  float confidence;      // Confidence level 0~2.5
  int status;            // Localization status
} VmrLocationInfo;
// Localization Status
// status       Meaning
// 0            Normal
// 1            Relocalization failed
// 2            Slippage
// 6            Relocalizing
// Relocalization Information
typedef struct {
  uint64_t timestamp_ns; // Timestamp (nanoseconds)
  float x;               // Relocalization coordinate x (meters)
  float y;               // Relocalization coordinate y (meters)
  float theta;           // Relocalization angle (radians)
} VmrRelocateInfo;
typedef enum {
  kROS = 0,    // ROS IMU
  kHipnuc = 1, // Built-in IMU
  kMid360 = 2, // mid360 IMU
  kT2 = 3,     // T2 IMU
  kRsAIRY = 4, // RsAIRY IMU
  kBMI088 = 5, // BMI088 IMU
  kS10U = 6    // S10U IMU
} VmrImuType;
typedef struct {
  double x, y, z, w; // Attitude quaternion
} VmrQuaternion;
typedef struct {
  double x, y, z;
} VmrVector3;
typedef struct {
  uint64_t timestamp_ns;          // Timestamp (nanoseconds)
  VmrImuType type;                // IMU type
  VmrQuaternion orientation;      // Attitude quaternion
  VmrVector3 angular_velocity;    // Angular velocity
  VmrVector3 linear_acceleration; // Linear acceleration
} VmrImuInfo;
typedef struct {
  uint64_t timestamp_ns;
  float voltage;         // Voltage (Volts)
  float current;         // Current (Amperes)
  float charge;          // Charging/discharging status
  float capacity;        // Battery capacity
  float design_capacity; // Rated battery capacity
  float percentage;      // Power percentage
  int8_t power_supply_status{0}; // Battery status: 0-Unknown, 1-Charging, 2-Discharging
  std::string serial_number; // Serial number
} VmrBatteryInfo;
typedef struct {
  int32_t module{0}; // Exception module
  int32_t code{0};   // Exception code
} VmrException;
typedef struct {
  uint64_t timestamp_ns;                // Timestamp (nanoseconds)
  std::vector<VmrException> exceptions; // Exception list
} VmrRobotException;
typedef struct {
  uint64_t timestamp_ns; // Timestamp (nanoseconds)
  int32_t height;        // Lift height (millimeters)
} VmrLiftInfo;
typedef struct {
  VmrVector3 linear;  // Linear velocity
  VmrVector3 angular; // Angular velocity
} VmrTwistInfo;
typedef struct {
  int task_flag;       // Task flag
  int task_result;     // Task result code (0: Success, -1: In progress, >1: Execution exception)
  std::string task_id; // Task ID
} VmrTaskResult;
typedef struct {
  double x = 0;     // Current map coordinate system x (meters)
  double y = 0;     // Current map coordinate system y (meters)
  double theta = 0; // Current map coordinate system angle (radians)
} VmrPose;


// Movement mode
typedef enum {
    MOVE_MODE_HEADLESS = -1,   // Headless mode (no specific orientation)
    MOVE_MODE_FORWARD = 0,     // Forward (default)
    MOVE_MODE_BACKWARD = 1,     // Backward
    MOVE_MODE_OMNI = 2     // Omnidirectional
} VmrMoveMode;

// Navigation mode
typedef enum {
    NAVI_MODE_POINT_TO_POINT = 0,   // Point-to-point navigation (with obstacle avoidance)
    NAVI_MODE_STRAIGHT_LINE = 1     // Straight line to point (no obstacle avoidance, stop on obstacle only)
} VmrNaviMode;

typedef struct {
  VmrPose target_pose; // Target pose for navigation
  VmrMoveMode move_mode{MOVE_MODE_HEADLESS};
  bool forbid_rotation_on_start{false};
  VmrNaviMode navi_mode{NAVI_MODE_POINT_TO_POINT};
} VmrNavTask;


typedef void (*LaserCallback)(const VmrLaserScan &scan);
typedef void (*LocationCallback)(const VmrLocationInfo &loc_data);
typedef void (*OdomCallback)(const VmrOdomInfo &odom_data);
typedef void (*ImuCallback)(const VmrImuInfo &imu_data);
typedef void (*BatteryCallback)(const VmrBatteryInfo &battery_data);
typedef void (*ExceptionCallback)(const VmrRobotException &exception_data);
typedef void (*LiftCallback)(const VmrLiftInfo &left_data);
typedef int VMR_Handle;
#endif