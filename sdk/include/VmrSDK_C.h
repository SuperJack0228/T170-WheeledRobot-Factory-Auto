/**
 * @file VmrSDK_C.h
 * @brief VMR Robot SDK Pure C Interface Definition
 * @details This file encapsulates the C++ SDK functionality into a C interface, 
 * allowing calls from C or other languages supporting C ABI (e.g., Python, C#, Rust).
 * @version 1.0.0
 * @date 2024-01-01
 */

#ifndef VMR_SDK_C_H
#define VMR_SDK_C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/** * @brief SDK Handle Type 
 * @note Used to distinguish between different robot connection instances.
 */
typedef int VMR_Handle;

/** @brief Maximum length of string data (including null terminator \0) */
#define VMR_MAX_ID_LEN 64
/** @brief Maximum length of message text */
#define VMR_MAX_MSG_LEN 256

// =============================================================================
// Data Structure Definitions (C Compatible)
// =============================================================================

/**
 * @brief Robot Odometry Information
 */
typedef struct {
    uint64_t timestamp_ns; /**< Timestamp (nanoseconds) */
    float x;               /**< Position X in odometry frame (meters) */
    float y;               /**< Position Y in odometry frame (meters) */
    float theta;           /**< Yaw angle (radians) */
    float vx;              /**< Linear velocity X (m/s) */
    float vy;              /**< Linear velocity Y (m/s) */
    float w;               /**< Angular velocity (rad/s) */
} VmrOdomInfo_C;

/**
 * @brief 3D Point
 */
typedef struct {
    float x; /**< X coordinate (meters) */
    float y; /**< Y coordinate (meters) */
    float z; /**< Z coordinate (meters), usually 0 for 2D navigation */
} VmrPoint_C;

/**
 * @brief Raw Laser Scan Data (Input to SDK)
 * @note Corresponds to VmrScan in C++
 */
typedef struct {
    uint64_t timestamp_ns;          /**< Timestamp (nanoseconds) */
    char frame_id[VMR_MAX_ID_LEN];  /**< Frame ID (e.g., "laser") */
    
    // Scan Parameters
    float angle_min;       /**< Start angle (radians) */
    float angle_max;       /**< End angle (radians) */
    float angle_increment; /**< Angular resolution (radians/point) */
    float time_increment;  /**< Time interval between points (seconds) */
    float scan_time;       /**< Total scan time (seconds) */
    float range_min;       /**< Minimum valid range (meters) */
    float range_max;       /**< Maximum valid range (meters) */
    
    // Scan Data (Array Pointers)
    float* ranges;         /**< Pointer to range data array (meters) */
    uint32_t ranges_count; /**< Number of range data points */
    
    float* intensities;         /**< Pointer to intensity data array */
    uint32_t intensities_count; /**< Number of intensity data points (0 if not available) */
} VmrScan_C;

/**
 * @brief Processed Laser Point Cloud Data (Output from SDK Callback)
 * @note Corresponds to VmrLaserScan in C++
 */
typedef struct {
    uint64_t timestamp_ns;          /**< Timestamp (nanoseconds) */
    char frame_id[VMR_MAX_ID_LEN];  /**< Frame ID */
    
    VmrPoint_C* laser_scan; /**< Pointer to point cloud array (relative to robot frame) */
    uint32_t point_count;   /**< Number of points */
    
    float* intensities;         /**< Pointer to intensity data array */
    uint32_t intensities_count; /**< Number of intensity data points */
} VmrLaserScan_C;

/**
 * @brief Localization Information
 * @details Contains the robot's pose in the map and localization status.
 */
typedef struct {
    uint64_t timestamp_ns; /**< Timestamp (nanoseconds) */
    float x;               /**< Map coordinate X (meters) */
    float y;               /**< Map coordinate Y (meters) */
    float theta;           /**< Map angle (radians) */
    float confidence;      /**< Localization confidence (0.0 ~ 2.5) */
    
    /**
     * @brief Localization Status Code
     * - 0: Normal
     * - 1: Relocalization failed
     * - 2: Slippage
     * - 6: Relocalizing
     */
    int status;
} VmrLocationInfo_C;

/**
 * @brief Relocalization Request Info
 */
typedef struct {
    uint64_t timestamp_ns; /**< Timestamp (nanoseconds) */
    float x;               /**< Initial guess X (meters) */
    float y;               /**< Initial guess Y (meters) */
    float theta;           /**< Initial guess angle (radians) */
} VmrRelocateInfo_C;

/**
 * @brief IMU Type Enumeration
 */
typedef enum {
    kROS_C = 0,    /**< ROS Standard IMU */
    kHipnuc_C = 1, /**< Internal IMU */
    kMid360_C = 2, /**< Mid360 LiDAR Internal IMU */
    kT2_C = 3,     /**< T2 IMU */
    kRsAIRY_C = 4, /**< RsAIRY IMU */
    kBMI088_C = 5, /**< BMI088 IMU */
    kS10U_C = 6    /**< S10U IMU */
} VmrImuType_C;

/** @brief Quaternion (Representation of Orientation) */
typedef struct {
    double x, y, z, w;
} VmrQuaternion_C;

/** @brief 3D Vector (Velocity/Acceleration) */
typedef struct {
    double x, y, z;
} VmrVector3_C;

/**
 * @brief IMU Sensor Data
 */
typedef struct {
    uint64_t timestamp_ns;          /**< Timestamp (nanoseconds) */
    VmrImuType_C type;              /**< IMU Device Type */
    VmrQuaternion_C orientation;    /**< Orientation Quaternion */
    VmrVector3_C angular_velocity;  /**< Angular Velocity (rad/s) */
    VmrVector3_C linear_acceleration; /**< Linear Acceleration (m/s^2) */
} VmrImuInfo_C;

/**
 * @brief Battery Information
 */
typedef struct {
    uint64_t timestamp_ns; /**< Timestamp (nanoseconds) */
    float voltage;         /**< Voltage (V) */
    float current;         /**< Current (A) */
    float charge;          /**< Charge/Discharge state mask or value */
    float capacity;        /**< Current Capacity (Ah) */
    float design_capacity; /**< Design Capacity (Ah) */
    float percentage;      /**< Battery Percentage (0.0 ~ 100.0) */
    
    /** * @brief Power Supply Status 
     * 0: Unknown, 1: Charging, 2: Discharging 
     */
    int8_t power_supply_status; 
    
    char serial_number[VMR_MAX_ID_LEN]; /**< Battery Serial Number */
} VmrBatteryInfo_C;

/** @brief Single Exception Info */
typedef struct {
    int32_t module; /**< Exception Module ID */
    int32_t code;   /**< Exception Error Code */
} VmrException_C;

/**
 * @brief Robot Exception List
 */
typedef struct {
    uint64_t timestamp_ns;      /**< Timestamp (nanoseconds) */
    VmrException_C* exceptions; /**< Pointer to exceptions array */
    uint32_t count;             /**< Number of exceptions */
} VmrRobotException_C;

/**
 * @brief Lift Mechanism Status
 */
typedef struct {
    uint64_t timestamp_ns; /**< Timestamp (nanoseconds) */
    int32_t height;        /**< Current Height (mm) */
} VmrLiftInfo_C;

/**
 * @brief Velocity Control Command
 */
typedef struct {
    VmrVector3_C linear;  /**< Linear Velocity (m/s) */
    VmrVector3_C angular; /**< Angular Velocity (rad/s) */
} VmrTwistInfo_C;

/**
 * @brief Task Query Result
 */
typedef struct {
    int task_flag;   /**< Task Flag */
    int task_result; /**< Task Result Code (0: Success, Other: Error Code) */
    char task_id[VMR_MAX_ID_LEN]; /**< Task ID */
} VmrTaskResult_C;

/**
 * @brief Target Pose (For Navigation Tasks)
 */
typedef struct {
    double x;     /**< Target X (meters) */
    double y;     /**< Target Y (meters) */
    double theta; /**< Target Angle (radians) */
} VmrPose_C;

// =============================================================================
// Callback Function Type Definitions (C Function Pointers)
// =============================================================================

/** @brief Laser Data Callback Pointer */
typedef void (*LaserCallback_C)(const VmrLaserScan_C* scan);
/** @brief Location Status Callback Pointer */
typedef void (*LocationCallback_C)(const VmrLocationInfo_C* loc_data);
/** @brief Odometry Callback Pointer */
typedef void (*OdomCallback_C)(const VmrOdomInfo_C* odom_data);
/** @brief IMU Data Callback Pointer */
typedef void (*ImuCallback_C)(const VmrImuInfo_C* imu_data);
/** @brief Battery Status Callback Pointer */
typedef void (*BatteryCallback_C)(const VmrBatteryInfo_C* battery_data);
/** @brief Exception Report Callback Pointer */
typedef void (*ExceptionCallback_C)(const VmrRobotException_C* exception_data);
/** @brief Lift Status Callback Pointer */
typedef void (*LiftCallback_C)(const VmrLiftInfo_C* left_data);

// =============================================================================
// SDK API Function Declarations
// =============================================================================

/**
 * @brief Initialize Robot SDK
 * @param cfg_file Path to configuration file. Pass empty string "" to use default (127.0.0.1:9000).
 * @return int Initialization result (1: Success, 0 or other: Failure)
 */
int VMR_init(const char *cfg_file);

/**
 * @brief Initialize Robot SDK with server IP/port (no log file required)
 * @param ip Server IP address
 * @param port Server port
 * @return int Initialization result (1: Success, 0 or other: Failure)
 */
int VMR_initWithServer(const char *ip, int port);

/**
 * @brief Create SDK Handle
 * @return VMR_Handle The newly created handle ID
 */
VMR_Handle VMR_Handle_Create();

/**
 * @brief Destroy SDK Handle and release resources
 * @param h Handle to destroy
 */
void VMR_Handle_Destroy(VMR_Handle h);

// ---------------- Data Input Interfaces ----------------

/**
 * @brief Input Robot Odometry Data (For Fusion Localization)
 * @param handle SDK Handle
 * @param odom Pointer to odometry data
 */
void VMR_setRobotOdometry(VMR_Handle handle, const VmrOdomInfo_C* odom);

/**
 * @brief Input Robot Current Velocity (Feedback)
 * @param handle SDK Handle
 * @param twist Pointer to velocity data
 */
void VMR_setRobotTwist(VMR_Handle handle, const VmrTwistInfo_C* twist);

/**
 * @brief Input Raw Laser Scan Data
 * @param handle SDK Handle
 * @param scan Pointer to laser scan data
 */
void VMR_setLaserScan(VMR_Handle handle, const VmrScan_C* scan);

// ---------------- Callback Registration Interfaces ----------------

/** @brief Register Laser Data Callback */
void VMR_registerLaserCallback(VMR_Handle h, LaserCallback_C cb);
/** @brief Register Location Status Callback */
void VMR_registerLocationCallback(VMR_Handle h, LocationCallback_C cb);
/** @brief Register Odometry Callback */
void VMR_registerOdomCallback(int handle, OdomCallback_C cb);
/** @brief Register IMU Callback */
void VMR_registerImuCallback(int handle, ImuCallback_C cb);
/** @brief Register Battery Status Callback */
void VMR_registerBatteryCallback(int handle, BatteryCallback_C cb);
/** @brief Register Exception Info Callback */
void VMR_registerExceptionCallback(int handle, ExceptionCallback_C cb);
/** @brief Register Lift Status Callback */
void VMR_registerLiftCallback(int handle, LiftCallback_C cb);

// ---------------- Task Control Interfaces ----------------

/**
 * @brief Execute Relocalization Task
 * @param handle SDK Handle
 * @param relocate Initial pose guess
 * @param out_task_id [Output] Buffer for Task ID
 * @param max_len Maximum length of the buffer
 * @return int 0: Success, -1: Failure
 */
int VMR_relocate(VMR_Handle handle, const VmrRelocateInfo_C* relocate, char* out_task_id, int max_len);

/**
 * @brief Switch Map
 * @param handle SDK Handle
 * @param map_name Target map name
 * @param out_task_id [Output] Buffer for Task ID
 * @param max_len Maximum length of the buffer
 * @return int 0: Success, -1: Failure
 */
int VMR_changeMap(VMR_Handle handle, const char *map_name, char* out_task_id, int max_len);

/**
 * @brief Control Lift Mechanism
 * @param handle SDK Handle
 * @param height Target height (mm)
 * @param out_task_id [Output] Buffer for Task ID
 * @param max_len Maximum length of the buffer
 * @return int 0: Success, -1: Failure
 */
int VMR_controlLift(VMR_Handle handle, uint32_t height, char* out_task_id, int max_len);

/**
 * @brief Control Lights
 * @param handle SDK Handle
 * @param light_state Light state code
 */
void VMR_controlLight(VMR_Handle handle, uint32_t light_state);

/**
 * @brief Enable/Disable SDK Velocity Control Override
 * @param handle SDK Handle
 * @param speed_control true: Enable, false: Disable
 * @param out_task_id [Output] Buffer for Task ID
 * @param max_len Maximum length of the buffer
 * @return int 0: Success, -1: Failure
 */
int VMR_controlTwist(VMR_Handle handle, bool speed_control, char* out_task_id, int max_len);

/**
 * @brief Start Mapping
 * @param handle SDK Handle
 * @param map_name Name for the new map
 * @param out_task_id [Output] Buffer for Task ID
 * @param max_len Maximum length of the buffer
 * @return int 0: Success, -1: Failure
 */
int VMR_startMapping(VMR_Handle handle, const char *map_name, char* out_task_id, int max_len);

/**
 * @brief Stop Mapping
 * @param handle SDK Handle
 * @param out_task_id [Output] Buffer for Task ID
 * @param max_len Maximum length of the buffer
 * @return int 0: Success, -1: Failure
 */
int VMR_stopMapping(VMR_Handle handle, char* out_task_id, int max_len);

/**
 * @brief Start Map Extension (Resume Mapping)
 * @param handle SDK Handle
 * @param out_task_id [Output] Buffer for Task ID
 * @param max_len Maximum length of the buffer
 * @return int 0: Success, -1: Failure
 */
int VMR_startMapExtension(VMR_Handle handle, char* out_task_id, int max_len);

/**
 * @brief Issue Navigation Task (Move to Point)
 * @param handle SDK Handle
 * @param pose Target pose
 * @param out_task_id [Output] Buffer for Task ID
 * @param max_len Maximum length of the buffer
 * @return int 0: Success, -1: Failure
 */
int VMR_moveTasks(VMR_Handle handle, const VmrPose_C* pose, char* out_task_id, int max_len);
/**
 * @brief Issue Navigation Task (Move to Point)
 * @param handle SDK Handle
 * @param enable true: Enable, false: Disable
 * @return int 0: Success, -1: Failure
 */
int VMR_enableSdkCtrlSpeed(VMR_Handle handle, bool enable, char* out_task_id, int max_len);

/**
 * @brief Check Task Status
 * @param handle SDK Handle
 * @param task_id Task ID to query
 * @param out_result [Output] Pointer to result structure
 */
void VMR_checkTaskStatus(VMR_Handle handle, const char *task_id, VmrTaskResult_C* out_result);

/**
 * @brief Cancel Task
 * @param handle SDK Handle
 * @param task_id Task ID to cancel
 */
void VMR_cancelTask(VMR_Handle handle, const char *task_id);

/**
 * @brief Set Maximum Speed Limit (Deprecated: Use VMR_setSpeedFactor instead)
 * @param handle SDK Handle
 * @param speed Max speed (m/s)
 * @return int Operation result
 * @deprecated This function is deprecated. Use VMR_setSpeedFactor instead.
 */
int VMR_setMaxSpeed(VMR_Handle handle, double speed);

/**
 * @brief Set Maximum Acceleration Limit (Deprecated: Use VMR_setSpeedFactor instead)
 * @param handle SDK Handle
 * @param acc Max acceleration (m/s^2)
 * @return int Operation result
 * @deprecated This function is deprecated. Use VMR_setSpeedFactor instead.
 */
int VMR_setMaxAcc(VMR_Handle handle, double acc);

/**
 * @brief Set Speed Factor (Percentage of maximum speed)
 * @param handle SDK Handle
 * @param factor Speed factor (0.0 ~ 1.0, where 1.0 = 100% of max speed)
 * @return int Operation result (0: Success, -1: Failure)
 * @note This function replaces VMR_setMaxSpeed and VMR_setMaxAcc
 */
int VMR_setSpeedFactor(VMR_Handle handle, double factor);

/**
 * @brief  Perform a relative linear translation along a specific direction.
 * * For omnidirectional robots, this executes a holonomic translation where the 
 * chassis orientation remains fixed while the robot moves along the vector 
 * defined by rotate_angle.
 *
 * @param  handle       SDK handle for the robot instance.
 * @param  move_dist    Translation distance in meters (m).
 * @param  rotate_angle The direction of the movement vector in degrees (deg).
 * 0: Straight Forward, 90: Left, -90: Right.
 * @return std::string  Returns "OK" on success.
 */
int VMR_moveRelative(VMR_Handle handle, float move_dist,float rotate_angle, char* out_task_id, int max_len);
/**
 * @brief  Perform an in-place rotation to change the robot's heading.
 * * The robot's XY coordinates remain constant while the chassis rotates.
 *
 * @param  handle       SDK handle for the robot instance.
 * @param  rotate_angle Relative rotation angle in degrees (deg). 
 * Positive (+) for Left, Negative (-) for Right.
 * @return std::string  Returns "OK" on success.
 */
int VMR_rotateInPlace(VMR_Handle handle,float rotate_angle, char* out_task_id, int max_len);

#ifdef __cplusplus
}
#endif

#endif // VMR_SDK_C_H