#ifndef VMR_SDK_API_H
#define VMR_SDK_API_H
#include "VmrDefine.h"
#ifndef __cplusplus
#error "This header file is for C++ only. For C projects, please use VmrSDK_C.h."
#endif
/*Initialize SDK - can pass an empty string, defaults to 127.0.0.1:9000*/
int VMR_init(const char *cfg_file);
/*Initialize SDK with server IP/port (no log file required)*/
int VMR_initWithServer(const char *ip, int port);
/*Create handle*/
VMR_Handle VMR_Handle_Create();
/*Destroy handle*/
void VMR_Handle_Destroy(VMR_Handle h);
//**************** Input via SDK *************
/**
 * @brief Input odometry data
 * @param handle SDK handle
 * @param odom Odometry data (VmrOdomInfo type)
 * @return void
 */
void VMR_setRobotOdometry(VMR_Handle handle, VmrOdomInfo odom);
/**
 * @brief Issue speed command
 * @param handle SDK handle
 * @param twist Speed data (VmrTwistInfo type)
 * @return void
 */
void VMR_setRobotTwist(VMR_Handle handle, VmrTwistInfo twist);
/**
 * @brief Input laser data
 * @param handle SDK handle
 * @param scan Laser data (VmrScan type)
 * @return void
 */
void VMR_setLaserScan(VMR_Handle handle, VmrScan scan);
//**************** Output via SDK *************
/**
 * @brief Register laser output callback
 * @param h SDK handle
 * @param cb Laser data callback function (LaserCallback type)
 * @return void
 */
void VMR_registerLaserCallback(VMR_Handle h, LaserCallback cb);
/**
 * @brief Register localization status output callback
 * @param h SDK handle
 * @param cb Localization status callback function (LocationCallback type)
 * @return void
 */
void VMR_registerLocationCallback(VMR_Handle h, LocationCallback cb);
/**
 * @brief Register odometry status output callback
 * @param handle SDK handle
 * @param cb Odometry data callback function (OdomCallback type)
 * @return void
 */
void VMR_registerOdomCallback(int handle, OdomCallback cb);
/**
 * @brief Register IMU status output callback
 * @param handle SDK handle
 * @param cb IMU data callback function (ImuCallback type)
 * @return void
 */
void VMR_registerImuCallback(int handle, ImuCallback cb);
/**
 * @brief Register battery status output callback
 * @param handle SDK handle
 * @param cb Battery status callback function (BatteryCallback type)
 * @return void
 */
void VMR_registerBatteryCallback(int handle, BatteryCallback cb);
/**
 * @brief Register exception status output callback
 * @param handle SDK handle
 * @param cb Exception status callback function (ExceptionCallback type)
 * @return void
 */
void VMR_registerExceptionCallback(int handle, ExceptionCallback cb);
/**
 * @brief Register lift status output callback
 * @param handle SDK handle
 * @param cb Lift status callback function (LiftCallback type)
 * @return void
 */
void VMR_registerLiftCallback(int handle, LiftCallback cb);
//**************** Issue tasks via SDK *************
/**
 * @brief Perform relocalization
 * @param handle SDK handle
 * @param relocate Current robot pose (VmrRelocateInfo type)
 * @return Task ID string; empty string indicates failure
 */
std::string VMR_relocate(VMR_Handle handle, VmrRelocateInfo relocate);
/**
 * @brief Switch map
 * @param handle SDK handle
 * @param map_name Name of the target map
 * @return Task ID string; empty string indicates failure
 */
std::string VMR_changeMap(VMR_Handle handle, const char *map_name);
/**
 * @brief Control lift height and return task ID
 * @param handle SDK handle
 * @param height Target height (unit: mm, range 0-xxx)
 * @return Task ID string; empty string indicates failure
 */
std::string VMR_controlLift(VMR_Handle handle, uint32_t height);

/**
 * @brief Control Shelf Rotate and return task ID
 * @param handle SDK handle
 * @param shelf_rotate_theta Target shelf rotate angle (unit: degree, range 0-360)
 * @return Task ID string; empty string indicates failure
 */
std::string VMR_controlShelfRotate(VMR_Handle handle, float shelf_rotate_theta);
/**
 * @brief Control light state
 * @param handle SDK handle
 * @param light_state Target light state
 * @return void
 */
void VMR_controlLight(VMR_Handle handle, uint32_t light_state);
/**
 * @brief Start mapping
 * @param handle SDK handle
 * @param map_name Name of the map to be created
 * @return Task ID string; empty string indicates failure
 */
std::string VMR_startMapping(VMR_Handle handle, const char *map_name);
/**
 * @brief Stop mapping
 * @param handle SDK handle
 * @return Task ID string; empty string indicates failure
 */
std::string VMR_stopMapping(VMR_Handle handle);
/**
 * @brief Resume map building (extend existing map)
 * @param handle SDK handle
 * @return Task ID string; empty string indicates failure
 */
std::string VMR_startMapExtension(VMR_Handle handle);
/**
 * @brief Query task result
 * @param handle SDK handle
 * @param task_id ID of the task to query
 * @return Task result (VmrTaskResult type)
 */
VmrTaskResult VMR_checkTaskStatus(VMR_Handle handle, const char *task_id);
/**
 * @brief Cancel a task
 * @param handle SDK handle
 * @param task_id ID of the task to cancel
 * @return void
 */
void VMR_cancelTask(VMR_Handle handle, const char *task_id);
/**
 * @brief Issue point-to-point movement task
 * @param handle SDK handle
 * @param task  NavTask (VmrNavTask type)
 * @return Task ID string; empty string indicates failure
 */
std::string VMR_moveTasks(VMR_Handle handle, VmrNavTask task);
/**
 * @brief Issue point-to-point movement task
 * @param handle SDK handle
 * @param pose Target pose (VmrPose type)
 * @return Task ID string; empty string indicates failure
 */
std::string VMR_moveTasks(VMR_Handle handle, VmrPose pose);

/**
 * @brief Set maximum speed (Deprecated: Use VMR_setSpeedFactor instead)
 * @param handle SDK handle
 * @param speed Maximum speed
 * @return int Result of the operation
 * @deprecated This function is deprecated. Use VMR_setSpeedFactor instead.
 */
[[deprecated("Use VMR_setSpeedFactor instead")]]
int VMR_setMaxSpeed(VMR_Handle handle, double speed);

/**
 * @brief Set maximum acceleration (Deprecated: Use VMR_setSpeedFactor instead)
 * @param handle SDK handle
 * @param acc Maximum acceleration
 * @return int Result of the operation
 * @deprecated This function is deprecated. Use VMR_setSpeedFactor instead.
 */
[[deprecated("Use VMR_setSpeedFactor instead")]]
int VMR_setMaxAcc(VMR_Handle handle, double acc);

/**
 * @brief Set speed factor (percentage of max speed)
 * @param handle SDK handle
 * @param factor Speed factor (0.0 ~ 1.0, where 1.0 = 100% max speed)
 * @return int Result of the operation (0: Success, -1: Failure)
 */
int VMR_setSpeedFactor(VMR_Handle handle, double factor);
/**
 * @brief Enable Sdk CtrlSpeed
 * @param handle SDK handle
 * @param enable true: Enable, false: Disable
 * @return int Result of the operation
 */
std::string VMR_enableSdkCtrlSpeed(VMR_Handle handle, bool enable);

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
 * @return std::string  task_id Returns not empty on success.
 */
std::string VMR_moveRelative(VMR_Handle handle, float move_dist, float rotate_angle);

/**
 * @brief  Perform an in-place rotation to change the robot's heading.
 * * The robot's XY coordinates remain constant while the chassis rotates.
 *
 * @param  handle       SDK handle for the robot instance.
 * @param  rotate_angle Relative rotation angle in degrees (deg). 
 * Positive (+) for Left, Negative (-) for Right.
 * @return std::string  task_id Returns not empty on success
 */
std::string VMR_rotateInPlace(VMR_Handle handle, float rotate_angle);

#endif