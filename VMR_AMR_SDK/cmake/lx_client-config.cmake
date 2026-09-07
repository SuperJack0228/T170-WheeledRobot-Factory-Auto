@PACKAGE_INIT@

include(CMakeFindDependencyMacro)
find_dependency(Threads)

# 获取当前文件所在目录 (lx_client-config.cmake 位于 client/cmake/ 目录下，需要向上返回一级)
get_filename_component(LX_CLIENT_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(LX_CLIENT_ROOT "${LX_CLIENT_CMAKE_DIR}/.." ABSOLUTE)

# 设置公共路径
set(LX_CLIENT_INCLUDE_DIR "${LX_CLIENT_ROOT}/include")
# 根据平台选择对应的库目录
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(LX_CLIENT_LIB_DIR "${LX_CLIENT_ROOT}/lib/arm64")
else()
    set(LX_CLIENT_LIB_DIR "${LX_CLIENT_ROOT}/lib/x86")
endif()

# ============================================
# 所有 Namespace 配置
# ============================================
# Namespace: laser_2d
add_library(lx_client::laser_2d SHARED IMPORTED)
set_target_properties(lx_client::laser_2d PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_laser_2d.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "Threads::Threads"
)

add_library(lx_client::laser_2d_internal SHARED IMPORTED)
set_target_properties(lx_client::laser_2d_internal PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# 创建统一的接口库
add_library(lx_client::lx_client_laser_2d INTERFACE IMPORTED)
set_target_properties(lx_client::lx_client_laser_2d PROPERTIES
    INTERFACE_LINK_LIBRARIES "lx_client::laser_2d;lx_client::laser_2d_internal"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# Namespace: map_reader
add_library(lx_client::map_reader SHARED IMPORTED)
set_target_properties(lx_client::map_reader PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_map_reader.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "Threads::Threads"
)

add_library(lx_client::map_reader_internal SHARED IMPORTED)
set_target_properties(lx_client::map_reader_internal PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# 创建统一的接口库
add_library(lx_client::lx_client_map_reader INTERFACE IMPORTED)
set_target_properties(lx_client::lx_client_map_reader PROPERTIES
    INTERFACE_LINK_LIBRARIES "lx_client::map_reader;lx_client::map_reader_internal"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# Namespace: bridge_scan
add_library(lx_client::bridge_scan SHARED IMPORTED)
set_target_properties(lx_client::bridge_scan PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_bridge_scan.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "Threads::Threads"
)

add_library(lx_client::bridge_scan_internal SHARED IMPORTED)
set_target_properties(lx_client::bridge_scan_internal PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# 创建统一的接口库
add_library(lx_client::lx_client_bridge_scan INTERFACE IMPORTED)
set_target_properties(lx_client::lx_client_bridge_scan PROPERTIES
    INTERFACE_LINK_LIBRARIES "lx_client::bridge_scan;lx_client::bridge_scan_internal"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# Namespace: depth_semantic
add_library(lx_client::depth_semantic SHARED IMPORTED)
set_target_properties(lx_client::depth_semantic PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_depth_semantic.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "Threads::Threads"
)

add_library(lx_client::depth_semantic_internal SHARED IMPORTED)
set_target_properties(lx_client::depth_semantic_internal PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# 创建统一的接口库
add_library(lx_client::lx_client_depth_semantic INTERFACE IMPORTED)
set_target_properties(lx_client::lx_client_depth_semantic PROPERTIES
    INTERFACE_LINK_LIBRARIES "lx_client::depth_semantic;lx_client::depth_semantic_internal"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# Namespace: io_manager
add_library(lx_client::io_manager SHARED IMPORTED)
set_target_properties(lx_client::io_manager PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_io_manager.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "Threads::Threads"
)

add_library(lx_client::io_manager_internal SHARED IMPORTED)
set_target_properties(lx_client::io_manager_internal PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# 创建统一的接口库
add_library(lx_client::lx_client_io_manager INTERFACE IMPORTED)
set_target_properties(lx_client::lx_client_io_manager PROPERTIES
    INTERFACE_LINK_LIBRARIES "lx_client::io_manager;lx_client::io_manager_internal"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# Namespace: bridge_drive
add_library(lx_client::bridge_drive SHARED IMPORTED)
set_target_properties(lx_client::bridge_drive PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_bridge_drive.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "Threads::Threads"
)

add_library(lx_client::bridge_drive_internal SHARED IMPORTED)
set_target_properties(lx_client::bridge_drive_internal PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# 创建统一的接口库
add_library(lx_client::lx_client_bridge_drive INTERFACE IMPORTED)
set_target_properties(lx_client::lx_client_bridge_drive PROPERTIES
    INTERFACE_LINK_LIBRARIES "lx_client::bridge_drive;lx_client::bridge_drive_internal"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# Namespace: imu
add_library(lx_client::imu SHARED IMPORTED)
set_target_properties(lx_client::imu PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_imu.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "Threads::Threads"
)

add_library(lx_client::imu_internal SHARED IMPORTED)
set_target_properties(lx_client::imu_internal PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# 创建统一的接口库
add_library(lx_client::lx_client_imu INTERFACE IMPORTED)
set_target_properties(lx_client::lx_client_imu PROPERTIES
    INTERFACE_LINK_LIBRARIES "lx_client::imu;lx_client::imu_internal"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# Namespace: exception
add_library(lx_client::exception SHARED IMPORTED)
set_target_properties(lx_client::exception PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_exception.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "Threads::Threads"
)

add_library(lx_client::exception_internal SHARED IMPORTED)
set_target_properties(lx_client::exception_internal PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# 创建统一的接口库
add_library(lx_client::lx_client_exception INTERFACE IMPORTED)
set_target_properties(lx_client::lx_client_exception PROPERTIES
    INTERFACE_LINK_LIBRARIES "lx_client::exception;lx_client::exception_internal"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# Namespace: nav
add_library(lx_client::nav SHARED IMPORTED)
set_target_properties(lx_client::nav PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_nav.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "Threads::Threads"
)

add_library(lx_client::nav_internal SHARED IMPORTED)
set_target_properties(lx_client::nav_internal PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
add_library(lx_client::nav_cmd SHARED IMPORTED)
set_target_properties(lx_client::nav_cmd PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libcmd_promise_manager.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)

# 创建统一的接口库
add_library(lx_client::lx_client_nav INTERFACE IMPORTED)
set_target_properties(lx_client::lx_client_nav PROPERTIES
    INTERFACE_LINK_LIBRARIES "lx_client::nav;lx_client::nav_internal"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# Namespace: point_cloud
add_library(lx_client::point_cloud SHARED IMPORTED)
set_target_properties(lx_client::point_cloud PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_point_cloud.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "Threads::Threads"
)

add_library(lx_client::point_cloud_internal SHARED IMPORTED)
set_target_properties(lx_client::point_cloud_internal PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# 创建统一的接口库
add_library(lx_client::lx_client_point_cloud INTERFACE IMPORTED)
set_target_properties(lx_client::lx_client_point_cloud PROPERTIES
    INTERFACE_LINK_LIBRARIES "lx_client::point_cloud;lx_client::point_cloud_internal"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# Namespace: audio
add_library(lx_client::audio SHARED IMPORTED)
set_target_properties(lx_client::audio PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_audio.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "Threads::Threads"
)

add_library(lx_client::audio_internal SHARED IMPORTED)
set_target_properties(lx_client::audio_internal PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# 创建统一的接口库
add_library(lx_client::lx_client_audio INTERFACE IMPORTED)
set_target_properties(lx_client::lx_client_audio PROPERTIES
    INTERFACE_LINK_LIBRARIES "lx_client::audio;lx_client::audio_internal"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# Namespace: obstacle_avoidance
add_library(lx_client::obstacle_avoidance SHARED IMPORTED)
set_target_properties(lx_client::obstacle_avoidance PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_obstacle_avoidance.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "Threads::Threads"
)

add_library(lx_client::obstacle_avoidance_internal SHARED IMPORTED)
set_target_properties(lx_client::obstacle_avoidance_internal PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# 创建统一的接口库
add_library(lx_client::lx_client_obstacle_avoidance INTERFACE IMPORTED)
set_target_properties(lx_client::lx_client_obstacle_avoidance PROPERTIES
    INTERFACE_LINK_LIBRARIES "lx_client::obstacle_avoidance;lx_client::obstacle_avoidance_internal"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# Namespace: sensor
add_library(lx_client::sensor SHARED IMPORTED)
set_target_properties(lx_client::sensor PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_sensor.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "Threads::Threads"
)

add_library(lx_client::sensor_internal SHARED IMPORTED)
set_target_properties(lx_client::sensor_internal PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# 创建统一的接口库
add_library(lx_client::lx_client_sensor INTERFACE IMPORTED)
set_target_properties(lx_client::lx_client_sensor PROPERTIES
    INTERFACE_LINK_LIBRARIES "lx_client::sensor;lx_client::sensor_internal"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# Namespace: localization
add_library(lx_client::localization SHARED IMPORTED)
set_target_properties(lx_client::localization PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_localization.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "Threads::Threads"
)

add_library(lx_client::localization_internal SHARED IMPORTED)
set_target_properties(lx_client::localization_internal PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# 创建统一的接口库
add_library(lx_client::lx_client_localization INTERFACE IMPORTED)
set_target_properties(lx_client::lx_client_localization PROPERTIES
    INTERFACE_LINK_LIBRARIES "lx_client::localization;lx_client::localization_internal"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# Namespace: robot_task
add_library(lx_client::robot_task SHARED IMPORTED)
set_target_properties(lx_client::robot_task PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_robot_task.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "Threads::Threads"
)

add_library(lx_client::robot_task_internal SHARED IMPORTED)
set_target_properties(lx_client::robot_task_internal PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
add_library(lx_client::robot_task_cmd SHARED IMPORTED)
set_target_properties(lx_client::robot_task_cmd PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libcmd_promise_manager.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)

# 创建统一的接口库
add_library(lx_client::lx_client_robot_task INTERFACE IMPORTED)
set_target_properties(lx_client::lx_client_robot_task PROPERTIES
    INTERFACE_LINK_LIBRARIES "lx_client::robot_task;lx_client::robot_task_internal"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# Namespace: light
add_library(lx_client::light SHARED IMPORTED)
set_target_properties(lx_client::light PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_light.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "Threads::Threads"
)

add_library(lx_client::light_internal SHARED IMPORTED)
set_target_properties(lx_client::light_internal PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# 创建统一的接口库
add_library(lx_client::lx_client_light INTERFACE IMPORTED)
set_target_properties(lx_client::lx_client_light PROPERTIES
    INTERFACE_LINK_LIBRARIES "lx_client::light;lx_client::light_internal"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# Namespace: lift
add_library(lx_client::lift SHARED IMPORTED)
set_target_properties(lx_client::lift PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_lift.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "Threads::Threads"
)

add_library(lx_client::lift_internal SHARED IMPORTED)
set_target_properties(lx_client::lift_internal PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
add_library(lx_client::lift_cmd SHARED IMPORTED)
set_target_properties(lx_client::lift_cmd PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libcmd_promise_manager.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)

# 创建统一的接口库
add_library(lx_client::lx_client_lift INTERFACE IMPORTED)
set_target_properties(lx_client::lx_client_lift PROPERTIES
    INTERFACE_LINK_LIBRARIES "lx_client::lift;lx_client::lift_internal"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# Namespace: odometry
add_library(lx_client::odometry SHARED IMPORTED)
set_target_properties(lx_client::odometry PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_odometry.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "Threads::Threads"
)

add_library(lx_client::odometry_internal SHARED IMPORTED)
set_target_properties(lx_client::odometry_internal PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# 创建统一的接口库
add_library(lx_client::lx_client_odometry INTERFACE IMPORTED)
set_target_properties(lx_client::lx_client_odometry PROPERTIES
    INTERFACE_LINK_LIBRARIES "lx_client::odometry;lx_client::odometry_internal"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# Namespace: battery
add_library(lx_client::battery SHARED IMPORTED)
set_target_properties(lx_client::battery PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_battery.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "Threads::Threads"
)

add_library(lx_client::battery_internal SHARED IMPORTED)
set_target_properties(lx_client::battery_internal PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# 创建统一的接口库
add_library(lx_client::lx_client_battery INTERFACE IMPORTED)
set_target_properties(lx_client::lx_client_battery PROPERTIES
    INTERFACE_LINK_LIBRARIES "lx_client::battery;lx_client::battery_internal"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# Namespace: qrcamera
add_library(lx_client::qrcamera SHARED IMPORTED)
set_target_properties(lx_client::qrcamera PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_qrcamera.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "Threads::Threads"
)

add_library(lx_client::qrcamera_internal SHARED IMPORTED)
set_target_properties(lx_client::qrcamera_internal PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# 创建统一的接口库
add_library(lx_client::lx_client_qrcamera INTERFACE IMPORTED)
set_target_properties(lx_client::lx_client_qrcamera PROPERTIES
    INTERFACE_LINK_LIBRARIES "lx_client::qrcamera;lx_client::qrcamera_internal"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
# Namespace: rpc_test
add_library(lx_client::rpc_test SHARED IMPORTED)
set_target_properties(lx_client::rpc_test PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_rpc_test.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "Threads::Threads"
)

add_library(lx_client::rpc_test_internal SHARED IMPORTED)
set_target_properties(lx_client::rpc_test_internal PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)
add_library(lx_client::rpc_test_cmd SHARED IMPORTED)
set_target_properties(lx_client::rpc_test_cmd PROPERTIES
    IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libcmd_promise_manager.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)

# 创建统一的接口库
add_library(lx_client::lx_client_rpc_test INTERFACE IMPORTED)
set_target_properties(lx_client::lx_client_rpc_test PROPERTIES
    INTERFACE_LINK_LIBRARIES "lx_client::rpc_test;lx_client::rpc_test_internal"
    INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
)

# ============================================
# 检查必需组件
check_required_components(lx_client)

# 提供旧式变量兼容
set(LX_CLIENT_FOUND TRUE)
set(LX_CLIENT_INCLUDE_DIRS "${LX_CLIENT_INCLUDE_DIR}")
set(LX_CLIENT_LIBRARY_DIRS "${LX_CLIENT_LIB_DIR}")

# 所有可用库列表
set(LX_CLIENT_AVAILABLE_NAMESPACES
    laser_2d
    map_reader
    bridge_scan
    depth_semantic
    io_manager
    bridge_drive
    imu
    exception
    nav
    point_cloud
    audio
    obstacle_avoidance
    sensor
    localization
    robot_task
    light
    lift
    odometry
    battery
    qrcamera
    rpc_test
)

# 辅助函数：链接所有可用的 lx_client 库
function(lx_client_link_all target)
    if(TARGET lx_client::lx_client_laser_2d)
        target_link_libraries(${target} lx_client::lx_client_laser_2d)
    endif()
    if(TARGET lx_client::lx_client_map_reader)
        target_link_libraries(${target} lx_client::lx_client_map_reader)
    endif()
    if(TARGET lx_client::lx_client_bridge_scan)
        target_link_libraries(${target} lx_client::lx_client_bridge_scan)
    endif()
    if(TARGET lx_client::lx_client_depth_semantic)
        target_link_libraries(${target} lx_client::lx_client_depth_semantic)
    endif()
    if(TARGET lx_client::lx_client_io_manager)
        target_link_libraries(${target} lx_client::lx_client_io_manager)
    endif()
    if(TARGET lx_client::lx_client_bridge_drive)
        target_link_libraries(${target} lx_client::lx_client_bridge_drive)
    endif()
    if(TARGET lx_client::lx_client_imu)
        target_link_libraries(${target} lx_client::lx_client_imu)
    endif()
    if(TARGET lx_client::lx_client_exception)
        target_link_libraries(${target} lx_client::lx_client_exception)
    endif()
    if(TARGET lx_client::lx_client_nav)
        target_link_libraries(${target} lx_client::lx_client_nav)
    endif()
    if(TARGET lx_client::lx_client_point_cloud)
        target_link_libraries(${target} lx_client::lx_client_point_cloud)
    endif()
    if(TARGET lx_client::lx_client_audio)
        target_link_libraries(${target} lx_client::lx_client_audio)
    endif()
    if(TARGET lx_client::lx_client_obstacle_avoidance)
        target_link_libraries(${target} lx_client::lx_client_obstacle_avoidance)
    endif()
    if(TARGET lx_client::lx_client_sensor)
        target_link_libraries(${target} lx_client::lx_client_sensor)
    endif()
    if(TARGET lx_client::lx_client_localization)
        target_link_libraries(${target} lx_client::lx_client_localization)
    endif()
    if(TARGET lx_client::lx_client_robot_task)
        target_link_libraries(${target} lx_client::lx_client_robot_task)
    endif()
    if(TARGET lx_client::lx_client_light)
        target_link_libraries(${target} lx_client::lx_client_light)
    endif()
    if(TARGET lx_client::lx_client_lift)
        target_link_libraries(${target} lx_client::lx_client_lift)
    endif()
    if(TARGET lx_client::lx_client_odometry)
        target_link_libraries(${target} lx_client::lx_client_odometry)
    endif()
    if(TARGET lx_client::lx_client_battery)
        target_link_libraries(${target} lx_client::lx_client_battery)
    endif()
    if(TARGET lx_client::lx_client_qrcamera)
        target_link_libraries(${target} lx_client::lx_client_qrcamera)
    endif()
    if(TARGET lx_client::lx_client_rpc_test)
        target_link_libraries(${target} lx_client::lx_client_rpc_test)
    endif()
endfunction()

message(STATUS "LX Client Libraries found in: ${LX_CLIENT_ROOT}")
message(STATUS "  - lx_client::lx_client_laser_2d (laser_2d)")
message(STATUS "  - lx_client::lx_client_map_reader (map_reader)")
message(STATUS "  - lx_client::lx_client_bridge_scan (bridge_scan)")
message(STATUS "  - lx_client::lx_client_depth_semantic (depth_semantic)")
message(STATUS "  - lx_client::lx_client_io_manager (io_manager)")
message(STATUS "  - lx_client::lx_client_bridge_drive (bridge_drive)")
message(STATUS "  - lx_client::lx_client_imu (imu)")
message(STATUS "  - lx_client::lx_client_exception (exception)")
message(STATUS "  - lx_client::lx_client_nav (nav)")
message(STATUS "  - lx_client::lx_client_point_cloud (point_cloud)")
message(STATUS "  - lx_client::lx_client_audio (audio)")
message(STATUS "  - lx_client::lx_client_obstacle_avoidance (obstacle_avoidance)")
message(STATUS "  - lx_client::lx_client_sensor (sensor)")
message(STATUS "  - lx_client::lx_client_localization (localization)")
message(STATUS "  - lx_client::lx_client_robot_task (robot_task)")
message(STATUS "  - lx_client::lx_client_light (light)")
message(STATUS "  - lx_client::lx_client_lift (lift)")
message(STATUS "  - lx_client::lx_client_odometry (odometry)")
message(STATUS "  - lx_client::lx_client_battery (battery)")
message(STATUS "  - lx_client::lx_client_qrcamera (qrcamera)")
message(STATUS "  - lx_client::lx_client_rpc_test (rpc_test)")