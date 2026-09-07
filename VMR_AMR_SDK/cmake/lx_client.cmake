# LX Client Libraries
# 自动生成的 CMake 配置文件 - 包含所有 RPC/Publish/Cmd namespace
#
# 使用方法:
#   include(<path>/lx_client.cmake)
#   target_link_libraries(your_target lx_client_<namespace>)
#
# 可用的 namespace 目标:
#   - lx_client_laser_2d (laser_2d)
#   - lx_client_map_reader (map_reader)
#   - lx_client_bridge_scan (bridge_scan)
#   - lx_client_depth_semantic (depth_semantic)
#   - lx_client_io_manager (io_manager)
#   - lx_client_bridge_drive (bridge_drive)
#   - lx_client_imu (imu)
#   - lx_client_exception (exception)
#   - lx_client_nav (nav)
#   - lx_client_point_cloud (point_cloud)
#   - lx_client_audio (audio)
#   - lx_client_obstacle_avoidance (obstacle_avoidance)
#   - lx_client_sensor (sensor)
#   - lx_client_localization (localization)
#   - lx_client_robot_task (robot_task)
#   - lx_client_light (light)
#   - lx_client_lift (lift)
#   - lx_client_odometry (odometry)
#   - lx_client_battery (battery)
#   - lx_client_qrcamera (qrcamera)
#   - lx_client_rpc_test (rpc_test)
#
# 获取当前文件所在目录 (lx_client.cmake 位于 client/cmake/ 目录下，需要向上返回一级)
get_filename_component(LX_CLIENT_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(LX_CLIENT_ROOT "${LX_CLIENT_CMAKE_DIR}/.." ABSOLUTE)

# 设置公共包含目录
set(LX_CLIENT_INCLUDE_DIR "${LX_CLIENT_ROOT}/include")
# 根据平台选择对应的库目录
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(LX_CLIENT_LIB_DIR "${LX_CLIENT_ROOT}/lib/arm64")
else()
    set(LX_CLIENT_LIB_DIR "${LX_CLIENT_ROOT}/lib/x86")
endif()

# 添加头文件搜索路径
include_directories(${LX_CLIENT_INCLUDE_DIR})

# 查找依赖库
find_package(Threads REQUIRED)

# ============================================
# Namespace 配置
# ============================================
# ----------------------------------------
# Namespace: laser_2d
# ----------------------------------------

# 查找库文件
find_library(CLIENT_laser_2d_LIB 
    NAMES client_laser_2d
    PATHS ${LX_CLIENT_LIB_DIR}
    NO_DEFAULT_PATH
)
# 检查库是否找到
if(CLIENT_laser_2d_LIB)
    # 创建导入目标
    if(NOT TARGET client_laser_2d)
        add_library(client_laser_2d SHARED IMPORTED)
        set_target_properties(client_laser_2d PROPERTIES
            IMPORTED_LOCATION "${CLIENT_laser_2d_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "Threads::Threads"
        )
    endif()

    if(NOT TARGET client_internal_laser_2d)
        add_library(client_internal_laser_2d SHARED IMPORTED)
        set_target_properties(client_internal_laser_2d PROPERTIES
            IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
        )
    endif()

    # 创建统一的接口库
    if(NOT TARGET lx_client_laser_2d)
        add_library(lx_client_laser_2d INTERFACE)
        target_link_libraries(lx_client_laser_2d INTERFACE
            client_laser_2d
            client_internal_laser_2d
        )
        target_include_directories(lx_client_laser_2d INTERFACE
            ${LX_CLIENT_INCLUDE_DIR}
        )
    endif()

    set(LX_CLIENT_laser_2d_FOUND TRUE)
    message(STATUS "  [OK] laser_2d: ${CLIENT_laser_2d_LIB}")
else()
    set(LX_CLIENT_laser_2d_FOUND FALSE)
    message(WARNING "  [MISSING] laser_2d: client_laser_2d not found")
endif()
# ----------------------------------------
# Namespace: map_reader
# ----------------------------------------

# 查找库文件
find_library(CLIENT_map_reader_LIB 
    NAMES client_map_reader
    PATHS ${LX_CLIENT_LIB_DIR}
    NO_DEFAULT_PATH
)
# 检查库是否找到
if(CLIENT_map_reader_LIB)
    # 创建导入目标
    if(NOT TARGET client_map_reader)
        add_library(client_map_reader SHARED IMPORTED)
        set_target_properties(client_map_reader PROPERTIES
            IMPORTED_LOCATION "${CLIENT_map_reader_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "Threads::Threads"
        )
    endif()

    if(NOT TARGET client_internal_map_reader)
        add_library(client_internal_map_reader SHARED IMPORTED)
        set_target_properties(client_internal_map_reader PROPERTIES
            IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
        )
    endif()

    # 创建统一的接口库
    if(NOT TARGET lx_client_map_reader)
        add_library(lx_client_map_reader INTERFACE)
        target_link_libraries(lx_client_map_reader INTERFACE
            client_map_reader
            client_internal_map_reader
        )
        target_include_directories(lx_client_map_reader INTERFACE
            ${LX_CLIENT_INCLUDE_DIR}
        )
    endif()

    set(LX_CLIENT_map_reader_FOUND TRUE)
    message(STATUS "  [OK] map_reader: ${CLIENT_map_reader_LIB}")
else()
    set(LX_CLIENT_map_reader_FOUND FALSE)
    message(WARNING "  [MISSING] map_reader: client_map_reader not found")
endif()
# ----------------------------------------
# Namespace: bridge_scan
# ----------------------------------------

# 查找库文件
find_library(CLIENT_bridge_scan_LIB 
    NAMES client_bridge_scan
    PATHS ${LX_CLIENT_LIB_DIR}
    NO_DEFAULT_PATH
)
# 检查库是否找到
if(CLIENT_bridge_scan_LIB)
    # 创建导入目标
    if(NOT TARGET client_bridge_scan)
        add_library(client_bridge_scan SHARED IMPORTED)
        set_target_properties(client_bridge_scan PROPERTIES
            IMPORTED_LOCATION "${CLIENT_bridge_scan_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "Threads::Threads"
        )
    endif()

    if(NOT TARGET client_internal_bridge_scan)
        add_library(client_internal_bridge_scan SHARED IMPORTED)
        set_target_properties(client_internal_bridge_scan PROPERTIES
            IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
        )
    endif()

    # 创建统一的接口库
    if(NOT TARGET lx_client_bridge_scan)
        add_library(lx_client_bridge_scan INTERFACE)
        target_link_libraries(lx_client_bridge_scan INTERFACE
            client_bridge_scan
            client_internal_bridge_scan
        )
        target_include_directories(lx_client_bridge_scan INTERFACE
            ${LX_CLIENT_INCLUDE_DIR}
        )
    endif()

    set(LX_CLIENT_bridge_scan_FOUND TRUE)
    message(STATUS "  [OK] bridge_scan: ${CLIENT_bridge_scan_LIB}")
else()
    set(LX_CLIENT_bridge_scan_FOUND FALSE)
    message(WARNING "  [MISSING] bridge_scan: client_bridge_scan not found")
endif()
# ----------------------------------------
# Namespace: depth_semantic
# ----------------------------------------

# 查找库文件
find_library(CLIENT_depth_semantic_LIB 
    NAMES client_depth_semantic
    PATHS ${LX_CLIENT_LIB_DIR}
    NO_DEFAULT_PATH
)
# 检查库是否找到
if(CLIENT_depth_semantic_LIB)
    # 创建导入目标
    if(NOT TARGET client_depth_semantic)
        add_library(client_depth_semantic SHARED IMPORTED)
        set_target_properties(client_depth_semantic PROPERTIES
            IMPORTED_LOCATION "${CLIENT_depth_semantic_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "Threads::Threads"
        )
    endif()

    if(NOT TARGET client_internal_depth_semantic)
        add_library(client_internal_depth_semantic SHARED IMPORTED)
        set_target_properties(client_internal_depth_semantic PROPERTIES
            IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
        )
    endif()

    # 创建统一的接口库
    if(NOT TARGET lx_client_depth_semantic)
        add_library(lx_client_depth_semantic INTERFACE)
        target_link_libraries(lx_client_depth_semantic INTERFACE
            client_depth_semantic
            client_internal_depth_semantic
        )
        target_include_directories(lx_client_depth_semantic INTERFACE
            ${LX_CLIENT_INCLUDE_DIR}
        )
    endif()

    set(LX_CLIENT_depth_semantic_FOUND TRUE)
    message(STATUS "  [OK] depth_semantic: ${CLIENT_depth_semantic_LIB}")
else()
    set(LX_CLIENT_depth_semantic_FOUND FALSE)
    message(WARNING "  [MISSING] depth_semantic: client_depth_semantic not found")
endif()
# ----------------------------------------
# Namespace: io_manager
# ----------------------------------------

# 查找库文件
find_library(CLIENT_io_manager_LIB 
    NAMES client_io_manager
    PATHS ${LX_CLIENT_LIB_DIR}
    NO_DEFAULT_PATH
)
# 检查库是否找到
if(CLIENT_io_manager_LIB)
    # 创建导入目标
    if(NOT TARGET client_io_manager)
        add_library(client_io_manager SHARED IMPORTED)
        set_target_properties(client_io_manager PROPERTIES
            IMPORTED_LOCATION "${CLIENT_io_manager_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "Threads::Threads"
        )
    endif()

    if(NOT TARGET client_internal_io_manager)
        add_library(client_internal_io_manager SHARED IMPORTED)
        set_target_properties(client_internal_io_manager PROPERTIES
            IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
        )
    endif()

    # 创建统一的接口库
    if(NOT TARGET lx_client_io_manager)
        add_library(lx_client_io_manager INTERFACE)
        target_link_libraries(lx_client_io_manager INTERFACE
            client_io_manager
            client_internal_io_manager
        )
        target_include_directories(lx_client_io_manager INTERFACE
            ${LX_CLIENT_INCLUDE_DIR}
        )
    endif()

    set(LX_CLIENT_io_manager_FOUND TRUE)
    message(STATUS "  [OK] io_manager: ${CLIENT_io_manager_LIB}")
else()
    set(LX_CLIENT_io_manager_FOUND FALSE)
    message(WARNING "  [MISSING] io_manager: client_io_manager not found")
endif()
# ----------------------------------------
# Namespace: bridge_drive
# ----------------------------------------

# 查找库文件
find_library(CLIENT_bridge_drive_LIB 
    NAMES client_bridge_drive
    PATHS ${LX_CLIENT_LIB_DIR}
    NO_DEFAULT_PATH
)
# 检查库是否找到
if(CLIENT_bridge_drive_LIB)
    # 创建导入目标
    if(NOT TARGET client_bridge_drive)
        add_library(client_bridge_drive SHARED IMPORTED)
        set_target_properties(client_bridge_drive PROPERTIES
            IMPORTED_LOCATION "${CLIENT_bridge_drive_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "Threads::Threads"
        )
    endif()

    if(NOT TARGET client_internal_bridge_drive)
        add_library(client_internal_bridge_drive SHARED IMPORTED)
        set_target_properties(client_internal_bridge_drive PROPERTIES
            IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
        )
    endif()

    # 创建统一的接口库
    if(NOT TARGET lx_client_bridge_drive)
        add_library(lx_client_bridge_drive INTERFACE)
        target_link_libraries(lx_client_bridge_drive INTERFACE
            client_bridge_drive
            client_internal_bridge_drive
        )
        target_include_directories(lx_client_bridge_drive INTERFACE
            ${LX_CLIENT_INCLUDE_DIR}
        )
    endif()

    set(LX_CLIENT_bridge_drive_FOUND TRUE)
    message(STATUS "  [OK] bridge_drive: ${CLIENT_bridge_drive_LIB}")
else()
    set(LX_CLIENT_bridge_drive_FOUND FALSE)
    message(WARNING "  [MISSING] bridge_drive: client_bridge_drive not found")
endif()
# ----------------------------------------
# Namespace: imu
# ----------------------------------------

# 查找库文件
find_library(CLIENT_imu_LIB 
    NAMES client_imu
    PATHS ${LX_CLIENT_LIB_DIR}
    NO_DEFAULT_PATH
)
# 检查库是否找到
if(CLIENT_imu_LIB)
    # 创建导入目标
    if(NOT TARGET client_imu)
        add_library(client_imu SHARED IMPORTED)
        set_target_properties(client_imu PROPERTIES
            IMPORTED_LOCATION "${CLIENT_imu_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "Threads::Threads"
        )
    endif()

    if(NOT TARGET client_internal_imu)
        add_library(client_internal_imu SHARED IMPORTED)
        set_target_properties(client_internal_imu PROPERTIES
            IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
        )
    endif()

    # 创建统一的接口库
    if(NOT TARGET lx_client_imu)
        add_library(lx_client_imu INTERFACE)
        target_link_libraries(lx_client_imu INTERFACE
            client_imu
            client_internal_imu
        )
        target_include_directories(lx_client_imu INTERFACE
            ${LX_CLIENT_INCLUDE_DIR}
        )
    endif()

    set(LX_CLIENT_imu_FOUND TRUE)
    message(STATUS "  [OK] imu: ${CLIENT_imu_LIB}")
else()
    set(LX_CLIENT_imu_FOUND FALSE)
    message(WARNING "  [MISSING] imu: client_imu not found")
endif()
# ----------------------------------------
# Namespace: exception
# ----------------------------------------

# 查找库文件
find_library(CLIENT_exception_LIB 
    NAMES client_exception
    PATHS ${LX_CLIENT_LIB_DIR}
    NO_DEFAULT_PATH
)
# 检查库是否找到
if(CLIENT_exception_LIB)
    # 创建导入目标
    if(NOT TARGET client_exception)
        add_library(client_exception SHARED IMPORTED)
        set_target_properties(client_exception PROPERTIES
            IMPORTED_LOCATION "${CLIENT_exception_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "Threads::Threads"
        )
    endif()

    if(NOT TARGET client_internal_exception)
        add_library(client_internal_exception SHARED IMPORTED)
        set_target_properties(client_internal_exception PROPERTIES
            IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
        )
    endif()

    # 创建统一的接口库
    if(NOT TARGET lx_client_exception)
        add_library(lx_client_exception INTERFACE)
        target_link_libraries(lx_client_exception INTERFACE
            client_exception
            client_internal_exception
        )
        target_include_directories(lx_client_exception INTERFACE
            ${LX_CLIENT_INCLUDE_DIR}
        )
    endif()

    set(LX_CLIENT_exception_FOUND TRUE)
    message(STATUS "  [OK] exception: ${CLIENT_exception_LIB}")
else()
    set(LX_CLIENT_exception_FOUND FALSE)
    message(WARNING "  [MISSING] exception: client_exception not found")
endif()
# ----------------------------------------
# Namespace: nav
# ----------------------------------------

# 查找库文件
find_library(CLIENT_nav_LIB 
    NAMES client_nav
    PATHS ${LX_CLIENT_LIB_DIR}
    NO_DEFAULT_PATH
)
find_library(CMD_PROMISE_MANAGER_nav_LIB 
    NAMES cmd_promise_manager
    PATHS ${LX_CLIENT_LIB_DIR}
    NO_DEFAULT_PATH
)

# 检查库是否找到
if(CLIENT_nav_LIB)
    # 创建导入目标
    if(NOT TARGET client_nav)
        add_library(client_nav SHARED IMPORTED)
        set_target_properties(client_nav PROPERTIES
            IMPORTED_LOCATION "${CLIENT_nav_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "Threads::Threads"
        )
    endif()

    if(NOT TARGET client_internal_nav)
        add_library(client_internal_nav SHARED IMPORTED)
        set_target_properties(client_internal_nav PROPERTIES
            IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
        )
    endif()
    if(CMD_PROMISE_MANAGER_nav_LIB AND NOT TARGET cmd_promise_manager_nav)
        add_library(cmd_promise_manager_nav SHARED IMPORTED)
        set_target_properties(cmd_promise_manager_nav PROPERTIES
            IMPORTED_LOCATION "${CMD_PROMISE_MANAGER_nav_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
        )
    endif()

    # 创建统一的接口库
    if(NOT TARGET lx_client_nav)
        add_library(lx_client_nav INTERFACE)
        target_link_libraries(lx_client_nav INTERFACE
            client_nav
            client_internal_nav
            $<$<TARGET_EXISTS:cmd_promise_manager_nav>:cmd_promise_manager_nav>
        )
        target_include_directories(lx_client_nav INTERFACE
            ${LX_CLIENT_INCLUDE_DIR}
        )
    endif()

    set(LX_CLIENT_nav_FOUND TRUE)
    message(STATUS "  [OK] nav: ${CLIENT_nav_LIB}")
else()
    set(LX_CLIENT_nav_FOUND FALSE)
    message(WARNING "  [MISSING] nav: client_nav not found")
endif()
# ----------------------------------------
# Namespace: point_cloud
# ----------------------------------------

# 查找库文件
find_library(CLIENT_point_cloud_LIB 
    NAMES client_point_cloud
    PATHS ${LX_CLIENT_LIB_DIR}
    NO_DEFAULT_PATH
)
# 检查库是否找到
if(CLIENT_point_cloud_LIB)
    # 创建导入目标
    if(NOT TARGET client_point_cloud)
        add_library(client_point_cloud SHARED IMPORTED)
        set_target_properties(client_point_cloud PROPERTIES
            IMPORTED_LOCATION "${CLIENT_point_cloud_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "Threads::Threads"
        )
    endif()

    if(NOT TARGET client_internal_point_cloud)
        add_library(client_internal_point_cloud SHARED IMPORTED)
        set_target_properties(client_internal_point_cloud PROPERTIES
            IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
        )
    endif()

    # 创建统一的接口库
    if(NOT TARGET lx_client_point_cloud)
        add_library(lx_client_point_cloud INTERFACE)
        target_link_libraries(lx_client_point_cloud INTERFACE
            client_point_cloud
            client_internal_point_cloud
        )
        target_include_directories(lx_client_point_cloud INTERFACE
            ${LX_CLIENT_INCLUDE_DIR}
        )
    endif()

    set(LX_CLIENT_point_cloud_FOUND TRUE)
    message(STATUS "  [OK] point_cloud: ${CLIENT_point_cloud_LIB}")
else()
    set(LX_CLIENT_point_cloud_FOUND FALSE)
    message(WARNING "  [MISSING] point_cloud: client_point_cloud not found")
endif()
# ----------------------------------------
# Namespace: audio
# ----------------------------------------

# 查找库文件
find_library(CLIENT_audio_LIB 
    NAMES client_audio
    PATHS ${LX_CLIENT_LIB_DIR}
    NO_DEFAULT_PATH
)
# 检查库是否找到
if(CLIENT_audio_LIB)
    # 创建导入目标
    if(NOT TARGET client_audio)
        add_library(client_audio SHARED IMPORTED)
        set_target_properties(client_audio PROPERTIES
            IMPORTED_LOCATION "${CLIENT_audio_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "Threads::Threads"
        )
    endif()

    if(NOT TARGET client_internal_audio)
        add_library(client_internal_audio SHARED IMPORTED)
        set_target_properties(client_internal_audio PROPERTIES
            IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
        )
    endif()

    # 创建统一的接口库
    if(NOT TARGET lx_client_audio)
        add_library(lx_client_audio INTERFACE)
        target_link_libraries(lx_client_audio INTERFACE
            client_audio
            client_internal_audio
        )
        target_include_directories(lx_client_audio INTERFACE
            ${LX_CLIENT_INCLUDE_DIR}
        )
    endif()

    set(LX_CLIENT_audio_FOUND TRUE)
    message(STATUS "  [OK] audio: ${CLIENT_audio_LIB}")
else()
    set(LX_CLIENT_audio_FOUND FALSE)
    message(WARNING "  [MISSING] audio: client_audio not found")
endif()
# ----------------------------------------
# Namespace: obstacle_avoidance
# ----------------------------------------

# 查找库文件
find_library(CLIENT_obstacle_avoidance_LIB 
    NAMES client_obstacle_avoidance
    PATHS ${LX_CLIENT_LIB_DIR}
    NO_DEFAULT_PATH
)
# 检查库是否找到
if(CLIENT_obstacle_avoidance_LIB)
    # 创建导入目标
    if(NOT TARGET client_obstacle_avoidance)
        add_library(client_obstacle_avoidance SHARED IMPORTED)
        set_target_properties(client_obstacle_avoidance PROPERTIES
            IMPORTED_LOCATION "${CLIENT_obstacle_avoidance_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "Threads::Threads"
        )
    endif()

    if(NOT TARGET client_internal_obstacle_avoidance)
        add_library(client_internal_obstacle_avoidance SHARED IMPORTED)
        set_target_properties(client_internal_obstacle_avoidance PROPERTIES
            IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
        )
    endif()

    # 创建统一的接口库
    if(NOT TARGET lx_client_obstacle_avoidance)
        add_library(lx_client_obstacle_avoidance INTERFACE)
        target_link_libraries(lx_client_obstacle_avoidance INTERFACE
            client_obstacle_avoidance
            client_internal_obstacle_avoidance
        )
        target_include_directories(lx_client_obstacle_avoidance INTERFACE
            ${LX_CLIENT_INCLUDE_DIR}
        )
    endif()

    set(LX_CLIENT_obstacle_avoidance_FOUND TRUE)
    message(STATUS "  [OK] obstacle_avoidance: ${CLIENT_obstacle_avoidance_LIB}")
else()
    set(LX_CLIENT_obstacle_avoidance_FOUND FALSE)
    message(WARNING "  [MISSING] obstacle_avoidance: client_obstacle_avoidance not found")
endif()
# ----------------------------------------
# Namespace: sensor
# ----------------------------------------

# 查找库文件
find_library(CLIENT_sensor_LIB 
    NAMES client_sensor
    PATHS ${LX_CLIENT_LIB_DIR}
    NO_DEFAULT_PATH
)
# 检查库是否找到
if(CLIENT_sensor_LIB)
    # 创建导入目标
    if(NOT TARGET client_sensor)
        add_library(client_sensor SHARED IMPORTED)
        set_target_properties(client_sensor PROPERTIES
            IMPORTED_LOCATION "${CLIENT_sensor_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "Threads::Threads"
        )
    endif()

    if(NOT TARGET client_internal_sensor)
        add_library(client_internal_sensor SHARED IMPORTED)
        set_target_properties(client_internal_sensor PROPERTIES
            IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
        )
    endif()

    # 创建统一的接口库
    if(NOT TARGET lx_client_sensor)
        add_library(lx_client_sensor INTERFACE)
        target_link_libraries(lx_client_sensor INTERFACE
            client_sensor
            client_internal_sensor
        )
        target_include_directories(lx_client_sensor INTERFACE
            ${LX_CLIENT_INCLUDE_DIR}
        )
    endif()

    set(LX_CLIENT_sensor_FOUND TRUE)
    message(STATUS "  [OK] sensor: ${CLIENT_sensor_LIB}")
else()
    set(LX_CLIENT_sensor_FOUND FALSE)
    message(WARNING "  [MISSING] sensor: client_sensor not found")
endif()
# ----------------------------------------
# Namespace: localization
# ----------------------------------------

# 查找库文件
find_library(CLIENT_localization_LIB 
    NAMES client_localization
    PATHS ${LX_CLIENT_LIB_DIR}
    NO_DEFAULT_PATH
)
# 检查库是否找到
if(CLIENT_localization_LIB)
    # 创建导入目标
    if(NOT TARGET client_localization)
        add_library(client_localization SHARED IMPORTED)
        set_target_properties(client_localization PROPERTIES
            IMPORTED_LOCATION "${CLIENT_localization_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "Threads::Threads"
        )
    endif()

    if(NOT TARGET client_internal_localization)
        add_library(client_internal_localization SHARED IMPORTED)
        set_target_properties(client_internal_localization PROPERTIES
            IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
        )
    endif()

    # 创建统一的接口库
    if(NOT TARGET lx_client_localization)
        add_library(lx_client_localization INTERFACE)
        target_link_libraries(lx_client_localization INTERFACE
            client_localization
            client_internal_localization
        )
        target_include_directories(lx_client_localization INTERFACE
            ${LX_CLIENT_INCLUDE_DIR}
        )
    endif()

    set(LX_CLIENT_localization_FOUND TRUE)
    message(STATUS "  [OK] localization: ${CLIENT_localization_LIB}")
else()
    set(LX_CLIENT_localization_FOUND FALSE)
    message(WARNING "  [MISSING] localization: client_localization not found")
endif()
# ----------------------------------------
# Namespace: robot_task
# ----------------------------------------

# 查找库文件
find_library(CLIENT_robot_task_LIB 
    NAMES client_robot_task
    PATHS ${LX_CLIENT_LIB_DIR}
    NO_DEFAULT_PATH
)
find_library(CMD_PROMISE_MANAGER_robot_task_LIB 
    NAMES cmd_promise_manager
    PATHS ${LX_CLIENT_LIB_DIR}
    NO_DEFAULT_PATH
)

# 检查库是否找到
if(CLIENT_robot_task_LIB)
    # 创建导入目标
    if(NOT TARGET client_robot_task)
        add_library(client_robot_task SHARED IMPORTED)
        set_target_properties(client_robot_task PROPERTIES
            IMPORTED_LOCATION "${CLIENT_robot_task_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "Threads::Threads"
        )
    endif()

    if(NOT TARGET client_internal_robot_task)
        add_library(client_internal_robot_task SHARED IMPORTED)
        set_target_properties(client_internal_robot_task PROPERTIES
            IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
        )
    endif()
    if(CMD_PROMISE_MANAGER_robot_task_LIB AND NOT TARGET cmd_promise_manager_robot_task)
        add_library(cmd_promise_manager_robot_task SHARED IMPORTED)
        set_target_properties(cmd_promise_manager_robot_task PROPERTIES
            IMPORTED_LOCATION "${CMD_PROMISE_MANAGER_robot_task_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
        )
    endif()

    # 创建统一的接口库
    if(NOT TARGET lx_client_robot_task)
        add_library(lx_client_robot_task INTERFACE)
        target_link_libraries(lx_client_robot_task INTERFACE
            client_robot_task
            client_internal_robot_task
            $<$<TARGET_EXISTS:cmd_promise_manager_robot_task>:cmd_promise_manager_robot_task>
        )
        target_include_directories(lx_client_robot_task INTERFACE
            ${LX_CLIENT_INCLUDE_DIR}
        )
    endif()

    set(LX_CLIENT_robot_task_FOUND TRUE)
    message(STATUS "  [OK] robot_task: ${CLIENT_robot_task_LIB}")
else()
    set(LX_CLIENT_robot_task_FOUND FALSE)
    message(WARNING "  [MISSING] robot_task: client_robot_task not found")
endif()
# ----------------------------------------
# Namespace: light
# ----------------------------------------

# 查找库文件
find_library(CLIENT_light_LIB 
    NAMES client_light
    PATHS ${LX_CLIENT_LIB_DIR}
    NO_DEFAULT_PATH
)
# 检查库是否找到
if(CLIENT_light_LIB)
    # 创建导入目标
    if(NOT TARGET client_light)
        add_library(client_light SHARED IMPORTED)
        set_target_properties(client_light PROPERTIES
            IMPORTED_LOCATION "${CLIENT_light_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "Threads::Threads"
        )
    endif()

    if(NOT TARGET client_internal_light)
        add_library(client_internal_light SHARED IMPORTED)
        set_target_properties(client_internal_light PROPERTIES
            IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
        )
    endif()

    # 创建统一的接口库
    if(NOT TARGET lx_client_light)
        add_library(lx_client_light INTERFACE)
        target_link_libraries(lx_client_light INTERFACE
            client_light
            client_internal_light
        )
        target_include_directories(lx_client_light INTERFACE
            ${LX_CLIENT_INCLUDE_DIR}
        )
    endif()

    set(LX_CLIENT_light_FOUND TRUE)
    message(STATUS "  [OK] light: ${CLIENT_light_LIB}")
else()
    set(LX_CLIENT_light_FOUND FALSE)
    message(WARNING "  [MISSING] light: client_light not found")
endif()
# ----------------------------------------
# Namespace: lift
# ----------------------------------------

# 查找库文件
find_library(CLIENT_lift_LIB 
    NAMES client_lift
    PATHS ${LX_CLIENT_LIB_DIR}
    NO_DEFAULT_PATH
)
find_library(CMD_PROMISE_MANAGER_lift_LIB 
    NAMES cmd_promise_manager
    PATHS ${LX_CLIENT_LIB_DIR}
    NO_DEFAULT_PATH
)

# 检查库是否找到
if(CLIENT_lift_LIB)
    # 创建导入目标
    if(NOT TARGET client_lift)
        add_library(client_lift SHARED IMPORTED)
        set_target_properties(client_lift PROPERTIES
            IMPORTED_LOCATION "${CLIENT_lift_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "Threads::Threads"
        )
    endif()

    if(NOT TARGET client_internal_lift)
        add_library(client_internal_lift SHARED IMPORTED)
        set_target_properties(client_internal_lift PROPERTIES
            IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
        )
    endif()
    if(CMD_PROMISE_MANAGER_lift_LIB AND NOT TARGET cmd_promise_manager_lift)
        add_library(cmd_promise_manager_lift SHARED IMPORTED)
        set_target_properties(cmd_promise_manager_lift PROPERTIES
            IMPORTED_LOCATION "${CMD_PROMISE_MANAGER_lift_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
        )
    endif()

    # 创建统一的接口库
    if(NOT TARGET lx_client_lift)
        add_library(lx_client_lift INTERFACE)
        target_link_libraries(lx_client_lift INTERFACE
            client_lift
            client_internal_lift
            $<$<TARGET_EXISTS:cmd_promise_manager_lift>:cmd_promise_manager_lift>
        )
        target_include_directories(lx_client_lift INTERFACE
            ${LX_CLIENT_INCLUDE_DIR}
        )
    endif()

    set(LX_CLIENT_lift_FOUND TRUE)
    message(STATUS "  [OK] lift: ${CLIENT_lift_LIB}")
else()
    set(LX_CLIENT_lift_FOUND FALSE)
    message(WARNING "  [MISSING] lift: client_lift not found")
endif()
# ----------------------------------------
# Namespace: odometry
# ----------------------------------------

# 查找库文件
find_library(CLIENT_odometry_LIB 
    NAMES client_odometry
    PATHS ${LX_CLIENT_LIB_DIR}
    NO_DEFAULT_PATH
)
# 检查库是否找到
if(CLIENT_odometry_LIB)
    # 创建导入目标
    if(NOT TARGET client_odometry)
        add_library(client_odometry SHARED IMPORTED)
        set_target_properties(client_odometry PROPERTIES
            IMPORTED_LOCATION "${CLIENT_odometry_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "Threads::Threads"
        )
    endif()

    if(NOT TARGET client_internal_odometry)
        add_library(client_internal_odometry SHARED IMPORTED)
        set_target_properties(client_internal_odometry PROPERTIES
            IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
        )
    endif()

    # 创建统一的接口库
    if(NOT TARGET lx_client_odometry)
        add_library(lx_client_odometry INTERFACE)
        target_link_libraries(lx_client_odometry INTERFACE
            client_odometry
            client_internal_odometry
        )
        target_include_directories(lx_client_odometry INTERFACE
            ${LX_CLIENT_INCLUDE_DIR}
        )
    endif()

    set(LX_CLIENT_odometry_FOUND TRUE)
    message(STATUS "  [OK] odometry: ${CLIENT_odometry_LIB}")
else()
    set(LX_CLIENT_odometry_FOUND FALSE)
    message(WARNING "  [MISSING] odometry: client_odometry not found")
endif()
# ----------------------------------------
# Namespace: battery
# ----------------------------------------

# 查找库文件
find_library(CLIENT_battery_LIB 
    NAMES client_battery
    PATHS ${LX_CLIENT_LIB_DIR}
    NO_DEFAULT_PATH
)
# 检查库是否找到
if(CLIENT_battery_LIB)
    # 创建导入目标
    if(NOT TARGET client_battery)
        add_library(client_battery SHARED IMPORTED)
        set_target_properties(client_battery PROPERTIES
            IMPORTED_LOCATION "${CLIENT_battery_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "Threads::Threads"
        )
    endif()

    if(NOT TARGET client_internal_battery)
        add_library(client_internal_battery SHARED IMPORTED)
        set_target_properties(client_internal_battery PROPERTIES
            IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
        )
    endif()

    # 创建统一的接口库
    if(NOT TARGET lx_client_battery)
        add_library(lx_client_battery INTERFACE)
        target_link_libraries(lx_client_battery INTERFACE
            client_battery
            client_internal_battery
        )
        target_include_directories(lx_client_battery INTERFACE
            ${LX_CLIENT_INCLUDE_DIR}
        )
    endif()

    set(LX_CLIENT_battery_FOUND TRUE)
    message(STATUS "  [OK] battery: ${CLIENT_battery_LIB}")
else()
    set(LX_CLIENT_battery_FOUND FALSE)
    message(WARNING "  [MISSING] battery: client_battery not found")
endif()
# ----------------------------------------
# Namespace: qrcamera
# ----------------------------------------

# 查找库文件
find_library(CLIENT_qrcamera_LIB 
    NAMES client_qrcamera
    PATHS ${LX_CLIENT_LIB_DIR}
    NO_DEFAULT_PATH
)
# 检查库是否找到
if(CLIENT_qrcamera_LIB)
    # 创建导入目标
    if(NOT TARGET client_qrcamera)
        add_library(client_qrcamera SHARED IMPORTED)
        set_target_properties(client_qrcamera PROPERTIES
            IMPORTED_LOCATION "${CLIENT_qrcamera_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "Threads::Threads"
        )
    endif()

    if(NOT TARGET client_internal_qrcamera)
        add_library(client_internal_qrcamera SHARED IMPORTED)
        set_target_properties(client_internal_qrcamera PROPERTIES
            IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
        )
    endif()

    # 创建统一的接口库
    if(NOT TARGET lx_client_qrcamera)
        add_library(lx_client_qrcamera INTERFACE)
        target_link_libraries(lx_client_qrcamera INTERFACE
            client_qrcamera
            client_internal_qrcamera
        )
        target_include_directories(lx_client_qrcamera INTERFACE
            ${LX_CLIENT_INCLUDE_DIR}
        )
    endif()

    set(LX_CLIENT_qrcamera_FOUND TRUE)
    message(STATUS "  [OK] qrcamera: ${CLIENT_qrcamera_LIB}")
else()
    set(LX_CLIENT_qrcamera_FOUND FALSE)
    message(WARNING "  [MISSING] qrcamera: client_qrcamera not found")
endif()
# ----------------------------------------
# Namespace: rpc_test
# ----------------------------------------

# 查找库文件
find_library(CLIENT_rpc_test_LIB 
    NAMES client_rpc_test
    PATHS ${LX_CLIENT_LIB_DIR}
    NO_DEFAULT_PATH
)
find_library(CMD_PROMISE_MANAGER_rpc_test_LIB 
    NAMES cmd_promise_manager
    PATHS ${LX_CLIENT_LIB_DIR}
    NO_DEFAULT_PATH
)

# 检查库是否找到
if(CLIENT_rpc_test_LIB)
    # 创建导入目标
    if(NOT TARGET client_rpc_test)
        add_library(client_rpc_test SHARED IMPORTED)
        set_target_properties(client_rpc_test PROPERTIES
            IMPORTED_LOCATION "${CLIENT_rpc_test_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "Threads::Threads"
        )
    endif()

    if(NOT TARGET client_internal_rpc_test)
        add_library(client_internal_rpc_test SHARED IMPORTED)
        set_target_properties(client_internal_rpc_test PROPERTIES
            IMPORTED_LOCATION "${LX_CLIENT_LIB_DIR}/libclient_internal.so"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
        )
    endif()
    if(CMD_PROMISE_MANAGER_rpc_test_LIB AND NOT TARGET cmd_promise_manager_rpc_test)
        add_library(cmd_promise_manager_rpc_test SHARED IMPORTED)
        set_target_properties(cmd_promise_manager_rpc_test PROPERTIES
            IMPORTED_LOCATION "${CMD_PROMISE_MANAGER_rpc_test_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${LX_CLIENT_INCLUDE_DIR}"
        )
    endif()

    # 创建统一的接口库
    if(NOT TARGET lx_client_rpc_test)
        add_library(lx_client_rpc_test INTERFACE)
        target_link_libraries(lx_client_rpc_test INTERFACE
            client_rpc_test
            client_internal_rpc_test
            $<$<TARGET_EXISTS:cmd_promise_manager_rpc_test>:cmd_promise_manager_rpc_test>
        )
        target_include_directories(lx_client_rpc_test INTERFACE
            ${LX_CLIENT_INCLUDE_DIR}
        )
    endif()

    set(LX_CLIENT_rpc_test_FOUND TRUE)
    message(STATUS "  [OK] rpc_test: ${CLIENT_rpc_test_LIB}")
else()
    set(LX_CLIENT_rpc_test_FOUND FALSE)
    message(WARNING "  [MISSING] rpc_test: client_rpc_test not found")
endif()

# ============================================
# 辅助函数：链接所有 lx_client 库
# ============================================
function(lx_client_link_all target)
    if(TARGET lx_client_laser_2d)
        target_link_libraries(${target} lx_client_laser_2d)
    endif()
    if(TARGET lx_client_map_reader)
        target_link_libraries(${target} lx_client_map_reader)
    endif()
    if(TARGET lx_client_bridge_scan)
        target_link_libraries(${target} lx_client_bridge_scan)
    endif()
    if(TARGET lx_client_depth_semantic)
        target_link_libraries(${target} lx_client_depth_semantic)
    endif()
    if(TARGET lx_client_io_manager)
        target_link_libraries(${target} lx_client_io_manager)
    endif()
    if(TARGET lx_client_bridge_drive)
        target_link_libraries(${target} lx_client_bridge_drive)
    endif()
    if(TARGET lx_client_imu)
        target_link_libraries(${target} lx_client_imu)
    endif()
    if(TARGET lx_client_exception)
        target_link_libraries(${target} lx_client_exception)
    endif()
    if(TARGET lx_client_nav)
        target_link_libraries(${target} lx_client_nav)
    endif()
    if(TARGET lx_client_point_cloud)
        target_link_libraries(${target} lx_client_point_cloud)
    endif()
    if(TARGET lx_client_audio)
        target_link_libraries(${target} lx_client_audio)
    endif()
    if(TARGET lx_client_obstacle_avoidance)
        target_link_libraries(${target} lx_client_obstacle_avoidance)
    endif()
    if(TARGET lx_client_sensor)
        target_link_libraries(${target} lx_client_sensor)
    endif()
    if(TARGET lx_client_localization)
        target_link_libraries(${target} lx_client_localization)
    endif()
    if(TARGET lx_client_robot_task)
        target_link_libraries(${target} lx_client_robot_task)
    endif()
    if(TARGET lx_client_light)
        target_link_libraries(${target} lx_client_light)
    endif()
    if(TARGET lx_client_lift)
        target_link_libraries(${target} lx_client_lift)
    endif()
    if(TARGET lx_client_odometry)
        target_link_libraries(${target} lx_client_odometry)
    endif()
    if(TARGET lx_client_battery)
        target_link_libraries(${target} lx_client_battery)
    endif()
    if(TARGET lx_client_qrcamera)
        target_link_libraries(${target} lx_client_qrcamera)
    endif()
    if(TARGET lx_client_rpc_test)
        target_link_libraries(${target} lx_client_rpc_test)
    endif()
endfunction()

message(STATUS "LX Client Libraries loaded from: ${LX_CLIENT_ROOT}")