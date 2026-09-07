cmake_minimum_required(VERSION 3.10)
project(T230cmake1)

set(CMAKE_CXX_STANDARD 17)

# 工程根目录（main 内解析配置路径）
add_compile_definitions(MARKET_SIMPLE_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

# 启用 pthread 并查找线程库
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# 查找 X11 库
find_package(X11 REQUIRED)

# Lanxin VMR SDK（lanxin_control 整合）
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# feeding_cylindrical_parts_alg 依赖的 conda 环境
set(HUMAN_INTERACTION_ENV_ROOT "/home/ti5robot/anaconda3/envs/human_interaction_env" CACHE PATH
    "conda env: human_interaction_env")
set(Python3_EXECUTABLE "${HUMAN_INTERACTION_ENV_ROOT}/bin/python")
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 CONFIG REQUIRED)
message(STATUS "seg_pose Python: ${Python3_EXECUTABLE} (${Python3_VERSION})")

# DEX1 夹爪（工程内自包含，不依赖 ~/dex1_1_service）
set(DEX1_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/third_party/dex1/include")
if(NOT EXISTS "${DEX1_INCLUDE_DIR}/serialPort/SerialPort.h")
    message(FATAL_ERROR
        "缺少夹爪 SDK 头文件: ${DEX1_INCLUDE_DIR}\n"
        "请确认 third_party/dex1/include 已随工程一起拷贝")
endif()
include_directories(${DEX1_INCLUDE_DIR})

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(UNITREE_MOTOR_LIB "${PROJECT_SOURCE_DIR}/lib/libUnitreeMotorSDK_Arm64.so")
else()
    set(UNITREE_MOTOR_LIB "${PROJECT_SOURCE_DIR}/lib/libUnitreeMotorSDK_Linux64.so")
endif()
if(NOT EXISTS "${UNITREE_MOTOR_LIB}")
    message(FATAL_ERROR "缺少夹爪动态库: ${UNITREE_MOTOR_LIB}")
endif()

find_library(SERIALPORT_LIB serialport
    PATHS /lib/aarch64-linux-gnu /usr/lib/aarch64-linux-gnu /usr/local/lib
)
if(NOT SERIALPORT_LIB)
    message(WARNING "系统未安装 libserialport，新机器请运行: sudo dpkg -i third_party/debs/libserialport*.deb")
endif()

# 解算器 SDK（include/Ti5_Arm.h + lib/Ti5_Arm_2204.so）
set(TI5_ARM_SOLVER_LIB "${PROJECT_SOURCE_DIR}/lib/Ti5_Arm_2204.so")
if(NOT EXISTS "${TI5_ARM_SOLVER_LIB}")
    message(FATAL_ERROR "缺少解算器 SDK 动态库: ${TI5_ARM_SOLVER_LIB}")
endif()

# VMR_AMR_SDK（已整合到 sdk/include + sdk/lib，libvmr_sdk.so 仍用于 VMR 旋转/点导航）
set(VMR_AMR_SDK_ROOT "${PROJECT_SOURCE_DIR}/VMR_AMR_SDK")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(VMR_SDK_ARCH "arm64")
else()
    set(VMR_SDK_ARCH "x86")
endif()
if(NOT VMR_SDK_UBUNTU_VERSION)
    if(EXISTS "/etc/os-release")
        file(READ "/etc/os-release" OS_RELEASE_CONTENT)
        if(OS_RELEASE_CONTENT MATCHES "VERSION_ID=\"([0-9]+\\.[0-9]+)\"")
            set(VMR_SDK_UBUNTU_VERSION "${CMAKE_MATCH_1}")
        endif()
    endif()
endif()
set(VMR_SDK_LIB_CANDIDATE "${VMR_AMR_SDK_ROOT}/lib/${VMR_SDK_ARCH}/${VMR_SDK_UBUNTU_VERSION}")
if(EXISTS "${VMR_SDK_LIB_CANDIDATE}/libclient_internal.so")
    set(VMR_AMR_SDK_LIB_DIR "${VMR_SDK_LIB_CANDIDATE}")
elseif(EXISTS "${VMR_AMR_SDK_ROOT}/lib/${VMR_SDK_ARCH}/libclient_internal.so")
    set(VMR_AMR_SDK_LIB_DIR "${VMR_AMR_SDK_ROOT}/lib/${VMR_SDK_ARCH}")
else()
    set(VMR_AMR_SDK_LIB_DIR "${PROJECT_SOURCE_DIR}/sdk/lib")
endif()
message(STATUS "VMR_AMR_SDK lib dir: ${VMR_AMR_SDK_LIB_DIR}")

# 更明确的头文件包含方式
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/eigen3
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/sdk/include
    ${VMR_AMR_SDK_ROOT}/include
    /usr/local/include
    ${X11_INCLUDE_DIR}  # 添加 X11 头文件路径
)

# 更可靠的源文件收集方式
set(SRC_FILES 
    ${PROJECT_SOURCE_DIR}/src/aoyihand.cpp
    ${PROJECT_SOURCE_DIR}/src/function.cpp
    ${PROJECT_SOURCE_DIR}/src/arm_robot.cpp
    ${PROJECT_SOURCE_DIR}/src/waist.cpp
    ${PROJECT_SOURCE_DIR}/src/main3.cpp
    # ${PROJECT_SOURCE_DIR}/src/mainbiaoding.cpp
    ${PROJECT_SOURCE_DIR}/src/motor_driver.cpp
    ${PROJECT_SOURCE_DIR}/src/reals_tcp.cpp
    # ${PROJECT_SOURCE_DIR}/src/speech_recognizer.cpp
    ${PROJECT_SOURCE_DIR}/src/Ti5_socketcan.cpp
    ${PROJECT_SOURCE_DIR}/src/wt_client.cpp
    ${PROJECT_SOURCE_DIR}/src/wt_box_tcp.cpp
    ${PROJECT_SOURCE_DIR}/src/lanxincontrol.cpp
    ${PROJECT_SOURCE_DIR}/src/gripper_interface.cpp
    ${PROJECT_SOURCE_DIR}/src/tts_http_client.cpp
    ${PROJECT_SOURCE_DIR}/src/realsense_get.cpp
    ${PROJECT_SOURCE_DIR}/src/seg_pose_bridge.cpp
    ${PROJECT_SOURCE_DIR}/src/move_box_config.cpp
    ${PROJECT_SOURCE_DIR}/src/move_box_runtime.cpp
    ${PROJECT_SOURCE_DIR}/src/grasp_to_standby.cpp
    ${PROJECT_SOURCE_DIR}/src/place_grid_correct.cpp
)

# 可执行文件
add_executable(move ${SRC_FILES})

# 腰 TCP 直线 demo（不依赖相机/Python/底盘）
add_executable(waist_demo
    ${PROJECT_SOURCE_DIR}/src/waist_demo.cpp
    ${PROJECT_SOURCE_DIR}/src/waist.cpp
    ${PROJECT_SOURCE_DIR}/src/Ti5_socketcan.cpp
    ${PROJECT_SOURCE_DIR}/src/function.cpp
)

# 库路径
link_directories(${PROJECT_SOURCE_DIR}/lib /usr/local/lib ${PROJECT_SOURCE_DIR}/sdk/lib)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)
pkg_check_modules(REALSENSE2 realsense2)

if(NOT REALSENSE2_FOUND)
    set(REALSENSE2_INCLUDE_DIRS /usr/local/include)
    set(REALSENSE2_LIBRARIES realsense2)
    link_directories(/usr/local/lib)
else()
    include_directories(${REALSENSE2_INCLUDE_DIRS})
endif()

include_directories(${JSONCPP_INCLUDE_DIRS})

# 查找 serial 库（手动查找，因为 pkg-config 找不到）
find_library(SERIAL_LIB serial
    PATHS
    /usr/local/lib
    /usr/lib
    /usr/lib/aarch64-linux-gnu
    NO_DEFAULT_PATH
)

if(SERIAL_LIB)
    message(STATUS "Found serial library: ${SERIAL_LIB}")
else()
    message(WARNING "serial library not found, trying alternative names...")
    # 尝试其他可能的库名
    find_library(SERIAL_LIB libserial.a
        PATHS
        /usr/local/lib
        /usr/lib
        /usr/lib/aarch64-linux-gnu
    )
    if(SERIAL_LIB)
        message(STATUS "Found serial library as: ${SERIAL_LIB}")
    else()
        message(FATAL_ERROR "serial library not found. Please install libserial.")
    endif()
endif()

# Lanxin SDK：RPC(robot_task) + VMR(旋转/点导航)
set(LANXIN_SDK_LIBS
    "${PROJECT_SOURCE_DIR}/sdk/lib/libvmr_sdk.so"
    "${VMR_AMR_SDK_LIB_DIR}/libclient_robot_task.so"
    "${VMR_AMR_SDK_LIB_DIR}/libclient_internal.so"
    "${VMR_AMR_SDK_LIB_DIR}/libcmd_promise_manager.so"
)

# 调整库链接顺序
target_link_libraries(move
    controlcan
    ${TI5_ARM_SOLVER_LIB}
    ${SERIAL_LIB}  # 使用找到的 serial 库
    portaudio
    # vosk
    Threads::Threads
    ${LIBUSB_LIBRARIES}
    ${CMAKE_DL_LIBS}
    ${X11_LIBRARIES}  # 添加 X11 库链接
    ${JSONCPP_LIBRARIES}
    ${LANXIN_SDK_LIBS}
    Boost::system
    Boost::filesystem
    ${OpenCV_LIBS}
    yaml-cpp
    ${REALSENSE2_LIBRARIES}
    pybind11::embed
    Python3::Python
)

if(EXISTS "${UNITREE_MOTOR_LIB}" AND SERIALPORT_LIB)
    target_link_libraries(move ${UNITREE_MOTOR_LIB} ${SERIALPORT_LIB})
    message(STATUS "DEX1 gripper: ${UNITREE_MOTOR_LIB}, ${SERIALPORT_LIB}")
else()
    message(WARNING "夹爪库未完整: motor=${UNITREE_MOTOR_LIB}, serialport=${SERIALPORT_LIB}")
endif()

# 运行时加载 sdk/lib（libvmr_sdk）+ VMR_AMR_SDK RPC 库
set(LANXIN_SDK_LIB_DIR "${PROJECT_SOURCE_DIR}/sdk/lib")
set(CONDA_ENV_LIB_DIR "${HUMAN_INTERACTION_ENV_ROOT}/lib")
set(THIRD_PARTY_RPATH_DIRS "${LANXIN_SDK_LIB_DIR}:${VMR_AMR_SDK_LIB_DIR}:${PROJECT_SOURCE_DIR}/lib:/usr/local/lib:${CONDA_ENV_LIB_DIR}")
set_target_properties(move PROPERTIES
    BUILD_RPATH "${THIRD_PARTY_RPATH_DIRS}"
    INSTALL_RPATH "${THIRD_PARTY_RPATH_DIRS}"
    BUILD_RPATH_USE_ORIGIN ON
)
target_link_options(move PRIVATE "-Wl,--disable-new-dtags")
target_compile_definitions(move PRIVATE SEG_POSE_CONDA_PREFIX="${HUMAN_INTERACTION_ENV_ROOT}")

target_link_libraries(waist_demo
    ${TI5_ARM_SOLVER_LIB}
    Threads::Threads
)
set_target_properties(waist_demo PROPERTIES
    BUILD_RPATH "${PROJECT_SOURCE_DIR}/lib"
    INSTALL_RPATH "${PROJECT_SOURCE_DIR}/lib"
    BUILD_RPATH_USE_ORIGIN ON
)
target_link_options(waist_demo PRIVATE "-Wl,--disable-new-dtags")

# 编排台硬件守护进程：上身动作 + 视觉抓取，不链底盘 SDK
set(ORCH_HW_SOURCES
    ${PROJECT_SOURCE_DIR}/orchestrator/cpp/orch_hw_main.cpp
    ${PROJECT_SOURCE_DIR}/orchestrator/cpp/robot_runtime.cpp
    ${PROJECT_SOURCE_DIR}/src/grasp_to_standby.cpp
    ${PROJECT_SOURCE_DIR}/src/aoyihand.cpp
    ${PROJECT_SOURCE_DIR}/src/function.cpp
    ${PROJECT_SOURCE_DIR}/src/arm_robot.cpp
    ${PROJECT_SOURCE_DIR}/src/waist.cpp
    ${PROJECT_SOURCE_DIR}/src/motor_driver.cpp
    ${PROJECT_SOURCE_DIR}/src/reals_tcp.cpp
    ${PROJECT_SOURCE_DIR}/src/Ti5_socketcan.cpp
    ${PROJECT_SOURCE_DIR}/src/gripper_interface.cpp
    ${PROJECT_SOURCE_DIR}/src/realsense_get.cpp
    ${PROJECT_SOURCE_DIR}/src/seg_pose_bridge.cpp
    ${PROJECT_SOURCE_DIR}/src/move_box_config.cpp
    ${PROJECT_SOURCE_DIR}/src/move_box_runtime.cpp
    ${PROJECT_SOURCE_DIR}/src/place_grid_correct.cpp
)
add_executable(orch_hw ${ORCH_HW_SOURCES})
target_include_directories(orch_hw PRIVATE ${PROJECT_SOURCE_DIR}/orchestrator/cpp)
target_compile_definitions(orch_hw PRIVATE
    ORCH_NO_CHASSIS
    SEG_POSE_CONDA_PREFIX="${HUMAN_INTERACTION_ENV_ROOT}"
)
target_link_libraries(orch_hw
    controlcan
    ${TI5_ARM_SOLVER_LIB}
    ${SERIAL_LIB}
    Threads::Threads
    ${LIBUSB_LIBRARIES}
    ${CMAKE_DL_LIBS}
    ${X11_LIBRARIES}
    ${JSONCPP_LIBRARIES}
    Boost::system
    Boost::filesystem
    ${OpenCV_LIBS}
    yaml-cpp
    ${REALSENSE2_LIBRARIES}
    pybind11::embed
    Python3::Python
)
if(EXISTS "${UNITREE_MOTOR_LIB}" AND SERIALPORT_LIB)
    target_link_libraries(orch_hw ${UNITREE_MOTOR_LIB} ${SERIALPORT_LIB})
endif()
set_target_properties(orch_hw PROPERTIES
    BUILD_RPATH "${PROJECT_SOURCE_DIR}/lib:/usr/local/lib:${CONDA_ENV_LIB_DIR}"
    INSTALL_RPATH "${PROJECT_SOURCE_DIR}/lib:/usr/local/lib:${CONDA_ENV_LIB_DIR}"
    BUILD_RPATH_USE_ORIGIN ON
)
target_link_options(orch_hw PRIVATE "-Wl,--disable-new-dtags")
