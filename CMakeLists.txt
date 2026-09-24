cmake_minimum_required(VERSION 3.16)
project(t170c_cylinder_debug LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_definitions(MARKET_SIMPLE_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

set(HUMAN_INTERACTION_ENV_ROOT "/home/ti5robot/anaconda3/envs/human_interaction_env" CACHE PATH
    "Conda environment containing ultralytics, OpenCV and pybind11")
set(Python3_EXECUTABLE "${HUMAN_INTERACTION_ENV_ROOT}/bin/python")
list(APPEND CMAKE_PREFIX_PATH "${HUMAN_INTERACTION_ENV_ROOT}")

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 CONFIG REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)
pkg_check_modules(REALSENSE2 realsense2)

if(NOT REALSENSE2_FOUND)
    set(REALSENSE2_INCLUDE_DIRS /usr/local/include)
    set(REALSENSE2_LIBRARIES realsense2)
endif()

find_library(SERIALPORT_LIB serialport PATHS /lib/aarch64-linux-gnu /usr/lib/aarch64-linux-gnu /usr/local/lib)
if(NOT SERIALPORT_LIB AND EXISTS "${PROJECT_SOURCE_DIR}/third_party/libserialport/lib/libserialport.so.0.1.0")
    set(SERIALPORT_LIB "${PROJECT_SOURCE_DIR}/third_party/libserialport/lib/libserialport.so.0.1.0")
endif()
if(NOT SERIALPORT_LIB)
    message(FATAL_ERROR "libserialport not found")
endif()

set(TI5_DEVICE_SDK_ROOT "/opt/ti5-device-sdk")
set(TI5_MOTOR_CODEC_ROOT "/opt/ti5-motor-codec")
set(TI5_SERIAL_ROOT "/opt/ti5-serial")
set(TI5_CAN_MINI_ROOT "/opt/ti5-can-mini")
set(TI5_ROHAND_CODEC_ROOT "/opt/ti5-rohand-codec")
set(TI5_DEX1_CODEC_ROOT "/opt/ti5-dex1-codec")
set(TI5_AGV_SEER_ROOT "/opt/ti5-agv-seer")
set(TI5_ARM_SOLVER_LIB "${PROJECT_SOURCE_DIR}/lib/Ti5_Arm_2204.so")
set(TI5_DEVICE_SDK_LIB "${TI5_DEVICE_SDK_ROOT}/lib/libcommunication_manager.so")
set(TI5_MOTOR_CODEC_LIB "${TI5_MOTOR_CODEC_ROOT}/lib/libti5_motor.so")
set(TI5_SERIAL_SDK_LIB "${TI5_SERIAL_ROOT}/lib/libserial_port.so")
set(TI5_ROHAND_CODEC_LIB "${TI5_ROHAND_CODEC_ROOT}/lib/librohand_codec.so")
set(TI5_DEX1_CODEC_LIB "${TI5_DEX1_CODEC_ROOT}/lib/libTi5_Dex1_codec.so")
set(TI5_AGV_SEER_LIB "${TI5_AGV_SEER_ROOT}/lib/libseer.so")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(UNITREE_MOTOR_LIB "${PROJECT_SOURCE_DIR}/lib/libUnitreeMotorSDK_Arm64.so")
else()
    set(UNITREE_MOTOR_LIB "${PROJECT_SOURCE_DIR}/lib/libUnitreeMotorSDK_Linux64.so")
endif()
foreach(required_lib IN ITEMS
        "${TI5_ARM_SOLVER_LIB}"
        "${UNITREE_MOTOR_LIB}"
        "${TI5_DEVICE_SDK_LIB}"
        "${TI5_MOTOR_CODEC_LIB}"
        "${TI5_SERIAL_SDK_LIB}"
        "${TI5_ROHAND_CODEC_LIB}"
        "${TI5_DEX1_CODEC_LIB}"
        "${TI5_AGV_SEER_LIB}")
    if(NOT EXISTS "${required_lib}")
        message(FATAL_ERROR "Required robot library not found: ${required_lib}")
    endif()
endforeach()

if(NOT TARGET Eigen3::Eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    target_include_directories(Eigen3::Eigen INTERFACE "${PROJECT_SOURCE_DIR}/eigen3")
endif()
add_subdirectory(LowerBody)

add_executable(t170c_debug
    orchestrator/cpp/orch_hw_main.cpp
    orchestrator/cpp/robot_runtime.cpp
    src/grasp_to_standby.cpp
    src/function.cpp
    src/waist.cpp
    src/Ti5_socketcan.cpp
    src/gripper_interface.cpp
    src/realsense_get.cpp
    src/seg_pose_bridge.cpp
    src/move_box_config.cpp
    src/move_box_runtime.cpp
    src/head_cam2robot.cpp
    src/head_control.cpp
)

target_include_directories(t170c_debug PRIVATE
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/eigen3
    ${PROJECT_SOURCE_DIR}/third_party/dex1/include
    ${TI5_DEVICE_SDK_ROOT}/include
    ${TI5_AGV_SEER_ROOT}/include
    ${JSONCPP_INCLUDE_DIRS}
    ${REALSENSE2_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/third_party/libserialport/include
)

target_compile_definitions(t170c_debug PRIVATE
    SEG_POSE_CONDA_PREFIX="${HUMAN_INTERACTION_ENV_ROOT}"
)

set(TI5_DEVICE_RPATH
    "${TI5_DEVICE_SDK_ROOT}/lib:${TI5_MOTOR_CODEC_ROOT}/lib:${TI5_SERIAL_ROOT}/lib:${TI5_CAN_MINI_ROOT}/lib:${TI5_ROHAND_CODEC_ROOT}/lib:${TI5_DEX1_CODEC_ROOT}/lib:${TI5_AGV_SEER_ROOT}/lib")

target_link_directories(t170c_debug PRIVATE
    ${PROJECT_SOURCE_DIR}/lib
    /usr/local/lib
    ${TI5_DEVICE_SDK_ROOT}/lib
    ${TI5_MOTOR_CODEC_ROOT}/lib
    ${TI5_SERIAL_ROOT}/lib
    ${TI5_CAN_MINI_ROOT}/lib
    ${TI5_ROHAND_CODEC_ROOT}/lib
    ${TI5_DEX1_CODEC_ROOT}/lib
    ${TI5_AGV_SEER_ROOT}/lib
)
target_link_libraries(t170c_debug PRIVATE
    LowerBody::lower_body
    ${TI5_ARM_SOLVER_LIB}
    ${UNITREE_MOTOR_LIB}
    ${TI5_DEVICE_SDK_LIB}
    ${TI5_AGV_SEER_LIB}
    ${TI5_MOTOR_CODEC_LIB}
    ${TI5_ROHAND_CODEC_LIB}
    ${TI5_DEX1_CODEC_LIB}
    ${TI5_SERIAL_SDK_LIB}
    ${SERIALPORT_LIB}
    Threads::Threads
    ${CMAKE_DL_LIBS}
    ${JSONCPP_LIBRARIES}
    ${OpenCV_LIBS}
    yaml-cpp
    ${REALSENSE2_LIBRARIES}
    pybind11::embed
    Python3::Python
)

set_target_properties(t170c_debug PROPERTIES
    BUILD_RPATH "${PROJECT_SOURCE_DIR}/lib:${PROJECT_SOURCE_DIR}/third_party/libserialport/lib:/usr/local/lib:${HUMAN_INTERACTION_ENV_ROOT}/lib:${TI5_DEVICE_RPATH}"
    INSTALL_RPATH "${PROJECT_SOURCE_DIR}/lib:${PROJECT_SOURCE_DIR}/third_party/libserialport/lib:/usr/local/lib:${HUMAN_INTERACTION_ENV_ROOT}/lib:${TI5_DEVICE_RPATH}"
    BUILD_RPATH_USE_ORIGIN ON
)
target_link_options(t170c_debug PRIVATE "-Wl,--disable-new-dtags")
