cmake_minimum_required(VERSION 3.16)
project(t170_factory LANGUAGES CXX)
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build configuration" FORCE)
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include(cmake/T170Foundation.cmake)
option(T170_SERVO_TESTS_ONLY "Build hardware-independent regression tests" OFF)
option(T170_BUILD_DIAGNOSTICS "Build optional robot-moving diagnostic tools" OFF)
if(T170_SERVO_TESTS_ONLY)
    include(cmake/T170OfflineTests.cmake)
    return()
endif()

add_custom_target(servo_build_identity
    COMMAND ${CMAKE_COMMAND} -DSOURCE_DIR=${PROJECT_SOURCE_DIR}
        -DOUTPUT_FILE=${CMAKE_CURRENT_BINARY_DIR}/servo_build_identity.h
        -P ${PROJECT_SOURCE_DIR}/cmake/servo_build_identity.cmake
    BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/servo_build_identity.h
    VERBATIM)

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

set(TI5_DEVICE_SDK_ROOT "/opt/ti5-device-sdk" CACHE PATH "Robot SDK installation root")
set(TI5_MOTOR_CODEC_ROOT "/opt/ti5-motor-codec" CACHE PATH "Robot SDK installation root")
set(TI5_SERIAL_ROOT "/opt/ti5-serial" CACHE PATH "Robot SDK installation root")
set(TI5_CAN_MINI_ROOT "/opt/ti5-can-mini" CACHE PATH "Robot SDK installation root")
set(TI5_ROHAND_CODEC_ROOT "/opt/ti5-rohand-codec" CACHE PATH "Robot SDK installation root")
set(TI5_DEX1_CODEC_ROOT "/opt/ti5-dex1-codec" CACHE PATH "Robot SDK installation root")
set(TI5_AGV_SEER_ROOT "/opt/ti5-agv-seer" CACHE PATH "Robot SDK installation root")
set(TI5_ARM_SOLVER_LIB "${PROJECT_SOURCE_DIR}/lib/Ti5_Arm_2204.so")
set(TI5_DEVICE_SDK_LIB "${TI5_DEVICE_SDK_ROOT}/lib/libcommunication_manager.so")
set(TI5_MOTOR_CODEC_LIB "${TI5_MOTOR_CODEC_ROOT}/lib/libti5_motor.so")
set(TI5_SERIAL_SDK_LIB "${TI5_SERIAL_ROOT}/lib/libserial_port.so")
set(TI5_ROHAND_CODEC_LIB "${TI5_ROHAND_CODEC_ROOT}/lib/librohand_codec.so")
set(TI5_DEX1_CODEC_LIB "${TI5_DEX1_CODEC_ROOT}/lib/libTi5_Dex1_codec.so")
set(TI5_AGV_SEER_LIB "${TI5_AGV_SEER_ROOT}/lib/libseer.so")
set(TI5_JINDI_CLIENT_ROOT "/opt/ti5-jindi-faactory-robot-client" CACHE PATH "Robot SDK installation root")
set(TI5_JINDI_CLIENT_LIB "${TI5_JINDI_CLIENT_ROOT}/lib/libti5_jindi_faactory_robot_client.so")
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
        "${TI5_AGV_SEER_LIB}"
        "${TI5_JINDI_CLIENT_LIB}")
    if(NOT EXISTS "${required_lib}")
        message(FATAL_ERROR "Required robot library not found: ${required_lib}")
    endif()
endforeach()


add_subdirectory(libs/lower_body)
t170_add_configuration()
t170_add_observation_support()
add_dependencies(t170_configuration servo_build_identity)
target_include_directories(t170_configuration PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

add_library(t170_robot_platform INTERFACE)
target_include_directories(t170_robot_platform INTERFACE ${CMAKE_CURRENT_BINARY_DIR}
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/orchestrator/cpp
    ${PROJECT_SOURCE_DIR}/libs/lower_body/include
    ${PROJECT_SOURCE_DIR}/eigen3
    ${PROJECT_SOURCE_DIR}/third_party/dex1/include
    ${TI5_DEVICE_SDK_ROOT}/include
    ${TI5_AGV_SEER_ROOT}/include
    ${TI5_JINDI_CLIENT_ROOT}/include
    ${JSONCPP_INCLUDE_DIRS}
    ${REALSENSE2_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/third_party/libserialport/include
)
target_compile_definitions(t170_robot_platform INTERFACE
    SEG_POSE_CONDA_PREFIX="${HUMAN_INTERACTION_ENV_ROOT}"
)
set(TI5_DEVICE_RPATH
    "${TI5_DEVICE_SDK_ROOT}/lib:${TI5_MOTOR_CODEC_ROOT}/lib:${TI5_SERIAL_ROOT}/lib:${TI5_CAN_MINI_ROOT}/lib:${TI5_ROHAND_CODEC_ROOT}/lib:${TI5_DEX1_CODEC_ROOT}/lib:${TI5_AGV_SEER_ROOT}/lib:${TI5_JINDI_CLIENT_ROOT}/lib")

target_link_directories(t170_robot_platform INTERFACE
    ${PROJECT_SOURCE_DIR}/lib
    /usr/local/lib
    ${TI5_DEVICE_SDK_ROOT}/lib
    ${TI5_MOTOR_CODEC_ROOT}/lib
    ${TI5_SERIAL_ROOT}/lib
    ${TI5_CAN_MINI_ROOT}/lib
    ${TI5_ROHAND_CODEC_ROOT}/lib
    ${TI5_DEX1_CODEC_ROOT}/lib
    ${TI5_AGV_SEER_ROOT}/lib
    ${TI5_JINDI_CLIENT_ROOT}/lib
)
target_link_libraries(t170_robot_platform INTERFACE
    ${TI5_ARM_SOLVER_LIB}
    ${UNITREE_MOTOR_LIB}
    ${TI5_DEVICE_SDK_LIB}
    ${TI5_AGV_SEER_LIB}
    ${TI5_JINDI_CLIENT_LIB}
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


# Object modules have explicit host bindings listed in STEP1_COMPATIBILITY.md.
# The app helper adds these objects explicitly; none links back to runtime.
add_library(t170_hardware OBJECT src/hardware/device_bus.cpp src/hardware/gripper.cpp
    src/hardware/head.cpp src/hardware/waist.cpp)
add_library(t170::hardware ALIAS t170_hardware)
add_library(t170_vision OBJECT src/vision/camera/realsense_camera.cpp src/vision/camera/camera.cpp
    src/vision/detection/dl.cpp src/vision/geometry/marker.cpp src/vision/geometry/pose.cpp
    src/vision/geometry/head_transform.cpp src/vision/pipeline.cpp
    src/vision/bridge/python_bridge.cpp src/vision/bridge/visualization.cpp src/vision/bridge/debug.cpp)
add_library(t170::vision ALIAS t170_vision)
add_library(t170_observer OBJECT src/vision/tracking/wrist_observer.cpp)
add_library(t170::observer ALIAS t170_observer)
foreach(module IN ITEMS t170_hardware t170_vision t170_observer)
    target_link_libraries(${module} PUBLIC t170_robot_platform t170::configuration t170::common
        t170::control LowerBody::lower_body)
    add_dependencies(${module} servo_build_identity)
endforeach()

# Step 2 will split this remaining task/skill/executor host.
add_library(t170_runtime OBJECT
    orchestrator/cpp/robot_runtime.cpp orchestrator/cpp/dispatch_board.cpp
    src/grasp_to_standby.cpp src/grasp_execution.cpp src/function.cpp src/servo/runtime.cpp
    src/move_box_config.cpp src/move_box_runtime.cpp)
add_library(t170::runtime ALIAS t170_runtime)
add_dependencies(t170_runtime servo_build_identity)
target_link_libraries(t170_runtime PUBLIC t170_robot_platform t170::common t170::configuration
    t170::motion_math t170::servo_trace t170::tray_geometry t170::model_registry LowerBody::lower_body)

function(t170_add_robot_app target entry)
    add_executable(${target} ${entry})
    target_sources(${target} PRIVATE $<TARGET_OBJECTS:t170_hardware>
        $<TARGET_OBJECTS:t170_vision> $<TARGET_OBJECTS:t170_observer>)
    target_link_libraries(${target} PRIVATE t170::runtime)
    set_target_properties(${target} PROPERTIES
    BUILD_RPATH "${PROJECT_SOURCE_DIR}/lib:${PROJECT_SOURCE_DIR}/third_party/libserialport/lib:/usr/local/lib:${HUMAN_INTERACTION_ENV_ROOT}/lib:${TI5_DEVICE_RPATH}"
    INSTALL_RPATH "${PROJECT_SOURCE_DIR}/lib:${PROJECT_SOURCE_DIR}/third_party/libserialport/lib:/usr/local/lib:${HUMAN_INTERACTION_ENV_ROOT}/lib:${TI5_DEVICE_RPATH}"
    BUILD_RPATH_USE_ORIGIN ON
    )
    target_link_options(${target} PRIVATE "-Wl,--disable-new-dtags")
endfunction()

t170_add_robot_app(t170_factory apps/t170_factory_main.cpp)
if(T170_BUILD_DIAGNOSTICS)
    t170_add_robot_app(arm_plane_repeat tools/arm_plane_repeat.cpp)
endif()
