cmake_minimum_required(VERSION 3.16)
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/packaging/VERSION" T170_RELEASE_VERSION LIMIT_COUNT 1)
if(NOT T170_RELEASE_VERSION MATCHES "^[0-9]+\\.[0-9]+\\.[0-9]+(-[0-9A-Za-z.-]+)?$")
    message(FATAL_ERROR "packaging/VERSION must contain a semantic version")
endif()
string(REGEX REPLACE "-.*$" "" T170_PROJECT_VERSION "${T170_RELEASE_VERSION}")
project(t170_factory VERSION ${T170_PROJECT_VERSION} LANGUAGES CXX)
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build configuration" FORCE)
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include(cmake/T170Dependencies.cmake)
include(cmake/T170Foundation.cmake)
include(cmake/T170Motion.cmake)
include(cmake/T170Skills.cmake)
include(cmake/T170Factory.cmake)
include(cmake/T170ArmSdk.cmake)
option(T170_ARM_SDK_CHECK_ONLY "Build a calculation-only Linux SDK check, without any device library" OFF)
option(T170_SERVO_TESTS_ONLY "Build hardware-independent regression tests" OFF)
if(T170_SERVO_TESTS_ONLY)
    include(cmake/T170OfflineTests.cmake)
    return()
endif()
if(T170_ARM_SDK_CHECK_ONLY)
    find_package(Threads REQUIRED)
    find_package(yaml-cpp REQUIRED)
    t170_add_arm_support()
    t170_add_vendor_arm()
    add_executable(t170_arm_sdk_check tools/arm_sdk_check.cpp)
    target_link_libraries(t170_arm_sdk_check PRIVATE t170_arm_vendor)
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
t170_add_arm_support()
t170_add_vendor_arm()
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
if(NOT SERIALPORT_LIB AND EXISTS "${T170_SERIALPORT_ROOT}/lib/libserialport.so.0.1.0")
    set(SERIALPORT_LIB "${T170_SERIALPORT_ROOT}/lib/libserialport.so.0.1.0")
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
set(TI5_DEVICE_SDK_LIB "${TI5_DEVICE_SDK_ROOT}/lib/libcommunication_manager.so")
set(TI5_MOTOR_CODEC_LIB "${TI5_MOTOR_CODEC_ROOT}/lib/libti5_motor.so")
set(TI5_SERIAL_SDK_LIB "${TI5_SERIAL_ROOT}/lib/libserial_port.so")
set(TI5_ROHAND_CODEC_LIB "${TI5_ROHAND_CODEC_ROOT}/lib/librohand_codec.so")
set(TI5_DEX1_CODEC_LIB "${TI5_DEX1_CODEC_ROOT}/lib/libTi5_Dex1_codec.so")
set(TI5_AGV_SEER_LIB "${TI5_AGV_SEER_ROOT}/lib/libseer.so")
set(TI5_JINDI_CLIENT_ROOT "/opt/ti5-jindi-faactory-robot-client" CACHE PATH "Robot SDK installation root")
set(TI5_JINDI_CLIENT_LIB "${TI5_JINDI_CLIENT_ROOT}/lib/libti5_jindi_faactory_robot_client.so")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(UNITREE_MOTOR_LIB "${T170_UNITREE_ROOT}/lib/arm64/libUnitreeMotorSDK_Arm64.so")
else()
    set(UNITREE_MOTOR_LIB "${T170_UNITREE_ROOT}/lib/x86_64/libUnitreeMotorSDK_Linux64.so")
endif()
get_filename_component(UNITREE_MOTOR_LIBRARY_DIR "${UNITREE_MOTOR_LIB}" DIRECTORY)
add_library(t170_unitree SHARED IMPORTED)
set_target_properties(t170_unitree PROPERTIES IMPORTED_LOCATION "${UNITREE_MOTOR_LIB}" IMPORTED_NO_SONAME TRUE)
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


add_subdirectory(dependencies/sdk/lower_body)
t170_add_configuration()
t170_add_observation_support()
add_dependencies(t170_configuration servo_build_identity)
target_include_directories(t170_configuration PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

add_library(t170_robot_platform INTERFACE)
target_include_directories(t170_robot_platform INTERFACE ${CMAKE_CURRENT_BINARY_DIR}
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${T170_LOWER_BODY_ROOT}/include
    ${T170_EIGEN_INCLUDE_DIR}
    ${T170_ARM_INCLUDE_DIR}
    ${TI5_DEVICE_SDK_ROOT}/include
    ${TI5_AGV_SEER_ROOT}/include
    ${TI5_JINDI_CLIENT_ROOT}/include
    ${JSONCPP_INCLUDE_DIRS}
    ${REALSENSE2_INCLUDE_DIRS}
    ${T170_SERIALPORT_ROOT}/include
)
target_compile_definitions(t170_robot_platform INTERFACE
    SEG_POSE_CONDA_PREFIX="${HUMAN_INTERACTION_ENV_ROOT}"
)
set(TI5_DEVICE_RPATH
    "${TI5_DEVICE_SDK_ROOT}/lib:${TI5_MOTOR_CODEC_ROOT}/lib:${TI5_SERIAL_ROOT}/lib:${TI5_CAN_MINI_ROOT}/lib:${TI5_ROHAND_CODEC_ROOT}/lib:${TI5_DEX1_CODEC_ROOT}/lib:${TI5_AGV_SEER_ROOT}/lib:${TI5_JINDI_CLIENT_ROOT}/lib")

target_link_directories(t170_robot_platform INTERFACE
    ${T170_ARM_LIBRARY_DIR}
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
    t170_arm_vendor
    t170_unitree
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
    src/hardware/head.cpp src/hardware/waist.cpp src/hardware/chassis_seer.cpp src/hardware/body_power.cpp)
add_library(t170::hardware ALIAS t170_hardware)
add_library(t170_vision OBJECT src/vision/camera/realsense_camera.cpp src/vision/camera/camera.cpp
    src/vision/detection/dl.cpp src/vision/geometry/marker.cpp src/vision/geometry/pose.cpp
    src/vision/geometry/head_transform.cpp src/vision/pipeline.cpp
    src/vision/bridge/python_bridge.cpp src/vision/bridge/python_runtime.cpp src/vision/bridge/visualization.cpp src/vision/bridge/debug.cpp)
add_library(t170::vision ALIAS t170_vision)
add_library(t170_observer OBJECT src/vision/tracking/wrist_observer.cpp)
add_library(t170::observer ALIAS t170_observer)
foreach(module IN ITEMS t170_hardware t170_vision t170_observer)
    target_link_libraries(${module} PUBLIC t170_robot_platform t170::configuration t170::common
        t170::control t170::telemetry t170::lifecycle LowerBody::lower_body)
    add_dependencies(${module} servo_build_identity)
endforeach()

# Motion owns SDK adaptation, planning, playback, feedback and servo sessions.
add_library(t170_motion OBJECT ${T170_ARM_MOTION_SOURCES} ${T170_SERVO_RUNTIME_SOURCES})
add_library(t170::motion ALIAS t170_motion)
add_dependencies(t170_motion servo_build_identity)
target_link_libraries(t170_motion PUBLIC t170::telemetry t170_robot_platform t170::common t170::configuration
    t170::motion_math t170::pose_math t170::servo_trace t170::tray_geometry
    t170::model_registry LowerBody::lower_body)

add_library(t170_skills OBJECT ${T170_SKILLS_SOURCES})
add_library(t170::skills ALIAS t170_skills)
add_dependencies(t170_skills servo_build_identity)
target_link_libraries(t170_skills PUBLIC t170::motion t170::skill_contract t170_robot_platform
    t170::configuration t170::common t170::tray_geometry LowerBody::lower_body)

# External SDKs are confined to their adapters; tasks consume only public ports.
add_library(t170_dispatch_adapter OBJECT src/integration/dispatch/robot_client_adapter.cpp)
add_library(t170::dispatch_adapter ALIAS t170_dispatch_adapter)
add_library(t170_tasks OBJECT ${T170_TASK_SOURCES})
add_library(t170::tasks ALIAS t170_tasks)
add_library(t170_calibration OBJECT ${T170_CALIBRATION_SOURCES})
add_library(t170::calibration ALIAS t170_calibration)
add_library(t170_application OBJECT ${T170_APPLICATION_SOURCES})
add_library(t170::application ALIAS t170_application)
target_link_libraries(t170_dispatch_adapter PUBLIC t170_robot_platform)
target_link_libraries(t170_tasks PUBLIC t170_robot_platform t170::configuration t170::common
    t170::telemetry t170::dispatch_inbox t170::skills t170::motion
    t170::model_registry LowerBody::lower_body)
target_link_libraries(t170_calibration PUBLIC t170_robot_platform t170::configuration
    t170::skills t170::motion t170::tray_geometry t170::telemetry t170::model_registry LowerBody::lower_body)
target_link_libraries(t170_application PUBLIC t170_robot_platform t170_tasks t170_calibration
    t170::app_policy t170::lifecycle t170::telemetry t170::common)
foreach(module IN ITEMS t170_dispatch_adapter t170_tasks t170_calibration t170_application)
    add_dependencies(${module} servo_build_identity)
endforeach()
# Compatibility aggregation has no implementation source of its own.
add_library(t170_runtime INTERFACE)
add_library(t170::runtime ALIAS t170_runtime)
target_link_libraries(t170_runtime INTERFACE t170_application t170_tasks t170_calibration
    t170_dispatch_adapter)

function(t170_add_robot_app target entry)
    add_executable(${target} ${entry})
    target_sources(${target} PRIVATE $<TARGET_OBJECTS:t170_hardware>
        $<TARGET_OBJECTS:t170_vision> $<TARGET_OBJECTS:t170_observer>
        $<TARGET_OBJECTS:t170_motion> $<TARGET_OBJECTS:t170_skills>
        $<TARGET_OBJECTS:t170_application> $<TARGET_OBJECTS:t170_tasks>
        $<TARGET_OBJECTS:t170_calibration> $<TARGET_OBJECTS:t170_dispatch_adapter>)
    target_link_libraries(${target} PRIVATE t170::runtime)
    set_target_properties(${target} PROPERTIES
    BUILD_RPATH "${TI5_ARM_SOLVER_DIRECTORY}:${UNITREE_MOTOR_LIBRARY_DIR}:${T170_SERIALPORT_ROOT}/lib:/usr/local/lib:${HUMAN_INTERACTION_ENV_ROOT}/lib:${TI5_DEVICE_RPATH}"
    INSTALL_RPATH "$ORIGIN/../lib:/usr/local/lib:${HUMAN_INTERACTION_ENV_ROOT}/lib:${TI5_DEVICE_RPATH}"
    BUILD_RPATH_USE_ORIGIN ON
    )
    target_link_options(${target} PRIVATE "-Wl,--disable-new-dtags")
endfunction()

t170_add_robot_app(t170_factory apps/t170_factory_main.cpp)
add_dependencies(t170_factory t170_config_dump)
include(cmake/T170Packaging.cmake)
