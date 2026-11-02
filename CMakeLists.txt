cmake_minimum_required(VERSION 3.16)
project(t170_factory LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# No SDK, Python embed, CAN or camera dependencies in the foundation library.
add_library(t170_common STATIC src/common/paths.cpp src/common/logging.cpp)
add_library(t170::common ALIAS t170_common)
target_include_directories(t170_common PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_compile_definitions(t170_common PRIVATE T170_SOURCE_ROOT="${PROJECT_SOURCE_DIR}")

# Existing controller/kinematics implementation, also used by config validation.
add_library(t170_servo_core STATIC src/servo/visual_servo.cpp src/servo/kinematics.cpp)
add_library(t170::servo_core ALIAS t170_servo_core)
target_include_directories(t170_servo_core PUBLIC ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/eigen3)

function(t170_add_configuration)
    add_library(t170_configuration STATIC src/common/config.cpp src/servo/config.cpp)
    add_library(t170::configuration ALIAS t170_configuration)
    target_include_directories(t170_configuration PUBLIC ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/eigen3)
    target_link_libraries(t170_configuration PUBLIC t170::common t170::servo_core)
    if(TARGET yaml-cpp::yaml-cpp)
        target_link_libraries(t170_configuration PUBLIC yaml-cpp::yaml-cpp)
    else()
        target_link_libraries(t170_configuration PUBLIC yaml-cpp)
    endif()
endfunction()

# All offline regressions are required; missing dependencies must not silently skip tests.
option(T170_SERVO_TESTS_ONLY "Build only hardware-independent regression tests" OFF)
option(T170_BUILD_DIAGNOSTICS "Build optional robot-moving diagnostic tools" OFF)
if(T170_SERVO_TESTS_ONLY)
    find_package(Threads REQUIRED)
    find_package(yaml-cpp REQUIRED)
    find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d aruco)
    t170_add_configuration()
    enable_testing()
    add_executable(common_paths_test tests/common_paths_test.cpp)
    target_link_libraries(common_paths_test PRIVATE t170::common)
    add_test(NAME common_paths_test COMMAND common_paths_test)
    add_executable(visual_servo_test tests/visual_servo_test.cpp src/servo/trace.cpp)
    target_include_directories(visual_servo_test PRIVATE include eigen3)
    target_link_libraries(visual_servo_test PRIVATE Threads::Threads t170::servo_core)
    add_test(NAME visual_servo_test COMMAND visual_servo_test)
    add_executable(visual_servo_runtime_test tests/visual_servo_runtime_test.cpp
        src/servo/trace.cpp src/servo/runtime.cpp)
    target_include_directories(visual_servo_runtime_test PRIVATE include eigen3)
    target_link_libraries(visual_servo_runtime_test PRIVATE Threads::Threads t170::common t170::servo_core)
    add_test(NAME visual_servo_runtime_test COMMAND visual_servo_runtime_test)
    add_executable(retract_trajectory_test tests/retract_trajectory_test.cpp)
    target_include_directories(retract_trajectory_test PRIVATE include eigen3)
    add_test(NAME retract_trajectory_test COMMAND retract_trajectory_test)
    add_executable(grasp_execution_test tests/grasp_execution_test.cpp src/grasp_execution.cpp)
    target_include_directories(grasp_execution_test PRIVATE include eigen3 LowerBody/include)
    target_link_libraries(grasp_execution_test PRIVATE Threads::Threads)
    add_test(NAME grasp_execution_test COMMAND grasp_execution_test)
    add_executable(visual_servo_config_test tests/visual_servo_config_test.cpp)
    target_link_libraries(visual_servo_config_test PRIVATE t170::configuration)
    add_test(NAME visual_servo_config_test COMMAND visual_servo_config_test
        ${PROJECT_SOURCE_DIR}/config/move_box_params.yaml)
    add_executable(tray_localizer_test tests/tray_localizer_test.cpp src/servo/tray_localizer.cpp)
    target_include_directories(tray_localizer_test PRIVATE include eigen3 ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(tray_localizer_test PRIVATE ${OpenCV_LIBS})
    if(TARGET yaml-cpp::yaml-cpp)
        target_link_libraries(tray_localizer_test PRIVATE yaml-cpp::yaml-cpp)
    else()
        target_link_libraries(tray_localizer_test PRIVATE yaml-cpp)
    endif()
    add_test(NAME tray_localizer_test COMMAND tray_localizer_test
        ${PROJECT_SOURCE_DIR}/board_yf100_aruco/config.yaml)
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    add_test(NAME python_runtime_paths_test COMMAND ${Python3_EXECUTABLE}
        ${PROJECT_SOURCE_DIR}/tests/runtime_paths_test.py)
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

if(NOT TARGET Eigen3::Eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    target_include_directories(Eigen3::Eigen INTERFACE "${PROJECT_SOURCE_DIR}/eigen3")
endif()
add_subdirectory(LowerBody)

t170_add_configuration()
add_dependencies(t170_configuration servo_build_identity)
target_include_directories(t170_configuration PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

# Compile the shared robot implementation once for the app and optional diagnostics.
add_library(t170_runtime OBJECT
    orchestrator/cpp/robot_runtime.cpp
    orchestrator/cpp/dispatch_board.cpp
    orchestrator/cpp/dispatch_timing.cpp
    src/grasp_to_standby.cpp
    src/grasp_execution.cpp
    src/function.cpp
    src/hand_shadow.cpp
    src/servo/runtime.cpp
    src/servo/trace.cpp
    src/servo/tray_localizer.cpp
    src/waist.cpp
    src/Ti5_socketcan.cpp
    src/gripper_interface.cpp
    src/realsense_get.cpp
    src/seg_pose_bridge.cpp
    src/vision_debug.cpp
    src/vision/camera.cpp
    src/vision/dl.cpp
    src/vision/marker.cpp
    src/vision/pose.cpp
    src/vision/pipeline.cpp
    src/move_box_config.cpp
    src/move_box_runtime.cpp
    src/head_cam2robot.cpp
    src/head_control.cpp
)
add_library(t170::runtime ALIAS t170_runtime)
add_dependencies(t170_runtime servo_build_identity)
target_include_directories(t170_runtime PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/orchestrator/cpp
    ${PROJECT_SOURCE_DIR}/eigen3
    ${PROJECT_SOURCE_DIR}/third_party/dex1/include
    ${TI5_DEVICE_SDK_ROOT}/include
    ${TI5_AGV_SEER_ROOT}/include
    ${TI5_JINDI_CLIENT_ROOT}/include
    ${JSONCPP_INCLUDE_DIRS}
    ${REALSENSE2_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/third_party/libserialport/include
)
target_compile_definitions(t170_runtime PUBLIC
    SEG_POSE_CONDA_PREFIX="${HUMAN_INTERACTION_ENV_ROOT}"
)
set(TI5_DEVICE_RPATH
    "${TI5_DEVICE_SDK_ROOT}/lib:${TI5_MOTOR_CODEC_ROOT}/lib:${TI5_SERIAL_ROOT}/lib:${TI5_CAN_MINI_ROOT}/lib:${TI5_ROHAND_CODEC_ROOT}/lib:${TI5_DEX1_CODEC_ROOT}/lib:${TI5_AGV_SEER_ROOT}/lib:${TI5_JINDI_CLIENT_ROOT}/lib")

target_link_directories(t170_runtime PUBLIC
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
target_link_libraries(t170_runtime PUBLIC
    t170::common
    t170::configuration
    LowerBody::lower_body
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

function(t170_add_robot_app target entry)
    add_executable(${target} ${entry})
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
