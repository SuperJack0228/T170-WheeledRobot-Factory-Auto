    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    set(T170_TEST_RUNTIME "${CMAKE_BINARY_DIR}/test_runtime")
    # Test-only materialization; never starts factory hardware or copies SDK binaries.
    file(REMOVE_RECURSE "${T170_TEST_RUNTIME}")
    execute_process(COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tools/resolve_deployment.py
        ${PROJECT_SOURCE_DIR}/config/deployments/TH-T170C-0124.yaml ${T170_TEST_RUNTIME}
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} RESULT_VARIABLE profile_result ERROR_VARIABLE profile_error OUTPUT_QUIET)
    if(NOT profile_result EQUAL 0)
        message(FATAL_ERROR "Cannot prepare offline configuration (PyYAML required): ${profile_error}")
    endif()
    find_package(Threads REQUIRED)
    find_package(yaml-cpp REQUIRED)
    t170_add_arm_support()
    add_executable(arm_model_test tests/arm_model_test.cpp)
    target_link_libraries(arm_model_test PRIVATE t170_arm_core)
    add_test(NAME arm_model_test COMMAND arm_model_test ${T170_TEST_RUNTIME} ${PROJECT_SOURCE_DIR})
    add_executable(arm_motion_policy_test tests/arm_motion_policy_test.cpp)
    target_link_libraries(arm_motion_policy_test PRIVATE t170_arm_core)
    add_test(NAME arm_motion_policy_test COMMAND arm_motion_policy_test ${T170_TEST_RUNTIME} ${PROJECT_SOURCE_DIR})
    add_executable(arm_motion_timing_test tests/arm_motion_timing_test.cpp)
    target_link_libraries(arm_motion_timing_test PRIVATE t170_arm_core)
    add_test(NAME arm_motion_timing_test COMMAND arm_motion_timing_test ${T170_TEST_RUNTIME} ${PROJECT_SOURCE_DIR})
    add_executable(arm_io_test tests/arm_io_test.cpp src/motion/arm_io.cpp)
    target_link_libraries(arm_io_test PRIVATE t170_arm_configuration t170::control)
    add_test(NAME arm_io_test COMMAND arm_io_test ${T170_TEST_RUNTIME})
    add_library(arm_sdk_check_compilecheck OBJECT tools/arm_sdk_check.cpp)
    target_link_libraries(arm_sdk_check_compilecheck PRIVATE t170_arm_core)
    find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d)
    if(OpenCV_VERSION VERSION_LESS "4.7")
        find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d aruco)
    else()
        find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d objdetect)
    endif()
    t170_add_configuration()
    add_executable(config_reload_test tests/config_reload_test.cpp)
    target_link_libraries(config_reload_test PRIVATE t170::configuration)
    add_executable(extrinsics_test tests/extrinsics_test.cpp)
    target_link_libraries(extrinsics_test PRIVATE t170::configuration)
    add_test(NAME extrinsics_test COMMAND extrinsics_test ${T170_TEST_RUNTIME})
    add_test(NAME config_reload_test COMMAND config_reload_test ${T170_TEST_RUNTIME})
    t170_add_observation_support()
    # Compile real SDK-adapter/planner/player translation units against their public
    # declarations. No fake SDK implementation is linked; robot linking is a separate gate.
    add_library(t170_motion_compilecheck OBJECT ${T170_ARM_MOTION_SOURCES}
        src/app/diagnostic_gestures.cpp)
    target_include_directories(t170_motion_compilecheck PRIVATE include ${T170_ARM_INCLUDE_DIR} ${T170_EIGEN_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(t170_motion_compilecheck PRIVATE t170::configuration
        t170::motion_math t170::servo_trace)
    add_library(t170_skills_compilecheck OBJECT ${T170_SKILLS_SOURCES})
    target_include_directories(t170_skills_compilecheck PRIVATE include ${T170_ARM_INCLUDE_DIR} ${T170_EIGEN_INCLUDE_DIR} ${T170_LOWER_BODY_ROOT}/include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(t170_skills_compilecheck PRIVATE t170::configuration t170::skill_contract)
    set(LOWER_BODY_BUILD_EXAMPLES ON CACHE BOOL "Offline lower-body validation")
    add_subdirectory(dependencies/sdk/lower_body)
    enable_testing()
    add_executable(common_paths_test tests/common_paths_test.cpp)
    target_link_libraries(common_paths_test PRIVATE t170::common)
    add_test(NAME common_paths_test COMMAND common_paths_test)
    add_executable(visual_servo_test tests/visual_servo_test.cpp)
    target_include_directories(visual_servo_test PRIVATE include ${T170_ARM_INCLUDE_DIR} ${T170_EIGEN_INCLUDE_DIR})
    target_link_libraries(visual_servo_test PRIVATE Threads::Threads t170::servo_core t170::servo_trace)
    add_test(NAME visual_servo_test COMMAND visual_servo_test)
    add_executable(visual_servo_runtime_test tests/visual_servo_runtime_test.cpp
        ${T170_SERVO_RUNTIME_SOURCES})
    target_include_directories(visual_servo_runtime_test PRIVATE include ${T170_ARM_INCLUDE_DIR} ${T170_EIGEN_INCLUDE_DIR})
    target_link_libraries(visual_servo_runtime_test PRIVATE t170::telemetry Threads::Threads t170::servo_core t170::servo_trace)
    add_test(NAME visual_servo_runtime_test COMMAND visual_servo_runtime_test)
    add_executable(retract_trajectory_test tests/retract_trajectory_test.cpp)
    target_include_directories(retract_trajectory_test PRIVATE include ${T170_ARM_INCLUDE_DIR} ${T170_EIGEN_INCLUDE_DIR})
    add_test(NAME retract_trajectory_test COMMAND retract_trajectory_test)
    add_executable(retreat_gate_test tests/retreat_gate_test.cpp)
    target_include_directories(retreat_gate_test PRIVATE include)
    target_link_libraries(retreat_gate_test PRIVATE Threads::Threads)
    add_test(NAME retreat_gate_test COMMAND retreat_gate_test)
    add_executable(grasp_execution_test tests/grasp_execution_test.cpp src/skills/common/grasp_finish.cpp)
    target_include_directories(grasp_execution_test PRIVATE include ${T170_ARM_INCLUDE_DIR} ${T170_EIGEN_INCLUDE_DIR} ${T170_LOWER_BODY_ROOT}/include)
    target_link_libraries(grasp_execution_test PRIVATE Threads::Threads t170::skill_contract)
    add_test(NAME grasp_execution_test COMMAND grasp_execution_test)
    add_executable(visual_servo_config_test tests/visual_servo_config_test.cpp)
    target_link_libraries(visual_servo_config_test PRIVATE t170::configuration)
    add_test(NAME visual_servo_config_test COMMAND visual_servo_config_test
        ${T170_TEST_RUNTIME}/config/move_box_params.yaml)
    add_executable(tray_localizer_test tests/tray_localizer_test.cpp)
    add_executable(tray_acquisition_test tests/tray_acquisition_test.cpp)
    add_executable(tray_place_latency_test tests/tray_place_latency_test.cpp)
    target_link_libraries(tray_place_latency_test PRIVATE t170::configuration t170::servo_core)
    add_test(NAME tray_place_latency_test COMMAND tray_place_latency_test
        ${T170_TEST_RUNTIME}/config/move_box_params.yaml
        ${PROJECT_SOURCE_DIR}/tests/fixtures/tray_place_latency_20261106.json)
    target_link_libraries(tray_acquisition_test PRIVATE t170::tray_geometry)
    add_test(NAME tray_acquisition_test COMMAND tray_acquisition_test)
    add_executable(tray_localizer_replay tools/tray_localizer_replay.cpp)
    target_link_libraries(tray_localizer_replay PRIVATE t170::configuration t170::tray_geometry)
    target_link_libraries(tray_localizer_test PRIVATE t170::tray_geometry)
    add_test(NAME tray_localizer_test COMMAND tray_localizer_test
        ${T170_TEST_RUNTIME}/config/vision/tray.yaml
        ${PROJECT_SOURCE_DIR}/tests/fixtures/tray_neighbor_correspondences_20261105.json)
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    add_test(NAME python_runtime_paths_test COMMAND ${Python3_EXECUTABLE}
        ${PROJECT_SOURCE_DIR}/tests/runtime_paths_test.py)
    add_test(NAME lower_body_self_test COMMAND lower_body_self_test)
    add_executable(public_module_headers_test tests/public_module_headers_test.cpp)
    target_link_libraries(public_module_headers_test PRIVATE t170::control)
    add_test(NAME public_module_headers_test COMMAND public_module_headers_test)
    add_executable(model_registry_test tests/model_registry_test.cpp)
    target_link_libraries(model_registry_test PRIVATE t170::model_registry)
    add_test(NAME model_registry_test COMMAND model_registry_test)
    add_test(NAME python_vision_package_test COMMAND ${Python3_EXECUTABLE}
        ${PROJECT_SOURCE_DIR}/tests/python_vision_package_test.py)
    add_test(NAME module_boundaries_test COMMAND ${Python3_EXECUTABLE}
        ${PROJECT_SOURCE_DIR}/tests/module_boundaries_test.py)

    add_executable(joint_motion_test tests/joint_motion_test.cpp)
    target_link_libraries(joint_motion_test PRIVATE t170::motion_math Threads::Threads)
    add_test(NAME joint_motion_test COMMAND joint_motion_test)

    add_executable(pose_math_test tests/pose_math_test.cpp)
    target_link_libraries(pose_math_test PRIVATE t170::pose_math)
    add_test(NAME pose_math_test COMMAND pose_math_test)

    add_executable(placement_calibration_math_test tests/placement_calibration_math_test.cpp src/skills/tray/profile_policy.cpp)
    target_link_libraries(placement_calibration_math_test PRIVATE t170::configuration)
    add_test(NAME placement_calibration_math_test COMMAND placement_calibration_math_test ${T170_TEST_RUNTIME})
    add_test(NAME placement_calibration_web_test COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tests/placement_calibration_web_test.py)
    add_test(NAME web_profiles_test COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tests/web_profiles_test.py)

    find_package(jsoncpp CONFIG REQUIRED)
    add_executable(task_result_test tests/task_result_test.cpp src/app/task_result.cpp)
    target_link_libraries(task_result_test PRIVATE t170::configuration t170::skill_contract JsonCpp::JsonCpp)
    add_test(NAME task_result_test COMMAND task_result_test ${T170_TEST_RUNTIME})
    add_test(NAME web_command_result_test COMMAND ${Python3_EXECUTABLE}
        ${PROJECT_SOURCE_DIR}/tests/web_command_result_test.py $<TARGET_FILE:task_result_test> ${T170_TEST_RUNTIME})
    add_library(t170_factory_compilecheck OBJECT ${T170_TASK_SOURCES} ${T170_APPLICATION_SOURCES}
        ${T170_CALIBRATION_SOURCES} src/hardware/body_power.cpp src/hardware/chassis_seer.cpp
        src/hardware/waist.cpp src/vision/geometry/head_transform.cpp
        src/vision/tracking/wrist_observer.cpp src/vision/bridge/debug.cpp src/vision/camera/camera.cpp)
    target_include_directories(t170_factory_compilecheck PRIVATE include ${T170_ARM_INCLUDE_DIR} ${T170_EIGEN_INCLUDE_DIR}
        ${T170_LOWER_BODY_ROOT}/include ${T170_SEER_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(t170_factory_compilecheck PRIVATE t170::configuration t170::telemetry
        t170::app_policy t170::lifecycle t170::dispatch_inbox JsonCpp::JsonCpp)

    add_executable(placement_calibration_session_test tests/placement_calibration_session_test.cpp
        src/calibration/placement_session.cpp src/calibration/commands.cpp src/skills/tray/profile_policy.cpp)
    target_include_directories(placement_calibration_session_test PRIVATE include ${T170_ARM_INCLUDE_DIR} ${T170_EIGEN_INCLUDE_DIR} ${T170_LOWER_BODY_ROOT}/include)
    target_link_libraries(placement_calibration_session_test PRIVATE t170::configuration t170::motion_math t170::tray_geometry JsonCpp::JsonCpp Threads::Threads)
    add_test(NAME placement_calibration_session_test COMMAND placement_calibration_session_test ${T170_TEST_RUNTIME})

    add_executable(calibration_hole_test tests/calibration_hole_test.cpp)
    target_link_libraries(calibration_hole_test PRIVATE t170::tray_geometry)
    add_test(NAME calibration_hole_test COMMAND calibration_hole_test)

    add_executable(dispatch_metrics_emitter tests/dispatch_metrics_emitter.cpp)
    target_link_libraries(dispatch_metrics_emitter PRIVATE t170::telemetry)
    add_test(NAME dispatch_metrics_test COMMAND ${Python3_EXECUTABLE}
        ${PROJECT_SOURCE_DIR}/tests/dispatch_metrics_test.py $<TARGET_FILE:dispatch_metrics_emitter>)
    add_test(NAME dispatch_report_test COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tests/dispatch_report_test.py)
    add_test(NAME deployment_test COMMAND ${Python3_EXECUTABLE}
        ${PROJECT_SOURCE_DIR}/tests/deployment_test.py $<TARGET_FILE:t170_config_dump>)
    add_test(NAME web_launcher_test COMMAND ${Python3_EXECUTABLE}
        ${PROJECT_SOURCE_DIR}/tests/web_launcher_test.py)
    add_test(NAME arm_model_geometry_test COMMAND ${Python3_EXECUTABLE}
        ${PROJECT_SOURCE_DIR}/tests/arm_model_geometry_test.py)

    add_executable(skills_contract_test tests/skills_contract_test.cpp)
    target_link_libraries(skills_contract_test PRIVATE t170::skill_contract Eigen3::Eigen)
    add_test(NAME skills_contract_test COMMAND skills_contract_test)
    add_test(NAME skills_contract_json_test COMMAND ${Python3_EXECUTABLE}
        ${PROJECT_SOURCE_DIR}/tests/skills_contract_json_test.py $<TARGET_FILE:skills_contract_test>)
    add_executable(conveyor_skill_test tests/conveyor_skill_test.cpp
        src/skills/conveyor/execution.cpp src/skills/common/grasp_finish.cpp)
    target_include_directories(conveyor_skill_test PRIVATE include ${T170_ARM_INCLUDE_DIR} ${T170_EIGEN_INCLUDE_DIR} ${T170_LOWER_BODY_ROOT}/include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(conveyor_skill_test PRIVATE t170::skill_contract)
    add_test(NAME conveyor_skill_test COMMAND conveyor_skill_test)
    add_executable(tray_skill_service_test tests/tray_skill_service_test.cpp src/skills/tray/service.cpp)
    target_include_directories(tray_skill_service_test PRIVATE include ${T170_ARM_INCLUDE_DIR} ${T170_EIGEN_INCLUDE_DIR} ${T170_LOWER_BODY_ROOT}/include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(tray_skill_service_test PRIVATE t170::skill_contract LowerBody::lower_body)
    add_test(NAME tray_skill_service_test COMMAND tray_skill_service_test)
    add_library(skills_examples_compilecheck OBJECT examples/skills/typed_calls.cpp)
    target_link_libraries(skills_examples_compilecheck PRIVATE t170::skill_contract Eigen3::Eigen)

    add_executable(app_policy_test tests/app_policy_test.cpp)
    target_link_libraries(app_policy_test PRIVATE t170::app_policy)
    add_test(NAME app_policy_test COMMAND app_policy_test)
    add_test(NAME factory_entrypoints_test COMMAND ${Python3_EXECUTABLE}
        ${PROJECT_SOURCE_DIR}/tests/factory_entrypoints_test.py $<TARGET_FILE:app_policy_test>)
    add_executable(factory_navigation_test tests/factory_navigation_test.cpp
        src/tasks/navigation.cpp src/tasks/context.cpp)
    target_include_directories(factory_navigation_test PRIVATE include ${T170_ARM_INCLUDE_DIR} ${T170_EIGEN_INCLUDE_DIR} ${T170_LOWER_BODY_ROOT}/include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(factory_navigation_test PRIVATE t170::configuration t170::telemetry)
    add_test(NAME factory_navigation_test COMMAND factory_navigation_test)
    add_executable(factory_assignment_test tests/factory_assignment_test.cpp
        src/tasks/assignment.cpp src/tasks/context.cpp src/tasks/runtime_state.cpp)
    target_include_directories(factory_assignment_test PRIVATE include ${T170_ARM_INCLUDE_DIR} ${T170_EIGEN_INCLUDE_DIR} ${T170_LOWER_BODY_ROOT}/include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(factory_assignment_test PRIVATE t170::configuration t170::telemetry)
    add_test(NAME factory_assignment_test COMMAND factory_assignment_test)
    add_executable(dispatch_adapter_test tests/dispatch_adapter_test.cpp
        src/integration/dispatch/robot_client_adapter.cpp)
    # The test double MUST NOT leak into t170_dispatch_adapter/robot_platform.
    target_include_directories(dispatch_adapter_test PRIVATE tests/vendor_doubles)
    target_link_libraries(dispatch_adapter_test PRIVATE t170::dispatch_inbox)
    add_test(NAME dispatch_adapter_test COMMAND dispatch_adapter_test)
    set_tests_properties(dispatch_adapter_test PROPERTIES TIMEOUT 5)
