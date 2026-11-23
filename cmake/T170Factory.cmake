# Production assembly lists. No globbing in the build; all owners are reviewable here.
set(T170_TASK_SOURCES
    src/tasks/assignment.cpp
    src/tasks/automation.cpp
    src/tasks/charging.cpp
    src/tasks/context.cpp
    src/tasks/conveyor_grasp.cpp
    src/tasks/conveyor_perception.cpp
    src/tasks/conveyor_place.cpp
    src/tasks/conveyor_routing.cpp
    src/tasks/dispatch_feedback.cpp
    src/tasks/holding_state.cpp
    src/tasks/load.cpp
    src/tasks/map_sync.cpp
    src/tasks/navigation.cpp
    src/tasks/overlap.cpp
    src/tasks/pick_sequence.cpp
    src/tasks/place_sequence.cpp
    src/tasks/preparation.cpp
    src/tasks/recovery.cpp
    src/tasks/runtime_state.cpp
    src/tasks/status_reporter.cpp
    src/tasks/transfer.cpp
    src/tasks/tray_grasp.cpp
    src/tasks/tray_place.cpp
    src/tasks/unload.cpp
)
set(T170_APPLICATION_SOURCES
    src/app/task_result.cpp
    src/app/application.cpp
    src/app/audio.cpp
    src/app/boot_dispatch.cpp
    src/app/command_router.cpp
    src/app/commands_inspection.cpp
    src/app/commands_maintenance.cpp
    src/app/commands_manual_tasks.cpp
    src/app/commands_operation.cpp
    src/app/commands_runtime.cpp
    src/app/commands_status.cpp
    src/app/diagnostic_gestures.cpp
    src/app/hardware_factory.cpp
    src/app/json_view.cpp
    src/app/local_server.cpp
    src/app/maintenance/body_diagnostics.cpp
   
    src/app/maintenance/charge.cpp
   
    src/app/maintenance/tray_commands.cpp
   
    src/app/maintenance/vision_inspection.cpp
    src/app/shutdown.cpp
    src/app/state.cpp
    src/app/status.cpp
   
)
set(T170_CALIBRATION_SOURCES
    src/calibration/placement_session.cpp
    src/calibration/commands.cpp
)
add_library(t170_app_policy STATIC src/app/options.cpp src/app/command_catalog.cpp)
add_library(t170::app_policy ALIAS t170_app_policy)
target_include_directories(t170_app_policy PUBLIC ${PROJECT_SOURCE_DIR}/include)
add_library(t170_lifecycle STATIC src/app/lifecycle.cpp)
add_library(t170::lifecycle ALIAS t170_lifecycle)
target_include_directories(t170_lifecycle PUBLIC ${PROJECT_SOURCE_DIR}/include)
add_library(t170_dispatch_inbox STATIC src/integration/dispatch/inbox.cpp)
add_library(t170::dispatch_inbox ALIAS t170_dispatch_inbox)
target_include_directories(t170_dispatch_inbox PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(t170_dispatch_inbox PUBLIC Threads::Threads)
