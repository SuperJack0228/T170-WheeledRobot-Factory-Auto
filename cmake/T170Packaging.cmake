# Installation rules are shared with the relocation test fixture.
function(t170_install_resources root)
    install(DIRECTORY "${root}/python/" DESTINATION python
        PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
    install(DIRECTORY "${root}/web_debug/" DESTINATION web_debug
        PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
    install(DIRECTORY "${root}/config/" DESTINATION share/config
        PATTERN "tasks" EXCLUDE PATTERN "*_before_*" EXCLUDE)
    install(DIRECTORY "${root}/assets/" DESTINATION assets
        PATTERN "archive" EXCLUDE PATTERN "legacy" EXCLUDE)
    install(FILES "${root}/assets/models/runtime_manifest.json" DESTINATION assets/models RENAME manifest.json)
    install(DIRECTORY "${root}/deploy/" DESTINATION deploy USE_SOURCE_PERMISSIONS)
    install(PROGRAMS "${root}/tools/resolve_deployment.py" "${root}/tools/active_runtime.py"
        "${root}/tools/apply_config.py" "${root}/tools/setup_robot.py" "${root}/tools/upgrade_robot.py" DESTINATION tools)
    install(FILES "${root}/docs/deployment/V1_QUICK_START.md" "${root}/docs/deployment/PACKAGE_UPGRADE.md" DESTINATION docs)
endfunction()

if(T170_PACKAGING_RESOURCES_ONLY)
    return()
endif()

t170_install_resources("${PROJECT_SOURCE_DIR}")
install(TARGETS t170_factory t170_config_dump RUNTIME DESTINATION bin)
set_target_properties(t170_config_dump PROPERTIES INSTALL_RPATH "$ORIGIN/../lib:${HUMAN_INTERACTION_ENV_ROOT}/lib")
install(FILES "${TI5_ARM_SOLVER_LIB}" "${UNITREE_MOTOR_LIB}" DESTINATION lib)
get_filename_component(serialport_runtime "${SERIALPORT_LIB}" REALPATH)
install(FILES "${serialport_runtime}" DESTINATION lib RENAME libserialport.so.0)
file(STRINGS "${PROJECT_SOURCE_DIR}/packaging/VERSION" T170_RELEASE_VERSION LIMIT_COUNT 1)
execute_process(COMMAND git rev-parse HEAD WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
    OUTPUT_VARIABLE release_commit OUTPUT_STRIP_TRAILING_WHITESPACE)
file(WRITE "${CMAKE_BINARY_DIR}/release.json"
    "{\n  \"version\": \"${T170_RELEASE_VERSION}\",\n  \"git_commit\": \"${release_commit}\",\n  \"system\": \"${CMAKE_SYSTEM_NAME}\",\n  \"architecture\": \"${CMAKE_SYSTEM_PROCESSOR}\",\n  \"arm_sdk_platform\": \"${TI5_ARM_SDK_PLATFORM}\",\n  \"python_environment\": \"${HUMAN_INTERACTION_ENV_ROOT}\"\n}\n")
execute_process(COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/packaging/record_environment.py"
    "${CMAKE_BINARY_DIR}/environment.lock.json" RESULT_VARIABLE lock_result)
if(NOT lock_result EQUAL 0)
    message(FATAL_ERROR "Cannot record release environment")
endif()
install(FILES "${CMAKE_BINARY_DIR}/release.json" "${CMAKE_BINARY_DIR}/environment.lock.json" DESTINATION .)
set(CPACK_PACKAGE_NAME "t170-factory")
set(CPACK_PACKAGE_VERSION "${T170_RELEASE_VERSION}")
set(CPACK_PACKAGE_FILE_NAME "t170-factory-${T170_RELEASE_VERSION}-${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")
set(CPACK_GENERATOR TGZ)
set(CPACK_PACKAGE_CHECKSUM SHA256)
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY ON)
include(CPack)
