cmake_minimum_required(VERSION 3.16)
project(T230cmake1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O3)
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-g)
endif()

if(NOT CMAKE_SYSTEM_PROCESSOR)
    execute_process(COMMAND uname -m OUTPUT_VARIABLE _host_arch OUTPUT_STRIP_TRAILING_WHITESPACE)
    set(CMAKE_SYSTEM_PROCESSOR "${_host_arch}")
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|armv8")
    set(TI5_CAN_DRIVER_ARM "${PROJECT_SOURCE_DIR}/lib/lib_ti5_can_driver_arm.so")
    set(TI5_CAN_DRIVER_LIB "${TI5_CAN_DRIVER_ARM}")
    # 运行时按 SONAME 查找 lib_ti5_can_driver.so，ARM 需链到 arm 版本
    set(TI5_CAN_DRIVER_SONAME "${PROJECT_SOURCE_DIR}/lib/lib_ti5_can_driver.so")
    if(EXISTS "${TI5_CAN_DRIVER_ARM}" AND NOT EXISTS "${TI5_CAN_DRIVER_SONAME}")
        file(CREATE_LINK "${TI5_CAN_DRIVER_ARM}" "${TI5_CAN_DRIVER_SONAME}" SYMBOLIC)
    elseif(EXISTS "${TI5_CAN_DRIVER_ARM}")
        execute_process(
            COMMAND file -b "${TI5_CAN_DRIVER_SONAME}"
            OUTPUT_VARIABLE _soname_file_info
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )
        if(_soname_file_info MATCHES "x86-64")
            file(REMOVE "${TI5_CAN_DRIVER_SONAME}")
            file(CREATE_LINK "${TI5_CAN_DRIVER_ARM}" "${TI5_CAN_DRIVER_SONAME}" SYMBOLIC)
        endif()
    endif()
else()
    set(TI5_CAN_DRIVER_LIB "${PROJECT_SOURCE_DIR}/lib/lib_ti5_can_driver.so")
endif()
message(STATUS "TI5 CAN driver: ${TI5_CAN_DRIVER_LIB}")

if(NOT EXISTS "${TI5_CAN_DRIVER_LIB}")
    message(FATAL_ERROR "Prebuilt library not found: ${TI5_CAN_DRIVER_LIB}")
endif()

add_library(ti5_can_driver SHARED IMPORTED GLOBAL)
set_target_properties(ti5_can_driver PROPERTIES
    IMPORTED_LOCATION "${TI5_CAN_DRIVER_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${PROJECT_SOURCE_DIR}/include"
)

find_package(Threads REQUIRED)

add_executable(move src/main.cpp)
target_include_directories(move PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(move
    PRIVATE
        ti5_can_driver
        Threads::Threads
)

set_target_properties(move PROPERTIES
    BUILD_RPATH "$ORIGIN/../lib"
    INSTALL_RPATH "$ORIGIN/../lib"
    BUILD_RPATH_USE_ORIGIN ON
)
