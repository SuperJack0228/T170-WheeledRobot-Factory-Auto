cmake_minimum_required(VERSION 3.12)
project(client_demo LANGUAGES CXX)  # 显式指定语言，避免隐式行为

# 显式列出源文件（避免 GLOB 的潜在问题）
file(GLOB DEV_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
)
add_executable(${PROJECT_NAME})
target_sources(${PROJECT_NAME} PRIVATE ${DEV_SOURCES})
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm|aarch64")
    # ARM 架构（包括 32位/64位）
    set(ARCH_DIR arm64)
else()
    # x86 架构（x86_64 / amd64）
    set(ARCH_DIR x86)
endif()

set(SDK_ARCH_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/${ARCH_DIR})

# lib 目录可能按 Ubuntu 版本拆分，例如 lib/arm64/22.04 或 lib/x86/18.04。
# 优先使用当前系统版本；找不到时兼容旧目录 lib/<arch>，再兜底选同架构最新版本。
set(SDK_UBUNTU_VERSION "" CACHE STRING "Ubuntu version subdirectory under lib/<arch>, e.g. 18.04, 20.04, 22.04")
if(NOT SDK_UBUNTU_VERSION AND EXISTS /etc/os-release)
  file(STRINGS /etc/os-release OS_RELEASE_VERSION REGEX "^VERSION_ID=")
  if(OS_RELEASE_VERSION)
    string(REGEX REPLACE "^VERSION_ID=\"?([^\"]+)\"?$" "\\1" SDK_UBUNTU_VERSION "${OS_RELEASE_VERSION}")
  endif()
endif()

set(SDK_LIB_DIR "")
if(SDK_UBUNTU_VERSION AND EXISTS ${SDK_ARCH_LIB_DIR}/${SDK_UBUNTU_VERSION}/libclient_internal.so)
  set(SDK_LIB_DIR ${SDK_ARCH_LIB_DIR}/${SDK_UBUNTU_VERSION})
elseif(EXISTS ${SDK_ARCH_LIB_DIR}/libclient_internal.so)
  set(SDK_LIB_DIR ${SDK_ARCH_LIB_DIR})
else()
  file(GLOB SDK_VERSION_DIRS LIST_DIRECTORIES true ${SDK_ARCH_LIB_DIR}/*)
  set(SDK_AVAILABLE_VERSION_DIRS "")
  foreach(SDK_VERSION_DIR ${SDK_VERSION_DIRS})
    if(IS_DIRECTORY ${SDK_VERSION_DIR} AND EXISTS ${SDK_VERSION_DIR}/libclient_internal.so)
      list(APPEND SDK_AVAILABLE_VERSION_DIRS ${SDK_VERSION_DIR})
    endif()
  endforeach()
  if(SDK_AVAILABLE_VERSION_DIRS)
    list(SORT SDK_AVAILABLE_VERSION_DIRS)
    list(REVERSE SDK_AVAILABLE_VERSION_DIRS)
    list(GET SDK_AVAILABLE_VERSION_DIRS 0 SDK_LIB_DIR)
  endif()
endif()

if(NOT SDK_LIB_DIR)
  message(FATAL_ERROR "No SDK library directory found under ${SDK_ARCH_LIB_DIR}")
endif()

message(STATUS "SDK library dir: ${SDK_LIB_DIR}")
target_link_directories(${PROJECT_NAME} PRIVATE
  ${SDK_LIB_DIR}
)

set(CLIENT_LIBS
  client_audio
  client_battery
  client_exception
  client_imu
  client_io_manager
  client_laser_2d
  client_lift
  client_light
  client_localization
  client_nav
  client_obstacle_avoidance
  client_odometry
  client_point_cloud
  client_qrcamera
  client_robot_task
)

if(EXISTS ${SDK_LIB_DIR}/libclient_depth_semantic.so)
  list(APPEND CLIENT_LIBS client_depth_semantic)
  target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_DEPTH_SEMANTIC=1)
endif()

if(EXISTS ${SDK_LIB_DIR}/libclient_map_manager.so)
  list(APPEND CLIENT_LIBS client_map_manager)
  target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_MAP_MANAGER=1)
endif()

if(EXISTS ${SDK_LIB_DIR}/libclient_map_reader.so)
  list(APPEND CLIENT_LIBS client_map_reader)
  target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_MAP_READER=1)
endif()

target_link_libraries(${PROJECT_NAME}
  ${CLIENT_LIBS}
  client_internal
  pthread
)
