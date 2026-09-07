cmake_minimum_required(VERSION 3.14)
project(gripper CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 自包含：优先 gripper/ 内 vendor，否则用上级工程（与 market_simple build 一致）
set(GRIPPER_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
if(EXISTS "${GRIPPER_DIR}/third_party/dex1/include/serialPort/SerialPort.h")
  set(VENDOR_ROOT "${GRIPPER_DIR}")
elseif(EXISTS "${GRIPPER_DIR}/../third_party/dex1/include/serialPort/SerialPort.h")
  set(VENDOR_ROOT "${GRIPPER_DIR}/..")
else()
  message(FATAL_ERROR
    "缺少 third_party/dex1。请在上级工程目录运行:\n"
    "  cd gripper && ./sync_vendor.sh")
endif()

set(DEX1_INCLUDE_DIR "${VENDOR_ROOT}/third_party/dex1/include")
set(DEX1_LIB_DIR "${VENDOR_ROOT}/lib")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(UNITREE_MOTOR_LIB "${DEX1_LIB_DIR}/libUnitreeMotorSDK_Arm64.so")
else()
  set(UNITREE_MOTOR_LIB "${DEX1_LIB_DIR}/libUnitreeMotorSDK_Linux64.so")
endif()

if(NOT EXISTS "${UNITREE_MOTOR_LIB}")
  message(FATAL_ERROR "缺少 ${UNITREE_MOTOR_LIB}，请运行 ./sync_vendor.sh")
endif()

find_library(SERIALPORT_LIB serialport
  PATHS /lib/aarch64-linux-gnu /usr/lib/aarch64-linux-gnu /usr/local/lib
)
if(NOT SERIALPORT_LIB)
  message(WARNING "未找到 libserialport，请运行: sudo ./install_deps.sh")
endif()

add_library(gripper STATIC gripper_interface.cpp)
target_include_directories(gripper PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${DEX1_INCLUDE_DIR}
)
target_link_libraries(gripper PUBLIC
  ${UNITREE_MOTOR_LIB}
  ${SERIALPORT_LIB}
  pthread
)
set_target_properties(gripper PROPERTIES
  BUILD_RPATH "${DEX1_LIB_DIR}"
  INSTALL_RPATH "${DEX1_LIB_DIR}"
)

add_executable(gripper_demo example_gripper_demo.cpp)
target_link_libraries(gripper_demo PRIVATE gripper)

add_executable(gripper_grasp_demo example_grasp_demo.cpp)
target_link_libraries(gripper_grasp_demo PRIVATE gripper)

add_executable(gripper_grasp_soft_demo example_grasp_soft_demo.cpp)
target_link_libraries(gripper_grasp_soft_demo PRIVATE gripper)

add_executable(gripper_calibrate gripper_calibrate.cpp)
target_link_libraries(gripper_calibrate PRIVATE gripper)

install(TARGETS gripper gripper_demo gripper_grasp_demo gripper_grasp_soft_demo gripper_calibrate
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES gripper_interface.hpp DESTINATION include/gripper)
