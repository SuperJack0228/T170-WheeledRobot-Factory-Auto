cmake_minimum_required(VERSION 3.16)
project(t170_home_failure_replay LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
get_filename_component(T170_REPOSITORY "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
find_package(yaml-cpp CONFIG REQUIRED)
add_executable(home_failure_replay replay.cpp
    "${T170_REPOSITORY}/src/motion/arm.cpp"
    "${T170_REPOSITORY}/src/motion/arm_configuration.cpp"
    "${T170_REPOSITORY}/src/motion/joint_motion.cpp"
    "${T170_REPOSITORY}/src/motion/bounded_ik.cpp"
    "${T170_REPOSITORY}/src/common/paths.cpp")
target_include_directories(home_failure_replay PRIVATE
    "${T170_REPOSITORY}/include"
    "${T170_REPOSITORY}/dependencies/libraries/eigen")
if(TARGET yaml-cpp::yaml-cpp)
    target_link_libraries(home_failure_replay PRIVATE yaml-cpp::yaml-cpp)
else()
    target_link_libraries(home_failure_replay PRIVATE yaml-cpp)
endif()
