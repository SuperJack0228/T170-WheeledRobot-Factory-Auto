cmake_minimum_required(VERSION 3.16)

project(LowerBody
    VERSION 1.0.0
    DESCRIPTION "170C 腰腿联动解析正逆运动学"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT TARGET Eigen3::Eigen)
    find_package(Eigen3 REQUIRED)
endif()

add_library(lower_body STATIC src/LowerBody.cpp)
add_library(LowerBody::lower_body ALIAS lower_body)
target_include_directories(lower_body
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(lower_body PUBLIC Eigen3::Eigen)
target_compile_features(lower_body PUBLIC cxx_std_17)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(lower_body PRIVATE -Wall -Wextra -Wpedantic)
endif()

set(_lower_body_standalone OFF)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(_lower_body_standalone ON)
endif()
option(LOWER_BODY_BUILD_EXAMPLES "构建自检与轨迹打印程序" ${_lower_body_standalone})

if(LOWER_BODY_BUILD_EXAMPLES)
    add_executable(lower_body_self_test examples/self_test.cpp)
    target_link_libraries(lower_body_self_test PRIVATE lower_body)

    add_executable(lower_body_traj_print examples/traj_print.cpp)
    target_link_libraries(lower_body_traj_print PRIVATE lower_body)
endif()

include(GNUInstallDirs)
install(TARGETS lower_body
    EXPORT LowerBodyTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(FILES include/LowerBody.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT LowerBodyTargets
    FILE LowerBodyTargets.cmake
    NAMESPACE LowerBody::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/LowerBody
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/LowerBodyConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/LowerBodyConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/LowerBodyConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/LowerBody
)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/LowerBodyConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/LowerBodyConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/LowerBody
)
