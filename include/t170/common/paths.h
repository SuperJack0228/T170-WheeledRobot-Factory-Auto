#pragma once

#include <filesystem>
#include <string>

namespace t170::paths {
// Explicit T170_ROOT, legacy MARKET_SIMPLE_ROOT, working/executable ancestors,
// then the build's source root. Relative directory overrides are root-relative.
std::filesystem::path resource_root();
std::filesystem::path find_resource_root(std::filesystem::path start);
std::filesystem::path resource(const std::filesystem::path &relative);
std::filesystem::path config_directory();
std::filesystem::path config_file(const std::filesystem::path &relative);
std::filesystem::path log_directory();
}

// Compatibility entry points. They no longer depend on the Python vision bridge.
std::string project_root_dir();
std::string default_cameras_yaml_path();
std::string default_pose_yaml_path();
std::string default_cylinder_pose_root();
std::string default_online_pose_root();
std::string default_aruco_pose_yaml_path();
std::string default_board_yf100_root();
std::string default_board_yf100_yaml_path();
std::string default_board_belt_root();
std::string default_board_belt_yaml_path();
std::string default_camera_to_robot_yaml_path();
std::string default_left_hand_to_robot_yaml_path();
std::string default_right_hand_to_robot_yaml_path();
