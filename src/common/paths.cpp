#include "t170/common/paths.h"

#include <cstdlib>
#include <vector>
#if defined(__APPLE__)
#include <mach-o/dyld.h>
#elif defined(__linux__)
#include <unistd.h>
#endif

namespace {
const char *nonempty_env(const char *name) {
    const char *value=std::getenv(name);
    return value && *value ? value : nullptr;
}
std::filesystem::path absolute_normal(const std::filesystem::path &path) {
    return std::filesystem::absolute(path).lexically_normal();
}
std::filesystem::path executable_directory() {
#if defined(__APPLE__)
    uint32_t size=0;
    _NSGetExecutablePath(nullptr,&size);
    std::vector<char> buffer(size);
    if(_NSGetExecutablePath(buffer.data(),&size)==0)
        return absolute_normal(buffer.data()).parent_path();
#elif defined(__linux__)
    std::error_code ec;
    const auto path=std::filesystem::read_symlink("/proc/self/exe",ec);
    if(!ec) return path.parent_path();
#endif
    return {};
}
std::filesystem::path directory_override(const char *name,const char *fallback) {
    const auto root=t170::paths::resource_root();
    if(const char *value=nonempty_env(name)) return absolute_normal(root/value);
    return root/fallback;
}
}

namespace t170::paths {
std::filesystem::path find_resource_root(std::filesystem::path start) {
    if(start.empty()) return {};
    start=absolute_normal(start);
    for(;;) {
        std::error_code ec;
        if(std::filesystem::is_regular_file(start/"config/realsense_cameras.yaml",ec))
            return start;
        const auto parent=start.parent_path();
        if(parent==start) return {};
        start=parent;
    }
}
std::filesystem::path resource_root() {
    if(const char *value=nonempty_env("T170_ROOT")) return absolute_normal(value);
    if(const char *value=nonempty_env("MARKET_SIMPLE_ROOT")) return absolute_normal(value);
    if(const auto path=find_resource_root(std::filesystem::current_path());!path.empty()) return path;
    if(const auto path=find_resource_root(executable_directory());!path.empty()) return path;
#ifdef T170_SOURCE_ROOT
    if(const auto path=find_resource_root(T170_SOURCE_ROOT);!path.empty()) return path;
#endif
    return std::filesystem::current_path();
}
std::filesystem::path resource(const std::filesystem::path &relative) {
    return (resource_root()/relative).lexically_normal();
}
std::filesystem::path config_directory() { return directory_override("T170_CONFIG_DIR","config"); }
std::filesystem::path config_file(const std::filesystem::path &relative) {
    return (config_directory()/relative).lexically_normal();
}
std::filesystem::path log_directory() { return directory_override("T170_LOG_DIR","logs"); }
}

std::string project_root_dir() { return t170::paths::resource_root().string(); }
std::string default_cameras_yaml_path() { return t170::paths::config_file("realsense_cameras.yaml").string(); }
std::string default_pose_yaml_path() { return t170::paths::resource("feeding_cylindrical_parts_alg/config/pose_params.yaml").string(); }
std::string default_cylinder_pose_root() { return t170::paths::resource("feeding_cylindrical_parts_alg").string(); }
std::string default_online_pose_root() { return t170::paths::resource("online_pose").string(); }
std::string default_aruco_pose_yaml_path() { return t170::paths::resource("online_pose/config.yaml").string(); }
std::string default_board_yf100_root() { return t170::paths::resource("board_yf100_aruco").string(); }
std::string default_board_yf100_yaml_path() { return t170::paths::resource("board_yf100_aruco/config.yaml").string(); }
std::string default_board_belt_root() { return t170::paths::resource("board_belt_aruco").string(); }
std::string default_board_belt_yaml_path() { return t170::paths::resource("board_belt_aruco/config.yaml").string(); }
std::string default_camera_to_robot_yaml_path() { return t170::paths::config_file("camera_to_base_result.yaml").string(); }
std::string default_left_hand_to_robot_yaml_path() { return t170::paths::config_file("left_hand_to_base_result.yaml").string(); }
std::string default_right_hand_to_robot_yaml_path() { return t170::paths::config_file("right_hand_to_base_result.yaml").string(); }
