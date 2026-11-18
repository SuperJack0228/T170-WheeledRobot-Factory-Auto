#include "t170/config/access.h"
#include "t170/calibration/identity.h"
#include <filesystem>
#include <exception>
#include <map>

namespace t170::config {
namespace {
std::map<std::filesystem::path,std::string> startup_resources;
std::string fingerprint_or_missing(const std::filesystem::path &path) {
    return std::filesystem::is_regular_file(path) ? calibration::fingerprint(path) : "missing";
}
std::string restart_reason(const MoveBoxConfig &a,const MoveBoxConfig &b) {
    if(a.chassis.host!=b.chassis.host || a.chassis.robot_type!=b.chassis.robot_type ||
       a.chassis.sequence_number!=b.chassis.sequence_number)
        return "机器人身份或底盘连接已改变";
    const auto &x=a.cameras;const auto &y=b.cameras;
    if(x.width!=y.width || x.height!=y.height || x.head_fps!=y.head_fps || x.hand_fps!=y.hand_fps ||
       x.hand_live_preview!=y.hand_live_preview || x.hand_descent_correct!=y.hand_descent_correct)
        return "相机流或采集所有权配置已改变";
    const auto &u=a.retract_motion;const auto &v=b.retract_motion;
    if(u.max_joint_velocity_rad_s!=v.max_joint_velocity_rad_s ||
       u.max_joint_acceleration_rad_s2!=v.max_joint_acceleration_rad_s2 ||
       u.max_joint_jerk_rad_s3!=v.max_joint_jerk_rad_s3)
        return "Arm 实例持有的轨迹速度/加速度/jerk 配置已改变";
    for(const auto &[path,before]:startup_resources)
        if(fingerprint_or_missing(path)!=before) return "已加载的资源已改变: "+path.filename().string();
    return {};
}
}
std::mutex &access_mutex() { static std::mutex mutex;return mutex; }

void capture_startup_resources(const std::string &directory) {
    std::lock_guard lock(access_mutex());
    startup_resources.clear();
    for(const char *name:{"Robot_Model.yaml","arm_binding.yaml","arm_model_metadata.yaml",
        "arm_frames.yaml","arm_joint_conventions.yaml","realsense_cameras.yaml","devices.json",
        "left_hand_to_base_result.yaml","right_hand_to_base_result.yaml","camera_to_base_result.yaml",
        "head/extrinsics.yaml","head/geometry.yaml","vision/pose_params.yaml",
        "vision/tray.yaml","vision/belt.yaml","vision/marker.yaml"}) {
        const auto path=std::filesystem::absolute(std::filesystem::path(directory)/name);
        startup_resources[path]=fingerprint_or_missing(path);
    }
}

ReloadResult reload_when_idle(std::mutex &hardware_mutex,
    const std::function<bool()> &calibration_active,const std::function<bool()> &stopping,
    const std::string &path,MoveBoxConfig &current) {
    // Never wait behind a long production task or hold a control-loop mutex
    // while parsing YAML. An explicit rejection is preferable to a stale queued reload.
    std::unique_lock hardware(hardware_mutex,std::try_to_lock);
    if(!hardware.owns_lock()) return {false,"busy","机器人动作或调度占用中，请退出并等待停止后重载"};
    std::lock_guard config(access_mutex());
    if(stopping()) return {false,"stopping","机器人服务正在退出"};
    if(calibration_active()) return {false,"calibration_active","标定期间不能重载正式配置"};
    MoveBoxConfig next;
    std::string error;
    if(!load_move_box_config(path,next,error)) return {false,"invalid_config",error};
    try {
        if(const auto reason=restart_reason(current,next);!reason.empty())
            return {false,"restart_required",reason+"，请重新生成运行目录并重启"};
    } catch(const std::exception &ex) {return {false,"resource_check_failed",ex.what()};}
    if(stopping()) return {false,"stopping","机器人服务正在退出"};
    current=std::move(next);
    return {true,"reloaded",{}};
}
}
