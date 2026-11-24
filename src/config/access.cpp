#include "t170/config/access.h"
#include "t170/calibration/identity.h"
#include <filesystem>
#include <exception>
#include <map>
#include <fstream>
#include <iterator>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace t170::config {
namespace {
std::map<std::filesystem::path,std::string> startup_resources;
std::filesystem::path startup_directory;
std::string fingerprint_or_missing(const std::filesystem::path &path) {
    return std::filesystem::is_regular_file(path) ? calibration::fingerprint(path) : "missing";
}
void publish_snapshots(const std::vector<std::pair<std::filesystem::path,std::filesystem::path>> &files) {
    struct File {std::filesystem::path target,temporary;bool existed;std::string previous;};
    std::vector<File> staged;std::size_t published=0;
    try {
        for(const auto &[source,target]:files) {
            File item{target,target.string()+".reload.tmp",std::filesystem::exists(target),{}};
            if(item.existed) {
                std::ifstream before(target,std::ios::binary);
                if(!before)throw std::runtime_error("无法读取旧运行快照");
                item.previous.assign(std::istreambuf_iterator<char>(before),{});
            }
            staged.push_back(std::move(item));
            std::ifstream input(source,std::ios::binary);
            std::ofstream output(staged.back().temporary,std::ios::binary|std::ios::trunc);
            if(!input || !output)throw std::runtime_error("无法写入运行配置快照");
            output<<input.rdbuf();output.flush();
            if(!output)throw std::runtime_error("无法完成运行配置快照");
        }
        for(const auto &file:staged) {std::filesystem::rename(file.temporary,file.target);++published;}
    } catch(const std::exception &error) {
        std::string reason=error.what();
        for(std::size_t i=0;i<published;++i) {
            const auto &file=staged[i];
            try {
                if(file.existed) {
                    std::ofstream out(file.temporary,std::ios::binary|std::ios::trunc);
                    out<<file.previous;out.flush();
                    if(!out)throw std::runtime_error("快照恢复写入失败");
                    out.close();std::filesystem::rename(file.temporary,file.target);
                } else std::filesystem::remove(file.target);
            } catch(const std::exception &rollback) {reason+="; "+std::string(rollback.what());}
        }
        for(const auto &file:staged){std::error_code ignored;std::filesystem::remove(file.temporary,ignored);}
        throw std::runtime_error(reason);
    }
}
std::string restart_reason(const MoveBoxConfig &a,const MoveBoxConfig &b,const std::filesystem::path &candidate_directory) {
    if(!same_conveyor_identity(a.conveyor_heights,b.conveyor_heights))
        return "机床二维码绑定或类型已改变，视觉引擎需要重启";
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
    for(const auto &[path,before]:startup_resources) {
        if(fingerprint_or_missing(path)!=before) return "已加载的资源已改变: "+path.filename().string();
        const auto candidate=candidate_directory/path.lexically_relative(startup_directory);
        if(fingerprint_or_missing(candidate)!=before) return "持久配置中的资源已改变: "+path.filename().string();
    }
    return {};
}
}
std::mutex &access_mutex() { static std::mutex mutex;return mutex; }

void capture_startup_resources(const std::string &directory) {
    std::lock_guard lock(access_mutex());
    startup_resources.clear();
    startup_directory=std::filesystem::absolute(directory);
    for(const char *name:{"Robot_Model.yaml","arm_binding.yaml","arm_model_metadata.yaml",
        "arm_frames.yaml","arm_joint_conventions.yaml","realsense_cameras.yaml","devices.json",
        "left_hand_to_base_result.yaml","right_hand_to_base_result.yaml","camera_to_base_result.yaml",
        "head/extrinsics.yaml","head/geometry.yaml","vision/pose_params.yaml",
        "vision/tray.yaml","vision/belt.yaml","vision/marker.yaml"}) {
        const auto path=std::filesystem::absolute(std::filesystem::path(directory)/name);
        startup_resources[path]=fingerprint_or_missing(path);
    }
    const auto geometry=std::filesystem::path(directory)/"head/geometry.yaml";
    if(std::filesystem::is_regular_file(geometry)) {
        const auto doc=YAML::LoadFile(geometry.string());
        const auto name=doc["camera_to_head_yaml"].as<std::string>("extrinsics.yaml");
        const auto path=std::filesystem::absolute(std::filesystem::path(directory)/"head"/name);
        startup_resources[path]=fingerprint_or_missing(path);
    }
}

ReloadResult reload_when_idle(std::mutex &hardware_mutex,
    const std::function<bool()> &calibration_active,const std::function<bool()> &stopping,
    const std::string &path,MoveBoxConfig &current,const std::string &publish_path) {
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
        if(const auto reason=restart_reason(current,next,std::filesystem::absolute(path).parent_path());!reason.empty())
            return {false,"restart_required",reason+"，下次启动将加载新配置"};
    } catch(const std::exception &ex) {return {false,"resource_check_failed",ex.what()};}
    if(stopping()) return {false,"stopping","机器人服务正在退出"};
    if(!publish_path.empty() && std::filesystem::absolute(path)!=std::filesystem::absolute(publish_path)) {
        try {
            std::vector<std::pair<std::filesystem::path,std::filesystem::path>> files;
            const auto heights=std::filesystem::path(path).parent_path()/"conveyor_heights.yaml";
            if(std::filesystem::is_regular_file(heights))files.emplace_back(heights,std::filesystem::path(publish_path).parent_path()/heights.filename());
            files.emplace_back(path,publish_path);
            // Hardware/config locks cover publication and the in-memory commit.
            // Stage all files first; roll back earlier renames on any failure.
            publish_snapshots(files);
        } catch(const std::exception &ex) {
            return {false,"snapshot_write_failed",ex.what()};
        }
    }
    current=std::move(next);
    return {true,"reloaded",{}};
}
}
