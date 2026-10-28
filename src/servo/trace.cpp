#include "servo/trace.h"
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <unistd.h>

namespace wrist_servo {
namespace {
std::string cell(std::string s) {
    for (char &c:s) if (c==',' || c=='\n' || c=='\r') c=' ';
    return s;
}
}
TraceWriter &trace_writer() { static TraceWriter writer; return writer; }
TraceWriter::~TraceWriter() {
    { std::lock_guard<std::mutex> lock(mu_); stopping_=true; }
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
}
bool TraceWriter::start(const std::string &dir) {
    std::lock_guard<std::mutex> lock(mu_);
    if (thread_.joinable()) return healthy_.load();
    std::error_code ec; std::filesystem::create_directories(dir,ec);
    if (ec) return false;
    const auto us=std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    path_=(std::filesystem::path(dir)/("servo_"+std::to_string(us)+"_"+std::to_string(getpid())+".csv")).string();
    file_.open(path_);
    if (!file_) return false;
    file_ << "host_epoch_s,steady_s,session,hand,target_class,task_tag,event,reason,map_mode,frame,capture_s,completed_s,clock_uncertainty_ms,device_timestamp_ms,timestamp_source,"
             "fx,fy,cx,cy,image_width,image_height,"
             "grab_ms,infer_ms,confidence,u,v,ref_u,ref_v,du,dv,height_m,depth_m,map_condition,a00,a01,a10,a11,"
             "capture_dx_m,capture_dy_m,goal_x_m,goal_y_m,actual_x_m,actual_y_m,actual_z_m,"
             "actual_vx_m_s,actual_vy_m_s,actual_vz_m_s,remaining_x_m,remaining_y_m,"
             "reference_x_m,reference_y_m,reference_z_m,command_vx_m_s,command_vy_m_s,command_vz_m_s,"
             "phase,observation_age_ms,brake_speed_m_s,descent_scale,confirmed_frames,"
             "dt_ms,read_ms,solve_ms,solver_xyz_m,solver_rotation_rad,solver_failures,dropped_rows,"
             "requested_vx_m_s,requested_vy_m_s,requested_vz_m_s,applied_scale,solver_reason,gripper_position_rad,gripper_torque_nm,"
             "actual_rotation_error_rad,reference_lead_m,joint_lead_rad";
    for(int j=0;j<7;++j) file_ << ",actual_q" << j;
    for(int j=0;j<7;++j) file_ << ",command_q" << j;
    file_ << '\n'; file_.flush();
    if(!file_) return false;
    healthy_.store(true);
    thread_=std::thread(&TraceWriter::run,this);
    std::cout << "[servo] 双手调试日志 " << path_ << std::endl;
    return true;
}
std::string TraceWriter::path() const { std::lock_guard<std::mutex> lock(mu_); return path_; }
void TraceWriter::push(TraceRow r) {
    std::lock_guard<std::mutex> lock(mu_);
    if (!thread_.joinable() || stopping_) return;
    if (queue_.size()>=16384) { ++dropped_; return; }
    queue_.push_back(std::move(r)); cv_.notify_one();
}
void TraceWriter::run() {
    const double epoch_offset=std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count()-monotonic_seconds();
    std::size_t since_flush=0;
    for (;;) {
        TraceRow r; std::uint64_t dropped;
        { std::unique_lock<std::mutex> lock(mu_);
          cv_.wait_for(lock,std::chrono::milliseconds(200),[&]{ return stopping_ || !queue_.empty(); });
          if(queue_.empty()) { file_.flush(); if(stopping_) break; continue; }
          r=std::move(queue_.front()); queue_.pop_front(); dropped=dropped_;
        }
        const auto &o=r.observation; const auto &m=r.mapping; const auto &s=r.measured; const auto &c=r.control;
        file_ << std::fixed << std::setprecision(9) << r.time_s+epoch_offset << ',' << r.time_s << ',' << r.session << ',' << r.hand
              << ',' << r.target_class << ',' << cell(r.task_tag)
              << ',' << cell(r.event) << ',' << cell(r.reason) << ',' << cell(r.profile_mode)
              << ',' << o.frame << ',' << o.capture_s << ',' << o.completed_s << ',' << o.clock_uncertainty_s*1000
              << ',' << o.device_timestamp_ms << ',' << o.timestamp_source
              << ',' << o.fx << ',' << o.fy << ',' << o.cx << ',' << o.cy << ',' << o.width << ',' << o.height
              << ',' << o.grab_ms << ',' << o.infer_ms << ',' << o.confidence
              << ',' << o.pixel.x() << ',' << o.pixel.y() << ',' << m.reference_px.x() << ',' << m.reference_px.y()
              << ',' << m.error_px.x() << ',' << m.error_px.y() << ',' << s.position.z()-r.final_z
              << ',' << m.depth_m << ',' << m.condition << ',' << m.jacobian(0,0) << ',' << m.jacobian(0,1)
              << ',' << m.jacobian(1,0) << ',' << m.jacobian(1,1) << ',' << m.remaining_m.x() << ',' << m.remaining_m.y()
              << ',' << r.goal.x() << ',' << r.goal.y();
        for(int j=0;j<3;++j) file_ << ',' << s.position(j);
        for(int j=0;j<3;++j) file_ << ',' << s.velocity(j);
        for(int j=0;j<2;++j) file_ << ',' << c.remaining_m(j);
        for(int j=0;j<3;++j) file_ << ',' << r.reference(j);
        file_ << ',' << c.velocity_xy.x() << ',' << c.velocity_xy.y() << ',' << c.velocity_z
              << ',' << phase_name(c.phase) << ',' << c.age_s*1000 << ',' << c.brake_speed << ',' << c.descent_scale
              << ',' << c.confirmations << ',' << r.dt_ms << ',' << r.read_ms << ',' << r.solve_ms
              << ',' << r.solver_xyz_m << ',' << r.solver_rotation_rad << ',' << r.solver_failures << ',' << dropped;
        for(int j=0;j<3;++j) file_ << ',' << r.requested_velocity(j);
        file_ << ',' << r.applied_scale << ',' << cell(r.solver_reason);
        file_ << ',' << r.gripper_position_rad << ',' << r.gripper_torque_nm;
        file_ << ',' << r.actual_rotation_error_rad << ',' << r.reference_lead_m << ',' << r.joint_lead_rad;
        for(int j=0;j<7;++j) file_ << ',' << s.joints(j);
        for(int j=0;j<7;++j) file_ << ',' << r.commanded(j);
        file_ << '\n';
        if(++since_flush>=20 || r.event=="end" || r.event=="reject") { file_.flush(); since_flush=0; }
        if(!file_ && healthy_.exchange(false)) std::cerr << "[servo] 调试日志写入失败 " << path_ << '\n';
    }
    file_.flush();
}
}
