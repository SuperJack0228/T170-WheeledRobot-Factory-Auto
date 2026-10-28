// Integration regression with an explicitly synthetic plant and SDK adapters.
// Exercises the production runtime/queues/history/PD/limits together; not T170 kinematics.
#include "servo/runtime.h"
#include "function.h"
#include "move_box_config.h"
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>

MoveBoxConfig g_move_cfg;
namespace {
struct Plant {
    std::mutex mutex;
    wrist_servo::Joints actual=wrist_servo::Joints::Zero(), command=actual;
    double time=0;
};
Plant plants[2];
std::atomic<bool> abort_test{false};
std::atomic<bool> unsafe_reference{false};
}
Robot_Arm::Robot_Arm(const std::string &,const std::string &,const std::string &side,const std::string &) {
    can_id_list={side=="left"?23:16};
}
MatrixXd TR(Matrix<double,1,6> p) {
    Eigen::Matrix4d t=Eigen::Matrix4d::Identity();
    t.block<3,1>(0,3)=p.head<3>().transpose(); return t;
}
Matrix<double,1,6> arm_tcp_from_joints(Robot_Arm &,const Eigen::Ref<const Eigen::RowVectorXd> &q) {
    Matrix<double,1,6> p=Matrix<double,1,6>::Zero(); p.head<3>()=q.head<3>(); return p;
}
std::string project_root_dir() {
    return (std::filesystem::temp_directory_path()/"t170_servo_runtime_model_test").string();
}
std::string default_left_hand_to_robot_yaml_path() { return "unused_synthetic_test"; }
std::string default_right_hand_to_robot_yaml_path() { return "unused_synthetic_test"; }
bool load_cam2robot_matrix(const std::string &,std::array<double,16> &,std::string &) { return false; }
bool hardware_abort_requested() { return abort_test.load(); }
bool can_io_faulted() { return false; }

namespace wrist_servo {
int hand_index(const Robot_Arm &arm) { return arm.can_id_list[0]==23?0:1; }
Eigen::Matrix4d tool_transform(const Robot_Arm &) { return Eigen::Matrix4d::Identity(); }
bool read_joint_feedback(int hand,Joints &q) {
    auto &p=plants[hand]; std::lock_guard<std::mutex> lock(p.mutex);
    const double now=monotonic_seconds(),dt=p.time>0?now-p.time:0;
    p.actual+=(1-std::exp(-dt/.30))*(p.command-p.actual); p.time=now; q=p.actual; return true;
}
void send_joint_reference(int hand,const Joints &q) {
    auto &p=plants[hand]; std::lock_guard<std::mutex> lock(p.mutex);
    if(q(2)<0) unsafe_reference.store(true);
    p.command=q;
}
bool solve_local_pose(Robot_Arm &,const Joints &seed,const Vec3 &position,const Eigen::Matrix3d &,
                      const Config &,Joints &out,double &xyz,double &rotation) {
    out=seed; out.head<3>()=position.transpose(); xyz=0; rotation=0; return true;
}
bool write_config_snapshot(const Config &,const std::string &path,std::string &) {
    std::ofstream file(path); file << "synthetic_runtime_test: true\n"; return bool(file);
}
}

int main() {
    using namespace wrist_servo;
    Robot_Arm left("synthetic","synthetic","left"),right("synthetic","synthetic","right");
    Robot_Arm *arms[]={&left,&right};
    for(auto &profile:g_move_cfg.visual_servo.profiles) {
        profile.mode="table";
        MapKnot a,b; a.height_m=-.01; b.height_m=.16;
        a.reference_px << 640,360; b.reference_px=a.reference_px;
        a.image_jacobian=-10000*Mat2::Identity(); b.image_jacobian=a.image_jacobian;
        profile.knots={a,b};
    }
    Vec2 targets[2]; Joints starts[2];
    Matrix<double,1,6> goals[2];
    prepare_group();
    for(int h=0;h<2;++h) {
        starts[h]=Joints::Zero(); starts[h].head<3>() << .45,h==0?.16:-.16,.12;
        plants[h].actual=starts[h]; plants[h].command=starts[h]; plants[h].time=monotonic_seconds();
        targets[h]=starts[h].head<2>().transpose()+Vec2(h==0?.012:-.010,h==0?-.006:.005);
        goals[h]=Matrix<double,1,6>::Zero(); goals[h].head<2>()=starts[h].head<2>();
        if(!begin_session(*arms[h],goals[h],0)) return 2;
    }
    std::atomic<bool> done{false};
    std::thread vision([&] {
        struct Pending { int hand; double ready; PixelObservation observation; };
        std::deque<Pending> pending; std::uint64_t frames[2]{};
        double next=monotonic_seconds(); int hand=0;
        while(!done.load()) {
            const double now=monotonic_seconds();
            if(now>=next) {
                ObservationContext ctx;
                if(observation_context(hand,ctx) && ctx.descending) {
                    PixelObservation o; o.session=ctx.session; o.frame=++frames[hand];
                    o.capture_s=ctx.latest.time_s; o.completed_s=now+.10; o.confidence=.95;
                    o.pixel=Vec2(640,360)-10000*(ctx.latest.position.head<2>()-targets[hand]);
                    o.fx=o.fy=800; o.cx=640; o.cy=360;
                    pending.push_back({hand,now+.10,o});
                }
                hand=1-hand; next=now+.08;
            }
            while(!pending.empty() && pending.front().ready<=now) {
                submit_observation(pending.front().hand,pending.front().observation); pending.pop_front();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    });
    int result[2]={-99,-99};
    std::thread workers[2];
    for(int h=0;h<2;++h) workers[h]=std::thread([&,h] {
        try { result[h]=run_descent(*arms[h],goals[h],starts[h],starts[h],.03); }
        catch(const std::exception &ex) { std::cerr << ex.what() << '\n'; result[h]=-99; }
        end_session(h,result[h]==0);
    });
    for(auto &thread:workers) thread.join();
    done.store(true); vision.join();
    for(int h=0;h<2;++h) {
        Joints q; read_joint_feedback(h,q);
        const double residual=(q.head<2>().transpose()-targets[h]).norm();
        double x=0,y=0;
        if(result[h]!=0 || residual>.0025 || !completed_xy(h,x,y)) {
            std::cerr << "runtime FAIL hand=" << h << " code=" << result[h] << " residual=" << residual << '\n'; return 1;
        }
    }
    if(unsafe_reference.load()) { std::cerr << "runtime FAIL: reference crossed C\n"; return 1; }
    if(!ready_for_gripper(left) || !ready_for_gripper(right)) {
        std::cerr << "runtime FAIL: stable dual result not available at gripper gate\n"; return 1;
    }
    // A subsequently changed pose must not inherit the earlier permission to close.
    Joints saved;
    { std::lock_guard<std::mutex> lock(plants[0].mutex);
      saved=plants[0].actual; plants[0].actual(2)=-.001; plants[0].command=plants[0].actual;
    }
    if(ready_for_gripper(left) || !group_is_failed()) {
        std::cerr << "runtime FAIL: gripper gate accepted a pose below C\n"; return 1;
    }
    { std::lock_guard<std::mutex> lock(plants[0].mutex);
      plants[0].actual=saved; plants[0].command=saved;
    }
    // A new session with no observations must fail and must not expose the previous success.
    prepare_group();
    goals[0](2)=-.03;
    if(!begin_session(left,goals[0],0)) return 2;
    Joints q; read_joint_feedback(0,q);
    result[0]=run_descent(left,goals[0],q,q,.03); end_session(0,result[0]==0);
    double x=0,y=0;
    if(result[0]==0 || completed_xy(0,x,y)) {
        std::cerr << "runtime FAIL: missing vision reused a previous success\n"; return 1;
    }
    if(last_failure(0)!="no_valid_observation_timeout") {
        std::cerr << "runtime FAIL: initiating fault not exposed\n"; return 1;
    }
    std::cout << "visual_servo_runtime: dual delayed vision + 300ms position following, bounded Z and missing-vision failure passed (synthetic adapters)\n";
    return 0;
}
