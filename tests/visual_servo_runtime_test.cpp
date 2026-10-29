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
#include <condition_variable>
#include <vector>

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
struct HoldRace {
    std::mutex mutex;
    std::condition_variable cv;
    bool enabled=false, entered=false, release=false;
    std::thread::id sender;
    std::vector<wrist_servo::Joints> writes;
} hold_race;
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
    if(hand==0) {
        std::unique_lock<std::mutex> lock(hold_race.mutex);
        if(hold_race.enabled) {
            if(std::this_thread::get_id()==hold_race.sender) {
                hold_race.entered=true; hold_race.cv.notify_all();
                hold_race.cv.wait(lock,[]{ return hold_race.release; });
            }
            hold_race.writes.push_back(q);
        }
    }
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
    // Deterministically freeze an old hold INSIDE the send adapter. The real
    // runtime must not transfer ownership until that packet has finished.
    Joints held;
    if(!peek_final_hold(0,held)) return 1;
    Joints lifted=held; lifted(2)+=.001;
    { std::lock_guard<std::mutex> lock(hold_race.mutex); hold_race.enabled=true; }
    std::thread sender([&] {
        { std::lock_guard<std::mutex> lock(hold_race.mutex); hold_race.sender=std::this_thread::get_id(); }
        repeat_final_hold(0);
    });
    bool entered=false;
    { std::unique_lock<std::mutex> lock(hold_race.mutex);
      entered=hold_race.cv.wait_for(lock,std::chrono::seconds(2),[]{return hold_race.entered;});
    }
    std::atomic<bool> taking{false},taken{false};
    bool take_ok=false;
    std::thread taker([&] {
        taking.store(true);
        Joints transferred; take_ok=take_final_hold(0,transferred);
        if(take_ok) send_joint_reference(0,lifted);
        taken.store(true);
    });
    while(!taking.load()) std::this_thread::yield();
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    const bool premature=taken.load();
    { std::lock_guard<std::mutex> lock(hold_race.mutex); hold_race.release=true; }
    hold_race.cv.notify_all(); sender.join(); taker.join();
    repeat_final_hold(0); // After ownership transfer, repeating is a no-op.
    bool ordered=false;
    { std::lock_guard<std::mutex> lock(hold_race.mutex);
      ordered=hold_race.writes.size()==2 && (hold_race.writes[0]-held).norm()<1e-12 &&
              (hold_race.writes[1]-lifted).norm()<1e-12;
      hold_race.enabled=false;
    }
    if(!entered || premature || !take_ok || !ordered) {
        std::cerr << "runtime FAIL: old hold packet crossed the retract handoff\n"; return 1;
    }
    send_joint_reference(0,held);
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
    // Both hands use class-1 virtual hole centers. The same controller must
    // converge with placement tolerances and expose a stable hold for opening.
    prepare_group(); unsafe_reference.store(false);
    for(int h=0;h<2;++h) {
        starts[h]=Joints::Zero();starts[h].head<3>()<<.45,h==0?.16:-.16,.10;
        {std::lock_guard<std::mutex> lock(plants[h].mutex);
         plants[h].actual=starts[h];plants[h].command=starts[h];plants[h].time=monotonic_seconds();}
        targets[h]=starts[h].head<2>().transpose()+Vec2(h==0?.008:-.008,.004);
        goals[h]=Matrix<double,1,6>::Zero();goals[h].head<2>()=starts[h].head<2>();
        TrayTarget t;t.valid=true;t.row=3;t.col=h==0?4:1;stage_tray_target(h,t);
        if(!begin_session(*arms[h],goals[h],1,"tray_place")) return 2;
        ObservationContext ctx;observation_context(h,ctx);
        if(!ctx.tray_target.valid || ctx.tray_target.row!=3) return 1;
    }
    std::atomic<bool> place_done{false};
    std::thread place_vision([&] {
        struct Pending {int hand;double ready;PixelObservation observation;};
        std::deque<Pending> pending;std::uint64_t frame[2]{};int hand=0;double next=0;
        while(!place_done.load()) {
            const double now=monotonic_seconds();
            if(now>=next) {
                ObservationContext ctx;
                if(observation_context(hand,ctx) && ctx.descending && ctx.vision_required) {
                    PixelObservation o;o.session=ctx.session;o.frame=++frame[hand];
                    o.capture_s=ctx.latest.time_s;o.completed_s=now+.08;o.confidence=.95;
                    o.pixel=Vec2(640,360)-10000*(ctx.latest.position.head<2>()-targets[hand]);
                    o.fx=o.fy=800;o.cx=640;o.cy=360;o.virtual_target=true;
                    o.target_row=3;o.target_col=ctx.tray_target.col;o.neighbor_count=2;o.vision_source="neighbor_translation";
                    pending.push_back({hand,now+.08,o});
                }
                hand=1-hand;next=now+.06;
            }
            while(!pending.empty() && pending.front().ready<=now) {
                submit_observation(pending.front().hand,pending.front().observation);pending.pop_front();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    });
    for(int h=0;h<2;++h) workers[h]=std::thread([&,h] {
        result[h]=run_descent(*arms[h],goals[h],starts[h],starts[h],.05);
        end_session(h,result[h]==0);
    });
    for(auto &thread:workers) thread.join();place_done.store(true);place_vision.join();
    if(result[0]!=0 || result[1]!=0 || !ready_for_gripper(left) || !ready_for_gripper(right)) {
        std::cerr<<"runtime FAIL: dual virtual-hole placement did not converge "<<result[0]<<','<<result[1]<<'\n';return 1;
    }
    {std::lock_guard<std::mutex> lock(plants[0].mutex);plants[0].actual(0)+=.004;plants[0].command=plants[0].actual;}
    if(ready_for_gripper(left)) {std::cerr<<"runtime FAIL: placement inherited relaxed grasp XY gate\n";return 1;}
    std::cout << "visual_servo_runtime: dual delayed vision, bounded Z, missing vision and exclusive hold-to-retract handoff passed (synthetic adapters)\n";
    return 0;
}
