#include "servo/visual_servo.h"
#include "servo/trace.h"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <tuple>

using namespace wrist_servo;
void check(bool value,const char *message) { if(!value) throw std::runtime_error(message); }
void near(double a,double b,double tolerance,const char *message) { check(std::abs(a-b)<=tolerance,message); }
State state_at(double t,Vec3 position=Vec3::Zero()) { State s; s.time_s=t; s.position=position; return s; }
PixelObservation observation_at(double t,std::uint64_t frame) {
    PixelObservation o; o.capture_s=t; o.completed_s=t+0.1; o.frame=frame; o.confidence=0.9;
    o.fx=800; o.fy=800; o.cx=640; o.cy=360; o.width=1280; o.height=720; return o;
}

void test_mapping() {
    ImageProfile p; p.reference_px << 680,380;
    State s=state_at(1,Vec3(0,0,0.08));
    const Eigen::Matrix3d R=Eigen::AngleAxisd(M_PI,Vec3::UnitX()).toRotationMatrix();
    // Camera is offset from TCP and 80mm above the feature at final TCP Z.
    Vec3 origin(0,0,0.16);
    auto o=observation_at(1,1);
    o.pixel << 660,370;
    auto m=map_observation(p,o,s,0,origin,R);
    check(m.valid,"geometry map valid");
    near(m.reference_px.x(),660,1e-8,"height-dependent reference u");
    near(m.reference_px.y(),370,1e-8,"height-dependent reference v");
    near(m.remaining_m.norm(),0,1e-10,"correct XY at different height has zero error");
    o.pixel.x()+=100;
    m=map_observation(p,o,s,0,origin,R);
    check(m.valid,"100px map valid");
    near(m.remaining_m.x(),0.020,1e-8,"100px at 160mm depth maps to 20mm");
    o.pixel.y()+=50;
    m=map_observation(p,o,s,0,origin,R);
    near(m.remaining_m.y(),-0.010,1e-8,"vertical pixel error is controlled");
    origin.z()=0.001;
    check(!map_observation(p,o,s,0,origin,R).valid,"invalid depth rejected instead of clamped");
    p.mode="table";
    MapKnot a,b; a.height_m=0; b.height_m=.1;
    a.reference_px << 600,400; b.reference_px << 620,420;
    a.image_jacobian << -10000,2000,1000,9000;
    b.image_jacobian=.5*a.image_jacobian; p.knots={a,b};
    s.position.z()=.05; o.pixel << 650,430;
    m=map_observation(p,o,s,0,origin,R);
    check(m.valid,"table map valid");
    near(m.reference_px.x(),610,1e-8,"table interpolation");
    near((m.jacobian*m.remaining_m+m.error_px).norm(),0,1e-8,"coupled 2D inverse");
    s.position.z()=.2;
    check(!map_observation(p,o,s,0,origin,R).valid,"no table extrapolation");
}

void test_history_and_delay() {
    StateHistory history;
    history.push(state_at(1,Vec3(0,0,.1)));
    history.push(state_at(1.02,Vec3(.002,0,.1)));
    State capture;
    check(history.interpolate(1.01,.05,capture),"state interpolation available");
    near(capture.position.x(),.001,1e-12,"interpolated measured state");
    check(!history.interpolate(.9,.05,capture),"past state never invented");
    check(!history.interpolate(1.03,.05,capture),"future state never invented");
    check(!history.interpolate(1.01,.005,capture),"large sampling gaps rejected");
    Config cfg; Controller c; State start=state_at(1,Vec3(0,0,.1)); c.reset(cfg,start,0,.03);
    auto obs=observation_at(1,1); Mapping map; map.valid=true; map.remaining_m << .010,0;
    std::string reason;
    check(c.observe(obs,start,map,1.1,reason),"delayed observation accepted");
    State now=state_at(1.1,Vec3(.006,0,.1));
    const auto out=c.tick(now,.01);
    near(out.remaining_m.x(),.004,1e-10,"already completed motion subtracted");
    check(!c.observe(obs,start,map,1.1,reason),"same frame not consumed twice");
    obs.frame=2; obs.capture_s=.5;
    check(!c.observe(obs,start,map,1.1,reason),"out-of-order frame rejected");
    obs.frame=3; obs.capture_s=1.11; map.remaining_m << .03,0;
    check(!c.observe(obs,now,map,1.12,reason),"target jump rejected");
}

void test_tilted_geometry() {
    for(double yaw:{-.7,.7}) {
        const Eigen::Matrix3d R=(Eigen::AngleAxisd(yaw,Vec3::UnitZ()) *
            Eigen::AngleAxisd(M_PI-.08,Vec3::UnitX())).toRotationMatrix();
        const Vec3 offset(.025,-.015,.09), feature(.45,.1,0);
        PixelObservation o=observation_at(1,1);
        auto project=[&](const Vec3 &origin) -> Vec2 {
            const Vec3 point=R.transpose()*(feature-origin);
            return Vec2(o.cx+o.fx*point.x()/point.z(),o.cy+o.fy*point.y()/point.z());
        };
        ImageProfile p; p.reference_px=project(Vec3(.45,.1,0)+offset);
        const Vec3 tcp(.446,.106,.07), origin=tcp+offset;
        State s=state_at(1,tcp); o.pixel=project(origin);
        const auto m=map_observation(p,o,s,0,origin,R);
        check(m.valid,"tilted camera geometry valid");
        const Vec2 expected=feature.head<2>()-tcp.head<2>();
        check((m.remaining_m-expected).norm()<.0002,"tilted 2D map handles axis coupling and height");
    }
}

void test_commit_evidence() {
    Config cfg; Controller c; State s=state_at(1,Vec3(0,0,.0085)); c.reset(cfg,s,0,.03);
    Mapping m; m.valid=true; m.remaining_m.setZero(); std::string reason;
    auto o=observation_at(1,1);
    check(c.observe(o,s,m,1.01,reason),"first aligned observation");
    s.time_s=1.01;
    check(c.tick(s,.01).phase!=Phase::Commit,"one frame cannot authorize final stroke");
    for(int i=0;i<10;++i) c.tick(s,.01);
    check(c.phase()!=Phase::Commit,"motion ticks cannot manufacture visual confirmations");
    s.time_s=1.10; o=observation_at(1.10,2);
    check(c.observe(o,s,m,1.11,reason),"second independent aligned observation");
    s.time_s=1.11;
    check(c.tick(s,.01).phase==Phase::Commit,"two new frames and settled XY authorize bounded stroke");
    // Missing images during expected last-stroke occlusion are allowed only after COMMIT.
    s.time_s=1.6;
    check(c.tick(s,.01).phase==Phase::Commit,"bounded commit tolerates expected occlusion");
    c.fail("initiating_fault"); c.fail("peer_arm_failed");
    check(c.tick(s,.01).reason=="initiating_fault","root failure reason retained");
}

void test_braking_and_loss() {
    const double v=braking_speed(.003,.1,.15);
    near(v*.15+v*v/(2*.1),.003,1e-12,"braking envelope includes delay");
    Config cfg; Controller c;
    State s=state_at(1,Vec3(0,0,.02)); c.reset(cfg,s,0,.03);
    auto out=c.tick(s,.01);
    check(out.reference.z()>=.008 && out.velocity_z>=-.03,"pre-vision descent remains above acquisition guard");
    s.time_s=3;
    check(c.tick(s,.01).phase==Phase::Failed,"no observation times out");
    c.reset(cfg,state_at(1,Vec3(0,0,.02)),0,.03);
    auto obs=observation_at(1,1); Mapping m; m.valid=true; m.remaining_m << .02,0;
    std::string reason; check(c.observe(obs,state_at(1,Vec3(0,0,.02)),m,1.1,reason),"large error accepted in envelope");
    s=state_at(1.1,Vec3(0,0,.02)); out=c.tick(s,.01);
    check(out.descent_scale<1,"remaining alignment time slows descent for large XY error");
    s.time_s=1.6; out=c.tick(s,.01);
    check(out.phase==Phase::Hold,"expired vision holds");
    s.time_s=3; check(c.tick(s,.01).phase==Phase::Failed,"lost vision fails rather than blind grasp");
}

void test_local_pose_solver() {
    Config cfg;
    const ForwardModel forward=[](const Joints &q) {
        Eigen::Matrix4d t=Eigen::Matrix4d::Identity();
        t.block<3,3>(0,0)=(Eigen::AngleAxisd(q(3),Vec3::UnitZ())*
            Eigen::AngleAxisd(q(4),Vec3::UnitY())*Eigen::AngleAxisd(q(5),Vec3::UnitX())).toRotationMatrix();
        t.block<3,1>(0,3) << q(0)+.1*std::sin(q(3))+.05*std::sin(q(6)),
                             q(1)+.1*std::sin(q(4)),q(2)+.1*std::sin(q(5));
        return t;
    };
    const JointLimitCheck limits=[](const Joints &q) { return q.cwiseAbs().maxCoeff()>1.6; };
    Joints seed; seed << .1,.2,.3,.2,-.3,.1,.4;
    const auto initial=forward(seed); const Vec3 goal=initial.block<3,1>(0,3)+Vec3(.003,-.002,0);
    Joints solution; double xyz=0,rotation=0;
    check(solve_pose_with_fk(seed,goal,initial.block<3,3>(0,0),cfg,forward,limits,solution,xyz,rotation),
          "6D local solver finds a nearby pose");
    const auto final=forward(solution);
    check((final.block<3,1>(0,3)-goal).norm()<cfg.solver_position_tolerance_m,"solver checks true translation residual");
    check(Eigen::AngleAxisd(final.block<3,3>(0,0)*initial.block<3,3>(0,0).transpose()).angle()<cfg.solver_rotation_tolerance_rad,
          "XY correction preserves orientation despite translation/rotation coupling");
    check(!solve_pose_with_fk(seed,Vec3(10,10,10),initial.block<3,3>(0,0),cfg,forward,limits,solution,xyz,rotation),
          "nonconverged finite joint vector is not success");
}

void test_delayed_dual_arm_simulation() {
    Config cfg; cfg.task_timeout_s=25;
    struct Arm {
        Controller controller;
        State state;
        Vec2 target;
        std::deque<std::pair<double,std::pair<PixelObservation,std::pair<State,Mapping>>>> pending;
        int frames=0, accepted=0; double next_image=0;
        bool done=false; double max_speed=0, min_x=1, max_x=-1;
    } arms[2];
    for(int h=0;h<2;++h) {
        auto &a=arms[h]; a.state=state_at(1,Vec3(0,h==0?.15:-.15,.12));
        a.target=a.state.position.head<2>()+Vec2(h==0?.018:-.014,h==0?-.008:.010);
        a.controller.reset(cfg,a.state,0,.03); a.next_image=1;
    }
    for(int tick=0;tick<2500;++tick) {
        const double t=1+tick*.01;
        for(int h=0;h<2;++h) {
            auto &a=arms[h]; if(a.done) continue;
            a.state.time_s=t;
            // Alternating 6.25Hz per hand, asymmetric 120/170ms latency and mild feature noise.
            if(t>=a.next_image) {
                auto obs=observation_at(t,++a.frames);
                Mapping m; m.valid=true;
                m.remaining_m=a.target-a.state.position.head<2>();
                m.remaining_m.x()+=.0001*std::sin(t*7);
                a.pending.push_back({t+(h==0?.12:.17),{obs,{a.state,m}}});
                a.next_image+=.16;
            }
            while(!a.pending.empty() && a.pending.front().first<=t) {
                auto item=a.pending.front().second; a.pending.pop_front();
                std::string reason;
                if(a.controller.observe(item.first,item.second.first,item.second.second,t,reason)) ++a.accepted;
            }
            const auto out=a.controller.tick(a.state,.01);
            check(out.phase!=Phase::Failed,"dual simulation controller does not fail");
            check(out.velocity_xy.norm()<=cfg.max_xy_speed_m_s+1e-9,"XY speed limit");
            if(out.phase==Phase::Done) { a.done=true; continue; }
            // First-order response represents actuator following; controller uses measured state.
            Vec3 desired(out.velocity_xy.x(),out.velocity_xy.y(),out.velocity_z);
            a.state.velocity+=.4*(desired-a.state.velocity);
            a.state.position+=.01*a.state.velocity;
            a.max_speed=std::max(a.max_speed,a.state.velocity.head<2>().norm());
            a.min_x=std::min(a.min_x,a.state.position.x()); a.max_x=std::max(a.max_x,a.state.position.x());
        }
        if(arms[0].done && arms[1].done) break;
    }
    for(int h=0;h<2;++h) {
        const auto &a=arms[h];
        check(a.done,"both delayed arms reach final state");
        check((a.target-a.state.position.head<2>()).norm()<.0025,"final XY within configured model tolerance");
        check(a.state.position.z()>=0 && a.state.position.z()<=cfg.grasp_z_accept_above_m,"final Z is one-sided above C");
        check(a.accepted>2,"continuous correction exceeds old two-shot limit");
        std::cout << "sim hand=" << h << " accepted=" << a.accepted
                  << " residual_mm=" << (a.target-a.state.position.head<2>()).norm()*1000 << '\n';
    }
}

void test_trace_and_config() {
    Config c; std::string error; check(validate_config(c,error),"default config valid");
    c.kp=NAN; check(!validate_config(c,error),"NaN gains rejected");
    c=Config(); c.profiles[0].mode="table";
    check(!validate_config(c,error),"empty measured table rejected");
    const auto dir=std::filesystem::temp_directory_path()/"t170_servo_trace_test";
    std::string file;
    { TraceWriter writer; check(writer.start(dir.string()),"trace starts"); file=writer.path();
      TraceRow row; row.time_s=monotonic_seconds(); row.event="end"; row.reason="done,checked";
      writer.push(row);
    }
    std::ifstream in(file); std::string header,line; std::getline(in,header); std::getline(in,line);
    check(!line.empty(),"writer drains on shutdown");
    check(std::count(header.begin(),header.end(),',')==std::count(line.begin(),line.end(),','),"CSV schema consistent");
    std::filesystem::remove(file);
}

// Position-command plant (not a velocity plant): reproduces the field's ~9mm
// following lead at 30mm/s with a 300ms response. XY can be intentionally blocked.
void test_position_following_and_grasp_tolerance() {
    for(const auto &[residual,grasp,should_finish]:
        std::vector<std::tuple<double,bool,bool>>{{.004,true,true},{.009,true,false},{.004,false,false}}) {
        Config cfg; Controller c; State s=state_at(1,Vec3(0,0,.13));
        c.reset(cfg,s,0,.03,Vec2::Zero(),grasp);
        bool done=false; std::uint64_t frame=0; double next_frame=1;
        for(int tick=0;tick<1900;++tick) {
            s.time_s=1+tick*.01;
            if(s.time_s>=next_frame && c.phase()!=Phase::Failed) {
                auto o=observation_at(s.time_s,++frame); Mapping m; m.valid=true;
                m.remaining_m=Vec2(residual,0); std::string reason;
                c.observe(o,s,m,s.time_s,reason); next_frame+=.13;
            }
            const auto out=c.tick(s,.01);
            check(out.reference.z()>=(grasp?cfg.grasp_height_margin_m:0.0)-1e-10,"reference never crosses one-sided clearance");
            check(out.reference.x()<=residual+1e-8,"XY reference cannot wind up beyond the static target");
            if(out.phase==Phase::Done) {
                check(out.velocity_xy.norm()==0 && out.velocity_z==0,"done carries zero motion command");
                check(s.position.z()>=0,"measured completion is not below C");
                check(out.relaxed_grasp,"4mm residual uses explicit grasp tolerance");
                done=true; break;
            }
            if(out.phase==Phase::Failed) break;
            const Vec3 previous=s.position;
            s.position.z()+=(1-std::exp(-.01/.30))*(out.reference.z()-s.position.z());
            // Fixed 4/9mm residual represents an uncorrectable small model/mechanical offset.
            s.velocity=(s.position-previous)/.01;
        }
        check(done==should_finish,"grasp may accept 4mm, but 9mm and strict place must not blind-close");
    }
}

int main() {
    try {
        test_mapping(); test_tilted_geometry(); test_history_and_delay(); test_braking_and_loss();
        test_commit_evidence(); test_local_pose_solver();
        test_delayed_dual_arm_simulation(); test_trace_and_config();
        test_position_following_and_grasp_tolerance();
        std::cout << "visual_servo: all regression checks passed (model tests, not robot validation)\n";
        return 0;
    } catch(const std::exception &ex) { std::cerr << "FAIL: " << ex.what() << '\n'; return 1; }
}
