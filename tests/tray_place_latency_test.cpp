#include "t170/control/visual_servo.h"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <deque>
#include <stdexcept>
using namespace wrist_servo;
void require(bool ok,const char *s) {if(!ok) throw std::runtime_error(s);}
PixelObservation image(double capture,std::uint64_t frame) {
    PixelObservation o;o.frame=frame;o.capture_s=capture;o.clock_uncertainty_s=.06;o.confidence=.95;o.virtual_target=true;return o;
}
void timing_plant(const Config &cfg,const YAML::Node &session) {
    Controller c;State state;state.time_s=1;state.position=Vec3(0,0,.14);
    c.reset(cfg,state,0,.08,Vec2::Zero(),false,true);
    const Vec2 goal=session["hand"].as<int>()==0?Vec2(.015,.006):Vec2(-.008,.025);
    Vec3 reference=state.position;StateHistory history;history.push(state);
    std::size_t event=0;int accepted=0;bool static_motion=false,done=false;
    for(int tick=1;tick<1500;++tick) {
        const Vec3 previous=state.position;
        state.position+=(1-std::exp(-.01/.08))*(reference-state.position);
        state.velocity=(state.position-previous)/.01;state.time_s=1+tick*.01;history.push(state);
        while(event<session["events"].size() && state.time_s>=1+session["events"][event]["arrival_s"].as<double>()) {
            const auto e=session["events"][event++];
            if(c.phase()==Phase::Commit || c.phase()==Phase::Settling || c.phase()==Phase::Done) continue;
            const auto capture_s=1+e["capture_s"].as<double>();State capture;
            require(history.interpolate(capture_s,.04,capture),"synthetic capture history missing");
            auto o=image(capture_s,e["frame"].as<std::uint64_t>());o.clock_uncertainty_s=e["uncertainty_s"].as<double>();
            Mapping m;m.valid=true;m.remaining_m=goal-capture.position.head<2>();
            c.tray_view_status(state.time_s,false);std::string reason;
            require(c.observe(o,capture,m,state.time_s,reason),"recorded field timing rejected a valid new observation");++accepted;
        }
        const auto out=c.tick(state,.01);reference=out.reference;
        require(out.phase!=Phase::Failed,"recorded 0.5s/0.4s timing caused alignment failure");
        require(out.reason!="place_B_recovery_lift_align_XY","processing gaps caused a false observation lift");
        require(out.reference.z()>=cfg.place_height_margin_m-1e-9,"latency handling changed safe C height");
        static_motion|=out.static_goal_motion;
        if(out.phase==Phase::Done) {done=true;break;}
    }
    require(done && accepted>=2 && static_motion,"delayed independent images did not complete smooth bounded placement");
    require((state.position.head<2>()-goal).norm()<=cfg.place_xy_accept_m,"synthetic placement completed outside XY envelope");
    std::cout<<"field timing session="<<session["session"].as<int>()<<" completed t="<<state.time_s-1<<" accepted="<<accepted<<'\n';
}
void terminal_budget(Config cfg) {
    Controller c;State state;state.time_s=1;state.position=Vec3(0,0,.14);c.reset(cfg,state,0,.08,Vec2::Zero(),false,true);
    bool extended=false,locked=false;std::string reason;
    for(int i=1;i<850;++i) {
        state.time_s=1+i*.01;
        if(i==580 || i==640) {
            Mapping m;m.valid=true;m.remaining_m=Vec2(.003,0);
            require(c.observe(image(state.time_s-.35,i),state,m,state.time_s,reason),"terminal independent frame rejected");
        }
        const auto out=c.tick(state,.01);extended|=out.alignment_extended;
        require(out.phase!=Phase::Failed,"near-target progress did not receive bounded confirmation time");
        if(i<640) require(!out.place_locked,"one image authorized lowering");
        if(out.place_locked) {locked=true;break;}
    }
    require(extended && locked,"two fresh terminal observations failed to lock after original deadline");
    Controller missing;state.time_s=1;missing.reset(cfg,state,0,.08,Vec2::Zero(),false,true);
    for(int i=1;i<850;++i) {
        state.time_s=1+i*.01;
        if(i==580) {Mapping m;m.valid=true;m.remaining_m=Vec2(.003,0);missing.observe(image(state.time_s-.35,i),state,m,state.time_s,reason);}
        const auto out=missing.tick(state,.01);require(!out.place_locked,"no independent confirmation became a blind placement");
    }
    require(missing.phase()==Phase::Failed,"confirmation extension became an infinite retry");
    Controller no_view;state.time_s=1;no_view.reset(cfg,state,0,.08,Vec2::Zero(),false,true);
    for(int i=1;i<500;++i) {state.time_s=1+i*.01;no_view.tray_view_status(state.time_s,false);
        const auto out=no_view.tick(state,.01);require(out.reference.z()==.14,"visible but late frames caused an upward search");}
}
int main(int argc,char **argv) {
    try {
        require(argc==3,"config and timing fixture required");Config cfg;load_config(YAML::LoadFile(argv[1])["visual_servo"],cfg);
        const auto fixture=YAML::LoadFile(argv[2]);
        for(const auto &session:fixture["sessions"]) timing_plant(cfg,session);
        terminal_budget(cfg);
        std::cout<<"tray latency regression passed (recorded timing with synthetic mechanics, not robot accuracy)\n";
    } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
}
