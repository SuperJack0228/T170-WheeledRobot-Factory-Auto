// Run the real dispatch loop with scripted missions; no devices or real SDK.
#include "support/factory_ports.h"
#include "support/reference_arm.h"
#include "../src/tasks/internal/runtime.h"
#include <condition_variable>
#include <deque>
#include <iostream>
#include <stdexcept>
MoveBoxConfig g_move_cfg;
namespace {
std::atomic<bool> aborted{false};
bool io_fault=false;
struct Script {
    std::mutex mutex;std::condition_variable ready;
    std::deque<std::string> missions,replies;
    std::deque<int> results;
    std::vector<std::string> tasks,messages;
    int homes=0,navigations=0,closes=0,map_result=0,motor_calls=0;
    bool closed=false,stop_on_failed_report=false,start_disabled=false,enable_fails=false,fault_on_first_task=false,stop_on_first_task=false;
};
Script *script=nullptr;
void check(bool ok,const char *message){if(!ok)throw std::runtime_error(message);}
struct Client final : factory_test::Client {
    ~Client(){close();}
    std::string apply_mission() override {
        std::unique_lock lock(script->mutex);
        script->ready.wait(lock,[]{return script->closed || !script->missions.empty();});
        if(script->closed)return {};
        auto mission=script->missions.front();script->missions.pop_front();return mission;
    }
    std::string report_exception(const std::string &message) override {
        script->messages.push_back(message);
        auto result=script->replies.empty()?std::string():script->replies.front();
        if(!script->replies.empty())script->replies.pop_front();
        return result;
    }
    void close() noexcept override {
        std::lock_guard lock(script->mutex);
        if(!script->closed){script->closed=true;++script->closes;script->ready.notify_all();}
    }
};
struct Gateway final : factory_test::Gateway {
    std::unique_ptr<t170::integration::dispatch::Client> connect(const std::string &,const std::string &) override {
        return std::make_unique<Client>();
    }
};
int task(const char *name){
    script->tasks.emplace_back(name);
    if(script->fault_on_first_task && script->tasks.size()==1)io_fault=true;
    if(script->stop_on_first_task && script->tasks.size()==1)aborted.store(true);
    check(!script->results.empty(),"unexpected extra task after operator decision");
    int result=script->results.front();script->results.pop_front();return result;
}
}
bool hardware_abort_requested(){return aborted.load();}
bool can_io_faulted(){return io_fault;}
void hardware_abort_sleep_ms(int ms){
    if(ms==500 && script->stop_on_failed_report)aborted.store(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}
namespace move_box {void tray1_far_band_clear(){}}
namespace orch {
bool read_pose_before_dispatch(double &,double &,double &){return true;}
bool stage_dispatch_map_name(){return true;}
int finish_dispatch_map(DispatchClient &,double,double,double){return script->map_result;}
std::string dispatch_idle_station(){return "standby";}
std::string chassis_location_station(){return "here";}
int set_body_motors(bool){++script->motor_calls;if(script->enable_fails)return -1;g_body_disabled_for_charge=false;return 0;}
int go_home(Arm &,Arm &){++script->homes;return 0;}
int go_home_arms_keep_waist_z(Arm &,Arm &){++script->homes;return 0;}
int chassis_goto_station(const std::string &,bool,const std::function<void()> &){++script->navigations;return 0;}
void boot_announce(const std::string &){}
bool ensure_chassis(){return false;}
Chassis &chassis_seer(){return t170::tasks::services().chassis;}
void publish_dispatch_battery(double){}
std::string dispatch_exception_state(){return io_fault?dispatch_msg::ERR_MOTOR_LOST:"";}
int dispatch_wait_system(DispatchClient &client,const std::string &state){
    const int choice=dispatch_report_ex(client,state.c_str());if(choice==0)io_fault=false;return choice;
}
int dispatch_load(DispatchClient &){return task("load");}
int dispatch_transfer(DispatchClient &){return task("transfer");}
int dispatch_unload(DispatchClient &){return task("unload");}
int dispatch_charge(DispatchClient &){return task("charge");}
}
int main(){try {
    using namespace orch;
    setenv("T170_TIMING","0",1);setenv("T170_DISPATCH_VERBOSE","1",1);
    g_move_cfg.chassis.robot_type="T170C2.0";g_move_cfg.chassis.sequence_number="test-120";
    t170::motion::ArmDefinition definition;
    Arm arm(definition,std::make_shared<ReferenceBackend>(definition));
    HwSession hardware;hardware.arm_l=hardware.arm_r=&arm;g_hw=&hardware;
    factory_test::Chassis chassis;Gateway gateway;
    t170::tasks::Services services{chassis,gateway};t170::tasks::bind_services(&services);
    auto run=[&](Script &s){script=&s;aborted.store(false);io_fault=false;g_body_disabled_for_charge=s.start_disabled;
        hardware.last_grasped_l=true;return run_dispatch_mode();};
    {
        Script s;s.missions={dispatch_msg::TASK_LOAD,dispatch_msg::TASK_TRANSFER};
        s.results={kDispatchTaskSkip,kDispatchOffline};
        check(run(s)==0 && s.tasks==std::vector<std::string>{"load","transfer"},"SKIP did not wait for a new mission");
        check(s.homes==1 && !s.navigations && s.closes==1 && hardware.last_grasped_l,
              "SKIP/OFFLINE moved robot, dropped held-state or failed to close");
        check(dispatch_board_snapshot().tasks_done==0 && !dispatch_board_snapshot().active,"skipped task marked completed");
    }
    for(int decision:{kDispatchTaskSkip,kDispatchOffline,kDispatchReportFailed}) {
        Script s;s.missions={dispatch_msg::TASK_CHARGE};s.results={decision};
        if(decision==kDispatchTaskSkip){s.missions.push_back(dispatch_msg::TASK_LOAD);s.results.push_back(kDispatchOffline);}
        s.stop_on_failed_report=decision==kDispatchReportFailed;
        check(run(s)==(s.stop_on_failed_report?-4:0),"charge operator decision lost");
        check(s.homes==1 && !s.navigations && s.closes==1,"charge choice triggered recovery movement");
        check(dispatch_board_snapshot().tasks_done==0,"charge skip/offline became completion");
    }
    {
        Script s;s.missions={dispatch_msg::TASK_LOAD};s.results={kVisionQrFail,kDispatchOffline};s.replies={"RETRY"};
        check(run(s)==0 && s.tasks==std::vector<std::string>{"load","load"} && s.messages.size()==1,
              "RETRY did not retry the interrupted task");
        check(s.homes==1 && !s.navigations,"RETRY unnecessarily ran recovery movement");
    }
    {
        Script s;s.missions={dispatch_msg::TASK_LOAD};s.results={-4,kDispatchOffline};
        s.replies={"RETRY"};s.fault_on_first_task=true;
        check(run(s)==0 && s.tasks.size()==2 && s.messages.size()==1 && !aborted.load(),
              "CAN failure -4 was treated as operator cancellation");
    }
    {
        Script s;s.missions={dispatch_msg::TASK_LOAD};s.results={-4};
        s.fault_on_first_task=s.stop_on_first_task=true;
        check(run(s)==-4 && s.tasks.size()==1 && s.messages.empty() && !s.navigations,
              "operator stop lost priority over CAN failure");
    }
    for(int decision:{kDispatchTaskSkip,kDispatchOffline,kDispatchReportFailed}) {
        Script s;s.map_result=decision;s.stop_on_failed_report=decision==kDispatchReportFailed;
        check(run(s)==(s.stop_on_failed_report?-4:0) && s.tasks.empty() && !s.homes && !s.navigations,
              "invalid map allowed robot motion or new tasks");
    }
    {
        Script s;s.start_disabled=s.enable_fails=true;s.replies={"OFFLINE"};
        check(run(s)==0 && s.motor_calls==1 && !s.homes && !s.navigations,
              "OFFLINE destructor issued another motor enable");
    }
    g_hw=nullptr;t170::tasks::bind_services(nullptr);
    std::cout<<"dispatch flow: SKIP/OFFLINE/failed report, charge precedence, RETRY and map entry passed\n";
}catch(const std::exception &error){std::cerr<<error.what()<<'\n';return 1;}}
