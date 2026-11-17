#include "support/factory_ports.h"
#include "../src/tasks/internal/runtime.h"
#include <iostream>
#include <stdexcept>
MoveBoxConfig g_move_cfg;
namespace {bool aborted=false;void require(bool ok,const char *text){if(!ok)throw std::runtime_error(text);}}
bool hardware_abort_requested(){return aborted;}
bool can_io_faulted(){return false;}
void hardware_abort_sleep_ms(int){}
namespace orch {
HwSession *g_hw=nullptr;
std::string g_dispatch_nav_fault;
std::chrono::steady_clock::time_point g_station_arrived_at;
bool g_station_arrived=false;
void publish_dispatch_pose(double,double,double,const std::string &){}
void board_station(const std::string &,const std::string &){}
void board_error(const std::string &){}
}
namespace {
struct Chassis : factory_test::Chassis {
    bool already=false,blocked=false,cancel_after_go=false;
    int sends=0,cancels=0,polls=0,finish=4,send_code=0;
    int GetLocation(double &x,double &y,double &a,std::string &station,std::string &,double &c,int &state,bool &forbidden) override {
        x=1;y=2;a=.3;c=1;state=0;forbidden=false;
        station=(already || sends)?"group5-out":"start";return 0;
    }
    int GoToStation(const std::string &) override{++sends;if(cancel_after_go)aborted=true;return send_code;}
    int GetNavStatus(int &state,int &,std::string &target,double &,double &,double &,
                     std::vector<std::string> &,std::vector<std::string> &,double &) override {
        state=++polls<3?2:finish;target="group5-out";return 0;
    }
    int GetBlocked(bool &out,bool &,double &,double &,std::vector<std::string> &) override{out=blocked;return 0;}
    int CancelNav() override{++cancels;return 0;}
};
}
int main() {
    try {
        setenv("T170_TIMING","0",1);g_move_cfg.chassis.host="synthetic";
        orch::HwSession hardware;orch::g_hw=&hardware;factory_test::Gateway gateway;
        Chassis chassis;t170::tasks::Services services{chassis,gateway};t170::tasks::bind_services(&services);
        int early=0;int code=orch::chassis_goto_station("group5-out",false,[&]{++early;});
        require(code==0 && chassis.sends==1 && chassis.polls==3 && early==1,
                "station-name transition was mistaken for stopped arrival");
        require(hardware.last_chassis_ok && orch::g_station_arrived,"arrival state not published");
        chassis=Chassis{};chassis.already=true;code=orch::chassis_goto_station("group5-out");
        require(code==0 && chassis.sends==0,"already-at-target shortcut changed");
        chassis=Chassis{};chassis.blocked=true;code=orch::chassis_goto_station("group5-out",true);
        require(code==-11 && chassis.cancels==1 && orch::g_dispatch_nav_fault==orch::dispatch_msg::ERR_AGV_OBSTACLE,
                "obstacle did not cancel or retain its recovery state");
        chassis=Chassis{};chassis.cancel_after_go=true;code=orch::chassis_goto_station("group5-out");
        require(code==-4 && chassis.cancels==1,"cancellation did not cancel navigation");
        aborted=false;chassis=Chassis{};chassis.finish=5;code=orch::chassis_goto_station("group5-out");
        require(code==-6 && !hardware.last_chassis_ok,"failed navigation was counted as arrived");
        chassis=Chassis{};chassis.send_code=-42;code=orch::chassis_goto_station("group5-out");
        require(code==-42,"native negative navigation code was lost");
        t170::tasks::bind_services(nullptr);
        std::cout<<"factory navigation: station-name timing, arrival, obstacle, cancel and native errors passed\n";
    }catch(const std::exception &ex){std::cerr<<ex.what()<<'\n';return 1;}
}
