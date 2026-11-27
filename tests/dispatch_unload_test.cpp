// Exercise the actual task orchestration with inert perception/motion ports.
#include "support/factory_ports.h"
#include "../src/tasks/internal/runtime.h"
#include <iostream>
#include <stdexcept>
MoveBoxConfig g_move_cfg;
namespace {
bool pick_left=true, pick_right=true, placed_left=true, placed_right=true;
int place_result=0, done_count=0, full_count=0, pick_result=0, place_calls=0;
int exception_choice=orch::kDispatchReportFailed;
bool qr_once=false;
}
namespace orch {
void board_step(const std::string &) {}
void board_activity(const std::string &) {}
void board_note(const std::string &,const std::string &) {}
void board_tray2(int,int) {}
bool belt_ready_to_pick(const std::string &,const ConveyorStationConfig &,const char *) {return true;}
int drive_belt_ready(const std::string &,const ConveyorStationConfig &,const char *) {return 0;}
bool apply_pick_well(DispatchClient &,int) {return true;}
int dispatch_pick_pair(DispatchClient &,bool (*)(DispatchClient &,int),
    const std::string &,const std::string &,const ConveyorStationConfig &,const std::string &,
    const char *,bool,bool,const char *,const char *,const std::string &,bool)
{
    g_hw->last_grasped_l=pick_left;g_hw->last_grasped_r=pick_right;
    return pick_result;
}
int vision_place_tray2(bool,bool,bool)
{
    ++place_calls;
    if(qr_once){qr_once=false;return kVisionQrFail;}
    g_hw->last_placed_l=placed_left;g_hw->last_placed_r=placed_right;
    if(placed_left)g_hw->last_grasped_l=false;
    if(placed_right)g_hw->last_grasped_r=false;
    return place_result;
}
int dispatch_report_ex(DispatchClient &,const char *) {return exception_choice;}
void dispatch_send(DispatchClient &,const char *,bool) {++full_count;}
int dispatch_report(DispatchClient &,DispatchNote,const char *) {++done_count;return 0;}
}
void require(bool ok,const char *message) {if(!ok)throw std::runtime_error(message);}
int main() {
 try {
    factory_test::Chassis chassis;factory_test::Gateway gateway;factory_test::Client client;
    t170::tasks::Services services{chassis,gateway};t170::tasks::bind_services(&services);
    gateway.state.well_done_point="tray-B";gateway.state.sequence_two_port_one="belt-B";
    orch::HwSession session;orch::g_hw=&session;
    auto run=[&](int code,bool want_l,bool want_r,bool release_l,bool release_r) {
        session={};done_count=full_count=place_calls=0;place_result=code;
        pick_left=want_l;pick_right=want_r;placed_left=release_l;placed_right=release_r;
        orch::g_unload_empty_known=false;
        return orch::dispatch_unload(client);
    };
    require(run(0,true,true,true,true)==0 && done_count==1,"confirmed pair was not completed");
    require(run(0,true,false,true,false)==0 && done_count==1,"confirmed single was not completed");
    require(run(1,true,true,false,false)==orch::kDispatchHoldStay && !done_count,"no hole became DONE");
    require(run(2,true,true,true,false)==orch::kDispatchHoldStay && !done_count && session.last_grasped_r,
            "partial placement lost held part or became DONE");
    require(run(0,true,true,true,false)==orch::kDispatchHoldStay && !done_count,"missing release became DONE");
    require(run(0,false,false,false,false)==orch::kDispatchHoldStay && !done_count,"no action became DONE");
    require(run(-31,true,true,false,false)==-31 && !done_count,"servo failure was not propagated");
    for(int choice:{orch::kDispatchTaskSkip,orch::kDispatchOffline,orch::kDispatchReportFailed}) {
        pick_result=choice;
        require(run(0,true,true,true,true)==choice && !place_calls && !done_count,
                "pick operator choice did not stop downstream placement");
        pick_result=0;exception_choice=choice;qr_once=true;
        require(run(0,true,true,true,true)==choice && place_calls==1 && !done_count,
                "place operator choice became DONE or a retry");
    }
    exception_choice=0;qr_once=true;
    require(run(0,true,true,true,true)==0 && place_calls==2 && done_count==1,
            "RETRY failed to resume placement without another pick");
    // A full-tray signal remains useful, but never completes an unplaced task.
    orch::g_unload_empty_known=true;orch::g_unload_empty_left=orch::g_unload_empty_right=0;
    place_result=1;pick_left=pick_right=true;placed_left=placed_right=false;done_count=full_count=0;
    require(orch::dispatch_unload(client)==orch::kDispatchHoldStay && !done_count && full_count==1,
            "full-tray report suppressed or incomplete task acknowledged");
    orch::g_hw=nullptr;t170::tasks::bind_services(nullptr);
    std::cout<<"dispatch unload: confirmed releases, partial/no-hole/no-action and fatal failure passed\n";
 } catch(const std::exception &error) {std::cerr<<error.what()<<'\n';return 1;}
}
