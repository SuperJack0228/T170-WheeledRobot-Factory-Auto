#include "support/factory_ports.h"
#include "../src/tasks/internal/runtime.h"
#include <iostream>
#include <stdexcept>
MoveBoxConfig g_move_cfg;
namespace orch {void board_note(const std::string &,const std::string &) {}}
void require(bool value,const char *message){if(!value)throw std::runtime_error(message);}
int main(){try {
    factory_test::Chassis chassis;factory_test::Gateway gateway;
    t170::tasks::Services services{chassis,gateway};t170::tasks::bind_services(&services);
    gateway.state.raw_material_point="tray-A";gateway.state.well_done_point="tray-B";
    gateway.state.sequence_one_port_one="machine-group1-in";gateway.state.sequence_two_port_two="machine-group1-out";
    auto points=orch::read_task_points();
    require(orch::task_points_ready(orch::dispatch_msg::TASK_LOAD,points) &&
            orch::task_points_ready(orch::dispatch_msg::TASK_TRANSFER,points) &&
            orch::task_points_ready(orch::dispatch_msg::TASK_UNLOAD,points),"valid assignment rejected");
    require(points.unload=="machine-group1-out","port-two fallback lost");
    gateway.state.sequence_one_port_one.clear();gateway.state.sequence_one_port_two="machine-group5-in";
    gateway.state.sequence_two_port_two="machine-group5-out";
    auto next=orch::read_task_points();
    require(next.load=="machine-group5-in" && next.unload=="machine-group5-out" && points.load=="machine-group1-in",
            "robot was bound to an earlier machine or points were not copied");
    next.raw.clear();require(!orch::task_points_ready(orch::dispatch_msg::TASK_LOAD,next),"missing rack admitted");
    require(!orch::task_points_ready("unknown",points),"unknown task admitted");
    g_move_cfg.chassis.dispatch_wait_station="fallback-wait";g_move_cfg.chassis.charge_station="fallback-charge";
    require(orch::dispatch_idle_station()=="fallback-wait" && orch::resolve_charge_station()=="fallback-charge","fallback changed");
    gateway.state.standby_point="assigned-wait";gateway.state.charge_point="assigned-charge";
    require(orch::dispatch_idle_station()=="assigned-wait" && orch::resolve_charge_station()=="assigned-charge","dispatch override lost");
    auto original=g_move_cfg.chassis.belt_station;
    {orch::StationOverride guard(g_move_cfg.chassis);g_move_cfg.chassis.belt_station="temporary";}
    require(g_move_cfg.chassis.belt_station==original,"temporary station override leaked");
    t170::tasks::bind_services(nullptr);
    std::cout<<"factory assignment: dispatch point changes, no robot-machine pairing, fallbacks and restoration passed\n";
}catch(const std::exception &ex){std::cerr<<ex.what()<<'\n';return 1;}}
