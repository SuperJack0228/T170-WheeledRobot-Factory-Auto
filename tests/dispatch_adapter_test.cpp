#include "t170/integration/dispatch/client.h"
#include "t170/integration/dispatch/inbox.h"
#include "robot_client.hpp"
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>
bool hardware_abort_requested(){return false;}
void hardware_abort_sleep_ms(int ms){std::this_thread::sleep_for(std::chrono::milliseconds(ms));}
void require(bool value,const char *message){if(!value)throw std::runtime_error(message);}
int main(){
    try {
        using namespace t170::integration::dispatch;
        auto gateway=make_robot_client_gateway();
        RobotClient::raw_material_point="tray-A";RobotClient::sequence_one_port_two="machine-5";
        require(gateway->snapshot().raw_material_point=="tray-A" && gateway->snapshot().sequence_one_port_two=="machine-5","SDK points not mapped");
        gateway->update_pose(1.5,-2.25,.5,"station");gateway->update_pose(0,0,0,{});gateway->update_battery(83);
        require(RobotClient::current_point=="station" && RobotClient::power_percent==83,"status publication changed");
        RobotClient::is_map_updated=true;gateway->mark_map_current();require(!RobotClient::is_map_updated,"map completion flag lost");
        const std::pair<Fault,int> codes[]={{Fault::CameraDisconnected,RobotClient::CameraDisconnected},
            {Fault::InitFailed,RobotClient::InitFailed},{Fault::LoadPortBlocked,RobotClient::LoadPortBlocked},
            {Fault::MotorDisconnected,RobotClient::MotorDisconnected},{Fault::QrUnrecognized,RobotClient::QrUnrecognized},
            {Fault::UnloadPointEmpty,RobotClient::UnloadPointEmpty},{Fault::WorkpieceFallen,RobotClient::WorkpieceFallen}};
        for(auto [fault,code]:codes)require(gateway->exception_code(fault)==code,"native fault mapping changed");
        require(gateway->exception_code(Fault::GripperNotReady)==kGripperNotReadyCode,"gripper ready fault code changed");
        auto client=gateway->connect("T170","synthetic-robot");
        require(RobotClient::last_type=="T170" && RobotClient::last_serial=="synthetic-robot","identity lost");
        require(client->apply_place_raw_material(10) && RobotClient::last_timeout==10,"raw grant timeout changed");
        require(client->apply_pick_up_half_done(20) && RobotClient::last_timeout==20,"half pick grant changed");
        require(client->apply_place_half_done(30) && RobotClient::last_timeout==30,"half place grant changed");
        require(client->apply_pick_up_well_done(40) && RobotClient::last_timeout==40,"finished grant changed");
        client->mission_done();client->pick_up_done();client->place_done();client->raw_material_empty();client->well_done_material_full();
        require(RobotClient::acks==5,"completion messages lost");
        client->report_exception(809);require(RobotClient::last_exception==809,"native error changed");
        orch::DispatchInbox inbox;orch::inbox_start(inbox,*client);
        auto receiving=RobotClient::last_receive;
        {std::unique_lock<std::mutex> lock(receiving->mutex);
         require(receiving->ready.wait_for(lock,std::chrono::seconds(1),[&]{return receiving->entered;}),"receiver did not start");}
        orch::close_dispatch_inbox(client,inbox);
        require(!client && receiving->returned && !inbox.worker.joinable(),"transport was not closed before receiver join");
        std::cout<<"dispatch adapter: identities, grants, native faults, status and blocked receiver shutdown passed (contract double)\n";
    }catch(const std::exception &ex){std::cerr<<ex.what()<<'\n';return 1;}
}
