// Real exception handling, recovery and map decisions with inert SDK/IO ports.
#include "support/factory_ports.h"
#include "../src/tasks/internal/runtime.h"
#include <deque>
#include <iostream>
#include <stdexcept>
MoveBoxConfig g_move_cfg;
namespace {
bool aborted=false,io_fault=false;
int clears=0;
factory_test::Chassis chassis;
factory_test::Gateway gateway;
void check(bool ok,const char *message){if(!ok)throw std::runtime_error(message);}
struct Client : factory_test::Client {
    std::deque<std::string> replies;
    int calls=0;
    bool abort_on_response=false,throw_on_response=false;
    std::string message;
    std::string report_exception(const std::string &text) override {
        ++calls;message=text;
        const auto board=orch::dispatch_board_snapshot();
        check(board.last_error.find(text)!=std::string::npos && board.report_what==text && board.report_state=="pending",
              "operator prompt not published before blocking SDK call");
        if(throw_on_response)throw std::runtime_error("synthetic disconnect");
        if(abort_on_response)aborted=true;
        auto response=replies.empty()?std::string():replies.front();
        if(!replies.empty())replies.pop_front();
        return response;
    }
};
}
bool hardware_abort_requested(){return aborted;}
void hardware_abort_sleep_ms(int){}
bool can_io_faulted(){return io_fault;}
std::string can_io_fault_message(){return "synthetic motor disconnect";}
void can_io_fault_clear(){++clears;io_fault=false;}
gripper::SideFeedback gripper::Gripper::feedback(gripper::Side) const {return {};}
namespace orch {
Chassis &chassis_seer(){return chassis;}
bool ensure_chassis(){return false;}
}
int main(){try {
    using namespace orch;
    setenv("T170_TIMING","0",1);
    board_begin("synthetic","test-120");
    t170::tasks::Services services{chassis,gateway};t170::tasks::bind_services(&services);
    for(const auto &[response,expected]:std::vector<std::pair<std::string,int>>{
            {"RETRY",0},{"SKIP",kDispatchTaskSkip},{"OFFLINE",kDispatchOffline},
            {"",kDispatchReportFailed},{"invalid",kDispatchReportFailed}}) {
        Client client;client.replies={response};
        check(dispatch_report_ex(client,"  相机连接掉线 \n")==expected,"decision mapping failed");
        check(client.calls==1 && client.message=="相机连接掉线","exception description lost");
        if(expected==kDispatchReportFailed)
            check(dispatch_board_snapshot().report_state=="failed","invalid response marked sent");
    }
    Client empty;empty.replies={"RETRY"};
    check(dispatch_report_ex(empty,nullptr)==kDispatchReportFailed &&
          dispatch_report_ex(empty," \n\t")==kDispatchReportFailed && !empty.calls,
          "empty description entered SDK wait");
    aborted=true;
    check(dispatch_report_ex(empty,"故障")==-4 && !empty.calls,"abort sent another prompt");
    aborted=false;Client interrupted;interrupted.replies={"RETRY"};interrupted.abort_on_response=true;
    check(dispatch_report_ex(interrupted,"故障")==-4,"late RETRY overrode stop request");
    aborted=false;Client disconnected;disconnected.throw_on_response=true;
    check(dispatch_report_ex(disconnected,"故障")==kDispatchReportFailed,"SDK failure escaped stop policy");
    Client gripper;gripper.replies={"SKIP"};
    check(dispatch_wait_system(gripper,dispatch_msg::ERR_GRIPPER_LOST)==kDispatchTaskSkip &&
          gripper.message=="夹爪连接掉线","gripper fault never entered operator prompt");
    io_fault=true;Client motor;motor.replies={"SKIP","RETRY"};
    check(dispatch_wait_system(motor,dispatch_msg::ERR_MOTOR_LOST)==kDispatchTaskSkip && !clears && io_fault,
          "SKIP cleared a latched IO fault");
    check(dispatch_wait_system(motor,dispatch_msg::ERR_MOTOR_LOST)==0 && clears==1 && !io_fault,
          "RETRY could not perform fresh motor IO checks");
    Client belt;int budget=1;
    check(dispatch_belt_vision_again(belt,budget,"belt")==0 && !budget && !belt.calls,
          "local visual retry budget changed");
    belt.replies={"SKIP","RETRY"};
    check(dispatch_belt_vision_again(belt,budget,"belt")==kDispatchTaskSkip,"SKIP became visual retry");
    check(dispatch_belt_vision_again(belt,budget,"belt")==0 && budget==1,"RETRY did not renew visual budget");
    // No hardware/real map writes: missing map or inert chassis always refuses load.
    gateway.state.is_map_updated=false;Client map;
    check(finish_dispatch_map(map,0,0,0)==0 && !map.calls,"current map prompted operator");
    gateway.state.is_map_updated=true;
    for(const auto &[response,expected]:std::vector<std::pair<std::string,int>>{
            {"SKIP",kDispatchTaskSkip},{"OFFLINE",kDispatchOffline},{"",kDispatchReportFailed}}) {
        map.replies={"RETRY","RETRY",response};map.calls=0;
        check(finish_dispatch_map(map,0,0,0)==expected && map.calls==3,"map swallowed operator decision");
    }
    aborted=true;check(finish_dispatch_map(map,0,0,0)==-4,"map retry ignored stop");
    t170::tasks::bind_services(nullptr);
    std::cout<<"exception handling: replies, empty/disconnect, prompt visibility, stop, IO retry and map decisions passed\n";
}catch(const std::exception &error){std::cerr<<error.what()<<'\n';return 1;}}
