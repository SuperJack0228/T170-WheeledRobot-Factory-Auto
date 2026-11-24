#include "t170/app/task_result.h"
#include "t170/config/extrinsics.h"
#include "t170/tasks/skill_failure.h"
#include <filesystem>
#include <iostream>
#include <stdexcept>

void check(bool ok,const char *message){if(!ok)throw std::runtime_error(message);}
int main(int argc,char **argv) {
 using namespace t170::skills;
 try {
    check(argc>=2,"runtime required");
    std::array<double,16> matrix;std::string error;
    for(auto side:{"left","right"})check(load_cam2robot_matrix((std::filesystem::path(argv[1])/"config"/(std::string(side)+"_hand_to_base_result.yaml")).string(),matrix,error),error.c_str());
    Result failure;failure.skill=Skill::TrayGrasp;failure.status=Status::Failed;failure.stage=Stage::AlignDescend;
    failure.failure_stage=Stage::AlignDescend;failure.error=Error::Servo;failure.native_code=-30;
    failure.message="native phase returned failure";
    const auto current=t170::app::task_command_result(-30,0,failure,"","料盘抓取");
    if(argc==3 && std::string(argv[2])=="--emit-failure") {std::cout<<current;return 0;}
    check(!current["ok"].asBool() && current["native_code"].asInt()==-30 &&
          current["failure_stage"].asString()=="align_descend" && current["skill_result"]["error_code"].asInt()==1302,"native error lost");
    check(t170::tasks::terminal_skill_failure(failure)==-30,"fatal servo failure became retryable");
    const auto historical=t170::app::task_command_result(2,0,failure,"","料盘抓取");
    check(!historical["ok"].asBool() && historical["status"].asString()=="failed","historical code=2/count=0 became success");
    const auto partial=t170::app::task_command_result(2,1,std::nullopt,"","料盘放置");
    check(!partial["ok"].asBool() && partial["status"].asString()=="partial","partial task became success");
    check(!t170::app::task_command_result(0,0,std::nullopt,"","料盘抓取")["ok"].asBool(),"zero confirmed actions became success");
    check(t170::app::task_command_result(0,2,std::nullopt,"","料盘抓取")["ok"].asBool(),"confirmed success rejected");
    check(!t170::app::task_command_result(1,0,std::nullopt,"","料盘抓取")["ok"].asBool(),"no target became success");
    const auto rejected=t170::app::task_command_result(-30,0,t170::tasks::configuration_failure(Skill::TrayGrasp,"missing camera yaml"),"","料盘抓取");
    check(!rejected["ok"].asBool() && rejected["status"].asString()=="rejected" && rejected["failure_stage"].asString()=="validate","preflight rejection was flattened");
    check(t170::app::task_command_result(-4,0,std::nullopt,"","料盘抓取")["status"].asString()=="cancelled","cancelled task became generic failure");
    auto retry=failure;retry.status=Status::NoTarget;retry.error=Error::NoTarget;retry.native_code.reset();
    check(t170::tasks::terminal_skill_failure(retry)==0,"no-target retry policy changed");
    retry.error=Error::EmptyGrasp;check(t170::tasks::terminal_skill_failure(retry)==0,"empty grasp retry policy changed");
    auto held=failure;held.hands[1].part=PartState::Holding;held.hands[1].action_completed=true;
    const auto held_reply=t170::app::task_command_result(-30,1,held,"","料盘抓取");
    check(!held_reply["ok"].asBool() && held_reply["skill_result"]["hands"]["right"]["part_state"].asString()=="holding","failure lost physical holding state");
    std::cout<<"Deployed matrices -> task contract -> JSON: failure, no target, partial and success passed\n";
 }catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
}
