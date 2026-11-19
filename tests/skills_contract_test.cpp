#include "t170/skills/types.h"
#include "t170/skills/tray.h"
#include "t170/skills/conveyor.h"
#include "../src/skills/internal/execution.h"
#include <atomic>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>
using namespace t170::skills;
using t170::skills::detail::Execution;
namespace { void require(bool ok,const char *message){if(!ok) throw std::runtime_error(message);} }
int main(int argc,char **) {
    try {
        if(argc>1) {
            Execution run(Skill::TrayPlace,{"request-\"1\"\n","机床组5","r5c2"},{true,true},{},7);
            run.stage(Stage::Gripper,{true,true});
            run.result.hands[0].part=PartState::Empty;run.result.hands[0].action_completed=true;
            run.result.hands[0].evidence=Evidence::GripperFeedback;
            NominalTarget target;target.frame="upright_arm_base";target.pose={.4,.2,.1,0,0,0};target.tray_row_aruco=5;target.tray_column_aruco=2;
            run.result.hands[0].nominal_target=target;
            run.native(-38,{false,true},Stage::Gripper);
            std::cout<<to_json(run.finish())<<'\n';return 0;
        }
        {
            Execution right(Skill::ConveyorGrasp,{}, {false,true},{},2);
            require(right.entered(),"first arm lease failed");
            Execution left(Skill::ConveyorRetreat,{}, {true,false},{},1);
            require(left.entered(),"opposite-arm overlap was rejected");
            Execution same(Skill::ConveyorPlace,{}, {false,true},{},2);
            require(!same.entered() && same.finish().error==Error::Busy,"same-arm double writer allowed");
            Execution tray(Skill::TrayGrasp,{}, {true,true},{},7);
            require(!tray.entered(),"tray/waist operation overlapped an active conveyor arm");
        }
        {
            Execution tray(Skill::TrayGrasp,{}, {true,true},{},7);
            require(tray.entered(),"resource lease leaked");
            tray.result.hands[0].action_completed=true;tray.result.hands[0].part=PartState::Holding;
            auto result=tray.finish();
            require(result.status==Status::Partial && !result.ok() && result.error==Error::NoTarget,
                    "partial execution was flattened into success");
            require(tray.finish().status==Status::Partial,"finalizing twice changed a partial result");
        }
        {
            Execution run(Skill::TrayGrasp,{}, {true,true},{},7);
            run.result.hands[1].action_completed=true;run.result.hands[1].part=PartState::Holding;
            run.stage(Stage::Lift,{false,true});run.native(-16,{false,true},Stage::Lift);
            run.stage(Stage::Return,{true,false});
            const auto result=run.finish();
            require(result.status==Status::Failed && result.hands[1].part==PartState::Holding &&
                    result.hands[1].action_completed && !result.hands[1].retreat_completed,
                    "retreat failure erased held part or claimed completion");
            require(result.failure_stage==Stage::Lift && result.hands[1].failure_stage==Stage::Lift,
                    "later stage overwrote the failure stage");
        }
        {
            Execution cancelled(Skill::ConveyorGrasp,{}, {false,true},{[]{return true;},{}},2);
            require(cancelled.cancelled() && cancelled.finish().status==Status::Cancelled,"cancel ignored");
        }
        {
            Execution callback(Skill::ConveyorGrasp,{}, {false,true},{[]{throw std::runtime_error("cancel hook");return false;},{}},2);
            require(callback.cancelled() && callback.finish().error==Error::Exception,"throwing cancel hook escaped");
        }
        {
            std::vector<std::uint64_t> sequences;
            Control control;control.progress=[&](const Progress &e){sequences.push_back(e.sequence);if(e.sequence==2) throw 1;};
            Execution run(Skill::ConveyorGrasp,{}, {false,true},control,2);
            run.stage(Stage::AlignDescend,{false,true});run.stage(Stage::Gripper,{false,true});
            run.result.hands[1].action_completed=true;
            require(run.finish().ok(),"observer exception broke physical execution");
            require(sequences==std::vector<std::uint64_t>({1,2,3,4}),"progress order is not monotonic");
        }
        {
            Execution invalid(Skill::TrayGrasp,{}, {false,false},{},7);
            require(invalid.finish().status==Status::Rejected,"empty request did not reject");
            Execution unknown(Skill::ConveyorPlace,{}, {true,false},{},1);
            require(unknown.result.hands[0].part==PartState::Unknown && !unknown.result.hands[0].native_code,
                    "unknown state was initialized as a successful release");
        }
        require(from_native_code(-38,Stage::Gripper)==Error::ReleaseUnconfirmed,"release error translation wrong");
        require(from_native_code(-16,Stage::Lift)==Error::Motion,"motion error translation wrong");
        std::cout<<"skills contract: leases, overlap, partial effects, cancellation, progress and failure stages passed\n";
    } catch(const std::exception &ex) {std::cerr<<ex.what()<<'\n';return 1;}
}
