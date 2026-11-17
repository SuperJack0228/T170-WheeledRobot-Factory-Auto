// Actual standardized service with a synthetic legacy cycle and inert hardware objects.
#include "t170/skills/tray.h"
#include "../src/skills/internal/tray_cycle.h"
#include "../src/skills/internal/execution.h"
#include <iostream>
#include <stdexcept>
namespace {
int scenario=0,calls=0;
bool skip_right_seen=false,skip_left_seen=false,aborted=false,io_fault=false;
move_box::TrayHoleTask mode=move_box::TrayHoleTask::GraspParts;
void require(bool ok,const char *message){if(!ok)throw std::runtime_error(message);}
}
bool hardware_abort_requested(){return aborted;}
bool can_io_faulted(){return io_fault;}
Arm::Arm(const std::string &,const std::string &,const std::string &,const std::string &){}
WaistRobot::WaistRobot(){}
struct RealSenseMultiCam::Impl {};
RealSenseMultiCam::RealSenseMultiCam(std::string,int,int,int,int,bool){}
RealSenseMultiCam::~RealSenseMultiCam()=default;
struct SegPoseBridge::Impl {};
SegPoseBridge::SegPoseBridge(){}
SegPoseBridge::~SegPoseBridge()=default;
namespace gripper {
struct Gripper::SideImpl {};
Gripper::Gripper(const Config &c):config_(c){}
Gripper::~Gripper()=default;
}
namespace move_box {
TrayHoleTask tray_hole_task(){return mode;}
void set_tray_hole_task(TrayHoleTask value){mode=value;}
GraspToStandbyResult run_tray_cycle_impl(gripper::Gripper &,Arm &,Arm &,WaistRobot &,
 Eigen::Matrix<double,1,6> &,RealSenseMultiCam &,SegPoseBridge &,const std::array<double,16> &,
 std::string &,const std::function<bool()> &cancel,SegEngineId,bool,bool skip_r,bool skip_l,
 const std::function<void(bool,bool)> &,bool,const std::function<void()> &) {
    using namespace t170::skills;
    ++calls;skip_right_seen=skip_r;skip_left_seen=skip_l;GraspToStandbyResult result;
    detail::report(Stage::Observe,!skip_r,!skip_l);
    if(cancel && cancel()){result.status=GraspToStandbyStatus::Aborted;return result;}
    result.zone_count_ok=true;result.zone_left=result.zone_right=scenario==1?0:10;
    if(scenario==1)return result;
    detail::report(Stage::Approach,!skip_r,!skip_l);
    detail::report(Stage::Gripper,!skip_r,!skip_l);
    if(scenario==2) {
        result.status=GraspToStandbyStatus::ReleaseFail;
        result.grasped_l=result.st.released_l=!skip_l;result.st.release_failed=true;
        detail::report_action(false,PartState::Empty,Evidence::GripperFeedback,true);
        detail::report_native(-38,0,!skip_r,false,Stage::Gripper);return result;
    }
    if(!skip_r) detail::report_action(true,mode==TrayHoleTask::PlaceEmpty?PartState::Empty:PartState::Holding,Evidence::LegacyPolicy,true);
    if(!skip_l) detail::report_action(false,mode==TrayHoleTask::PlaceEmpty?PartState::Empty:PartState::Holding,Evidence::LegacyPolicy,true);
    if(scenario==4)throw std::runtime_error("business hook after grasp");
    result.grasped_r=!skip_r;result.grasped_l=!skip_l;
    if(mode==TrayHoleTask::PlaceEmpty){result.st.released_r=!skip_r;result.st.released_l=!skip_l;}
    detail::report(Stage::Return,!skip_r,!skip_l);
    if(scenario==3){detail::report_native(-16,0,!skip_r,false,Stage::Return);result.status=GraspToStandbyStatus::MotionFail;}
    else result.status=GraspToStandbyStatus::Ok;
    return result;
}
}
int main() {
    using namespace t170::skills;
    try {
        Arm right("","","right"),left("","","left");gripper::Gripper gripper;
        WaistRobot waist;RealSenseMultiCam cameras("synthetic");SegPoseBridge vision;
        Eigen::Matrix<double,1,6> waist_pose=Eigen::Matrix<double,1,6>::Zero();
        TrayContext context{gripper,right,left,waist,waist_pose,cameras,vision,{}, {}};
        TrayGraspRequest request;request.metadata.workcell_id="any-machine-group";
        auto result=execute_tray_grasp(context,request);
        require(result.ok() && result.hands[1].part==PartState::Holding && result.hands[1].retreat_completed,
                "successful tray grasp result is wrong");
        require(result.metadata.workcell_id==request.metadata.workcell_id && result.available_left==10,
                "target identity/count was dropped");
        request.hands={true,false};result=execute_tray_grasp(context,request);
        require(result.ok() && skip_right_seen && !skip_left_seen && !result.hands[1].requested,
                "left-only selection mapped to wrong legacy hand");
        request.hands={true,true};request.already_holding.right=true;result=execute_tray_grasp(context,request);
        require(result.ok() && skip_right_seen && result.hands[1].part==PartState::Holding &&
                result.hands[1].evidence==Evidence::CallerReported && !result.hands[1].started,
                "already-held hand was moved or lost");
        request.already_holding.right=false;scenario=1;result=execute_tray_grasp(context,request);
        require(result.status==Status::NoTarget && result.available_right==0 && !result.hands[1].started,
                "empty tray conflated with a motion failure");
        scenario=2;TrayPlaceRequest place;result=execute_tray_place(context,place);
        require(result.error==Error::ReleaseUnconfirmed && result.hands[0].part==PartState::Empty &&
                result.hands[0].action_completed && result.hands[1].part==PartState::Unknown,
                "partial tray release was flattened");
        require(mode==move_box::TrayHoleTask::GraspParts,"placement leaked global task mode");
        scenario=3;result=execute_tray_grasp(context,request);
        require(result.native_code==-16 && result.legacy_status==5 && result.hands[1].part==PartState::Holding &&
                !result.hands[1].retreat_completed,"return failure lost held-part state");
        scenario=4;result=execute_tray_grasp(context,request);
        require(result.error==Error::Exception && result.hands[1].part==PartState::Holding &&
                mode==move_box::TrayHoleTask::GraspParts,"exception lost effects or scope cleanup");
        scenario=0;int before=calls;result=execute_tray_grasp(context,request,{[]{return true;},{}});
        require(result.status==Status::Cancelled && calls==before,"pre-cancelled request entered cycle");
        place.initial_parts[0]=PartState::Empty;result=execute_tray_place(context,place);
        require(result.error==Error::InvalidRequest && calls==before,"known empty claw accepted for placement");
        std::cout<<"tray service: selection, target identity, no target, partial release, held-part failure and scope cleanup passed\n";
    }catch(const std::exception &ex){std::cerr<<ex.what()<<'\n';return 1;}
}
