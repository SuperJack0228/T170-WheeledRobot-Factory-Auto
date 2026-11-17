#include "t170/skills/tray.h"
#include "../internal/tray_cycle.h"
#include "../internal/execution.h"
#include "t170/hardware/device_bus.h"
#include <utility>
#include <iostream>

namespace t170::skills {
namespace {
struct Invocation {
    Metadata metadata;
    Hands hands, skipped;
    std::array<PartState,2> initial{PartState::Unknown,PartState::Unknown};
    Detector detector=Detector::Default;
    bool place=false, adaptive=false, rise=false;
    std::function<void(bool,bool)> on_clearance;
    std::function<void()> on_waist_up;
};
Result execute(TrayContext &c,const Invocation &request,const Control &control,
               move_box::GraspToStandbyResult *legacy=nullptr,std::string *legacy_error=nullptr) {
    using namespace detail;
    Execution run(request.place?Skill::TrayPlace:Skill::TrayGrasp,request.metadata,request.hands,control,7);
    if(!run.entered()) return run.finish();
    ExecutionScope scope(run);
    if(int(request.detector)<0 || int(request.detector)>2) {
        run.fail(Error::InvalidRequest,"detector must be default, head or hand");return run.finish();
    }
    if(!c.waist_pose.allFinite()) {
        run.fail(Error::InvalidRequest,"waist pose contains non-finite values");return run.finish();
    }
    for(int h=0;h<2;++h) {
        auto &out=run.result.hands[h];out.part=request.initial[h];
        if(out.part!=PartState::Unknown) out.evidence=Evidence::CallerReported;
        if(out.requested && selected(request.skipped,Hand(h))) out.action_completed=true;
    }
    for(int h=0;h<2;++h) if(request.place && run.result.hands[h].requested &&
        !selected(request.skipped,Hand(h)) && request.initial[h]==PartState::Empty) {
        run.fail(Error::InvalidRequest,"cannot place with a caller-reported empty gripper");return run.finish();
    }
    if(can_io_faulted()) {run.fail(Error::Device,"CAN feedback unavailable",-2);return run.finish();}
    auto aborted=[&] {return hardware_abort_requested() || run.cancelled();};
    if(aborted()) {run.fail(Error::Cancelled,"aborted before tray motion",-4);return run.finish();}
    struct TaskScope {
        move_box::TrayHoleTask previous=move_box::tray_hole_task();
        ~TaskScope(){move_box::set_tray_hole_task(previous);}
    } task_scope;
    move_box::set_tray_hole_task(request.place?move_box::TrayHoleTask::PlaceEmpty:move_box::TrayHoleTask::GraspParts);
    std::string error;
    move_box::GraspToStandbyResult raw;
    try {
        run.stage(Stage::Prepare,request.hands);
        raw=move_box::run_tray_cycle_impl(c.gripper,c.right,c.left,c.waist,c.waist_pose,
            c.cameras,c.vision,c.head_camera_to_robot,error,aborted,SegEngineId(int(request.detector)),
            request.adaptive,!request.hands.right || request.skipped.right,
            !request.hands.left || request.skipped.left,request.on_clearance,request.rise,request.on_waist_up);
    } catch(const std::exception &ex) {
        run.fail(Error::Exception,ex.what());raw.status=move_box::GraspToStandbyStatus::MotionFail;
    } catch(...) {
        run.fail(Error::Exception,"untyped tray execution exception");raw.status=move_box::GraspToStandbyStatus::MotionFail;
    }
    if(run.result.error==Error::Exception) {
        // Preserve physical effects even when a business hook threw before the old result was returned.
        raw.grasped_r=run.result.hands[1].started && run.result.hands[1].action_completed;
        raw.grasped_l=run.result.hands[0].started && run.result.hands[0].action_completed;
        if(request.place) {
            raw.st.released_r=raw.grasped_r;raw.st.released_l=raw.grasped_l;
        }
    }
    if(legacy) *legacy=raw;
    if(legacy_error) *legacy_error=error;
    run.result.legacy_status=int(raw.status);
    run.result.waist_adjusted=raw.waist_adjusted;
    if(raw.zone_count_ok) {run.result.available_left=raw.zone_left;run.result.available_right=raw.zone_right;}
    for(int h=0;h<2;++h) {
        auto &out=run.result.hands[h];
        const bool have=h?raw.st.have_r:raw.st.have_l;
        const auto &goal=h?raw.st.goal_last_r:raw.st.goal_last_l;
        if(have && goal.allFinite()) {
            NominalTarget target;target.frame="upright_arm_base";
            for(int i=0;i<6;++i) target.pose[i]=goal(i);
            const int row=h?raw.st.row_aruco_r:raw.st.row_aruco_l;
            const int col=h?raw.st.col_aruco_r:raw.st.col_aruco_l;
            if(row>0) target.tray_row_aruco=row;
            if(col>0) target.tray_column_aruco=col;
            out.nominal_target=target;
        }
        const bool acted=h?raw.grasped_r:raw.grasped_l;
        const bool empty=h?raw.empty_r:raw.empty_l;
        const bool released=h?raw.st.released_r:raw.st.released_l;
        if(request.place && released) {
            out.action_completed=true;out.part=PartState::Empty;out.evidence=Evidence::GripperFeedback;
        } else if(!request.place && (acted || empty)) {
            out.action_completed=acted;out.part=acted?PartState::Holding:PartState::Empty;out.evidence=Evidence::LegacyPolicy;
        }
        // Successful return applies only to hands that actually participated.
        if(raw.status==move_box::GraspToStandbyStatus::Ok && out.started && out.error==Error::None)
            out.retreat_completed=true;
    }
    using Old=move_box::GraspToStandbyStatus;
    switch(raw.status) {
        case Old::Aborted:run.fail(Error::Cancelled,"tray action aborted",-4);break;
        case Old::CamFail:run.fail(Error::Perception,error.empty()?"tray perception/preparation failed":error);break;
        case Old::MotionFail:run.fail(Error::Motion,"tray retreat or motion failed");break;
        case Old::ReleaseFail:run.fail(Error::ReleaseUnconfirmed,"tray release was not confirmed",-38);break;
        default:break;
    }
    return run.finish();
}
}
Result execute_tray_grasp(TrayContext &c,const TrayGraspRequest &r,const Control &control) {
    Invocation request;request.metadata=r.metadata;request.hands=r.hands;request.skipped=r.already_holding;
    request.initial={r.already_holding.left?PartState::Holding:PartState::Unknown,
                     r.already_holding.right?PartState::Holding:PartState::Unknown};
    request.detector=r.detector;request.adaptive=r.adaptive_approach_rpy;request.rise=r.allow_waist_rise;
    request.on_clearance=c.hooks.on_clearance;request.on_waist_up=c.hooks.on_waist_up;
    return execute(c,request,control);
}
Result execute_tray_place(TrayContext &c,const TrayPlaceRequest &r,const Control &control) {
    Invocation request;request.metadata=r.metadata;request.hands=r.hands;request.skipped=r.already_placed;
    request.initial=r.initial_parts;
    if(r.already_placed.left) request.initial[0]=PartState::Empty;
    if(r.already_placed.right) request.initial[1]=PartState::Empty;
    request.detector=r.detector;request.place=true;request.on_clearance=c.hooks.on_clearance;
    return execute(c,request,control);
}
} // namespace t170::skills

namespace move_box {
GraspToStandbyResult run_grasp_to_standby(
    gripper::Gripper &g,Robot_Arm &right,Robot_Arm &left,WaistRobot &waist,
    Eigen::Matrix<double,1,6> &waist_pose,RealSenseMultiCam &cameras,SegPoseBridge &vision,
    const std::array<double,16> &head_matrix,std::string &error,const std::function<bool()> &abort,
    SegEngineId detector,bool adaptive,bool skip_right,bool skip_left,
    const std::function<void(bool,bool)> &on_clearance,bool rise,const std::function<void()> &on_waist_up) {
    if(skip_right && skip_left) return {};
    t170::skills::TrayContext context{g,right,left,waist,waist_pose,cameras,vision,head_matrix};
    t170::skills::Invocation request;
    request.hands={!skip_left,!skip_right};request.skipped={false,false};
    request.place=tray_hole_task()==TrayHoleTask::PlaceEmpty;
    request.detector=t170::skills::Detector(int(detector));request.adaptive=adaptive;
    request.rise=rise;request.on_clearance=on_clearance;request.on_waist_up=on_waist_up;
    GraspToStandbyResult raw;
    const auto result=t170::skills::execute(context,request,{abort,{}},&raw,&error);
    std::cout<<"[skill_result] "<<t170::skills::to_json(result)<<'\n';
    if(!result.legacy_status) {
        raw.status=result.error==t170::skills::Error::Cancelled?GraspToStandbyStatus::Aborted:GraspToStandbyStatus::MotionFail;
        error=result.message;
    }
    return raw;
}
} // namespace move_box
