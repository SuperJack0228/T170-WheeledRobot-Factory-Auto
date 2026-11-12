// Real session command gates with non-operational hardware adapters.
// No motion, CAN, camera or SDK is opened in this test.
#include "t170/calibration/placement_session.h"
#include "robot_runtime.h"
#include "function.h"
#include "move_box_runtime.h"
#include "move_box_config.h"
#include "servo/runtime.h"
#include "t170/vision/pipeline.h"
#include <filesystem>
#include <iostream>
#include <stdexcept>
MoveBoxConfig g_move_cfg;
namespace {bool aborted=false;int hardware_calls=0;void no_hardware(){++hardware_calls;throw std::runtime_error("test hardware adapter must not run");}}
bool hardware_abort_requested(){return aborted;}
bool can_io_faulted(){return false;}
bool motor_enable_id(std::uint32_t){no_hardware();return false;}
bool right_arm_motors_locked(){return false;}
MatrixXd TR(Matrix<double,1,6>){return Eigen::Matrix4d::Identity();}
Matrix<double,1,6> T2PosEulerAngles(Matrix<double,4,4>){return Matrix<double,1,6>::Zero();}
Matrix<double,1,6> arm_get_tcp_pos(Robot_Arm &){no_hardware();return Matrix<double,1,6>::Zero();}
Matrix<double,1,7> arm_get_joint_pos(Robot_Arm &){no_hardware();return Matrix<double,1,7>::Zero();}
int arm_line_move(Robot_Arm &,Matrix<double,1,6>&,double,bool){no_hardware();return -1;}
bool record_arm_cartesian_arrival(Robot_Arm &,const Matrix<double,1,6>&,const char*,double){no_hardware();return false;}
int arm_vertical_lift(Robot_Arm &,double,double,const std::function<void(double)>&){no_hardware();return -1;}
int arm_return_after_lift(Robot_Arm &,const Matrix<double,1,7>&,const std::function<void()>&,double,double){no_hardware();return -1;}
ArmLineMoveResult arm_dual_line_then_line_nostop(Robot_Arm &,const Matrix<double,1,6>&,const Matrix<double,1,6>&,bool,
 Robot_Arm &,const Matrix<double,1,6>&,const Matrix<double,1,6>&,bool,double,double,const std::function<void()>&,bool,const char*,bool,bool,int){no_hardware();return {-1,-1};}
int enable_head_calib_pose(){no_hardware();return -1;}
int enable_head_far_pitch(){no_hardware();return -1;}
Eigen::Matrix4d Robot_Arm::Forward_Kinematics(const Eigen::Ref<const Eigen::RowVectorXd>&){no_hardware();return Eigen::Matrix4d::Identity();}
bool Robot_Arm::Check_Joint_Limit(const Eigen::Ref<const Eigen::RowVectorXd>&){no_hardware();return true;}
RealSenseMultiCam &PosePipeline::cameras(){no_hardware();throw std::runtime_error("no camera");}
SegPoseBridge &PosePipeline::bridge(){no_hardware();throw std::runtime_error("no vision");}
CameraFrameData RealSenseMultiCam::grab_fresh(CameraSlot){no_hardware();return {};}
bool load_cam2robot_matrix(const std::string &,std::array<double,16>&,std::string&){no_hardware();return false;}
namespace vision {HeadTrayPerception perceive_tray_head(RealSenseMultiCam&,SegPoseBridge&,const PerceiveTrayOptions&){no_hardware();return {};}}
namespace move_box {
Eigen::Matrix4d placement_calibration_frame(){no_hardware();return Eigen::Matrix4d::Identity();}
double tray_z_ref_now(){no_hardware();return 0;}
double expected_object_top_z(){no_hardware();return 0;}
bool ensure_waist_ready_start(WaistRobot&,Matrix<double,1,6>&){no_hardware();return false;}
void waist_layer3_set_synced(bool){no_hardware();}
double waist_layer3_ensure_far_row_forward(WaistRobot&,Matrix<double,1,6>&,double){no_hardware();return 0;}
}
namespace wrist_servo {
void send_joint_reference(int,const Joints&){no_hardware();}
bool take_final_hold(int,Joints&){no_hardware();return false;}
void hold_joint_reference(int,const Joints&){no_hardware();}
void start_final_hold_repeat(){no_hardware();}
bool ready_for_gripper(Robot_Arm&){no_hardware();return false;}
void stage_tray_target(int,const TrayTarget&){no_hardware();}
Eigen::Matrix4d tool_transform(const Robot_Arm&){no_hardware();return Eigen::Matrix4d::Identity();}
}
namespace gripper {
bool Gripper::hasSide(Side)const {no_hardware();return false;}
GraspFeedback Gripper::graspAndWait(Side,double,std::chrono::milliseconds){no_hardware();return {};}
bool Gripper::holdCurrent(Side){no_hardware();return false;}
SideFeedback Gripper::feedback(Side)const {no_hardware();return {};}
bool Gripper::openAndWait(Side,double,std::chrono::milliseconds){no_hardware();return false;}
bool Gripper::closeAndWait(Side,double,double,std::chrono::milliseconds){no_hardware();return false;}
}
void check(bool v,const char *s){if(!v)throw std::runtime_error(s);}
int main(int argc,char **argv) {
 try {
    check(argc==2,"repository required");
    const auto root=std::filesystem::temp_directory_path()/"t170-calibration-session-test";
    std::filesystem::remove_all(root);std::filesystem::create_directories(root/"config");
    for(const auto &name:{"move_box_params.yaml","realsense_cameras.yaml","Robot_Arm_Model.yaml","right_hand_to_base_result.yaml","left_hand_to_base_result.yaml"})
        std::filesystem::copy_file(std::filesystem::path(argv[1])/"config"/name,root/"config"/name);
    setenv("T170_ROOT",root.c_str(),1);unsetenv("T170_CONFIG_DIR");
    std::string error;check(load_move_box_config((root/"config/move_box_params.yaml").string(),g_move_cfg,error),error.c_str());
    t170::calibration::PlacementSession session;orch::HwSession hardware;
    auto call=[&](std::string action,std::string rid="new"){
        Json::Value q;q["action"]=action;q["request_id"]=rid;q["session"]=session.status()["session"];q["hand"]=1;
        return session.execute(q,hardware);
    };
    check(!call("save")["ok"].asBool(),"save before session accepted");
    check(call("begin")["ok"].asBool() && session.active(),"begin failed");
    const auto id=session.status()["session"].asString();
    for(const auto &a:{"grip","move_d","jog","observe","record_coarse","lower_c","record_aligned","release","capture","reference","verify_run","return_ready","save"}) {
        check(!call(a,a)["ok"].asBool(),"out-of-order action accepted");
        check(hardware_calls==0,"out-of-order action reached hardware");
    }
    auto rejected=call("save","duplicate");auto repeated=call("save","duplicate");
    check(rejected==repeated,"request retry was not idempotent");
    session.interrupt();check(session.status()["interrupted"].asBool(),"stop not reflected in status");
    check(!call("prepare","stopped")["ok"].asBool() && hardware_calls==0,"stopped session reached hardware");
    check(call("end","end")["ok"].asBool() && !session.active(),"cannot exit stopped session");
    check(hardware_calls==0,"ending session moved/opened hardware");
    // Both tasks and hands share the simplified gates, not the old C/feeding workflow.
    for(const char *task:{"grasp","placement"}) for(int hand=0;hand<2;++hand) {
        Json::Value begin;begin["action"]="begin";begin["request_id"]="begin-task";begin["task"]=task;begin["hand"]=hand;
        const auto begun=session.execute(begin,hardware);
        check(begun["ok"].asBool() && begun["task"].asString()==task && begun["robot_row"].asInt()==5 &&
              begun["from_left_col"].asInt()==(hand?5:2),"task or fixed physical hole incorrect");
        for(const char *old:{"grip","lower_c","record_coarse","verify_run","verify_return"})
            check(!call(old,old)["ok"].asBool(),"obsolete feeding/C/replay action accepted");
        check(!call("record_aligned","too-early")["ok"].asBool() && !call("return_ready","no-save")["ok"].asBool(),
              "out-of-order D/save gate bypassed");
        check(call("end","task-end")["ok"].asBool(),"task did not end");
    }
    check(hardware_calls==0,"task ordering check touched hardware");
    check(!call("prepare","expired")["ok"].asBool(),"ended session accepted motion");
    check(std::filesystem::exists(root/"debug_picture/calibration"/id/"events.jsonl"),"event log missing");
    std::filesystem::remove_all(root);std::cout<<"session ordering, stop, replay and no-motion exit checks passed\n";
 } catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
}
