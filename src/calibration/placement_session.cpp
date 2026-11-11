#include "t170/calibration/placement_session.h"
#include "t170/calibration/identity.h"
#include "t170/calibration/tray_policy.h"
#include "robot_runtime.h"
#include "function.h"
#include "move_box_runtime.h"
#include "move_box_config.h"
#include "servo/runtime.h"
#include "t170/motion/pose_solver.h"
#include "t170/hardware/head.h"
#include "t170/hardware/device_bus.h"
#include "t170/vision/tray_localizer.h"
#include "t170/vision/calibration_hole.h"
#include "t170/vision/pipeline.h"
#include "t170/common/abort_query.h"
#include <yaml-cpp/yaml.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <thread>
#include <cmath>
#include <algorithm>

namespace t170::calibration {
namespace {
using Pose=Eigen::Matrix<double,1,6>;
using Joints=wrist_servo::Joints;
using Clock=std::chrono::steady_clock;
void require(bool ok,const std::string &why) {if(!ok) throw std::runtime_error(why);}
Json::Value array(const Eigen::Ref<const Eigen::RowVectorXd> &v) {
    Json::Value a(Json::arrayValue);for(int i=0;i<v.cols();++i) a.append(v(i));return a;
}
void atomic_json(const std::filesystem::path &path,const Json::Value &data) {
    std::filesystem::create_directories(path.parent_path());
    const auto temp=path.string()+".tmp";
    {std::ofstream out(temp);require(bool(out),"无法写入标定文件");out<<data<<'\n';out.flush();require(bool(out),"标定文件写入失败");}
    std::filesystem::rename(temp,path);
}
}
struct PlacementSession::Impl {
    mutable std::mutex mutex;
    Json::Value view{Json::objectValue};
    std::atomic<bool> active{false},interrupted{false};
    int hand=1,step=0,row=0,col=0;
    bool grasp=false;
    std::string id,phase="idle",serial,eye_hash,model_hash,saved_file;
    std::filesystem::path directory;
    orch::HwSession *hw=nullptr;
    Pose initial=Pose::Zero(),nominal=Pose::Zero(),aligned=Pose::Zero(),reference_pose=Pose::Zero(),latest=Pose::Zero();
    Eigen::Matrix4d frame=Eigen::Matrix4d::Identity(),base_camera=Eigen::Matrix4d::Identity();
    Joints ready=Joints::Zero();
    Eigen::Vector2d coarse=Eigen::Vector2d::Zero(),bias=Eigen::Vector2d::Zero(),reference=Eigen::Vector2d::Zero();
    double clearance=.02,plane=0,feature_plane=0,reference_above_plane=0,reference_fit_residual=0;
    std::string reference_method;
    int reference_coverage=0;
    bool held=false,aligned_recorded=false,have_reference=false;
    CameraFrameData frozen;
    std::uint64_t image_revision=0,head_revision=0;
    std::vector<TrayHoleResult> holes;
    wrist_servo::TrayTarget target;
    Json::Value candidates{Json::arrayValue};
    std::map<std::string,Json::Value> replies;
    Robot_Arm &arm() {return hand?*hw->arm_r:*hw->arm_l;}
    gripper::Side side() const {return hand?gripper::Side::Right:gripper::Side::Left;}
    std::string side_name() const {return hand?"right":"left";}
    void stopped() const {require(!interrupted && !hardware_abort_requested() && !can_io_faulted(),"标定已停止，请退出会话后重新开始；不会自动松爪或回位");}
    Pose actual() {auto p=arm_get_tcp_pos(arm());require(p.allFinite(),"TCP反馈无效");latest=p;return p;}
    void state(const std::string &p,int s) {phase=p;step=s;publish();}
    void publish() {
        Json::Value v(Json::objectValue);v["ok"]=true;v["active"]=active.load();v["session"]=id;
        v["hand"]=hand;v["task"]=grasp?"grasp":"placement";v["scope"]="tray_only";v["step"]=step;v["phase"]=phase;v["held"]=held;
        v["interrupted"]=interrupted.load();v["robot_row"]=5;v["from_left_col"]=hand?5:2;v["row"]=row;v["col"]=col;
        v["d_clearance_mm"]=clearance*1000;v["holes"]=candidates;v["validated"]=!saved_file.empty();v["field_verified"]=false;
        v["coarse_xy_mm"]=array((coarse*1000).transpose().eval());
        v["compensation_xy_mm"]=array((bias*1000).transpose().eval());
        v["reference_px"]=array(reference.transpose());v["reference_tcp_above_plane_mm"]=reference_above_plane*1000;
        v["nominal_tcp"]=array(nominal);v["aligned_tcp"]=array(aligned);v["current_tcp"]=array(latest);v["plane_z_m"]=plane;v["reference_pose_tcp"]=array(reference_pose);
        v["head_image_revision"]=Json::UInt64(head_revision);
        if(head_revision)v["head_image_path"]=std::filesystem::relative(directory/"head.jpg",project_root_dir()).string();
        v["image_tcp_z_m"]=reference_pose(2);v["image_revision"]=Json::UInt64(image_revision);v["width"]=frozen.width;v["height"]=frozen.height;
        v["reference_fit_residual"]=reference_fit_residual;v["reference_coverage"]=reference_coverage;v["have_reference"]=have_reference;v["reference_method"]=reference_method;v["feature_plane_z_m"]=feature_plane;
        if(image_revision) v["image_path"]=std::filesystem::relative(directory/("frame_"+std::to_string(image_revision)+".jpg"),project_root_dir()).string();
        v["log_directory"]=directory.string();v["saved_file"]=saved_file;
        {std::lock_guard<std::mutex> lock(mutex);view=v;}
        if(!directory.empty()) atomic_json(directory/"session.json",v);
    }
    void audit(const std::string &action,const Json::Value &result,const Json::Value &request) {
        if(directory.empty()) return;
        Json::Value r(Json::objectValue);r["action"]=action;r["phase"]=phase;r["result"]=result;r["request"]=request;
        r["time_s"]=wrist_servo::monotonic_seconds();
        r["host_epoch_s"]=std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
        Json::StreamWriterBuilder b;b["indentation"]="";
        std::ofstream out(directory/"events.jsonl",std::ios::app);out<<Json::writeString(b,r)<<'\n';
    }
    void check_workspace(const Pose &p) {
        const auto &w=g_move_cfg.grasp_valid;
        require(p.allFinite() && p(0)>=w.x_min && p(0)<=w.x_max && p(2)>=w.z_min && p(2)<=w.z_max &&
            p(1)>=(hand?w.right_y_min:w.left_y_min) && p(1)<=(hand?w.right_y_max:w.left_y_max),"目标超出当前手臂工作包络");
        const Pose peer=arm_get_tcp_pos(hand?*hw->arm_l:*hw->arm_r);
        require((p.head<3>()-peer.head<3>()).norm()>=g_move_cfg.visual_servo.pair_clearance_m,"目标距离另一只手过近");
    }
    void line(Pose goal,double speed) {
        stopped();check_workspace(goal);
        Joints previous;wrist_servo::take_final_hold(hand,previous);
        const int rc=arm_line_move(arm(),goal,speed,false);
        require(rc==0,"机械臂接近失败 code="+std::to_string(rc));
        require(record_arm_cartesian_arrival(arm(),goal,"标定到位",4.0),"标定接近到位反馈超时");stopped();
        wrist_servo::hold_joint_reference(hand,arm_get_joint_pos(arm()));wrist_servo::start_final_hold_repeat();
    }
    // Short teach moves use the same FK/local IK math, continuous seeds and rest-to-rest timing.
    // A full path is checked before sending any point; never use uninitialised servo-session limits.
    void local(Pose goal) {
        stopped();check_workspace(goal);const Pose start=actual();
        require((goal.head<3>()-start.head<3>()).norm()<=.16,"微调/下降距离过大");
        const Joints seed=arm_get_joint_pos(arm());
        auto cfg=g_move_cfg.visual_servo;std::vector<Joints> path;
        const Eigen::Quaterniond start_rotation(TR(start).block<3,3>(0,0)),goal_rotation(TR(goal).block<3,3>(0,0));
        double duration=std::max(.35,1.9*(goal.head<3>()-start.head<3>()).norm()/.015);
        bool planned=false;
        for(int attempt=0;attempt<5 && !planned;++attempt,duration*=1.7) {
            const int count=static_cast<int>(std::ceil(duration/.01));path.clear();path.push_back(seed);
            Joints q=seed,previous_v=Joints::Zero();planned=true;
            for(int i=1;i<=count;++i) {
                const double u=double(i)/count,f=u*u*u*(10+u*(-15+6*u));
                Eigen::Vector3d p=start.head<3>().transpose()+f*(goal.head<3>()-start.head<3>()).transpose();
                Joints next;double xyz=0,rot=0;
                const auto fk=[&](const Joints &j){return arm().Forward_Kinematics(Eigen::RowVectorXd(j));};
                const auto limit=[&](const Joints &j){return (j-seed).cwiseAbs().maxCoeff()>cfg.max_joint_excursion_rad ||
                    (cfg.enforce_sdk_joint_limits && arm().Check_Joint_Limit(Eigen::RowVectorXd(j)));};
                if(!wrist_servo::solve_pose_with_fk(q,p,start_rotation.slerp(f,goal_rotation).toRotationMatrix(),cfg,fk,limit,next,xyz,rot)) {planned=false;break;}
                const Joints v=(next-q)/.01;
                if(v.cwiseAbs().maxCoeff()>cfg.max_joint_speed_rad_s ||
                   ((v-previous_v)/.01).cwiseAbs().maxCoeff()>cfg.max_joint_accel_rad_s2) {planned=false;break;}
                previous_v=v;q=next;path.push_back(q);
            }
        }
        require(planned,"局部微调轨迹无法平滑求解，尚未下发");
        Joints previous;wrist_servo::take_final_hold(hand,previous);
        for(std::size_t i=0;i<path.size();++i) {
            const auto cycle=Clock::now();stopped();wrist_servo::send_joint_reference(hand,path[i]);
            if(i%10==0) {
                const auto measured=arm_get_joint_pos(arm());
                if((measured-path[i]).cwiseAbs().maxCoeff()>.18) {
                    wrist_servo::hold_joint_reference(hand,measured);wrist_servo::start_final_hold_repeat();
                    throw std::runtime_error("微调跟随偏差过大，保持当前关节位置");
                }
            }
            std::this_thread::sleep_until(cycle+std::chrono::milliseconds(10));
        }
        wrist_servo::hold_joint_reference(hand,path.back());wrist_servo::start_final_hold_repeat();
        require(record_arm_cartesian_arrival(arm(),goal,"标定微调",3.0),"微调到位反馈超时");actual();stopped();
    }
    void check_held() {
        stopped();require(hw->g->hasSide(side()),"所选夹爪未连接");
        const auto fb=hw->g->feedback(side());
        require(fb.have_feedback && fb.io_ok && fb.merror==0 && std::isfinite(fb.position_rad) &&
            fb.position_rad>=g_move_cfg.gripper_check.empty_q_rad && fb.position_rad<=g_move_cfg.gripper_check.hold_q_max_rad,
            "未确认正常持件：请先退出标定，使用实际传送带抓取流程取件；不要人工喂件");
        held=true;
    }
    void close_empty() {
        stopped();require(hw->g->hasSide(side()),"所选夹爪未连接");
        // Retain torque protection, but success here means EMPTY position reached,
        // unlike a production grasp which expects a contact/torque event.
        const auto closed=hw->g->graspAndWait(side(),-1.0,std::chrono::milliseconds(3000));
        const bool maintained=hw->g->holdCurrent(side());
        if(!maintained) interrupted=true;
        const auto fb=hw->g->feedback(side());
        require(closed.result==gripper::GraspResult::PositionReached && maintained && fb.have_feedback && fb.io_ok && fb.merror==0 &&
                std::isfinite(fb.position_rad) && fb.position_rad<=g_move_cfg.gripper_check.empty_q_rad,
                "空爪闭合未到位或检测到夹持物，请检查并重新进入；不会继续顶着闭合");
        held=false;stopped();
    }
    void open() {
        stopped();require(hw->g->hasSide(side()),"夹爪未连接");
        require(hw->g->openAndWait(side(),g_move_cfg.tray2_place.open_tolerance_rad,
            std::chrono::milliseconds(g_move_cfg.tray2_place.open_timeout_ms)),"张爪未确认");
        const auto fb=hw->g->feedback(side());require(fb.have_feedback && fb.io_ok && fb.merror==0,"张爪反馈异常");
        held=false;
    }
    void camera_identity() {
        auto root=std::filesystem::path(default_move_box_config_path()).parent_path();
        const auto cameras=YAML::LoadFile((root/"realsense_cameras.yaml").string());
        serial=cameras["realsense"][side_name()+"_hand"]["serial"].as<std::string>();
        eye_hash=fingerprint(root/(side_name()+"_hand_to_base_result.yaml"));model_hash=fingerprint(root/"Robot_Arm_Model.yaml");
    }
    const GraspRpyDeg &rpy() const {
        return grasp?grasp_rpy_deg_for_row(5,hand==1):tray2_place_rpy_deg_for_row(5,hand==1);
    }
    std::vector<wrist_servo::TrayAnchor> anchors() const {
        std::vector<wrist_servo::TrayAnchor> result;
        for(const auto &h:holes) if(h.in_robot && std::isfinite(h.x_level) && std::isfinite(h.y_level))
            result.push_back({h.row,h.col,Eigen::Vector3d(h.x_level,h.y_level,move_box::tray_z_ref_now()),h.class_id});
        return result;
    }
    void head() {
        stopped();PerceiveTrayOptions options;options.tray2_markers=!grasp;options.run_yolo=true;
        const auto result=vision::perceive_tray_head(hw->pipeline->cameras(),hw->pipeline->bridge(),options);
        require(result.tray.ok,"头相机未定位到料盘: "+result.tray.message);
        if(!result.tray.save_path.empty() && std::filesystem::is_regular_file(result.tray.save_path)) {
            std::filesystem::copy_file(result.tray.save_path,directory/"head.jpg",std::filesystem::copy_options::overwrite_existing);++head_revision;
        }
        holes=result.tray.holes;const auto selected=teaching_hole(anchors(),hand);row=selected.row;col=selected.col;
        candidates=Json::Value(Json::arrayValue);Json::Value c(Json::objectValue);
        c["row"]=row;c["col"]=col;c["class_id"]=selected.class_id;c["x"]=selected.base.x();c["y"]=selected.base.y();candidates.append(c);
    }
    void bind() {
        const auto all=anchors();const auto selected=teaching_hole(all,hand);
        row=selected.row;col=selected.col;frame=move_box::placement_calibration_frame();
        wrist_servo::TrayBoardGeometry board;std::string error;
        const auto configured=std::filesystem::path(g_move_cfg.visual_servo.tray_place.board_config);
        const auto path=configured.is_absolute()?configured:std::filesystem::path(project_root_dir())/configured;
        require(wrist_servo::load_tray_board(path.string(),board,error),error);
        require(wrist_servo::make_tray_target(all,board,row,col,target,error),error);
        // Exactly the production fixed horizontal tray; no head depth/tilt.
        const double yaw=std::atan2(target.base_from_tray(1,0),target.base_from_tray(0,0));
        target.base_from_tray.block<3,3>(0,0)=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();
        target.base_from_tray(2,3)=move_box::tray_z_ref_now();target.base_from_tray=frame*target.base_from_tray;
        const auto xy=board.hole(row,col);plane=(target.base_from_tray*Eigen::Vector4d(xy.x(),xy.y(),0,1)).z();
        double zoff=0;
        if(grasp) {
            const auto &p=g_move_cfg.head_grasp;
            zoff=(hand?p.right_grasp_z_offset_m:p.grasp_z_offset_m)+(p.use_row_offset?(hand?p.right_row_offset[4].z:p.left_row_offset[4].z):0.0);
        } else {
            const auto &p=g_move_cfg.tray2_place;
            zoff=(hand?p.right_grasp_z_offset_m:p.grasp_z_offset_m)+(p.use_row_offset?(hand?p.right_row_offset[4].z:p.left_row_offset[4].z):0.0);
        }
        const auto &r=rpy();Pose upright;
        upright<<selected.base.x(),selected.base.y(),move_box::expected_object_top_z()+zoff,r.rx*M_PI/180,r.ry*M_PI/180,r.rz*M_PI/180;
        nominal=T2PosEulerAngles(frame*TR(upright));initial=nominal;initial(2)+=g_move_cfg.head_grasp.hover_above_m;
        feature_plane=grasp?(frame*Eigen::Vector4d(selected.base.x(),selected.base.y(),move_box::expected_object_top_z(),1)).z():plane;
        require(initial(2)>nominal(2)+clearance,"观察高度必须高于D点");check_workspace(nominal);
    }
    void retract() {
        stopped();const double lift=std::max(.08,initial(2)-actual()(2));
        require(arm_vertical_lift(arm(),lift,.08)==0,"标定竖直撤离失败，保持夹爪状态");
        require(arm_return_after_lift(arm(),ready)==0,"标定返回准备位失败");stopped();
    }
    Eigen::Matrix4d camera(const Pose &p) {
        std::array<double,16> values;std::string error;
        require(load_cam2robot_matrix(hand?default_right_hand_to_robot_yaml_path():default_left_hand_to_robot_yaml_path(),values,error),error);
        Eigen::Matrix4d eye;for(int r=0;r<4;++r) for(int c=0;c<4;++c) eye(r,c)=values[r*4+c];
        return TR(p)*wrist_servo::tool_transform(arm()).inverse()*eye;
    }
    void capture(bool check_alignment=true) {
        stopped();const Pose before=actual();
        require(!check_alignment || ((before.head<3>()-aligned.head<3>()).norm()<=.002 &&
            Eigen::AngleAxisd(TR(before).block<3,3>(0,0)*TR(aligned).block<3,3>(0,0).transpose()).angle()<=.015),
            "松爪取件后位姿发生变化，不能直接使用这张图标定；请重新对准");
        auto &cams=hw->pipeline->cameras();auto next=cams.grab_fresh(hand?CameraSlot::RightHand:CameraSlot::LeftHand);
        require(next.ok,"手相机取图失败: "+next.message);
        if(!hand) {
            cv::Mat bgr(next.height,next.width,CV_8UC3,next.color_bgr.data());cv::rotate(bgr,bgr,cv::ROTATE_180);
            next.K[2]=next.width-1-next.K[2];next.K[5]=next.height-1-next.K[5];next.dist[2]*=-1;next.dist[3]*=-1;
        }
        const Pose next_pose=actual();require((before.head<3>()-next_pose.head<3>()).norm()<=.001,"拍照时机械臂未稳定");
        const auto next_camera=camera(next_pose);const auto revision=image_revision+1;
        cv::Mat image(next.height,next.width,CV_8UC3,next.color_bgr.data());
        require(cv::imwrite((directory/("frame_"+std::to_string(revision)+".jpg")).string(),image),"标定图像写入失败");
        Json::Value m;m["tcp"]=array(next_pose);m["aligned_tcp"]=array(aligned);m["plane_z_m"]=plane;m["feature_plane_z_m"]=feature_plane;
        m["frame"]=Json::UInt64(next.frame_number);m["capture_s"]=next.capture_steady_s;
        m["K"]=Json::Value(Json::arrayValue);for(auto v:next.K)m["K"].append(v);
        m["dist"]=Json::Value(Json::arrayValue);for(auto v:next.dist)m["dist"].append(v);
        atomic_json(directory/("frame_"+std::to_string(revision)+".json"),m);
        // Commit pixels, pose and image revision together. A rejected re-capture
        // must never leave an old displayed image paired with new internal pixels.
        frozen=std::move(next);reference_pose=next_pose;base_camera=next_camera;image_revision=revision;have_reference=false;
    }

    Eigen::Vector2d refine(double u,double v) {
        require(std::isfinite(u) && std::isfinite(v) && u>=0 && v>=0 && u<frozen.width && v<frozen.height,"参考点超出当前图像");
        if(grasp) {reference_method="manual_part_top_center";reference_fit_residual=0;reference_coverage=0;return {u,v};}
        const auto fit=vision::refine_calibration_hole(frozen,base_camera,target.base_from_tray,target.board.radius_m,u,v);
        require(fit.valid,fit.reason);reference_method="rectified_orange_hole_center";reference_fit_residual=fit.residual_rectified_px;reference_coverage=fit.coverage_bins;return fit.pixel;
    }
    Json::Value record(bool validated) {
        Json::Value r;r["schema_version"]=2;r["validated"]=validated;
        r["task"]=grasp?"grasp":"placement";r["scope"]="tray_only";r["validation_kind"]="manual_at_D";r["field_verified"]=false;
        r["reference_method"]=reference_method;r["feature_plane_z_m"]=feature_plane;r["robot_row"]=5;r["from_left_col"]=hand?5:2;r["hand"]=hand;r["session"]=id;
        r["robot_id"]=g_move_cfg.chassis.sequence_number;r["camera_serial"]=serial;
        r["eye_fingerprint"]=eye_hash;r["model_fingerprint"]=model_hash;
        r["width"]=frozen.width;r["height"]=frozen.height;r["row"]=row;r["col"]=col;r["d_clearance_m"]=clearance;
        r["reference_fit_residual_rectified_px"]=reference_fit_residual;r["reference_coverage_bins"]=reference_coverage;
        r["reference_px"]=array(reference.transpose());r["reference_tcp_above_plane_m"]=reference_above_plane;
        r["reference_pose_tcp"]=array(reference_pose);r["aligned_tcp"]=array(aligned);r["plane_z_m"]=plane;
        r["compensation_xy_upright_m"]=array(bias.transpose());r["coarse_xy_upright_m"]=array(coarse.transpose());
        r["K"]=Json::Value(Json::arrayValue);for(double v:frozen.K)r["K"].append(v);
        r["dist"]=Json::Value(Json::arrayValue);for(double v:frozen.dist)r["dist"].append(v);
        r["pixel_coordinates"]="raw_color_left_rotated_180";r["log_directory"]=directory.string();return r;
    }
};
PlacementSession::PlacementSession():impl_(std::make_unique<Impl>()) {impl_->publish();}
PlacementSession::~PlacementSession()=default;
bool PlacementSession::active() const {return impl_->active.load();}
Json::Value PlacementSession::status() const {std::lock_guard<std::mutex> lock(impl_->mutex);auto v=impl_->view;v["interrupted"]=impl_->interrupted.load();return v;}
void PlacementSession::interrupt() {if(active()) impl_->interrupted=true;}
Json::Value PlacementSession::execute(const Json::Value &request,orch::HwSession &hardware) {
    auto &s=*impl_;const std::string action=request.get("action","").asString();
    const std::string rid=request.get("request_id","").asString();
    if(action!="begin") {
        if(!s.active || request.get("session","").asString()!=s.id) return []{Json::Value r;r["ok"]=false;r["error"]="标定会话已失效，请刷新";return r;}();
        if(auto it=s.replies.find(rid);!rid.empty() && it!=s.replies.end()) return it->second;
    }
    Json::Value out;
    try {
        require(!rid.empty() && rid.size()<120,"缺少动作唯一编号");s.hw=&hardware;
        if(action=="begin") {
            require(!s.active,"已有标定会话，请恢复或退出");
            require(request["hand"].isInt() && (request["hand"].asInt()==0 || request["hand"].asInt()==1),"请选择左手或右手");
            s.hand=request["hand"].asInt();const auto task=request.get("task","placement").asString();
            require(task=="grasp" || task=="placement","标定任务必须为料盘抓取或料盘放置");s.grasp=task=="grasp";
            s.clearance=request.get("clearance_mm",20.).asDouble()/1000;
            require(std::isfinite(s.clearance) && s.clearance>=.01 && s.clearance<=.02,"D点距离必须在10–20mm之间");
            require(!s.hand || !right_arm_motors_locked(),"右手电机当前锁定");
            s.id=std::to_string(std::chrono::duration_cast<std::chrono::microseconds>(Clock::now().time_since_epoch()).count());
            s.directory=std::filesystem::path(project_root_dir())/"debug_picture"/"calibration"/s.id;
            s.replies.clear();s.held=s.have_reference=s.aligned_recorded=false;
            s.reference_fit_residual=0;s.reference_coverage=0;s.reference_method.clear();s.feature_plane=s.plane=s.reference_above_plane=0;
            s.image_revision=0;s.head_revision=0;s.saved_file.clear();s.frozen={};s.nominal.setZero();s.aligned.setZero();s.reference_pose.setZero();s.latest.setZero();s.row=s.col=0;s.coarse.setZero();s.bias.setZero();s.reference.setZero();s.candidates=Json::Value(Json::arrayValue);
            s.camera_identity();
            std::filesystem::create_directories(s.directory/"inputs");
            for(const auto &name:std::vector<std::string>{"move_box_params.yaml","realsense_cameras.yaml","Robot_Arm_Model.yaml",s.side_name()+"_hand_to_base_result.yaml"})
                std::filesystem::copy_file(std::filesystem::path(default_move_box_config_path()).parent_path()/name,s.directory/"inputs"/name);
            s.interrupted=false;s.active=true;s.state("selected",0);
        } else if(action=="end") {s.active=false;s.phase="ended";s.step=0;s.publish();}
        else {
            s.stopped();
            if(action=="prepare") {
                require(s.phase=="selected","准备动作只允许执行一次");
                require(s.hw->g && s.hw->arm_r && s.hw->arm_l && s.hw->waist && s.hw->layer3 && s.hw->pipeline,"标定所需硬件尚未初始化");
                require(request.get("setup_confirmed",false).asBool(),"请确认工位、第五行空盘以及空爪/实际传送带持件状态");
                if(!s.grasp) s.check_held(); // Must precede all preparation motion; never open a loaded gripper.
                require(enable_head_calib_pose()==0,"头部准备失败");
                for(const auto motor:s.arm().can_id_list) {s.stopped();require(motor_enable_id(static_cast<std::uint32_t>(motor)),"所选手臂电机使能失败");}
                Pose p=s.hand?g_move_cfg.standby.right:g_move_cfg.standby.left;const auto &r=s.rpy();
                p.tail<3>()<<r.rx*M_PI/180,r.ry*M_PI/180,r.rz*M_PI/180;s.line(p,.10);
                require(move_box::ensure_waist_ready_start(*s.hw->waist,*s.hw->layer3),"腰部到准备位失败");
                move_box::waist_layer3_set_synced(true);s.ready=arm_get_joint_pos(s.arm());
                if(s.grasp) s.close_empty();
                s.state("prepared",1);
            } else if(action=="detect") {require(s.phase=="prepared","请先完成准备");s.head();s.publish();}
            else if(action=="move_d") {
                require(s.phase=="prepared" && !s.candidates.empty(),"请先准备并识别固定标定孔");
                if(!s.grasp) s.check_held();s.bind();
                s.line(s.initial,.10);Pose d=s.nominal;d(2)+=s.clearance;s.local(d);s.state("at_d",2);s.capture(false);s.publish();
            } else if(action=="jog") {
                require(s.phase=="at_d","只能在D点对准页微调");
                const double dx=request.get("dx_mm",0.).asDouble()/1000,dy=request.get("dy_mm",0.).asDouble()/1000;
                require(valid_teach_jog(dx,dy),"单次XY微调合位移必须在5mm内");
                Pose p=s.actual();p.tail<3>()=s.nominal.tail<3>();p(0)+=dx;p(1)+=dy;
                require((p.head<2>()-s.nominal.head<2>()).norm()<=.10,"累计补偿超过100mm，请检查识别和工具参数");s.local(p);s.publish();s.capture(false);s.publish();
            } else if(action=="observe") {
                require(s.phase=="at_d","只允许在D点刷新观察画面");s.capture(false);s.publish();
            } else if(action=="record_aligned") {
                require(s.phase=="at_d" && request.get("physically_centered",false).asBool(),"请在D点确认竖直下降的工作中心已对准目标孔");
                s.aligned=s.actual();Eigen::Vector3d d=(s.aligned.head<3>()-s.nominal.head<3>()).transpose();d.z()=0;
                s.bias=s.frame.block<2,2>(0,0).inverse()*d.head<2>();s.coarse=s.bias;
                s.aligned_recorded=true;s.state("aligned",3);
            } else if(action=="release") {require(s.phase=="aligned","先记录D点对准位姿");s.open();s.state("released",3);}
            else if(action=="capture") {
                require((s.phase=="released" || s.phase=="captured") && request.get("scene_ready",false).asBool(),
                        s.grasp?"请原地张爪并将居中的标定零件放入目标孔":"请张爪并取走零件，露出同一个目标孔");
                s.capture();s.state("captured",4);
            }
            else if(action=="reference") {
                require(s.phase=="captured" && request["image_revision"].asUInt64()==s.image_revision,"图像已更新，请重新点击当前图像");
                require(request["u"].isNumeric() && request["v"].isNumeric(),"请点击当前图像中的参考中心");
                const auto now=s.actual();
                require((now.head<3>()-s.reference_pose.head<3>()).norm()<=.001 &&
                    Eigen::AngleAxisd(TR(now).block<3,3>(0,0)*TR(s.reference_pose).block<3,3>(0,0).transpose()).angle()<=.01,
                    "拍照后位姿发生变化，请重新拍摄或对准");
                s.reference=s.refine(request["u"].asDouble(),request["v"].asDouble());
                s.reference_above_plane=s.reference_pose(2)-s.feature_plane;s.have_reference=true;
                atomic_json(s.directory/"draft.yaml",s.record(false));s.state("reference",5);
            } else if(action=="return_ready") {
                require(s.phase=="saved","请先保存标定；退出本身不会运动");s.retract();s.state("returned",5);
            } else if(action=="save") {
                require(s.phase=="reference" && s.have_reference && s.aligned_recorded && request.get("reference_confirmed",false).asBool(),
                        "请先确认D点对准和参考点；不需要下到C点或重新装件回放");
                const auto dir=std::filesystem::path(default_move_box_config_path()).parent_path()/"calibration"/(s.grasp?"grasp":"placement");
                const std::string name=s.side_name();const auto path=dir/(name+".yaml");const auto data=s.record(true);
                const auto old=dir/(name+"_before_"+s.id+".yaml");
                if(std::filesystem::exists(path)) std::filesystem::copy_file(path,old);
                atomic_json(path,data);MoveBoxConfig next;std::string error;
                const bool loaded=load_move_box_config(default_move_box_config_path(),next,error);
                const bool applied=loaded && (s.grasp?wrist_servo::calibrated_tray_grasp_for(next.visual_servo,s.hand):
                    wrist_servo::calibrated_placement_for(next.visual_servo,s.hand,s.nominal(3),s.nominal(4)));
                if(!applied) {
                    std::filesystem::remove(path);
                    if(std::filesystem::exists(old)) std::filesystem::copy_file(old,path);
                    throw std::runtime_error("标定未应用，已恢复旧版本: "+error);
                }
                g_move_cfg=std::move(next);s.saved_file=path.string();atomic_json(s.directory/"validated.yaml",data);s.state("saved",5);
            } else throw std::runtime_error("未知标定动作");
        }
        out=status();out["ok"]=true;
    } catch(const std::exception &e) {out=status();out["ok"]=false;out["error"]=e.what();}
    s.audit(action,out,request);if(action!="begin" && !rid.empty()) {if(s.replies.size()>100)s.replies.erase(s.replies.begin());s.replies[rid]=out;}
    return out;
}
}
