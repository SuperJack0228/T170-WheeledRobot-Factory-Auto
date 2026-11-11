#include "t170/control/visual_servo.h"
#include "t170/common/config.h"
#include "t170/calibration/identity.h"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
void check(bool v,const char *s){if(!v)throw std::runtime_error(s);}
int main(int argc,char **argv) {
 try {
    using namespace wrist_servo;
    ImageProfile p;p.reference_px={700,430};p.plane_relative_reference=true;p.reference_tcp_above_plane_m=.04;
    PixelObservation o;o.pixel={650,420};o.fx=o.fy=650;o.cx=640;o.cy=360;o.width=1280;o.height=720;
    State s;s.position={0,0,.18};Eigen::Matrix3d r=Eigen::Matrix3d::Identity();r(1,1)=r(2,2)=-1;
    Vec3 origin(.03,.02,.28);const double plane=-.01;
    p.feature_z_offset_m=plane;
    const auto a=map_observation(p,o,s,0,origin,r);p.feature_z_offset_m=plane-.025;
    const auto b=map_observation(p,o,s,.025,origin,r);
    check(a.valid&&b.valid,"plane anchor invalid");
    check((a.reference_px-b.reference_px).norm()<1e-10 && (a.remaining_m-b.remaining_m).norm()<1e-10,
          "changing C moved the plane-anchored reference");
    p.reference_tcp_above_plane_m=s.position.z()-plane;
    check((map_observation(p,o,s,.025,origin,r).reference_px-p.reference_px).norm()<1e-10,
          "reference at its teach height is not the input pixel");
    p.plane_relative_reference=false;p.reference_height_m=s.position.z()-.025;
    check((map_observation(p,o,s,.025,origin,r).reference_px-p.reference_px).norm()<1e-10,"legacy anchor changed");
    check(argc==2,"repository required");
    const auto root=std::filesystem::temp_directory_path()/"t170-calibration-config-test";
    std::filesystem::remove_all(root);std::filesystem::create_directories(root/"calibration/placement");
    const auto repository=std::filesystem::path(argv[1]);
    for(const auto &name:{"move_box_params.yaml","realsense_cameras.yaml","right_hand_to_base_result.yaml","Robot_Arm_Model.yaml"})
        std::filesystem::copy_file(repository/"config"/name,root/name);
    MoveBoxConfig base;std::string error;check(load_move_box_config((root/"move_box_params.yaml").string(),base,error),error.c_str());
    const auto cameras=YAML::LoadFile((root/"realsense_cameras.yaml").string());
    YAML::Node record;record["schema_version"]=1;record["validated"]=true;record["hand"]=1;
    record["robot_id"]=base.chassis.sequence_number;record["camera_serial"]=cameras["realsense"]["right_hand"]["serial"];
    record["width"]=base.cameras.width;record["height"]=base.cameras.height;
    record["eye_fingerprint"]=t170::calibration::fingerprint(root/"right_hand_to_base_result.yaml");
    record["model_fingerprint"]=t170::calibration::fingerprint(root/"Robot_Arm_Model.yaml");
    record["reference_px"].push_back(701.5);record["reference_px"].push_back(430.2);
    record["reference_tcp_above_plane_m"]=.025;
    for(double v:{.4,-.2,-.4,0.,.95,.7})record["reference_pose_tcp"].push_back(v);
    record["compensation_xy_upright_m"].push_back(.012);record["compensation_xy_upright_m"].push_back(-.008);
    auto write=[&]{std::ofstream f(root/"calibration/placement/right.yaml");f<<record;};write();
    MoveBoxConfig loaded;check(load_move_box_config((root/"move_box_params.yaml").string(),loaded,error),error.c_str());
    const auto *entry=calibrated_placement_for(loaded.visual_servo,1,0,.95);
    check(entry && entry->image.plane_relative_reference && entry->image.reference_px.x()==701.5,
          "saved reference not loaded");
    check(entry->head_xy_bias==Vec2(.012,-.008),"saved correction not loaded");
    check(!calibrated_placement_for(loaded.visual_servo,1,0,.7),"another tool tilt reused a wrong reference");
    check(loaded.tray2_place.right_goal_x_offset==base.tray2_place.right_goal_x_offset,"catalog overwrote explicit legacy fallback");
    check(loaded.visual_servo.profiles[1].reference_px==base.visual_servo.profiles[1].reference_px,"placement changed grasp reference");
    check(load_move_box_config((root/"move_box_params.yaml").string(),loaded,error),error.c_str());
    check(loaded.visual_servo.calibrated_placement[1].size()==1,"reload accumulated calibration entries");
    record["camera_serial"]="different-camera";write();
    check(!load_move_box_config((root/"move_box_params.yaml").string(),loaded,error),"camera change silently reused calibration");
    check(loaded.visual_servo.calibrated_placement[1].size()==1 && calibrated_placement_for(loaded.visual_servo,1,0,.95)->head_xy_bias==Vec2(.012,-.008),"rejected calibration changed last valid config");
    record["camera_serial"]=cameras["realsense"]["right_hand"]["serial"];record["reference_px"][0]=1300;write();
    check(!load_move_box_config((root/"move_box_params.yaml").string(),loaded,error),"out-of-image calibration loaded");
    std::filesystem::remove_all(root);std::cout<<"placement calibration geometry and configuration checks passed\n";
 } catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
}
