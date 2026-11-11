#include "t170/control/visual_servo.h"
#include "t170/common/config.h"
#include "t170/calibration/identity.h"
#include "t170/calibration/tray_policy.h"
#include "move_box_config.h"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
MoveBoxConfig g_move_cfg;
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
    record["robot_id"]=base.chassis.sequence_number;record["camera_serial"]=cameras["realsense"]["right_hand"]["serial"].as<std::string>();
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
    record["camera_serial"]=cameras["realsense"]["right_hand"]["serial"].as<std::string>();record["reference_px"][0]=1300;write();
    check(!load_move_box_config((root/"move_box_params.yaml").string(),loaded,error),"out-of-image calibration loaded");
    // New canonical file supersedes the old tilt catalog, without changing belt profiles.
    record["reference_px"][0]=701.5;record["schema_version"]=2;record["task"]="placement";
    record["scope"]="tray_only";record["validation_kind"]="manual_at_D";record["field_verified"]=false;write();
    {std::ofstream stale(root/"calibration/placement/right_tilt_stale.yaml");stale<<"bad legacy YAML [";}
    std::filesystem::create_directories(root/"calibration/grasp");
    auto grasp=YAML::Clone(record);grasp["task"]="grasp";grasp["reference_px"][0]=620.;grasp["reference_px"][1]=380.;
    grasp["reference_tcp_above_plane_m"]=.032;grasp["compensation_xy_upright_m"][0]=-.004;
    auto write_grasp=[&]{std::ofstream f(root/"calibration/grasp/right.yaml");f<<grasp;};write_grasp();
    check(load_move_box_config((root/"move_box_params.yaml").string(),loaded,error),error.c_str());
    check(calibrated_placement_for(loaded.visual_servo,1,.2,.7)->universal,"canonical reference still restricted to tilt");
    check(loaded.visual_servo.calibrated_placement[1].size()==1,"new and obsolete placement references were combined");
    check(calibrated_tray_grasp_for(loaded.visual_servo,1)->head_xy_bias.x()==-.004,"tray grasp compensation missing");
    for(int i=0;i<4;++i) check(loaded.visual_servo.profiles[i].reference_px==base.visual_servo.profiles[i].reference_px,
                             "loading tray catalog changed a global/belt image reference");
    check(loaded.conveyor.grasp_offset_left.x==base.conveyor.grasp_offset_left.x &&
          loaded.conveyor2.grasp_offset_right.y==base.conveyor2.grasp_offset_right.y,"tray catalog changed belt offsets");
    auto session=loaded.visual_servo;
    check(apply_tray_calibration(session,1,0,-.30,-.32,0,.7),"scoped tray grasp reference not applied");
    check(session.profiles[1].reference_px==Vec2(620,380) && std::abs(session.profiles[1].feature_z_offset_m-.02)<1e-12,
          "grasp used hole plane instead of object top plane");
    check(session.profiles[3].reference_px==base.visual_servo.profiles[3].reference_px,"grasp changed placement reference");
    session=loaded.visual_servo;
    check(apply_tray_calibration(session,1,1,-.34,-.32,0,.7) && session.profiles[3].reference_px.x()==701.5,
          "placement reference not applied for unified pose");
    check(session.profiles[1].reference_px==base.visual_servo.profiles[1].reference_px,"placement changed grasp reference");
    check(!apply_tray_calibration(session,0,0,-.3,-.32,0,0),"unmarked hand inherited the other hand's reference");
    grasp["scope"]="conveyor";write_grasp();
    check(!load_move_box_config((root/"move_box_params.yaml").string(),loaded,error),"wrong task scope accepted");
    check(calibrated_tray_grasp_for(loaded.visual_servo,1)->head_xy_bias.x()==-.004,"rejected reload lost last valid grasp");
    grasp["scope"]="tray_only";grasp["task"]="placement";write_grasp();
    check(!load_move_box_config((root/"move_box_params.yaml").string(),loaded,error),"placement file accepted in grasp catalog");
    g_move_cfg=base;
    const auto expected_r=base.tray2_place.right_row_rpy_deg[1],expected_l=base.tray2_place.left_row_rpy_deg[1];
    for(int row=1;row<=6;++row) for(bool right:{false,true}) {
        const auto &expected=right?expected_r:expected_l;const auto &actual=tray2_place_rpy_deg_for_row(row,right);
        check(actual.rx==expected.rx && actual.ry==expected.ry && actual.rz==expected.rz,"placement row retained a different pose");
        const auto &ready=tray2_place_rpy_deg_for_ready(row,right);
        check(ready.ry==expected.ry && ready.rz==expected.rz,"placement ready retained a different pose");
    }
    check(grasp_rpy_deg_for_row(1,true).ry==base.head_grasp.right_row6_rpy_deg.ry &&
          grasp_rpy_deg_for_row(5,true).rx==base.head_grasp.right_row_rpy_deg[1].rx,"grasp poses changed");
    for(bool rotated:{false,true}) {
        std::vector<TrayAnchor> grid;
        for(int r=1;r<=6;++r) for(int c=1;c<=6;++c)
            grid.push_back({rotated?7-r:r,rotated?7-c:c,Vec3(.9-.075*r,.30-.075*c,-.3),1});
        for(int hand=0;hand<2;++hand) {
            const auto selected=t170::calibration::teaching_hole(grid,hand);
            check(std::abs(selected.base.x()-(.9-.075*5))<1e-12 &&
                  std::abs(selected.base.y()-(.30-.075*(hand?5:2)))<1e-12,"robot-view teaching hole swapped with ArUco indices");
        }
    }
    check(t170::calibration::valid_teach_jog(.005,0) && !t170::calibration::valid_teach_jog(.0051,0) &&
          !t170::calibration::valid_teach_jog(.004,.004),"5mm total XY jog bound incorrect");
    std::filesystem::remove_all(root);std::cout<<"tray calibration geometry, task isolation, fixed poses and hole selection passed\n";
 } catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
}
