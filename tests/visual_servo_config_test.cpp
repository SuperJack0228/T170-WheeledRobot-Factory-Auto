#include "servo/visual_servo.h"
#include "move_box_config.h"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <fstream>

MoveBoxConfig g_move_cfg;
std::string project_root_dir() { return std::filesystem::current_path().string(); }

void require(bool ok,const char *message) { if(!ok) throw std::runtime_error(message); }
int main(int argc,char **argv) {
    try {
        if(argc!=2) throw std::runtime_error("usage: visual_servo_config_test config/move_box_params.yaml");
        auto root=YAML::LoadFile(argv[1]); wrist_servo::Config c;
        MoveBoxConfig full;
        std::string config_error;
        if(!load_move_box_config(argv[1],full,config_error)) throw std::runtime_error(config_error);
        require(full.head_grasp.retract_lift_vel_m_s==.4,"grasp lift configuration not loaded");
        require(full.tray2_place.retract_lift_vel_m_s==.4 && full.tray2_place.pre_open_settle_sec==0,
                "tray placement motion configuration not loaded");
        const auto retract_config=(std::filesystem::temp_directory_path()/"t170_retract_config_test.yaml").string();
        auto changed=YAML::Clone(root);
        changed["head_grasp"]["retract_lift_vel_m_s"]=.31;
        changed["head_grasp"]["retract_connector_speed_rad_s"]=.9;
        changed["head_grasp"]["retract_connector_accel_rad_s2"]=4.2;
        changed["head_grasp"]["retract_connector_jerk_rad_s3"]=50;
        changed["head_grasp"]["retract_connector_max_s"]=.3;
        { std::ofstream out(retract_config); out<<changed; }
        if(!load_move_box_config(retract_config,full,config_error)) throw std::runtime_error(config_error);
        require(full.head_grasp.retract_lift_vel_m_s==.31 &&
                full.head_grasp.retract_connector_speed_rad_s==.9 &&
                full.head_grasp.retract_connector_accel_rad_s2==4.2 &&
                full.head_grasp.retract_connector_jerk_rad_s3==50 &&
                full.head_grasp.retract_connector_max_s==.3,"external retract tuning ignored by production loader");
        changed["head_grasp"]["retract_lift_vel_m_s"]=0;
        { std::ofstream out(retract_config); out<<changed; }
        require(!load_move_box_config(retract_config,full,config_error),"zero retract speed silently accepted");
        for(const char *key:{"retract_lift_vel_m_s","retract_connector_speed_rad_s",
                            "retract_connector_accel_rad_s2","retract_connector_jerk_rad_s3","retract_connector_max_s"})
            changed["head_grasp"].remove(key);
        { std::ofstream out(retract_config); out<<changed; }
        if(!load_move_box_config(retract_config,full,config_error)) throw std::runtime_error(config_error);
        require(full.head_grasp.retract_lift_vel_m_s==.4 && full.head_grasp.retract_connector_jerk_rad_s3==80,
                "old YAML missing optional retract keys has no defaults");
        std::filesystem::remove(retract_config);
        c.profiles[0].reference_px << root["cameras"]["left_gripper_u"].as<double>(),root["cameras"]["left_gripper_v"].as<double>();
        c.profiles[1].reference_px << root["cameras"]["right_gripper_u"].as<double>(),root["cameras"]["right_gripper_v"].as<double>();
        c.profiles[2].reference_px=c.profiles[0].reference_px;
        c.profiles[3].reference_px=c.profiles[1].reference_px;
        wrist_servo::load_config(root["visual_servo"],c);
        require(c.tray_place.enabled && c.tray_place.min_neighbors==2,"new tray placement mode not loaded");
        require(c.period_s==.01 && c.stable_frames>=2,"actual controller parameters loaded");
        require(c.profiles[0].reference_px.x()==598 && c.profiles[1].reference_px.x()==702,"per-hand profiles loaded");
        const auto snapshot=(std::filesystem::temp_directory_path()/"t170_servo_config_roundtrip.yaml").string();
        std::string error;
        require(wrist_servo::write_config_snapshot(c,snapshot,error),"snapshot written");
        wrist_servo::Config replay;
        wrist_servo::load_config(YAML::LoadFile(snapshot)["visual_servo"],replay);
        require(replay.tray_place.enabled==c.tray_place.enabled &&
                replay.tray_place.board_config==c.tray_place.board_config &&
                replay.tray_place.association_gate_m==c.tray_place.association_gate_m,"tray vision snapshot roundtrip");
        require(replay.kp==c.kp && replay.kd==c.kd && replay.commit_height_m==c.commit_height_m &&
            replay.grasp_xy_accept_m==c.grasp_xy_accept_m && replay.grasp_height_margin_m==c.grasp_height_margin_m &&
            replay.hard_reference_lead_m==c.hard_reference_lead_m &&
            replay.enforce_sdk_joint_limits==c.enforce_sdk_joint_limits &&
            replay.max_joint_excursion_rad==c.max_joint_excursion_rad,"snapshot roundtrip");
        std::filesystem::remove(snapshot);
        bool rejected=false;
        try { auto bad=YAML::Load("kp: .nan"); wrist_servo::load_config(bad,c); }
        catch(const std::exception &) { rejected=true; }
        require(rejected,"invalid YAML gains rejected");
        rejected=false; c=wrist_servo::Config();
        try { auto bad=YAML::Load("left_grasp: {mode: table, knots: []}"); wrist_servo::load_config(bad,c); }
        catch(const std::exception &) { rejected=true; }
        require(rejected,"empty calibration table rejected");
        c=wrist_servo::Config();
        const auto table=YAML::Load(R"(
left_grasp:
  mode: table
  knots:
    - {height_m: 0.0, reference_px: [600, 400], jacobian_px_per_m: [[-10000, 100], [100, 10000]]}
    - {height_m: 0.14, reference_px: [620, 410], jacobian_px_per_m: [[-5000, 50], [50, 5000]]}
)");
        wrist_servo::load_config(table,c);
        require(c.profiles[0].knots.size()==2,"measured table YAML parsed");
        std::cout << "visual_servo_config: actual YAML, snapshot and invalid-input checks passed\n";
        return 0;
    } catch(const std::exception &ex) { std::cerr << ex.what() << '\n'; return 1; }
}
