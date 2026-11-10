#include "t170/control/visual_servo.h"
#include "move_box_config.h"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <fstream>

MoveBoxConfig g_move_cfg;

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
        require(std::abs(full.retract_motion.ready_right(1)-root["retract_motion"]["ready_right_deg"][1].as<double>()*M_PI/180)<1e-9,
                "fixed ready joints not converted from degrees");
        require(full.retract_motion.max_joint_velocity_rad_s==root["retract_motion"]["max_joint_velocity_rad_s"].as<double>() &&
                full.retract_motion.stable_samples==root["retract_motion"]["stable_samples"].as<int>(),
                "joint motion limits not loaded");
        const auto retract_config=(std::filesystem::temp_directory_path()/"t170_retract_config_test.yaml").string();
        auto changed=YAML::Clone(root);
        changed["retract_motion"]["ready_left_deg"]=std::vector<double>{1,2,3};
        { std::ofstream out(retract_config);out<<changed; }
        require(!load_move_box_config(retract_config,full,config_error),"short joint target accepted");
        changed=YAML::Clone(root);
        changed["retract_motion"]["max_joint_jerk_rad_s3"]=0;
        { std::ofstream out(retract_config);out<<changed; }
        require(!load_move_box_config(retract_config,full,config_error),"zero jerk accepted");
        changed=YAML::Clone(root);
        changed["retract_motion"]["ready_right_deg"][0]=12.5;
        changed["retract_motion"]["max_joint_velocity_rad_s"]=.7;
        { std::ofstream out(retract_config);out<<changed; }
        require(load_move_box_config(retract_config,full,config_error),"valid retract override rejected");
        require(std::abs(full.retract_motion.ready_right(0)-12.5*M_PI/180)<1e-9 &&
                full.retract_motion.max_joint_velocity_rad_s==.7,"retract override ignored");
        changed=YAML::Clone(root);
        changed["head_grasp"]["retract_lift_vel_m_s"]=.31;
        changed["head_grasp"]["retract_connector_speed_rad_s"]=.9;
        changed["head_grasp"]["retract_connector_accel_rad_s2"]=4.2;
        changed["head_grasp"]["retract_connector_jerk_rad_s3"]=50;
        changed["head_grasp"]["retract_connector_max_s"]=.3;
        changed["head_grasp"]["rise_waist_with_lift"]=false;
        changed["head_grasp"]["waist_lift_start_clearance_m"]=.02;
        changed["head_grasp"]["retract_arrival_tolerance_m"]=.01;
        changed["head_grasp"]["retract_arrival_timeout_s"]=3.0;
        changed["conveyor"]["next_hand_clearance_m"]=.04;
        { std::ofstream out(retract_config); out<<changed; }
        if(!load_move_box_config(retract_config,full,config_error)) throw std::runtime_error(config_error);
        require(full.head_grasp.retract_lift_vel_m_s==.31 &&
                full.head_grasp.retract_connector_speed_rad_s==.9 &&
                full.head_grasp.retract_connector_accel_rad_s2==4.2 &&
                full.head_grasp.retract_connector_jerk_rad_s3==50 &&
                full.head_grasp.retract_connector_max_s==.3,"external retract tuning ignored by production loader");
        require(!full.head_grasp.rise_waist_with_lift && full.head_grasp.waist_lift_start_clearance_m==.02 &&
                full.head_grasp.retract_arrival_tolerance_m==.01 && full.head_grasp.retract_arrival_timeout_s==3.0 &&
                full.conveyor.next_hand_clearance_m==.04,"staged retreat/waist/peer tuning ignored by loader");
        changed["head_grasp"]["retract_lift_vel_m_s"]=0;
        { std::ofstream out(retract_config); out<<changed; }
        require(!load_move_box_config(retract_config,full,config_error),"zero retract speed silently accepted");
        require(full.head_grasp.retract_lift_vel_m_s==.31 &&
                full.head_grasp.retract_connector_accel_rad_s2==4.2,
                "rejected reload replaced the previous complete configuration");
        require(!load_move_box_config(retract_config+".missing",full,config_error) &&
                full.head_grasp.retract_lift_vel_m_s==.31,
                "missing file replaced live configuration with defaults");
        { std::ofstream out(retract_config); out<<"standby: [invalid YAML\n"; }
        require(!load_move_box_config(retract_config,full,config_error) &&
                full.head_grasp.retract_lift_vel_m_s==.31,
                "malformed file replaced live configuration");
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
        require(c.tray_place.enabled && c.tray_place.min_neighbors==2 && c.tray_place.xy_only,
                "tray place open-loop for hand photo run, neighbor params loaded");
        require(c.belt_grasp_enabled==root["visual_servo"]["belt_grasp_enabled"].as<bool>() &&
                c.belt_place_enabled==root["visual_servo"]["belt_place_enabled"].as<bool>(),
                "belt grasp/place servo switches differ from YAML");
        require(c.period_s==.01 && c.stable_frames>=2,"actual controller parameters loaded");
        require(c.profiles[0].reference_px.x()==root["cameras"]["left_gripper_u"].as<double>() &&
                c.profiles[1].reference_px.x()==root["cameras"]["right_gripper_u"].as<double>(),
                "grasp reference pixels");
        require(c.profiles[2].reference_px.x()==root["visual_servo"]["left_place"]["reference_px"][0].as<double>() &&
                c.profiles[2].reference_px.y()==root["visual_servo"]["left_place"]["reference_px"][1].as<double>() &&
                c.profiles[3].reference_px.x()==root["visual_servo"]["right_place"]["reference_px"][0].as<double>() &&
                c.profiles[3].reference_px.y()==root["visual_servo"]["right_place"]["reference_px"][1].as<double>(),
                "place reference pixels");
        require(wrist_servo::profile_for(c,0,0).reference_px.x()==c.profiles[0].reference_px.x() &&
                wrist_servo::profile_for(c,1,1).reference_px.x()==c.profiles[3].reference_px.x(),
                "place servo uses the place reference");
        const auto snapshot=(std::filesystem::temp_directory_path()/"t170_servo_config_roundtrip.yaml").string();
        std::string error;
        require(wrist_servo::write_config_snapshot(c,snapshot,error),"snapshot written");
        wrist_servo::Config replay;
        wrist_servo::load_config(YAML::LoadFile(snapshot)["visual_servo"],replay);
        require(replay.tray_place.enabled==c.tray_place.enabled &&
                replay.tray_place.board_config==c.tray_place.board_config &&
                replay.tray_place.association_gate_m==c.tray_place.association_gate_m,"tray vision snapshot roundtrip");
        require(replay.tray_place.affine_enabled==c.tray_place.affine_enabled &&
                replay.tray_place.observe_at_B==c.tray_place.observe_at_B &&
                replay.tray_place.observe_timeout_s==c.tray_place.observe_timeout_s &&
                replay.tray_place.affine_min_scale==c.tray_place.affine_min_scale &&
                replay.tray_place.affine_max_scale==c.tray_place.affine_max_scale &&
                replay.tray_place.affine_max_rotation_rad==c.tray_place.affine_max_rotation_rad &&
                replay.tray_place.max_extrapolation_pitches==c.tray_place.max_extrapolation_pitches,
                "B-observation / affine parameters lost in snapshot");
        require(replay.place_lock_height_m==c.place_lock_height_m &&
                replay.place_visibility_floor_m==c.place_visibility_floor_m &&
                replay.place_xy_accept_m==c.place_xy_accept_m && replay.place_lock_frames==c.place_lock_frames &&
                replay.tray_place.min_coverage_bins==c.tray_place.min_coverage_bins &&
                replay.grasp_z_feedback_tolerance_m==c.grasp_z_feedback_tolerance_m,
                "placement visibility and feedback tolerances lost in snapshot");
        require(replay.tray_place.target_hold_s==c.tray_place.target_hold_s &&
                replay.tray_place.alignment_extension_s==c.tray_place.alignment_extension_s &&
                replay.tray_place.confirm_stable_s==c.tray_place.confirm_stable_s &&
                replay.tray_place.tracking_max_neighbors==c.tray_place.tracking_max_neighbors &&
                replay.tray_place.tracking_scan_limit==c.tray_place.tracking_scan_limit,
                "fast tracking or static-goal budget lost in snapshot");
        auto invalid_hold=c;invalid_hold.tray_place.target_hold_s=.1;
        require(!wrist_servo::validate_config(invalid_hold,error),"target lifetime shorter than image freshness was accepted");
        auto invalid_affine=c;invalid_affine.tray_place.affine_min_scale=1.1;
        require(replay.tray_place.global_acquisition && replay.tray_place.global_search_m==c.tray_place.global_search_m &&
                replay.tray_place.global_max_hypotheses==c.tray_place.global_max_hypotheses &&
                replay.tray_place.single_neighbor_max_age_s==c.tray_place.single_neighbor_max_age_s &&
                replay.tray_place.observe_recovery_lift_m==c.tray_place.observe_recovery_lift_m &&
                replay.tray_place.observation_age_s==c.tray_place.observation_age_s &&
                replay.tray_place.save_debug_images==c.tray_place.save_debug_images,
                "acquisition/recovery parameters lost in snapshot");
        auto invalid_recovery=c;invalid_recovery.tray_place.observe_recovery_lift_m=.20;
        require(!wrist_servo::validate_config(invalid_recovery,error),"unbounded recovery lift accepted");
        require(!wrist_servo::validate_config(invalid_affine,error),"invalid affine scale accepted");
        auto invalid_observe=c;invalid_observe.tray_place.observe_timeout_s=c.task_timeout_s;
        require(!wrist_servo::validate_config(invalid_observe,error),"unbounded B wait accepted");
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
