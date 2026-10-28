#include "servo/visual_servo.h"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <iostream>
#include <stdexcept>

void require(bool ok,const char *message) { if(!ok) throw std::runtime_error(message); }
int main(int argc,char **argv) {
    try {
        if(argc!=2) throw std::runtime_error("usage: visual_servo_config_test config/move_box_params.yaml");
        auto root=YAML::LoadFile(argv[1]); wrist_servo::Config c;
        c.profiles[0].reference_px << root["cameras"]["left_gripper_u"].as<double>(),root["cameras"]["left_gripper_v"].as<double>();
        c.profiles[1].reference_px << root["cameras"]["right_gripper_u"].as<double>(),root["cameras"]["right_gripper_v"].as<double>();
        c.profiles[2].reference_px=c.profiles[0].reference_px;
        c.profiles[3].reference_px=c.profiles[1].reference_px;
        wrist_servo::load_config(root["visual_servo"],c);
        require(c.period_s==.01 && c.stable_frames>=2,"actual controller parameters loaded");
        require(c.profiles[0].reference_px.x()==598 && c.profiles[1].reference_px.x()==702,"per-hand profiles loaded");
        const auto snapshot=(std::filesystem::temp_directory_path()/"t170_servo_config_roundtrip.yaml").string();
        std::string error;
        require(wrist_servo::write_config_snapshot(c,snapshot,error),"snapshot written");
        wrist_servo::Config replay;
        wrist_servo::load_config(YAML::LoadFile(snapshot)["visual_servo"],replay);
        require(replay.kp==c.kp && replay.kd==c.kd && replay.commit_height_m==c.commit_height_m,"snapshot roundtrip");
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
