#include "t170/common/paths.h"
#include "t170/common/logging.h"
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <regex>
#include <stdexcept>

namespace {
void require(bool ok,const char *why) { if(!ok) throw std::runtime_error(why); }
struct Sandbox {
    std::filesystem::path cwd=std::filesystem::current_path();
    std::filesystem::path root=std::filesystem::temp_directory_path()/
        ("t170 common paths "+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::map<std::string,std::optional<std::string>> environment;
    Sandbox() {
        for(const char *name:{"T170_ROOT","MARKET_SIMPLE_ROOT","T170_CONFIG_DIR","T170_LOG_DIR"}) {
            if(const char *value=std::getenv(name)) environment[name]=value;
            else environment[name]=std::nullopt;
            unsetenv(name);
        }
        std::filesystem::create_directories(root/"config");
        std::filesystem::create_directories(root/"build_robot/nested");
        std::ofstream(root/"config/realsense_cameras.yaml")<<"test: true\n";
    }
    ~Sandbox() {
        std::filesystem::current_path(cwd);
        for(const auto &[name,value]:environment) {
            if(value) setenv(name.c_str(),value->c_str(),1); else unsetenv(name.c_str());
        }
        std::error_code ec;std::filesystem::remove_all(root,ec);
    }
};
}
int main() {
    try {
        Sandbox box;
        std::filesystem::current_path(box.root/"build_robot/nested");
        const auto root=std::filesystem::current_path().parent_path().parent_path();
        require(t170::paths::resource_root()==root,"nested working-directory discovery failed");
        require(t170::paths::find_resource_root(root/"build_robot")==root,"ancestor discovery failed");
        require(default_cameras_yaml_path()==(root/"config/realsense_cameras.yaml").string(),"default config moved");
        require(t170::logging::file("visual_servo/a.csv")==root/"logs/visual_servo/a.csv","default logs moved");
        setenv("MARKET_SIMPLE_ROOT",(root/"legacy").c_str(),1);
        require(t170::paths::resource_root()==root/"legacy","legacy root compatibility failed");
        setenv("T170_ROOT",root.c_str(),1);
        require(t170::paths::resource_root()==root,"new root did not override legacy root");
        setenv("T170_CONFIG_DIR","machine config",1);setenv("T170_LOG_DIR","run logs",1);
        require(default_left_hand_to_robot_yaml_path()==(root/"machine config/left_hand_to_base_result.yaml").string(),
                "calibration/config override ignored");
        require(t170::logging::file("timing.jsonl")==root/"run logs/timing.jsonl","relative log override ignored");
        require(default_pose_yaml_path()==(root/"feeding_cylindrical_parts_alg/config/pose_params.yaml").string(),
                "config override incorrectly moved algorithm resources");
        const auto external=root/"external";
        setenv("T170_CONFIG_DIR",external.c_str(),1);setenv("T170_LOG_DIR",external.c_str(),1);
        require(t170::paths::config_directory()==external && t170::paths::log_directory()==external,
                "absolute directory overrides ignored");
        setenv("T170_ROOT","",1);setenv("MARKET_SIMPLE_ROOT","",1);
        setenv("T170_CONFIG_DIR","",1);setenv("T170_LOG_DIR","",1);
        require(project_root_dir()==root.string(),"empty override changed fallback");
        require(std::regex_match(t170::logging::local_time("%Y%m%d_%H%M%S"),std::regex("[0-9]{8}_[0-9]{6}")),
                "log filename timestamp changed");
        std::cout<<"common paths: root precedence, old entry points, external config/logs and defaults passed\n";
    } catch(const std::exception &ex) {std::cerr<<ex.what()<<'\n';return 1;}
}
