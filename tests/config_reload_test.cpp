#include "t170/config/access.h"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <unistd.h>

namespace {
void require(bool ok,const char *message) { if(!ok) throw std::runtime_error(message); }
}
int main(int argc,char **argv) {
    try {
        require(argc==2,"runtime path required");
        const auto directory=std::filesystem::temp_directory_path()/("t170-reload-"+std::to_string(getpid()));
        std::filesystem::create_directories(directory);
        struct Cleanup {std::filesystem::path path;~Cleanup(){std::filesystem::remove_all(path);}} cleanup{directory};
        for(const auto &entry:std::filesystem::directory_iterator(std::filesystem::path(argv[1])/"config")) {
            const auto target=directory/entry.path().filename();
            if(entry.is_regular_file())std::filesystem::copy_file(entry.path(),target,std::filesystem::copy_options::overwrite_existing);
            else if(entry.path().filename()!="calibration")std::filesystem::create_directory_symlink(std::filesystem::absolute(entry.path()),target);
        }
        std::filesystem::create_directories(directory/"calibration");
        const auto path=directory/"move_box_params.yaml";
        const auto original=YAML::LoadFile(path.string());
        auto write=[&](const YAML::Node &node){std::ofstream out(path);out<<node;};
        MoveBoxConfig current;std::string error;
        require(load_move_box_config(path.string(),current,error),error.c_str());
        t170::config::capture_startup_resources(directory.string());
        std::mutex hardware;
        bool calibration=false,stopping=false;
        auto reload=[&]{return t170::config::reload_when_idle(hardware,[&]{return calibration;},[&]{return stopping;},path.string(),current);};
        const double before=current.visual_servo.kp;
        auto changed=YAML::Clone(original);changed["visual_servo"]["kp"]=before+.2;write(changed);
        std::promise<void> occupied,release;
        auto released=release.get_future();
        auto worker=std::async(std::launch::async,[&]{std::lock_guard lock(hardware);occupied.set_value();released.wait();});
        occupied.get_future().wait();
        const auto busy=reload();release.set_value();worker.get();
        require(!busy.ok && busy.code=="busy" && current.visual_servo.kp==before,"busy action allowed config replacement");
        calibration=true;require(reload().code=="calibration_active","active calibration reload allowed");calibration=false;
        stopping=true;require(reload().code=="stopping","reload during shutdown allowed");stopping=false;
        require(reload().ok && current.visual_servo.kp==before+.2,"idle tuning did not commit");
        auto invalid=YAML::Clone(original);invalid["visual_servo"]["kp"]=-1;write(invalid);
        require(!reload().ok && current.visual_servo.kp==before+.2,"invalid config replaced accepted config");
        auto identity=YAML::Clone(original);identity["chassis"]["host"]="different-host";write(identity);
        require(reload().code=="restart_required","live device binding changed without restart");
        write(original);
        {std::ofstream out(directory/"Robot_Model.yaml",std::ios::app);out<<"\n# changed after startup\n";}
        require(reload().code=="restart_required","changed loaded SDK resource silently accepted");
        std::filesystem::copy_file(std::filesystem::path(argv[1])/"config/Robot_Model.yaml",directory/"Robot_Model.yaml",std::filesystem::copy_options::overwrite_existing);
        // A background reader holding the common mutex must see a complete old
        // object; the writer commits only after that reader has released it.
        changed["visual_servo"]["kp"]=before+.4;write(changed);
        std::unique_lock read(t170::config::access_mutex());
        std::promise<void> attempting;
        auto pending=std::async(std::launch::async,[&]{attempting.set_value();return reload();});
        attempting.get_future().wait();
        require(pending.wait_for(std::chrono::milliseconds(20))==std::future_status::timeout,"writer bypassed common reader lock");
        const auto observed=current.visual_servo.kp;read.unlock();
        require(pending.get().ok && observed==before+.2 && current.visual_servo.kp==before+.4,"reader saw partial config");
        std::cout<<"Reload: busy/calibration/shutdown exclusion, invalid rollback, restart contracts and background reader serialization passed\n";
    }catch(const std::exception &ex){std::cerr<<ex.what()<<'\n';return 1;}
}
