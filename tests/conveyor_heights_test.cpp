#include "t170/config/conveyor_heights.h"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <cmath>
#include <iostream>
#include <stdexcept>
void check(bool ok,const char *why){if(!ok)throw std::runtime_error(why);}
int main(int argc,char **argv){try{
    using namespace t170::config;
    check(argc==2,"catalog required");const auto catalog=load_conveyor_heights(argv[1]);
    check(catalog.enabled && catalog.machines.size()==10 && catalog.reference_machine=="1.2","catalog incomplete");
    const double measured_cm[]={123.6,122.7,122.0,122.8,122.5,121.0,123.2,122.7,123.0,121.6};
    for(int i=0;i<10;++i){
        ConveyorHeightMatch match;std::string error;
        const std::string role=i%2?"conveyor2":"conveyor";
        check(match_conveyor_height(catalog,{500,550+i},role,-.29,.63,.63,match,error),error.c_str());
        check(std::abs(match.machine.qr_plane_height_m-measured_cm[i]*.01)<1e-12,"height/ID binding wrong");
        check(std::abs(match.plane_z_m-(-.29+(measured_cm[i]-123.6)*.01))<1e-12,"wrong cm conversion or height sign");
        check(match.machine.machine_id==std::to_string(i/2+1)+(i%2?".1":".2"),"machine type reversed");
        ConveyorHeightMatch raised;
        check(match_conveyor_height(catalog,{550+i},role,-.29,.63,.64,raised,error),"information-only marker rejected");
        check(std::abs(raised.plane_z_m-(match.plane_z_m-.01))<1e-12,"waist correction sign wrong");
    }
    ConveyorHeightMatch match;std::string error;
    check(!match_conveyor_height(catalog,{500},"conveyor",-.29,.63,.63,match,error),"common marker used as identity");
    check(!match_conveyor_height(catalog,{500,999},"conveyor",-.29,.63,.63,match,error),"unknown code used fallback height");
    check(!match_conveyor_height(catalog,{550,552},"conveyor",-.29,.63,.63,match,error),"ambiguous identity accepted");
    check(!match_conveyor_height(catalog,{551},"conveyor",-.29,.63,.63,match,error),"wrong machine type accepted");
    auto changed=catalog;changed.machines[0].qr_plane_height_m+=.001;
    check(same_conveyor_identity(catalog,changed),"height-only change requires engine restart");
    changed.machines[0].info_id=590;check(!same_conveyor_identity(catalog,changed),"ID change can silently reuse detector");
    auto doc=YAML::LoadFile(argv[1]);doc["machines"][1]["info_id"]=550;
    const auto invalid=std::filesystem::temp_directory_path()/"t170-invalid-height-catalog.yaml";
    {std::ofstream out(invalid);out<<doc;}
    bool rejected=false;try{load_conveyor_heights(invalid.string());}catch(const std::exception&){rejected=true;}
    std::filesystem::remove(invalid);check(rejected,"duplicate information code accepted");
    std::cout<<"10 heights, identities, waist compensation and invalid observations passed\n";return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
