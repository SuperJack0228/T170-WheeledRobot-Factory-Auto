#include "t170/config/extrinsics.h"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unistd.h>

void check(bool ok,const std::string &message){if(!ok)throw std::runtime_error(message);}
int main(int argc,char **argv) {
 try {
    check(argc==2,"resolved runtime required");
    std::string error;std::array<double,16> matrix{};
    // Exercise real materializer output, including the converted V2 flange frame.
    for(auto name:{"left_hand_to_base_result.yaml","right_hand_to_base_result.yaml","camera_to_base_result.yaml"}) {
        auto path=std::filesystem::path(argv[1])/"config"/name;
        check(load_cam2robot_matrix(path.string(),matrix,error),error);
        const auto root=YAML::LoadFile(path.string());const auto node=root["a. matrix"]?root["a. matrix"]:root["matrix"];
        for(int r=0;r<4;++r)for(int c=0;c<4;++c) {
            const double expected=node[r][c].as<double>()/(c==3 && r<3?1000.:1.);
            check(std::abs(matrix[r*4+c]-expected)<1e-12,"converted runtime matrix or mm/m contract changed");
        }
        const auto before=matrix;
        check(load_cam2robot_matrix(path.string(),matrix,error) && matrix==before,"repeated read converted twice");
    }
    const auto folder=std::filesystem::temp_directory_path()/("t170-extrinsics-"+std::to_string(getpid()));
    std::filesystem::create_directories(folder);const auto file=folder/"matrix.yaml";
    auto write=[&](const std::string &text){std::ofstream out(file);out<<text;};
    write("# comment before matrix\na. matrix:\n - [1, 0, 0, 125]\n - [0, 1, 0, -30]\n - [0, 0, 1, 8]\n - [0, 0, 0, 1]\n");
    check(load_cam2robot_matrix(file.string(),matrix,error),error);
    check(matrix[3]==.125 && matrix[7]==-.03 && matrix[11]==.008,"multiline/prefixed matrix conversion");
    write("matrix: [1,0,0,125, 0,1,0,-30, 0,0,1,8, 0,0,0,1]\n");
    check(load_cam2robot_matrix(file.string(),matrix,error),error);const auto valid=matrix;
    for(auto text:{"matrix: [1,2]\n","matrix: bad\n","matrix: [[1,0],[0,1]]\n",
        "matrix: [1,0,0,.nan, 0,1,0,0, 0,0,1,0, 0,0,0,1]\n",
        "matrix: [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,1,1]\n"}) {
        write(text);check(!load_cam2robot_matrix(file.string(),matrix,error),"malformed transform accepted");
        check(matrix==valid && error.find(file.string())!=std::string::npos,"failure overwrote matrix or hid file path");
    }
    check(!load_cam2robot_matrix((folder/"missing.yaml").string(),matrix,error) && matrix==valid,"missing file changed output");
    std::filesystem::remove_all(folder);
    std::cout<<"Real V2 runtime matrices, inline/multiline YAML, units, atomic failure passed\n";
 }catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
}
