// Offline RGB geometry only. This executable has no robot/SDK/motion dependency.
#include "t170/vision/tray_localizer.h"
#include "t170/control/visual_servo.h"
#include "../src/vision/geometry/tray_registration.h"
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <map>
using namespace wrist_servo;

template<int N> Eigen::Matrix<double,N,N> matrix(const YAML::Node &n) {
    if(!n.IsSequence() || n.size()!=N) throw std::runtime_error("matrix dimensions");
    Eigen::Matrix<double,N,N> out;
    for(int r=0;r<N;++r) for(int c=0;c<N;++c) out(r,c)=n[r][c].as<double>();return out;
}
int main(int argc,char **argv) {
    try {
        if(argc<3) {
            std::cerr<<"Usage: tray_localizer_replay config.yaml frame.jpg.yaml [more metadata...]\n"
                       "       tray_localizer_replay --projection image.jpg projection.yaml\n"
                       "Projection mode probes discovery/association only; it is not a full camera-state replay.\n";
            return 2;
        }
        if(std::string(argv[1])=="--projection") {
            if(argc!=4) return 2;
            const auto root=YAML::LoadFile(argv[3]);TrayTarget target;target.valid=true;
            target.row=root["target_row"].as<int>();target.col=root["target_col"].as<int>();
            const auto frame=cv::imread(argv[2]);if(frame.empty()) throw std::runtime_error("missing image");
            TrayPlaceVisionConfig cfg;
            const auto start=std::chrono::steady_clock::now();
            const auto scene=tray_registration::discover(frame,matrix<3>(root["seed_projection"]),target.board,cfg);
            const auto found=tray_registration::acquire(scene,target,cfg);
            std::cout<<"mode=projection_probe valid="<<found.valid<<" reason="<<found.reason<<" candidates="<<scene.rims.size()
                     <<" "<<found.detail<<" elapsed_ms="<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()<<'\n';
            if(found.valid) std::cout<<"correction=\n"<<found.fit.transform<<'\n';
            return found.valid?0:1;
        }
        Config config;const auto root=YAML::LoadFile(argv[1]);load_config(root["visual_servo"],config);
        std::string error;if(!validate_config(config,error)) throw std::runtime_error(error);
        std::map<std::pair<int,std::uint64_t>,std::unique_ptr<TrayLocalizer>> localizers;
        for(int i=2;i<argc;++i) {
            const std::filesystem::path path(argv[i]);const auto n=YAML::LoadFile(path.string());
            TrayTarget target;target.valid=true;target.row=n["target_row"].as<int>();target.col=n["target_col"].as<int>();
            target.board.rows=n["rows"].as<int>();target.board.cols=n["cols"].as<int>();
            target.board.pitch_m=n["pitch_m"].as<double>();target.board.radius_m=n["radius_m"].as<double>();
            target.board.offset_x_m=n["offset_x_m"].as<double>(0);target.board.offset_y_m=n["offset_y_m"].as<double>(0);
            target.base_from_tray=matrix<4>(n["base_from_tray"]);
            for(const auto &v:n["occupied_holes"]) target.occupied_holes.push_back(v.as<int>());
            auto &localizer=localizers[{n["hand"].as<int>(),n["session"].as<std::uint64_t>()}];
            if(!localizer) {localizer=std::make_unique<TrayLocalizer>();localizer->reset(target,config.tray_place);}
            auto image_path=path;image_path.replace_extension();
            const cv::Mat image=cv::imread(image_path.string()+".raw.png");
            if(image.empty() || !image.isContinuous()) throw std::runtime_error("missing raw PNG: "+image_path.string());
            CameraFrameData frame;frame.ok=true;frame.width=image.cols;frame.height=image.rows;
            frame.frame_number=n["frame"].as<std::uint64_t>();frame.capture_steady_s=n["capture_s"].as<double>();
            for(int j=0;j<9;++j) frame.K[j]=n["K"][j].as<double>();
            for(int j=0;j<5;++j) frame.dist[j]=n["dist"][j].as<double>();
            frame.color_bgr.assign(image.data,image.data+image.total()*image.elemSize());
            const auto start=std::chrono::steady_clock::now();
            const auto camera=matrix<4>(n["base_from_camera"]);
            TrayPixelGate gate;
            Config frame_config=config;
            if(n["profile"]) {YAML::Node patch;patch["left_place"]=n["profile"];load_config(patch,frame_config);}
            State capture;Eigen::Vector2d nominal=Eigen::Vector2d::Zero();double final_z=0;
            auto profile=frame_config.profiles[2];
            if(n["base_from_tcp"] && n["profile"] && n["nominal_xy"]) {
                const auto tcp=matrix<4>(n["base_from_tcp"]);capture.position=tcp.block<3,1>(0,3);capture.rotation=tcp.block<3,3>(0,0);
                nominal<<n["nominal_xy"][0].as<double>(),n["nominal_xy"][1].as<double>();final_z=n["final_z"].as<double>();
                const auto xy=target.board.hole(target.row,target.col);
                profile.feature_z_offset_m=(target.base_from_tray*Eigen::Vector4d(xy.x(),xy.y(),0,1)).z()-final_z;
                gate=[&](const Eigen::Vector2d &p) {
                    PixelObservation o;o.pixel=p;o.fx=frame.K[0];o.fy=frame.K[4];o.cx=frame.K[2];o.cy=frame.K[5];o.width=frame.width;o.height=frame.height;
                    const auto m=map_observation(profile,o,capture,final_z,camera.block<3,1>(0,3),camera.block<3,3>(0,0));
                    return m.valid && (capture.position.head<2>()+m.remaining_m-nominal).norm()<=config.max_correction_m;
                };
            }
            const auto located=localizer->locate(frame,camera,gate);
            std::cout<<"frame="<<frame.frame_number<<" valid="<<located.valid<<" stage="<<located.stage
                     <<" reason="<<located.reason<<" n="<<located.neighbors<<" uv="<<located.pixel.transpose()
                     <<" elapsed_ms="<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()
                     <<" "<<located.detail<<'\n';
        }
    } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 2;}
}
