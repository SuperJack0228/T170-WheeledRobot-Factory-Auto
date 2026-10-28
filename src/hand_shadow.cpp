#include "hand_shadow.h"
#include "servo/runtime.h"
#include "Ti5_socketcan.h"
#include "move_box_config.h"
#include "seg_pose_bridge.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <mutex>
#include <thread>

namespace {
struct Watch {
    bool active=false, locked=false;
    std::uint64_t generation=0, session=0, last_frame=0;
    Eigen::Vector2d pixel=Eigen::Vector2d::Zero();
};
Watch watches[2];
std::mutex watch_mutex, start_mutex, legacy_log_mutex;
std::condition_variable wake;
std::atomic<SegPoseBridge*> engine{nullptr};
std::atomic<bool> running{false}, save_raw{false};
std::thread worker;

void rotate_left(CameraFrameData &f,int hand) {
    if(hand!=0) return;
    cv::Mat view(f.height,f.width,CV_8UC3,f.color_bgr.data());
    cv::rotate(view,view,cv::ROTATE_180);
    f.K[2]=f.width-1-f.K[2]; f.K[5]=f.height-1-f.K[5];
    f.dist[2]=-f.dist[2]; f.dist[3]=-f.dist[3];
}

void observe(int hand) {
    wrist_servo::ObservationContext context;
    if(!wrist_servo::observation_context(hand,context) || !context.vision_required || !context.descending)
        return;
    std::uint64_t generation;
    { std::lock_guard<std::mutex> lock(watch_mutex);
      if(!watches[hand].active) return;
      generation=watches[hand].generation;
      if(watches[hand].session!=context.session) {
          watches[hand].session=context.session; watches[hand].locked=false; watches[hand].last_frame=0;
      }
    }
    auto *cams=RealSenseMultiCam::live(); auto *bridge=engine.load();
    if(!cams || !bridge) return;
    if((context.latest.position.head<2>()-context.nominal_xy).norm()>context.config.max_correction_m)
        return; // Do not lock an unrelated workpiece while traversing above the tray.
    const CameraSlot camera=hand==1?CameraSlot::RightHand:CameraSlot::LeftHand;
    const double before=wrist_servo::monotonic_seconds();
    // A single round-robin worker grabs immediately before inference: no stale frame waits on a per-hand lock.
    CameraFrameData frame=cams->grab(camera,100);
    wrist_servo::PixelObservation obs;
    obs.session=context.session; obs.frame=frame.frame_number;
    obs.capture_s=frame.capture_steady_s; obs.clock_uncertainty_s=frame.timestamp_uncertainty_s;
    obs.device_timestamp_ms=frame.device_timestamp_ms; obs.timestamp_source=frame.timestamp_source;
    obs.grab_ms=(wrist_servo::monotonic_seconds()-before)*1000;
    if(!frame.ok) { wrist_servo::observation_event(hand,context.session,"grab_failed",obs); return; }
    rotate_left(frame,hand);
    obs.fx=frame.K[0]; obs.fy=frame.K[4]; obs.cx=frame.K[2]; obs.cy=frame.K[5];
    obs.width=frame.width; obs.height=frame.height;
    const auto &cfg=context.config;
    if(frame.width!=context.image_width || frame.height!=context.image_height) {
        wrist_servo::observation_event(hand,context.session,"image_size_profile_mismatch",obs); return;
    }
    { std::lock_guard<std::mutex> lock(watch_mutex);
      if(watches[hand].generation!=generation || obs.frame<=watches[hand].last_frame) return;
      watches[hand].last_frame=obs.frame;
    }
    PoseRunResult result;
    const double infer_start=wrist_servo::monotonic_seconds();
    try { result=bridge->run_yolo(frame,algorithm_id_for_slot(camera),false,camera,-1,SegEngineId::Hand); }
    catch(const std::exception &ex) { result.ok=false; result.message=ex.what(); }
    obs.completed_s=wrist_servo::monotonic_seconds();
    obs.infer_ms=(obs.completed_s-infer_start)*1000;
    if(!result.ok) { wrist_servo::observation_event(hand,context.session,"inference_failed",obs); return; }
    if(obs.completed_s-obs.capture_s+obs.clock_uncertainty_s>cfg.max_observation_age_s) {
        wrist_servo::observation_event(hand,context.session,"frame_expired_before_association",obs); return;
    }
    // Reference prediction is only for association. The motion owner maps the observation again
    // using interpolated MEASURED state at the actual capture time.
    obs.pixel=context.profile.reference_px;
    const auto model=wrist_servo::map_observation(context.profile,obs,context.latest,context.final_z,
                                                context.camera_origin,context.camera_rotation);
    if(!model.valid) { wrist_servo::observation_event(hand,context.session,model.reason.c_str(),obs); return; }
    const PoseTargetResult *best=nullptr;
    double best_distance=std::numeric_limits<double>::infinity();
    {
        std::lock_guard<std::mutex> lock(watch_mutex);
        Watch &w=watches[hand];
        if(!w.active || w.generation!=generation || w.session!=context.session) return;
        const Eigen::Vector2d expected=model.reference_px+
            model.jacobian*(context.latest.position.head<2>()-context.nominal_xy);
        const Eigen::Vector2d anchor=w.locked?w.pixel:expected;
        const double radius=w.locked?cfg.track_radius_px:cfg.acquire_radius_px;
        for(const auto &target:result.targets) {
            if(target.class_id!=context.target_class || target.confidence<cfg.min_confidence ||
               target.box_u<0 || target.box_v<0) continue;
            const Eigen::Vector2d pixel(target.box_u,target.box_v);
            const double distance=(pixel-anchor).norm();
            if(distance<=radius && distance<best_distance) { best=&target; best_distance=distance; }
        }
        if(best) {
            obs.pixel << best->box_u,best->box_v; obs.confidence=best->confidence;
            w.pixel=obs.pixel; w.locked=true;
        }
    }
    if(!best) { wrist_servo::observation_event(hand,context.session,"locked_target_missing",obs); return; }
    if(!wrist_servo::submit_observation(hand,obs))
        wrist_servo::observation_event(hand,context.session,"session_not_accepting_submit",obs);
    if(save_raw.load() && context.descending) {
        const auto directory=std::filesystem::path(project_root_dir())/"hand_descent_raw";
        std::error_code ec; std::filesystem::create_directories(directory,ec);
        if(!ec) {
            const auto name="session_"+std::to_string(context.session)+"_hand_"+std::to_string(hand)+
                            "_frame_"+std::to_string(obs.frame)+".jpg";
            cv::imwrite((directory/name).string(),cv::Mat(frame.height,frame.width,CV_8UC3,frame.color_bgr.data()));
        }
    }
}

void loop() {
    while(running.load()) {
        bool any=false;
        // Fair single-engine scheduling. Each hand receives the next available latest frame.
        for(int hand=0;hand<2 && running.load();++hand) {
            bool active;
            { std::lock_guard<std::mutex> lock(watch_mutex); active=watches[hand].active; }
            if(!active || hardware_abort_requested()) continue;
            any=true;
            try { observe(hand); }
            catch(const std::exception &ex) {
                wrist_servo::ObservationContext context;
                if(wrist_servo::observation_context(hand,context))
                    wrist_servo::observation_event(hand,context.session,"observer_exception");
                std::cerr << "[servo] 观察线程异常 hand=" << hand << ' ' << ex.what() << '\n';
            }
        }
        // Also avoids a busy loop while the motion owner initializes the session.
        std::unique_lock<std::mutex> lock(watch_mutex);
        wake.wait_for(lock,std::chrono::milliseconds(any?2:20));
    }
}
}

void hand_shadow_bind(SegPoseBridge *bridge) {
    std::lock_guard<std::mutex> lock(start_mutex);
    if(!bridge) {
        running.store(false); wake.notify_all();
        if(worker.joinable()) worker.join();
        engine.store(nullptr); return;
    }
    engine.store(bridge);
}
void hand_shadow_ensure(int hand,Robot_Arm *arm,const Eigen::Matrix<double,1,6> &goal,
                        const char *tag,int cls) {
    if(hand<0 || hand>1 || !arm || !engine.load()) return;
    { std::lock_guard<std::mutex> lock(start_mutex);
      if(!worker.joinable()) { running.store(true); worker=std::thread(loop); }
    }
    hand_shadow_begin(hand,arm,goal,tag,cls);
}
void hand_shadow_begin(int hand,Robot_Arm *,const Eigen::Matrix<double,1,6> &,const char *,int) {
    if(hand<0 || hand>1) return;
    { std::lock_guard<std::mutex> lock(watch_mutex);
      const auto generation=watches[hand].generation+1;
      watches[hand]={}; watches[hand].generation=generation; watches[hand].active=true;
    }
    wake.notify_all();
}
void hand_shadow_end(int hand) {
    if(hand<0 || hand>1) return;
    std::lock_guard<std::mutex> lock(watch_mutex);
    watches[hand].active=false; watches[hand].locked=false; ++watches[hand].generation;
}
void hand_shadow_set_descent_raw(bool enable) { save_raw.store(enable); }
void hand_track_log(const std::string &hand,const std::string &result,const std::string &detail) {
    std::lock_guard<std::mutex> lock(legacy_log_mutex);
    const auto path=std::filesystem::path(project_root_dir())/"logs"/"hand_track.csv";
    std::error_code ec; std::filesystem::create_directories(path.parent_path(),ec);
    std::ofstream out(path,std::ios::app);
    out << wrist_servo::monotonic_seconds() << ',' << hand << ',' << result << ',' << detail << '\n';
}
