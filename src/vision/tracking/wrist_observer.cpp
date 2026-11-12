#include "t170/common/paths.h"
#include "t170/common/logging.h"
#include "t170/vision/wrist_observer.h"
#include "t170/vision/servo_observer_port.h"
#include "t170/vision/tray_localizer.h"
#include "t170/vision/belt_circle.h"
#include "t170/vision/debug.h"
#include "t170/common/abort_query.h"
#include "t170/vision/python_bridge.h"
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
#include <optional>
#include <sstream>
#include <iomanip>

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
wrist_servo::TrayLocalizer tray_localizers[2];
std::uint64_t tray_sessions[2]{}; // exclusively owned by the observation worker

// At most one pending frame per hand. Disk latency never blocks the observer.
class DebugImageWriter {
    struct Item {std::filesystem::path path;cv::Mat overlay,raw;std::string metadata;};
    std::mutex mutex_;std::condition_variable wake_;bool stop_=false;
    std::array<std::optional<Item>,2> pending_;
    std::thread worker_;
public:
    DebugImageWriter():worker_([this] {
        for(;;) {
            std::optional<Item> item;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                wake_.wait(lock,[&]{return stop_ || pending_[0] || pending_[1];});
                if(stop_ && !pending_[0] && !pending_[1]) return;
                static int next=0;
                for(int i=0;i<2;++i) {int h=(next+i)%2;if(pending_[h]) {item=std::move(pending_[h]);pending_[h].reset();next=1-h;break;}}
            }
            try {
                std::error_code ec;std::filesystem::create_directories(item->path.parent_path(),ec);
                if(ec) {std::cerr<<"[tray_debug] directory: "<<ec.message()<<'\n';continue;}
                const auto slot=item->path.parent_path().filename();
                const auto raw_path=item->path.parent_path().parent_path()/"original"/slot/
                    (item->path.stem().string()+".png");
                std::filesystem::create_directories(raw_path.parent_path(),ec);
                if(!cv::imwrite(item->path.string(),item->overlay)) std::cerr<<"[vision_debug] overlay write failed\n";
                if(!item->raw.empty() && !cv::imwrite(raw_path.string(),item->raw)) std::cerr<<"[vision_debug] raw write failed\n";
                std::ofstream meta(item->path.string()+".yaml");meta<<item->metadata;
                if(!meta) std::cerr<<"[tray_debug] metadata write failed\n";
            } catch(const std::exception &ex) {std::cerr<<"[tray_debug] "<<ex.what()<<'\n';}
        }
    }) {}
    ~DebugImageWriter() {{std::lock_guard<std::mutex> lock(mutex_);stop_=true;}wake_.notify_all();worker_.join();}
    void put(int hand,const std::filesystem::path &path,cv::Mat overlay,cv::Mat raw,std::string metadata) {
        {std::lock_guard<std::mutex> lock(mutex_);pending_[hand]=Item{path,std::move(overlay),std::move(raw),std::move(metadata)};}
        wake_.notify_one();
    }
};

void save_place_overlay(int hand,std::uint64_t session,std::uint64_t frame_id,
                        const CameraFrameData &frame,const wrist_servo::TrayLocalization &located,
                        const wrist_servo::ObservationContext &context,const wrist_servo::State &capture,
                        const Eigen::Matrix4d &base_camera) {
    const auto &target=context.tray_target;
    const int target_row=target.row,target_col=target.col;
    if(!frame.ok || frame.width<1 || frame.height<1 ||
       frame.color_bgr.size()!=static_cast<std::size_t>(frame.width)*frame.height*3) return;
    const char *slot=hand==1?"right":"left";
    const auto path=std::filesystem::path(vision_debug_record_path(slot,"servo"));
    cv::Mat canvas(frame.height,frame.width,CV_8UC3,
                   const_cast<unsigned char*>(frame.color_bgr.data()));
    cv::Mat raw=canvas.clone();
    canvas=canvas.clone();
    for(const auto &p:located.candidate_pixels)
        if(p.allFinite()) cv::circle(canvas,cv::Point(cvRound(p.x()),cvRound(p.y())),6,cv::Scalar(255,0,255),2,cv::LINE_AA);
    auto inside=[&](float u,float v) {
        return u>=0 && v>=0 && u<frame.width && v<frame.height;
    };
    for(const auto &mark:located.marks) {
        if(!inside(mark.predicted_u,mark.predicted_v)) continue;
        const cv::Point predicted(static_cast<int>(mark.predicted_u),static_cast<int>(mark.predicted_v));
        const cv::Scalar color=mark.occupied?cv::Scalar(0,140,255):cv::Scalar(255,255,0);
        cv::circle(canvas,predicted,10,color,1,cv::LINE_AA);
        if(mark.accepted && inside(mark.observed_u,mark.observed_v)) {
            const cv::Point observed(static_cast<int>(mark.observed_u),static_cast<int>(mark.observed_v));
            cv::line(canvas,predicted,observed,cv::Scalar(0,255,0),1,cv::LINE_AA);
            cv::circle(canvas,observed,5,cv::Scalar(0,255,0),-1,cv::LINE_AA);
        }
        cv::putText(canvas,"r"+std::to_string(mark.row)+"c"+std::to_string(mark.col),
                    predicted+cv::Point(8,-8),cv::FONT_HERSHEY_SIMPLEX,0.4,color,1,cv::LINE_AA);
    }
    if(inside(located.target_u,located.target_v)) {
        const cv::Point target(static_cast<int>(located.target_u),static_cast<int>(located.target_v));
        cv::drawMarker(canvas,target,cv::Scalar(0,0,255),cv::MARKER_CROSS,22,2,cv::LINE_AA);
        cv::putText(canvas,"target r"+std::to_string(target_row)+"c"+std::to_string(target_col),
                    target+cv::Point(12,16),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255),1,cv::LINE_AA);
    }
    if(located.valid && inside(static_cast<float>(located.pixel.x()),static_cast<float>(located.pixel.y()))) {
        const cv::Point locked(static_cast<int>(located.pixel.x()),static_cast<int>(located.pixel.y()));
        cv::circle(canvas,locked,8,cv::Scalar(0,255,255),2,cv::LINE_AA);
    }
    const std::string headline=std::string(hand==1?"right ":"left ")+located.stage+" "+located.reason;
    const std::string counts="neighbors "+std::to_string(located.neighbors)+"/"+std::to_string(located.rois)+
        " reject_size "+std::to_string(located.rejected_size)+
        " shape "+std::to_string(located.rejected_shape)+
        " residual "+std::to_string(located.rejected_residual)+
        " occupied "+std::to_string(located.skipped_occupied);
    cv::rectangle(canvas,cv::Point(0,0),cv::Point(canvas.cols,46),cv::Scalar(0,0,0),cv::FILLED);
    cv::putText(canvas,headline,cv::Point(8,18),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,255,255),1,cv::LINE_AA);
    cv::putText(canvas,counts,cv::Point(8,38),cv::FONT_HERSHEY_SIMPLEX,0.45,cv::Scalar(255,255,255),1,cv::LINE_AA);
    std::ostringstream metadata;metadata<<std::setprecision(17);
    metadata<<"hand: "<<hand<<"\nsession: "<<session<<"\nwidth: "<<frame.width<<"\nheight: "<<frame.height<<"\nframe: "<<frame_id
        <<"\ncapture_s: "<<frame.capture_steady_s<<"\ntarget_row: "<<target_row<<"\ntarget_col: "<<target_col
        <<"\nrows: "<<target.board.rows<<"\ncols: "<<target.board.cols<<"\npitch_m: "<<target.board.pitch_m
        <<"\nradius_m: "<<target.board.radius_m<<"\noffset_x_m: "<<target.board.offset_x_m<<"\noffset_y_m: "<<target.board.offset_y_m<<"\nK: [";
    for(int i=0;i<9;++i) metadata<<(i?", ":"")<<frame.K[i];metadata<<"]\ndist: [";
    for(int i=0;i<5;++i) metadata<<(i?", ":"")<<frame.dist[i];metadata<<"]\noccupied_holes: [";
    for(std::size_t i=0;i<target.occupied_holes.size();++i) metadata<<(i?", ":"")<<target.occupied_holes[i];metadata<<"]\n";
    auto matrix=[&](const char *name,const Eigen::Matrix4d &a) {
        metadata<<name<<":\n";
        for(int r=0;r<4;++r) {metadata<<"  - [";for(int c=0;c<4;++c) metadata<<(c?", ":"")<<a(r,c);metadata<<"]\n";}
    };
    matrix("base_from_tray",target.base_from_tray);matrix("base_from_camera",base_camera);
    Eigen::Matrix4d tcp=Eigen::Matrix4d::Identity();tcp.block<3,3>(0,0)=capture.rotation;tcp.block<3,1>(0,3)=capture.position;
    matrix("base_from_tcp",tcp);
    metadata<<"final_z: "<<context.final_z<<"\nnominal_xy: ["<<context.nominal_xy.x()<<", "<<context.nominal_xy.y()<<"]\n";
    const auto &profile=context.profile;
    metadata<<"profile:\n  mode: "<<profile.mode<<"\n  reference_px: ["<<profile.reference_px.x()<<", "<<profile.reference_px.y()
        <<"]\n  reference_height_m: "<<profile.reference_height_m<<"\n  feature_z_offset_m: "<<profile.feature_z_offset_m
        <<"\n  min_depth_m: "<<profile.min_depth_m<<"\n  max_depth_m: "<<profile.max_depth_m<<"\n";
    if(!profile.knots.empty()) {
        metadata<<"  knots:\n";
        for(const auto &k:profile.knots) metadata<<"    - height_m: "<<k.height_m<<"\n      reference_px: ["<<k.reference_px.x()<<", "<<k.reference_px.y()
            <<"]\n      jacobian_px_per_m: [["<<k.image_jacobian(0,0)<<", "<<k.image_jacobian(0,1)<<"], ["
            <<k.image_jacobian(1,0)<<", "<<k.image_jacobian(1,1)<<"]]\n";
    }
    wrist_servo::PixelObservation probe;
    probe.pixel=context.profile.reference_px;
    probe.fx=frame.K[0]; probe.fy=frame.K[4]; probe.cx=frame.K[2]; probe.cy=frame.K[5];
    probe.width=frame.width; probe.height=frame.height;
    auto display_profile=context.profile;
    const auto hole_xy=context.tray_target.board.hole(target_row,target_col);
    display_profile.feature_z_offset_m=(context.tray_target.base_from_tray*
        Eigen::Vector4d(hole_xy.x(),hole_xy.y(),0,1)).z()-context.final_z;
    const auto mapped=wrist_servo::map_observation(
        display_profile,probe,capture,context.final_z,
        base_camera.block<3,1>(0,3),base_camera.block<3,3>(0,0));
    const Eigen::Vector2d ref=mapped.valid?mapped.reference_px:context.profile.reference_px;
    if(std::isfinite(ref.x()) && std::isfinite(ref.y()) && inside(static_cast<float>(ref.x()),static_cast<float>(ref.y()))) {
        const cv::Point mark(cvRound(ref.x()),cvRound(ref.y()));
        cv::drawMarker(canvas,mark,cv::Scalar(255,0,0),cv::MARKER_CROSS,36,2,cv::LINE_AA);
        cv::circle(canvas,mark,16,cv::Scalar(255,0,0),2,cv::LINE_AA);
        cv::putText(canvas,"ref "+std::to_string(cvRound(ref.x()))+","+std::to_string(cvRound(ref.y())),
                    mark+cv::Point(12,-12),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,0,0),1,cv::LINE_AA);
    }
    static DebugImageWriter writer;writer.put(hand,path,std::move(canvas),std::move(raw),metadata.str());
}

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
    // No inference during traversal or before the fixed grasp orientation is established.
    if(Eigen::AngleAxisd(context.desired_rotation*context.latest.rotation.transpose()).angle()>
       context.config.max_orientation_error_rad) return;
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
    if(context.target_class==1 && cfg.tray_place.enabled && context.tray_target.valid) {
        Eigen::Matrix4d base_camera;
        wrist_servo::State capture;
        bool have_capture=wrist_servo::camera_at_capture(hand,context.session,obs.capture_s,base_camera,&capture);
        // A just-captured frame may precede the next 10ms feedback sample. Wait
        // only in this observer, never extrapolate a commanded camera pose.
        for(int retry=0;retry<6 && !have_capture;++retry) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            have_capture=wrist_servo::camera_at_capture(hand,context.session,obs.capture_s,base_camera,&capture);
        }
        if(!have_capture) {
            wrist_servo::observation_event(hand,context.session,"tray_capture_state_unavailable",obs);return;
        }
        obs.clock_uncertainty_s+=capture.sampling_uncertainty_s;obs.capture_timing_accounted=true;
        if(tray_sessions[hand]!=context.session) {
            tray_localizers[hand].reset(context.tray_target,cfg.tray_place);
            tray_sessions[hand]=context.session;
        }
        const auto &target=context.tray_target;
        const auto xy=target.board.hole(target.row,target.col);
        const double plane_z=(target.base_from_tray*Eigen::Vector4d(xy.x(),xy.y(),0,1)).z();
        auto profile=context.profile;profile.feature_z_offset_m=plane_z-context.final_z;
        std::string gate_reason;
        const auto executable=[&](const Eigen::Vector2d &pixel) {
            auto candidate=obs;candidate.pixel=pixel;
            const auto map=wrist_servo::map_observation(profile,candidate,capture,context.final_z,
                base_camera.block<3,1>(0,3),base_camera.block<3,3>(0,0));
            const bool valid=map.valid && (capture.position.head<2>()+map.remaining_m-context.nominal_xy).norm()<=cfg.max_correction_m;
            if(!valid) gate_reason=map.valid?"outside_correction_envelope":map.reason;
            return valid;
        };
        auto located=tray_localizers[hand].locate(frame,base_camera,executable);
        if(!located.valid && !gate_reason.empty()) located.detail+=" task_hint="+gate_reason;
        obs.completed_s=wrist_servo::monotonic_seconds(); obs.infer_ms=(obs.completed_s-infer_start)*1000;
        if(located.reason!="tray_acquire_wait_next_frame") {
            const bool insufficient=located.reason=="tray_acquire_no_rims" ||
                located.reason=="tray_acquire_insufficient_rims" || located.reason=="tray_insufficient_visible_neighbors";
            wrist_servo::report_tray_view(hand,context.session,obs.completed_s,insufficient);
        }
        obs.virtual_target=true; obs.target_row=context.tray_target.row; obs.target_col=context.tray_target.col;
        obs.neighbor_count=located.neighbors; obs.marker_corner_count=located.marker_corners;
        obs.vision_source=located.source; obs.plane_reprojection_px=located.reprojection_px;
        obs.plane_z_m=located.plane_z_m;
        obs.tray_rois=located.rois; obs.tray_contours=located.contours;
        obs.tray_rejected_size=located.rejected_size; obs.tray_rejected_shape=located.rejected_shape;
        obs.tray_rejected_coverage=located.rejected_coverage; obs.tray_rejected_residual=located.rejected_residual;
        obs.tray_skipped_occupied=located.skipped_occupied;
        obs.tray_candidates=located.candidate_pixels.size();obs.tray_stage=located.stage;
        obs.tray_rejected_distance=located.rejected_distance;obs.tray_rejected_target=located.rejected_target;
        obs.tray_rejected_short=located.rejected_short;obs.tray_global_hypotheses=located.global_hypotheses;
        obs.tray_occupied_used=located.occupied_used;obs.tray_partial_used=located.partial_used;
        obs.tray_identity_margin=located.identity_margin;
        obs.tray_undistort_ms=located.undistort_ms;obs.tray_acquisition_ms=located.acquisition_ms;
        obs.tray_prepare_ms=located.prepare_ms;obs.tray_roi_ms=located.roi_ms;
        obs.tray_partial_ms=located.partial_ms;obs.tray_fit_ms=located.fit_ms;
        obs.tray_partial_calls=located.partial_calls;obs.tray_pruned_neighbors=located.pruned_neighbors;
        auto save_debug=[&] {
            static double last_image_s[2]{};
            if(cfg.tray_place.save_debug_images && located.reason!="tray_acquire_wait_next_frame" &&
               obs.completed_s-last_image_s[hand]>=cfg.tray_place.debug_image_period_s) {
                save_place_overlay(hand,context.session,obs.frame,frame,located,context,capture,base_camera);
                last_image_s[hand]=obs.completed_s;
            }
        };
        if(!located.valid) {
            const std::string reason=located.reason+" "+located.detail;
            wrist_servo::observation_event(hand,context.session,reason.c_str(),obs);save_debug();return;
        }
        obs.submit_age_ms=(wrist_servo::monotonic_seconds()-obs.capture_s+obs.clock_uncertainty_s)*1000;
        if(obs.submit_age_ms>cfg.tray_place.observation_age_s*1000) {
            wrist_servo::observation_event(hand,context.session,"tray_geometry_frame_expired",obs);save_debug();return;
        }
        obs.pixel=located.pixel;
        obs.confidence=.98/(1+located.reprojection_px/cfg.tray_place.reprojection_gate_px);
        wrist_servo::observation_event(hand,context.session,("tray_neighbor_model "+located.detail).c_str(),obs);
        if(!wrist_servo::submit_observation(hand,obs))
            wrist_servo::observation_event(hand,context.session,"session_not_accepting_submit",obs);
        save_debug(); // Valid observations reach the controller before overlay construction.
        if(save_raw.load()) {
            const auto raw_path=vision_debug_record_path(
                hand==1?"original/right":"original/left","servo_raw");
            cv::imwrite(raw_path,cv::Mat(frame.height,frame.width,CV_8UC3,frame.color_bgr.data()));
        }
        return; // no YOLO call and no closest-box fallback for tray placement
    }
    try { result=bridge->run_yolo(frame,algorithm_id_for_slot(camera),false,camera,-1,SegEngineId::Hand); }
    catch(const std::exception &ex) { result.ok=false; result.message=ex.what(); }
    obs.completed_s=wrist_servo::monotonic_seconds();
    obs.infer_ms=(obs.completed_s-infer_start)*1000;
    if(!result.ok) { wrist_servo::observation_event(hand,context.session,"inference_failed",obs); if(!context.belt_circle_fallback)return; }
    if(wrist_servo::monotonic_seconds()-obs.capture_s+obs.clock_uncertainty_s>cfg.max_observation_age_s) {
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
    PoseTargetResult circle_target;
    if(!best && context.belt_circle_fallback) {
        wrist_servo::observation_event(hand,context.session,"circle_attempt",obs);
        Eigen::Matrix4d base_camera;wrist_servo::State capture;
        if(wrist_servo::camera_at_capture(hand,context.session,obs.capture_s,base_camera,&capture)) {
            auto geometric_profile=context.profile;
            geometric_profile.feature_z_offset_m=cfg.belt2_circle.feature_above_final_m;
            const auto predicted=wrist_servo::map_observation(geometric_profile,obs,capture,context.final_z,
                base_camera.block<3,1>(0,3),base_camera.block<3,3>(0,0));
            if(predicted.valid) {
                const auto expected=predicted.reference_px+predicted.jacobian*(capture.position.head<2>()-context.nominal_xy);
                const auto circle=vision::locate_belt_circle(frame,base_camera,
                    context.final_z+cfg.belt2_circle.feature_above_final_m,expected,cfg.belt2_circle);
                obs.vision_source="belt_circle";obs.geometry_radius_m=circle.radius_m;
                obs.geometry_residual_m=circle.residual_m;obs.geometry_coverage_bins=circle.coverage_bins;
                obs.geometry_inliers=circle.inliers;obs.geometry_ms=circle.elapsed_ms;
                obs.saturated_fraction=circle.saturated_fraction;
                if(circle.valid) {
                    circle_target.box_u=circle.pixel.x();circle_target.box_v=circle.pixel.y();
                    circle_target.class_id=0;circle_target.confidence=.75;
                    best=&circle_target;obs.pixel=circle.pixel;obs.confidence=.75;
                    std::lock_guard<std::mutex> lock(watch_mutex);
                    if(watches[hand].session!=context.session || watches[hand].generation!=generation)return;
                    watches[hand].pixel=obs.pixel;watches[hand].locked=true;
                }
                wrist_servo::observation_event(hand,context.session,circle.reason.c_str(),obs);
            } else wrist_servo::observation_event(hand,context.session,"circle_prediction_invalid",obs);
        } else wrist_servo::observation_event(hand,context.session,"circle_capture_state_unavailable",obs);
        obs.completed_s=wrist_servo::monotonic_seconds();obs.infer_ms=(obs.completed_s-infer_start)*1000;
        if(obs.completed_s-obs.capture_s+obs.clock_uncertainty_s>cfg.max_observation_age_s) {
            wrist_servo::observation_event(hand,context.session,"circle_frame_expired",obs);return;
        }
    }
    // Submit first: debug JPEG encoding and storage must not age a usable observation.
    if(best && !wrist_servo::submit_observation(hand,obs))
        wrist_servo::observation_event(hand,context.session,"session_not_accepting_submit",obs);
    if(cfg.tray_place.save_debug_images || save_raw.load()) {
        static double last_grasp_image_s[2]{};
        const double now_s=wrist_servo::monotonic_seconds();
        if(now_s-last_grasp_image_s[hand]>=0.25 || save_raw.load()) {
            last_grasp_image_s[hand]=now_s;
            cv::Mat canvas(frame.height,frame.width,CV_8UC3,
                           const_cast<unsigned char*>(frame.color_bgr.data()));
            canvas=canvas.clone();
            const cv::Point mark(cvRound(model.reference_px.x()),cvRound(model.reference_px.y()));
            cv::drawMarker(canvas,mark,cv::Scalar(255,0,0),cv::MARKER_CROSS,36,2,cv::LINE_AA);
            cv::circle(canvas,mark,16,cv::Scalar(255,0,0),2,cv::LINE_AA);
            cv::putText(canvas,"ref "+std::to_string(mark.x)+","+std::to_string(mark.y),
                        mark+cv::Point(12,-12),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,0,0),1,cv::LINE_AA);
            if(best) cv::circle(canvas,cv::Point(cvRound(best->box_u),cvRound(best->box_v)),
                                6,cv::Scalar(0,255,0),-1,cv::LINE_AA);
            cv::rectangle(canvas,cv::Point(0,0),cv::Point(canvas.cols,28),cv::Scalar(0,0,0),cv::FILLED);
            cv::putText(canvas,std::string(hand==1?"right ":"left ")+(best?obs.vision_source:"grasp missing"),
                        cv::Point(8,20),cv::FONT_HERSHEY_SIMPLEX,0.55,cv::Scalar(255,255,255),1,cv::LINE_AA);
            std::ostringstream metadata;metadata<<std::setprecision(17);
            metadata<<"hand: "<<hand<<"\nsession: "<<context.session<<"\nframe: "<<obs.frame
                    <<"\nvision_source: "<<obs.vision_source<<"\ncapture_s: "<<obs.capture_s<<"\nfinal_z: "<<context.final_z
                    <<"\nfeature_above_final_m: "<<context.profile.feature_z_offset_m
                    <<"\ngeometry_radius_m: "<<obs.geometry_radius_m<<"\ngeometry_residual_m: "<<obs.geometry_residual_m
                    <<"\ngeometry_coverage_bins: "<<obs.geometry_coverage_bins<<"\ngeometry_ms: "<<obs.geometry_ms<<"\nK: [";
            for(int i=0;i<9;++i)metadata<<(i?", ":"")<<frame.K[i];metadata<<"]\ndist: [";
            for(int i=0;i<5;++i)metadata<<(i?", ":"")<<frame.dist[i];metadata<<"]\n";
            Eigen::Matrix4d bc;
            if(wrist_servo::camera_at_capture(hand,context.session,obs.capture_s,bc)) {
                metadata<<"base_from_camera:\n";for(int r=0;r<4;++r){metadata<<"  - [";for(int c=0;c<4;++c)metadata<<(c?", ":"")<<bc(r,c);metadata<<"]\n";}
            }
            cv::Mat raw=save_raw.load()?cv::Mat(frame.height,frame.width,CV_8UC3,frame.color_bgr.data()).clone():cv::Mat();
            static DebugImageWriter writer;
            writer.put(hand,vision_debug_record_path(hand==1?"right":"left","servo"),std::move(canvas),std::move(raw),metadata.str());
        }
    }
    if(!best) wrist_servo::observation_event(hand,context.session,"locked_target_missing",obs);
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
    const auto path=t170::logging::file("hand_track.csv");
    std::error_code ec; std::filesystem::create_directories(path.parent_path(),ec);
    std::ofstream out(path,std::ios::app);
    out << wrist_servo::monotonic_seconds() << ',' << hand << ',' << result << ',' << detail << '\n';
}
