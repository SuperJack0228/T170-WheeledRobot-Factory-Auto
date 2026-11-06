#include "t170/vision/tray_localizer.h"
#include "tray_registration.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>
#include <iomanip>
#include <chrono>

namespace wrist_servo {
namespace {
double now_ms() {return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();}
cv::Mat cv_matrix(const Eigen::Matrix3d &m) {
    cv::Mat out(3,3,CV_64F);
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) out.at<double>(i,j)=m(i,j);
    return out;
}
Eigen::Matrix3d eigen_matrix(const cv::Mat &m) {
    Eigen::Matrix3d out;
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) out(i,j)=m.at<double>(i,j);
    return out;
}
Eigen::Vector2d project(const Eigen::Matrix3d &h,const Eigen::Vector2d &p) {
    const Eigen::Vector3d v=h*Eigen::Vector3d(p.x(),p.y(),1);
    if(!v.allFinite() || std::abs(v.z())<1e-10) return Eigen::Vector2d::Constant(NAN);
    return v.head<2>()/v.z();
}
double median(std::vector<double> values) {
    std::sort(values.begin(),values.end());
    const auto n=values.size(); return n%2 ? values[n/2] : .5*(values[n/2-1]+values[n/2]);
}
Eigen::Matrix3d homography(const Eigen::Matrix4d &camera_from_tray,const Eigen::Matrix3d &k) {
    Eigen::Matrix3d h;
    h.col(0)=camera_from_tray.block<3,1>(0,0);
    h.col(1)=camera_from_tray.block<3,1>(0,1);
    h.col(2)=camera_from_tray.block<3,1>(0,3);
    return k*h;
}
bool rigid_pose(const std::vector<cv::Point2f> &plane,const std::vector<cv::Point2f> &image,
                const cv::Mat &mask,const Eigen::Matrix3d &k,const Eigen::Matrix4d &prior,Eigen::Matrix4d &pose) {
    std::vector<cv::Point3d> objects;std::vector<cv::Point2d> observations;
    for(std::size_t i=0;i<plane.size();++i) if(mask.empty() || mask.at<unsigned char>(i)) {
        objects.emplace_back(plane[i].x,plane[i].y,0);observations.emplace_back(image[i].x,image[i].y);
    }
    if(objects.size()<4) return false;
    cv::Mat rotation_vector,translation=(cv::Mat_<double>(3,1)<<prior(0,3),prior(1,3),prior(2,3));
    cv::Rodrigues(cv_matrix(prior.block<3,3>(0,0)),rotation_vector);
    // Refine a rigid pose, rather than accepting the extra degrees of freedom
    // of a noisy homography as a physical camera pose. Input pixels are undistorted.
    if(!cv::solvePnP(objects,observations,cv_matrix(k),cv::noArray(),rotation_vector,translation,true,cv::SOLVEPNP_ITERATIVE)) return false;
    cv::Mat rotation;cv::Rodrigues(rotation_vector,rotation);
    pose=Eigen::Matrix4d::Identity();pose.block<3,3>(0,0)=eigen_matrix(rotation);
    pose.block<3,1>(0,3)<<translation.at<double>(0),translation.at<double>(1),translation.at<double>(2);
    if(!pose.allFinite() || pose(2,3)<=.015) return false;
    for(const auto &p:objects) if((pose*Eigen::Vector4d(p.x,p.y,0,1)).z()<=.015) return false;
    return true;
}
bool translate_plane(const std::vector<cv::Point2f> &plane,const std::vector<cv::Point2f> &image,
                     const Eigen::Matrix4d &base_camera,const Eigen::Matrix3d &k,
                     const Eigen::Matrix4d &prior,double gate,int minimum,double max_shift,
                     Eigen::Matrix4d &refined,double &reprojection,std::string &detail) {
    const auto inverse=homography(base_camera.inverse()*prior,k).inverse().eval();
    detail="invalid_prior_or_insufficient_points";
    if(!inverse.allFinite() || plane.size()<static_cast<std::size_t>(minimum)) return false;
    std::vector<Eigen::Vector2d> shifts;std::vector<double> dx,dy;
    for(std::size_t i=0;i<plane.size();++i) {
        const auto p=project(inverse,{image[i].x,image[i].y});
        if(!p.allFinite()) return false;
        const Eigen::Vector2d d=p-Eigen::Vector2d(plane[i].x,plane[i].y);
        shifts.push_back(d);dx.push_back(d.x());dy.push_back(d.y());
    }
    const Eigen::Vector2d shift(median(dx),median(dy));
    auto diagnostic=[&](const char *reason,int agree) {
        std::ostringstream msg;msg<<std::fixed<<std::setprecision(2)<<reason
            <<" candidates="<<shifts.size()<<" agree="<<agree<<" median_shift_mm="
            <<shift.x()*1000<<":"<<shift.y()*1000<<" consensus_gate_mm="<<gate*1000;
        detail=msg.str();
    };
    if(shift.norm()>max_shift) {diagnostic("shift_limit",0);return false;}
    int agree=0;Eigen::Vector2d sum=Eigen::Vector2d::Zero();
    for(const auto &d:shifts) if((d-shift).norm()<=gate) {++agree;sum+=d;}
    if(agree<std::max(minimum,static_cast<int>(std::ceil(shifts.size()*.6)))) {
        diagnostic("insufficient_consensus",agree);return false;
    }
    refined=prior;
    const Eigen::Vector3d delta=prior.block<3,3>(0,0)*Eigen::Vector3d(sum.x()/agree,sum.y()/agree,0);
    refined(0,3)+=delta.x();refined(1,3)+=delta.y(); // never update tray Z/normal from a close-up
    const auto h=homography(base_camera.inverse()*refined,k);double square=0;
    for(std::size_t i=0;i<shifts.size();++i) if((shifts[i]-shift).norm()<=gate)
        square+=(project(h,{plane[i].x,plane[i].y})-Eigen::Vector2d(image[i].x,image[i].y)).squaredNorm();
    reprojection=std::sqrt(square/agree);diagnostic("fit",agree);
    detail+=" reprojection_px="+std::to_string(reprojection);
    return std::isfinite(reprojection);
}
}

NeighborGridFit fit_neighbor_grid(const std::vector<Eigen::Vector2d> &nominal,
    const std::vector<Eigen::Vector2d> &observed,const Eigen::Vector2d &target,
    double pitch,const TrayPlaceVisionConfig &cfg,bool identity_resolved) {
    NeighborGridFit out;
    if(nominal.size()!=observed.size() || nominal.size()<static_cast<std::size_t>(cfg.min_neighbors) ||
       !target.allFinite() || !std::isfinite(pitch) || pitch<=0) {out.detail="insufficient_pairs";return out;}
    std::vector<cv::Point2f> from,to;
    for(std::size_t i=0;i<nominal.size();++i) {
        if(!nominal[i].allFinite() || !observed[i].allFinite()) {out.detail="nonfinite_pair";return out;}
        from.emplace_back(nominal[i].x(),nominal[i].y());to.emplace_back(observed[i].x(),observed[i].y());
    }
    // Affine needs noncollinear spatial support. Collinear/sparse sets use the
    // lower-degree similarity model rather than an unconstrained affine matrix.
    auto spread=[&](const std::vector<int> &ids) {
        Eigen::Vector2d mean=Eigen::Vector2d::Zero();
        for(int i:ids) mean+=nominal[i];mean/=ids.size();
        Eigen::Matrix2d covariance=Eigen::Matrix2d::Zero();
        for(int i:ids) {const Eigen::Vector2d d=nominal[i]-mean;covariance+=d*d.transpose();}
        Eigen::JacobiSVD<Eigen::Matrix2d> svd(covariance);
        return svd.singularValues()(0)>pitch*pitch*.1 &&
               svd.singularValues()(1)>svd.singularValues()(0)*.02;
    };
    std::vector<int> all;for(int i=0;i<static_cast<int>(nominal.size());++i) all.push_back(i);
    for(bool full:{true,false}) {
        if(full && (from.size()<4 || !spread(all))) continue;
        cv::Mat mask;
        const double gate=2*cfg.circle_residual_m;
        cv::Mat fitted=full ? cv::estimateAffine2D(from,to,mask,cv::RANSAC,gate,1000,.99,10) :
            cv::estimateAffinePartial2D(from,to,mask,cv::RANSAC,gate,1000,.99,10);
        if(fitted.empty() || !cv::checkRange(fitted)) continue;
        std::vector<int> ids;
        for(int i=0;i<mask.rows;++i) if(mask.at<unsigned char>(i)) ids.push_back(i);
        if(ids.size()<static_cast<std::size_t>(std::max(full?4:cfg.min_neighbors,
                    static_cast<int>(std::ceil(from.size()*.6))))) continue;
        if(full && !spread(ids)) continue;
        Eigen::Matrix3d a=Eigen::Matrix3d::Identity();
        for(int r=0;r<2;++r) for(int c=0;c<3;++c) a(r,c)=fitted.at<double>(r,c);
        const Eigen::Matrix2d linear=a.block<2,2>(0,0);
        Eigen::JacobiSVD<Eigen::Matrix2d> svd(linear,Eigen::ComputeFullU|Eigen::ComputeFullV);
        const Eigen::Matrix2d rotation=svd.matrixU()*svd.matrixV().transpose();
        const double angle=std::abs(std::atan2(rotation(1,0),rotation(0,0)));
        if(linear.determinant()<=0 || svd.singularValues()(1)<cfg.affine_min_scale ||
           svd.singularValues()(0)>cfg.affine_max_scale ||
           svd.singularValues()(0)/svd.singularValues()(1)>1.8 || angle>cfg.affine_max_rotation_rad) continue;
        const Eigen::Vector2d predicted=project(a,target);
        const double budget=identity_resolved?cfg.global_search_m:pitch*.45;
        if((predicted-target).norm()>=budget) {out.detail="target_identity_budget";continue;}
        std::vector<cv::Point2f> support,hull;
        for(int i:ids) support.push_back(from[i]);
        if(support.size()>=3) cv::convexHull(support,hull);
        double outside=INFINITY;
        if(hull.size()>=3 && std::abs(cv::contourArea(hull))>1e-8)
            outside=std::max(0.0,-cv::pointPolygonTest(hull,{float(target.x()),float(target.y())},true));
        else for(int i:ids) outside=std::min(outside,(nominal[i]-target).norm());
        if(outside>cfg.max_extrapolation_pitches*pitch) {out.detail="target_too_far_from_support";continue;}
        double error=0;
        for(int i:ids) error+=(project(a,nominal[i])-observed[i]).squaredNorm();
        std::ostringstream detail;detail<<"model="<<(full?"affine":"similarity")<<" inliers="<<ids.size()
            <<"/"<<from.size()<<" scales="<<svd.singularValues().transpose()<<" rotation_rad="<<angle
            <<" rms_model_mm="<<1000*std::sqrt(error/ids.size())<<" outside_support_mm="<<outside*1000;
        out.valid=true;out.transform=a;out.target=predicted;out.inliers=std::move(ids);
        out.model=full?"neighbor_affine":"neighbor_similarity";out.detail=detail.str();return out;
    }
    if(out.detail.empty()) out.detail="no_supported_grid_model";
    return out;
}

bool load_tray_board(const std::string &path,TrayBoardGeometry &b,std::string &error) {
    try {
        const auto root=YAML::LoadFile(path),tray=root["tray"],holes=tray["holes"];
        b.rows=holes["rows"].as<int>(); b.cols=holes["cols"].as<int>();
        b.pitch_m=holes["pitch_m"].as<double>(); b.radius_m=holes["diameter_m"].as<double>()*.5;
        b.offset_x_m=holes["offset_x_m"].as<double>(0); b.offset_y_m=holes["offset_y_m"].as<double>(0);
        // Marker dimensions are retained only for legacy diagnostics/fixtures.
        // Wrist localization requires hole geometry, not any marker dictionary/ID.
        const auto code=root["tray_grasp_code"],t2=root["tray2"];
        if(code) {
            b.marker_pitch_m=code["pitch_xy_m"].as<double>(b.marker_pitch_m);
            b.marker_side_m=code["black_side_m"].as<double>(b.marker_side_m);
            if(code["locate_id"]) b.marker_ids.push_back(code["locate_id"].as<int>());
            if(code["info_ids"]) for(const auto &v:code["info_ids"]) b.marker_ids.push_back(v.as<int>());
        }
        if(t2) {
            b.marker_pitch_m=t2["pitch_xy_m"].as<double>(b.marker_pitch_m);
            b.marker_side_m=t2["black_side_m"].as<double>(b.marker_side_m);
        }
        if(b.rows<2 || b.rows>32 || b.cols<2 || b.cols>32 || !std::isfinite(b.pitch_m) || b.pitch_m<=0 ||
           !std::isfinite(b.radius_m) || b.radius_m<=0 || 2*b.radius_m>=b.pitch_m ||
           !std::isfinite(b.offset_x_m) || !std::isfinite(b.offset_y_m))
            throw std::runtime_error("invalid tray geometry");
        error.clear(); return true;
    } catch(const std::exception &ex) { error=ex.what(); return false; }
}

bool make_tray_target(const std::vector<TrayAnchor> &anchors,const TrayBoardGeometry &board,
                      int row,int col,TrayTarget &target,std::string &error) {
    target={};
    if(row<1 || row>board.rows || col<1 || col>board.cols || anchors.size()<4) {
        error="missing assigned hole identity or head tray anchors"; return false;
    }
    Eigen::MatrixXd x(anchors.size(),3),y(anchors.size(),3);
    for(std::size_t i=0;i<anchors.size();++i) {
        if(!anchors[i].base.allFinite()) {error="invalid head tray anchor"; return false;}
        x.row(i).head<2>()=board.hole(anchors[i].row,anchors[i].col).transpose(); x(i,2)=1;
        y.row(i)=anchors[i].base.transpose();
    }
    const auto qr=x.colPivHouseholderQr();
    if(qr.rank()<3) {error="head tray anchors are collinear";return false;}
    const Eigen::Matrix3d affine=qr.solve(y);
    Eigen::Matrix3d r;
    r.col(0)=affine.row(0).transpose(); r.col(1)=affine.row(1).transpose(); r.col(2)=r.col(0).cross(r.col(1));
    if(std::abs(r.col(0).norm()-1)>.05 || std::abs(r.col(1).norm()-1)>.05 ||
       (x*affine-y).rowwise().norm().maxCoeff()>.003) {error="head tray geometry/board dimensions disagree";return false;}
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(r,Eigen::ComputeFullU|Eigen::ComputeFullV);
    target.base_from_tray.block<3,3>(0,0)=svd.matrixU()*svd.matrixV().transpose();
    target.base_from_tray.block<3,1>(0,3)=affine.row(2).transpose();
    target.board=board; target.row=row; target.col=col; target.valid=true;
    for(const auto &a:anchors) if(a.class_id==0 || a.class_id==2)
        target.occupied_holes.push_back(a.row*100+a.col);
    error.clear(); return true;
}

struct TrayLocalizer::Impl {
    TrayTarget target;
    TrayPlaceVisionConfig cfg;
    Eigen::Matrix4d refined=Eigen::Matrix4d::Identity();
    Eigen::Matrix3d image_correction=Eigen::Matrix3d::Identity();
    bool registered=false;
    cv::Mat undistort_map1,undistort_map2;
    std::array<double,9> cached_k{};std::array<double,5> cached_dist{};
    int cached_width=0,cached_height=0;
    std::vector<int> support;
    int misses=0, acquisition_frames=0, single_frames=0;
    double full_capture_s=0,single_travel=0;
};
TrayLocalizer::TrayLocalizer():impl_(std::make_unique<Impl>()) {}
TrayLocalizer::~TrayLocalizer()=default;
void TrayLocalizer::reset(const TrayTarget &target,const TrayPlaceVisionConfig &cfg) {
    impl_->target=target; impl_->cfg=cfg; impl_->refined=target.base_from_tray;
    impl_->image_correction.setIdentity();
    impl_->registered=false;impl_->misses=impl_->acquisition_frames=impl_->single_frames=0;
    impl_->single_travel=0;impl_->full_capture_s=0;impl_->support.clear();
}

TrayLocalization TrayLocalizer::locate(const CameraFrameData &frame,const Eigen::Matrix4d &base_camera,
                                     const TrayPixelGate &executable_target) {
    auto &s=*impl_; const auto &board=s.target.board; const auto &cfg=s.cfg;
    TrayLocalization out;out.source=cfg.xy_only?"neighbor_xy":"neighbor_geometry";
    if(cfg.association_gate_m>=board.pitch_m*.45) {out.reason="tray_association_gate_exceeds_half_pitch";return out;}
    if(!s.target.valid || !frame.ok || frame.width<1 || frame.height<1 ||
       frame.color_bgr.size()!=std::size_t(frame.width)*frame.height*3 || !base_camera.allFinite()) {
        out.reason="invalid_tray_frame_or_target"; return out;
    }
    Eigen::Matrix3d k=Eigen::Matrix3d::Identity();
    k(0,0)=frame.K[0]; k(1,1)=frame.K[4]; k(0,2)=frame.K[2]; k(1,2)=frame.K[5];
    if(k(0,0)<=1 || k(1,1)<=1) {out.reason="invalid_tray_intrinsics";return out;}
    const cv::Mat K=cv_matrix(k),dist(1,5,CV_64F,const_cast<double*>(frame.dist.data()));
    cv::Mat undistorted,color;
    const cv::Mat bgr(frame.height,frame.width,CV_8UC3,const_cast<unsigned char*>(frame.color_bgr.data()));
    const double undistort_start=now_ms();
    if(std::all_of(frame.dist.begin(),frame.dist.end(),[](double x){return x==0;})) color=bgr;
    else {
        if(s.cached_width!=frame.width || s.cached_height!=frame.height || s.cached_k!=frame.K || s.cached_dist!=frame.dist) {
            cv::initUndistortRectifyMap(K,dist,cv::Mat(),K,{frame.width,frame.height},CV_16SC2,s.undistort_map1,s.undistort_map2);
            s.cached_width=frame.width;s.cached_height=frame.height;s.cached_k=frame.K;s.cached_dist=frame.dist;
        }
        cv::remap(bgr,color,s.undistort_map1,s.undistort_map2,cv::INTER_LINEAR);
    }
    out.undistort_ms=now_ms()-undistort_start;
    Eigen::Matrix4d working_base=s.refined,camera_tray=base_camera.inverse()*working_base;
    if(camera_tray(2,3)<=.015) {out.reason="tray_behind_camera";return out;}
    const Eigen::Matrix3d seed_projection=homography(camera_tray,k);
    const Eigen::Matrix3d seed_inverse=seed_projection.inverse();
    Eigen::Matrix3d prior=seed_projection;
    if(cfg.affine_enabled && cfg.xy_only) prior=seed_projection*s.image_correction;
    Eigen::Matrix3d inverse=prior.inverse();
    if(!inverse.allFinite()) {out.reason="singular_tray_prior";return out;}
    auto to_raw=[&](const Eigen::Vector2d &undistorted_px)->Eigen::Vector2d {
        if(!undistorted_px.allFinite()) return {-1,-1};
        const Eigen::Vector3d ray=k.inverse()*Eigen::Vector3d(undistorted_px.x(),undistorted_px.y(),1);
        std::vector<cv::Point2d> distorted;
        cv::projectPoints(std::vector<cv::Point3d>{{ray.x(),ray.y(),1}},cv::Vec3d(0,0,0),
                          cv::Vec3d(0,0,0),K,dist,distorted);
        if(distorted.empty()) return {-1,-1};
        return {distorted[0].x,distorted[0].y};
    };
    const auto target_xy=board.hole(s.target.row,s.target.col);
    out.stage=s.registered?"tracking":"acquiring";
    std::string acquisition_detail;
    if(cfg.xy_only && cfg.affine_enabled && cfg.global_acquisition && !s.registered) {
        const bool attempt=(s.acquisition_frames++%cfg.reacquire_after_frames)==0;
        if(!attempt) {out.reason="tray_acquire_wait_next_frame";return out;}
        const double acquisition_start=now_ms();
        const auto scene=tray_registration::discover(color,seed_projection,board,cfg);
        for(const auto &rim:scene.rims) out.candidate_pixels.push_back(to_raw(project(seed_projection,rim.point)));
        TrayPixelGate plane_gate;
        if(executable_target) plane_gate=[&](const Eigen::Vector2d &p) {return executable_target(to_raw(project(seed_projection,p)));};
        const auto acquired=tray_registration::acquire(scene,s.target,cfg,plane_gate);
        out.acquisition_ms=now_ms()-acquisition_start;
        out.global_hypotheses=acquired.hypotheses;out.identity_margin=acquired.margin;
        out.detail=acquired.detail;
        if(!acquired.valid) {
            out.reason=acquired.reason;
            const auto raw=to_raw(project(prior,target_xy));out.target_u=raw.x();out.target_v=raw.y();
            for(int r=1;r<=board.rows;++r) for(int c=1;c<=board.cols;++c) {
                if(r==s.target.row && c==s.target.col) continue;
                const auto p=to_raw(project(prior,board.hole(r,c)));
                TrayImageMark mark;mark.row=r;mark.col=c;mark.predicted_u=p.x();mark.predicted_v=p.y();
                mark.occupied=std::find(s.target.occupied_holes.begin(),s.target.occupied_holes.end(),r*100+c)!=s.target.occupied_holes.end();
                out.marks.push_back(mark);
            }
            return out;
        }
        // This update is an image-model correction, never a commanded TCP displacement.
        s.image_correction=acquired.fit.transform;s.registered=true;s.misses=0;
        prior=seed_projection*s.image_correction;inverse=prior.inverse();
        out.stage="acquired_refining";acquisition_detail=acquired.detail+" ";
    }
    const auto target_raw=to_raw(project(prior,target_xy));
    out.target_u=static_cast<float>(target_raw.x());
    out.target_v=static_cast<float>(target_raw.y());
    std::vector<cv::Point2f> plane_points,image_points;
    std::vector<Eigen::Vector2d> neighbor_shifts,nominal_points,observed_points;
    // Wrist placement uses visible hole rims only. Head perception owns board
    // identity; a missing, partial or inconsistent marker cannot veto neighbors.
    std::ostringstream neighbor_detail;
    neighbor_detail<<std::fixed<<std::setprecision(2);
    const bool fast=cfg.xy_only && cfg.affine_enabled;
    const double prepare_start=now_ms();
    const double scale=cfg.rectified_pixels_per_m,extent=board.radius_m+cfg.association_gate_m;
    const int side=static_cast<int>(std::ceil(2*extent*scale))+1;
    struct Candidate {int row,col;bool occupied;Eigen::Vector2d xy,predicted;};
    std::vector<Candidate> candidates;
    for(int row=1;row<=board.rows;++row) for(int col=1;col<=board.cols;++col) {
        if(row==s.target.row && col==s.target.col) continue;
        const bool occupied=std::find(s.target.occupied_holes.begin(),s.target.occupied_holes.end(),row*100+col)!=s.target.occupied_holes.end();
        if(occupied) {++out.skipped_occupied;if(!cfg.orange_rim_enabled || !cfg.occupied_orange_rims) continue;}
        const auto xy=board.hole(row,col),predicted=project(prior,xy);
        if(!predicted.allFinite() || predicted.x()<0 || predicted.y()<0 || predicted.x()>=frame.width || predicted.y()>=frame.height) continue;
        candidates.push_back({row,col,occupied,xy,predicted});
    }
    if(fast) {
        auto rank=[&](const Candidate &c) {
            const bool known=std::find(s.support.begin(),s.support.end(),c.row*100+c.col)!=s.support.end();
            return (c.xy-target_xy).norm()-(known?.025:0);
        };
        std::stable_sort(candidates.begin(),candidates.end(),[&](const auto &a,const auto &b){return rank(a)<rank(b);});
        // Round-robin quadrants keep the near-target support spatially distributed.
        std::array<std::vector<Candidate>,4> sectors;
        for(const auto &c:candidates) {
            const Eigen::Vector2d d=c.xy-target_xy;
            const int sector=(d.x()<0?2:0)+(d.y()<0?1:0);sectors[sector].push_back(c);
        }
        candidates.clear();
        for(std::size_t i=0;candidates.size()<static_cast<std::size_t>(cfg.tracking_scan_limit);++i) {
            bool more=false;
            for(auto &sector:sectors) if(i<sector.size()) {candidates.push_back(sector[i]);more=true;
                if(candidates.size()==static_cast<std::size_t>(cfg.tracking_scan_limit)) break;}
            if(!more) break;
        }
    }
    cv::Mat orange_source,rectified_mask;
    Eigen::Vector2d rectified_origin=Eigen::Vector2d::Zero();
    if(cfg.orange_rim_enabled) {
        cv::Mat hsv;cv::cvtColor(color,hsv,cv::COLOR_BGR2HSV);
        cv::inRange(hsv,cv::Scalar(0,85,65),cv::Scalar(28,255,255),orange_source);
        cv::morphologyEx(orange_source,orange_source,cv::MORPH_CLOSE,cv::getStructuringElement(cv::MORPH_ELLIPSE,{3,3}));
        if(fast && !candidates.empty()) {
            Eigen::Vector2d low=candidates[0].xy,high=low;
            for(const auto &c:candidates) {low=low.cwiseMin(c.xy);high=high.cwiseMax(c.xy);}
            low.array()-=extent;high.array()+=extent;
            const int width=int(std::ceil((high.x()-low.x())*scale))+3,height=int(std::ceil((high.y()-low.y())*scale))+3;
            if(width<=2048 && height<=2048) {
                Eigen::Matrix3d plane_from_patch=Eigen::Matrix3d::Identity();
                plane_from_patch(0,0)=plane_from_patch(1,1)=1/scale;plane_from_patch.block<2,1>(0,2)=low;
                cv::warpPerspective(orange_source,rectified_mask,cv_matrix(prior*plane_from_patch),{width,height},
                    cv::INTER_LINEAR|cv::WARP_INVERSE_MAP,cv::BORDER_CONSTANT,cv::Scalar(0));
                cv::threshold(rectified_mask,rectified_mask,127,255,cv::THRESH_BINARY);rectified_origin=low;
            }
        }
    }
    out.prepare_ms=now_ms()-prepare_start;
    const double roi_start=now_ms();
    std::vector<int> pair_ids;
    for(const auto &candidate:candidates) {
        if(fast && out.neighbors>=cfg.tracking_max_neighbors) break;
        const int row=candidate.row,col=candidate.col;
        const bool occupied=candidate.occupied;
        const auto &xy=candidate.xy;const auto &predicted=candidate.predicted;
        const auto predicted_raw=to_raw(predicted);
        TrayImageMark mark;mark.row=row;mark.col=col;mark.occupied=occupied;
        mark.predicted_u=predicted_raw.x();mark.predicted_v=predicted_raw.y();
        Eigen::Vector2d roi_origin=xy-Eigen::Vector2d::Constant(extent);
        cv::Mat mask;
        if(!rectified_mask.empty()) {
            const int x=cvRound((roi_origin.x()-rectified_origin.x())*scale),y=cvRound((roi_origin.y()-rectified_origin.y())*scale);
            mask=rectified_mask(cv::Rect(x,y,side,side));roi_origin=rectified_origin+Eigen::Vector2d(x/scale,y/scale);
        } else if(!orange_source.empty()) {
            Eigen::Matrix3d transform=Eigen::Matrix3d::Identity();transform(0,0)=transform(1,1)=1/scale;
            transform.block<2,1>(0,2)=roi_origin;
            cv::warpPerspective(orange_source,mask,cv_matrix(prior*transform),{side,side},
                cv::INTER_LINEAR|cv::WARP_INVERSE_MAP,cv::BORDER_CONSTANT,cv::Scalar(0));
            cv::threshold(mask,mask,127,255,cv::THRESH_BINARY);
        }
        ++out.rois;
        double best_score=INFINITY;Eigen::Vector2d found;bool best_partial=false;
        auto full_rim=[&](const std::vector<cv::Point> &contour,bool orange) {
            ++out.contours;
            if(contour.size()<30) {++out.rejected_short;return;}
            const auto ellipse=cv::fitEllipse(contour);
            const double ra=ellipse.size.width*.5/scale,rb=ellipse.size.height*.5/scale;
            const double radius=orange?.5*(ra+rb):board.radius_m;
            if(orange ? (std::min(ra,rb)<board.radius_m*cfg.rim_min_radius_ratio ||
                         std::max(ra,rb)>std::min(board.radius_m*cfg.rim_max_radius_ratio,board.pitch_m*.45)) :
                        (std::abs(ra/board.radius_m-1)>(cfg.affine_enabled?.35:.22) ||
                         std::abs(rb/board.radius_m-1)>(cfg.affine_enabled?.35:.22))) {++out.rejected_size;return;}
            if(std::min(ra,rb)/std::max(ra,rb)<(cfg.affine_enabled?.65:.8)) {++out.rejected_shape;return;}
            const Eigen::Vector2d point=roi_origin+Eigen::Vector2d(ellipse.center.x/scale,ellipse.center.y/scale),delta=point-xy;
            if(delta.norm()>cfg.association_gate_m) {++out.rejected_distance;return;}
            if((point-target_xy).norm()<board.radius_m*1.5) {++out.rejected_target;return;}
            double residual=0;std::array<bool,16> bins{};
            const double theta=ellipse.angle*std::acos(-1)/180,cs=std::cos(theta),sn=std::sin(theta);
            for(const auto &p:contour) {
                const double dx=(p.x-ellipse.center.x)/scale,dy=(p.y-ellipse.center.y)/scale;
                const double ex=(cs*dx+sn*dy)/ra,ey=(-sn*dx+cs*dy)/rb;
                residual+=cfg.affine_enabled?std::pow((std::hypot(ex,ey)-1)*.5*(ra+rb),2):std::pow(std::hypot(dx,dy)-radius,2);
                const double angle=cfg.affine_enabled?std::atan2(ey,ex):std::atan2(dy,dx);
                bins[std::clamp(int((angle+std::acos(-1))*8/std::acos(-1)),0,15)]=true;
            }
            residual=std::sqrt(residual/contour.size());
            if(residual>cfg.circle_residual_m) {++out.rejected_residual;return;}
            if(std::count(bins.begin(),bins.end(),true)<cfg.min_coverage_bins) {++out.rejected_coverage;return;}
            const double score=residual+.02*delta.norm()+(orange?0:.0005);
            if(score<best_score) {best_score=score;found=point;best_partial=false;}
        };
        if(!mask.empty()) {
            std::vector<std::vector<cv::Point>> rims;cv::findContours(mask,rims,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_NONE);
            for(const auto &contour:rims) full_rim(contour,true);
            // Complete rims terminate the fast path. Partial fitting is reserved for
            // plausible, unresolved orange contours (at most two per selected hole).
            if(!std::isfinite(best_score) && cfg.partial_rims && s.registered) {
                std::sort(rims.begin(),rims.end(),[](const auto &a,const auto &b){return a.size()>b.size();});
                int attempts=0;
                for(const auto &contour:rims) {
                    if(contour.size()<30) continue;
                    const auto box=cv::boundingRect(contour);const double diameter=std::max(box.width,box.height)/scale;
                    if(diameter<board.radius_m*.8 || diameter>board.radius_m*3.5) continue;
                    if(attempts++>=2) break;
                    const double partial_start=now_ms();++out.partial_calls;cv::Point2d center;
                    const bool valid=tray_registration::fit_partial_rim(contour,mask,scale,board.radius_m,cfg,center);
                    out.partial_ms+=now_ms()-partial_start;
                    if(!valid) continue;
                    const Eigen::Vector2d point=roi_origin+Eigen::Vector2d(center.x/scale,center.y/scale),delta=point-xy;
                    if(delta.norm()>std::min(cfg.association_gate_m,.010) || (point-target_xy).norm()<board.radius_m*1.5) continue;
                    const double score=cfg.circle_residual_m+.02*delta.norm()+.001;
                    if(score<best_score) {best_score=score;found=point;best_partial=true;}
                }
            }
        }
        if(!std::isfinite(best_score) && !occupied) {
            if(undistorted.empty()) cv::cvtColor(color,undistorted,cv::COLOR_BGR2GRAY);
            Eigen::Matrix3d transform=Eigen::Matrix3d::Identity();transform(0,0)=transform(1,1)=1/scale;transform.block<2,1>(0,2)=roi_origin;
            cv::Mat patch,edges;cv::warpPerspective(undistorted,patch,cv_matrix(prior*transform),{side,side},
                cv::INTER_LINEAR|cv::WARP_INVERSE_MAP,cv::BORDER_CONSTANT,cv::Scalar(127));
            cv::GaussianBlur(patch,patch,{3,3},.8);cv::Canny(patch,edges,35,100);
            std::vector<std::vector<cv::Point>> contours;cv::findContours(edges,contours,cv::RETR_LIST,cv::CHAIN_APPROX_NONE);
            for(const auto &contour:contours) full_rim(contour,false);
        }
        if(!std::isfinite(best_score)) {out.marks.push_back(mark);continue;}
        const auto observed=project(prior,found),observed_raw=to_raw(observed);
        mark.accepted=true;mark.observed_u=observed_raw.x();mark.observed_v=observed_raw.y();out.marks.push_back(mark);
        plane_points.emplace_back(xy.x(),xy.y());image_points.emplace_back(observed.x(),observed.y());
        nominal_points.push_back(xy);observed_points.push_back(project(seed_inverse,observed));
        neighbor_shifts.push_back(found-xy);pair_ids.push_back(row*100+col);++out.neighbors;
        if(occupied) ++out.occupied_used;if(best_partial) ++out.partial_used;
        neighbor_detail<<" r"<<row<<"c"<<col<<"_shift_mm="<<(found.x()-xy.x())*1000<<":"<<(found.y()-xy.y())*1000;
    }
    out.roi_ms=now_ms()-roi_start;
    NeighborGridFit single;
    if(out.neighbors==1 && s.registered && s.single_frames<cfg.single_neighbor_max_frames &&
       frame.capture_steady_s>0 && s.full_capture_s>0 && frame.capture_steady_s>=s.full_capture_s &&
       frame.capture_steady_s-s.full_capture_s<=cfg.single_neighbor_max_age_s) {
        const Eigen::Vector2d shift=observed_points[0]-project(s.image_correction,nominal_points[0]);
        if(shift.norm()<=cfg.single_neighbor_step_m && s.single_travel+shift.norm()<=cfg.single_neighbor_budget_m) {
            single.transform=s.image_correction;single.transform.block<2,1>(0,2)+=shift;
            single.target=project(single.transform,target_xy);
            single.valid=(single.target-target_xy).norm()<cfg.global_search_m;
            single.inliers={0};single.model="neighbor_translation_tracked";
            single.detail="single_neighbor_prior_only frames="+std::to_string(s.single_frames+1);
            if(single.valid) {++s.single_frames;s.single_travel+=shift.norm();out.stage="sparse_tracking";}
        }
    }
    auto miss=[&]() {
        if(++s.misses>=cfg.reacquire_after_frames) {s.registered=false;s.acquisition_frames=0;}
    };
    if(out.neighbors<cfg.min_neighbors && !single.valid) {
        miss();out.reason="tray_insufficient_visible_neighbors";
        out.detail=acquisition_detail+neighbor_detail.str();return out;
    }
    if(cfg.xy_only && cfg.affine_enabled) {
        const double fit_start=now_ms();
        NeighborGridFit fit;
        std::vector<int> active;for(int i=0;i<out.neighbors;++i) active.push_back(i);
        for(int attempt=0;attempt<3;++attempt) {
            if(single.valid) fit=single;
            else {
                std::vector<Eigen::Vector2d> from,to;
                for(int i:active) {from.push_back(nominal_points[i]);to.push_back(observed_points[i]);}
                fit=fit_neighbor_grid(from,to,target_xy,board.pitch_m,cfg,s.registered);
                for(auto &i:fit.inliers) i=active[i];
            }
            if(!fit.valid) break;
            const Eigen::Matrix3d model=seed_projection*fit.transform;
            double error=0,worst_error=-1;int worst=-1;
            for(int i:fit.inliers) {
                const double e=(project(model,nominal_points[i])-Eigen::Vector2d(image_points[i].x,image_points[i].y)).squaredNorm();
                error+=e;if(e>worst_error) {worst_error=e;worst=i;}
            }
            out.reprojection_px=std::sqrt(error/fit.inliers.size());
            if(out.reprojection_px<=cfg.reprojection_gate_px || single.valid || attempt==2 || fit.inliers.size()<=4) break;
            active=fit.inliers;active.erase(std::remove(active.begin(),active.end(),worst),active.end());++out.pruned_neighbors;
        }
        out.fit_ms=now_ms()-fit_start;
        out.detail=acquisition_detail+fit.detail+" pruned="+std::to_string(out.pruned_neighbors)+neighbor_detail.str();
        if(!fit.valid) {miss();out.reason="tray_neighbor_registration_rejected";return out;}
        const Eigen::Matrix3d image_model=seed_projection*fit.transform;
        if(out.reprojection_px>cfg.reprojection_gate_px) {miss();out.reason="tray_affine_reprojection_gate";return out;}
        const auto undistorted_target=project(image_model,target_xy);
        const Eigen::Vector3d ray=k.inverse()*Eigen::Vector3d(undistorted_target.x(),undistorted_target.y(),1);
        std::vector<cv::Point2d> distorted;
        cv::projectPoints(std::vector<cv::Point3d>{{ray.x(),ray.y(),1}},cv::Vec3d(0,0,0),
            cv::Vec3d(0,0,0),K,dist,distorted);
        out.pixel={distorted[0].x,distorted[0].y};
        if(!out.pixel.allFinite() || out.pixel.x()<0 || out.pixel.y()<0 ||
           out.pixel.x()>=frame.width || out.pixel.y()>=frame.height) {
            out.reason="virtual_target_outside_image";return out;
        }
        out.plane_z_m=(s.target.base_from_tray*Eigen::Vector4d(target_xy.x(),target_xy.y(),0,1)).z();
        if(executable_target && !executable_target(out.pixel)) {
            miss();out.reason="tray_target_outside_tcp_envelope";return out;
        }
        s.image_correction=fit.transform;
        s.misses=0;
        s.support.clear();for(int i:fit.inliers) s.support.push_back(pair_ids[i]);
        if(!single.valid) {s.single_frames=0;s.single_travel=0;s.full_capture_s=frame.capture_steady_s;}
        out.valid=true;out.reason="ok";out.source=fit.model;return out;
    }
    Eigen::Matrix4d refined=working_base;
    bool updated=false;
    if(cfg.xy_only) {
        std::string fit_detail;
        updated=translate_plane(plane_points,image_points,base_camera,k,working_base,
            cfg.circle_residual_m*2,cfg.min_neighbors,cfg.max_plane_update_m,refined,
            out.reprojection_px,fit_detail);
        out.detail=fit_detail+neighbor_detail.str();
        if(updated && out.reprojection_px>cfg.reprojection_gate_px) {
            updated=false;out.detail="reprojection_gate "+out.detail;
        }
        out.translation_only=updated;
    }
    // Four noncollinear neighbor centers support a full homography. RANSAC rejects a mistaken rim; do not update from a single point.
    if(!cfg.xy_only && plane_points.size()>=4) {
        cv::Mat mask;
        const auto candidate=cv::findHomography(plane_points,image_points,cv::RANSAC,cfg.reprojection_gate_px,mask);
        if(!candidate.empty() && cv::countNonZero(mask)>=std::max(4,static_cast<int>(std::ceil(plane_points.size()*.6)))) {
            Eigen::Matrix4d pose;
            if(rigid_pose(plane_points,image_points,mask,k,camera_tray,pose)) {
                const auto model=homography(pose,k); double square_error=0; int inliers=0;
                for(std::size_t i=0;i<plane_points.size();++i) if(mask.at<unsigned char>(i)) {
                    const auto p=project(model,{plane_points[i].x,plane_points[i].y});
                    square_error+=(p-Eigen::Vector2d(image_points[i].x,image_points[i].y)).squaredNorm(); ++inliers;
                }
                out.reprojection_px=std::sqrt(square_error/std::max(1,inliers));
                refined=base_camera*pose;
                const auto old_target=(working_base*Eigen::Vector4d(target_xy.x(),target_xy.y(),0,1)).head<3>().eval();
                const auto new_target=(refined*Eigen::Vector4d(target_xy.x(),target_xy.y(),0,1)).head<3>().eval();
                updated=out.reprojection_px<=cfg.reprojection_gate_px &&
                    (new_target-old_target).norm()<=cfg.max_plane_update_m &&
                    Eigen::AngleAxisd(refined.block<3,3>(0,0)*working_base.block<3,3>(0,0).transpose()).angle()<.15;
            }
        }
    }
    if(!cfg.xy_only && !updated && neighbor_shifts.size()>=static_cast<std::size_t>(cfg.min_neighbors)) {
        std::vector<double> dx,dy;
        for(const auto &d:neighbor_shifts) {dx.push_back(d.x());dy.push_back(d.y());}
        const Eigen::Vector2d shift(median(dx),median(dy));
        int agree=0; double residual=0;
        for(const auto &d:neighbor_shifts) if((d-shift).norm()<=cfg.circle_residual_m*2) {
            ++agree;residual+=(d-shift).squaredNorm();
        }
        if(agree>=std::max(cfg.min_neighbors,static_cast<int>(std::ceil(neighbor_shifts.size()*.6))) && shift.norm()<=cfg.max_plane_update_m) {
            refined=working_base;
            refined.block<3,1>(0,3)+=refined.block<3,3>(0,0)*Eigen::Vector3d(shift.x(),shift.y(),0);
            out.reprojection_px=std::sqrt(residual/agree)*k(0,0)/camera_tray(2,3);
            out.translation_only=true; updated=out.reprojection_px<=cfg.reprojection_gate_px;
        }
    }
    if(!updated) {out.reason="tray_neighbor_fit_rejected";return out;}
    const Eigen::Vector4d base_target=refined*Eigen::Vector4d(target_xy.x(),target_xy.y(),0,1);
    const Eigen::Vector4d seed_target=s.target.base_from_tray*Eigen::Vector4d(target_xy.x(),target_xy.y(),0,1);
    const Eigen::Vector3d anchor=seed_target.head<3>();
    const double identity_budget=board.pitch_m*.45;
    if((base_target.head<3>()-anchor).norm()>=identity_budget) {
        out.reason="tray_total_update_exceeds_identity_budget";return out;
    }
    const Eigen::Vector4d camera_target=base_camera.inverse()*base_target;
    if(camera_target.z()<=.015) {out.reason="target_behind_camera";return out;}
    std::vector<cv::Point2d> distorted;
    cv::projectPoints(std::vector<cv::Point3d>{{camera_target.x(),camera_target.y(),camera_target.z()}},
                      cv::Vec3d(0,0,0),cv::Vec3d(0,0,0),K,dist,distorted);
    out.pixel={distorted[0].x,distorted[0].y}; out.plane_z_m=base_target.z();
    if(!out.pixel.allFinite() || out.pixel.x()<0 || out.pixel.x()>=frame.width || out.pixel.y()<0 || out.pixel.y()>=frame.height) {
        out.reason="virtual_target_outside_image";return out;
    }
    s.refined=refined;
    out.valid=true; out.reason="ok";
    out.source=cfg.xy_only ? "neighbor_xy" :
        (out.translation_only ? "neighbor_translation" : "neighbor_homography");
    return out;
}
}
