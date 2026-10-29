#include "servo/tray_localizer.h"
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>

namespace wrist_servo {
namespace {
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
}

bool load_tray_board(const std::string &path,TrayBoardGeometry &b,std::string &error) {
    try {
        const auto root=YAML::LoadFile(path),tray=root["tray"],holes=tray["holes"];
        b.rows=holes["rows"].as<int>(); b.cols=holes["cols"].as<int>();
        b.pitch_m=holes["pitch_m"].as<double>(); b.radius_m=holes["diameter_m"].as<double>()*.5;
        b.offset_x_m=holes["offset_x_m"].as<double>(0); b.offset_y_m=holes["offset_y_m"].as<double>(0);
        const auto code=root["tray_grasp_code"],t2=root["tray2"];
        if(code["dictionary"].as<std::string>()!="DICT_5X5_1000") throw std::runtime_error("unsupported tray dictionary");
        b.marker_pitch_m=t2["pitch_xy_m"].as<double>(code["pitch_xy_m"].as<double>());
        b.marker_side_m=t2["black_side_m"].as<double>(code["black_side_m"].as<double>());
        std::set<int> ids{code["locate_id"].as<int>()};
        for(const auto &v:code["info_ids"]) ids.insert(v.as<int>());
        b.marker_ids.assign(ids.begin(),ids.end());
        if(b.rows<2 || b.rows>32 || b.cols<2 || b.cols>32 || !std::isfinite(b.pitch_m) || b.pitch_m<=0 ||
           !std::isfinite(b.radius_m) || b.radius_m<=0 || 2*b.radius_m>=b.pitch_m ||
           !std::isfinite(b.marker_pitch_m) || b.marker_pitch_m<=0 ||
           !std::isfinite(b.marker_side_m) || b.marker_side_m<=0 ||
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
    error.clear(); return true;
}

struct TrayLocalizer::Impl {
    TrayTarget target;
    TrayPlaceVisionConfig cfg;
    Eigen::Matrix4d refined=Eigen::Matrix4d::Identity();
    bool marker_anchored=false;
    Eigen::Vector3d identity_anchor=Eigen::Vector3d::Zero();
#if CV_VERSION_MAJOR>4 || (CV_VERSION_MAJOR==4 && CV_VERSION_MINOR>=7)
    cv::Ptr<cv::aruco::Dictionary> dictionary=cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000));
#else
    cv::Ptr<cv::aruco::Dictionary> dictionary=cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
#endif
    cv::Ptr<cv::aruco::DetectorParameters> parameters=cv::makePtr<cv::aruco::DetectorParameters>();
};
TrayLocalizer::TrayLocalizer():impl_(std::make_unique<Impl>()) {}
TrayLocalizer::~TrayLocalizer()=default;
void TrayLocalizer::reset(const TrayTarget &target,const TrayPlaceVisionConfig &cfg) {
    impl_->target=target; impl_->cfg=cfg; impl_->refined=target.base_from_tray;
    impl_->marker_anchored=false;
    impl_->parameters->cornerRefinementMethod=cv::aruco::CORNER_REFINE_SUBPIX;
}

TrayLocalization TrayLocalizer::locate(const CameraFrameData &frame,const Eigen::Matrix4d &base_camera) {
    auto &s=*impl_; const auto &board=s.target.board; const auto &cfg=s.cfg;
    TrayLocalization out;
    if(cfg.association_gate_m>=board.pitch_m*.45) {out.reason="tray_association_gate_exceeds_half_pitch";return out;}
    if(!s.target.valid || !frame.ok || frame.width<1 || frame.height<1 ||
       frame.color_bgr.size()!=std::size_t(frame.width)*frame.height*3 || !base_camera.allFinite()) {
        out.reason="invalid_tray_frame_or_target"; return out;
    }
    Eigen::Matrix3d k=Eigen::Matrix3d::Identity();
    k(0,0)=frame.K[0]; k(1,1)=frame.K[4]; k(0,2)=frame.K[2]; k(1,2)=frame.K[5];
    if(k(0,0)<=1 || k(1,1)<=1) {out.reason="invalid_tray_intrinsics";return out;}
    const cv::Mat K=cv_matrix(k),dist(1,5,CV_64F,const_cast<double*>(frame.dist.data()));
    cv::Mat gray,undistorted;
    cv::cvtColor(cv::Mat(frame.height,frame.width,CV_8UC3,const_cast<unsigned char*>(frame.color_bgr.data())),gray,cv::COLOR_BGR2GRAY);
    cv::undistort(gray,undistorted,K,dist);
    Eigen::Matrix4d working_base=s.refined,camera_tray=base_camera.inverse()*working_base;
    if(camera_tray(2,3)<=.015) {out.reason="tray_behind_camera";return out;}
    Eigen::Matrix3d prior=homography(camera_tray,k),inverse=prior.inverse();
    if(!inverse.allFinite()) {out.reason="singular_tray_prior";return out;}
    const auto target_xy=board.hole(s.target.row,s.target.col);
    std::vector<cv::Point2f> plane_points,image_points;
    std::vector<Eigen::Vector2d> neighbor_shifts;
    // Decode marker corners directly; duplicated IDs are disambiguated by the
    // head-anchored plane, NOT by inventing a unique ID per corner.
    std::vector<int> ids; std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(undistorted,s.dictionary,corners,ids,s.parameters);
    const double half=board.marker_pitch_m*.5,h=board.marker_side_m*.5;
    const std::array<Eigen::Vector2d,4> slots{{{half,half},{half,-half},{-half,half},{-half,-half}}};
    const std::array<Eigen::Vector2d,4> local{{{h,h},{h,-h},{-h,-h},{-h,h}}};
    std::set<int> used_slots;
    for(std::size_t i=0;i<ids.size();++i) {
        if(std::find(board.marker_ids.begin(),board.marker_ids.end(),ids[i])==board.marker_ids.end()) continue;
        Eigen::Vector2d center=Eigen::Vector2d::Zero();
        for(const auto &p:corners[i]) center+=Eigen::Vector2d(p.x,p.y)*.25;
        const auto guessed=project(inverse,center);
        // Marker centers are 300mm apart, so they tolerate a larger initial
        // hand-eye translation error than a periodic 75mm hole lattice.
        const double marker_gate=board.marker_pitch_m*.25;
        int slot=-1; double best=marker_gate;
        for(int j=0;j<4;++j) if(!used_slots.count(j) && (guessed-slots[j]).norm()<best) {
            best=(guessed-slots[j]).norm(); slot=j;
        }
        if(slot<0) continue;
        // Check decoded corner ordering too, rejecting a flipped/unrelated marker.
        bool good=true;
        for(int j=0;j<4;++j) {
            const auto p=project(inverse,{corners[i][j].x,corners[i][j].y});
            if((p-slots[slot]-local[j]).norm()>marker_gate) good=false;
        }
        if(!good) continue;
        used_slots.insert(slot);
        for(int j=0;j<4;++j) {
            const auto xy=slots[slot]+local[j];
            plane_points.emplace_back(xy.x(),xy.y()); image_points.push_back(corners[i][j]);
        }
        out.marker_corners+=4;
    }
    if(out.marker_corners>=4) {
        cv::Mat marker_inliers;
        const auto fitted=cv::findHomography(plane_points,image_points,cv::RANSAC,cfg.reprojection_gate_px,marker_inliers);
        Eigen::Matrix4d pose;
        if(fitted.empty() || !rigid_pose(plane_points,image_points,marker_inliers,k,camera_tray,pose)) {
            out.reason="tray_marker_pose_invalid";return out;
        }
        const Eigen::Matrix4d candidate=base_camera*pose;
        const Eigen::Vector4d xy(target_xy.x(),target_xy.y(),0,1);
        if(((candidate-s.refined)*xy).head<3>().norm()>board.marker_pitch_m*.25 ||
           Eigen::AngleAxisd(candidate.block<3,3>(0,0)*s.refined.block<3,3>(0,0).transpose()).angle()>.20) {
            out.reason="tray_marker_identity_disagrees_with_head";return out;
        }
        // Bootstrap from decoded corners BEFORE looking for periodic holes.
        // Otherwise a biased seed could pair holes one pitch away and outvote markers.
        working_base=candidate;camera_tray=pose;prior=homography(pose,k);inverse=prior.inverse();
    }
    // Rectify EACH visible neighbor ROI into the tray plane before circle fitting.
    // Neither a detection-box center nor a raw-image ellipse center is a hole center.
    const double scale=cfg.rectified_pixels_per_m,extent=board.radius_m+cfg.association_gate_m;
    const int side=static_cast<int>(std::ceil(2*extent*scale))+1;
    for(int row=1;row<=board.rows;++row) for(int col=1;col<=board.cols;++col) {
        if(row==s.target.row && col==s.target.col) continue; // always exclude the occluded target
        const Eigen::Vector2d xy=board.hole(row,col),predicted=project(prior,xy);
        if(!predicted.allFinite() || predicted.x()<0 || predicted.y()<0 || predicted.x()>=frame.width || predicted.y()>=frame.height) continue;
        Eigen::Matrix3d plane_from_roi=Eigen::Matrix3d::Identity();
        plane_from_roi(0,0)=plane_from_roi(1,1)=1/scale;
        plane_from_roi(0,2)=xy.x()-extent; plane_from_roi(1,2)=xy.y()-extent;
        cv::Mat patch,edges;
        cv::warpPerspective(undistorted,patch,cv_matrix(prior*plane_from_roi),{side,side},cv::INTER_LINEAR|cv::WARP_INVERSE_MAP,cv::BORDER_CONSTANT,cv::Scalar(127));
        cv::GaussianBlur(patch,patch,{3,3},.8); cv::Canny(patch,edges,35,100);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(edges,contours,cv::RETR_LIST,cv::CHAIN_APPROX_NONE);
        double best_score=INFINITY; Eigen::Vector2d found;
        for(const auto &contour:contours) {
            if(contour.size()<30) continue;
            const auto ellipse=cv::fitEllipse(contour);
            const double ra=ellipse.size.width*.5/scale,rb=ellipse.size.height*.5/scale;
            if(std::abs(ra/board.radius_m-1)>.22 || std::abs(rb/board.radius_m-1)>.22 ||
               std::min(ra,rb)/std::max(ra,rb)<.8) continue;
            const Eigen::Vector2d delta(ellipse.center.x/scale-extent,ellipse.center.y/scale-extent);
            if(delta.norm()>cfg.association_gate_m ||
               (xy+delta-target_xy).norm()<board.radius_m*1.5) continue;
            double residual=0; std::array<bool,16> bins{};
            for(const auto &p:contour) {
                const double dx=(p.x-ellipse.center.x)/scale,dy=(p.y-ellipse.center.y)/scale;
                residual+=std::pow(std::hypot(dx,dy)-board.radius_m,2);
                const int bin=std::clamp(static_cast<int>((std::atan2(dy,dx)+std::acos(-1))*8/std::acos(-1)),0,15);
                bins[bin]=true;
            }
            residual=std::sqrt(residual/contour.size());
            if(residual>cfg.circle_residual_m || std::count(bins.begin(),bins.end(),true)<14) continue;
            const double score=residual+.02*delta.norm();
            if(score<best_score) {best_score=score;found=xy+delta;}
        }
        if(!std::isfinite(best_score)) continue;
        const auto observed=project(prior,found);
        plane_points.emplace_back(xy.x(),xy.y()); image_points.emplace_back(observed.x(),observed.y());
        neighbor_shifts.push_back(found-xy); ++out.neighbors;
    }
    if(out.marker_corners==0 && out.neighbors<cfg.min_neighbors) {
        out.reason="tray_insufficient_visible_landmarks";return out;
    }
    Eigen::Matrix4d refined=working_base;
    bool updated=false;
    // Four noncollinear neighbor centers or a decoded marker support a full
    // homography. RANSAC rejects a mistaken rim; do not update from a single point.
    if(plane_points.size()>=4) {
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
    if(!updated && neighbor_shifts.size()>=static_cast<std::size_t>(cfg.min_neighbors)) {
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
    if(!updated) {out.reason="tray_inconsistent_or_degenerate_landmarks";return out;}
    const Eigen::Vector4d base_target=refined*Eigen::Vector4d(target_xy.x(),target_xy.y(),0,1);
    const Eigen::Vector4d seed_target=s.target.base_from_tray*Eigen::Vector4d(target_xy.x(),target_xy.y(),0,1);
    const Eigen::Vector3d anchor=s.marker_anchored ? s.identity_anchor : seed_target.head<3>().eval();
    const double identity_budget=!s.marker_anchored && out.marker_corners>=4
                                     ? board.marker_pitch_m*.25 : board.pitch_m*.45;
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
    if(!s.marker_anchored && out.marker_corners>=4) {s.marker_anchored=true;s.identity_anchor=base_target.head<3>();}
    out.valid=true; out.reason="ok";
    out.source=out.translation_only ? "neighbor_translation" : (out.marker_corners ? "marker_homography" : "neighbor_homography");
    return out;
}
}
