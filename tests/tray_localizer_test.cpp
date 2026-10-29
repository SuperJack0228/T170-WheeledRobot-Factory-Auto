#include "servo/tray_localizer.h"
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <iostream>
#include <set>
#include <stdexcept>

using namespace wrist_servo;
void require(bool ok,const char *reason) {if(!ok) throw std::runtime_error(reason);}
CameraFrameData blank_frame() {
    CameraFrameData f;f.ok=true;f.width=1280;f.height=720;
    f.K={900,0,640,0,900,360,0,0,1};f.color_bgr.resize(f.width*f.height*3,220);return f;
}
std::vector<cv::Point2f> pixels(const CameraFrameData &f,const Eigen::Matrix4d &base_camera,
                               const Eigen::Matrix4d &base_tray,const std::vector<Eigen::Vector2d> &xy) {
    std::vector<cv::Point3f> xyz;const Eigen::Matrix4d camera_tray=base_camera.inverse()*base_tray;
    for(const auto &p:xy) {
        const Eigen::Vector4d c=camera_tray*Eigen::Vector4d(p.x(),p.y(),0,1);
        xyz.emplace_back(c.x(),c.y(),c.z());
    }
    cv::Mat k(3,3,CV_64F,const_cast<double*>(f.K.data())),d(1,5,CV_64F,const_cast<double*>(f.dist.data()));
    std::vector<cv::Point2f> out;cv::projectPoints(xyz,cv::Vec3d(0,0,0),cv::Vec3d(0,0,0),k,d,out);return out;
}
CameraFrameData render(const TrayTarget &target,const Eigen::Matrix4d &actual,const Eigen::Matrix4d &camera,
                       bool markers=false,const std::set<int> &only={},double distortion=0) {
    auto f=blank_frame();f.dist[0]=distortion;f.dist[1]=-distortion*.3;
    cv::Mat image(f.height,f.width,CV_8UC3,f.color_bgr.data());
    for(int r=1;r<=target.board.rows;++r) for(int c=1;c<=target.board.cols;++c) {
        if(r==target.row && c==target.col) continue; // the target is completely hidden
        if(!only.empty() && !only.count(r*100+c)) continue;
        std::vector<Eigen::Vector2d> circle;
        const auto center=target.board.hole(r,c);
        for(int i=0;i<180;++i) {
            const double a=i*2*std::acos(-1)/180;
            circle.push_back(center+target.board.radius_m*Eigen::Vector2d(std::cos(a),std::sin(a)));
        }
        const auto projected=pixels(f,camera,actual,circle);std::vector<cv::Point> contour;
        for(const auto &p:projected) contour.emplace_back(cvRound(p.x),cvRound(p.y));
        cv::fillConvexPoly(image,contour,cv::Scalar(25,25,25),cv::LINE_AA);
    }
    if(markers) {
#if CV_VERSION_MAJOR>4 || (CV_VERSION_MAJOR==4 && CV_VERSION_MINOR>=7)
        auto dictionary=cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
#else
        auto dictionary=*cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
#endif
        const double half=target.board.marker_pitch_m*.5,h=target.board.marker_side_m*.5;
        const std::vector<Eigen::Vector2d> centers{{half,half},{half,-half},{-half,half},{-half,-half}};
        int slot=0;
        for(const auto &center:centers) {
            cv::Mat marker;
#if CV_VERSION_MAJOR>4 || (CV_VERSION_MAJOR==4 && CV_VERSION_MINOR>=7)
            cv::aruco::generateImageMarker(dictionary,slot%2 ? 2 : 100,200,marker);
#else
            cv::aruco::drawMarker(cv::makePtr<cv::aruco::Dictionary>(dictionary),slot%2 ? 2 : 100,200,marker);
#endif
            ++slot;
            const auto dst=pixels(f,camera,actual,{center+Eigen::Vector2d(h,h),center+Eigen::Vector2d(h,-h),
                                                  center+Eigen::Vector2d(-h,-h),center+Eigen::Vector2d(-h,h)});
            const std::vector<cv::Point2f> src{{0,0},{199,0},{199,199},{0,199}};
            cv::Mat projected;cv::warpPerspective(marker,projected,cv::getPerspectiveTransform(src,dst),image.size(),cv::INTER_NEAREST,cv::BORDER_CONSTANT,cv::Scalar(255));
            cv::Mat dark;cv::threshold(projected,dark,80,255,cv::THRESH_BINARY_INV);image.setTo(cv::Scalar(0,0,0),dark);
        }
    }
    return f;
}
int main(int argc,char **argv) {
    try {
        require(argc==2,"board path required");
        TrayBoardGeometry board;std::string error;require(load_tray_board(argv[1],board,error),"actual board did not load");
        Eigen::Matrix4d actual=Eigen::Matrix4d::Identity(),camera=Eigen::Matrix4d::Identity();
        camera(1,1)=camera(2,2)=-1;camera(2,3)=.5;
        std::vector<TrayAnchor> anchors;
        for(int r=1;r<=6;++r) for(int c=1;c<=6;++c) {
            const auto xy=board.hole(r,c);anchors.push_back({r,c,Eigen::Vector3d(xy.x(),xy.y(),0)});
        }
        TrayTarget target;require(make_tray_target(anchors,board,3,4,target,error),"head anchor fit failed");
        require((target.base_from_tray-actual).norm()<1e-9,"head row/column coordinate convention changed");
        TrayPlaceVisionConfig cfg;cfg.enabled=true;
        auto seed=target;seed.base_from_tray(0,3)=.010;seed.base_from_tray(1,3)=-.008;
        TrayLocalizer localizer;localizer.reset(seed,cfg);
        auto frame=render(target,actual,camera);
        const auto start=std::chrono::steady_clock::now();
        auto located=localizer.locate(frame,camera);
        std::cout << "neighbor result=" << located.reason << " source=" << located.source << " n=" << located.neighbors
                  << " residual=" << located.reprojection_px << '\n';
        require(located.valid && located.neighbors>=4,"hidden-target neighbor homography failed");
        const auto expected=pixels(frame,camera,actual,{board.hole(target.row,target.col)})[0];
        require((located.pixel-Eigen::Vector2d(expected.x,expected.y)).norm()<1.5,"virtual target center inaccurate");
        // A visible target is not required, even with only two neighbors.
        localizer.reset(seed,cfg);auto two=render(target,actual,camera,false,{303,404});
        auto sparse=localizer.locate(two,camera);
        require(sparse.valid && sparse.translation_only && sparse.neighbors==2,"two-neighbor translation fallback failed");
        require((sparse.pixel-Eigen::Vector2d(expected.x,expected.y)).norm()<1.5,"two-neighbor target drifted");
        auto partial=two;
        const auto occluded=pixels(partial,camera,actual,{board.hole(3,3)})[0];
        cv::Mat partial_image(partial.height,partial.width,CV_8UC3,partial.color_bgr.data());
        cv::rectangle(partial_image,cv::Rect(cvRound(occluded.x)-45,cvRound(occluded.y)-45,46,90),cv::Scalar(220,220,220),cv::FILLED);
        localizer.reset(seed,cfg);
        require(!localizer.locate(partial,camera).valid,"partially occluded neighbor rim was accepted as complete");
        auto invisible=blank_frame();
        require(!localizer.locate(invisible,camera).valid,"invisible tray produced a fresh observation");
        // A partial arc must not stand in for a reliable neighbor rim.
        localizer.reset(seed,cfg);
        auto one=render(target,actual,camera,false,{303});
        require(!localizer.locate(one,camera).valid,"single ambiguous neighbor accepted");
        // Perspective and lens distortion: render physical circles, not ideal image ellipses.
        actual.block<3,3>(0,0)=(Eigen::AngleAxisd(.25,Eigen::Vector3d::UnitY())*
                                               Eigen::AngleAxisd(.10,Eigen::Vector3d::UnitX())).toRotationMatrix();
        seed=target;seed.base_from_tray=actual;seed.base_from_tray(0,3)+=.006;
        seed.base_from_tray.block<3,3>(0,0)=Eigen::AngleAxisd(.05,Eigen::Vector3d::UnitX()).toRotationMatrix()*actual.block<3,3>(0,0);
        localizer.reset(seed,cfg);auto tilted=render(target,actual,camera,false,{},.12);
        const auto perspective=localizer.locate(tilted,camera);
        const auto true_pixel=pixels(tilted,camera,actual,{board.hole(target.row,target.col)})[0];
        std::cout << "perspective=" << perspective.reason << " n=" << perspective.neighbors << '\n';
        require(perspective.valid && (perspective.pixel-Eigen::Vector2d(true_pixel.x,true_pixel.y)).norm()<2,
                "rectified-circle perspective/distortion correction failed");
        // Duplicate marker IDs and a whole-tray 180 degree rotation.
        actual.block<3,3>(0,0)=Eigen::AngleAxisd(std::acos(-1),Eigen::Vector3d::UnitZ()).toRotationMatrix();
        seed=target;seed.base_from_tray=actual;seed.base_from_tray(0,3)+=.040;
        localizer.reset(seed,cfg);auto marked=render(target,actual,camera,true);
        auto decoded=localizer.locate(marked,camera);
        std::cout << "markers=" << decoded.reason << " corners=" << decoded.marker_corners << '\n';
        require(decoded.valid && decoded.marker_corners>=8,"duplicate IDs/rotated tray markers failed");
        localizer.reset(seed,cfg);
        auto markers_only=render(target,actual,camera,true,{999});
        auto marker_pose=localizer.locate(markers_only,camera);
        require(marker_pose.valid && marker_pose.neighbors==0 && marker_pose.marker_corners>=8,
                "marker-only occlusion handling failed");
        // Camera moves continuously; the same physical hole identity remains locked.
        camera(0,3)=.012;camera(2,3)=.42;
        auto moved=render(target,actual,camera,true);
        auto tracked=localizer.locate(moved,camera);
        const auto moved_pixel=pixels(moved,camera,actual,{board.hole(target.row,target.col)})[0];
        require(tracked.valid && (tracked.pixel-Eigen::Vector2d(moved_pixel.x,moved_pixel.y)).norm()<2,
                "moving camera reused a stale pixel or changed target identity");
        auto flipped=moved;
        cv::Mat flipped_image(flipped.height,flipped.width,CV_8UC3,flipped.color_bgr.data());
        cv::rotate(flipped_image,flipped_image,cv::ROTATE_180);
        flipped.K[2]=flipped.width-1-flipped.K[2];flipped.K[5]=flipped.height-1-flipped.K[5];
        flipped.dist[2]=-flipped.dist[2];flipped.dist[3]=-flipped.dist[3];
        Eigen::Matrix4d optical=Eigen::Matrix4d::Identity();optical(0,0)=optical(1,1)=-1;
        seed=target;seed.base_from_tray=actual;localizer.reset(seed,cfg);
        const auto left_view=localizer.locate(flipped,camera*optical);
        require(left_view.valid && (left_view.pixel-Eigen::Vector2d(flipped.width-1-moved_pixel.x,flipped.height-1-moved_pixel.y)).norm()<2,
                "left-camera 180-degree calibrated image convention changed");
        const auto elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        std::cout << "tray_localizer: hidden target, sparse support, lost features, perspective, distortion, duplicate IDs and moving camera passed; fixture suite ms=" << elapsed << '\n';
    } catch(const std::exception &ex) {std::cerr << "tray_localizer FAIL: " << ex.what() << '\n';return 1;}
}
