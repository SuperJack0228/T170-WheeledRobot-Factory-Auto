#include "t170/vision/tray_localizer.h"
#include "../src/vision/geometry/tray_registration.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <set>
#include <stdexcept>
using namespace wrist_servo;
void check(bool ok,const char *message) {if(!ok) throw std::runtime_error(message);}
Eigen::Vector2d pixel(const Eigen::Vector2d &p,const Eigen::Matrix4d &camera) {
    const Eigen::Vector4d q=camera.inverse()*Eigen::Vector4d(p.x(),p.y(),0,1);
    return {640+900*q.x()/q.z(),480+900*q.y()/q.z()};
}
CameraFrameData image(const TrayTarget &target,const Eigen::Matrix4d &camera,
                      const std::set<int> &visible={},bool partial=false,double time=1) {
    CameraFrameData f;f.ok=true;f.width=1280;f.height=960;f.capture_steady_s=time;
    f.K={900,0,640,0,900,480,0,0,1};f.color_bgr.resize(f.width*f.height*3,240);
    cv::Mat bgr(f.height,f.width,CV_8UC3,f.color_bgr.data());
    std::vector<cv::Point> board;
    for(auto p:std::vector<Eigen::Vector2d>{{-.225,-.225},{.225,-.225},{.225,.225},{-.225,.225}}) {
        auto v=pixel(p,camera);board.emplace_back(cvRound(v.x()),cvRound(v.y()));
    }
    cv::fillConvexPoly(bgr,board,cv::Scalar(65,65,65));
    for(int r=1;r<=6;++r) for(int c=1;c<=6;++c) {
        if(r==target.row && c==target.col) continue;
        if(!visible.empty() && !visible.count(r*100+c)) continue;
        const auto p=target.board.hole(r,c);const auto center=pixel(p,camera);
        const int radius=cvRound((pixel(p+Eigen::Vector2d(target.board.radius_m,0),camera)-center).norm());
        cv::circle(bgr,{cvRound(center.x()),cvRound(center.y())},cvRound(radius*1.35),{30,110,230},cv::FILLED,cv::LINE_AA);
        cv::circle(bgr,{cvRound(center.x()),cvRound(center.y())},radius,{40,40,40},cv::FILLED,cv::LINE_AA);
        if(partial) cv::rectangle(bgr,cv::Rect(cvRound(center.x()),cvRound(center.y()-radius*1.5),
            cvRound(radius*1.5),cvRound(radius*3)),cv::Scalar(120,120,120),cv::FILLED);
    }
    return f;
}
int main() {
    try {
        TrayTarget target;target.valid=true;target.row=3;target.col=3;
        Eigen::Matrix4d camera=Eigen::Matrix4d::Identity();camera(1,1)=camera(2,2)=-1;camera(2,3)=.50;
        TrayPlaceVisionConfig cfg;cfg.enabled=true;
        auto seed=target;seed.base_from_tray(0,3)=.050;seed.base_from_tray(1,3)=.015;
        TrayLocalizer localizer;localizer.reset(seed,cfg);
        auto f=image(target,camera);auto out=localizer.locate(f,camera);
        std::cout<<"large bias: "<<out.reason<<" "<<out.detail<<'\n';
        const auto truth=pixel(target.board.hole(3,3),camera);
        check(out.valid && out.neighbors>=4 && (out.pixel-truth).norm()<2,"50mm bootstrap or absolute grid identity failed");
        check(out.neighbors<=cfg.tracking_max_neighbors && out.rois<=cfg.tracking_scan_limit,
              "normal tracking ignored the bounded feature budget");
        check(out.partial_calls==0,"complete rings still paid for partial-arc RANSAC");
        auto distant=seed;distant.base_from_tray(0,3)=.080;
        TrayLocalizer wide;wide.reset(distant,cfg);const auto wide_result=wide.locate(f,camera);
        check(wide_result.valid && (wide_result.pixel-truth).norm()<2,
              "large initial projection correction was confused with TCP displacement or a neighboring row");
        // Only one trusted neighbor may update translation briefly, never initialize a grid.
        camera(0,3)=.002;
        auto one=image(target,camera,{302},false,1.1);out=localizer.locate(one,camera);
        check(out.valid && out.source=="neighbor_translation_tracked" &&
            (out.pixel-pixel(target.board.hole(3,3),camera)).norm()<2,"trusted single-neighbor tracking failed");
        auto late=image(target,camera,{302},false,2.0);out=localizer.locate(late,camera);
        check(!out.valid,"single neighbor was trusted indefinitely");
        localizer.reset(seed,cfg);out=localizer.locate(one,camera);
        check(!out.valid,"single neighbor initialized an absolute grid");
        // Periodic ambiguity without a boundary must remain ambiguous, even with a perfect residual.
        tray_registration::Scene scene;
        for(int r=2;r<=3;++r) for(int c=1;c<=2;++c)
            scene.rims.push_back({target.board.hole(r,c)+Eigen::Vector2d(-.05,0),false});
        const auto ambiguous=tray_registration::acquire(scene,target,cfg);
        std::cout<<"ambiguous: "<<ambiguous.reason<<" "<<ambiguous.detail<<'\n';
        check(!ambiguous.valid && ambiguous.reason=="tray_acquire_ambiguous_grid","periodic grid chose an arbitrary row/column");
        const Eigen::Vector2d physical_goal=target.board.hole(3,3)+Eigen::Vector2d(-.05,0);
        const auto executable=tray_registration::acquire(scene,target,cfg,[&](const Eigen::Vector2d &p) {
            return (p-physical_goal).norm()<.025;
        });
        check(executable.valid && (executable.fit.target-physical_goal).norm()<1e-5,
              "existing executable TCP envelope did not resolve a periodic visual ambiguity");
        scene.edges.push_back({{.175,-.20},{.175,.20},{-1,0}});
        const auto anchored=tray_registration::acquire(scene,target,cfg);
        check(anchored.valid && (anchored.fit.target-target.board.hole(3,3)-Eigen::Vector2d(-.05,0)).norm()<1e-5,
              "visible tray boundary did not resolve sparse absolute row identity");
        tray_registration::Scene corner;
        for(int c=1;c<=2;++c) corner.rims.push_back({target.board.hole(2,c)+Eigen::Vector2d(-.05,.015),false});
        corner.edges={{{.175,-.21},{.175,.24},{-1,0}},{{-.25,-.21},{.175,-.21},{0,1}}};
        const auto two_at_corner=tray_registration::acquire(corner,target,cfg);
        check(two_at_corner.valid && (two_at_corner.fit.target-target.board.hole(3,3)-Eigen::Vector2d(-.05,.015)).norm()<1e-5,
              "two known rims plus two visible boundaries still required many holes");
        corner.edges.clear();
        const auto two_executable=tray_registration::acquire(corner,target,cfg,[&](const Eigen::Vector2d &p) {
            return (p-target.board.hole(3,3)-Eigen::Vector2d(-.05,.015)).norm()<.025;
        });
        check(two_executable.valid,"two complete rims plus an unambiguous executable target were discarded");
        TrayLocalizer blocked;blocked.reset(seed,cfg);
        const auto unexecutable=blocked.locate(f,camera,[](const Eigen::Vector2d &){return false;});
        check(!unexecutable.valid && unexecutable.detail.find("reject_task_envelope=")!=std::string::npos,
              "unexecutable visual target was locked instead of triggering recovery");
        // Losing features then seeing them again must reinitialize, not remain stuck in identity.
        localizer.reset(seed,cfg);out=localizer.locate(f,camera);
        for(int i=0;i<3;++i) {
            auto blank=image(target,camera,{999},false,3+i*.1);out=localizer.locate(blank,camera);
            check(!out.valid,"blank tray produced fresh localization");
        }
        bool recovered=false;
        for(int i=0;i<4;++i) {auto full=image(target,camera,{},false,4+i*.1);out=localizer.locate(full,camera);recovered|=out.valid;}
        check(recovered,"lost tracking could not reacquire");
        // An occupied hole can contribute the remaining outer orange half-rim.
        auto occupied_seed=seed;
        for(int r=1;r<=6;++r) for(int c=1;c<=6;++c) occupied_seed.occupied_holes.push_back(r*100+c);
        localizer.reset(occupied_seed,cfg);f=image(target,camera,{},false,5);out=localizer.locate(f,camera);
        check(out.valid,"partial-rim fixture initialization failed");
        check(out.occupied_used>=4,"nearly full tray discarded usable orange rims");
        auto arcs=image(target,camera,{302,304},true,5.1);out=localizer.locate(arcs,camera);
        std::cout<<"arcs: "<<out.reason<<" neighbors="<<out.neighbors<<" partial="<<out.partial_used<<'\n';
        check(out.valid && out.partial_used>=2 && (out.pixel-pixel(target.board.hole(3,3),camera)).norm()<3,
              "constrained half-rim recovery failed");
        cv::Mat short_arc(140,140,CV_8UC1,cv::Scalar(0));
        cv::ellipse(short_arc,{70,70},{51,51},0,0,80,cv::Scalar(255),12);
        std::vector<std::vector<cv::Point>> contours;cv::findContours(short_arc,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_NONE);
        cv::Point2d unsupported;
        check(!tray_registration::fit_partial_rim(contours.front(),short_arc,2000,.019,cfg,unsupported),
              "a short ambiguous arc became a reliable center");
        // Left/right sessions must not share a learned image correction.
        TrayLocalizer other;other.reset(seed,cfg);
        check(!other.locate(one,camera).valid,"registration leaked between hands");
        std::cout<<"tray acquisition: large bias, identity ambiguity, sparse history, partial rims and reacquisition passed\n";
    } catch(const std::exception &e) {std::cerr<<"tray acquisition FAIL: "<<e.what()<<'\n';return 1;}
}
