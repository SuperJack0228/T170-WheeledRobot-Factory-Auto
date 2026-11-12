#include "t170/vision/belt_circle.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <stdexcept>
void check(bool ok,const std::string &why){if(!ok)throw std::runtime_error(why);}
int main(){try{
    CameraFrameData f;f.ok=true;f.width=1280;f.height=720;f.K={650,0,640,0,655,360,0,0,1};
    f.dist={-.05,.04,.0004,-.001,0};f.color_bgr.resize(1280*720*3);
    cv::Mat image(f.height,f.width,CV_8UC3,f.color_bgr.data());
    Eigen::Matrix4d bc=Eigen::Matrix4d::Identity();bc.block<3,3>(0,0)=Eigen::AngleAxisd(.18,Eigen::Vector3d::UnitY()).toRotationMatrix()*Eigen::Vector3d(1,-1,-1).asDiagonal();bc(2,3)=.17;
    const Eigen::Matrix4d cb=bc.inverse();const Eigen::Vector2d center(.008,-.007);
    cv::Mat K(3,3,CV_64F,f.K.data()),dist(1,5,CV_64F,f.dist.data());
    auto project=[&](Eigen::Vector2d c,double r,double start=0,double arc=2*M_PI){
        std::vector<cv::Point3d> pts;for(int i=0;i<=180;++i){double a=start+arc*i/180;const Eigen::Vector4d p=cb*Eigen::Vector4d(c.x()+r*std::cos(a),c.y()+r*std::sin(a),0,1);pts.emplace_back(p.x(),p.y(),p.z());}
        std::vector<cv::Point2d> projected;cv::projectPoints(pts,cv::Vec3d(),cv::Vec3d(),K,dist,projected);
        std::vector<cv::Point> out;for(auto p:projected)out.emplace_back(cvRound(p.x),cvRound(p.y));return out;
    };
    const auto true_pixel=project(center,0)[0];const Eigen::Vector2d predicted(true_pixel.x+4,true_pixel.y-3);
    vision::BeltCircleConfig c;c.enabled=true;c.radius_m=.0135;c.min_radius_m=.010;c.max_radius_m=.017;c.max_time_ms=100;c.ransac_trials=650;
    image.setTo(cv::Scalar(25,25,25));cv::fillConvexPoly(image,project(center,.0135),cv::Scalar(160,160,160));
    // A concentric dark hole and a clipped, quarter-disc specular highlight.
    cv::fillConvexPoly(image,project(center,.006),cv::Scalar(25,25,25));
    auto glare=project(center,.014,0,M_PI*.5);glare.push_back(true_pixel);cv::fillConvexPoly(image,glare,cv::Scalar(255,255,255));
    auto result=vision::locate_belt_circle(f,bc,0,predicted,c);
    check(result.valid,result.reason);check((result.pixel-Eigen::Vector2d(true_pixel.x,true_pixel.y)).norm()<2.0,"partial highlight moved physical circle center");
    check(std::abs(result.radius_m-.0135)<.0015,"inner hole mistaken for the outer circle");
    image.setTo(cv::Scalar(25,25,25));check(!vision::locate_belt_circle(f,bc,0,predicted,c).valid,"blank background accepted");
    image.setTo(cv::Scalar(255,255,255));check(!vision::locate_belt_circle(f,bc,0,predicted,c).valid,"fully clipped image accepted");
    image.setTo(cv::Scalar(25,25,25));cv::polylines(image,std::vector<std::vector<cv::Point>>{project(center,.0135,0,M_PI/3)},false,cv::Scalar(160,160,160),2);
    check(!vision::locate_belt_circle(f,bc,0,predicted,c).valid,"short arc produced a fake confident center");
    image.setTo(cv::Scalar(25,25,25));
    cv::polylines(image,std::vector<std::vector<cv::Point>>{project(center+Eigen::Vector2d(-.013,0),.0135),project(center+Eigen::Vector2d(.013,0),.0135)},false,cv::Scalar(160,160,160),1);
    check(!vision::locate_belt_circle(f,bc,0,predicted,c).valid,"ambiguous nearby circles accepted");
    std::string error;c.min_coverage_bins=1;check(!vision::validate_belt_circle(c,error),"unsafe arc coverage accepted");
    std::cout<<"RGB circle fallback: distortion, perspective, glare, inner hole and rejection checks passed (synthetic)\n";
}catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}}
