#include "t170/vision/calibration_hole.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <stdexcept>
void check(bool b,const std::string &s){if(!b)throw std::runtime_error(s);}
int main() {
 try {
    CameraFrameData f;f.ok=true;f.width=1280;f.height=720;f.K={650,0,640,0,655,360,0,0,1};
    f.dist={-.05,.04,.0004,-.001,0};f.color_bgr.resize(1280*720*3);
    cv::Mat image(f.height,f.width,CV_8UC3,f.color_bgr.data());image.setTo(cv::Scalar(25,25,25));
    Eigen::Matrix4d camera=Eigen::Matrix4d::Identity();
    camera.block<3,3>(0,0)=Eigen::AngleAxisd(.10,Eigen::Vector3d::UnitY()).toRotationMatrix()*Eigen::Vector3d(1,-1,-1).asDiagonal();
    camera(2,3)=.16;const Eigen::Matrix4d plane=Eigen::Matrix4d::Identity();
    const Eigen::Matrix4d from=camera.inverse()*plane;
    cv::Mat K(3,3,CV_64F,f.K.data()),dist(1,5,CV_64F,f.dist.data());
    auto projected=[&](double radius) {
        std::vector<cv::Point3d> points;
        for(int i=0;i<200;++i) {
            const double angle=i*2*CV_PI/200;
            const Eigen::Vector4d p=from*Eigen::Vector4d(.02+radius*std::cos(angle),.01+radius*std::sin(angle),0,1);
            points.emplace_back(p.x(),p.y(),p.z());
        }
        std::vector<cv::Point2d> out;cv::projectPoints(points,cv::Vec3d(),cv::Vec3d(),K,dist,out);
        std::vector<cv::Point> pixels;for(auto p:out)pixels.emplace_back(cvRound(p.x),cvRound(p.y));return pixels;
    };
    cv::fillConvexPoly(image,projected(.022),cv::Scalar(0,125,255));
    cv::fillConvexPoly(image,projected(.016),cv::Scalar(25,25,25));
    const auto true_center=projected(0)[0];
    const auto result=vision::refine_calibration_hole(f,camera,plane,.019,true_center.x+4,true_center.y-3);
    check(result.valid,result.reason);
    check((result.pixel-Eigen::Vector2d(true_center.x,true_center.y)).norm()<1.3,"distorted tilted ring center inaccurate");
    check(result.coverage_bins>=12 && result.residual_rectified_px<2.5,"unreliable rim accepted");
    check(!vision::refine_calibration_hole(f,camera,plane,.019,-1,0).valid,"out-of-image click accepted");
    check(!vision::refine_calibration_hole(f,camera,plane,.019,50,50).valid,"background produced a reference");
    image.setTo(cv::Scalar(25,25,25));
    check(!vision::refine_calibration_hole(f,camera,plane,.019,true_center.x,true_center.y).valid,"no rim produced a reference");
    std::cout<<"calibration visible-rim refinement passed (synthetic image, not physical accuracy)\n";
 } catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
}
