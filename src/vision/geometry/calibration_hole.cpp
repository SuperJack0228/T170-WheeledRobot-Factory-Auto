#include "t170/vision/calibration_hole.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace vision {
CalibrationHole refine_calibration_hole(const CameraFrameData &frame,const Eigen::Matrix4d &base_camera,
    const Eigen::Matrix4d &base_tray,double radius_m,double u,double v) {
    CalibrationHole result;
    auto require=[](bool v,const char *why) {if(!v)throw std::runtime_error(why);};
    try {
        require(frame.ok && frame.width>0 && frame.height>0 && frame.color_bgr.size()==std::size_t(frame.width)*frame.height*3,"invalid frame");
        require(base_camera.allFinite() && base_tray.allFinite() && std::isfinite(radius_m) && radius_m>0,"invalid geometry");
        auto frozen=frame;
        require(std::isfinite(u) && std::isfinite(v) && u>=0 && v>=0 && u<frozen.width && v<frozen.height,"点击超出图像");
        cv::Mat K(3,3,CV_64F,frozen.K.data()),dist(1,5,CV_64F,frozen.dist.data());
        cv::Mat raw(frozen.height,frozen.width,CV_8UC3,frozen.color_bgr.data()),undistorted;cv::undistort(raw,undistorted,K,dist);
        const Eigen::Matrix4d ct=base_camera.inverse()*base_tray;
        Eigen::Matrix3d H;H.col(0)=ct.block<3,1>(0,0);H.col(1)=ct.block<3,1>(0,1);H.col(2)=ct.block<3,1>(0,3);
        Eigen::Matrix3d k;for(int i=0;i<3;++i)for(int j=0;j<3;++j)k(i,j)=frozen.K[i*3+j];H=k*H;
        require(k.allFinite() && k(0,0)>1 && k(1,1)>1 && std::abs(H.determinant())>1e-10,"invalid camera/plane projection");
        std::vector<cv::Point2d> clicked,normal;clicked.emplace_back(u,v);cv::undistortPoints(clicked,normal,K,dist,cv::noArray(),K);
        Eigen::Vector3d center=H.inverse()*Eigen::Vector3d(normal[0].x,normal[0].y,1);center/=center.z();
        Eigen::Matrix3d M=Eigen::Matrix3d::Identity();M(0,0)=M(1,1)=2000;M(0,2)=80-2000*center.x();M(1,2)=80-2000*center.y();
        const Eigen::Matrix3d warp=M*H.inverse();cv::Mat w(3,3,CV_64F);for(int i=0;i<3;++i)for(int j=0;j<3;++j)w.at<double>(i,j)=warp(i,j);
        cv::Mat rect,hsv,mask;cv::warpPerspective(undistorted,rect,w,{160,160});cv::cvtColor(rect,hsv,cv::COLOR_BGR2HSV);
        cv::inRange(hsv,cv::Scalar(0,75,65),cv::Scalar(28,255,255),mask);
        std::vector<std::vector<cv::Point>> contours;cv::findContours(mask,contours,cv::RETR_LIST,cv::CHAIN_APPROX_NONE);
        cv::Point2f best;double distance=INFINITY;
        for(const auto &contour:contours) if(contour.size()>=25) {
            const auto e=cv::fitEllipse(contour);const double lo=std::min(e.size.width,e.size.height),hi=std::max(e.size.width,e.size.height);
            const double radius=.25*(lo+hi),expected=radius_m*2000;
            const double d=cv::norm(e.center-cv::Point2f(80,80));
            if(lo<=0 || hi<=0) continue;
            const double angle=e.angle*CV_PI/180,cs=std::cos(angle),sn=std::sin(angle);
            bool bins[16]{};double residual=0;
            for(const auto &point:contour) {
                const double dx=point.x-e.center.x,dy=point.y-e.center.y;
                const double x=(cs*dx+sn*dy)/(e.size.width*.5),y=(-sn*dx+cs*dy)/(e.size.height*.5);
                residual+=std::abs(std::hypot(x,y)-1)*radius;
                const int bin=std::clamp(int((std::atan2(y,x)+CV_PI)/(2*CV_PI)*16),0,15);bins[bin]=true;
            }
            residual/=contour.size();const int covered=std::count(std::begin(bins),std::end(bins),true);
            if(covered<12 || residual>2.5)continue;
            if(lo/hi>.65 && radius>.55*expected && radius<1.6*expected && d<30 && d<distance) {best=e.center;distance=d;result.coverage_bins=covered;result.residual_rectified_px=residual;}
        }
        require(std::isfinite(distance),"点击附近没有可靠的橙色孔环，请点目标孔中心附近或重新拍照");
        Eigen::Vector3d p=H*M.inverse()*Eigen::Vector3d(best.x,best.y,1);p/=p.z();
        Eigen::Vector3d ray=k.inverse()*p;std::vector<cv::Point2d> output;
        cv::projectPoints(std::vector<cv::Point3d>{{ray.x(),ray.y(),1}},cv::Vec3d(0,0,0),cv::Vec3d(0,0,0),K,dist,output);
        result.pixel={output[0].x,output[0].y};result.valid=true;result.reason="ok";
    } catch(const std::exception &e) {result.reason=e.what();}
    return result;
}
} // namespace vision
