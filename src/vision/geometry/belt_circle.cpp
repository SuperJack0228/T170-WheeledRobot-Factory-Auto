#include "t170/vision/belt_circle.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>
#include <vector>
namespace vision {
BeltCircleResult locate_belt_circle(const CameraFrameData &f,const Eigen::Matrix4d &bc,double z,
                                   const Eigen::Vector2d &predicted,const BeltCircleConfig &c) {
    BeltCircleResult out;const auto start=std::chrono::steady_clock::now();
    auto elapsed=[&]{return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();};
    auto reject=[&](const std::string &why){out.reason=why;out.elapsed_ms=elapsed();return out;};
    std::string error;if(!validate_belt_circle(c,error))return reject(error);
    if(!f.ok||f.width<=0||f.height<=0||f.color_bgr.size()!=std::size_t(f.width)*f.height*3||!bc.allFinite()||!std::isfinite(z)||!predicted.allFinite())return reject("invalid_input");
    if(f.K[0]<=1||f.K[4]<=1||predicted.x()<0||predicted.y()<0||predicted.x()>=f.width||predicted.y()>=f.height)return reject("target_outside_image");
    try {
        cv::Mat K(3,3,CV_64F,const_cast<double*>(f.K.data())),dist(1,5,CV_64F,const_cast<double*>(f.dist.data()));
        std::vector<cv::Point2d> normalized;
        cv::undistortPoints(std::vector<cv::Point2d>{{predicted.x(),predicted.y()}},normalized,K,dist);
        const Eigen::Vector3d ray=bc.block<3,3>(0,0)*Eigen::Vector3d(normalized[0].x,normalized[0].y,1);
        if(std::abs(ray.z())<.15)return reject("ray_parallel_to_top");
        const double depth=(z-bc(2,3))/ray.z();if(depth<.015||depth>.8)return reject("top_plane_out_of_range");
        const Eigen::Vector3d origin=bc.block<3,1>(0,3)+depth*ray;
        const double half=c.search_radius_m+c.max_radius_m+.003;
        const int size=int(std::ceil(2*half*c.pixels_per_m));if(size<32||size>512)return reject("rectified_roi_too_large");
        const Eigen::Matrix4d cb=bc.inverse();std::vector<cv::Point3d> points;points.reserve(size*size);
        for(int y=0;y<size;++y)for(int x=0;x<size;++x){
            const Eigen::Vector4d p=cb*Eigen::Vector4d(origin.x()+(x-size*.5)/c.pixels_per_m,origin.y()+(y-size*.5)/c.pixels_per_m,z,1);
            points.emplace_back(p.x(),p.y(),p.z());
        }
        std::vector<cv::Point2d> image_points;cv::projectPoints(points,cv::Vec3d(),cv::Vec3d(),K,dist,image_points);
        cv::Mat mx(size,size,CV_32F),my(size,size,CV_32F),valid(size,size,CV_8U);
        for(int i=0;i<size*size;++i){const auto &p=image_points[i];mx.at<float>(i/size,i%size)=float(p.x);my.at<float>(i/size,i%size)=float(p.y);
            valid.at<unsigned char>(i/size,i%size)=points[i].z>0&&p.x>=2&&p.y>=2&&p.x<f.width-2&&p.y<f.height-2?255:0;}
        cv::Mat raw(f.height,f.width,CV_8UC3,const_cast<unsigned char*>(f.color_bgr.data())),rect,gray,edges,saturated;
        cv::remap(raw,rect,mx,my,cv::INTER_LINEAR,cv::BORDER_CONSTANT);
        cv::cvtColor(rect,gray,cv::COLOR_BGR2GRAY);
        // Exclude clipped highlights and their boundaries; do not fit the highlight itself.
        cv::inRange(rect,cv::Scalar(c.saturation_threshold,c.saturation_threshold,c.saturation_threshold),cv::Scalar(255,255,255),saturated);
        out.saturated_fraction=double(cv::countNonZero(saturated))/double(size*size);
        cv::dilate(saturated,saturated,cv::getStructuringElement(cv::MORPH_ELLIPSE,{5,5}));
        cv::GaussianBlur(gray,gray,{3,3},.7);cv::Canny(gray,edges,35,100);
        cv::erode(valid,valid,cv::getStructuringElement(cv::MORPH_RECT,{5,5}));edges.setTo(0,saturated);cv::bitwise_and(edges,valid,edges);
        std::vector<cv::Point> found;cv::findNonZero(edges,found);
        std::vector<Eigen::Vector2d> pts;pts.reserve(found.size());
        for(auto p:found){Eigen::Vector2d v((p.x-size*.5)/c.pixels_per_m,(p.y-size*.5)/c.pixels_per_m);if(v.norm()<=half-.002)pts.push_back(v);}
        if(pts.size()<std::size_t(c.min_inliers))return reject("insufficient_non_glare_edges");
        // Bound the RANSAC cost even in very textured scenes.
        if(pts.size()>1800){std::vector<Eigen::Vector2d> reduced;for(std::size_t i=0;i<1800;++i)reduced.push_back(pts[i*pts.size()/1800]);pts.swap(reduced);}
        struct Candidate {Eigen::Vector2d center;double r=0,residual=0,score=0;int bins=0,n=0;};
        std::vector<Candidate> candidates;
        auto in_bounds=[&](const Eigen::Vector2d &center,double radius){return center.allFinite()&&std::isfinite(radius)&&center.norm()<=c.search_radius_m&&radius>=c.min_radius_m&&radius<=c.max_radius_m&&
             (c.radius_m==0||std::abs(radius-c.radius_m)<=c.radius_tolerance*c.radius_m);};
        auto evaluate=[&](Candidate &candidate,bool refine){
            std::vector<Eigen::Vector2d> inliers;
            for(const auto &p:pts)if(std::abs((p-candidate.center).norm()-candidate.r)<=c.residual_m)inliers.push_back(p);
            if(inliers.size()<std::size_t(c.min_inliers))return false;
            if(refine){Eigen::Matrix3d A=Eigen::Matrix3d::Zero();Eigen::Vector3d b=Eigen::Vector3d::Zero();
                for(const auto &p:inliers){const Eigen::Vector3d v(2*p.x(),2*p.y(),1);A+=v*v.transpose();b+=v*p.squaredNorm();}
                Eigen::FullPivLU<Eigen::Matrix3d> solve(A);if(!solve.isInvertible())return false;
                const Eigen::Vector3d q=solve.solve(b);candidate.center=q.head<2>();candidate.r=std::sqrt(q.z()+candidate.center.squaredNorm());
                if(!in_bounds(candidate.center,candidate.r))return false;
            }
            bool bins[16]{};double residual=0;int n=0;
            for(const auto &p:pts){const auto d=p-candidate.center;const double e=std::abs(d.norm()-candidate.r);
                if(e>c.residual_m)continue;residual+=e;++n;const int bin=std::clamp(int((std::atan2(d.y(),d.x())+M_PI)*16/(2*M_PI)),0,15);bins[bin]=true;}
            candidate.n=n;candidate.bins=std::count(std::begin(bins),std::end(bins),true);candidate.residual=n?residual/n:1;
            if(n<c.min_inliers||candidate.bins<c.min_coverage_bins||candidate.residual>.75*c.residual_m)return false;
            candidate.score=candidate.bins*10+std::min(1.,double(n)/(2*M_PI*candidate.r*c.pixels_per_m))*15-candidate.residual/c.residual_m*10;
            return true;
        };
        std::mt19937 rng(1729);std::uniform_int_distribution<std::size_t> pick(0,pts.size()-1);
        for(int trial=0;trial<c.ransac_trials;++trial){if(elapsed()>.80*c.max_time_ms)break;
            const Eigen::Vector2d a=pts[pick(rng)],b=pts[pick(rng)],d=pts[pick(rng)];Eigen::Matrix2d A;
            A.row(0)=2*(b-a).transpose();A.row(1)=2*(d-a).transpose();if(std::abs(A.determinant())<1e-7)continue;
            Candidate v;v.center=A.inverse()*Eigen::Vector2d(b.squaredNorm()-a.squaredNorm(),d.squaredNorm()-a.squaredNorm());v.r=(a-v.center).norm();
            if(in_bounds(v.center,v.r)&&evaluate(v,true))candidates.push_back(v);
        }
        if(candidates.empty())return reject(elapsed()>c.max_time_ms?"circle_budget_no_solution":"no_supported_circle");
        std::sort(candidates.begin(),candidates.end(),[](const auto &a,const auto &b){return a.score>b.score;});const auto &best=candidates.front();
        for(const auto &other:candidates)if((other.center-best.center).norm()>.003&&other.score>=best.score*.90)return reject("ambiguous_circle_centers");
        const Eigen::Vector4d point=cb*Eigen::Vector4d(origin.x()+best.center.x(),origin.y()+best.center.y(),z,1);
        std::vector<cv::Point2d> projected;cv::projectPoints(std::vector<cv::Point3d>{{point.x(),point.y(),point.z()}},cv::Vec3d(),cv::Vec3d(),K,dist,projected);
        out.pixel={projected[0].x,projected[0].y};out.center_xy=origin.head<2>()+best.center;
        out.radius_m=best.r;out.inliers=best.n;out.coverage_bins=best.bins;out.residual_m=best.residual;
        if(elapsed()>c.max_time_ms)return reject("circle_processing_budget_exceeded");
        if(out.pixel.x()<0||out.pixel.y()<0||out.pixel.x()>=f.width||out.pixel.y()>=f.height)return reject("fitted_center_outside_image");
        out.valid=true;out.reason="circle_supported";out.elapsed_ms=elapsed();return out;
    }catch(const std::exception &e){return reject(std::string("circle_exception:")+e.what());}
}
}
