#include "tray_registration.h"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <random>
#include <set>
#include <sstream>
#include <tuple>

namespace wrist_servo::tray_registration {
namespace {
Eigen::Vector2d project(const Eigen::Matrix3d &h,const Eigen::Vector2d &p) {
    const Eigen::Vector3d q=h*Eigen::Vector3d(p.x(),p.y(),1);
    return q.head<2>()/q.z();
}
cv::Mat matrix(const Eigen::Matrix3d &h) {
    cv::Mat a(3,3,CV_64F);
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) a.at<double>(i,j)=h(i,j);
    return a;
}
bool on(const cv::Mat &mask,const cv::Point2d &p) {
    const int x=cvRound(p.x),y=cvRound(p.y);
    return x>=0 && y>=0 && x<mask.cols && y<mask.rows && mask.at<unsigned char>(y,x)>0;
}
struct Pair { int grid=0,rim=0; double distance=0; };
struct Hypothesis {
    Eigen::Matrix3d transform=Eigen::Matrix3d::Identity();
    std::vector<Pair> pairs;
    double score=-INFINITY;
    int outside=0;
};
// Only bright background OUTSIDE the conservative tray hull is negative evidence.
// Reflective parts inside the hull and points cropped by the image are not negatives.
int outside_tray(const Scene &s,const Eigen::Matrix3d &a,const std::vector<Eigen::Vector2d> &grid,double radius) {
    int outside=0;
    for(const auto &p:grid) {
        const auto xy=project(a,p),uv=project(s.seed,xy);
        bool across_edge=false;
        for(const auto &edge:s.edges) {
            const Eigen::Vector2d d=edge.end-edge.begin;const double t=(xy-edge.begin).dot(d)/d.squaredNorm();
            if(t>=0 && t<=1 && (xy-edge.begin).dot(edge.inward)<-radius*.3) {across_edge=true;break;}
        }
        if(across_edge) {++outside;continue;}
        if(s.tray_hull.size()<3 || s.hsv.empty()) continue;
        const double r=(project(s.seed,xy+Eigen::Vector2d(radius,0))-uv).norm();
        if(!uv.allFinite() || !std::isfinite(r) || r<1 || uv.x()<r || uv.y()<r || uv.x()>=s.hsv.cols-r || uv.y()>=s.hsv.rows-r) continue;
        if(cv::pointPolygonTest(s.tray_hull,cv::Point2f(uv.x(),uv.y()),true)>=-r*.3) continue;
        int bright=0;
        for(const auto &d:std::array<cv::Point2d,5>{{{0,0},{.5,0},{-.5,0},{0,.5},{0,-.5}}}) {
            const auto v=s.hsv.at<cv::Vec3b>(cvRound(uv.y()+r*d.y),cvRound(uv.x()+r*d.x));
            if(v[1]<50 && v[2]>205) ++bright;
        }
        if(bright==5) ++outside;
    }
    return outside;
}
Hypothesis evaluate(const Eigen::Matrix3d &a,const Scene &s,const std::vector<Eigen::Vector2d> &grid,
                    const std::vector<Eigen::Vector2d> &whole,const TrayPlaceVisionConfig &cfg,double radius) {
    Hypothesis out;out.transform=a;
    std::vector<Pair> nearest;
    for(int g=0;g<static_cast<int>(grid.size());++g) {
        const auto p=project(a,grid[g]);int best=-1;double distance=cfg.global_match_gate_m;
        for(int i=0;i<static_cast<int>(s.rims.size());++i) {
            const double d=(p-s.rims[i].point).norm();
            if(d<distance) {best=i;distance=d;}
        }
        if(best>=0) nearest.push_back({g,best,distance});
    }
    std::sort(nearest.begin(),nearest.end(),[](const Pair &a,const Pair &b){return a.distance<b.distance;});
    std::set<int> used;
    out.score=0;
    for(const auto &pair:nearest) if(used.insert(pair.rim).second) {
        out.pairs.push_back(pair);
        out.score+=(s.rims[pair.rim].partial?.65:1.0)-.15*pair.distance/cfg.global_match_gate_m;
    }
    out.outside=outside_tray(s,a,whole,radius);
    out.score-=2.0*out.outside;
    return out;
}
}

bool fit_partial_rim(const std::vector<cv::Point> &contour,const cv::Mat &mask,double scale,
                     double radius,const TrayPlaceVisionConfig &cfg,cv::Point2d &center) {
    if(!cfg.partial_rims || contour.size()<30 || scale<=0) return false;
    std::vector<Eigen::Vector2d> points;
    const int stride=std::max(1,static_cast<int>(contour.size()/160));
    for(int i=0;i<static_cast<int>(contour.size());i+=stride) points.emplace_back(contour[i].x/scale,contour[i].y/scale);
    std::mt19937 random(170);int best=0;Eigen::Vector2d chosen;double chosen_radius=0;
    auto support=[&](const Eigen::Vector2d &c,double r,std::vector<int> *ids) {
        std::array<bool,16> bins{};int count=0;
        for(int i=0;i<static_cast<int>(points.size());++i) {
            const auto delta=(points[i]-c).eval();const double length=delta.norm();
            if(length<1e-8 || std::abs(length-r)>cfg.circle_residual_m) continue;
            const cv::Point2d p(points[i].x()*scale,points[i].y()*scale);
            const cv::Point2d direction(delta.x()/length,delta.y()/length);
            // Inner ring and artificial cut edges have the opposite/no orange transition.
            if(!on(mask,p-direction*1.5) || on(mask,p+direction*2.0)) continue;
            const int bin=std::clamp(int((std::atan2(delta.y(),delta.x())+std::acos(-1))*8/std::acos(-1)),0,15);
            bins[bin]=true;++count;if(ids) ids->push_back(i);
        }
        return std::count(bins.begin(),bins.end(),true)>=cfg.partial_min_bins?count:0;
    };
    for(int k=0;k<96;++k) {
        const auto &p=points[random()%points.size()],&q=points[random()%points.size()],&r=points[random()%points.size()];
        Eigen::Matrix2d a;a.row(0)=2*(q-p).transpose();a.row(1)=2*(r-p).transpose();
        if(std::abs(a.determinant())<1e-6) continue;
        const Eigen::Vector2d c=a.inverse()*Eigen::Vector2d(q.squaredNorm()-p.squaredNorm(),r.squaredNorm()-p.squaredNorm());
        const double rad=(p-c).norm();
        if(rad<radius*.9 || rad>radius*cfg.rim_max_radius_ratio) continue;
        const int n=support(c,rad,nullptr);
        if(n>best) {best=n;chosen=c;chosen_radius=rad;}
    }
    if(best<12 || best<static_cast<int>(points.size()*.20)) return false;
    std::vector<int> ids;support(chosen,chosen_radius,&ids);
    Eigen::MatrixXd a(ids.size(),3);Eigen::VectorXd b(ids.size());
    for(int i=0;i<static_cast<int>(ids.size());++i) {
        const auto &p=points[ids[i]];a.row(i)<<2*p.x(),2*p.y(),1;b(i)=p.squaredNorm();
    }
    const Eigen::Vector3d x=a.colPivHouseholderQr().solve(b);
    const double rad=std::sqrt(std::max(0.0,x.z()+x.head<2>().squaredNorm()));
    if(!x.allFinite() || rad<radius*.9 || rad>radius*cfg.rim_max_radius_ratio || support(x.head<2>(),rad,nullptr)<best*.8) return false;
    center={x.x()*scale,x.y()*scale};return true;
}

Scene discover(const cv::Mat &bgr,const Eigen::Matrix3d &seed,const TrayBoardGeometry &board,const TrayPlaceVisionConfig &cfg) {
    Scene out;out.seed=seed;cv::cvtColor(bgr,out.hsv,cv::COLOR_BGR2HSV);
    cv::Mat dark;cv::inRange(out.hsv,cv::Scalar(0,0,0),cv::Scalar(180,255,180),dark);
    cv::morphologyEx(dark,dark,cv::MORPH_CLOSE,cv::getStructuringElement(cv::MORPH_ELLIPSE,{9,9}));
    std::vector<std::vector<cv::Point>> domains;cv::findContours(dark,domains,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
    auto largest=std::max_element(domains.begin(),domains.end(),[](const auto &a,const auto &b){return cv::contourArea(a)<cv::contourArea(b);});
    if(cfg.boundary_enabled && largest!=domains.end() && cv::contourArea(*largest)>.08*bgr.total() && cv::contourArea(*largest)<.95*bgr.total())
        cv::convexHull(*largest,out.tray_hull);
    const double scale=std::min(1200.0,cfg.rectified_pixels_per_m);
    const double margin=cfg.global_search_m+board.radius_m*cfg.rim_max_radius_ratio;
    const Eigen::Vector2d low=board.hole(board.rows,1)-Eigen::Vector2d::Constant(margin);
    const Eigen::Vector2d high=board.hole(1,board.cols)+Eigen::Vector2d::Constant(margin);
    const int width=int(std::ceil((high.x()-low.x())*scale))+1,height=int(std::ceil((high.y()-low.y())*scale))+1;
    if(width<1 || height<1 || width>2048 || height>2048) return out;
    Eigen::Matrix3d plane_from_patch=Eigen::Matrix3d::Identity();
    plane_from_patch(0,0)=plane_from_patch(1,1)=1/scale;plane_from_patch.block<2,1>(0,2)=low;
    cv::Mat rectified,hsv,mask;
    cv::warpPerspective(bgr,rectified,matrix(seed*plane_from_patch),{width,height},cv::INTER_LINEAR|cv::WARP_INVERSE_MAP,cv::BORDER_CONSTANT,cv::Scalar(127,127,127));
    cv::cvtColor(rectified,hsv,cv::COLOR_BGR2HSV);cv::inRange(hsv,cv::Scalar(0,85,65),cv::Scalar(28,255,255),mask);
    cv::morphologyEx(mask,mask,cv::MORPH_CLOSE,cv::getStructuringElement(cv::MORPH_ELLIPSE,{3,3}));
    std::vector<std::vector<cv::Point>> contours;cv::findContours(mask,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_NONE);
    // Complete rims first, so a duplicate partial fit cannot replace a complete one.
    std::vector<Rim> partial;
    for(const auto &contour:contours) {
        if(contour.size()<30) continue;
        const auto e=cv::fitEllipse(contour);
        const double ra=e.size.width*.5/scale,rb=e.size.height*.5/scale;
        std::array<bool,16> bins{};double error=0;
        const double angle=e.angle*std::acos(-1)/180;
        for(const auto &p:contour) {
            const double dx=(p.x-e.center.x)/scale,dy=(p.y-e.center.y)/scale;
            const double x=(std::cos(angle)*dx+std::sin(angle)*dy)/std::max(ra,1e-9);
            const double y=(-std::sin(angle)*dx+std::cos(angle)*dy)/std::max(rb,1e-9);
            error+=std::pow((std::hypot(x,y)-1)*(ra+rb)*.5,2);
            bins[std::clamp(int((std::atan2(y,x)+std::acos(-1))*8/std::acos(-1)),0,15)]=true;
        }
        const bool full=std::min(ra,rb)>board.radius_m*cfg.rim_min_radius_ratio &&
            std::max(ra,rb)<board.radius_m*cfg.rim_max_radius_ratio &&
            std::min(ra,rb)/std::max(ra,rb)>.5 && std::sqrt(error/contour.size())<cfg.circle_residual_m*1.5 &&
            std::count(bins.begin(),bins.end(),true)>=cfg.min_coverage_bins;
        if(full) out.rims.push_back({low+Eigen::Vector2d(e.center.x/scale,e.center.y/scale),false});
        else {
            cv::Point2d center;
            if(fit_partial_rim(contour,mask,scale,board.radius_m,cfg,center))
                partial.push_back({low+Eigen::Vector2d(center.x/scale,center.y/scale),true});
        }
    }
    out.rims.insert(out.rims.end(),partial.begin(),partial.end());
    std::vector<Rim> distinct;
    for(const auto &r:out.rims) {
        if(!r.point.allFinite()) continue;
        if(std::none_of(distinct.begin(),distinct.end(),[&](const Rim &v){return (v.point-r.point).norm()<board.radius_m*.6;})) distinct.push_back(r);
        if(distinct.size()>=64) break;
    }
    out.rims=std::move(distinct);
    // A long dark-to-bright boundary, aligned with a tray axis and containing
    // nearly all observed rims on its dark side, supplies absolute grid evidence.
    // We do not assume a plate size or require a marker. Finite segments are not
    // extrapolated beyond what this image actually observes.
    if(cfg.boundary_enabled && out.rims.size()>=2) {
        cv::Mat white,edge_image;
        cv::inRange(out.hsv,cv::Scalar(0,0,205),cv::Scalar(180,50,255),white);
        cv::Canny(white,edge_image,50,120);
        std::vector<cv::Vec4i> lines;cv::HoughLinesP(edge_image,lines,1,std::acos(-1)/180,70,100,12);
        const Eigen::Matrix3d inverse=seed.inverse();
        std::sort(lines.begin(),lines.end(),[](const auto &a,const auto &b) {
            return std::hypot(a[2]-a[0],a[3]-a[1])>std::hypot(b[2]-b[0],b[3]-b[1]);
        });
        for(const auto &line:lines) {
            const auto begin=project(inverse,{double(line[0]),double(line[1])});
            const auto end=project(inverse,{double(line[2]),double(line[3])});
            const Eigen::Vector2d d=end-begin;
            if(!begin.allFinite() || !end.allFinite() || d.norm()<board.pitch_m*(out.rims.size()<4?1.5:1.2) ||
               std::min(std::abs(d.x()),std::abs(d.y()))/d.norm()>.20) continue;
            const Eigen::Vector2d normal(-d.y()/d.norm(),d.x()/d.norm());
            int positive=0,negative=0;
            for(double t:{.15,.325,.5,.675,.85}) {
                auto sample=[&](double sign) {
                    const auto uv=project(seed,(begin+t*d+sign*.008*normal).eval());
                    if(!uv.allFinite() || uv.x()<0 || uv.y()<0 || uv.x()>=bgr.cols || uv.y()>=bgr.rows) return -1;
                    const auto v=out.hsv.at<cv::Vec3b>(cvRound(std::clamp(uv.y(),0.0,double(bgr.rows-1))),cvRound(std::clamp(uv.x(),0.0,double(bgr.cols-1))));
                    return v[2]<180?0:(v[1]<50 && v[2]>205?1:-1);
                };
                const int plus=sample(1),minus=sample(-1);
                if(plus==0 && minus==1) ++positive;
                if(plus==1 && minus==0) ++negative;
            }
            if(std::max(positive,negative)<(out.rims.size()<4?5:4)) continue;
            const Eigen::Vector2d inward=positive>negative?normal:(-normal).eval();
            int contained=0;
            for(const auto &rim:out.rims) if((rim.point-begin).dot(inward)>-board.radius_m*.3) ++contained;
            if(contained<std::ceil(out.rims.size()*.9)) continue;
            out.edges.push_back({begin,end,inward});
            if(out.edges.size()>=8) break;
        }
    }
    return out;
}

Result acquire(const Scene &scene,const TrayTarget &target,const TrayPlaceVisionConfig &cfg,
               const TrayPixelGate &executable_target) {
    Result out;out.reason="tray_acquire_no_rims";
    if(scene.rims.size()<static_cast<std::size_t>(cfg.min_neighbors)) {
        if(!scene.rims.empty()) out.reason="tray_acquire_insufficient_rims";
        out.detail="candidates="+std::to_string(scene.rims.size())+" minimum="+std::to_string(cfg.min_neighbors);return out;
    }
    const auto &board=target.board;const auto goal=board.hole(target.row,target.col);
    std::vector<Eigen::Vector2d> grid,whole;
    for(int r=1;r<=board.rows;++r) for(int c=1;c<=board.cols;++c) {
        whole.push_back(board.hole(r,c));
        if(r!=target.row || c!=target.col) grid.push_back(board.hole(r,c));
    }
    std::vector<Hypothesis> top;
    std::set<std::tuple<int,int,int,int>> visited;
    bool exhausted=false;
    auto consider=[&](const Eigen::Matrix2d &linear,const Eigen::Vector2d &shift) {
        if(out.hypotheses>=cfg.global_max_hypotheses) {exhausted=true;return;}
        const double scale=std::sqrt(linear.determinant()),angle=std::atan2(linear(1,0),linear(0,0));
        if(!std::isfinite(scale) || scale<cfg.affine_min_scale || scale>cfg.affine_max_scale || std::abs(angle)>cfg.affine_max_rotation_rad ||
           (linear*goal+shift-goal).norm()>=cfg.global_search_m) return;
        const auto key=std::make_tuple(int(std::round(shift.x()/.002)),int(std::round(shift.y()/.002)),int(std::round(scale/.025)),int(std::round(angle/.025)));
        if(!visited.insert(key).second) return;
        ++out.hypotheses;Eigen::Matrix3d a=Eigen::Matrix3d::Identity();a.block<2,2>(0,0)=linear;a.block<2,1>(0,2)=shift;
        auto h=evaluate(a,scene,grid,whole,cfg,board.radius_m);
        if(h.pairs.size()<static_cast<std::size_t>(cfg.min_neighbors)) return;
        // Keep alternative absolute grid identities, not 100 near-identical transforms.
        for(auto &old:top) if((project(old.transform,goal)-project(a,goal)).norm()<.012) {
            if(h.score>old.score) old=std::move(h);return;
        }
        top.push_back(std::move(h));
        if(top.size()>24) {auto worst=std::min_element(top.begin(),top.end(),[](const auto &a,const auto &b){return a.score<b.score;});top.erase(worst);}
    };
    // Translation hypotheses are cheap; pair geometry adds scale/rotation without a large parameter grid.
    for(const auto &rim:scene.rims) for(const auto &p:grid) consider(Eigen::Matrix2d::Identity(),rim.point-p);
    for(std::size_t i=0;i<scene.rims.size();++i) for(std::size_t j=i+1;j<scene.rims.size();++j) {
        const Eigen::Vector2d d=scene.rims[j].point-scene.rims[i].point;
        if(d.norm()<board.pitch_m*cfg.affine_min_scale || d.norm()>board.pitch_m*cfg.affine_max_scale) continue;
        for(const Eigen::Vector2d v:std::array<Eigen::Vector2d,4>{{{board.pitch_m,0},{-board.pitch_m,0},{0,board.pitch_m},{0,-board.pitch_m}}}) {
            const double a=d.dot(v)/v.squaredNorm(),b=(v.x()*d.y()-v.y()*d.x())/v.squaredNorm();
            Eigen::Matrix2d linear;linear<<a,-b,b,a;
            for(const auto &p:grid) consider(linear,scene.rims[i].point-linear*p);
        }
    }
    struct Refined {Hypothesis h;NeighborGridFit fit;bool eligible;};std::vector<Refined> refined;
    int rejected_fit=0,rejected_background=0,rejected_task=0;
    bool corner_visible=false;
    for(std::size_t i=0;i<scene.edges.size();++i) for(std::size_t j=i+1;j<scene.edges.size();++j)
        if(std::abs(scene.edges[i].inward.dot(scene.edges[j].inward))<.5) corner_visible=true;
    for(auto &h:top) {
        NeighborGridFit fit;
        for(int iteration=0;iteration<2;++iteration) {
            std::vector<Eigen::Vector2d> from,to;
            for(const auto &p:h.pairs) {from.push_back(grid[p.grid]);to.push_back(scene.rims[p.rim].point);}
            fit=fit_neighbor_grid(from,to,goal,board.pitch_m,cfg,true);
            if(!fit.valid) break;
            h=evaluate(fit.transform,scene,grid,whole,cfg,board.radius_m);
        }
        if(!fit.valid) {++rejected_fit;continue;}
        if(executable_target && !executable_target(fit.target)) {++rejected_task;continue;}
        // The broad acquisition gate proposes correspondences; it must not reward
        // pairs that the final RANSAC model rejected. Score on the consensus gate.
        auto consensus_cfg=cfg;consensus_cfg.global_match_gate_m=2*cfg.circle_residual_m;
        h=evaluate(fit.transform,scene,grid,whole,consensus_cfg,board.radius_m);
        if(h.outside>0) {++rejected_background;continue;}
        int complete=0;for(const auto &p:h.pairs) if(!scene.rims[p.rim].partial) ++complete;
        // A large correction needs redundant evidence. Two partial arcs cannot establish identity.
        const bool eligible=complete>=2 && ((fit.target-goal).norm()<board.pitch_m*.45 || h.pairs.size()>=4 || corner_visible || executable_target);
        // Insufficiently supported alternatives still count as ambiguity evidence.
        refined.push_back({std::move(h),std::move(fit),eligible});
    }
    const std::string diagnostics="candidates="+std::to_string(scene.rims.size())+" hypotheses="+std::to_string(out.hypotheses)+
        " top="+std::to_string(top.size())+" reject_fit="+std::to_string(rejected_fit)+" reject_background="+std::to_string(rejected_background)+
        " reject_task_envelope="+std::to_string(rejected_task);
    if(exhausted) {out.reason="tray_acquire_hypothesis_budget";out.detail=diagnostics;return out;}
    if(refined.empty()) {out.reason="tray_acquire_no_supported_grid";out.detail=diagnostics;return out;}
    std::sort(refined.begin(),refined.end(),[](const auto &a,const auto &b){return a.h.score>b.h.score;});
    const auto &best=refined.front();out.margin=100;
    for(std::size_t i=1;i<refined.size();++i) if((refined[i].fit.target-best.fit.target).norm()>board.pitch_m*.3) {
        out.margin=best.h.score-refined[i].h.score;break;
    }
    std::ostringstream detail;detail<<"candidates="<<scene.rims.size()<<" hypotheses="<<out.hypotheses<<" reject_task_envelope="<<rejected_task
        <<" boundary_edges="<<scene.edges.size()<<" matches="<<best.h.pairs.size()<<" identity_margin="<<out.margin<<" "<<best.fit.detail;
    for(std::size_t i=0;i<std::min<std::size_t>(refined.size(),4);++i)
        detail<<" alternative"<<i<<"_shift_mm="<<1000*(refined[i].fit.target.x()-goal.x())<<":"
              <<1000*(refined[i].fit.target.y()-goal.y())<<"_score="<<refined[i].h.score<<"_support="<<refined[i].h.pairs.size();
    out.detail=detail.str();
    if(out.margin<cfg.global_ambiguity_margin) {out.reason="tray_acquire_ambiguous_grid";return out;}
    if(!best.eligible) {out.reason="tray_acquire_no_supported_grid";return out;}
    out.valid=true;out.fit=best.fit;out.reason="ok";return out;
}
}
