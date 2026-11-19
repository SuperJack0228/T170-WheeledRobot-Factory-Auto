#include "t170/motion/arm.h"
#include "t170/motion/joint_motion.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <iostream>
#include <chrono>
#include <sstream>
using t170::motion::ArmJoints;
Arm::Arm(t170::motion::ArmDefinition d,std::shared_ptr<t170::motion::ArmBackend> b)
    :can_id_list(d.can_ids),definition_(std::move(d)),backend_(std::move(b)) {
    if(!backend_) throw std::invalid_argument("Arm requires an explicit backend");
}
Eigen::Matrix4d Arm::Forward_Kinematics(const Eigen::Ref<const Eigen::RowVectorXd> &q) {
    if(q.size()!=7 || !q.allFinite()) throw std::runtime_error("Invalid MODEL joints");
    const auto t=backend_->forward(ArmJoints(q));
    if(!t.allFinite()) throw std::runtime_error("Vendor FK returned nonfinite transform");
    return t;
}
Eigen::Matrix4d Arm::Flange_Transform(const Eigen::Ref<const Eigen::RowVectorXd> &q) {
    if(q.size()!=7 || !q.allFinite()) throw std::runtime_error("Invalid MODEL joints");
    const auto t=backend_->flange(ArmJoints(q));
    if(!t.allFinite()) throw std::runtime_error("Vendor flange FK returned nonfinite transform");
    return t;
}
namespace {
Eigen::Matrix4d mdh(double alpha,double a,double d,double theta) {
    Eigen::Matrix4d m;
    m<<cos(theta),-sin(theta),0,a,sin(theta)*cos(alpha),cos(theta)*cos(alpha),-sin(alpha),-sin(alpha)*d,
       sin(theta)*sin(alpha),cos(theta)*sin(alpha),cos(alpha),cos(alpha)*d,0,0,0,1;
    return m;
}
}
void Arm::install_device_limits(const ArmJoints &minimum,const ArmJoints &maximum) {
    const auto limits=t170::motion::intersect_device_bounds(definition_,minimum,maximum);
    if(!definition_.limits)definition_.limits=std::make_shared<t170::motion::ArmLimitState>();
    std::lock_guard lock(definition_.limits->mutex);definition_.limits->bounds=limits;
}
bool Arm::Check_Joint_Limit(const Eigen::Ref<const Eigen::RowVectorXd> &q) {
    const auto limits=effective_limits();
    return q.size()!=7 || !q.allFinite() || (q.array()<limits.minimum.array()-1e-8).any() ||
        (q.array()>limits.maximum.array()+1e-8).any();
}
Eigen::Vector3d Arm::elbow_position(const ArmJoints &q) const {
    Eigen::Matrix4d t=Eigen::Matrix4d::Identity();
    for(int i=0;i<4;++i)t*=mdh(definition_.mdh(i,0),definition_.mdh(i,1),definition_.mdh(i,2),q(i)+definition_.mdh(i,3));
    return t.topRightCorner<3,1>();
}
Eigen::MatrixXd Arm::Jacobian(const Eigen::Ref<const Eigen::RowVectorXd> &q) {
    const Eigen::Vector3d tcp=Forward_Kinematics(q).topRightCorner<3,1>();
    Eigen::Matrix<double,6,7> j;Eigen::Matrix4d t=Eigen::Matrix4d::Identity();
    for(int i=0;i<7;++i) {
        const double alpha=definition_.mdh(i,0),a=definition_.mdh(i,1);
        const Eigen::Vector3d origin=t.topRightCorner<3,1>()+t.topLeftCorner<3,3>()*Eigen::Vector3d(a,0,0);
        const Eigen::Vector3d axis=t.topLeftCorner<3,3>()*Eigen::Vector3d(0,-sin(alpha),cos(alpha));
        j.block<3,1>(0,i)=axis.cross(tcp-origin);j.block<3,1>(3,i)=axis;
        t*=mdh(alpha,a,definition_.mdh(i,2),q(i)+definition_.mdh(i,3));
    }return j;
}
t170::motion::IKPolicy Arm::ik_policy(const Eigen::Matrix4d &target) const {
    t170::motion::IKPolicy policy;const auto bounds=effective_limits();
    policy.minimum=bounds.minimum;policy.maximum=bounds.maximum;
    policy.preferred=definition_.preferred.cwiseMax(bounds.minimum).cwiseMin(bounds.maximum);
    policy.posture_gain=definition_.posture_gain;policy.elbow_gain=definition_.elbow_gain;policy.limit_gain=definition_.limit_gain;
    policy.jacobian=[this](const ArmJoints &q){return t170::motion::IKJacobian(const_cast<Arm *>(this)->Jacobian(q));};
    policy.elbow=[this](const ArmJoints &q){return elbow_position(q);};
    // The shoulder/elbow/wrist triangle defines an elbow circle. Prefer gravity-down
    // plus modest outward opening; reflection across Y gives the other arm's rule.
    const Eigen::Matrix4d flange=target*t170::motion::arm_pose_transform(definition_.tool).inverse();
    Eigen::Matrix4d shoulder_T=Eigen::Matrix4d::Identity();
    for(int i=0;i<2;++i)shoulder_T*=mdh(definition_.mdh(i,0),definition_.mdh(i,1),definition_.mdh(i,2),policy.preferred(i)+definition_.mdh(i,3));
    const Eigen::Vector3d shoulder=shoulder_T.topRightCorner<3,1>();
    const Eigen::Vector3d delta=flange.topRightCorner<3,1>()-shoulder;
    const double distance=delta.norm(),upper=std::abs(definition_.mdh(2,2)),fore=std::abs(definition_.mdh(4,2));
    if(distance>1e-6 && upper>0 && fore>0) {
        const Eigen::Vector3d direction=delta/distance;
        const double along=(upper*upper-fore*fore+distance*distance)/(2*distance);
        const double radius=std::sqrt(std::max(0.,upper*upper-along*along));
        Eigen::Vector3d down(0,definition_.hand==1?-definition_.elbow_outward_ratio:definition_.elbow_outward_ratio,-1);
        down-=direction*direction.dot(down);
        if(down.norm()<1e-6)down=direction.unitOrthogonal();
        policy.elbow_target=shoulder+along*direction+radius*down.normalized();
    }else policy.elbow_target=elbow_position(policy.preferred);
    return policy;
}
bool Arm::valid_solution(const Eigen::Matrix4d &target,const ArmJoints &q) {
    if(Check_Joint_Limit(q)) return false;
    const auto actual=Forward_Kinematics(q);
    return actual.allFinite() && (actual.topRightCorner<3,1>()-target.topRightCorner<3,1>()).norm()<=.0005 &&
        Eigen::AngleAxisd(target.topLeftCorner<3,3>()*actual.topLeftCorner<3,3>().transpose()).angle()<=.005;
}
int Arm::local_inverse(const Eigen::Matrix4d &target,ArmJoints &q,std::optional<double> j2) {
    auto policy=ik_policy(target);
    if(j2) {
        if(!std::isfinite(*j2) || *j2<policy.minimum(1) || *j2>policy.maximum(1))return -3;
        policy.minimum(1)=policy.maximum(1)=*j2;
    }
    ArmJoints seed=q;if(j2)seed(1)=*j2;
    t170::motion::IKReport report;ArmJoints work;
    const bool ok=t170::motion::solve_bounded_ik(seed,target,[this](const ArmJoints &v){return Forward_Kinematics(v);},
        policy,{},work,report);
    if(ok){q=work;return 0;}return report.reason=="seed_outside_effective_limits"?-3:-4;
}
int Arm::Inverse_Kinematics_Numeric(Eigen::Matrix<double,1,6> &p,Eigen::Ref<Eigen::RowVectorXd> q) {
    if(q.size()!=7 || !q.allFinite() || !p.allFinite())return -4;
    ArmJoints work=q;const int rc=local_inverse(t170::motion::arm_pose_transform(p),work,std::nullopt);if(rc==0)q=work;return rc;
}
int Arm::Inverse_Kinematics(Eigen::Matrix<double,1,6> &p,Eigen::Ref<Eigen::RowVectorXd> q) {
    if(q.size()!=7 || !q.allFinite() || !p.allFinite())return -4;
    const auto target=t170::motion::arm_pose_transform(p);const auto policy=ik_policy(target);
    const ArmJoints original=q;
    std::array<ArmJoints,4> seeds{original,policy.preferred,policy.preferred,policy.preferred};
    seeds[2](2)+=.6;seeds[3](2)-=.6;
    double best=INFINITY;ArmJoints selected;bool solved=false;
    for(auto seed:seeds) {
        seed=seed.cwiseMax(policy.minimum).cwiseMin(policy.maximum);
        if(local_inverse(target,seed,std::nullopt)!=0)continue;
        const double score=20*(elbow_position(seed)-policy.elbow_target).squaredNorm()+
            .03*(seed-policy.preferred).squaredNorm()+.001*(seed-original).squaredNorm();
        if(score<best){best=score;selected=seed;solved=true;}
    }
    // No fixed-default-seed vendor fallback: it has no continuity/posture contract.
    if(solved){q=selected;return 0;}return -4;
}
int Arm::Inverse_Kinematics_Analytic(const Eigen::Matrix<double,1,6> &p,Eigen::Ref<Eigen::RowVectorXd> q,double j2) {
    if(q.size()!=7 || !q.allFinite() || !p.allFinite() || !std::isfinite(j2))return -4;
    if(j2<definition_.minimum(1) || j2>definition_.maximum(1))return -3;
    const auto target=t170::motion::arm_pose_transform(p);ArmJoints work=q;
    // Exact 3kg URDF has non-ideal axes. Never use an unverified ideal-T7
    // analytic formula as if it solved this geometry; preserve J2 via local IK.
    if(definition_.ideal_t7 && backend_->inverse(target,work,j2)==0 && valid_solution(target,work) && std::abs(work(1)-j2)<1e-6){q=work;return 0;}
    work=q;const int rc=local_inverse(target,work,j2);if(rc==0)q=work;return rc;
}
int Arm::Joint_Trajectory(const Eigen::MatrixXd &a,const Eigen::MatrixXd &b,Eigen::MatrixXd &out) {
    out.resize(0,7);if(a.rows()!=1||b.rows()!=1||a.cols()!=7||b.cols()!=7||Check_Joint_Limit(a.row(0))||Check_Joint_Limit(b.row(0)))return -3;
    Eigen::MatrixXd knots(2,7);knots<<a,b;
    out=t170::motion::time_joint_path(knots,definition_.period,{definition_.velocity,definition_.acceleration,definition_.jerk});return out.rows()?0:-4;
}
int Arm::Line_Trajectory(const Eigen::MatrixXd &start,Eigen::Matrix<double,1,6> &p,Eigen::MatrixXd &out,std::optional<double> j2) {
    struct PlanLog {
        const std::string side;const Eigen::MatrixXd &path;double period;
        const std::chrono::steady_clock::time_point began=std::chrono::steady_clock::now();
        ~PlanLog(){
            const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
            std::ostringstream line;line<<"[arm_plan] side="<<side<<" type=line planning_ms="<<ms<<" rows="<<path.rows()
                <<" nominal_s="<<(path.rows()?path.rows()-1:0)*period<<" result="<<(path.rows()?"planned":"rejected")<<'\n';
            std::clog<<line.str();
        }
    } log{definition_.side,out,definition_.period};
    out.resize(0,7);if(start.rows()!=1||start.cols()!=7||!p.allFinite()||!std::isfinite(Cart_Linear_Velocity)||Cart_Linear_Velocity<=0)return -4;
    ArmJoints q=start.row(0);if(Check_Joint_Limit(q))return -3;
    const auto limits=effective_limits();
    if(j2 && (!std::isfinite(*j2) || *j2<limits.minimum(1) || *j2>limits.maximum(1)))return -3;
    const auto a=Forward_Kinematics(q),b=t170::motion::arm_pose_transform(p);
    const double distance=(a.topRightCorner<3,1>()-b.topRightCorner<3,1>()).norm();
    const Eigen::Quaterniond ra(a.topLeftCorner<3,3>()),rb(b.topLeftCorner<3,3>());
    const double angle=Eigen::AngleAxisd(ra.toRotationMatrix()*rb.toRotationMatrix().transpose()).angle();
    if(distance>2 || angle>M_PI)return -4;
    const int count=std::max(1,int(std::ceil(std::max(distance/.001,angle/.005))));
    Eigen::MatrixXd knots(count+1,7);knots.row(0)=q;const double j2_start=q(1);
    for(int i=1;i<=count;++i) {
        const double u=double(i)/count;Eigen::Matrix4d target=Eigen::Matrix4d::Identity();
        target.topRightCorner<3,1>()=(1-u)*a.topRightCorner<3,1>()+u*b.topRightCorner<3,1>();
        target.topLeftCorner<3,3>()=ra.slerp(u,rb).toRotationMatrix();
        const ArmJoints previous=q;
        auto policy=ik_policy(target);
        policy.minimum=policy.minimum.cwiseMax(previous-ArmJoints::Constant(.18));
        policy.maximum=policy.maximum.cwiseMin(previous+ArmJoints::Constant(.18));
        policy.null_step_rad=.003;policy.posture_iterations=2;
        if(j2)policy.minimum(1)=policy.maximum(1)=j2_start+u*(*j2-j2_start);
        ArmJoints seed=previous;if(j2)seed(1)=policy.minimum(1);
        t170::motion::IKReport report;
        const bool solved=t170::motion::solve_bounded_ik(seed,target,[this](const ArmJoints &v){return Forward_Kinematics(v);},policy,{},q,report);
        const int rc=solved?0:-4;
        if(rc!=0 || (q-previous).cwiseAbs().maxCoeff()>.20){
            std::cerr<<"[arm_ik] side="<<definition_.side<<" stage=line knot="<<i<<'/'<<count<<" code="<<(rc?rc:-7)
                <<" reason="<<report.reason<<" iterations="<<report.iterations<<" bound_joint="<<report.bound_joint+1
                <<" xyz_error_m="<<report.position_error<<" rotation_error_rad="<<report.rotation_error
                <<" start_model_rad="<<previous<<" target_xyz="<<target.topRightCorner<3,1>().transpose()<<'\n';return rc?rc:-7;
        }
        knots.row(i)=q;
    }
    out=t170::motion::time_joint_path(knots,definition_.period,{definition_.velocity,definition_.acceleration,definition_.jerk},
        1.875*std::max(distance,.2*angle)/Cart_Linear_Velocity,true);
    // Time interpolation is checked against the same SDK FK, not just knots.
    const Eigen::Vector3d line=b.topRightCorner<3,1>()-a.topRightCorner<3,1>();
    for(int i=0;i<out.rows();++i) {
        if(Check_Joint_Limit(out.row(i))){out.resize(0,7);return -3;}
        const auto t=Forward_Kinematics(out.row(i));double u=0;
        if(line.squaredNorm()>1e-14)u=std::clamp((t.topRightCorner<3,1>()-a.topRightCorner<3,1>()).dot(line)/line.squaredNorm(),0.,1.);
        else if(angle>1e-8)u=Eigen::AngleAxisd(ra.toRotationMatrix().transpose()*t.topLeftCorner<3,3>()).angle()/angle;
        if((t.topRightCorner<3,1>()-a.topRightCorner<3,1>()-u*line).norm()>.001 ||
            Eigen::AngleAxisd(ra.slerp(std::clamp(u,0.,1.),rb).toRotationMatrix()*t.topLeftCorner<3,3>().transpose()).angle()>.01){std::cerr<<"[arm_ik] side="<<definition_.side<<" interpolation_geometry row="<<i<<" rows="<<out.rows()<<" xyz_deviation_m="<<(t.topRightCorner<3,1>()-a.topRightCorner<3,1>()-u*line).norm()<<'\n';out.resize(0,7);return -5;}
    }
    return out.rows()?0:-4;
}
