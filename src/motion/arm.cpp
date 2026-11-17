#include "t170/motion/arm.h"
#include "t170/motion/joint_motion.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
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
bool Arm::Check_Joint_Limit(const Eigen::Ref<const Eigen::RowVectorXd> &q) {
    return q.size()!=7 || !q.allFinite() || (q.array()<definition_.minimum.array()-1e-8).any() ||
        (q.array()>definition_.maximum.array()+1e-8).any();
}
Eigen::MatrixXd Arm::Jacobian(const Eigen::Ref<const Eigen::RowVectorXd> &q) {
    const auto t=Forward_Kinematics(q);Eigen::Matrix<double,6,7> j;constexpr double h=1e-6;
    for(int i=0;i<7;++i) {
        Eigen::RowVectorXd p=q;p(i)+=h;const auto s=Forward_Kinematics(p);
        j.block<3,1>(0,i)=(s.topRightCorner<3,1>()-t.topRightCorner<3,1>())/h;
        const Eigen::AngleAxisd a(s.topLeftCorner<3,3>()*t.topLeftCorner<3,3>().transpose());
        j.block<3,1>(3,i)=a.angle()*a.axis()/h;
    }return j;
}
bool Arm::valid_solution(const Eigen::Matrix4d &target,const ArmJoints &q) {
    if(Check_Joint_Limit(q)) return false;
    const auto actual=Forward_Kinematics(q);
    return actual.allFinite() && (actual.topRightCorner<3,1>()-target.topRightCorner<3,1>()).norm()<=.0005 &&
        Eigen::AngleAxisd(target.topLeftCorner<3,3>()*actual.topLeftCorner<3,3>().transpose()).angle()<=.005;
}
int Arm::local_inverse(const Eigen::Matrix4d &target,ArmJoints &q,std::optional<double> j2) {
    if(!target.allFinite() || Check_Joint_Limit(q) || (j2 && (!std::isfinite(*j2)||*j2<definition_.minimum(1)||*j2>definition_.maximum(1)))) return -3;
    ArmJoints work=q;if(j2)work(1)=*j2;
    auto error=[&](const ArmJoints &v) -> Eigen::Matrix<double,6,1> {
        const auto t=Forward_Kinematics(v);const Eigen::AngleAxisd a(target.topLeftCorner<3,3>()*t.topLeftCorner<3,3>().transpose());
        Eigen::Matrix<double,6,1> e;e.head<3>()=target.topRightCorner<3,1>()-t.topRightCorner<3,1>();e.tail<3>()=.15*a.angle()*a.axis();return e;
    };
    for(int iteration=0;iteration<100;++iteration) {
        const auto e=error(work);
        if(e.head<3>().norm()<1e-5 && e.tail<3>().norm()<.15e-4) {q=work;return 0;}
        Eigen::Matrix<double,6,7> j=Jacobian(work);j.bottomRows<3>()*=.15;if(j2)j.col(1).setZero();
        Eigen::Matrix<double,6,6> a=j*j.transpose();a.diagonal().array()+=1e-7;
        Eigen::Matrix<double,7,1> step=j.transpose()*a.ldlt().solve(e);
        if(!step.allFinite()) return -4;
        const double largest=step.cwiseAbs().maxCoeff();if(largest>.10)step*=.10/largest;
        bool accepted=false;
        for(double scale=1;scale>=.015625;scale*=.5) {
            ArmJoints candidate=work+scale*step.transpose();if(j2)candidate(1)=*j2;
            if(!Check_Joint_Limit(candidate) && error(candidate).squaredNorm()<e.squaredNorm()) {work=candidate;accepted=true;break;}
        }
        if(!accepted)break;
    }
    if(valid_solution(target,work)) {q=work;return 0;}return -4;
}
int Arm::Inverse_Kinematics_Numeric(Eigen::Matrix<double,1,6> &p,Eigen::Ref<Eigen::RowVectorXd> q) {
    if(q.size()!=7 || !q.allFinite() || !p.allFinite())return -4;
    ArmJoints work=q;const int rc=local_inverse(t170::motion::arm_pose_transform(p),work,std::nullopt);if(rc==0)q=work;return rc;
}
int Arm::Inverse_Kinematics(Eigen::Matrix<double,1,6> &p,Eigen::Ref<Eigen::RowVectorXd> q) {
    if(q.size()!=7 || !q.allFinite() || !p.allFinite())return -4;
    const auto target=t170::motion::arm_pose_transform(p);ArmJoints work=q;
    if(local_inverse(target,work,std::nullopt)==0){q=work;return 0;}
    work=q;const int rc=backend_->inverse(target,work,std::nullopt);
    if(rc==0 && valid_solution(target,work)){q=work;return 0;}return rc==0?-5:rc;
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
    out.resize(0,7);if(start.rows()!=1||start.cols()!=7||!p.allFinite()||!std::isfinite(Cart_Linear_Velocity)||Cart_Linear_Velocity<=0)return -4;
    ArmJoints q=start.row(0);if(Check_Joint_Limit(q))return -3;
    const auto a=Forward_Kinematics(q),b=t170::motion::arm_pose_transform(p);
    const double distance=(a.topRightCorner<3,1>()-b.topRightCorner<3,1>()).norm();
    const Eigen::Quaterniond ra(a.topLeftCorner<3,3>()),rb(b.topLeftCorner<3,3>());
    const double angle=Eigen::AngleAxisd(ra.toRotationMatrix()*rb.toRotationMatrix().transpose()).angle();
    if(distance>2 || angle>M_PI)return -4;
    const int count=std::max(1,int(std::ceil(std::max(distance/.002,angle/.01))));
    Eigen::MatrixXd knots(count+1,7);knots.row(0)=q;const double j2_start=q(1);
    for(int i=1;i<=count;++i) {
        const double u=double(i)/count;Eigen::Matrix4d target=Eigen::Matrix4d::Identity();
        target.topRightCorner<3,1>()=(1-u)*a.topRightCorner<3,1>()+u*b.topRightCorner<3,1>();
        target.topLeftCorner<3,3>()=ra.slerp(u,rb).toRotationMatrix();
        const ArmJoints previous=q;
        const int rc=local_inverse(target,q,j2?std::optional<double>(j2_start+u*(*j2-j2_start)):std::nullopt);
        if(rc!=0 || (q-previous).cwiseAbs().maxCoeff()>.20)return rc?rc:-7;
        knots.row(i)=q;
    }
    out=t170::motion::time_joint_path(knots,definition_.period,{definition_.velocity,definition_.acceleration,definition_.jerk},
        1.875*std::max(distance,.2*angle)/Cart_Linear_Velocity);
    // Time interpolation is checked against the same SDK FK, not just knots.
    const Eigen::Vector3d line=b.topRightCorner<3,1>()-a.topRightCorner<3,1>();
    for(int i=0;i<out.rows();++i) {
        if(Check_Joint_Limit(out.row(i))){out.resize(0,7);return -3;}
        const auto t=Forward_Kinematics(out.row(i));double u=0;
        if(line.squaredNorm()>1e-14)u=std::clamp((t.topRightCorner<3,1>()-a.topRightCorner<3,1>()).dot(line)/line.squaredNorm(),0.,1.);
        else if(angle>1e-8)u=Eigen::AngleAxisd(ra.toRotationMatrix().transpose()*t.topLeftCorner<3,3>()).angle()/angle;
        if((t.topRightCorner<3,1>()-a.topRightCorner<3,1>()-u*line).norm()>.001 ||
            Eigen::AngleAxisd(ra.slerp(std::clamp(u,0.,1.),rb).toRotationMatrix()*t.topLeftCorner<3,3>().transpose()).angle()>.01){out.resize(0,7);return -5;}
    }
    return out.rows()?0:-4;
}
