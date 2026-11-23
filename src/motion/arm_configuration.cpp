#include "t170/motion/arm.h"
#include "t170/common/paths.h"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <cmath>
#include <set>
#include <stdexcept>

namespace t170::motion {
namespace {
template<int N> Eigen::Matrix<double,1,N> row(const YAML::Node &v,const char *key) {
    if(!v.IsSequence() || v.size()!=N) throw std::runtime_error(std::string("Expected ")+key+" length "+std::to_string(N));
    Eigen::Matrix<double,1,N> q;
    for(int i=0;i<N;++i) q(i)=v[i].as<double>();
    if(!q.allFinite()) throw std::runtime_error(std::string("Nonfinite ")+key);
    return q;
}
}
Eigen::Matrix4d arm_pose_transform(const Eigen::Matrix<double,1,6> &p) {
    Eigen::Matrix4d t=Eigen::Matrix4d::Identity();
    t.topLeftCorner<3,3>()=(Eigen::AngleAxisd(p(5),Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(p(4),Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(p(3),Eigen::Vector3d::UnitX())).toRotationMatrix();
    t.topRightCorner<3,1>()=p.head<3>().transpose();return t;
}
ArmDefinition load_arm_definition(const std::string &file,const std::string &side) {
    if(side!="left" && side!="right") throw std::runtime_error("Invalid arm side");
    const auto dir=std::filesystem::absolute(file).parent_path();
    const auto root=YAML::LoadFile(file),n=root["T7_170"][side];
    const auto binding=YAML::LoadFile((dir/"arm_binding.yaml").string());
    const auto b=binding[side];
    ArmDefinition d;d.side=side;d.hand=side=="left"?0:1;d.config_path=std::filesystem::absolute(file).string();
    d.model_id=binding["model_id"].as<std::string>();
    if(binding["joint_coordinates"].as<std::string>()!="sdk_mdh_model_rad") throw std::runtime_error("Invalid arm coordinate contract");
    d.motor_direction=row<7>(b["model_to_device_direction"],"motor direction");
    d.motor_offset=row<7>(b["model_to_device_offset_rad"],"motor offset");
    for(int i=0;i<7;++i) if(std::abs(std::abs(d.motor_direction(i))-1)>1e-12) throw std::runtime_error("Expected direction +/-1");
    d.minimum=row<7>(n["joint_limit_min"],"joint minimum");d.maximum=row<7>(n["joint_limit_max"],"joint maximum");
    if((d.minimum.array()>=d.maximum.array()).any()) throw std::runtime_error("Invalid arm joint limits");
    d.mdh.col(0)=row<7>(n["alpha"],"alpha").transpose();d.mdh.col(1)=row<7>(n["a"],"a").transpose();
    d.mdh.col(2)=row<7>(n["d"],"d").transpose();d.mdh.col(3)=row<7>(n["offset"],"DH offset").transpose();
    d.tool=row<6>(n["tool"],"tool");d.can_ids=b["can_ids"].as<std::vector<int>>();
    std::set<int> ids(d.can_ids.begin(),d.can_ids.end());
    if(d.can_ids.size()!=7 || ids.size()!=7 || *ids.begin()<1 || *ids.rbegin()>127) throw std::runtime_error("Invalid arm motor IDs");
    d.period=root["_plan"]["sample_rate"].as<double>();
    if(!std::isfinite(d.period) || std::abs(d.period-.005)>1e-12) throw std::runtime_error("Arm executor requires _plan.sample_rate=0.005 s");
    const auto limits=binding["trajectory_limits"];
    d.velocity=limits["velocity_rad_s"].as<double>();d.acceleration=limits["acceleration_rad_s2"].as<double>();d.jerk=limits["jerk_rad_s3"].as<double>();
    for(double v:{d.velocity,d.acceleration,d.jerk}) if(!std::isfinite(v)||v<=0) throw std::runtime_error("Invalid arm trajectory limits");
    d.preferred=(d.minimum+d.maximum)*.5;
    if(const auto policy=binding["motion_policy"];policy) {
        if(policy["joint_coordinates"].as<std::string>()!="sdk_mdh_model_deg")throw std::runtime_error("Invalid motion policy coordinates");
        d.home=row<7>(policy[side]["home_deg"],"home joints")*M_PI/180;
        d.preferred=row<7>(policy[side]["preferred_deg"],"preferred joints")*M_PI/180;
        d.posture_gain=policy["posture_gain"].as<double>();d.elbow_gain=policy["elbow_gain"].as<double>();d.limit_gain=policy["limit_gain"].as<double>();
        d.elbow_outward_ratio=policy["elbow_outward_ratio"].as<double>();
        d.wrist_margin_rad=policy["wrist_margin_rad"].as<double>(.14);
        d.wrist_gain=policy["wrist_gain"].as<double>(2.);
        d.elbow_clearance_m=policy["elbow_clearance_m"].as<double>(.08);
        if(!std::isfinite(d.wrist_margin_rad) || d.wrist_margin_rad<=0 || !std::isfinite(d.wrist_gain) || d.wrist_gain<0 ||
           !std::isfinite(d.elbow_clearance_m) || d.elbow_clearance_m<0)
            throw std::runtime_error("Invalid wrist preference");
        for(const auto &q:{d.home,d.preferred})if((q.array()<d.minimum.array()).any() || (q.array()>d.maximum.array()).any())throw std::runtime_error("Motion policy outside model limits");
        for(double v:{d.posture_gain,d.elbow_gain,d.limit_gain,d.elbow_outward_ratio})if(!std::isfinite(v)||v<0)throw std::runtime_error("Invalid posture policy gain");
    }
    d.limits=std::make_shared<ArmLimitState>();d.limits->bounds={d.minimum,d.maximum,false};
    d.ideal_t7=true;
    const double al[7]={M_PI/2,M_PI/2,M_PI/2,-M_PI/2,M_PI/2,-M_PI/2,-M_PI/2};
    const double off[7]={M_PI/2,-M_PI/2,M_PI/2,0,0,-M_PI/2,0};
    for(int i=0;i<7;++i) if(std::abs(d.mdh(i,0)-al[i])>1e-9 || std::abs(d.mdh(i,1))>1e-9 ||
        std::abs(d.mdh(i,3)-off[i])>1e-9 || ((i==1||i==3||i==5||i==6)&&std::abs(d.mdh(i,2))>1e-9)) d.ideal_t7=false;
    return d;
}
ArmBounds effective_arm_bounds(const ArmDefinition &d) {
    if(!d.limits)return {d.minimum,d.maximum,false};
    std::lock_guard lock(d.limits->mutex);return d.limits->bounds;
}
ArmBounds intersect_device_bounds(const ArmDefinition &d,const ArmJoints &low,const ArmJoints &high) {
    if(!low.allFinite() || !high.allFinite() || (low.array()>=high.array()).any())throw std::runtime_error("Invalid device arm limits");
    const ArmJoints a=(low-d.motor_offset).cwiseQuotient(d.motor_direction),b=(high-d.motor_offset).cwiseQuotient(d.motor_direction);
    ArmBounds result{d.minimum.cwiseMax(a.cwiseMin(b)),d.maximum.cwiseMin(a.cwiseMax(b)),true};
    // This machine's J2 driver range is about ±85 deg. After the ±90 deg offset
    // that becomes model ±5 deg. Open only that inner side by 2 deg (±5 → ±3)
    // and keep it inside the model range. The drive limit itself is not written.
    constexpr int kJ2 = 1;
    constexpr double kInnerOpenRad = 2.0 * M_PI / 180.0;
    if (std::abs(result.maximum(kJ2)) <= std::abs(result.minimum(kJ2)))
        result.maximum(kJ2) = std::min(d.maximum(kJ2), result.maximum(kJ2) + kInnerOpenRad);
    else
        result.minimum(kJ2) = std::max(d.minimum(kJ2), result.minimum(kJ2) - kInnerOpenRad);
    if((result.minimum.array()>=result.maximum.array()).any())throw std::runtime_error("Device/model arm limits have empty intersection");
    return result;
}
const ArmDefinition &arm_definition(int hand) {
    if(hand!=0 && hand!=1) throw std::runtime_error("Invalid arm hand index");
    static const std::array<ArmDefinition,2> defs={
        load_arm_definition(t170::paths::config_file("Robot_Model.yaml").string(),"left"),
        load_arm_definition(t170::paths::config_file("Robot_Model.yaml").string(),"right")};
    return defs[hand];
}
ArmJoints model_to_device(int hand,const ArmJoints &q) {
    const auto &d=arm_definition(hand);return q.cwiseProduct(d.motor_direction)+d.motor_offset;
}
ArmJoints device_to_model(int hand,const ArmJoints &q) {
    const auto &d=arm_definition(hand);return (q-d.motor_offset).cwiseQuotient(d.motor_direction);
}
}
