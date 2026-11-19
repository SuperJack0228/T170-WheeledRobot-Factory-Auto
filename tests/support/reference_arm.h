#pragma once
#include "t170/motion/arm.h"
using t170::motion::ArmJoints;
struct ReferenceBackend final : t170::motion::ArmBackend {
    t170::motion::ArmDefinition d;
    int inverse_calls=0;
    explicit ReferenceBackend(t170::motion::ArmDefinition v):d(std::move(v)){}
    Eigen::Matrix4d flange(const ArmJoints &q) const override {
        Eigen::Matrix4d t=Eigen::Matrix4d::Identity();
        for(int i=0;i<7;++i) {
            const double al=d.mdh(i,0),a=d.mdh(i,1),di=d.mdh(i,2),theta=q(i)+d.mdh(i,3);
            Eigen::Matrix4d m;
            m<<cos(theta),-sin(theta),0,a,sin(theta)*cos(al),cos(theta)*cos(al),-sin(al),-sin(al)*di,
               sin(theta)*sin(al),cos(theta)*sin(al),cos(al),cos(al)*di,0,0,0,1;
            t*=m;
        }return t;
    }
    Eigen::Matrix4d forward(const ArmJoints &q) const override {return flange(q)*t170::motion::arm_pose_transform(d.tool);}
    int inverse(const Eigen::Matrix4d &,ArmJoints &q,std::optional<double>) override {++inverse_calls;q.setConstant(999);return 1;}
};
