#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <utility>

namespace t170::motion {
// Operator-guided jogs need measured progress, without demanding accuracy finer
// than the local IK and joint feedback can provide. Small no-ops still fail.
inline double teach_jog_tolerance(double distance_m) {
    return std::min(.45*distance_m,std::clamp(.4*distance_m,.00008,.0015));
}
class CartesianArrivalGate {
public:
    CartesianArrivalGate(Eigen::Matrix4d goal,double position_tolerance,double rotation_tolerance=.01,
                         double stable_position=0,double stable_rotation=.001)
        :goal_(std::move(goal)),position_tolerance_(position_tolerance),rotation_tolerance_(rotation_tolerance),
         stable_position_(stable_position>0?stable_position:std::clamp(.25*position_tolerance,.00002,.0005)),
         stable_rotation_(stable_rotation) {}
    bool operator()(const Eigen::Matrix4d &actual) {
        if(!actual.allFinite()) {have_previous_=false;return false;}
        const double xyz=(actual.topRightCorner<3,1>()-goal_.topRightCorner<3,1>()).norm();
        const double rotation=Eigen::AngleAxisd(goal_.topLeftCorner<3,3>()*actual.topLeftCorner<3,3>().transpose()).angle();
        const bool stable=have_previous_ &&
            (actual.topRightCorner<3,1>()-previous_.topRightCorner<3,1>()).norm()<=stable_position_ &&
            Eigen::AngleAxisd(previous_.topLeftCorner<3,3>()*actual.topLeftCorner<3,3>().transpose()).angle()<=stable_rotation_;
        previous_=actual;have_previous_=true;
        return stable && xyz<=position_tolerance_ && rotation<=rotation_tolerance_;
    }
private:
    Eigen::Matrix4d goal_,previous_=Eigen::Matrix4d::Identity();
    double position_tolerance_,rotation_tolerance_,stable_position_,stable_rotation_;
    bool have_previous_=false;
};
}
