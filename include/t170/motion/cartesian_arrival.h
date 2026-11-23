#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <utility>

namespace t170::motion {
// Teach-jog accuracy must scale with the requested displacement; a 5 mm jog
// cannot be declared complete just because a coarse encoder band was reached.
inline double teach_jog_tolerance(double distance_m) {
    return std::clamp(.15*distance_m,.00003,.00075);
}
class CartesianArrivalGate {
public:
    CartesianArrivalGate(Eigen::Matrix4d goal,double position_tolerance,double rotation_tolerance=.01)
        :goal_(std::move(goal)),position_tolerance_(position_tolerance),rotation_tolerance_(rotation_tolerance) {}
    bool operator()(const Eigen::Matrix4d &actual) {
        if(!actual.allFinite()) {have_previous_=false;return false;}
        const double xyz=(actual.topRightCorner<3,1>()-goal_.topRightCorner<3,1>()).norm();
        const double rotation=Eigen::AngleAxisd(goal_.topLeftCorner<3,3>()*actual.topLeftCorner<3,3>().transpose()).angle();
        const bool stable=have_previous_ &&
            (actual.topRightCorner<3,1>()-previous_.topRightCorner<3,1>()).norm()<=std::clamp(.25*position_tolerance_,.00002,.0005) &&
            Eigen::AngleAxisd(previous_.topLeftCorner<3,3>()*actual.topLeftCorner<3,3>().transpose()).angle()<=.001;
        previous_=actual;have_previous_=true;
        return stable && xyz<=position_tolerance_ && rotation<=rotation_tolerance_;
    }
private:
    Eigen::Matrix4d goal_,previous_=Eigen::Matrix4d::Identity();
    double position_tolerance_,rotation_tolerance_;
    bool have_previous_=false;
};
}
