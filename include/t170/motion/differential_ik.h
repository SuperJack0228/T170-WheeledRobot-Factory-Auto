#pragma once
#include <cmath>
#include "t170/motion/bounded_ik.h"
namespace t170::motion {
struct DifferentialLimits {double velocity=.45,acceleration=2.,floor_z=-INFINITY;};
struct DifferentialReport {
    std::string reason;double position_error=0,rotation_error=0,velocity_peak=0,acceleration_peak=0;
};
// Velocity QP: position bounds, velocity, acceleration and braking distance are
// simultaneous inequalities (Pink's differential-IK formulation). No step halving.
bool differential_step(const IKJoints &q,const Eigen::Matrix4d &target,const IKJoints &previous_velocity,
    double dt,const IKPolicy &,const DifferentialLimits &,const IKForward &,
    IKJoints &next,DifferentialReport &);
}
