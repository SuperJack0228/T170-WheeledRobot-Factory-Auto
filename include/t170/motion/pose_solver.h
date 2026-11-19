#pragma once
#include "t170/control/visual_servo.h"
#include <functional>
#include "t170/motion/bounded_ik.h"

namespace wrist_servo {
using ForwardModel = std::function<Eigen::Matrix4d(const Joints &)>;
using JointLimitCheck = std::function<bool(const Joints &)>;
bool solve_pose_with_fk(const Joints &seed, const Vec3 &target, const Eigen::Matrix3d &rotation,
                        const Config &config, const ForwardModel &forward, const JointLimitCheck &out_of_limits,
                        Joints &solution, double &xyz_residual, double &rotation_residual,
                        const t170::motion::IKPolicy *policy = nullptr);

}
