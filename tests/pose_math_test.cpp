// Link and exercise the extracted pose utilities without a robot, SDK or camera.
#include "t170/common/pose_math.h"
#include <iostream>
#include <stdexcept>

namespace {
void require(bool ok, const char *message) {
    if (!ok) throw std::runtime_error(message);
}
}

int main() {
    Eigen::Matrix<double, 1, 6> pose;
    pose << .42, -.23, .16, .2, -.35, .6;
    const Eigen::Matrix4d transform = TR(pose);
    require((TR(T2PosEulerAngles(transform)) - transform).norm() < 1e-12,
            "pose/transform round trip changed");
    const Eigen::Matrix3d expected_rotation =
        (Eigen::AngleAxisd(pose(5), Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pose(4), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(pose(3), Eigen::Vector3d::UnitX())).toRotationMatrix();
    require((transform.block<3, 3>(0, 0) - expected_rotation).norm() < 1e-12,
            "ZYX rotation convention changed");
    require((transform.block<3, 1>(0, 3) - pose.head<3>().transpose()).norm() < 1e-12,
            "translation units or direction changed");

    Eigen::Matrix<double, 1, 6> axis_pose;
    axis_pose << .1, .2, .3, 0, 0, M_PI / 2;
    const Eigen::Matrix4d axis_transform = Axispos2T(axis_pose);
    require((axis_transform.block<3, 3>(0, 0) -
             Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix()).norm() < 1e-12,
            "axis-angle conversion changed");

    Eigen::MatrixXd rank_deficient(2, 3);
    rank_deficient << 1, 2, 3, 2, 4, 6;
    require((rank_deficient * pinv(rank_deficient) * rank_deficient - rank_deficient).norm() < 1e-10,
            "pseudoinverse no longer handles a rank-deficient Jacobian");
    const Eigen::Matrix<double, 1, 7> start = Eigen::Matrix<double, 1, 7>::Zero();
    const Eigen::Matrix<double, 1, 7> goal = Eigen::Matrix<double, 1, 7>::Ones();
    require(quinticInterp_JointSpace(-1, 2, start, goal).isApprox(start), "negative time clamp changed");
    require(quinticInterp_JointSpace(3, 2, start, goal).isApprox(goal), "end time clamp changed");
    require(quinticInterp_JointSpace(1, 2, start, goal).isApprox(.5 * goal), "joint midpoint changed");
    std::cout << "extracted pose math: conventions, singular matrix and interpolation passed\n";
}
