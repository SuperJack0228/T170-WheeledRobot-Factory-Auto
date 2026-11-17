#pragma once
#include "t170/common/units.h"
#include <Eigen/Dense>
#include <array>
#include <functional>
#include <string>
#include <tuple>
#include <vector>
class Robot_Arm;
Eigen::MatrixXd Transl_xyz(Eigen::Matrix<double, 1, 3> trans);

Eigen::MatrixXd Rot_zyx(Eigen::Matrix<double, 1, 3>  theta);

Eigen::MatrixXd TR(Eigen::Matrix<double, 1, 6> transpose);

Eigen::MatrixXd MDHTrans(double alpha, double  a, double  d, double  theta);

Eigen::Matrix<double, 1, 3> rotationMatrixToEulerAngles(Eigen::Matrix<double, 3, 3> R);

Eigen::Matrix<double, 1, 6> T2PosEulerAngles(Eigen::Matrix<double, 4, 4> T);

Eigen::Vector3d Quaterniond2EulerAngles(Eigen::Quaterniond q);

Eigen::MatrixXd T2Axispos(Eigen::Matrix<double, 4, 4> T);



Eigen::MatrixXd Axispos2T(Eigen::Matrix<double, 1, 6> pos);















Eigen::MatrixXd pinv(Eigen::MatrixXd A);







Eigen::Matrix<double, 1, 7> quinticInterp_JointSpace(double t, double totalTime, const Eigen::Matrix<double, 1, 7>& start,  const Eigen::Matrix<double, 1, 7>& target);



Eigen::Matrix3d Yaw_Rotation(float yaw);

Eigen::Matrix3d Pitch_Rotation(float pitch);

Eigen::Matrix3d Roll_Rotation(float roll);



double calculateTf(const Eigen::Matrix<double, 1, 6>& axis_p1,
                   const Eigen::Matrix<double, 1, 6>& axis_p2,
                   double V_DEFAULT);





std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd>
quinticInterp(
    double t,
    double totalTime,
    const Eigen::MatrixXd& start,  // 统一 Matrixd：1×N
    const Eigen::MatrixXd& target  // 统一 Matrixd：1×N
);
