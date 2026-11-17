#pragma once
#include <Eigen/Dense>
#include <array>
#include <functional>
#include <string>
#include <tuple>
#include <vector>
class Robot_Arm;
#include "t170/motion/types.h"


ArmLinkFk arm_fk_links(Robot_Arm &arm, const Eigen::Ref<const Eigen::RowVectorXd> &q_motor);

ArmLinkFk arm_get_link_fk(Robot_Arm &arm);



Eigen::Matrix<double, 1, 6> arm_get_tcp_pos(Robot_Arm &arm);

Eigen::Matrix<double, 1, 6> arm_get_base_visual_pos(Robot_Arm &arm, Eigen::Matrix<double, 1, 6> &falan_visualPos);



Eigen::Matrix<double, 1, 6> arm_tcp_from_joints(
    Robot_Arm &arm,
    const Eigen::Ref<const Eigen::RowVectorXd> &q_motor);
