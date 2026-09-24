#pragma once

#include <array>
#include <string>

/** free_calib：T_base_cam = Trans(0,0,L) @ Rz(yaw)Ry(pitch)Rx(roll) @ T_head_cam。
 *  平移输出为米，与 load_cam2robot_matrix 一致。 */

bool compute_head_cam2robot(
    double roll_deg,
    double pitch_deg,
    double yaw_deg,
    std::array<double, 16> &out_m,
    std::string &err);

/** 优先用头部编码器角现算；读不到电机则用 yaml 默认头角；再失败回退静态 camera_to_base yaml。 */
bool load_head_cam2robot(std::array<double, 16> &out_m, std::string &err);
