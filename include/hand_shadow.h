#pragma once

#include <Eigen/Dense>

class Robot_Arm;
class SegPoseBridge;

/** 绑定 YOLO 引擎。传入空指针时停掉影子线程，之后不再使用该引擎。 */
void hand_shadow_bind(SegPoseBridge *bridge);

/**
 * 轨迹线程每拍记下即将下发的电机角。只写内存，不改变下发内容。
 * hand：0 左手，1 右手。q_rad 长度 7。
 */
void hand_shadow_note_q(int hand, const double q_rad[7]);

/**
 * 伸手开始。影子线程取手相机、推理、用手眼变到基座，和 goal 比较。
 * 不修改 goal，不下发电机。
 */
void hand_shadow_begin(
    int hand,
    Robot_Arm *arm,
    const Eigen::Matrix<double, 1, 6> &goal,
    const char *tag);

/** 伸手结束。正在算的这一帧可以写完日志，不会再碰机械臂对象。 */
void hand_shadow_end(int hand);
