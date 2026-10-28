#pragma once

#include <Eigen/Dense>
#include <string>

class Robot_Arm;
class SegPoseBridge;

/** 绑定手相机推理引擎。传入空指针时停掉下降观测线程。 */
void hand_shadow_bind(SegPoseBridge *bridge);

/**
 * 开始双轴像素观测。只提交带时间戳的结果，不直接写电机或改写轨迹。
 */
void hand_shadow_begin(
    int hand,
    Robot_Arm *arm,
    const Eigen::Matrix<double, 1, 6> &goal,
    const char *tag,
    int want_class = -1);

/** 激活接近与下降期间的手相机观测。
 *  want_class：0 只认工件（抓取），1 只认孔（放置），-1 不限。 */
void hand_shadow_ensure(
    int hand,
    Robot_Arm *arm,
    const Eigen::Matrix<double, 1, 6> &goal,
    const char *tag,
    int want_class = -1);

/** 伸手结束。正在算的这一帧可以写完日志，不会再碰机械臂对象。 */
void hand_shadow_end(int hand);

/** 为真时，竖直下降段把未标注的彩色图写入仓库 hand_descent_raw。接近段不存。 */
void hand_shadow_set_descent_raw(bool enable);

/** 追加一行到 logs/hand_track.csv。手相机跟随和伺服共用，方便事后看有没有把点交给电机。 */
void hand_track_log(const std::string &hand, const std::string &result, const std::string &detail);
