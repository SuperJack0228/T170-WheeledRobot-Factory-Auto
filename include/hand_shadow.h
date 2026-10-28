#pragma once

#include <Eigen/Dense>
#include <string>

class Robot_Arm;
class SegPoseBridge;

/** 绑定手相机推理引擎。传入空指针时停掉下降观测线程。 */
void hand_shadow_bind(SegPoseBridge *bridge);

/**
 * 轨迹线程每拍记下即将下发的电机角。只写内存，不改变下发内容。
 * hand：0 左手，1 右手。q_rad 长度 7。
 */
void hand_shadow_note_q(int hand, const double q_rad[7]);

/**
 * 开始这一段的手相机观测。竖直下降且修正打开时，锁定后改尚未发出的水平位置。
 */
void hand_shadow_begin(
    int hand,
    Robot_Arm *arm,
    const Eigen::Matrix<double, 1, 6> &goal,
    const char *tag,
    int want_class = -1);

/** 竖直下降开始取帧。平时的伸手不取手相机。
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

/**
 * 取一帧手相机。class 0 里选离当前指尖最近的工件，写出基座 XY 和这段水平距离。
 * 不看锁定门，也不用深度门槛丢掉点。看不见工件时返回 false。
 */
bool hand_track_nearest_workpiece(
    int hand,
    Robot_Arm &arm,
    SegPoseBridge &bridge,
    double &x_m,
    double &y_m,
    double &tcp_gap_m);
