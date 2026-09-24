#pragma once

/** 头部 CAN 30=偏航 32=俯仰 31=横滚（本机 31/32 对调）。 */

/** 读当前头部编码器角（度）。成功返回 true。 */
bool read_head_rpy_deg(double &yaw_deg, double &pitch_deg, double &roll_deg);

/** 走到指定头角。enable_motors=true 时先清错/使能（home/grasp/ready 入口）。
 *  已在目标 ±1° 内则跳过运动。0=成功，-1=CAN，-4=中止。 */
int move_head_to_rpy_deg(double yaw_deg, double pitch_deg, double roll_deg, bool enable_motors);

/** home / grasp / ready：使能并走到 yaml head.pitch_deg（默认 40°）。 */
int enable_head_calib_pose();

/** 远三排：低头到 yaml head.far_pitch_deg（默认 60°），偏航/横滚保持标定值。 */
int enable_head_far_pitch();
