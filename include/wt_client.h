#ifndef WT_CLIENT_H
#define WT_CLIENT_H

#include <string>
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <vector>
#include <arpa/inet.h>
#include <nlohmann/json.hpp>
#include <errno.h>
#include <sys/time.h>
#include <cmath>
#include <Eigen/Dense>
#include "aoyihand.h"

using namespace Eigen;

using json = nlohmann::json;

/**
 * 连接到服务器，返回 socket 文件描述符
 * @return 成功返回 sock（>0），失败返回 -1
 */
int connectToServer();

bool sendRequest(int sock, const std::vector<std::string>& request_fields);

/**
 * 接收服务器响应
 * @param sock 已连接的 socket
 * @return 接收到的响应字符串，失败或超时返回空字符串 ""
 */
std::string receiveResponse(int sock);

/**
 * 将 4x4 齐次变换矩阵转换为位置姿态（6DOF）
 * @param matrix_4x4 4x4 矩阵的16个元素（按行主序存储）
 * @param pose_6dof 输出的6DOF姿态 [x, y, z, roll, pitch, yaw]
 * @return true 转换成功，false 转换失败
 */
bool matrix4x4ToPose(const double* matrix_4x4, double* pose_6dof);

/**
 * 解析服务器响应 JSON，提取目标的位置姿态
 * @param response_json 服务器响应的 JSON 字符串
 * @param target_index 目标索引（默认为0，表示第一个目标）
 * @param pose_6dof 输出的6DOF姿态 [x, y, z, roll, pitch, yaw]
 * @return true 解析成功，false 解析失败
 */
bool parseResponse(const std::string& response_json, int target_index, double* pose_6dof);

/**
 * 发送已经按新协议组好的字符串数组，并解析返回位姿
 * @param wt_sock 已连接的 socket
 * @param request_fields [alg_witch, head_roll, head_pitch, head_yaw,
 *                       预留*6, 类别_id, ..., 类别_id]
 */
int get_postion(int wt_sock, const std::vector<std::string>& request_fields, Matrix<double, 1, 6> &goal_last);

int diff_drink(double y_dis, double x_dis, Matrix<double, 1, 6> &goal_last_copy, aoyi_hand ti5_hand, string target);

int change_drink_x(string target, Matrix<double, 1, 6> &goal_last_copy, aoyi_hand ti5_hand);

std::tuple<int, int, int> calculate_shelf_position(const Eigen::Matrix<double, 1, 6>& target_pose);
#endif // WT_CLIENT_H
