#include "wt_client.h"
#include "reals_tcp.h"

int connectToServer()
{
    const char *host = "127.0.0.1";
    const int port = 12345;
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == -1)
    {
        std::cerr << "创建 socket 失败: " << strerror(errno) << std::endl;
        return -1;
    }

    // 设置超时（5秒）
    // struct timeval timeout{100, 0};
    // if (setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) < 0 ||
    //     setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0)
    // {
    //     std::cerr << "设置超时失败: " << strerror(errno) << std::endl;
    //     close(sock);
    //     return -1;
    // }

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);

    if (inet_pton(AF_INET, host, &server_addr.sin_addr) <= 0)
    {
        std::cerr << "无效的地址: " << host << std::endl;
        close(sock);
        return -1;
    }

    if (connect(sock, (sockaddr *)&server_addr, sizeof(server_addr)) == -1)
    {
      //  std::cerr << "连接失败: " << strerror(errno) << std::endl;
        close(sock);
        return -1;
    }

    std::cout << "连接服务器成功 (" << host << ":" << port << ")" << std::endl;
    return sock;
}

bool sendRequest(int sock, const std::vector<std::string> &request_fields)
{
    if (sock < 0)
    {
        std::cerr << "无效的 socket" << std::endl;
        return false;
    }

    // 新协议：发送一个字符串数组
    // [alg_witch, head_roll, head_pitch, head_yaw,
    //  预留, 预留, 预留, 预留, 预留, 预留,
    //  类别_id, ..., 类别_id]
    json request_json = json::array();
    for (const auto &field : request_fields)
    {
        request_json.push_back(field);
    }

    std::string request_str = request_json.dump();
    std::cout << "发送请求: " << request_str << std::endl;

    if (send(sock, request_str.c_str(), request_str.size(), 0) < 0)
    {
        std::cerr << "发送失败: " << strerror(errno) << std::endl;
        return false;
    }

    return true;
}

std::string receiveResponse(int sock)
{
    if (sock < 0)
    {
        std::cerr << "无效的 socket" << std::endl;
        return "";
    }

    char buffer[4096] = {0};
    int bytes_received = recv(sock, buffer, sizeof(buffer) - 1, 0);

    if (bytes_received > 0)
    {
        std::string response(buffer, bytes_received);
        return response;
    }
    else if (bytes_received == 0)
    {
        std::cerr << "连接关闭" << std::endl;
    }
    else
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            std::cerr << "接收超时（5秒）" << std::endl;
        }
        else
        {
            std::cerr << "接收失败: " << strerror(errno) << std::endl;
        }
    }

    return "";
}

bool matrix4x4ToPose(const double *matrix_4x4, double *pose_6dof)
{
    if (matrix_4x4 == nullptr || pose_6dof == nullptr)
    {
        std::cerr << "空指针错误" << std::endl;
        return false;
    }

    // 提取平移部分（矩阵的最后一列的前3个元素）
    // 矩阵按行主序存储：[m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33]
    // 平移是 m03, m13, m23
    pose_6dof[0] = matrix_4x4[3];  // x
    pose_6dof[1] = matrix_4x4[7];  // y
    pose_6dof[2] = matrix_4x4[11]; // z

    // 提取旋转矩阵（3x3部分）
    // [m00, m01, m02]
    // [m10, m11, m12]
    // [m20, m21, m22]
    double r11 = matrix_4x4[0], r12 = matrix_4x4[1], r13 = matrix_4x4[2];
    double r21 = matrix_4x4[4], r22 = matrix_4x4[5], r23 = matrix_4x4[6];
    double r31 = matrix_4x4[8], r32 = matrix_4x4[9], r33 = matrix_4x4[10];

    // 转换为 ZYX 欧拉角（Roll-Pitch-Yaw）
    // Roll (绕X轴旋转)
    double roll = atan2(r32, r33);

    // Pitch (绕Y轴旋转)
    double pitch = atan2(-r31, sqrt(r32 * r32 + r33 * r33));

    // Yaw (绕Z轴旋转)
    double yaw = atan2(r21, r11);

    pose_6dof[3] = roll;  // roll
    pose_6dof[4] = pitch; // pitch
    pose_6dof[5] = yaw;   // yaw

    return true;
}

bool parseResponse(const std::string &response_json, int target_index, double *pose_6dof)
{
    if (pose_6dof == nullptr)
    {
        std::cerr << "空指针错误" << std::endl;
        return false;
    }

    try
    {
        json response = json::parse(response_json);

        // 检查 targets 数组是否存在
        if (!response.contains("targets") || !response["targets"].is_array())
        {
            std::cerr << "响应中缺少 targets 数组" << std::endl;
            return false;
        }

        auto &targets = response["targets"];
        if (target_index < 0 || target_index >= static_cast<int>(targets.size()))
        {
            std::cerr << "目标索引超出范围: " << target_index << std::endl;
            return false;
        }

        auto &target = targets[target_index];

        // 检查 pose_in_robot 是否存在
        if (!target.contains("pose_in_robot") || !target["pose_in_robot"].is_array())
        {
            std::cerr << "目标中缺少 pose_in_robot 数组" << std::endl;
            return false;
        }

        auto &pose_matrix = target["pose_in_robot"];
        if (pose_matrix.size() != 16)
        {
            std::cerr << "pose_in_robot 数组大小不正确，期望16，实际: " << pose_matrix.size() << std::endl;
            return false;
        }

        // 将 JSON 数组转换为 double 数组
        double matrix_4x4[16];
        for (size_t i = 0; i < 16; ++i)
        {
            matrix_4x4[i] = pose_matrix[i].get<double>();
        }

        // 转换为6DOF姿态
        return matrix4x4ToPose(matrix_4x4, pose_6dof);
    }
    catch (const json::exception &e)
    {
        std::cerr << "JSON 解析错误: " << e.what() << std::endl;
        return false;
    }
    catch (...)
    {
        std::cerr << "未知错误" << std::endl;
        return false;
    }
}

int get_postion(int wt_sock, const std::vector<std::string> &request_fields, Matrix<double, 1, 6> &goal_last)
{
      cout << "wt_sock： "  << wt_sock << endl;
    double pose_6dof[6] = {};

    auto start = std::chrono::steady_clock::now();

    if (!sendRequest(wt_sock, request_fields))
    {
        close(wt_sock);
        return -1;
    }

    std::string response = receiveResponse(wt_sock);

    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    // 记录到日志文件
    std::ofstream log("vision_time.log", std::ios::app);
    if (log.is_open())
    {
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);

        log << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S");
        log << " | 请求字段数: " << request_fields.size();
        log << " | 耗时: " << duration.count() << " ms";
        log << " | 响应长度: " << response.length() << " bytes" << std::endl;
        log.close();
    }

    //cout << response << endl;
    if (response.empty())
    {
        std::cerr << "未收到响应" << std::endl;
        close(wt_sock);
        return -1;
    }

    if (parseResponse(response, 0, pose_6dof))
    {
        goal_last[0] = pose_6dof[0];
        goal_last[1] = pose_6dof[1];
         goal_last[2] = pose_6dof[2];
       // goal_last[2] = -0.21;
        // goal_last[3] = pose_6dof[3];
        // goal_last[4] = pose_6dof[4];
        // goal_last[5] = pose_6dof[5];

        goal_last[3] = 0;
        goal_last[4] = 0;
        goal_last[5] = 0;

        return 0;
    }
    else
    {
        std::cerr << "解析响应失败" << std::endl;
        return -1;
    }
}

int diff_drink(double y_dis, double x_dis, Matrix<double, 1, 6> &goal_last_copy, aoyi_hand ti5_hand, string target)
{
    
        if (ti5_hand == right_a)
        {


             goal_last_copy[0] += x_dis ;

            goal_last_copy[0] +=  readPiancha("8");

            goal_last_copy[1] += y_dis ;
             goal_last_copy[1]+=readPiancha("103");




        }
        else
        {

              goal_last_copy[0] += x_dis ;

            goal_last_copy[0] +=  readPiancha("8");

            goal_last_copy[1] -= y_dis ;
             goal_last_copy[1]+=readPiancha("102");
        }
    
    return 0;
}

int change_drink_x(string target, Matrix<double, 1, 6> &goal_last_copy, aoyi_hand ti5_hand)
{
   
        if (ti5_hand == right_a)
        {
            goal_last_copy[0] += readPiancha("107");
        }
        else
        {
            goal_last_copy[0] += readPiancha("106");
        }
    
      return 0;
}



std::tuple<int, int, int> calculate_shelf_position(const Eigen::Matrix<double, 1, 6>& target_pose) {
    // 1. 定义货架各维度的基准坐标（与题目描述完全对应）
    // 层高z值
    std::vector<double> shelf_z = {-0.12, -0.38, -0.63};
    // 列宽y值
    std::vector<double> shelf_y = {0.91, 0.67, 0.45, 0.22, 0.01, -0.21, -0.43, -0.70};
    // 排深x值
    std::vector<double> shelf_x = {1.44, 1.67};
    
    // 2. 提取目标的x,y,z坐标（忽略roll,pitch,yaw）
    double target_x = target_pose(0, 0);
    double target_y = target_pose(0, 1);
    double target_z = target_pose(0, 2);
    
    // 3. 计算各维度的绝对差值，找到最接近的基准值的索引
    // 计算层（z轴）：找到与target_z差值最小的shelf_z元素的索引（从0开始计数）
    int layer = 0;
    double min_z_diff = std::abs(target_z - shelf_z[0]);
    for (size_t i = 1; i < shelf_z.size(); ++i) {
        double diff = std::abs(target_z - shelf_z[i]);
        if (diff < min_z_diff) {
            min_z_diff = diff;
            layer = i;
        }
    }
    
    // 计算列（y轴）：找到与target_y差值最小的shelf_y元素的索引
    int column = 0;
    double min_y_diff = std::abs(target_y - shelf_y[0]);
    for (size_t i = 1; i < shelf_y.size(); ++i) {
        double diff = std::abs(target_y - shelf_y[i]);
        if (diff < min_y_diff) {
            min_y_diff = diff;
            column = i;
        }
    }
    
    // 计算排（x轴）：找到与target_x差值最小的shelf_x元素的索引
    int row = 0;
    double min_x_diff = std::abs(target_x - shelf_x[0]);
    for (size_t i = 1; i < shelf_x.size(); ++i) {
        double diff = std::abs(target_x - shelf_x[i]);
        if (diff < min_x_diff) {
            min_x_diff = diff;
            row = i;
        }
    }
    
    // 返回索引值（从0开始计数）
    return std::make_tuple(layer , column, row );
}
