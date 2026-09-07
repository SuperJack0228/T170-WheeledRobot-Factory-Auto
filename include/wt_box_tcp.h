#ifndef WT_BOX_TCP_H
#define WT_BOX_TCP_H

#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <iostream>
#include <cstring>
#include <sstream>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <jsoncpp/json/json.h>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <limits>
#include <map>

// 箱体位姿数据结构
struct BoxPose {
    int class_id;                    // 类别ID
    std::string class_name;          // 类别名称
    double confidence;               // 置信度
    
    // 4x4 位姿矩阵 (行优先)
    double pose_in_robot[16];        // 机器人坐标系下的位姿
    double poses_in_cam[16];         // 相机坐标系下的位姿
    
    // 位置和姿态 (6自由度)
    double x, y, z;                  // 位置 (米)
    double roll, pitch, yaw;         // 姿态 (度)
    
    // 可选的抓取点
    std::vector<double> grasps;      // 抓取点信息
};

// 协议数据包
struct ProtocolData {
    double timestamp;                 // 时间戳
    int num_objects;                 // 目标数量
    std::vector<BoxPose> objects;    // 目标列表
};

// TCP客户端类
class WTBoxTCPClient {
public:
    WTBoxTCPClient();
    ~WTBoxTCPClient();
    
    // 连接到服务器
    bool connect(const std::string& server_ip, int port);
    
    // 断开连接
    void disconnect();
    
    // 是否已连接
    bool isConnected() const;
    
    // 请求最新位姿数据
    bool requestLatestPose(ProtocolData& data);
    
    // 请求最新位姿数据（简化版，只获取第一个目标）
    bool requestLatestPose(BoxPose& pose);
    
    // 发送命令
    bool sendCommand(const std::string& cmd);
    
    // 获取错误信息
    std::string getLastError() const;
    
private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};


// 毫米到米的转换因子
const double MM_TO_M = 0.001;
const double M_TO_MM = 1000.0;

// 箱子尺寸结构体（毫米单位）
struct BoxSize {
    std::string class_name;
    double width_mm;    // 宽度（毫米）
    double height_mm;  // 高度（毫米）
    std::string description;
};

// 主处理函数声明
bool processAndSelectBoxMM(
    WTBoxTCPClient& client, 
    double result1[3],       // 输出：第一个点坐标（米）
    double result2[3],       // 输出：第二个点坐标（米）
    bool verbose = true,
    double min_confidence = 0.5
);



// 辅助函数声明
BoxSize getBoxInfoByClassName(const std::string& class_name, bool verbose = true);
void initializeBoxSizes();

#endif // WT_BOX_TCP_H