#include "wt_box_tcp.h"


class WTBoxTCPClient::Impl {
public:
    Impl() : sock_fd(-1), connected(false) {}
    
    ~Impl() {
        disconnect();
    }
    
    bool connect(const std::string& server_ip, int port) {
        if (connected) {
            disconnect();
        }
        
        // 创建socket
        sock_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (sock_fd < 0) {
            last_error = "创建socket失败";
            return false;
        }
        
        // 设置socket选项
        int flag = 1;
        setsockopt(sock_fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
        
        // 不设置超时，永久等待
        
        // 连接服务器
        struct sockaddr_in server_addr;
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port);
        server_addr.sin_addr.s_addr = inet_addr(server_ip.c_str());
        
        if (::connect(sock_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            last_error = "连接服务器失败: " + std::string(strerror(errno));
            close(sock_fd);
            sock_fd = -1;
            return false;
        }
        
        connected = true;
        last_error.clear();
        std::cout << "[TCP Client] 已连接到服务器 " << server_ip << ":" << port << std::endl;
        
        return true;
    }
    
    void disconnect() {
        if (sock_fd >= 0) {
            close(sock_fd);
            sock_fd = -1;
        }
        connected = false;
        std::cout << "[TCP Client] 已断开连接" << std::endl;
    }
    
    bool isConnected() const {
        return connected && sock_fd >= 0;
    }
    
    bool sendCommand(const std::string& cmd) {
        if (!isConnected()) {
            last_error = "未连接到服务器";
            return false;
        }
        
        std::string cmd_with_newline = cmd + "\n";
        ssize_t sent = send(sock_fd, cmd_with_newline.c_str(), cmd_with_newline.length(), 0);
        if (sent < 0) {
            last_error = "发送命令失败: " + std::string(strerror(errno));
            connected = false;
            return false;
        }
        
        return true;
    }
    
    bool receiveResponse(std::string& response) {
        if (!isConnected()) {
            last_error = "未连接到服务器";
            return false;
        }
        
        char buffer[65536];
        // 永久等待，直到收到数据或连接断开
        ssize_t received = recv(sock_fd, buffer, sizeof(buffer) - 1, 0);
        
        if (received < 0) {
            last_error = "接收数据失败: " + std::string(strerror(errno));
            connected = false;
            return false;
        }
        
        if (received == 0) {
            last_error = "服务器已断开连接";
            connected = false;
            return false;
        }
        
        buffer[received] = '\0';
        response = buffer;
        
        // 去除末尾的换行符
        if (!response.empty() && response.back() == '\n') {
            response.pop_back();
        }
        
        return true;
    }
    
    bool requestLatestPose(ProtocolData& data) {
        // 发送获取位姿命令
        if (!sendCommand("get_pose")) {
            return false;
        }
        
        // 接收响应（永久等待）
        std::string response;
        if (!receiveResponse(response)) {
            return false;
        }
        
        // 解析JSON
        return parseJSON(response, data);
    }
    
    bool parseJSON(const std::string& json_str, ProtocolData& data) {
        Json::Value root;
        Json::CharReaderBuilder builder;
        std::string errors;
        
        std::istringstream iss(json_str);
        if (!Json::parseFromStream(builder, iss, &root, &errors)) {
            last_error = "JSON解析失败: " + errors;
            return false;
        }
        
        // 检查是否有错误
        if (root.isMember("error")) {
            last_error = "服务器返回错误: " + root["error"].asString();
            return false;
        }
        
        // 解析数据
        data.timestamp = root.get("timestamp", 0.0).asDouble();
        data.num_objects = root.get("num_objects", 0).asInt();
        
        data.objects.clear();
        
        if (root.isMember("objects") && root["objects"].isArray()) {
            for (const auto& obj : root["objects"]) {
                BoxPose pose;
                pose.class_id = obj.get("class_id", 0).asInt();
                pose.class_name = obj.get("class_name", "unknown").asString();
                pose.confidence = obj.get("confidence", 0.0).asDouble();
                
                // 解析4x4矩阵
                if (obj.isMember("pose_in_robot") && obj["pose_in_robot"].isArray()) {
                    for (int i = 0; i < 16 && i < (int)obj["pose_in_robot"].size(); i++) {
                        pose.pose_in_robot[i] = obj["pose_in_robot"][i].asDouble();
                    }
                }
                
                if (obj.isMember("poses_in_cam") && obj["poses_in_cam"].isArray()) {
                    for (int i = 0; i < 16 && i < (int)obj["poses_in_cam"].size(); i++) {
                        pose.poses_in_cam[i] = obj["poses_in_cam"][i].asDouble();
                    }
                }
                
                // 解析位置和姿态
                if (obj.isMember("xyz") && obj["xyz"].isArray() && obj["xyz"].size() >= 3) {
                    pose.x = obj["xyz"][0].asDouble();
                    pose.y = obj["xyz"][1].asDouble();
                    pose.z = obj["xyz"][2].asDouble();
                }
                
                if (obj.isMember("rpy_deg") && obj["rpy_deg"].isArray() && obj["rpy_deg"].size() >= 3) {
                    pose.roll = obj["rpy_deg"][0].asDouble();
                    pose.pitch = obj["rpy_deg"][1].asDouble();
                    pose.yaw = obj["rpy_deg"][2].asDouble();
                }
                
                data.objects.push_back(pose);
            }
        }
        
        last_error.clear();
        return true;
    }
    
    std::string getLastError() const {
        return last_error;
    }
    
private:
    int sock_fd;
    bool connected;
    std::string last_error;
};

// 公共接口实现
WTBoxTCPClient::WTBoxTCPClient() : pImpl(std::make_unique<Impl>()) {}
WTBoxTCPClient::~WTBoxTCPClient() = default;

bool WTBoxTCPClient::connect(const std::string& server_ip, int port) {
    return pImpl->connect(server_ip, port);
}

void WTBoxTCPClient::disconnect() {
    pImpl->disconnect();
}

bool WTBoxTCPClient::isConnected() const {
    return pImpl->isConnected();
}

bool WTBoxTCPClient::requestLatestPose(ProtocolData& data) {
    return pImpl->requestLatestPose(data);
}

bool WTBoxTCPClient::requestLatestPose(BoxPose& pose) {
    ProtocolData data;
    if (!pImpl->requestLatestPose(data)) {
        return false;
    }
    if (data.objects.empty()) {
        return false;
    }
    pose = data.objects[0];
    return true;
}

bool WTBoxTCPClient::sendCommand(const std::string& cmd) {
    return pImpl->sendCommand(cmd);
}

std::string WTBoxTCPClient::getLastError() const {
    return pImpl->getLastError();
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////










// 使用map提高查找效率
std::map<std::string, BoxSize> boxSizesMap;

// 初始化箱子尺寸数据库
void initializeBoxSizes() {
    boxSizesMap = {
        {"big_570x370", {"big_570x370", 570.0, 370.0, "蓝色物流箱(570x370)"}},
        {"medium_370x270", {"medium_370x270", 370.0, 270.0, "中号箱(370x270)"}},
        {"medium_360x270", {"medium_360x270", 360.0, 270.0, "中号箱(360x270)"}},
        {"small_300x220", {"small_300x220", 300.0, 220.0, "小号箱(300x220)"}}
    };
}

// 计算距离的模板函数（毫米单位）
template<typename T>
double calculateDistance(T x, T y, T z) {
    return sqrt(x*x + y*y + z*z);
}

// 显式实例化模板函数
template double calculateDistance<double>(double, double, double);
template double calculateDistance<float>(float, float, float);

// 根据class_name获取箱子信息
BoxSize getBoxInfoByClassName(const std::string& class_name, bool verbose) {
    if (boxSizesMap.empty()) {
        initializeBoxSizes();
    }
    
    auto it = boxSizesMap.find(class_name);
    if (it != boxSizesMap.end()) {
        return it->second;
    }
    
    // 如果没有找到，尝试模糊匹配
    for (const auto& pair : boxSizesMap) {
        if (pair.first.find(class_name) != std::string::npos || 
            class_name.find(pair.first) != std::string::npos) {
            if (verbose) {
                std::cout << "注意: 使用近似匹配 '" << pair.first 
                         << "' 代替 '" << class_name << "'" << std::endl;
            }
            return pair.second;
        }
    }
    
    // 仍然没有找到，返回默认值
    if (verbose) {
        std::cerr << "警告: 未找到类名 '" << class_name 
                 << "' 对应的箱子尺寸，使用默认中号箱尺寸" << std::endl;
    }
    return {"default", 370.0, 270.0, "默认中号箱"};
}

// 毫米单位的主处理函数
bool processAndSelectBoxMM(
    WTBoxTCPClient& client, 
    double result1[3],
    double result2[3],
    bool verbose,
    double min_confidence
) {
    ProtocolData data;
    
    if (!client.requestLatestPose(data)) {
        std::cerr << "获取数据失败: " << client.getLastError() << std::endl;
        return false;
    }
    
    if (verbose) {
        std::cout << std::fixed << std::setprecision(3) 
                 << "时间戳: " << data.timestamp << std::endl;
        std::cout << "检测到 " << data.num_objects << " 个目标" << std::endl;
    }
    
    if (data.num_objects == 0) {
        if (verbose) std::cout << "没有检测到任何目标" << std::endl;
        return false;
    }
    
    // 过滤低置信度的目标
    std::vector<int> valid_indices;
    for (int i = 0; i < data.num_objects; i++) {
        if (data.objects[i].confidence >= min_confidence) {
            valid_indices.push_back(i);
        }
    }
    
    if (valid_indices.empty()) {
        if (verbose) {
            std::cout << "没有置信度超过 " << min_confidence << " 的目标" << std::endl;
        }
        return false;
    }
    
    if (verbose && valid_indices.size() != data.num_objects) {
        std::cout << "过滤后剩余 " << valid_indices.size() << " 个有效目标" << std::endl;
    }
    
    // // 打印目标信息
    // if (verbose) {
    //     for (int idx : valid_indices) {
    //         const auto& obj = data.objects[idx];
    //         std::cout << "  [" << obj.class_id << "] " << obj.class_name
    //                  << " (置信度: " << std::fixed << std::setprecision(2) << obj.confidence << ")" << std::endl;
    //         std::cout << "    位置: x=" << obj.x << "mm, y=" << obj.y << "mm, z=" << obj.z << "mm" << std::endl;
    //         std::cout << "    姿态: roll=" << obj.roll << "°, pitch=" << obj.pitch
    //                  << "°, yaw=" << obj.yaw << "°" << std::endl;
    //     }
    //     std::cout << "----------------------------------------" << std::endl;
    // }
    
    // 选择目标
    int selected_index_in_valid = 0;
    
    if (valid_indices.size() > 1) {
        // 如果有多个有效目标，选择距离原点最近的一个
        double min_distance = std::numeric_limits<double>::max();
        
        for (int i = 0; i < valid_indices.size(); i++) {
            int idx = valid_indices[i];
            const auto& obj = data.objects[idx];
            // 计算距离（毫米单位）
            double distance_mm = calculateDistance(obj.x, obj.y, obj.z);
            double distance_m = distance_mm * MM_TO_M;
            
            if (verbose) {
                std::cout << "目标 " << i+1 << " (" << obj.class_name 
                         << ") 距离: " << std::fixed << std::setprecision(1) 
                         << distance_mm << "mm (" << std::fixed << std::setprecision(3)
                         << distance_m << "m)" << std::endl;
            }
            
            if (distance_mm < min_distance) {
                min_distance = distance_mm;
                selected_index_in_valid = i;
            }
        }
        
        if (verbose) {
            int selected_idx = valid_indices[selected_index_in_valid];
            std::cout << "选择距离最近的目标: " << selected_idx+1 
                     << " (" << data.objects[selected_idx].class_name << ")" << std::endl;
        }
    }
    
    // 获取原始索引
    int selected_index = valid_indices[selected_index_in_valid];
    const auto& selected_obj = data.objects[selected_index];
    
    // if (verbose) {
    //     std::cout << "已选择: " << selected_obj.class_name 
    //              << " (置信度: " << std::fixed << std::setprecision(2) 
    //              << selected_obj.confidence << ")" << std::endl;
    // }
    
    // 获取箱子信息
    BoxSize box_info = getBoxInfoByClassName(selected_obj.class_name, verbose);
    
    // if (verbose) {
    //     std::cout << "箱子尺寸: " << box_info.width_mm << "x" << box_info.height_mm << "mm" << std::endl;
    //     if (!box_info.description.empty()) {
    //         std::cout << "描述: " << box_info.description << std::endl;
    //     }
    // }
    
    // 计算y的偏移值（毫米单位）
    double y_offset_mm = box_info.width_mm / 2.0;
    
    // 计算两个点的坐标（毫米单位）
    double point1_mm[3] = {
        selected_obj.x,                     // x保持不变
        selected_obj.y - y_offset_mm,       // y减去宽度的一半
        selected_obj.z                      // z保持不变
    };
    
    double point2_mm[3] = {
        selected_obj.x,                     // x保持不变
        selected_obj.y + y_offset_mm,       // y加上宽度的一半
        selected_obj.z                      // z保持不变
    };
    
    // 转换为米单位输出
    result1[0] = point1_mm[0] * MM_TO_M;
    result1[1] = point1_mm[1] * MM_TO_M;
    result1[2] = point1_mm[2] * MM_TO_M;
    
    result2[0] = point2_mm[0] * MM_TO_M;
    result2[1] = point2_mm[1] * MM_TO_M;
    result2[2] = point2_mm[2] * MM_TO_M;
    
    // 打印结果
    if (verbose) {
        // std::cout << std::fixed << std::setprecision(1);
        // std::cout << "计算的两个点（毫米单位）:" << std::endl;
        // std::cout << "点1 (y-" << y_offset_mm << "mm): [" 
        //           << point1_mm[0] << ", " << point1_mm[1] << ", " << point1_mm[2] << "] mm" << std::endl;
        // std::cout << "点2 (y+" << y_offset_mm << "mm): [" 
        //           << point2_mm[0] << ", " << point2_mm[2] << ", " << point2_mm[2] << "] mm" << std::endl;
        
        // std::cout << std::fixed << std::setprecision(3);
        // std::cout << "转换到米单位:" << std::endl;
        // std::cout << "点1: [" 
        //           << result1[0] << ", " << result1[1] << ", " << result1[2] << "] m" << std::endl;
        // std::cout << "点2: [" 
        //           << result2[0] << ", " << result2[1] << ", " << result2[2] << "] m" << std::endl;
    }
    
    return true;
}








