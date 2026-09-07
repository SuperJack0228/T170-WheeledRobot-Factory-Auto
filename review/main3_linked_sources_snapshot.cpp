/**
 * main3 搬箱工程 — 源码阅读快照（仅供查看，不参与编译）
 *
 * 本文件由 move 可执行目标链接的全部 .cpp 原样拼接而成，对应 CMakeLists.txt SRC_FILES。
 * 请勿在本工程中编译此文件；修改逻辑请改 src/ 下各源文件。
 *
 * 模块顺序（main 在最后）：
 *   aoyihand.cpp, function.cpp, arm_robot.cpp, waist.cpp, motor_driver.cpp,
 *   reals_tcp.cpp, Ti5_socketcan.cpp, wt_client.cpp, wt_box_tcp.cpp,
 *   lanxincontrol.cpp, gripper_interface.cpp, tts_http_client.cpp,
 *   realsense_get.cpp, seg_pose_bridge.cpp, main3.cpp
 */


// =============================================================================
// BEGIN: src/aoyihand.cpp
// =============================================================================
#include "aoyihand.h"

serial::Serial *serial_port = nullptr;
serial::Serial *serial_port2 = nullptr;

serial::Serial *right_port = nullptr;
serial::Serial *left_port = nullptr;

uint8_t aoyi_right_id = 0;
uint8_t aoyi_left_id = 0;

namespace
{
/** 去掉 \r\n 与首尾空白，避免设备路径尾部空格导致 open ENOENT */
void trimConfigLine(string &line)
{
    if (!line.empty() && line.back() == '\r')
        line.pop_back();
    if (!line.empty() && line.back() == '\n')
        line.pop_back();
    size_t start = line.find_first_not_of(" \t");
    if (start == string::npos)
        line.clear();
    else
    {
        size_t end = line.find_last_not_of(" \t");
        line = line.substr(start, end - start + 1);
    }
}
}
// std::unique_ptr<serial::Serial> serial_port=nullptr;
// std::unique_ptr<serial::Serial> serial_port2=nullptr;

// std::unique_ptr<serial::Serial> right_port=nullptr;
// std::unique_ptr<serial::Serial> left_port=nullptr;

uint8_t readConfigValue(int lineNum)
{
    // 检查输入参数
    if (lineNum != 3 && lineNum != 4)
    {
        std::cerr << "Error: lineNum must be 3 or 4" << std::endl;
        return 0; // 返回0表示错误
    }

    std::ifstream configFile("config.txt");
    if (!configFile.is_open())
    {
        std::cerr << "Error: Cannot open config.txt" << std::endl;
        return 0;
    }

    std::string line;
    int currentLine = 1;

    // 逐行读取，直到找到目标行
    while (std::getline(configFile, line))
    {
        if (currentLine == lineNum)
        {
            configFile.close();
            trimConfigLine(line);

            try
            {
                // 转换为整数
                int value = std::stoi(line);

                // 检查是否在uint8_t范围内
                if (value < 0 || value > 255)
                {
                    std::cerr << "Error: Value out of range (0-255): " << value << std::endl;
                    return 0;
                }

                // cout << " int value " << value << endl;
                // uint8_t result = static_cast<uint8_t>(value);
                // printf("uint8_t value: %d\n", result);

                return static_cast<uint8_t>(value);
            }
            catch (const std::exception &e)
            {
                std::cerr << "Error: Invalid integer at line " << lineNum
                          << ": " << e.what() << std::endl;
                return 0;
            }
        }
        currentLine++;
    }

    configFile.close();
    std::cerr << "Error: Line " << lineNum << " not found" << std::endl;
    return 0;
}

int aoyi_init()
{

    const string configPath = "config.txt"; // 配置文件名
    string s = getLineFromConfig(configPath);

    if (!s.empty())
    {
        cout << "读取到的串口设备name: " << s << endl;
    }
    else
    {
        cerr << "未能从配置文件中读取到有效内容" << endl;
        return -1;
    }
    if (!initSerial(s, 0))
    {
        return -1;
    }

    string s2 = getLineFromConfig(configPath, 1);

    if (!s2.empty())
    {
        cout << "读取到的串口设备name: " << s2 << endl;
    }
    else
    {
        cerr << "未能从配置文件中读取到有效内容" << endl;
        return -1;
    }
    if (!initSerial(s2, 1))
    {
        return -1;
    }

    aoyi_right_id = readConfigValue(3);
    aoyi_left_id = readConfigValue(4);

    return 0;
}

string getLineFromConfig(const string &configPath, int lineNumber)
{
    ifstream configFile(configPath);
    vector<string> lines;
    string line;

    if (configFile.is_open())
    {
        // 读取所有行
        while (getline(configFile, line))
        {
            trimConfigLine(line);
            lines.push_back(line);
        }
        configFile.close();

        // 检查请求的行号是否有效
        if (lineNumber >= 0 && lineNumber < lines.size())
        {
            return lines[lineNumber];
        }
        else
        {
            cerr << "无效的行号: " << lineNumber << endl;
            return "";
        }
    }
    else
    {
        cerr << "无法打开配置文件: " << configPath << endl;
        return "";
    }
}

// 工具函数
uint8_t calculateLRC(const uint8_t *data, size_t length)
{
    return accumulate(data, data + length, 0, [](uint8_t sum, uint8_t byte)
                      { return sum ^ byte; });
}

// 发送命令并接收响应
vector<uint8_t> sendHandCommand(uint8_t handID, uint8_t masterID, uint8_t command, const vector<uint8_t> &data, aoyi_hand side)
{
    vector<uint8_t> packet;
    vector<uint8_t> response;

    // 构建数据包
    packet.push_back(PROTOCOL_HEADER_1);
    packet.push_back(PROTOCOL_HEADER_2);
    packet.push_back(handID);
    packet.push_back(masterID);
    packet.push_back(command);
    packet.push_back(static_cast<uint8_t>(data.size()));
    packet.insert(packet.end(), data.begin(), data.end());

    // 计算校验和
    uint8_t checksum = calculateLRC(packet.data() + 2, packet.size() - 2);
    packet.push_back(checksum);

    //     printf("生成的数据包 (%zu 字节): ", packet.size());
    // for (size_t i = 0; i < packet.size(); i++) {
    //     printf("%02X ", packet[i]);
    // }
    // printf("\n");

    // 发送数据
    try
    {
        if (side == right_a)
        {

            if (right_port && right_port->isOpen())
            {
                // 发送命令
                size_t bytesWritten = right_port->write(packet);
                if (bytesWritten != packet.size())
                {
                    cerr << "Failed to send complete packet" << endl;
                    return response;
                }
                {
                    std::vector<uint8_t> buffer(64);
                    size_t bytesRead = right_port->read(buffer.data(), buffer.size());

                    if (bytesRead > 0)
                    {
                        // 十六进制打印
                        // std::cout << "十六进制 (" << bytesRead << " bytes):" << std::endl;
                        // for (size_t i = 0; i < bytesRead; ++i)
                        // {
                        //     printf("%02X ", buffer[i]);
                        //     if ((i + 1) % 16 == 0)
                        //         std::cout << std::endl;
                        // }
                        // if (bytesRead % 16 != 0)
                        //     std::cout << std::endl;

                        // // 原始格式打印
                        // std::cout << "\n原始格式 (" << bytesRead << " bytes):" << std::endl;
                        // for (size_t i = 0; i < bytesRead; ++i)
                        // {
                        //     printf("%d ", buffer[i]); // 十进制数字打印
                        // }
                        // std::cout << std::endl;
                         return buffer;
                    }
                    else
                    {
                         std::vector<uint8_t> buffer2;
                          return buffer2;
                        cout << "没有收到消息" << endl;
                    }
                   
                }

                // 读取响应
                // 首先读取固定长度的包头和基本信息(6字节)
                vector<uint8_t> header(6);
                size_t bytesRead = right_port->read(header.data(), header.size());

                if (bytesRead == header.size())
                {
                    // 检查包头
                    if (header[0] != PROTOCOL_HEADER_1 || header[1] != PROTOCOL_HEADER_2)
                    {
                        // cerr << "Invalid response header" << endl;
                        return response;
                    }

                    // 获取数据长度
                    uint8_t dataLen = header[5];
                    if (dataLen > 0)
                    {
                        vector<uint8_t> responseData(dataLen);
                        bytesRead = right_port->read(responseData.data(), dataLen);
                        if (bytesRead != dataLen)
                        {
                            cerr << "Incomplete response data" << endl;
                            return response;
                        }

                        // 读取校验和
                        uint8_t responseChecksum;
                        bytesRead = right_port->read(&responseChecksum, 1);
                        if (bytesRead != 1)
                        {
                            cerr << "Failed to read checksum" << endl;
                            return response;
                        }

                        // 验证校验和
                        vector<uint8_t> toCheck(header.begin() + 2, header.end());
                        toCheck.insert(toCheck.end(), responseData.begin(), responseData.end());
                        uint8_t calculatedChecksum = calculateLRC(toCheck.data(), toCheck.size());

                        if (calculatedChecksum != responseChecksum)
                        {
                            cerr << "Checksum mismatch" << endl;
                            return response;
                        }

                        // 构建完整响应
                        response = header;
                        response.insert(response.end(), responseData.begin(), responseData.end());
                        response.push_back(responseChecksum);
                    }
                    else
                    {
                        // 无数据部分的响应
                        uint8_t responseChecksum;
                        bytesRead = right_port->read(&responseChecksum, 1);
                        if (bytesRead != 1)
                        {
                            cerr << "Failed to read checksum" << endl;
                            return response;
                        }

                        // 验证校验和
                        uint8_t calculatedChecksum = calculateLRC(header.data() + 2, header.size() - 2);

                        if (calculatedChecksum != responseChecksum)
                        {
                            cerr << "Checksum mismatch" << endl;
                            return response;
                        }

                        response = header;
                        response.push_back(responseChecksum);
                    }
                }
            }
        }
        else
        {

            if (left_port && left_port->isOpen())
            {
                // 发送命令
                size_t bytesWritten = left_port->write(packet);
                if (bytesWritten != packet.size())
                {
                    cerr << "Failed to send complete packet" << endl;
                    return response;
                }
                {
                    std::vector<uint8_t> buffer(64);
                    size_t bytesRead = left_port->read(buffer.data(), buffer.size());

                    if (bytesRead > 0)
                    {
                        // 十六进制打印
                        // std::cout << "十六进制 (" << bytesRead << " bytes):" << std::endl;
                        // for (size_t i = 0; i < bytesRead; ++i)
                        // {
                        //     printf("%02X ", buffer[i]);
                        //     if ((i + 1) % 16 == 0)
                        //         std::cout << std::endl;
                        // }
                        // if (bytesRead % 16 != 0)
                        //     std::cout << std::endl;

                        // // 原始格式打印
                        // std::cout << "\n原始格式 (" << bytesRead << " bytes):" << std::endl;
                        // for (size_t i = 0; i < bytesRead; ++i)
                        // {
                        //     printf("%d ", buffer[i]); // 十进制数字打印
                        // }
                        // std::cout << std::endl;

                           return buffer;
                    }
                    else
                    {
                         std::vector<uint8_t> buffer2;
                          return buffer2;
                        cout << "没有收到消息" << endl;
                    }
                }

                // 读取响应
                // 首先读取固定长度的包头和基本信息(6字节)
                vector<uint8_t> header(6);
                size_t bytesRead = left_port->read(header.data(), header.size());

                if (bytesRead == header.size())
                {
                    // 检查包头
                    if (header[0] != PROTOCOL_HEADER_1 || header[1] != PROTOCOL_HEADER_2)
                    {
                        // cerr << "Invalid response header" << endl;
                        return response;
                    }

                    // 获取数据长度
                    uint8_t dataLen = header[5];
                    if (dataLen > 0)
                    {
                        vector<uint8_t> responseData(dataLen);
                        bytesRead = left_port->read(responseData.data(), dataLen);
                        if (bytesRead != dataLen)
                        {
                            cerr << "Incomplete response data" << endl;
                            return response;
                        }

                        // 读取校验和
                        uint8_t responseChecksum;
                        bytesRead = left_port->read(&responseChecksum, 1);
                        if (bytesRead != 1)
                        {
                            cerr << "Failed to read checksum" << endl;
                            return response;
                        }

                        // 验证校验和
                        vector<uint8_t> toCheck(header.begin() + 2, header.end());
                        toCheck.insert(toCheck.end(), responseData.begin(), responseData.end());
                        uint8_t calculatedChecksum = calculateLRC(toCheck.data(), toCheck.size());

                        if (calculatedChecksum != responseChecksum)
                        {
                            cerr << "Checksum mismatch" << endl;
                            return response;
                        }

                        // 构建完整响应
                        response = header;
                        response.insert(response.end(), responseData.begin(), responseData.end());
                        response.push_back(responseChecksum);
                    }
                    else
                    {
                        // 无数据部分的响应
                        uint8_t responseChecksum;
                        bytesRead = left_port->read(&responseChecksum, 1);
                        if (bytesRead != 1)
                        {
                            cerr << "Failed to read checksum" << endl;
                            return response;
                        }

                        // 验证校验和
                        uint8_t calculatedChecksum = calculateLRC(header.data() + 2, header.size() - 2);

                        if (calculatedChecksum != responseChecksum)
                        {
                            cerr << "Checksum mismatch" << endl;
                            return response;
                        }

                        response = header;
                        response.push_back(responseChecksum);
                    }
                }
            }
        }
    }
    catch (const exception &e)
    {
        cerr << "Serial communication error: " << e.what() << endl;
    }

    return response;
}

vector<uint8_t> verify_hand(uint8_t handID, uint8_t masterID, uint8_t command, serial::Serial *hand_port)
{
    vector<uint8_t> packet;
    vector<uint8_t> response;

    // 构建数据包
    packet.push_back(PROTOCOL_HEADER_1);
    packet.push_back(PROTOCOL_HEADER_2);
    packet.push_back(handID);
    packet.push_back(masterID);
    packet.push_back(command);
    packet.push_back(0);

    // 计算校验和
    uint8_t checksum = calculateLRC(packet.data() + 2, packet.size() - 2);
    packet.push_back(checksum);

    // 发送数据
    try
    {

        if (hand_port && hand_port->isOpen())
        {
            // 发送命令
            size_t bytesWritten = hand_port->write(packet);
            if (bytesWritten != packet.size())
            {
                cerr << "Failed to send complete packet" << endl;
                return response;
            }
            sleep(1);

            {
                std::vector<uint8_t> buffer(64);
                size_t bytesRead = hand_port->read(buffer.data(), buffer.size());

                if (bytesRead > 0)
                {
                    // // // 十六进制打印
                    // std::cout << "十六进制 (" << bytesRead << " bytes):" << std::endl;
                    // for (size_t i = 0; i < bytesRead; ++i)
                    // {
                    //     printf("%02X ", buffer[i]);
                    //     if ((i + 1) % 16 == 0)
                    //         std::cout << std::endl;
                    // }
                    // if (bytesRead % 16 != 0)
                    //     std::cout << std::endl;

                    cout << "bytesRead: " << bytesRead << endl;

                    // // 原始格式打印
                    // std::cout << "\n原始格式 (" << bytesRead << " bytes):" << std::endl;
                    // for (size_t i = 0; i < bytesRead; ++i)
                    // {
                    //     printf("%d ", buffer[i]); // 十进制数字打印
                    // }
                    // std::cout << std::endl;
                      cout << "收到消息" << endl;
                        return buffer;
                    }
                    else
                    {
                         std::vector<uint8_t> buffer2;
                          cout << "没有收到消息" << endl;
                          return buffer2;
                       
                    }
            }

            // 读取响应
            // 首先读取固定长度的包头和基本信息(6字节)
            vector<uint8_t> header(6);
            size_t bytesRead = hand_port->read(header.data(), header.size());

            if (bytesRead == header.size())
            {
                // 检查包头
                if (header[0] != PROTOCOL_HEADER_1 || header[1] != PROTOCOL_HEADER_2)
                {
                    cerr << "Invalid response header" << endl;
                    return response;
                }

                // 获取数据长度
                uint8_t dataLen = header[5];
                if (dataLen > 0)
                {

                    vector<uint8_t> responseData(dataLen);
                    bytesRead = hand_port->read(responseData.data(), dataLen);
                    if (bytesRead != dataLen)
                    {
                        cerr << "Incomplete response data" << endl;
                        return response;
                    }

                    // 读取校验和
                    uint8_t responseChecksum;
                    bytesRead = hand_port->read(&responseChecksum, 1);
                    if (bytesRead != 1)
                    {
                        cerr << "Failed to read checksum" << endl;
                        return response;
                    }

                    // 验证校验和
                    vector<uint8_t> toCheck(header.begin() + 2, header.end());
                    toCheck.insert(toCheck.end(), responseData.begin(), responseData.end());
                    uint8_t calculatedChecksum = calculateLRC(toCheck.data(), toCheck.size());

                    if (calculatedChecksum != responseChecksum)
                    {
                        cerr << "Checksum mismatch" << endl;
                        return response;
                    }

                    // 构建完整响应
                    response = header;
                    response.insert(response.end(), responseData.begin(), responseData.end());
                    response.push_back(responseChecksum);
                }
                else
                {
                    // 无数据部分的响应
                    uint8_t responseChecksum;
                    bytesRead = hand_port->read(&responseChecksum, 1);
                    if (bytesRead != 1)
                    {
                        cerr << "Failed to read checksum" << endl;
                        return response;
                    }

                    // 验证校验和
                    uint8_t calculatedChecksum = calculateLRC(header.data() + 2, header.size() - 2);

                    if (calculatedChecksum != responseChecksum)
                    {
                        cerr << "Checksum mismatch" << endl;
                        return response;
                    }

                    response = header;
                    response.push_back(responseChecksum);
                }
            }
        }
    }
    catch (const exception &e)
    {
        cerr << "Serial communication error: " << e.what() << endl;
    }

    return response;
}

// 初始化串口
bool initSerial(const string &portName, int num)
{

    sleep(1);
    if (num == 0)
    {
        try
        {
            serial_port = new serial::Serial(portName, 115200, serial::Timeout::simpleTimeout(1000));
            if (!serial_port->isOpen())
            {
                cerr << "Failed to open serial port" << endl;
                delete serial_port;
                serial_port = nullptr;
                return false;
            }
            return true;
        }
        catch (const exception &e)
        {
            cerr << "Serial port error: " << e.what() << endl;
            return false;
        }
    }
    else
    {
        try
        {
            serial_port2 = new serial::Serial(portName, 115200, serial::Timeout::simpleTimeout(1000));
            if (!serial_port2->isOpen())
            {
                cerr << "Failed to open serial port" << endl;
                delete serial_port2;
                serial_port2 = nullptr;
                return false;
            }
            return true;
        }
        catch (const exception &e)
        {
            cerr << "Serial port error: " << e.what() << endl;
            return false;
        }
    }
}

// 清理资源
void cleanup()
{
    // if (serial_port)
    // {

    // cout << "serial_port" << serial_port << endl;
    // cout << "serial_port2" << serial_port2 << endl;
    // cout << "right_port" << right_port << endl;
    // cout << "left_port" << left_port << endl;

    //     serial_port->close();
    //     delete serial_port;
    //     serial_port = nullptr;
    //     cout << "serial_port close" << endl;
    // //}
    // // if (serial_port2)
    // // {

    //     serial_port2->close();
    //     delete serial_port2;
    //     serial_port2 = nullptr;
    //     cout << "serial_port2 close" << endl;
    // //}
    // right_port = nullptr;
    // left_port = nullptr;

    if (right_port)
    {

        // 步骤1：正常关闭
        usleep(100000);
        try
        {
            if (right_port->isOpen())
            {
                right_port->close(); // 正式关闭

                right_port->flush(); // 清空缓冲区
            }
        }
        catch (...)
        {
        }

        // 步骤2：强制内核释放
        // system(("sudo bash -c 'exec 3<>/dev/" +
        //         std::string(right_port->getPort().substr(5)) +
        //         " && exec 3>&-'")
        //            .c_str());

        // 步骤3：释放内存
        delete right_port;
        right_port = nullptr;

        serial_port = nullptr;
        serial_port2 = nullptr;

        // 步骤4：短暂延迟
        usleep(100000); // 100ms

        cout << "右手关闭" << endl;
    }

    if (left_port)
    {

        // 步骤1：正常关闭
        usleep(100000);
        try
        {
            if (left_port->isOpen())
            {
                left_port->flush(); // 清空缓冲区
                left_port->close(); // 正式关闭
            }
        }
        catch (...)
        {
        }

        // 步骤2：强制内核释放
        // system(("sudo bash -c 'exec 3<>/dev/" +
        //         std::string(left_port->getPort().substr(5)) +
        //         " && exec 3>&-'")
        //            .c_str());

        // 步骤3：释放内存
        delete left_port;
        left_port = nullptr;

        serial_port = nullptr;
        serial_port2 = nullptr;

        // 步骤4：短暂延迟
        usleep(100000); // 100ms

        cout << "左手关闭" << endl;
    }
}

void aoyizhuaqu(aoyi_hand side)
{
    // 固定参数

    uint8_t handID = 0;
    if (side == right_a)
    {
        handID = aoyi_right_id;
    }
    else
    {
        handID = aoyi_left_id;
    }

    uint8_t masterID = 0x01;
    uint8_t command = 0x4c;

    vector<uint8_t> data1;

    const uint16_t value = 64000;

    data1.push_back(5);
    data1.push_back(value & 0xFF);        // 低字节
    data1.push_back((value >> 8) & 0xFF); // 高字节
    data1.push_back(255);                 // 固定值200

    vector<uint8_t> response1 = sendHandCommand(handID, masterID, command, data1, side);

    // 显示响应
    if (!response1.empty())
    {
        // cout << "收到响应: ";
        // for (uint8_t byte : response)
        // {
        //     cout << "0x" << hex << (int)byte << " ";
        // }
        // cout << endl;
    }
    else
    {
        cerr << "未收到有效响应" << endl;
    }

    sleep(1);

    // 准备数据
    const uint16_t values[6] = {60000, 60000, 60000, 60000, 60000, 64000};
    // const uint16_t values[6] = {0, 0, 0, 0, 0, 0};
    vector<uint8_t> data;

    command = 0x50;

    for (int i = 0; i < 6; i++)
    {
        // 分解uint16_t值为两个字节
        data.push_back(values[i] & 0xFF);        // 低字节
        data.push_back((values[i] >> 8) & 0xFF); // 高字节
        data.push_back(255);                     // 固定值200
    }

    // 显示要发送的数据
    // cout << "准备发送以下数据:" << endl;
    // cout << "灵巧手ID: 0x" << hex << (int)handID << endl;
    // cout << "主机ID: 0x" << hex << (int)masterID << endl;
    // cout << "命令: 0x" << hex << (int)command << endl;
    // cout << "数据: ";
    // for (size_t i = 0; i < data.size(); i++)
    // {
    //     cout << "0x" << hex << (int)data[i] << " ";
    //     if ((i + 1) % 3 == 0)
    //         cout << "| "; // 每组数据用竖线分隔
    // }
    // cout << endl;

    // 发送命令
    vector<uint8_t> response = sendHandCommand(handID, masterID, command, data, side);

    // 显示响应
    if (!response.empty())
    {
        // cout << "收到响应: ";
        // for (uint8_t byte : response)
        // {
        //     cout << "0x" << hex << (int)byte << " ";
        // }
        // cout << endl;
    }
    else
    {
        cerr << "未收到有效响应" << endl;
    }
}

void aoyisong(aoyi_hand side)
{
    uint8_t handID = 0;
    if (side == right_a)
    {
        handID = aoyi_right_id;
    }
    else
    {
        handID = aoyi_left_id;
    }
    uint8_t masterID = 0x01;
    uint8_t command = 0x4F;

    // 准备数据
    // const uint16_t values[6] = {2500, 20000, 20000, 20000, 20000, 0};
    const uint16_t values[6] = {0, 0, 0, 0, 0, 0};
    vector<uint8_t> data;

    for (int i = 0; i < 6; i++)
    {
        // 分解uint16_t值为两个字节
        data.push_back(values[i] & 0xFF);        // 低字节
        data.push_back((values[i] >> 8) & 0xFF); // 高字节
        data.push_back(255);                     // 固定值200
    }

    // 显示要发送的数据
    // cout << "准备发送以下数据:" << endl;
    // cout << "灵巧手ID: 0x" << hex << (int)handID << endl;
    // cout << "主机ID: 0x" << hex << (int)masterID << endl;
    // cout << "命令: 0x" << hex << (int)command << endl;
    // cout << "数据: ";
    // for (size_t i = 0; i < data.size(); i++)
    // {
    //     cout << "0x" << hex << (int)data[i] << " ";
    //     if ((i + 1) % 3 == 0)
    //         cout << "| "; // 每组数据用竖线分隔
    // }
    // cout << endl;

    // 发送命令
    vector<uint8_t> response = sendHandCommand(handID, masterID, command, data, side);

    // 显示响应
    if (!response.empty())
    {
        // cout << "收到响应: ";
        // for (uint8_t byte : response)
        // {
        //     cout << "0x" << hex << (int)byte << " ";
        // }
        // cout << endl;
    }
    else
    {
        cerr << "未收到有效响应" << endl;
    }
}

void aoyijiaodu(uint16_t values[6], uint8_t speed[6], aoyi_hand side)
{
    uint8_t handID = 0;
    if (side == right_a)
    {
        handID = aoyi_right_id;
    }
    else
    {
        handID = aoyi_left_id;
    }
    uint8_t masterID = 0x01;
    uint8_t command = 0x50;

    // 准备数据
    for (int i = 0; i < 6; i++)
    {
        // 将其中一个操作数转换为float以确保浮点除法
        float temp = static_cast<float>(values[i]) / 90.0f;
        values[i] = static_cast<uint16_t>(temp * 65535.0f);

        // 确保值不会超出范围
        if (values[i] > 65535)
            values[i] = 65535;
    }
    vector<uint8_t> data;

    for (int i = 0; i < 6; i++)
    {
        // 分解uint16_t值为两个字节
        data.push_back(values[i] & 0xFF);        // 低字节
        data.push_back((values[i] >> 8) & 0xFF); // 高字节
        data.push_back(speed[i]);                // 固定值200
    }

    // 显示要发送的数据
    // cout << "准备发送以下数据:" << endl;
    // cout << "灵巧手ID: 0x" << hex << (int)handID << endl;
    // cout << "主机ID: 0x" << hex << (int)masterID << endl;
    // cout << "命令: 0x" << hex << (int)command << endl;
    // cout << "数据: ";
    // for (size_t i = 0; i < data.size(); i++)
    // {
    //     cout << "0x" << hex << (int)data[i] << " ";
    //     if ((i + 1) % 3 == 0)
    //         cout << "| "; // 每组数据用竖线分隔
    // }
    // cout << endl;

    // 发送命令
    vector<uint8_t> response = sendHandCommand(handID, masterID, command, data, side);

    // 显示响应
    if (!response.empty())
    {
        // cout << "收到响应: ";
        // for (uint8_t byte : response)
        // {
        //     cout << "0x" << hex << (int)byte << " ";
        // }
        // cout << endl;
    }
    else
    {
        cerr << "未收到有效响应" << endl;
    }
}

void FINGER_CURRENT_LIMIT(uint16_t values[6], aoyi_hand side)
{
    uint8_t handID = 0;
    if (side == right_a)
    {
        handID = aoyi_right_id;
    }
    else
    {
        handID = aoyi_left_id;
    }
    uint8_t masterID = 0x01;
    uint8_t command = 0x46;

    // 准备数据

    // const uint16_t values[6] = {0, 0, 0, 0, 0, 0};
    vector<uint8_t> data;

    for (int i = 0; i < 6; i++)
    {
        // 分解uint16_t值为两个字节
        data.push_back(static_cast<uint8_t>(i));
        data.push_back(values[i] & 0xFF);        // 低字节
        data.push_back((values[i] >> 8) & 0xFF); // 高字节
                                                 // 固定值200

        // 显示要发送的数据
        // cout << "准备发送以下数据:" << endl;
        // cout << "灵巧手ID: 0x" << hex << (int)handID << endl;
        // cout << "主机ID: 0x" << hex << (int)masterID << endl;
        // cout << "命令: 0x" << hex << (int)command << endl;
        // cout << "数据: ";
        // for (size_t i = 0; i < data.size(); i++)
        // {
        //     cout << "0x" << hex << (int)data[i] << " ";
        //     if ((i + 1) % 3 == 0)
        //         cout << "| "; // 每组数据用竖线分隔
        // }
        // cout << endl;

        // 发送命令
        vector<uint8_t> response = sendHandCommand(handID, masterID, command, data, side);

        // 显示响应
        if (!response.empty())
        {
            // cout << "收到响应: ";
            // for (uint8_t byte : response)
            // {
            //     cout << "0x" << hex << (int)byte << " ";
            // }
            // cout << endl;
        }
        else
        {
            cerr << "未收到有效响应" << endl;
        }
        data.clear();
    }
}

void FINGER_FORCE_LIMIT(uint16_t values[6], aoyi_hand side)
{
    uint8_t handID = 0;
    if (side == right_a)
    {
        handID = aoyi_right_id;
    }
    else
    {
        handID = aoyi_left_id;
    }
    uint8_t masterID = 0x01;
    uint8_t command = 0x47;

    // 准备数据

    // const uint16_t values[6] = {0, 0, 0, 0, 0, 0};
    vector<uint8_t> data;

    for (int i = 0; i < 6; i++)
    {
        // 分解uint16_t值为两个字节
        data.push_back(static_cast<uint8_t>(i));
        data.push_back(values[i] & 0xFF);        // 低字节
        data.push_back((values[i] >> 8) & 0xFF); // 高字节
                                                 // 固定值200

        // 显示要发送的数据
        // cout << "准备发送以下数据:" << endl;
        // cout << "灵巧手ID: 0x" << hex << (int)handID << endl;
        // cout << "主机ID: 0x" << hex << (int)masterID << endl;
        // cout << "命令: 0x" << hex << (int)command << endl;
        // cout << "数据: ";
        // for (size_t i = 0; i < data.size(); i++)
        // {
        //     cout << "0x" << hex << (int)data[i] << " ";
        //     if ((i + 1) % 3 == 0)
        //         cout << "| "; // 每组数据用竖线分隔
        // }
        // cout << endl;

        // 发送命令
        vector<uint8_t> response = sendHandCommand(handID, masterID, command, data, side);

        // 显示响应
        if (!response.empty())
        {
            // cout << "收到响应: ";
            // for (uint8_t byte : response)
            // {
            //     cout << "0x" << hex << (int)byte << " ";
            // }
            // cout << endl;
        }
        else
        {
            cerr << "未收到有效响应" << endl;
        }
        data.clear();
    }
}

vector<string> readConfigFile(const string &configPath)
{
    ifstream configFile(configPath);
    vector<string> lines;
    string line;

    if (configFile.is_open())
    {
        while (getline(configFile, line))
        {
            trimConfigLine(line);
            lines.push_back(line);
        }
        configFile.close();
    }
    else
    {
        cerr << "无法打开配置文件: " << configPath << endl;
    }

    return lines;
}

void port_bind()
{

   
  

    uint8_t handID = 60;
    uint8_t masterID = 0x01;
    uint8_t command = 0x00;

    vector<uint8_t> response = verify_hand(handID, masterID, command, serial_port);

    // 显示响应
    if (!response.empty())
    {

        // for (uint8_t byte : response)
        // {
        //     cout << "0x" << hex << (int)byte << " ";
        // }
        left_port = serial_port2;
        right_port = serial_port;
        cout << "响应模式绑定: bind success"<< endl;
    }
    else
    {
        left_port = serial_port;
        right_port = serial_port2;
        cout << "非响应模式绑定: bind success" << endl;
    }
  /*  handID = 3;

    vector<uint8_t> response2 = verify_hand(handID, masterID, command, serial_port2);

    // 显示响应
    if (!response2.empty())
    {

        // for (uint8_t byte : response2)
        // {
        //     cout << "0x" << hex << (int)byte << " ";
        // }
        left_port = serial_port;
        right_port = serial_port2;
        cout << "响应模式绑定: bind success2"<< endl;
    }
    else
    {
        left_port = serial_port2;
        right_port = serial_port;
         cout << "非响应模式绑定: bind success" << endl;
    }
    */

      // 固定参数

     left_port = serial_port;
    right_port = serial_port2;

    serial_port2 = nullptr;
    serial_port = nullptr;
}

void grasp_hand(aoyi_hand side)
{
    // 固定参数

    uint8_t handID = 0;

    if (side == right_a)
    {
        handID = aoyi_right_id;
    }
    else
    {
        handID = aoyi_left_id;
    }

    uint8_t masterID = 0x01;
    uint8_t command = 0x50;

    // 准备数据
    uint16_t values[6] = {25000, 25000, 25000, 25000, 25000, 65000};
    if (side == left_a)
    {
        for (int i = 0; i < 5; i++)
        {
            values[i] = 25000;
        }
    }
    // const uint16_t values[6] = {0, 0, 0, 0, 0, 0};
    vector<uint8_t> data;

    for (int i = 0; i < 6; i++)
    {
        // 分解uint16_t值为两个字节
        data.push_back(values[i] & 0xFF);        // 低字节
        data.push_back((values[i] >> 8) & 0xFF); // 高字节
        data.push_back(255);                     // 固定值200
    }

    // 显示要发送的数据
    // cout << "准备发送以下数据:" << endl;
    // cout << "灵巧手ID: 0x" << hex << (int)handID << endl;
    // cout << "主机ID: 0x" << hex << (int)masterID << endl;
    // cout << "命令: 0x" << hex << (int)command << endl;
    // cout << "数据: ";
    // for (size_t i = 0; i < data.size(); i++)
    // {
    //     cout << "0x" << hex << (int)data[i] << " ";
    //     if ((i + 1) % 3 == 0)
    //         cout << "| "; // 每组数据用竖线分隔
    // }
    // cout << endl;

    // 发送命令
    vector<uint8_t> response = sendHandCommand(handID, masterID, command, data, side);

    // 显示响应
    if (!response.empty())
    {
        // cout << "收到响应: ";
        // for (uint8_t byte : response)
        // {
        //     cout << "0x" << hex << (int)byte << " ";
        // }
        // cout << endl;
    }
    else
    {
        // cerr << "未收到有效响应" << endl;
    }
}

void grasp_hand2(aoyi_hand side)
{
    // 固定参数

    uint8_t handID = 0;
    if (side == right_a)
    {
        handID = aoyi_right_id;
    }
    else
    {
        handID = aoyi_left_id;
    }

    uint8_t masterID = 0x01;
    uint8_t command = 0x50;

    // 准备数据
    uint16_t values[6] = {56000, 56000, 56000, 56000, 56000, 65000};
    if (side == left_a)
    {
        for (int i = 0; i < 5; i++)
        {
            values[i] = 25000;
        }
    }
    // const uint16_t values[6] = {0, 0, 0, 0, 0, 0};
    vector<uint8_t> data;

    for (int i = 0; i < 6; i++)
    {
        // 分解uint16_t值为两个字节
        data.push_back(values[i] & 0xFF);        // 低字节
        data.push_back((values[i] >> 8) & 0xFF); // 高字节
        data.push_back(255);                     // 固定值200
    }

    // 显示要发送的数据
    // cout << "准备发送以下数据:" << endl;
    // cout << "灵巧手ID: 0x" << hex << (int)handID << endl;
    // cout << "主机ID: 0x" << hex << (int)masterID << endl;
    // cout << "命令: 0x" << hex << (int)command << endl;
    // cout << "数据: ";
    // for (size_t i = 0; i < data.size(); i++)
    // {
    //     cout << "0x" << hex << (int)data[i] << " ";
    //     if ((i + 1) % 3 == 0)
    //         cout << "| "; // 每组数据用竖线分隔
    // }
    // cout << endl;

    // 发送命令
    vector<uint8_t> response = sendHandCommand(handID, masterID, command, data, side);

    // 显示响应
    if (!response.empty())
    {
        // cout << "收到响应: ";
        // for (uint8_t byte : response)
        // {
        //     cout << "0x" << hex << (int)byte << " ";
        // }
        // cout << endl;
    }
    else
    {
        cerr << "未收到有效响应" << endl;
    }
}

void tiger_hand(aoyi_hand side)
{
    // 固定参数

    uint8_t handID = 2;
    if (side == right_a)
    {
        handID = aoyi_right_id;
    }
    else
    {
        handID = aoyi_left_id;
       
    }

    // cout << "aoyi_left_id: " << aoyi_left_id << endl;
    // cout << "aoyi_right_id: " << aoyi_right_id << endl;
    // cout << "handID: " << handID << endl;

    uint8_t masterID = 0x01;
    uint8_t command = 0x50;

    // 准备数据
    const uint16_t values[6] = {0, 0, 0, 0, 0, 65000};
    // const uint16_t values[6] = {0, 0, 0, 0, 0, 0};
    vector<uint8_t> data;

    for (int i = 0; i < 6; i++)
    {
        // 分解uint16_t值为两个字节
        data.push_back(values[i] & 0xFF);        // 低字节
        data.push_back((values[i] >> 8) & 0xFF); // 高字节
        data.push_back(255);                     // 固定值200
    }

    // 显示要发送的数据
    // cout << "准备发送以下数据:" << endl;
    // cout << "灵巧手ID: 0x" << hex << (int)handID << endl;
    // cout << "主机ID: 0x" << hex << (int)masterID << endl;
    // cout << "命令: 0x" << hex << (int)command << endl;
    // cout << "数据: ";
    // for (size_t i = 0; i < data.size(); i++)
    // {
    //     cout << "0x" << hex << (int)data[i] << " ";
    //     if ((i + 1) % 3 == 0)
    //         cout << "| "; // 每组数据用竖线分隔
    // }
    // cout << endl;

    // 发送命令
    vector<uint8_t> response = sendHandCommand(handID, masterID, command, data, side);

    // 显示响应
    if (!response.empty())
    {
        // cout << "收到响应: ";
        // for (uint8_t byte : response)
        // {
        //     cout << "0x" << hex << (int)byte << " ";
        // }
        // cout << endl;
    }
    else
    {
        // cerr << "未收到有效响应" << endl;
    }
}

void grasp_hand_angle(aoyi_hand side, uint16_t values[5])
{
    // 固定参数

    uint8_t handID = 0;
    if (side == right_a)
    {
        handID = aoyi_right_id;
    }
    else
    {
        handID = aoyi_left_id;
    }

    uint8_t masterID = 0x01;
    uint8_t command = 0x4c;

    vector<uint8_t> data;

    for (int i = 0; i < 5; i++)
    {
        // 分解uint16_t值为两个字节
        data.push_back(static_cast<uint8_t>(i));
        data.push_back(values[i] & 0xFF);        // 低字节
        data.push_back((values[i] >> 8) & 0xFF); // 高字节
        data.push_back(255);                     // 固定值200

        // 显示要发送的数据
        // cout << "准备发送以下数据:" << endl;
        // cout << "灵巧手ID: 0x" << hex << (int)handID << endl;
        // cout << "主机ID: 0x" << hex << (int)masterID << endl;
        // cout << "命令: 0x" << hex << (int)command << endl;
        // cout << "数据: ";
        // for (size_t i = 0; i < data.size(); i++)
        // {
        //     cout << "0x" << hex << (int)data[i] << " ";
        //     if ((i + 1) % 3 == 0)
        //         cout << "| "; // 每组数据用竖线分隔
        // }
        // cout << endl;

        // 发送命令
        vector<uint8_t> response = sendHandCommand(handID, masterID, command, data, side);

        // 显示响应
        if (!response.empty())
        {
            // cout << "收到响应: ";
            // for (uint8_t byte : response)
            // {
            //     cout << "0x" << hex << (int)byte << " ";
            // }
            // cout << endl;
        }
        else
        {
            cerr << "未收到有效响应" << endl;
        }
        data.clear();
    }
}

int get_hand_status(aoyi_hand side)
{
    uint8_t handID = 0;
    if (side == right_a)
    {
        handID = aoyi_right_id;
    }
    else
    {
        handID = aoyi_left_id;
    }

    uint8_t masterID = 0x01;
    uint8_t command = 0x5F; // 使用 0x5F set_custom 命令

    // 准备数据：只包含子命令掩码 SUB_CMD_GET_STATUS = 1 << 7 = 0x80
    vector<uint8_t> data;
    data.push_back(0x80); // 子命令掩码：只获取状态

    // 发送命令
    vector<uint8_t> response = sendHandCommand(handID, masterID, command, data, side);

    // 解析和打印响应
    if (!response.empty())
    {
        // 直接打印每个字节，16进制格式，空格分开
        // for (size_t i = 0; i < response.size(); i++)
        // {
        //     printf("%02X ", response[i]);
        // }
        // printf("\n");

        // 检查响应格式是否正确
        if (response.size() >= 13) // 包头(2)+ID(2)+命令(1)+长度(1)+数据(6)+校验(1)=13字节
        {
            // 检查响应包头
            if (response[0] == 0x55 && response[1] == 0xAA && response[4] == 0x5F)
            {
                uint8_t data_length = response[5];

                // 检查数据长度是否为6（6个电机的状态）
                if (data_length == 6 && response.size() >= 13)
                {
                    // 提取6个电机的状态数据（位置6-11）
                    bool all_02 = true;
                    for (int i = 0; i < 6; i++)
                    {
                        uint8_t motor_status = response[6 + i];
                        if (motor_status != 0x02)
                        {
                            all_02 = false;
                            break;
                        }
                    }

                    // 全部都是0x02返回11，否则返回10
                    return all_02 ? 11 : 10;
                }
            }
        }

        // 响应格式不正确，返回0
        return -1;
    }
    else
    {
        cerr << "未收到有效响应" << endl;
        return -1;
    }
}

int set_hand_id(aoyi_hand side)
{
    uint8_t handID = 0;
    if (side == right_a)
    {
        handID = aoyi_right_id;
    }
    else
    {
        handID = aoyi_left_id;
    }

    uint8_t masterID = 0x01;
    uint8_t command = 0x42; // 使用 0x5F set_custom 命令

    // 准备数据：只包含子命令掩码 SUB_CMD_GET_STATUS = 1 << 7 = 0x80
    vector<uint8_t> data;
    data.push_back(0x03); // 子命令掩码：只获取状态

    // 发送命令
    vector<uint8_t> response = sendHandCommand(handID, masterID, command, data, side);

    // 解析和打印响应
    if (!response.empty())
    {
        // 直接打印每个字节，16进制格式，空格分开
        // for (size_t i = 0; i < response.size(); i++)
        // {
        //     printf("%02X ", response[i]);
        // }
        // printf("\n");

        // 检查响应格式是否正确
        if (response.size() >= 13) // 包头(2)+ID(2)+命令(1)+长度(1)+数据(6)+校验(1)=13字节
        {
            // 检查响应包头
            if (response[0] == 0x55 && response[1] == 0xAA && response[4] == 0x5F)
            {
                uint8_t data_length = response[5];

                // 检查数据长度是否为6（6个电机的状态）
                if (data_length == 6 && response.size() >= 13)
                {
                    // 提取6个电机的状态数据（位置6-11）
                    bool all_02 = true;
                    for (int i = 0; i < 6; i++)
                    {
                        uint8_t motor_status = response[6 + i];
                        if (motor_status != 0x02)
                        {
                            all_02 = false;
                            break;
                        }
                    }

                    // 全部都是0x02返回11，否则返回10
                    return all_02 ? 11 : 10;
                }
            }
        }

        // 响应格式不正确，返回0
        return -1;
    }
    else
    {
        cerr << "未收到有效响应" << endl;
        return -1;
    }
}
// =============================================================================
// END: src/aoyihand.cpp
// =============================================================================

// =============================================================================
// BEGIN: src/function.cpp
// =============================================================================
﻿
#include <iostream>
#include <fstream>

#include "function.h"

#include "Ti5_socketcan.h"

#include <chrono>
#include <thread>
#include <algorithm>
// 将沿着xyz坐标系平移运动转化为4*4矩阵
// #define pi acos(-1)
// const double eps = 1e-6;
// double rad=pi/180,deg=180/pi;
//  double rad()
//  {
//      return pi/180;
//  }
//  double deg()
//  {
//      return 180/pi;
//  }
// 将沿着xyz坐标系平移运动转化为4*4矩阵
MatrixXd Transl_xyz(Matrix<double, 1, 3> trans)
{
    double x = trans(0), y = trans(1), z = trans(2);
    Matrix<double, 4, 4> T_transl;
    T_transl << 1, 0, 0, x,
        0, 1, 0, y,
        0, 0, 1, z,
        0, 0, 0, 1;
    return T_transl;
}

// 将沿着xyz坐标系旋转运动转化为4*4矩阵
// 将沿着xyz坐标系旋转运动转化为4*4矩阵
MatrixXd Rot_zyx(Matrix<double, 1, 3> theta)
{
    Matrix<double, 4, 4> T_R;
    Matrix<double, 3, 3> Rotx, Roty, Rotz, RPY;
    double a = theta(0), b = theta(1), c = theta(2);
    Rotz(0, 0) = cos(c);
    Rotz(0, 1) = -sin(c);
    Rotz(0, 2) = 0;
    Rotz(1, 0) = sin(c);
    Rotz(1, 1) = cos(c);
    Rotz(1, 2) = 0;
    Rotz(2, 0) = 0;
    Rotz(2, 1) = 0;
    Rotz(2, 2) = 1;

    Roty(0, 0) = cos(b);
    Roty(0, 1) = 0;
    Roty(0, 2) = sin(b);
    Roty(1, 0) = 0;
    Roty(1, 1) = 1;
    Roty(1, 2) = 0;
    Roty(2, 0) = -sin(b);
    Roty(2, 1) = 0;
    Roty(2, 2) = cos(b);

    Rotx(0, 0) = 1;
    Rotx(0, 1) = 0;
    Rotx(0, 2) = 0;
    Rotx(1, 0) = 0;
    Rotx(1, 1) = cos(a);
    Rotx(1, 2) = -sin(a);
    Rotx(2, 0) = 0;
    Rotx(2, 1) = sin(a);
    Rotx(2, 2) = cos(a);

    RPY = Rotz * Roty * Rotx;
    T_R.row(0) << RPY.row(0), 0;
    T_R.row(1) << RPY.row(1), 0;
    T_R.row(2) << RPY.row(2), 0;
    T_R.row(3) << 0, 0, 0, 1;
    return T_R;
}

MatrixXd TR(Matrix<double, 1, 6> transpose)
{

    Matrix<double, 1, 3> transl, rot;
    Matrix<double, 4, 4> T;
    transl << transpose(0), transpose(1), transpose(2);
    rot << transpose(3), transpose(4), transpose(5);
    T = Transl_xyz(transl) * Rot_zyx(rot);
    return T;
}

// 修正DH坐标
MatrixXd MDHTrans(double alpha, double a, double d, double theta)
{
    MatrixXd T(4, 4);
    T(0, 0) = cos(theta);
    T(0, 1) = -sin(theta);
    T(0, 2) = 0;
    T(0, 3) = a;
    T(1, 0) = sin(theta) * cos(alpha);
    T(1, 1) = cos(theta) * cos(alpha);
    T(1, 2) = -sin(alpha);
    T(1, 3) = -sin(alpha) * d;
    T(2, 0) = sin(theta) * sin(alpha);
    T(2, 1) = cos(theta) * sin(alpha);
    T(2, 2) = cos(alpha);
    T(2, 3) = cos(alpha) * d;
    T(3, 0) = 0;
    T(3, 1) = 0;
    T(3, 2) = 0;
    T(3, 3) = 1;

    return T;
}
Matrix<double, 1, 3> rotationMatrixToEulerAngles(Matrix<double, 3, 3> R)
{
    //    assert(isRotationMatrix(R));
    double sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if (!singular)
    {
        x = atan2(R(2, 1), R(2, 2));
        y = atan2(-R(2, 0), sy);
        z = atan2(R(1, 0), R(0, 0));
    }
    else
    {
        x = atan2(-R(1, 2), R(1, 1));
        y = atan2(-R(2, 0), sy);
        z = 0;
    }
    return {x, y, z};
}

Matrix<double, 1, 6> T2PosEulerAngles(Matrix<double, 4, 4> T)
{
    Matrix<double, 1, 3> p, euler;
    Matrix<double, 1, 6> pos;
    euler = rotationMatrixToEulerAngles(T.block(0, 0, 3, 3));
    p << T(0, 3), T(1, 3), T(2, 3);
    pos << p, euler;
    return pos;
}

Eigen::Vector3d Quaterniond2EulerAngles(Eigen::Quaterniond q)
{
    Eigen::Vector3d angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    angles(0) = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        angles(1) = std::copysign(M_PI / 2, sinp); // use 90 deg()rees if out of range
    else
        angles(1) = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    angles(2) = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}
// 4*4位姿矩阵转化为1*3轴角
// MatrixXd T_to_AngleAxis(Matrix<double, 4, 4>  T)
//{
//     Matrix<double, 4, 4> Tcp;
//     Matrix<double, 1, 3> vec,AngleAxis;
//     double k,theta;
//     double r11,r12,r13,r21,r22,r23,r31,r32,r33;
//     r11 = T(0, 0); r12 = T(0, 1); r13 = T(0, 2);
//     r21 = T(1, 0); r22 = T(1, 1); r23 = T(1, 2);
//     r31 = T(2, 0); r32 = T(2, 1); r33 = T(2, 2);
//     theta=acos((r11+r22+r33-1)/2);
//     k=(1/(2*sin(theta)));
//     vec<<r32-r23,r13-r31,r21-r12;
//     AngleAxis=theta*k*vec;
//     return AngleAxis;
// }

MatrixXd T2Axispos(Matrix<double, 4, 4> T)
{

    Matrix<double, 3, 3> rotation_matrix;
    Matrix<double, 3, 1> axis;
    Matrix<double, 1, 6> pos;
    rotation_matrix = T.block(0, 0, 3, 3);
    Eigen::AngleAxisd angel_axisd(rotation_matrix);
    axis << angel_axisd.axis() * angel_axisd.angle();
    //      cout<<"axis="<<angel_axisd.axis()*angel_axisd.angle()<<endl;
    if (axis(1) > M_PI)
    {
        axis(1) = axis(1) - 2 * M_PI;
    }
    else if (axis(1) < -M_PI)
    {
        axis(1) = axis(1) + 2 * M_PI;
    }
    pos << T(0, 3), T(1, 3), T(2, 3), axis.transpose();
    return pos;
}
// 4*4位姿矩阵转化为1*6 Pos即位置xyz加轴角
Eigen::VectorXd T2Axis(const Eigen::Matrix4d &T)
{
    // 提取旋转矩阵元素
    double r11 = T(0, 0);
    double r12 = T(0, 1);
    double r13 = T(0, 2);
    double r21 = T(1, 0);
    double r22 = T(1, 1);
    double r23 = T(1, 2);
    double r31 = T(2, 0);
    double r32 = T(2, 1);
    double r33 = T(2, 2);

    // 计算旋转角度 theta
    //        cout<<"r11 + r22 + r33 - 1"<<","<<(r11 + r22 + r33 - 1) / 2<<endl;
    double theta;
    //        cout<<"T"<<T<<""<<","<<abs((r11 + r22 + r33 - 1) / 2)<<endl;
    Eigen::Vector3d angle_axis;
    if (std::abs(abs((r11 + r22 + r33 - 1) / 2) - 1) < eps)
    {
        // 当 theta 接近 0 时，轴角为零向量
        angle_axis.setZero();
    }
    else
    {
        theta = std::acos((r11 + r22 + r33 - 1) / 2);
        // 计算旋转轴 v
        Eigen::Vector3d v;
        v(0) = (r32 - r23) / (2 * std::sin(theta));
        v(1) = (r13 - r31) / (2 * std::sin(theta));
        v(2) = (r21 - r12) / (2 * std::sin(theta));

        // 计算轴角表示
        angle_axis = theta * v;
    }

    // 提取位置信息
    double x = T(0, 3);
    double y = T(1, 3);
    double z = T(2, 3);

    // 组合位置和轴角信息到 Pos 向量
    Eigen::VectorXd Pos(6);
    Pos << x, y, z, angle_axis(0), angle_axis(1), angle_axis(2);

    return Pos;
}
MatrixXd Axispos2T(Matrix<double, 1, 6> pos)
{
    Matrix<double, 4, 4> T;
    Matrix<double, 3, 1> vec;
    vec << pos(3), pos(4), pos(5);

    //    Eigen::AngleAxisd angel_axisd(rotation_matrix);
    Eigen::AngleAxisd rot_vec(vec.norm(), vec.normalized());
    Eigen::Matrix3d M = rot_vec.toRotationMatrix();
    T.block(0, 3, 3, 1) << pos(0), pos(1), pos(2);
    T.block(0, 0, 3, 3) << M;
    T.block(3, 0, 1, 4) << 0, 0, 0, 1;
    if (pos(3) == 0 && pos(4) == 0 && pos(5) == 0)
    {
        T.block(0, 0, 3, 3) << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
    }
    return T;
}

// MatrixXd ndi2T(Matrix<double, 1, 6>  pos)
// {
//     Matrix<double, 1, 6> transpose;
//     Matrix<double, 4, 4> T;
//     double x=pos(0)*pow(10,-3),y=pos(1)*pow(10,-3),z=pos(2)*pow(10,-3),
//         rx=pos(3)*rad(),ry=pos(4)*rad(),rz=pos(5)*rad();
//     transpose<<x,y,z,rx,ry,rz;
//     T=TR(transpose);
//     return T;
// }

// MatrixXd displaypos(Matrix<double, 1, 6>  pos)
// {
//     Matrix<double, 1, 6> displaypos;
//     displaypos<<pos(0)*pow(10,3),pos(1)*pow(10,3),pos(2)*pow(10,3),
//         pos(3)*deg(),pos(4)*deg(),pos(5)*deg();
//     return displaypos;
// }
MatrixXd skew(Matrix<double, 1, 3> s)

{
    double vx = s(0), vy = s(1), vz = s(2);
    Matrix<double, 3, 3> R;
    R << 0, -vz, vy,
        vz, 0, -vx,
        -vy, vx, 0;
    return R;
}
MatrixXd line_interp(Matrix<double, 1, 6> p1, Matrix<double, 1, 6> p2, double dx)

{

    Matrix<double, 1, 6> dp, d, a, alp, offset;
    int m, n, j;
    double len, k;

    dp = p1 - p2;
    len = sqrt(pow(dp(0), 2) + pow(dp(1), 2) + pow(dp(2), 2));
    n = len / dx;
    MatrixXd Pos(n + 2, 6);
    m = n + 1;
    j = 1;
    for (int i = 0; i <= m; i++)
    {
        k = i / double(m);
        Pos.block(i, 0, 1, 6) = p1 + k * (p2 - p1);
    }
    return Pos;
}

MatrixXd circular_interp(Matrix<double, 1, 6> p1, Matrix<double, 1, 6> p2, Matrix<double, 1, 6> p3, double dx)

{

    Matrix<double, 1, 6> o13;
    int m, n, j;
    double len, k, x1, y1, z1, x2, y2, z2, x3, y3, z3, A1, B1, C1, D1, A2, B2, C2, D2, A3, B3, C3, D3, x0, y0, z0, r,
        ax, ay, az, L, d13, do2, theta, q;
    Matrix<double, 3, 3> A;
    Matrix<double, 3, 1> b, C, u, v, p;

    x1 = p1(0);
    x2 = p2(0);
    x3 = p3(0);
    y1 = p1(1);
    y2 = p2(1);
    y3 = p3(1);
    z1 = p1(2);
    z2 = p2(2);
    z3 = p3(2);

    A1 = (y1 - y3) * (z2 - z3) - (y2 - y3) * (z1 - z3);
    B1 = (x2 - x3) * (z1 - z3) - (x1 - x3) * (z2 - z3);
    C1 = (x1 - x3) * (y2 - y3) - (x2 - x3) * (y1 - y3);
    D1 = -(A1 * x3 + B1 * y3 + C1 * z3);

    A2 = x2 - x1;
    B2 = y2 - y1;
    C2 = z2 - z1;
    D2 = -((pow(x2, 2) - pow(x1, 2)) + (pow(y2, 2) - pow(y1, 2)) + (pow(z2, 2) - pow(z1, 2))) / 2;

    A3 = x3 - x2;
    B3 = y3 - y2;
    C3 = z3 - z2;
    D3 = -((pow(x3, 2) - pow(x2, 2)) + (pow(y3, 2) - pow(y2, 2)) + (pow(z3, 2) - pow(z2, 2))) / 2;

    A << A1, B1, C1,
        A2, B2, C2,
        A3, B3, C3;
    b << -D1, -D2, -D3;

    //% 圆心
    C = A.inverse() * b;
    x0 = C(0);
    y0 = C(1);
    z0 = C(2);

    //% 外接圆半径
    r = sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2) + pow(z1 - z0, 2));

    //% 新坐标系Z0的方向余弦
    L = sqrt(pow(A1, 2) + pow(B1, 2) + pow(C1, 2));
    ax = A1 / L;
    ay = B1 / L;
    az = C1 / L;
    //% 新坐标系X0的方向余弦

    //%step2:得到13两点间的间距d
    d13 = sqrt(pow(p1(0) - p3(0), 2) + pow(p1(1) - p3(1), 2) + pow(p1(2) - p3(2), 2));
    o13 = (p1 + p3) / 2;

    do2 = sqrt(pow(o13(0) - p2(0), 2) + pow(o13(1) - p2(1), 2) + pow(o13(2) - p2(2), 2));
    // d13=sqrt((p1(1)-p3(1))^2+(p1(2)-p3(2))^2+(p1(3)-p3(3))^2);
    // o13=(p1+p3)/2;%得到13两点的中点，当弧角180度时，圆心在该点，以该点为衡量，判定优劣弧
    //%step3:得到13中点与2点间距do2
    // do2=sqrt((o13(1)-p2(1))^2+(o13(2)-p2(2))^2+(o13(3)-p2(3))^2);
    //%ste4:判断优弧还是劣弧
    if (do2 < d13 / 2)
    {
        theta = 2 * asin(d13 / 2 / r);
    }
    else
    {
        theta = 2 * M_PI - 2 * asin(d13 / 2 / r);
    }

    len = theta * r;
    //% 插补点数N
    n = len / dx;
    MatrixXd Pos(n + 2, 6);
    m = n + 1;
    v = (p1.block(0, 0, 1, 3).transpose() - C);
    u << ax, ay, az;
    j = 1;

    for (int i = 0; i <= m; i++)
    {
        k = i / double(m);
        q = theta * k;
        p = v * cos(q) + u.dot(v) * u * (1 - cos(q)) + u.cross(v) * sin(q) + C;

        Pos.block(i, 0, 1, 3) = p.transpose();
        Pos.block(i, 3, 1, 3) = p1.block(0, 3, 1, 3);
    }
    return Pos;
}

Eigen::MatrixXd pinv(Eigen::MatrixXd A) // 计算矩阵伪逆
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    double pinvtoler = 1.e-8; // tolerance
    int row = A.rows();
    int col = A.cols();
    int k = min(row, col);
    Eigen::MatrixXd X = Eigen::MatrixXd::Zero(col, row);
    Eigen::MatrixXd singularValues_inv = svd.singularValues(); // 奇异值
    Eigen::MatrixXd singularValues_inv_mat = Eigen::MatrixXd::Zero(col, row);
    for (long i = 0; i < k; ++i)
    {
        if (singularValues_inv(i) > pinvtoler)
            singularValues_inv(i) = 1.0 / singularValues_inv(i);
        else
            singularValues_inv(i) = 0;
    }
    for (long i = 0; i < k; ++i)
    {
        singularValues_inv_mat(i, i) = singularValues_inv(i);
    }
    X = (svd.matrixV()) * (singularValues_inv_mat) * (svd.matrixU().transpose());

    return X;
}

// 正运动学 根据输入关节角度求出末端法兰相对于基座4*4矩阵
MatrixXd fkine(Matrix<double, 1, 6> theta)
{

    Matrix<double, 1, 6> d, a, alp, offset;
    Matrix<double, 4, 4> T1, T2, T3, T4, T5, T6, T7, T;
    Matrix<double, 1, 6> p;

    //    alp << 0,90,180,90,-90,90;
    //    a << 0,0,0.480015,0,0,0;
    //    d <<0.26,0,0,0.520137,0,0.192;
    //    offset<<0,90,90,0,0,0;
    //    alp=alp*rad();
    //    offset=offset*rad();

    //    alp<<0,-1.57104,-0.000294356,1.57027,-1.57031,1.56901;
    //    a<<   0,0.000175529,0.419961,0.000144852,-1.9803e-05,-0.069077;
    //    d<< 0.2665,-0.000116339,0,0.380242,-0.000232702,0.0951;
    //    offset<< 0,-1.56438,1.57089,0.00632152,-0.00257079,0;
    alp << 0, -1.57109, -0.000382754, 1.5709, -1.57184, 1.57136;
    a << 0, -3.37469e-05, 0.469603, -2.73644e-05, -0.000133765, -0.0776986;
    d << 0.326, -7.60105e-05, 0, 0.429973, 9.031e-05, 0.0973;
    offset << 0, -1.57001, 1.56756, -0.00575697, -0.00305482, 0;
    theta = theta + offset;
    T1 = MDHTrans(alp(0), a(0), d(0), theta(0));
    T2 = MDHTrans(alp(1), a(1), d(1), theta(1));
    T3 = MDHTrans(alp(2), a(2), d(2), theta(2));
    T4 = MDHTrans(alp(3), a(3), d(3), theta(3));
    T5 = MDHTrans(alp(4), a(4), d(4), theta(4));
    T6 = MDHTrans(alp(5), a(5), d(5), theta(5));
    //    T7 = MDHTrans(alp(6), a(6), d(6), theta(6));
    T = T1 * T2 * T3 * T4 * T5 * T6;
    // p = T_to_RPY(T);

    //    cout<<T<<endl;
    return T;
}

Eigen::Matrix<double, 1, 6> quinticInterp_CartSpace(
    double t,
    double totalTime,
    const Eigen::Matrix<double, 1, 6> &start,
    const Eigen::Matrix<double, 1, 6> &target)
{
    // 处理总时间为0的特殊情况
    if (totalTime <= 1e-6)
        return target;

    // 时间边界处理（C++14兼容）
    double clampedTime = t;
    if (clampedTime < 0.0)
        clampedTime = 0.0;
    if (clampedTime > totalTime)
        clampedTime = totalTime;
    const double ratio = clampedTime / totalTime;

    // 五次多项式核心计算
    const double r2 = ratio * ratio;
    const double polyTerm = r2 * ratio * (10 - 15 * ratio + 6 * r2);

    // 计算当前位置
    return start + (target - start) * polyTerm;
}

Eigen::Matrix<double, 1, 7> quinticInterp_JointSpace(
    double t,
    double totalTime,
    const Eigen::Matrix<double, 1, 7> &start,
    const Eigen::Matrix<double, 1, 7> &target)
{
    // 处理总时间为0的特殊情况
    if (totalTime <= 1e-6)
        return target;

    // 时间边界处理（C++14兼容）
    double clampedTime = t;
    if (clampedTime < 0.0)
        clampedTime = 0.0;
    if (clampedTime > totalTime)
        clampedTime = totalTime;
    const double ratio = clampedTime / totalTime;

    // 五次多项式核心计算
    const double r2 = ratio * ratio;
    const double polyTerm = r2 * ratio * (10 - 15 * ratio + 6 * r2);

    // 计算当前位置
    return start + (target - start) * polyTerm;
}
// Matrix<double, 4, 4>  eyehand_falan_calib(Matrix<double, 8, 6> Joint, Matrix<double, 8, 6> NDIpose)
// {
//     int len=8,nn=len-1;
//     MatrixXd T_base_tool(4,4*len),T_cam_cal(4,4*len);
//     MatrixXd M(4*len,4),N(4*len,4),sum_T_tool_cal(4,4);
//     sum_T_tool_cal.setZero(4,4);
//     Matrix<double, 3, 3>  A1,B1;
//     MatrixXd A(4,4*nn),B(4,4*nn);
//     MatrixXd AA(4*nn,4);
//     Matrix<double, 1, 6> joint;
//     Matrix<double, 1, 6> ndi_pos;

//     for (int i = 0; i < len;i++)
//     {
//         joint<<Joint.block(i,0,1,6);
//         T_base_tool.block(0,4*i,4,4) =fkine(joint);
//         ndi_pos<<NDIpose.block(i,0,1,6);
//         T_cam_cal.block(0,4*i,4,4)=ndi2T(ndi_pos);
//     }

//     for  (int i=0;i<len-1;i++)
//     {
//         int j=i+1;
//         A.block(0,4*i,4,4) =T_base_tool.block(0,4*i,4,4).inverse()*T_base_tool.block(0,4*j,4,4);
//         B.block(0,4*i,4,4) =T_cam_cal.block(0,4*i,4,4).inverse()*T_cam_cal.block(0,4*j,4,4);

//     }

//     for  (int i=0;i<len-1;i++)
//     {
//         A1=A.block(0,4*i,3,3);
//         B1=B.block(0,4*i,3,3);
//         Eigen::Quaterniond a(A1);
//         Eigen::Quaterniond b(B1);
//         Matrix<double, 1, 3>  m1,m2;
//         Matrix<double, 1, 4>  aq;
//         Matrix<double, 3, 3>  C1;
//         Matrix<double, 4, 4> AA1,BB1,dd;
//         m1<<a.x(),a.y(),a.z();
//         m2<<b.x(),b.y(),b.z();
//         aq<<a.x(),a.y(),a.z(),a.w();
//         MatrixXd::Identity(3,3) ;
//         C1.setIdentity(3,3) ;
//         AA1.block(0,0,1,4)<<a.w(),-a.x(),-a.y(),-a.z();
//         AA1.block(1,0,3,1)<<a.x(),a.y(),a.z();
//         AA1.block(1,1,3,3)=a.w()*C1+skew(m1);

//         BB1.block(0,0,1,4)<<b.w(),-b.x(),-b.y(),-b.z();
//         BB1.block(1,0,3,1)<<b.x(),b.y(),b.z();
//         BB1.block(1,1,3,3)=b.w()*C1-skew(m2);

//         AA.block(4*i,0,4,4)=AA1-BB1;

//     }

//     JacobiSVD<Eigen::MatrixXd> svd(AA,  ComputeThinU | ComputeThinV);
//     Matrix<double, 4, 4>T_tcp_calib, V = svd.matrixV();

//     Matrix<double, 4, 1> V1;
//     V1.block(0,0,3,1)=V.block(1,3,3,1);
//     V1(3,0)=V(0,3);
//     Eigen::Quaterniond v1q(V1);
//     Matrix<double, 3, 3> R=v1q.toRotationMatrix();

//     MatrixXd C(3*nn,3);
//     MatrixXd d(3*nn,1);
//     Matrix<double, 3, 3>  I;
//     I.setIdentity(3,3) ;
//     for  (int i=0;i<len-1;i++)
//     {
//         C.block(3*i,0,3,3)=I-A.block(0,4*i,3,3);
//         d.block(3*i,0,3,1)=A.block(0,4*i+3,3,1) -R*B.block(0,4*i+3,3,1);

//     }
//     Matrix<double, 3, 1>  t;
//     Matrix<double, 4, 4>  X1,t_tool_cal;
//     t=(C.transpose()*C).inverse()*C.transpose()*d;

//     T_tcp_calib.block(0,0,3,3)<<R;
//     T_tcp_calib.block(0,3,3,1)<<t;
//     T_tcp_calib.block(3,0,1,4)<<0,0,0,1;
//     return T_tcp_calib;
// }

// void Delay_mSec(unsigned int msec)
// {
//     QTime dieTime = QTime::currentTime().addMSecs(msec);
//     while (QTime::currentTime() < dieTime)
//         QCoreApplication::processEvents(QEventLoop::AllEvents, 5);
// }

// 六个自由度和六维度力对应相乘的函数
MatrixXd multiplyWithDOF(Matrix<double, 1, 6> dof, Matrix<double, 1, 6> forces)
{
    Matrix<double, 1, 6> result;
    for (int i = 0; i < 6; ++i)
    {
        result[i] = dof[i] * forces[i];
    }
    return result;
}

MatrixXd slerp_interp(double t, Matrix<double, 1, 3> euler_start, Matrix<double, 1, 3> euler_end)
{
    Matrix<double, 4, 4> T_start, T_end;
    Matrix<double, 3, 3> R_start, R_end;

    Matrix<double, 1, 3> RPY_interpolated, euler;
    Matrix<double, 1, 4> q1, q2, q_inter;
    double epsilon = 1e-6, omega;
    T_start = Rot_zyx(euler_start);
    T_end = Rot_zyx(euler_end);
    R_start = T_start.block(0, 0, 3, 3);
    R_end = T_end.block(0, 0, 3, 3);
    Eigen::Quaterniond q_start(R_start);
    Eigen::Quaterniond q_end(R_end);
    q1 << q_start.w(), q_start.x(), q_start.y(), q_start.z();
    q2 << q_end.w(), q_end.x(), q_end.y(), q_end.z();

    q_inter = q1;
    omega = acos(q1.dot(q2));
    if (abs(omega) < epsilon)
    {
        q_inter = (1 - t) * q1 + q2 * t;
    }
    else
    {
        q_inter = (sin((1 - t) * omega) / sin(omega)) * q1 + (sin(t * omega) / sin(omega)) * q2;
    }
    q_inter = q_inter / q_inter.norm();
    Eigen::Quaterniond interpolated_quaternions(q_inter(0), q_inter(1), q_inter(2), q_inter(3));
    euler = Quaterniond2EulerAngles(interpolated_quaternions);

    return euler;
}
//********************************

int getFileRows(const char *fileName)
{
    ifstream fileStream;
    string tmp;
    int count = 0;                      // 行数计数器
    fileStream.open(fileName, ios::in); // ios::in 表示以只读的方式读取文件
    if (fileStream.fail())              // 文件打开失败:返回0
    {
        return 0;
    }
    else // 文件存在
    {
        while (getline(fileStream, tmp, '\n')) // 读取一行
        {
            if (tmp.size() > 0)
                count++;
        }
        fileStream.close();
        return count;
    }
}

int getFileColumns(const char *fileName)
{
    ifstream fileStream;
    fileStream.open(fileName, ios::in);

    double tmp = 0;
    int count = 0; // 列数计数器
    char c;        // 当前位置的字符
    c = fileStream.peek();
    while (('\n' != c) && (!fileStream.eof())) // 指针指向的当前字符，仅观测，不移动指针位置
    {
        fileStream >> tmp;
        ++count;
        c = fileStream.peek();
    }

    fileStream.close();
    return count;
}

double **getMatrix(const char *path, const int n, const int m)
{
    fstream myfile;
    myfile.open(path);

    double **mat = new double *[n];
    for (int i = 0; i < n; i++)
    {
        double *tmp = new double[m];
        mat[i] = tmp;
    }

    string tmpStr;
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < m; j++)
        {
            myfile >> tmpStr; //
            double dValue = atof(tmpStr.c_str());
            mat[i][j] = dValue;
        }
    }
    return mat;
}

std::array<double, 3> extractCoordinates(const std::string &input)
{
    size_t start = input.find('[');
    size_t end = input.find(']', start);

    if (start == std::string::npos || end == std::string::npos)
    {
        throw std::invalid_argument("无效输入：未找到 '[' 或 ']'");
    }

    std::string coords_str = input.substr(start + 1, end - start - 1);
    std::stringstream ss(coords_str);
    std::array<double, 3> coords;
    char comma;

    if (!(ss >> coords[0] >> comma >> coords[1] >> comma >> coords[2]))
    {
        throw std::invalid_argument("无效输入：坐标格式错误");
    }



    return coords;
}

Eigen::Matrix3d Yaw_Rotation(float yaw)
{
    Eigen::Matrix3d T;
    T << cos(yaw), -sin(yaw), 0,
        sin(yaw), cos(yaw), 0,
        0, 0, 1;
    return T;
}

Eigen::Matrix3d Pitch_Rotation(float pitch)
{
    Eigen::Matrix3d T;
    T << cos(pitch), 0, sin(pitch),
        0, 1, 0,
        -sin(pitch), 0, cos(pitch);
    return T;
}

Eigen::Matrix3d Roll_Rotation(float roll)
{
    Eigen::Matrix3d T;
    T << 1, 0, 0,
        0, cos(roll), -sin(roll),
        0, sin(roll), cos(roll);
    return T;
}

Eigen::Matrix4d neck_to_eye(int flag)
{
    int id30_position, id31_position, id32_position;
    // getPosition(hang_waist_id, 30, id30_position);
    // getPosition(hang_waist_id, 31, id31_position);
    // getPosition(hang_waist_id, 32, id32_position);

    id30_position = 0;
    // id31_position = 0;

    if (flag == 0)
    {
        id31_position = -20058;
    }

    else if (flag == 3)
    {
        id31_position = 10000;
    }
    else if (flag == 4)
    {
        id31_position = -10000;
    }
    else
    {
        id31_position = -22058;
    }

    id32_position = -0;

    // cout << "id30_position: " << id30_position << " " << "id31_position: " << id31_position << " " << "id32_position: " << id32_position << endl;
    float r_30 = id30_position / ((4 * 65536) / (2 * M_PI));
    float r_31 = id31_position / ((4 * 65536) / (2 * M_PI));
    float r_32 = id32_position / ((4 * 65536) / (2 * M_PI));
    // cout << "r_30: " << r_30 << " " << "r_31: " << r_31 << " " << "r_32: " << r_32 << endl;

    Eigen::Matrix4d T1 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T2 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T3 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T4 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T5 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T6 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T7 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T8 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T9 = Eigen::Matrix4d::Identity();
    // Eigen::Vector3d
    // T1.block<3, 1>(0, 3) = pos;
    T1.block<3, 3>(0, 0) = Yaw_Rotation(r_30);
    // Eigen::Vector3d diff1; diff1 << 0, 0, 0;
    Eigen::Vector3d diff1;
    diff1 << 0, 0, 145;
    T2.block<3, 1>(0, 3) = diff1;
    T3.block<3, 3>(0, 0) = Roll_Rotation(M_PI / 2);
    T4.block<3, 3>(0, 0) = Yaw_Rotation(r_31);
    T5.block<3, 3>(0, 0) = Pitch_Rotation(M_PI / 2);
    T6.block<3, 3>(0, 0) = Yaw_Rotation(r_32);
    Eigen::Vector3d diff2;
    diff2 << 30, 135, 32;
    // Eigen::Vector3d diff2; diff2 << 35, 12, 95;
    T7.block<3, 1>(0, 3) = diff2;
    T8.block<3, 3>(0, 0) = Roll_Rotation(15 * M_PI / 180);
    T9.block<3, 3>(0, 0) = Yaw_Rotation(M_PI);

    return T1 * T2 * T3 * T4 * T5 * T6 * T7 * T8 * T9;
}

double calculateTf(const Eigen::Matrix<double, 1, 6> &axis_p1,
                   const Eigen::Matrix<double, 1, 6> &axis_p2,
                   double V_DEFAULT)
{
    // // 1. 提取平移和旋转分量
    // Eigen::Vector3d trans_p1 = axis_p1.head(3);  // 起始平移 (x,y,z)
    // Eigen::Vector3d trans_p2 = axis_p2.head(3);  // 目标平移 (x,y,z)
    // Eigen::Vector3d rot_p1 = axis_p1.tail(3);    // 起始旋转 (rx,ry,rz)
    // Eigen::Vector3d rot_p2 = axis_p2.tail(3);    // 目标旋转 (rx,ry,rz)

    Matrix<double, 1, 6> axis_p;
    Matrix<double, 4, 4> T_p1_p2, T_p1, T_p2;
    T_p1 = Axispos2T(axis_p1);
    T_p2 = Axispos2T(axis_p2);
    axis_p = T2Axispos(T_p1.inverse() * T_p2);

    // 2. 计算平移距离和旋转等效距离
    double d_trans = (axis_p.head(3)).norm(); // 平移距离 (m)
    double d_rot = (axis_p.tail(3)).norm();   // 旋转角度差 (rad)
    double d_rot_eq = 0.2 * d_rot;            // 旋转等效距离 (特征长度0.5m)

    // 3. 处理微小距离（无需运动）
    const double EPS = 1e-6; // 微小距离阈值 (m)
    bool trans_need_move = (d_trans > EPS);
    bool rot_need_move = (d_rot_eq > EPS);

    if (!trans_need_move && !rot_need_move)
    {
        return 0.0; // 已到达目标，无需运动
    }

    // 4. 分别计算平移和旋转的理论时间
    double t_trans = trans_need_move ? (d_trans / V_DEFAULT) : 0.0;
    double t_rot = rot_need_move ? (d_rot_eq / V_DEFAULT) : 0.0;

    // 5. 总时间取最大值（确保同步完成）
    double Tf = std::max(t_trans, t_rot);

    // 6. 强制最小时间约束（避免超短时间导致冲击）
    const double MIN_TF = 0.1; // 最小运动时间 (s)，可根据机械臂特性调整
    if (Tf < MIN_TF)
    {
        Tf = MIN_TF;
    }

    // 调试信息（可选）
    // std::cout << "平移距离: " << d_trans*1000 << "mm, 旋转等效距离: " << d_rot_eq*1000 << "mm\n";
    // std::cout << "平移时间: " << t_trans << "s, 旋转时间: " << t_rot << "s, 总时间: " << Tf << "s\n";

    return Tf;
}

Eigen::Matrix<double, 1, 7> joint_range_normalize(
    const Eigen::Matrix<double, 1, 7> &q,
    const Eigen::Matrix<double, 1, 7> &q_min,
    const Eigen::Matrix<double, 1, 7> &q_max)
{
    // 计算中点和单侧范围（向量化操作）
    Matrix<double, 1, 7> q_mid = (q_min + q_max) / 2.0;
    Matrix<double, 1, 7> half_range = (q_max - q_min) / 2.0;

    // 避免除零（向量化处理）
    half_range = half_range.cwiseMax(1e-6);

    // 计算归一化距离并限制在[0,1]（单步完成核心逻辑）
    return (q - q_mid).cwiseAbs().cwiseQuotient(half_range).cwiseMax(0.0).cwiseMin(1.0);
}

Eigen::Matrix<double, 1, 7> calc_segment_weight(
    const Eigen::Matrix<double, 1, 7> &x,
    double w_max)
{
    // 参数校验：w_max必须为正标量
    if (w_max <= 0)
    {
        throw std::invalid_argument("w_max必须是正标量（如100）！");
    }

    // 核心参数（常量表达式，编译时确定）
    constexpr double safe_threshold = 0.7;          // 安全区与警戒区分界点
    constexpr double safe_w_base = 1e-3;            // 安全区基础权重
    constexpr double growth_factor = 5;             // 警戒区增长系数
    constexpr double interval = 1 - safe_threshold; // 警戒区区间长度（0.3）

    // 初始化输出矩阵（1×7固定尺寸，无需手动指定大小）
    Eigen::Matrix<double, 1, 7> w;

    // 循环处理每个维度（利用固定尺寸特性，循环次数编译时确定）
    for (int i = 0; i < 7; ++i)
    {
        const double xi = x(i); // 访问1×7矩阵的第i列元素（0~6）

        if (xi <= safe_threshold)
        {
            // 安全区：x ≤ 0.7 → 基础权重
            w(i) = safe_w_base;
        }
        else if (xi < 1.0)
        {
            // 警戒区：0.7 < x < 1 → 指数增长到w_max
            const double ratio = (xi - safe_threshold) / interval; // 0~1比例
            w(i) = safe_w_base + (w_max - safe_w_base) * (1 - std::exp(-growth_factor * ratio));
        }
        else
        {
            // 超限区：x ≥ 1 → 最大权重
            w(i) = w_max;
        }
    }

    return w;
}


std::tuple<MatrixXd, MatrixXd, MatrixXd>
quinticInterp(
    double t,
    double totalTime,
    const MatrixXd& start,  // 统一 Matrixd：1×N
    const MatrixXd& target  // 统一 Matrixd：1×N
) {
    // 1. 输入合法性校验（新增“必须是1行”校验，适配关节数据）
    // 总时间为0：返回终点+零速度/加速度
    if (totalTime <= 1e-6) {
        MatrixXd zero_mat(1, start.cols());
        zero_mat.setZero();
        return {target, zero_mat, zero_mat};
    }
    // 起点/终点必须是1行（避免多行矩阵输入）
    if (start.rows() != 1 || target.rows() != 1) {
        MatrixXd zero_mat(1, start.cols());
        zero_mat.setZero();
        return {start, zero_mat, zero_mat};
    }
    // 起点/终点列数（关节数）必须一致
    if (start.cols() != target.cols()) {
        MatrixXd zero_mat(1, start.cols());
        zero_mat.setZero();
        return {start, zero_mat, zero_mat};
    }

    // 2. 时间边界夹紧（避免超界）
    double clampedTime = std::max(0.0, std::min(t, totalTime));
    double ratio = clampedTime / totalTime;  // 时间占比（0~1）

    // 3. 五次多项式核心项预计算（无冲击插值，工业级平滑）
    const double r2 = ratio * ratio;
    const double r3 = r2 * ratio;
    const double r4 = r3 * ratio;
    const double r5 = r4 * ratio;

    // 位置/速度/加速度项（解析导数，保证连续）
    const double pos_term = 10 * r3 - 15 * r4 + 6 * r5;
    const double vel_term = (30 * r2 - 60 * r3 + 30 * r4) / totalTime;
    const double acc_term = (60 * ratio - 180 * r2 + 120 * r3) / (totalTime * totalTime);

    // 4. 计算结果（Matrixd 直接运算，维度自动匹配 1×N）
    MatrixXd delta = target - start;  // 位置差（1×N）
    MatrixXd pos = start + delta * pos_term;
    MatrixXd vel = delta * vel_term;
    MatrixXd acc = delta * acc_term;

    return {pos, vel, acc};
}

namespace
{

int arm_hand_side(const Robot_Arm &arm)
{
    if (!arm.can_id_list.empty() && arm.can_id_list[0] >= 23)
        return 0;
    return 1;
}

double encoder_cnt_to_rad(int cnt)
{
    const double deg = static_cast<double>(cnt) * 360.0 / 65536.0 / 4.0;
    return deg * M_PI / 180.0;
}

int rad_to_encoder_cnt(double q_rad)
{
    const double deg = q_rad * 180.0 / M_PI;
    return static_cast<int>(deg * 65536.0 * 4.0 / 360.0);
}

constexpr double k_arm_traj_dt = 5e-3;

int execute_arm_trajectory(const MatrixXd &traj, int hand, double dt = k_arm_traj_dt)
{
    if (traj.rows() == 0)
    {
        cout << "[Error] Trajectory is empty, execution aborted." << endl;
        return -1;
    }

    const int dof = static_cast<int>(std::min(traj.cols(), static_cast<Eigen::Index>(7)));
    for (int i = 0; i < traj.rows(); ++i)
    {
        const auto cycle_start = chrono::high_resolution_clock::now();

        int motor_cmd[7] = {0};
        for (int j = 0; j < dof; ++j)
            motor_cmd[j] = rad_to_encoder_cnt(traj(i, j));
        set_motor_position(motor_cmd, hand);

        const double elapsed_ms =
            chrono::duration<double, milli>(chrono::high_resolution_clock::now() - cycle_start).count();
        const double sleep_ms = std::max(0.0, dt * 1000.0 - elapsed_ms);
        if (sleep_ms > 1e-6)
            this_thread::sleep_for(chrono::duration<double, milli>(sleep_ms));
    }
    return 0;
}

Eigen::MatrixXd arm_read_motor_joints(int hand)
{
    int motor_cnt[7] = {0};
    get_motor_position(motor_cnt, hand);

    Eigen::MatrixXd q_start(1, 7);
    for (int i = 0; i < 7; ++i)
        q_start(0, i) = encoder_cnt_to_rad(motor_cnt[i]);
    return q_start;
}

Matrix<double, 1, 6> arm_flange_tool(const Robot_Arm &arm)
{
    Matrix<double, 1, 6> tool;
    if (arm_hand_side(arm) == 1)
        tool << 0.23, 0, 0, 0, 0, 0;
    else
        tool << 0.23, 0, 0, 0, 0, 0;
    return tool;
}

} // namespace

int arm_line_move(Robot_Arm &arm, Matrix<double, 1, 6> &pos, double cart_linear_velocity)
{
    const int hand = arm_hand_side(arm);
    Eigen::MatrixXd q_start = arm_read_motor_joints(hand);

    arm.Cart_Linear_Velocity = cart_linear_velocity;

    Eigen::MatrixXd traj;
    const int ret = arm.Line_Trajectory(q_start, pos, traj, q_start(0, 1));
    if (ret != 0)
        return ret;

    return execute_arm_trajectory(traj, hand, k_arm_traj_dt);
}

void arm_dual_line_move(
    Robot_Arm &arm_r,
    Matrix<double, 1, 6> &pos_r,
    Robot_Arm &arm_l,
    Matrix<double, 1, 6> &pos_l,
    double cart_linear_velocity)
{
    std::thread th_r([&]() { arm_line_move(arm_r, pos_r, cart_linear_velocity); });
    std::thread th_l([&]() { arm_line_move(arm_l, pos_l, cart_linear_velocity); });
    th_r.join();
    th_l.join();
}

void arm_dual_line_move_selective(
    Robot_Arm &arm_r,
    Matrix<double, 1, 6> &pos_r,
    const bool move_r,
    Robot_Arm &arm_l,
    Matrix<double, 1, 6> &pos_l,
    const bool move_l,
    const double cart_linear_velocity)
{
    if (move_r && move_l)
    {
        arm_dual_line_move(arm_r, pos_r, arm_l, pos_l, cart_linear_velocity);
        return;
    }
    if (move_r)
        arm_line_move(arm_r, pos_r, cart_linear_velocity);
    if (move_l)
        arm_line_move(arm_l, pos_l, cart_linear_velocity);
}

namespace
{

constexpr int kMotorStoppedMinZeroCount = 6;

bool motor_joint_speed_counts_stopped(int speed)
{
    return speed == 0;
}

void log_motor_running_speeds(const char *label, const int speed[7])
{
    std::cout << label;
    for (int i = 0; i < 7; ++i)
        std::cout << ' ' << speed[i];
    std::cout << std::endl << std::flush;
}

} // namespace

void log_arms_motor_running_speeds(bool log_r, bool log_l, const char *reason)
{
    int speed_r[7] = {0};
    int speed_l[7] = {0};
    if (log_r)
        get_motor_running_speed(speed_r, 1);
    if (log_l)
        get_motor_running_speed(speed_l, 0);
    std::cout << "[arm] " << reason;
    if (log_r)
    {
        int zero_r = 0;
        std::cout << " speed_r=";
        for (int i = 0; i < 7; ++i)
        {
            std::cout << ' ' << speed_r[i];
            if (speed_r[i] == 0)
                ++zero_r;
        }
        std::cout << " (zero=" << zero_r << "/7)";
    }
    if (log_l)
    {
        int zero_l = 0;
        std::cout << " speed_l=";
        for (int i = 0; i < 7; ++i)
        {
            std::cout << ' ' << speed_l[i];
            if (speed_l[i] == 0)
                ++zero_l;
        }
        std::cout << " (zero=" << zero_l << "/7)";
    }
    std::cout << std::endl << std::flush;
}

bool arm_motors_all_stopped(int hand, int speed_threshold)
{
    (void)speed_threshold;
    int speed[7] = {0};
    get_motor_running_speed(speed, hand);

    int zero_count = 0;
    for (int i = 0; i < 7; ++i)
    {
        if (motor_joint_speed_counts_stopped(speed[i]))
            ++zero_count;
    }
    return zero_count >= kMotorStoppedMinZeroCount;
}

bool wait_arms_motors_stopped(
    bool wait_r,
    bool wait_l,
    double timeout_sec,
    int min_zero_count)
{
    if (!wait_r && !wait_l)
        return true;

    const int required_zero =
        (min_zero_count > 0 && min_zero_count <= 7) ? min_zero_count : kMotorStoppedMinZeroCount;

    auto side_stopped = [required_zero](int hand) -> bool {
        int speed[7] = {0};
        get_motor_running_speed(speed, hand);
        int zero_count = 0;
        for (int i = 0; i < 7; ++i)
        {
            if (motor_joint_speed_counts_stopped(speed[i]))
                ++zero_count;
        }
        return zero_count >= required_zero;
    };

    constexpr int kRequiredStablePolls = 6; // 连续 6 次（约 300ms）判定停稳
    int stable_count = 0;

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(timeout_sec));
    while (std::chrono::steady_clock::now() < deadline)
    {
        bool ok = true;
        if (wait_r)
            ok = ok && side_stopped(1);
        if (wait_l)
            ok = ok && side_stopped(0);
        if (ok)
        {
            ++stable_count;
            if (stable_count >= kRequiredStablePolls)
            {
                log_arms_motor_running_speeds(wait_r, wait_l, "手相机前停稳确认(退出轮询)");
                return true;
            }
        }
        else
        {
            stable_count = 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    log_arms_motor_running_speeds(wait_r, wait_l, "手相机前停稳失败(超时)");
    std::cerr << "[arm] 等待电机停稳超时 wait_r=" << wait_r << " wait_l=" << wait_l << std::endl;
    return false;
}

int arm_joint_move(Robot_Arm &arm, Matrix<double, 1, 7> &qd)
{
    const int hand = arm_hand_side(arm);
    const Eigen::MatrixXd q_start = arm_read_motor_joints(hand);
    Eigen::MatrixXd q_end = qd;
    Eigen::MatrixXd traj;
    const int ret = arm.Joint_Trajectory(q_start, q_end, traj);
    if (ret != 0)
        return ret;
    return execute_arm_trajectory(traj, hand, k_arm_traj_dt);
}

Eigen::Matrix4d arm_fk_tcp_from_encoders(Robot_Arm &arm)
{
    const int hand = arm_hand_side(arm);
    const Eigen::MatrixXd q = arm_read_motor_joints(hand);
    return arm.Forward_Kinematics(q.row(0));
}

Matrix<double, 1, 6> arm_get_tcp_pos(Robot_Arm &arm)
{
    return T2PosEulerAngles(arm_fk_tcp_from_encoders(arm));
}

Matrix<double, 1, 6> arm_get_base_visual_pos(Robot_Arm &arm, Matrix<double, 1, 6> &falan_visualPos)
{
    const Eigen::Matrix4d T_tcp = arm_fk_tcp_from_encoders(arm);
    const Matrix<double, 1, 6> flange_tool = arm_flange_tool(arm);

    // cout << "==================================================== " << endl;

    // cout << "T_tcp: " << T_tcp << endl;
    // cout << "flange_tool: " << flange_tool << endl;
    // cout << "falan_visualPos: " << falan_visualPos << endl;
    // cout << "==================================================== " << endl;

    



    const Eigen::Matrix4d T_base_visual = T_tcp * TR(flange_tool).inverse() * TR(falan_visualPos);
    return T2PosEulerAngles(T_base_visual);
}
// =============================================================================
// END: src/function.cpp
// =============================================================================

// =============================================================================
// BEGIN: src/arm_robot.cpp
// =============================================================================
#include "arm_robot.h"

#include "Ti5_socketcan.h"
#include "seg_pose_bridge.h"

#include <algorithm>

namespace
{

std::string robot_arm_model_yaml_path()
{
    return project_root_dir() + "/config/Robot_Arm_Model.yaml";
}

} // namespace

ArmRobot::ArmRobot(int mod)
    : solver_(mod == -1 ? Robot_Arm("T7", "T170", "right", robot_arm_model_yaml_path())
                        : Robot_Arm("T7", "T170", "left", robot_arm_model_yaml_path())),
      hand_flag_(mod == -1 ? 1 : 0)
{
    if (mod == -1)
        flange_tool_ << 0, 0.08, -0.11, 0, 0, 0;
    else
        flange_tool_ << 0, -0.08, -0.11, 0, 0, 0;
}

Eigen::Matrix<double, 1, 7> ArmRobot::getJointPos()
{
    Eigen::Matrix<double, 1, 7> motor_q;
    int motor_pos[dof_] = {0};

    while (true)
    {
        get_motor_position(motor_pos, hand_flag_ == 0 ? 0 : 1);
        if (hand_flag_ == 0)
        {
            bool all_zero = true;
            for (int i = 0; i < dof_; ++i)
            {
                if (motor_pos[i] != 0)
                {
                    all_zero = false;
                    break;
                }
            }
            if (!all_zero)
                break;
        }
        else
        {
            break;
        }
    }

    for (int i = 0; i < dof_; ++i)
        motor_q(i) = static_cast<double>(motor_pos[i]) / rad2cnt_;

    return motor_q;
}

Eigen::Matrix<double, 1, 6> ArmRobot::getTcpPos()
{
    const Eigen::Matrix<double, 1, 7> q = getJointPos();
    const Eigen::Matrix4d T = solver_.Forward_Kinematics(q);
    return T2PosEulerAngles(T);
}

Eigen::Matrix<double, 1, 6> ArmRobot::get_base_visual_pos(
    const Eigen::Matrix<double, 1, 6> &falan_visualPos)
{
    const Eigen::Matrix<double, 1, 7> q = getJointPos();
    const Eigen::Matrix4d T_tcp = solver_.Forward_Kinematics(q);
    const Eigen::Matrix4d T_flange = T_tcp * TR(flange_tool_).inverse();
    const Eigen::Matrix4d T_base_visual = T_flange * TR(falan_visualPos);
    return T2PosEulerAngles(T_base_visual);
}

int ArmRobot::servoJ(Eigen::Matrix<double, 1, 7> &qd)
{
    int motor_positions[dof_];
    for (int i = 0; i < dof_; ++i)
        motor_positions[i] = static_cast<int>(qd(i) * rad2cnt_);

    set_motor_position(motor_positions, hand_flag_ == 0 ? 0 : 1);
    return 0;
}

void ArmRobot::play_joint_trajectory(const Eigen::MatrixXd &traj)
{
    if (traj.rows() == 0)
        return;

    for (int i = 0; i < traj.rows(); ++i)
    {
        const auto cycle_start = std::chrono::high_resolution_clock::now();
        Eigen::Matrix<double, 1, 7> q = traj.row(i);
        servoJ(q);

        const double elapsed_ms = std::chrono::duration<double, std::milli>(
                                      std::chrono::high_resolution_clock::now() - cycle_start)
                                      .count();
        const double sleep_ms = std::max(0.0, traj_dt_ * 1000.0 - elapsed_ms);
        if (sleep_ms > 1e-6)
            std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(sleep_ms));
    }
}

int ArmRobot::moveJtoJoint(Eigen::Matrix<double, 1, 7> &qd)
{
    const Eigen::Matrix<double, 1, 7> qc = getJointPos();
    Eigen::MatrixXd q_start = qc;
    Eigen::MatrixXd q_end = qd;
    Eigen::MatrixXd traj;
    const int ret = solver_.Joint_Trajectory(q_start, q_end, traj);
    if (ret != 0)
        return ret;

    play_joint_trajectory(traj);
    return 0;
}

int ArmRobot::moveL(Eigen::Matrix<double, 1, 6> &posd, double vel)
{
    (void)vel;
    solver_.Cart_Linear_Velocity = 0.10;

    const Eigen::Matrix<double, 1, 7> qc = getJointPos();
    Eigen::MatrixXd q_start = qc;
    Eigen::MatrixXd traj;
    const int ret = solver_.Line_Trajectory(q_start, posd, traj, qc(1));
    if (ret != 0)
        return ret;

    play_joint_trajectory(traj);
    return 0;
}

// =============================================================================
// END: src/arm_robot.cpp
// =============================================================================

// =============================================================================
// BEGIN: src/waist.cpp
// =============================================================================
#include "waist.h"
#include "Ti5_socketcan.h"

// 注意：所有函数实现前的 Robot:: 都要改为 WaistRobot::

Matrix<double, 1, 6> WaistRobot::getTool()
{
    return TR(tool);
}

MatrixXd WaistRobot::q2MotorAngle(const Matrix<double, 1, dof_3> &q)
{
    Matrix<double, 1, dof_3> MotorAngle;
    MotorAngle = q.cwiseProduct(q2motor_direct) + q2motor_offset;
    return MotorAngle;
}

MatrixXd WaistRobot::MotorAngle2q(const Matrix<double, 1, dof_3> &MotorAngle)
{
    Matrix<double, 1, dof_3> q;
    q = (MotorAngle - q2motor_offset).cwiseQuotient(q2motor_direct);
    return q;
}

MatrixXd WaistRobot::getJointPos()
{
    int dof_3 = 3; // 假设这是变量
    MatrixXd motor_q(1, dof_3), q(1, dof_3), motor_pos(1, dof_3);
    MatrixXd dataList(1, dof_3);
    int data[3] = {};

    get_motor_waist_position(data);

    // 使用循环赋值
    for (int i = 0; i < dof_3; i++)
    {
        dataList(i) = static_cast<double>(data[i]);
    }

    // dataList = getCSP(0, MotorsIDlist, dof_3);

  //  motor_pos = dataList.block(2, 0, 1, dof_3);
    motor_pos = dataList / rad2cnt;
    q = MotorAngle2q(motor_pos);

    return q;
}

MatrixXd WaistRobot::getCsp()
{
    Matrix<double, 1, dof_3> motor_current, motor_vel, motor_pos, q, dq, tor;

    int32_t MotorPosition[dof_3];
    Matrix<double, 3, dof_3> dataList, output;

    // auto cycle_start = chrono::high_resolution_clock::now();
    // dataList = getCSP(0, MotorsIDlist, dof_3);
    motor_current = dataList.block(0, 0, 1, dof_3);
    motor_vel = dataList.block(1, 0, 1, dof_3);
    motor_pos = dataList.block(2, 0, 1, dof_3);

    motor_pos = motor_pos / rad2cnt;
    motor_vel = motor_vel / vel2hz;
    q = MotorAngle2q(motor_pos);
    dq = motor_vel.cwiseQuotient(q2motor_direct);
    tor = motor_current.cwiseQuotient(q2motor_direct);
    output.row(0) = q;
    output.row(1) = dq;
    output.row(2) = tor;
    return output;
}

// 雅可比矩阵计算函数-矢量积法
MatrixXd WaistRobot::J_tcp_crossproduct(const Matrix<double, 1, dof_3> &q)
{
    VectorXd alp = MDH.col(0);
    VectorXd a = MDH.col(1);
    VectorXd d = MDH.col(2);
    VectorXd offset = MDH.col(3);
    VectorXd q_offset = q.transpose() + offset;
    Matrix4d T = Matrix4d::Identity();

    for (int i = 0; i < dof_3; ++i)
    {
        Matrix4d T_i = MDHTrans(alp(i), a(i), d(i), q_offset(i));
        T = T * T_i;
    }

    Matrix4d Tend = TR(tool);
    Matrix4d T_b_end = T * Tend;
    Matrix4d T1 = fk(q);
    Vector3d p_ee = T1.block<3, 1>(0, 3);

    MatrixXd J(6, dof_3);
    J.setZero();
    T = Matrix4d::Identity();
    for (int i = 0; i < dof_3; ++i)
    {
        Matrix4d T_i = MDHTrans(alp(i), a(i), d(i), q_offset(i));
        T = T * T_i;
        Vector3d z_i = T.block<3, 1>(0, 2);
        Vector3d p_i = T.block<3, 1>(0, 3);
        J.block<3, 1>(0, i) = z_i.cross(p_ee - p_i);
        J.block<3, 1>(3, i) = z_i;
    }

    MatrixXd transform(6, 6);
    transform.setZero();
    transform.block<3, 3>(0, 0) = T_b_end.block<3, 3>(0, 0);
    transform.block<3, 3>(3, 3) = T_b_end.block<3, 3>(0, 0);

    return transform.transpose() * J;
}

MatrixXd WaistRobot::getTcpPos()
{
    Matrix<double, 1, dof_3> motor_q, q;
    Matrix<double, 4, 4> T;
    Matrix<double, 1, 6> pos;
    q = getJointPos();

    T = fk(q);
    pos = T2PosEulerAngles(T);
    return pos;
}

// 正向运动学函数
Matrix4d WaistRobot::fk(const Matrix<double, 1, dof_3> &q)
{
    int n = dof_3;
    Matrix4d T = Matrix4d::Identity();
    VectorXd alp = MDH.col(0);
    VectorXd a = MDH.col(1);
    VectorXd d = MDH.col(2);
    VectorXd offset = MDH.col(3);

    for (int i = 0; i < n; ++i)
    {
        Matrix4d T_i = MDHTrans(alp(i), a(i), d(i), q(i) + offset(i));
        T = T * T_i;
    }
    Matrix4d T_tcp = T * TR(tool);
    return T_tcp;
}

// 核心函数：输入关节弧度值，无阻塞批量发送到电机
int WaistRobot::servoJ(const Matrix<double, 1, dof_3> &qd)
{
    Matrix<double, 1, dof_3> motor_q;
    motor_q = q2MotorAngle(qd);

    int32_t motorPositions[dof_3];
    for (int i = 0; i < dof_3; ++i)
    {
        motorPositions[i] = static_cast<int32_t>(motor_q(i) * rad2cnt);
    }

    set_waist_motor_position(motorPositions);
    return 0;
}

// 核心函数：输入关节弧度值，无阻塞批量发送到电机
int WaistRobot::speedJ(Matrix<double, 1, dof_3> &dq)
{
    Matrix<double, 1, dof_3> motor_dq;
    motor_dq = dq.cwiseProduct(q2motor_direct);

    int32_t motorSpeeds[dof_3];
    for (int i = 0; i < dof_3; ++i)
    {
        motorSpeeds[i] = static_cast<int32_t>(motor_dq(i) * vel2hz);
    }

    //  setSpeeds(0, MotorsIDlist, motorSpeeds);
    return 0;
}

// 核心函数：输入关节弧度值，无阻塞批量发送到电机
int WaistRobot::tauJ(Matrix<double, 1, dof_3> &tau)
{
    int32_t motorCurrents[dof_3];
    for (int i = 0; i < dof_3; ++i)
    {
        motorCurrents[i] = static_cast<int32_t>(tau(i) * q2motor_direct(i));
    }

    // setCurrents(0, MotorsIDlist, motorCurrents);
    return 0;
}

int WaistRobot::moveLToPos(Matrix<double, 1, 6> &posd, double vel)
{
    Matrix<double, 1, 6> axis_pos, posc, pos1, pos2, axis_p1, axis_p2, v_tcp;
    Matrix<double, 1, dof_3> qc, q, q1, q_current, dq_current, q_ref;
    Matrix<double, 4, 4> T1, T2, Tc, Td, Tv, T12;

    double t = 0, dt, lamda, Tf, k;

    qc = getJointPos();
    q_ref = qc;
    int ret = ik(posd, q_ref);

   // cout << "q_ref: " << q_ref*deg << endl;

    if (ret!= 0)
    {
        cout << "Move_Limit" << endl;
        return -2;
    }

    T1 = fk(qc);
    pos2 = posd;
    T2 = TR(pos2);
    T12 = T1.inverse() * T2;
    axis_p1.setZero();
    axis_p2 = T2Axispos(T12);

    Tf = calculateTf(axis_p1, axis_p2, vel);
    dt = 5e-3;
    auto cycle_start = chrono::high_resolution_clock::now();

    while (t < Tf)
    {
        cycle_start = chrono::high_resolution_clock::now();
        auto [axis_pos, daxis_pos, ddaxis_pos] = quinticInterp(t, Tf, axis_p1, axis_p2);

        Td = T1 * Axispos2T(axis_pos);
        posd = T2PosEulerAngles(Td);
        int ret1 = ik(posd, qc);
        // cout << "ret1: " <<  ret1 << " qc: "<<qc*deg<< endl;
        if (ret1 != 0)
        {
            cout << "Move_error" << endl;
            return -1;
        }
        servoJ(qc);
        double elapsed_ms = duration<double, milli>(chrono::high_resolution_clock::now() - cycle_start).count();

        double sleep_ms = max(0.0, dt * 1000 - elapsed_ms);
        if (sleep_ms > 1e-6)
        {
            auto sleep_duration = duration<double, milli>(sleep_ms);
            this_thread::sleep_for(sleep_duration);
        }
        //cout <<"qc="<< qc*deg <<"\n";

        t = t + dt;
    }

    return 0;
}

int WaistRobot::moveJToJoint(Matrix<double, 1, dof_3> &qd, double vel)
{
    Matrix<double, 4, 4> T1, T2;
    Matrix<double, 1, 6> axis_p1, axis_p2;
    Matrix<double, 1, dof_3> qc;
    qc = getJointPos();
    T1 = fk(qc);
    T2 = fk(qd);

    axis_p1 = T2Axispos(T1);
    axis_p2 = T2Axispos(T2);
    double Tf = calculateTf(axis_p1, axis_p2, vel);
    int ret = moveJToJoint_Tf(qd, Tf);
    return ret;
}

int WaistRobot::moveJToJoint_Tf(Matrix<double, 1, dof_3> &q_end, double Tf)
{
    Matrix<double, 1, dof_3> q_start, qc, q, dq, dqc;
    double t, dt;
    q_start = getJointPos();
    t = 0;
    dt = 5e-3;
    auto cycle_start = chrono::high_resolution_clock::now();

    while (t < Tf)
    {
        cycle_start = chrono::high_resolution_clock::now();
        auto [q, dq, ddq] = quinticInterp(t, Tf, q_start, q_end);
        servoJ(q);

        double elapsed_ms = duration<double, std::milli>(chrono::high_resolution_clock::now() - cycle_start).count();

        double sleep_ms = max(0.0, dt * 1000 - elapsed_ms);
        if (sleep_ms > 1e-6)
        {
            auto sleep_duration = duration<double, std::milli>(sleep_ms);
            std::this_thread::sleep_for(sleep_duration);
        }

        t = t + dt;
    }
    return 0;
}

int WaistRobot::moveJToPos(Matrix<double, 1, 6> &posd, double vel)
{
    Matrix<double, 1, dof_3> qc, qd;
    Matrix<double, 4, 4> T1, T2;
    Matrix<double, 1, 6> axis_p1, axis_p2;
    qc = getJointPos();
    qd = qc;
    int ret = ik(posd, qd);

    T1 = fk(qc);
    T2 = TR(posd);
    axis_p1 = T2Axispos(T1);
    axis_p2 = T2Axispos(T2);
    double Tf = calculateTf(axis_p1, axis_p2, vel);
    int ret1 = moveJToJoint_Tf(qd, Tf);

    return ret1;
}

void WaistRobot::getJointState(Matrix<double, 1, dof_3> &qc, Matrix<double, 1, dof_3> &dqc)
{
    // // auto data = getCsp(); // 假设t5是内部关节状态接口
    // qc = data.row(0);  // 当前关节位置
    // dqc = data.row(1); // 当前关节速度
}

int WaistRobot::ik(Matrix<double, 1, 6> &pos, Matrix<double, 1, dof_3> &q)
{
    const double eps = 1e-6;
    const double singular_threshold = 1 * M_PI / 180; // 对应MATLAB的1*tk.rad，约0.017rad

    // -------------------------- 1. 初始化与参数提取 --------------------------
    Eigen::MatrixXd solutions(0, 3); // 候选解集合（每行1个3关节解）

    // 提取机器人参数（MDH为3×4矩阵，列0:alpha, 列1:a, 列2:d, 列3:offset）
    const MatrixXd alpha = MDH.col(0);  // 3×1
    const MatrixXd a = MDH.col(1);      // 3×1
    const double a2 = a(1, 0);          // 对应MATLAB a2=a(2)（注意索引从0开始）
    const double a3 = a(2, 0);          // 对应MATLAB a3=a(3)
    const MatrixXd offset = MDH.col(3); // 3×1（关节偏移角）

    // 计算变换矩阵（基→末端法兰，去除工具坐标系影响）
    Matrix4d T_falan_tcp = TR(tool);
    Matrix4d T_base_tcp = TR(pos);
    Matrix4d T_base_falan = T_base_tcp * T_falan_tcp.inverse();
    Matrix4d Tend = T_base_falan;

    // 提取末端变换矩阵关键参数（位置+姿态）
    const double nx = Tend(0, 0);
    const double nz = Tend(2, 0);
    const double px = Tend(0, 3);
    const double pz = Tend(2, 3);

    // -------------------------- 2. 工作空间检查（末端是否接近原点） --------------------------
    const double p_sq = px * px + pz * pz;
    if (p_sq < eps)
    {
        return -1; // 末端位置接近原点，无法求解
    }

    // -------------------------- 3. 求解t2（核心关节，两个候选解） --------------------------
    double cos_t2 = (p_sq - a2 * a2 - a3 * a3) / (2 * a2 * a3);
    cos_t2 = clamp(cos_t2, -1.0, 1.0); // 数值保护，避免acos参数超范围

    const double t2_sol1 = acos(cos_t2); // 解1：肘上构型
    const double t2_sol2 = -t2_sol1;     // 解2：肘下构型
    const vector<double> t2_sols = {t2_sol1, t2_sol2};

    if (abs(t2_sol1) <= singular_threshold)
    {
        cout << "workapce_limit" << "\n";
        return -2;
    }

    // -------------------------- 4. 遍历t2解，求解t1和t3 --------------------------
    for (double t2 : t2_sols)
    {
        // 计算中间变量K和L（对应MATLAB逻辑）
        const double K = a2 + a3 * cos(t2);
        const double L = a3 * sin(t2);

        // 求解t1（四象限反正切，避免除零）
        const double num_t1 = pz * K - px * L;
        const double den_t1 = px * K + pz * L;
        const double t1 = atan2(num_t1, den_t1);

        // 求解总旋转角theta（基于末端姿态）
        const double theta = atan2(nz, nx);

        // 求解t3（theta = t1 + t2 + t3 → t3 = theta - t1 - t2）
        double t3 = theta - t1 - t2;

        // 角度归一化到[-π, π]
        t3 = fmod(t3 + M_PI, 2 * M_PI) - M_PI;

        // -------------------------- 5. 生成候选解并添加到集合 --------------------------
        MatrixXd q_candidate(1, 3);
        q_candidate << t1, t2, t3;

        // 候选解角度归一化（确保统一范围）
        for (int i = 0; i < dof_3; ++i)
        {
            q_candidate(0, i) = fmod(q_candidate(0, i) + M_PI, 2 * M_PI) - M_PI;
        }

        // 动态扩展候选解集合
        solutions.conservativeResize(solutions.rows() + 1, 3);
        solutions.row(solutions.rows() - 1) = q_candidate;
    }

    // -------------------------- 6. 检查是否有有效候选解 --------------------------
    if (solutions.rows() == 0)
    {
        return -3; // 所有解均奇异，位置无解
    }

    const MatrixXd q_best = solutions.row(0);

    // -------------------------- 8. 限位检查 --------------------------
    bool in_limit = true;
    for (int i = 0; i < dof_3; ++i)
    {
        if (q_best(0, i) < limit(0, i) - eps || q_best(0, i) > limit(1, i) + eps)
        {
            in_limit = false;
            break;
        }
    }

    // -------------------------- 9. 输出结果与返回状态 --------------------------
    if (in_limit)
    {
        q = q_best; // 输出最优解（1×3矩阵，适配dof_3=3）
        return 0;   // 成功：有解且在限位内
    }
    else
    {
        return 1; // 有解但超限位
    }
}
// =============================================================================
// END: src/waist.cpp
// =============================================================================

// =============================================================================
// BEGIN: src/motor_driver.cpp
// =============================================================================
// #include "motor_driver.h"

// int seven_motor_move(int CanDeviceId, vector<int> can_list, vector<vector<vector<int>>> position_velocity)
// {
//     cout << showpos;
//     int n = can_list.size();

//     vector<std::array<int, 3>> CSP_list(n);

//     while (1)
//     {
//         getCSP(CanDeviceId, can_list, CSP_list);
//         int count = 0;
//         for(int i = 0; i < n; i++){
//             if(CSP_list[i][0] == CSP_list[i][1] && CSP_list[i][1] == CSP_list[i][2]){
//                 i = 0;
//             }else{
//                 count++;
//             }
//         }
//         if (count == n)
//         {
//             break;
//         }
//     }
//     cout << "here" << endl;


//     int temp[n] = dataList1[2], temp2 = dataList2[2], temp3 = dataList3[2], temp4 = dataList4[2], temp5 = dataList5[2], temp6 = dataList6[2], temp7 = dataList7[2];

//     int stop;

//     int n = position_velocity1[1].size(), m = position_velocity2[1].size(), l = position_velocity3[1].size();
//     int q = position_velocity1[1].size(), w = position_velocity2[1].size(), e = position_velocity3[1].size(), r = position_velocity3[1].size();

//     int i = 0, j = 0, k = 0, z = 0, x = 0, c = 0, v = 0;
//     int count = 0;
//     auto start = std::chrono::high_resolution_clock::now();
//     while (1)
//     {

//         count++;
//         setSpeed(CanDeviceId, 0, numaccuator, can_list[0], dataList1, position_velocity1[1].data() + i);
//         setSpeed(CanDeviceId, 0, numaccuator, can_list[1], dataList2, position_velocity2[1].data() + j);
//         setSpeed(CanDeviceId, 0, numaccuator, canIdList3, dataList3, position_velocity3[1].data() + k);
//         setSpeed(CanDeviceId, 0, numaccuator, canIdList4, dataList4, position_velocity4[1].data() + z);
//         setSpeed(CanDeviceId, 0, numaccuator, canIdList5, dataList5, position_velocity5[1].data() + x);
//         setSpeed(CanDeviceId, 0, numaccuator, canIdList6, dataList6, position_velocity6[1].data() + c);
//         setSpeed(CanDeviceId, 0, numaccuator, canIdList7, dataList7, position_velocity7[1].data() + v);

//         if (i >= n - 1 && j >= m - 1 && k >= l - 1 && z >= q - 1 && x >= w - 1 && c >= e - 1 && v >= r - 1){

//             break;
//         }
            

//         if ((abs(temp1 - dataList1[2]) >= abs(position_velocity1[0][i]) || abs(position_velocity1[1][i]) == 0) && i < n - 1)
//             i++;
//         if ((abs(temp2 - dataList2[2]) >= abs(position_velocity2[0][j]) || abs(position_velocity2[1][j]) == 0) && j < m - 1)
//             j++;
//         if ((abs(temp3 - dataList3[2]) >= abs(position_velocity3[0][k]) || abs(position_velocity3[1][k]) == 0) && k < l - 1)
//             k++;
//         if ((abs(temp4 - dataList4[2]) >= abs(position_velocity4[0][z]) || abs(position_velocity4[1][z]) == 0) && z < q - 1)
//             z++;
//         if ((abs(temp5 - dataList5[2]) >= abs(position_velocity5[0][x]) || abs(position_velocity5[1][x]) == 0) && x < w - 1)
//             x++;
//         if ((abs(temp6 - dataList6[2]) >= abs(position_velocity6[0][c]) || abs(position_velocity6[1][c]) == 0) && c < e - 1)
//             c++;
//         if ((abs(temp7 - dataList7[2]) >= abs(position_velocity7[0][v]) || abs(position_velocity7[1][v]) == 0) && v < r - 1)
//             v++;

//     }
//     auto end = std::chrono::high_resolution_clock::now();
//     std::chrono::duration<double> gap = end - start;
//     cout << "time: " << gap.count() << endl;
//     cout << "count: " << count << endl;
//     cout << "frequence: " << count / gap.count() << endl;

//     return 0;
// }


// vector<vector<double>> sequence_generate(MatrixXd &Q, MatrixXd &DQ){
    
// }
// =============================================================================
// END: src/motor_driver.cpp
// =============================================================================

// =============================================================================
// BEGIN: src/reals_tcp.cpp
// =============================================================================
#include "reals_tcp.h"
#include "Ti5_socketcan.h"
#include "waist.h"

tcp::socket *g_socket = nullptr;

tcp::socket reals_tcp_init()
{
    boost::asio::io_service io;
    tcp::socket socket(io);

    // 连接 Python 服务器
    socket.connect(tcp::endpoint(boost::asio::ip::address::from_string("127.0.0.1"), 12345));

    std::cout << "✅ 已连接到 Python 服务器" << std::endl;
    return socket;
}

std::string reals_tcp_trsmit(tcp::socket &socket, std::string target)
{

    // 发送指令到 Python
    boost::asio::write(socket, boost::asio::buffer(target + "\n"));

    // 接收 Python 的响应（原始模式）
    boost::asio::streambuf buf;
    boost::system::error_code ec;

    // 读取所有可用数据（不等待换行符）
    size_t len = boost::asio::read(socket, buf, boost::asio::transfer_at_least(1), ec);

    if (ec && ec != boost::asio::error::eof)
    {
        throw boost::system::system_error(ec);
    }

    // 打印原始接收数据
    std::string raw_data(boost::asio::buffers_begin(buf.data()),
                         boost::asio::buffers_begin(buf.data()) + len);

    std::istream is(&buf);
    std::string response;
    while (std::getline(is, response))
    {
        // std::cout << "解析后的响应: " << response << std::endl;
    }

    return response;
}

int pos_move(double goal_last[3], Robot_Arm &Taihu)
{

    Matrix<double, 1, 6> posd = {goal_last[0], goal_last[1], goal_last[2], 0, 0, 0};

    sleep(1);
    return arm_line_move(Taihu, posd, 0.2);
}

int position_con(Robot_Arm &Taihu)
{
    cout << "开始移动操作" << endl;
    Matrix<double, 1, 6> current_hand_pos = arm_get_tcp_pos(Taihu);
    double curr_pos[6] = {current_hand_pos(0), current_hand_pos(1), current_hand_pos(2), 0, 0, 0};
    cout << "当前位置:" << current_hand_pos(0) << current_hand_pos(1) << current_hand_pos(2) << endl;

    while (true)
    {
        int j;
        cin >> j;
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
        cout << "j:" << j << endl;
        if (j == 1)
        {
            curr_pos[0] += 0.05;
            int ret = pos_move(curr_pos, Taihu);
            if (ret != 0)
            {
                curr_pos[0] -= 0.05;
            }
        }
        else if (j == 2)
        {
            curr_pos[0] -= 0.05;
            int ret = pos_move(curr_pos, Taihu);
            if (ret != 0)
            {
                curr_pos[0] += 0.05;
            }
        }
        else if (j == 4)
        {
            curr_pos[1] += 0.05;
            int ret = pos_move(curr_pos, Taihu);
            if (ret != 0)
            {
                curr_pos[1] -= 0.05;
            }
        }
        else if (j == 5)
        {
            curr_pos[1] -= 0.05;
            int ret = pos_move(curr_pos, Taihu);
            if (ret != 0)
            {
                curr_pos[1] += 0.05;
            }
        }
        else if (j == 7)
        {
            curr_pos[2] += 0.02;
            int ret = pos_move(curr_pos, Taihu);
            if (ret != 0)
            {
                curr_pos[2] -= 0.02;
            }
        }
        else if (j == 8)
        {
            curr_pos[2] -= 0.02;
            int ret = pos_move(curr_pos, Taihu);
            if (ret != 0)
            {
                curr_pos[2] += 0.02;
            }
        }
        else
        {
            break;
        }
    }
    return 0;
}

int mech(Robot_Arm &Taihu, Robot_Arm &Taihu_l)
{

    //     {

    Matrix<double, 1, 7> q_j_r;
    q_j_r << 0.000719053, -1.56991, 1.57187, -0.000167779, -1.57156, 0.00107858, 0.000527306;

    Matrix<double, 1, 7> q_j_l;
    q_j_l << 0.000191748, 1.56984, -1.5696, -0.000383495, 1.57204, 0.00093477, -0.000479369;

    arm_joint_move(Taihu, q_j_r);
    arm_joint_move(Taihu_l, q_j_l);
    // }

    // int data[7] = {0,0,0,0,0,0,0};

    // set_motor_position(data, 0);
    // set_motor_position(data, 1);

    return 0;
}

int start_pos(Robot_Arm &Taihu_r, Robot_Arm &Taihu_l, int flag)
{

    Eigen::Matrix<double, 1, 6> posup_down_3_layer;

    posup_down_3_layer << -0.132502, 4.36259e-17, 0.622464, -1.5708, -1.56703, 3.14159;

    int waist_p[1] = {0 * 65536 * 4 / 360};
    uint32_t waist_canID[] = {1};

    socketcan_sendcommand(waist_id, 1, waist_canID, 30, waist_p);

    sleep(2);

    WaistRobot Ti5_waist;

    posup_down_3_layer(0) = 0.13;

    Ti5_waist.moveLToPos(posup_down_3_layer, 0.1);

    Matrix<double, 1, 7> q_s_j_r2 = {-0.66117, -0.394449, 0.183646, 2.18791, 0.0589864, 0.123749, -0.0720971};
    Matrix<double, 1, 7> q_s_j_l2 = {-0.932517, 0.310511, -0.381362, 2.27168, -0.255743, 0.155867, 0.0662248};

    // Matrix<double, 1, 7> q_s_j_r = {-0.761221, -0.63564, 0, 1.290989, 0, -0.0, 0.0};
    // Matrix<double, 1, 7> q_s_j_l = {-0.744108, 0.63472, 0, 1.29906, 0, -0.0, 0.0};

    // arm_joint_move(Taihu_r, q_s_j_r2);
    // arm_joint_move(Taihu_l, q_s_j_l2);

    posup_down_3_layer(0) = -0.13;

    Ti5_waist.moveLToPos(posup_down_3_layer, 0.1);

    return 0;
}

int zhuaqu(Matrix<double, 1, 6> &goal_last, Robot_Arm &Taihu, int flag, std::string target, double lin)
{
    Matrix<double, 1, 6> pos1, pos2;

    int juli = 0;

    pos1 << 0.24, -0.4, -0.15, 0, 0, 0;
    pos2 << 0.24, 0.4, -0.15, 0, 0, 0;

    if (flag == 1)
    {
        tiger_hand(right_a);
    }
    else
    {
        tiger_hand(left_a);
    }

    goal_last[0] += readPiancha("1");
    juli = 1;

    int stop;

    int ret1;

    // cin >> stop;
    // cin.ignore(numeric_limits<streamsize>::max(), '\n');

    if (flag == 2) // left
    {

        ret1 = moveL(Taihu, goal_last, 0.2);

        if (ret1 == -1)
        {
            moveL(Taihu, pos2, 0.2);
            return -1;
        }
    }
    else
    {

        ret1 = moveL(Taihu, goal_last, 0.2);

        if (ret1 == -1)
        {
            moveL(Taihu, pos1, 0.2);
            return -1;
        }
    }
    // {
    //     sleep(2);
    //     MatrixXd current_hand_pos = Taihu.getTcpPos();
    //     cout << "抓取函数当前位置" << current_hand_pos << endl;
    //     cout << "抓取函数目标位置" << goal_last[0] << " " << goal_last[1] << " " << goal_last[2] << endl;
    // }

    // if (flag == 1)
    // {
    //     tiger_hand(right_a);
    // }
    // else
    // {
    //     tiger_hand(left_a);
    // }
    if (flag == 1)
    {

        double x_qian = -readPiancha("1") + readPiancha("3");
        goal_last[0] += x_qian;
    }
    else
    {

        double x_qian = -readPiancha("1") + readPiancha("4");
        goal_last[0] += x_qian;
    }

    // cin >> stop;
    // cin.ignore(numeric_limits<streamsize>::max(), '\n');

    if (flag == 2) // left
    {

        ret1 = moveL(Taihu, goal_last, 0.1);

        if (ret1 == -1)
        {

            moveL(Taihu, pos2, 0.2);
            return -1;
        }
    }
    else
    {

        ret1 = moveL(Taihu, goal_last, 0.1);

        if (ret1 == -1)
        {
            moveL(Taihu, pos1, 0.2);

            return -1;
        }
    }
    // {
    //     sleep(2);
    //     MatrixXd current_hand_pos = Taihu.getTcpPos();
    //     cout << "抓取函数last当前位置" << current_hand_pos << endl;
    //     cout << "抓取函数last目标位置" << goal_last[0] << " " << goal_last[1] << " " << goal_last[2] << endl;
    // }

    // sleep(1);

    if (flag == 1)
    {
        if (target == "liziyuan" || target == "green")
        {
            cout << "target == liziyuan || target == green" << endl;
            grasp_hand2(right_a);
        }
        else
        {
            grasp_hand(right_a);
            // sleep(1);
        }
    }
    else
    {
        if (target == "liziyuan" || target == "green")
        {
            cout << "target == liziyuan || target == green" << endl;
            grasp_hand2(left_a);
        }
        else
        {
            grasp_hand(left_a);
            // sleep(1);
        }
    }

    // cin >> stop;
    // cin.ignore(numeric_limits<streamsize>::max(), '\n');

    return 0;
}

int convert_postion(Matrix<double, 1, 6> &goal_last, std::array<double, 3> coords, int flag)
{
    // float rotation = 0 * M_PI / 180;
    // Eigen::Matrix3d R = Roll_Rotation(rotation);
    Eigen::Vector3d t;
    t << coords[0] * 1000, coords[1] * 1000, coords[2] * 1000;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    // T.block<3, 3>(0, 0) = R; // Set rotation
    T.block<3, 1>(0, 3) = t; // Set translation

    // cout << "-----------------------------------------------------------" << endl;

    Eigen::Vector3d goal_t;
    Eigen::Matrix4d robot_t = neck_to_eye(flag) * T;
    // cout << "robot_t: " << endl << robot_t << endl;
    goal_t << robot_t.block<3, 1>(0, 3);
    // cout << goal_t(0) << "," << goal_t(1) << "," << goal_t(2) << endl;
    goal_last << goal_t(0) / 1000, goal_t(1) / 1000, goal_t(2) / 1000, 0, 0, 0;

    return 0;
}

std::string read_server_response()
{
    boost::asio::streambuf response;
    boost::asio::read_until(*g_socket, response, '\n');
    std::istream is(&response);
    std::string result;
    std::getline(is, result);
    return result;
}

bool reals_tcp_init2(const std::string &host, int port)
{
    try
    {
        static boost::asio::io_service io_service;
        g_socket = new tcp::socket(io_service);
        g_socket->connect(tcp::endpoint(boost::asio::ip::address::from_string(host), port));

        // 关键修复：连接后立即读取并丢弃欢迎消息
        std::string welcome_msg = read_server_response();
        std::cout << "服务器问候: " << welcome_msg << std::endl;

        return true;
    }
    catch (...)
    {
        cout << "相机链接失败" << endl;
        return false;
    }
}

int reals_tcp_send_command(const std::string &command)
{
    try
    {
        boost::asio::write(*g_socket, boost::asio::buffer(command + "\n"));
        return 0; // 使用统一响应读取函数
    }
    catch (...)
    {
        return -1;
    }
}

void reals_tcp_close()
{
    if (g_socket)
    {
        try
        {
            if (g_socket->is_open())
            {
                g_socket->close();
            }
            delete g_socket;
            g_socket = nullptr;
            std::cout << "✅ 已断开与Python服务器的连接" << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << "❌ 关闭连接时出错: " << e.what() << std::endl;
        }
    }
}

bool extractCoordinates(const std::string &input, double coordinates[3])
{
    // 找到括号的位置
    size_t start = input.find('(');
    size_t end = input.find(')');

    if (start == std::string::npos || end == std::string::npos)
    {
        std::cerr << "错误：无法找到坐标数据" << std::endl;
        return false;
    }

    // 提取括号内的内容
    std::string coords_str = input.substr(start + 1, end - start - 1);

    // 使用stringstream分割字符串
    std::stringstream ss(coords_str);
    char comma;

    // 直接读取到数组中
    if (!(ss >> coordinates[0] >> comma >> coordinates[1] >> comma >> coordinates[2]))
    {
        std::cerr << "错误：坐标格式不正确" << std::endl;
        return false;
    }

    return true;
}

int extractSegmentNumber(const std::string &input)
{
    const std::string key = "分段编号: ";
    size_t pos = input.find(key); // 查找 "分段编号: " 的位置

    if (pos == std::string::npos)
    {
        std::cerr << "Error: '分段编号' not found in the input string." << std::endl;
        return -1; // 返回错误码
    }

    pos += key.length(); // 跳过 "分段编号: "，定位到数字开始位置
    int segmentNumber = 0;

    // 提取数字部分
    while (pos < input.size() && isdigit(input[pos]))
    {
        segmentNumber = segmentNumber * 10 + (input[pos] - '0');
        pos++;
    }

    return segmentNumber;
}

#include "wt_client.h"

void hand_mode(Robot_Arm &Taihu, int hand_flag, int sock, string target)
{
    // 注意：必须严格使用小写命令

    // string hand_move = "human_hand";

    Matrix<double, 1, 6> human_hand_position;
    // std::vector<std::string> req_right;
    // req_right.reserve(11);
    // req_right.push_back("0");  // alg_witch
    // req_right.push_back("0");  // head_roll
    // req_right.push_back("30"); // head_pitch
    // req_right.push_back("0");  // head_yaw
    // for (int i = 0; i < 6; ++i)
    //     req_right.push_back("0");   // 6 个预留
    // req_right.push_back(hand_move); // 类别_id
    // int error = 0;
    // while (1)
    // {
    //     if (error > 11)
    //     {
    //         break;
    //     }
    //     int get_drink_posi = get_postion(sock, req_right, human_hand_position);
    //     if (get_drink_posi != -1)
    //     {
    //         error=12;
    //         break;
    //     }
    //     ++error;
    // }

    // if (error > 11)
    // {
    if (hand_flag == 1)
    {
        human_hand_position << 0.55, -0.2, -0.21, 0, 0, 0;

        cout << "测试555" << endl;

        human_hand_position[0] = readPiancha("6000");
        human_hand_position[1] = readPiancha("6001");
        human_hand_position[2] = readPiancha("6002");

        moveL(Taihu, human_hand_position, 0.2);
        return;
    }
    else
    {
        human_hand_position << 0.55, 0.2, -0.21, 0, 0, 0;
        human_hand_position[0] = readPiancha("6003");
        human_hand_position[1] = readPiancha("6004");
        human_hand_position[2] = readPiancha("6005");
        moveL(Taihu, human_hand_position, 0.2);
        return;
    }
    // }
}

int moveL(Robot_Arm &Taihu, Matrix<double, 1, 6> posd, double speed)
{
    usleep(250000);
    return arm_line_move(Taihu, posd, speed);
}

string jianlue(string target)
{
    if (target == "mnd" || target == "meinianda")
    {
        return "meinianda";
    }
    else if (target == "xb" || target == "xuebi")
    {
        return "xuebi";
    }
    else if (target == "yq" || target == "yiquan")
    {
        return "yiquan";
    }
    else if (target == "cc" || target == "coca")
    {
        return "coca";
    }
    else if (target == "bskl" || target == "baishikele")
    {
        return "baishikele";
    }
    else if (target == "jlb" || target == "jianlibao")
    {
        return "jianlibao";
    }
    else if (target == "mz" || target == "mozhua")
    {
        return "mozhua";
    }
    else if (target == "wtkl" || target == "wutangkele")
    {
        return "wutangkele";
    }
    else if (target == "md" || target == "maidong")
    {
        return "maidong";
    }
    else if (target == "wlc" || target == "wulongcha")
    {
        return "wulongcha";
    }
    else if (target == "yq" || target == "yuanqi")
    {
        return "yuanqi";
    }
    else if (target == "mzy" || target == "meizhiyuan")
    {
        return "meizhiyuan";
    }
    else if (target == "bhc" || target == "binghongcha")
    {
        return "binghongcha";
    }
    else if (target == "yykx" || target == "yingyangkuaixian")
    {
        return "yingyangkuaixian";
    }
    else if (target == "bkl" || target == "baokuangli")
    {
        return "baokuangli";
    }
    else if (target == "dfsy" || target == "dongfangshuye")
    {
        return "dongfangshuye";
    }
    else
    {
        return "no_drink_name"; // or return target; depending on your needs
    }
}

// 读取偏差文件并返回指定位置的数值（每次调用完整重读文件，便于不停机改参；依赖进程当前工作目录下的 piancha.txt）
double readPiancha(const std::string &identifier)
{
    std::ifstream file("piancha.txt");
    std::map<std::string, double> dataMap; // 使用map存储标识符和数值的映射

    if (file.is_open())
    {
        std::string line;
        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string item;

            // 以逗号为分隔符读取每个条目
            while (std::getline(ss, item, ','))
            {
                // 去除首尾空格
                item.erase(0, item.find_first_not_of(" \t"));
                item.erase(item.find_last_not_of(" \t") + 1);

                // 查找冒号位置
                size_t colonPos = item.find(':');
                if (colonPos != std::string::npos)
                {
                    std::string key = item.substr(0, colonPos);
                    std::string valueStr = item.substr(colonPos + 1);

                    try
                    {
                        double value = std::stod(valueStr);
                        // 同时存储完整标识符和数字部分
                        dataMap[key] = value;

                        // 如果标识符以数字开头，也存储数字部分
                        size_t underscorePos = key.find('_');
                        if (underscorePos != std::string::npos)
                        {
                            std::string numPart = key.substr(0, underscorePos);
                            dataMap[numPart] = value;
                        }
                    }
                    catch (...)
                    {
                        continue; // 跳过无效数值
                    }
                }
            }
        }
        file.close();

        // 查找匹配的标识符
        auto it = dataMap.find(identifier);
        if (it != dataMap.end())
        {
            return it->second;
        }
        else
        {
            std::cerr << "错误：未找到标识符 '" << identifier << "'" << std::endl;
            return 0.0;
        }
    }
    else
    {
        std::cerr << "错误：无法打开文件 piancha.txt" << std::endl;
        return 0.0;
    }
}

void change_pian(string traget, Matrix<double, 1, 6> &goal_last, double lin, aoyi_hand ti5_hand, int drink_layer)
{
    goal_last[2] = -0.165;

    if (drink_layer == 1)
    {
        goal_last[2] = readPiancha("30");
    }
    else if (drink_layer == 2)
    {
        goal_last[2] = readPiancha("31");
    }
    else if (drink_layer == 3)
    {
        goal_last[2] = readPiancha("32");
    }

    if (traget == "meinianda" || traget == "xuebi" || traget == "yiquan" || traget == "coca" || traget == "baishikele" || traget == "jianlibao" || traget == "mozhua" || traget == "wutangkele")
    {
        if (lin > 0.59) // 第二排
        {
            if (ti5_hand == right_a)
            {

                goal_last[1] += readPiancha("13");
                goal_last[0] += readPiancha("9");
            }
            else
            {

                goal_last[1] += readPiancha("15");
                goal_last[0] += readPiancha("11");
            }
        }
        else
        {
            if (ti5_hand == right_a)
            {

                goal_last[1] += readPiancha("12");
                ;
                goal_last[0] += readPiancha("8");
            }
            else
            {

                goal_last[1] += readPiancha("14");
                goal_last[0] += readPiancha("10");
            }
        }
    }
    else
    {
        if (lin < 0.59)
        {
            if (ti5_hand == right_a)
            {

                goal_last[1] += readPiancha("20");
                goal_last[0] += readPiancha("16");
            }
            else
            {

                goal_last[1] += readPiancha("22");
                goal_last[0] += readPiancha("18");
            }
        }
        else
        {
            if (ti5_hand == right_a)
            {

                goal_last[1] += readPiancha("21");
                goal_last[0] += readPiancha("17");
            }
            else
            {

                goal_last[1] += readPiancha("23");
                goal_last[0] += readPiancha("19");
            }
        }
    }
}

int convert_biaoding(Matrix<double, 1, 6> &goal_last, std::array<double, 3> coords)
{

    //     euler_angles_xyz: [-134.1044, 1.6954, -88.71701]
    // matrix: [[0.02238, -0.69627, 0.71743, 98.13279], [-0.99931, 0.00566, 0.03666, 37.77919],
    //   [-0.02959, -0.71776, -0.69566, 180.15813], [0.0, 0.0, 0.0, 1.0]]
    Matrix<double, 4, 4> T_base_eye;
    Matrix<double, 4, 1> T_posd;
    Matrix<double, 4, 1> posd;
    T_base_eye << 0.02238, -0.69627, 0.71743, 98.13279 / 1000,
        -0.99931, 0.00566, 0.03666, 37.77919 / 1000,
        -0.02959, -0.71776, -0.69566, 180.15813 / 1000,
        0.0, 0.0, 0.0, 1.0;
    T_posd << coords[0], coords[1], coords[2], 1;

    posd = T_base_eye * T_posd;
    // cout << "posd: " << posd <<endl;
    // cout << "coords: "  <<coords[0]<< " "<< coords[1]<<" " <<coords[2]<<" " <<endl;
    // cout <<"T_base_eye" <<T_base_eye << endl;
    // cout << "T_posd" <<T_posd <<endl;
    for (int i = 0; i < 3; i++)
    {
        goal_last[i] = posd(i);
    }
    goal_last[3] = 0;
    goal_last[4] = 0;
    goal_last[5] = 0;
    return 0;
}

// =============================================================================
// END: src/reals_tcp.cpp
// =============================================================================

// =============================================================================
// BEGIN: src/Ti5_socketcan.cpp
// =============================================================================
#include "Ti5_socketcan.h"

#include <algorithm>
#include <cmath>
#include <vector>

std::array<int, CAN_CHANNEL_COUNT> can_sockets = {-1, -1, -1, -1, -1, -1};

 int left_h_id = -1, left_m_id = -1, left_l_id = -1, right_h_id = -1, right_m_id = -1, right_l_id = -1;

// int left_h_id = 3,
//     left_m_id = 3,
//     left_l_id = 3,
//     right_h_id = 2,
//     right_m_id = 2,
//     right_l_id = 2;


    // int head_id=1;
    // int waist_id=0;

      int head_id=-1;
    int waist_id=-1;



int convertHexArrayToDecimal2(const uint8_t hexArray[4])
{

    int result = 0;
    for (int i = 0; i < 4; i++)
        result = (result << 8) | hexArray[i];

    // 处理32位有符号数（补码转换）
    if (result > 0x7FFFFFFF)
        result -= 0x100000000;

    return result;
}

bool set_can_bitrate(const std::string &ifname, uint32_t bitrate)
{
    std::string cmd = "sudo ip link set " + ifname + " down && " +
                      "sudo ip link set " + ifname +
                      " up type can bitrate " + std::to_string(bitrate);

    std::cout << "Executing: " << cmd << std::endl;
    int ret = system(cmd.c_str());
    if (ret != 0)
    {
        std::cerr << "Command failed with code: " << ret << std::endl;
        return false;
    }
    return true;
}

bool init_can_channel(int channel, const CAN_CONFIG &config)
{
    if (channel < 0 || channel >= CAN_CHANNEL_COUNT)
    {
        std::cerr << "Invalid channel number: " << channel << std::endl;
        return false;
    }

    std::string ifname = "can" + std::to_string(channel);

    if (!set_can_bitrate(ifname, config.Bitrate))
    {
        std::cerr << "Failed to set bitrate for " << ifname << std::endl;
        return false;
    }

    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0)
    {
        perror(("Socket creation failed for " + ifname).c_str());
        return false;
    }

    if (config.Filter)
    {
        struct can_filter rfilter;
        rfilter.can_id = config.AccCode;
        rfilter.can_mask = config.AccMask;

        if (setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) < 0)
        {
            perror("Failed to set CAN filter");
            close(sock);
            return false;
        }
    }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, ifname.c_str());
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0)
    {
        perror(("Interface index get failed for " + ifname).c_str());
        close(sock);
        return false;
    }

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror(("Socket bind failed for " + ifname).c_str());
        close(sock);
        return false;
    }

    int loopback = config.Mode ? 1 : 0;
    if (setsockopt(sock, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback)) < 0)
    {
        perror("Failed to set loopback mode");
        close(sock);
        return false;
    }

    int flags = fcntl(sock, F_GETFL, 0);
    if (flags == -1)
    {
        perror("Failed to get socket flags");
        close(sock);
        return false;
    }
    if (fcntl(sock, F_SETFL, flags | O_NONBLOCK) == -1)
    {
        perror("Failed to set non-blocking mode");
        close(sock);
        return false;
    }

    // if (fcntl(sock, F_SETFL, 0) == -1)
    // {
    //     perror("Failed to set blocking mode");
    //     close(sock);
    //     return false;
    // }

    can_sockets[channel] = sock;
    std::cout << "Successfully initialized " << ifname
              << " with bitrate=" << config.Bitrate / 1000 << "kbps"
              << ", filter=0x" << std::hex << config.AccCode
              << ", mask=0x" << config.AccMask << std::dec << std::endl;

    return true;
}

bool init_socketcan()
{
    CAN_CONFIG config;
    config.AccCode = 0;
    config.AccMask = 0;
    config.Filter = 1;
    config.Bitrate = 1000000;
    config.Mode = 0;

    for (int i = 0; i < CAN_CHANNEL_COUNT; i++)
    {
        if (!init_can_channel(i, config))
        {
            std::cerr << "Failed to initialize channel " << i << std::endl;
            close_can_channels();
            return false;
        }
    }
    cout << endl;
    cout << endl;
    cout << endl;
    cout << endl;
    cout << endl;

    return true;
}

bool init_socketcan_can2()
{
    CAN_CONFIG config;
    config.AccCode = 0;
    config.AccMask = 0;
    config.Filter = 1;
    config.Bitrate = 1000000;
    config.Mode = 0;

  
        if (!init_can_channel(2, config))
        {
            std::cerr << "Failed to initialize channel " << 2 << std::endl;
            close_can_channels2();
            return false;
        }
    
    cout << endl;
    cout << endl;
    cout << endl;
    cout << endl;
    cout << endl;

    return true;
}

void close_can_channels()
{
    for (int i = 0; i < CAN_CHANNEL_COUNT; i++)
    {
        if (can_sockets[i] >= 0)
        {
            close(can_sockets[i]);
            can_sockets[i] = -1;
            std::cout << "Closed channel can" << i << std::endl;
        }
    }
}

void close_can_channels2()
{
    
        if (can_sockets[2] >= 0)
        {
            close(can_sockets[2]);
            can_sockets[2] = -1;
            std::cout << "Closed channel can" << 2 << std::endl;
        }
    
}

bool send_can_frame(int channel, uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    // if (channel < 0 || channel >= CAN_CHANNEL_COUNT)
    // {
    //     std::cerr << "Invalid channel number: " << channel << std::endl;
    //     return false;
    // }

    // if (can_sockets[channel] < 0)
    // {
    //     std::cerr << "Channel " << channel << " not initialized" << std::endl;
    //     return false;
    // }

    // if (dlc > 8)
    // {
    //     std::cerr << "DLC cannot be greater than 8" << std::endl;
    //     return false;
    // }

    struct can_frame frame;
    frame.can_id = can_id;
    frame.can_dlc = dlc;
    memcpy(frame.data, data, dlc);

    int bytes_sent = write(can_sockets[channel], &frame, sizeof(frame));

    if (bytes_sent != sizeof(frame))
    {
        // perror("CAN frame send failed");
        return false;
    }

    return true;
}

bool receive_can_frame_timeout(int channel, uint8_t *data)
{

    // if (can_sockets[channel] < 0)
    // {
    //     std::cerr << "Channel " << channel << " not initialized" << std::endl;
    //     return false;
    // }

    usleep(200);
    //  3. 直接尝试读取数据
    can_frame frame;

    int bytes_read = read(can_sockets[channel], &frame, sizeof(frame));

    // if(channel!=5)
    // {
    //     cout <<"bytes_read" <<bytes_read <<endl;
    // }

    // 5. 处理结果
    if (bytes_read == sizeof(frame))
    {

        memcpy(data, frame.data, frame.can_dlc);
        return true;
    }
    else if (bytes_read == -1 && errno == EAGAIN)
    {
        // cout <<"bytes_read" <<bytes_read <<endl;

        return false; // 无数据可读
    }

    else
    {
        perror("CAN frame receive failed");
        return false;
    }

    return false;
}

void toIntArray(int number, uint8_t *res, int size)
{
    unsigned int unsignedNumber = static_cast<unsigned int>(number);
    for (int i = 0; i < size; ++i)
    {
        res[i] = unsignedNumber & 0xFF;
        unsignedNumber >>= 8;
    }
}

int socketcan_sendcommand(int channel, int cannum, uint32_t *can_idlist, uint8_t command, int *data)
{
    int flag = 0;

    for (int i = 0; i < cannum; i++)
    {
        uint8_t combinedData[5] = {0};

        // int intValue = static_cast<int>(0 / 360.0 * 4 * 65536);

        combinedData[0] = command;

        uint8_t intBytes[4];
        toIntArray(data[i], intBytes, 4);

        for (int j = 1; j < 5; j++)
        {
            combinedData[j] = intBytes[j - 1];
        }

        if (send_can_frame(channel, can_idlist[i], combinedData, 5))
        {
            // std::cout << "Frame sent successfully on can" << channel << "___Frame sent successfully on canid" << can_idlist[i] << std::endl;
        }
        else
        {
             std::cout << "\033[31msend_mode sent failed on can" << channel << "___Frame sent failed on canid" << can_idlist[i] << "\033[0m" << std::endl;
            flag = -1;
        }
        usleep(14);
    }
    if (flag == -1)
    {
        return -1;
    }
    return 1;
    
}

int socketcan_sendsimplecommand(int channel, int cannum, uint32_t *can_idlist, uint8_t command, int *data)
{
    int flag = 0;

    for (int i = 0; i < cannum; i++)
    {
        uint8_t combinedData[1] = {0};

        combinedData[0] = command;

        // usleep(30);

        if (send_can_frame(channel, can_idlist[i], combinedData, 1))
        {
            // std::cout << "Frame sent successfully on can" << channel << "___Frame sent successfully on canid"<< can_idlist[i]<< std::endl;
        }
        else
        {
            std::cout << "Frame sent faild on can" << channel << "___Frame sent faild on canid" << can_idlist[i] << std::endl;
            flag = -1;
        }

        if(command == 11)
        {
            return 0;
        }

        uint8_t recv_data[4];
        int cnt = 11;

        while ((!(receive_can_frame_timeout(channel, recv_data))) && cnt)
        {
            cnt--;
        }
        if (cnt != 0)
        {
            // 1. 将 recv_data 按小端序排列（recv_data[0]是接收到的第一个字节，作为最低字节）
            uint8_t hexArray[4] = {recv_data[4], recv_data[3], recv_data[2], recv_data[1]};

            // 2. 调用转换函数
            data[i] = convertHexArrayToDecimal2(hexArray);
        }
        else
        {
            std::cout << "\033[1;31mcan can can can  recvie  recvie recvie failed on can" << channel 
          << "can can can recvie recvie recvie recvie recvie failed on canid" 
          << can_idlist[i] << "\033[0m" << std::endl; }

        // std::cout << "cnn: " << cnt << "次" << std::endl;

        // if ((receive_can_frame_timeout(channel, recv_data)))
        // {
        //     // 1. 将 recv_data 按小端序排列（recv_data[0]是接收到的第一个字节，作为最低字节）
        //     uint8_t hexArray[4] = {recv_data[4], recv_data[3], recv_data[2], recv_data[1]};

        //     // 2. 调用转换函数
        //     data[i] = convertHexArrayToDecimal2(hexArray);
        // }
        // else
        // {
        //     std::cout << "\033[31mFrame sent failed on can" << channel << "___Frame sent failed on canid" << can_idlist[i] << "\033[0m" << std::endl;
        // }
    }
    if (flag == -1)
    {
        return -1;
    }
    return 1;
}

bool select_bind(int &hand_id, int id, string name)
{
    uint8_t data[4];

    int flag_id = 0;

    uint8_t combinedData[1] = {3};

    while (1)
    {
        usleep(3000);
        if (hand_id == -1 && send_can_frame(flag_id, id, combinedData, 1))
        {
            // cout << "绑定中" << endl;
            usleep(3900);
            int cnt = 10;
            while ((!(receive_can_frame_timeout(flag_id, data))) && cnt)
            {
                cnt--;
            }
            if (cnt != 0)
            {
                hand_id = flag_id;
                cout << "successfull bind->" << name << " = " << flag_id << endl;
                break;
            }
        }
        else
        {
            break;
        }
        if (flag_id >= 5)
        {
            flag_id = 0;
        }
        flag_id++;
    }
    return true;
}

void hand_socketid_bind()
{

    // int id1 = 16;
    // int id2 = 19;
    // int id3 = 21;
    // int id4 = 23;
    // int id5 = 26;
    // int id6 = 28;

    int id1 = 23;
    int id2 = 26;
    int id3 = 28;
    int id4 = 16;
    int id5 = 19;
    int id6 = 21;

    select_bind(left_h_id, id1, "left_h_id");
    select_bind(left_m_id, id2, "left_m_id");
    select_bind(left_l_id, id3, "left_l_id");
    select_bind(right_h_id, id4, "right_h_id");
    select_bind(right_m_id, id5, "right_m_id");
    select_bind(right_l_id, id6, "right_l_id");

     select_bind(waist_id, 2, "waist_id");
    select_bind(head_id, 31, "head_id");
   
}

void get_motor_position(int *data, int a)
{
    if (a == 1)
    {
        // 定义CAN ID列表
        uint32_t can_idlist_h[2] = {16, 17};
        uint32_t can_idlist_m[2] = {18, 19};
        uint32_t can_idlist_l[3] = {20, 21, 22};

        // // 创建线程容器
        // std::vector<std::thread> threads;

        // // 线程1：高位指令
        // threads.emplace_back([&]()
        //                      { socketcan_sendsimplecommand(right_h_id, 2, can_idlist_h, 8, data); });

        // // 线程2：中位指令
        // threads.emplace_back([&]()
        //                      { socketcan_sendsimplecommand(right_m_id, 2, can_idlist_m, 8, data + 2); });

        // // 线程3：低位指令
        // threads.emplace_back([&]()

        //                      { socketcan_sendsimplecommand(right_l_id, 3, can_idlist_l, 8, data + 4); });

        // // 等待所有线程完成
        // for (auto &t : threads)
        // {
        //     if (t.joinable())
        //     {
        //         t.join();
        //     }
        // }

        socketcan_sendsimplecommand(right_h_id, 2, can_idlist_h, 8, data);

        socketcan_sendsimplecommand(right_m_id, 2, can_idlist_m, 8, data + 2);

        // 线程3：低位指令
        socketcan_sendsimplecommand(right_l_id, 3, can_idlist_l, 8, data + 4);
    }
    else
    {
        uint32_t can_idlist_h[2] = {23, 24};
        uint32_t can_idlist_m[2] = {25, 26};
        uint32_t can_idlist_l[3] = {27, 28, 29};

        // std::vector<std::thread> threads;

        // threads.emplace_back([&]()
        //                      { socketcan_sendsimplecommand(left_h_id, 2, can_idlist_h, 8, data); });

        // threads.emplace_back([&]()
        //                      { socketcan_sendsimplecommand(left_m_id, 2, can_idlist_m, 8, data + 2); });

        // threads.emplace_back([&]()
        //                      { socketcan_sendsimplecommand(left_l_id, 3, can_idlist_l, 8, data + 4); });

        // for (auto &t : threads)
        // {
        //     if (t.joinable())
        //     {
        //         t.join();
        //     }
        // }

        socketcan_sendsimplecommand(left_h_id, 2, can_idlist_h, 8, data);

        socketcan_sendsimplecommand(left_m_id, 2, can_idlist_m, 8, data + 2);

        // 线程3：低位指令
        socketcan_sendsimplecommand(left_l_id, 3, can_idlist_l, 8, data + 4);
    }
}

void get_motor_running_speed(int *data, int a)
{
    constexpr uint8_t kCmdReadRunSpeed = 6;
    if (a == 1)
    {
        uint32_t can_idlist_h[2] = {16, 17};
        uint32_t can_idlist_m[2] = {18, 19};
        uint32_t can_idlist_l[3] = {20, 21, 22};

        socketcan_sendsimplecommand(right_h_id, 2, can_idlist_h, kCmdReadRunSpeed, data);
        socketcan_sendsimplecommand(right_m_id, 2, can_idlist_m, kCmdReadRunSpeed, data + 2);
        socketcan_sendsimplecommand(right_l_id, 3, can_idlist_l, kCmdReadRunSpeed, data + 4);
    }
    else
    {
        uint32_t can_idlist_h[2] = {23, 24};
        uint32_t can_idlist_m[2] = {25, 26};
        uint32_t can_idlist_l[3] = {27, 28, 29};

        socketcan_sendsimplecommand(left_h_id, 2, can_idlist_h, kCmdReadRunSpeed, data);
        socketcan_sendsimplecommand(left_m_id, 2, can_idlist_m, kCmdReadRunSpeed, data + 2);
        socketcan_sendsimplecommand(left_l_id, 3, can_idlist_l, kCmdReadRunSpeed, data + 4);
    }
}

void set_motor_position(int *data, int a)
{
    if (a == 1)
    {
        uint32_t can_idlist_h[2] = {16, 17};
        uint32_t can_idlist_m[2] = {18, 19};
        uint32_t can_idlist_l[3] = {20, 21, 22};

        // // 创建线程容器
        // std::vector<std::thread> threads;

        // // 线程1：发送高位指令
        // threads.emplace_back([&]()
        //                      { socketcan_sendcommand(right_h_id, 2, can_idlist_h, 30, data); });

        // // 线程2：发送中位指令
        // threads.emplace_back([&]()
        //                      { socketcan_sendcommand(right_m_id, 2, can_idlist_m, 30, data + 2); });

        // // 线程3：发送低位指令
        // threads.emplace_back([&]()
        //                      { socketcan_sendcommand(right_l_id, 3, can_idlist_l, 30, data + 4); });

        // // 等待所有线程完成
        // for (auto &t : threads)
        // {
        //     t.join();
        // }

        socketcan_sendcommand(right_h_id, 2, can_idlist_h, 30, data);
        ;

        socketcan_sendcommand(right_m_id, 2, can_idlist_m, 30, data + 2);
        ;

        socketcan_sendcommand(right_l_id, 3, can_idlist_l, 30, data + 4);
        ;
    }
    else
    {
        uint32_t can_idlist_h[2] = {23, 24};
        uint32_t can_idlist_m[2] = {25, 26};
        uint32_t can_idlist_l[3] = {27, 28, 29};

        // std::vector<std::thread> threads;

        // threads.emplace_back([&]()
        //                      { socketcan_sendcommand(left_h_id, 2, can_idlist_h, 30, data); });

        // threads.emplace_back([&]()
        //                      { socketcan_sendcommand(left_m_id, 2, can_idlist_m, 30, data + 2); });

        // threads.emplace_back([&]()
        //                      { socketcan_sendcommand(left_l_id, 3, can_idlist_l, 30, data + 4); });

        // for (auto &t : threads)
        // {
        //     t.join();
        // }

        socketcan_sendcommand(left_h_id, 2, can_idlist_h, 30, data);
        ;

        socketcan_sendcommand(left_m_id, 2, can_idlist_m, 30, data + 2);
        ;

        socketcan_sendcommand(left_l_id, 3, can_idlist_l, 30, data + 4);
    }
}

void get_motor_speed(int *data, int a)
{

    uint8_t speed_m = 0;
    if (data[0] > 0)
    {
        speed_m = 24;
    }
    else
    {
        speed_m = 25;
    }
    if (a == 1)
    {
        // 定义CAN ID列表
        uint32_t can_idlist_h[2] = {16, 17};
        uint32_t can_idlist_m[2] = {18, 19};
        uint32_t can_idlist_l[3] = {20, 21, 22};

        // 创建线程容器
        std::vector<std::thread> threads;

        // 线程1：高位指令
        threads.emplace_back([&]()
                             { socketcan_sendsimplecommand(right_h_id, 2, can_idlist_h, speed_m, data); });

        // 线程2：中位指令
        threads.emplace_back([&]()
                             { socketcan_sendsimplecommand(right_m_id, 2, can_idlist_m, speed_m, data + 2); });

        // 线程3：低位指令
        threads.emplace_back([&]()

                             { socketcan_sendsimplecommand(right_l_id, 3, can_idlist_l, speed_m, data + 4); });

        // socketcan_sendsimplecommand(right_h_id, 2, can_idlist_h, 8, data);

        // socketcan_sendsimplecommand(right_m_id, 2, can_idlist_m, 8, data + 2);

        // // 线程3：低位指令
        // socketcan_sendsimplecommand(right_l_id, 3, can_idlist_l, 8, data + 4);

        // 等待所有线程完成
        for (auto &t : threads)
        {
            if (t.joinable())
            {
                t.join();
            }
        }
    }
    else
    {
        uint32_t can_idlist_h[2] = {23, 24};
        uint32_t can_idlist_m[2] = {25, 26};
        uint32_t can_idlist_l[3] = {27, 28, 29};

        std::vector<std::thread> threads;

        threads.emplace_back([&]()
                             { socketcan_sendsimplecommand(left_h_id, 2, can_idlist_h, speed_m, data); });

        threads.emplace_back([&]()
                             { socketcan_sendsimplecommand(left_m_id, 2, can_idlist_m, speed_m, data + 2); });

        threads.emplace_back([&]()
                             { socketcan_sendsimplecommand(left_l_id, 3, can_idlist_l, speed_m, data + 4); });

        for (auto &t : threads)
        {
            if (t.joinable())
            {
                t.join();
            }
        }
    }
}

void set_motor_addspeed(int *data, int a)
{
    uint8_t speed_m = 0;
    if (data[0] > 0)
    {
        speed_m = 34;
    }
    else
    {
        speed_m = 35;
    }
    if (a == 1)
    {

        uint32_t can_idlist_h[2] = {16, 17};
        uint32_t can_idlist_m[2] = {18, 19};
        uint32_t can_idlist_l[3] = {20, 21, 22};

        // 创建线程容器
        std::vector<std::thread> threads;

        // 线程1：发送高位指令
        threads.emplace_back([&]()
                             { socketcan_sendcommand(right_h_id, 2, can_idlist_h, speed_m, data); });

        // 线程2：发送中位指令
        threads.emplace_back([&]()
                             { socketcan_sendcommand(right_m_id, 2, can_idlist_m, speed_m, data + 2); });

        // 线程3：发送低位指令
        threads.emplace_back([&]()
                             { socketcan_sendcommand(right_l_id, 3, can_idlist_l, speed_m, data + 4); });

        // socketcan_sendcommand(right_h_id, 2, can_idlist_h, 30, data);
        // ;

        // socketcan_sendcommand(right_m_id, 2, can_idlist_m, 30, data + 2);
        // ;

        // socketcan_sendcommand(right_l_id, 3, can_idlist_l, 30, data + 4);
        // ;

        // 等待所有线程完成
        for (auto &t : threads)
        {
            t.join();
        }
    }
    else
    {
        uint32_t can_idlist_h[2] = {23, 24};
        uint32_t can_idlist_m[2] = {25, 26};
        uint32_t can_idlist_l[3] = {27, 28, 29};

        std::vector<std::thread> threads;

        threads.emplace_back([&]()
                             { socketcan_sendcommand(left_h_id, 2, can_idlist_h, speed_m, data); });

        threads.emplace_back([&]()
                             { socketcan_sendcommand(left_m_id, 2, can_idlist_m, speed_m, data + 2); });

        threads.emplace_back([&]()
                             { socketcan_sendcommand(left_l_id, 3, can_idlist_l, speed_m, data + 4); });

        for (auto &t : threads)
        {
            t.join();
        }
    }
}

void get_motor_addspeed(int *data, int a)
{

    uint8_t speed_m = 0;
    if (data[0] > 0)
    {
        speed_m = 22;
    }
    else
    {
        speed_m = 23;
    }
    if (a == 1)
    {
        // 定义CAN ID列表
        uint32_t can_idlist_h[2] = {16, 17};
        uint32_t can_idlist_m[2] = {18, 19};
        uint32_t can_idlist_l[3] = {20, 21, 22};

        // 创建线程容器
        std::vector<std::thread> threads;

        // 线程1：高位指令
        threads.emplace_back([&]()
                             { socketcan_sendsimplecommand(right_h_id, 2, can_idlist_h, speed_m, data); });

        // 线程2：中位指令
        threads.emplace_back([&]()
                             { socketcan_sendsimplecommand(right_m_id, 2, can_idlist_m, speed_m, data + 2); });

        // 线程3：低位指令
        threads.emplace_back([&]()

                             { socketcan_sendsimplecommand(right_l_id, 3, can_idlist_l, speed_m, data + 4); });

        // socketcan_sendsimplecommand(right_h_id, 2, can_idlist_h, 8, data);

        // socketcan_sendsimplecommand(right_m_id, 2, can_idlist_m, 8, data + 2);

        // // 线程3：低位指令
        // socketcan_sendsimplecommand(right_l_id, 3, can_idlist_l, 8, data + 4);

        // 等待所有线程完成
        for (auto &t : threads)
        {
            if (t.joinable())
            {
                t.join();
            }
        }
    }
    else
    {
        uint32_t can_idlist_h[2] = {23, 24};
        uint32_t can_idlist_m[2] = {25, 26};
        uint32_t can_idlist_l[3] = {27, 28, 29};

        std::vector<std::thread> threads;

        threads.emplace_back([&]()
                             { socketcan_sendsimplecommand(left_h_id, 2, can_idlist_h, speed_m, data); });

        threads.emplace_back([&]()
                             { socketcan_sendsimplecommand(left_m_id, 2, can_idlist_m, speed_m, data + 2); });

        threads.emplace_back([&]()
                             { socketcan_sendsimplecommand(left_l_id, 3, can_idlist_l, speed_m, data + 4); });

        for (auto &t : threads)
        {
            if (t.joinable())
            {
                t.join();
            }
        }
    }
}

void set_motor_speed(int *data, int a)
{
    uint8_t speed_m = 0;
    if (data[0] > 0)
    {
        speed_m = 36;
    }
    else
    {
        speed_m = 37;
    }
    if (a == 1)
    {

        uint32_t can_idlist_h[2] = {16, 17};
        uint32_t can_idlist_m[2] = {18, 19};
        uint32_t can_idlist_l[3] = {20, 21, 22};

        // 创建线程容器
        std::vector<std::thread> threads;

        // 线程1：发送高位指令
        threads.emplace_back([&]()
                             { socketcan_sendcommand(right_h_id, 2, can_idlist_h, speed_m, data); });

        // 线程2：发送中位指令
        threads.emplace_back([&]()
                             { socketcan_sendcommand(right_m_id, 2, can_idlist_m, speed_m, data + 2); });

        // 线程3：发送低位指令
        threads.emplace_back([&]()
                             { socketcan_sendcommand(right_l_id, 3, can_idlist_l, speed_m, data + 4); });

        // socketcan_sendcommand(right_h_id, 2, can_idlist_h, 30, data);
        // ;

        // socketcan_sendcommand(right_m_id, 2, can_idlist_m, 30, data + 2);
        // ;

        // socketcan_sendcommand(right_l_id, 3, can_idlist_l, 30, data + 4);
        // ;

        // 等待所有线程完成
        for (auto &t : threads)
        {
            t.join();
        }
    }
    else
    {
        uint32_t can_idlist_h[2] = {23, 24};
        uint32_t can_idlist_m[2] = {25, 26};
        uint32_t can_idlist_l[3] = {27, 28, 29};

        std::vector<std::thread> threads;

        threads.emplace_back([&]()
                             { socketcan_sendcommand(left_h_id, 2, can_idlist_h, speed_m, data); });

        threads.emplace_back([&]()
                             { socketcan_sendcommand(left_m_id, 2, can_idlist_m, speed_m, data + 2); });

        threads.emplace_back([&]()
                             { socketcan_sendcommand(left_l_id, 3, can_idlist_l, speed_m, data + 4); });

        for (auto &t : threads)
        {
            t.join();
        }
    }
}

void set_motor(int *data, uint8_t command, int a)
{
  
    if (a == 1)
    {

        uint32_t can_idlist_h[2] = {16, 17};
        uint32_t can_idlist_m[2] = {18, 19};
        uint32_t can_idlist_l[3] = {20, 21, 22};

        // // 创建线程容器
        // std::vector<std::thread> threads;

        // // 线程1：发送高位指令
        // threads.emplace_back([&]()
        //                      { socketcan_sendcommand(right_h_id, 2, can_idlist_h, command, data); });

        // // 线程2：发送中位指令
        // threads.emplace_back([&]()
        //                      { socketcan_sendcommand(right_m_id, 2, can_idlist_m, command, data + 2); });

        // // 线程3：发送低位指令
        // threads.emplace_back([&]()
        //                      { socketcan_sendcommand(right_l_id, 3, can_idlist_l, command, data + 4); });

        socketcan_sendcommand(right_h_id, 2, can_idlist_h, command, data);
        

        socketcan_sendcommand(right_m_id, 2, can_idlist_m, command, data + 2);
        

        socketcan_sendcommand(right_l_id, 3, can_idlist_l, command, data + 4);
        

        // // 等待所有线程完成
        // for (auto &t : threads)
        // {
        //     t.join();
        // }
    }
    else
    {
        uint32_t can_idlist_h[2] = {23, 24};
        uint32_t can_idlist_m[2] = {25, 26};
        uint32_t can_idlist_l[3] = {27, 28, 29};


        socketcan_sendcommand(left_h_id, 2, can_idlist_h, command, data);
        

        socketcan_sendcommand(left_m_id, 2, can_idlist_m, command, data + 2);
        

        socketcan_sendcommand(left_l_id, 3, can_idlist_l, command, data + 4);
        

        // std::vector<std::thread> threads;

        // threads.emplace_back([&]()
        //                      { socketcan_sendcommand(left_h_id, 2, can_idlist_h, command, data); });

        // threads.emplace_back([&]()
        //                      { socketcan_sendcommand(left_m_id, 2, can_idlist_m, command, data + 2); });

        // threads.emplace_back([&]()
        //                      { socketcan_sendcommand(left_l_id, 3, can_idlist_l, command, data + 4); });

        // for (auto &t : threads)
        // {
        //     t.join();
        // }
    }
}


void set_motor_singn_mode(int *data,uint8_t command, int a)
{
    if (a == 1)
    {
        // 定义CAN ID列表
        uint32_t can_idlist_h[2] = {16, 17};
        uint32_t can_idlist_m[2] = {18, 19};
        uint32_t can_idlist_l[3] = {20, 21, 22};

        // // 创建线程容器
        // std::vector<std::thread> threads;

        // // 线程1：高位指令
        // threads.emplace_back([&]()
        //                      { socketcan_sendsimplecommand(right_h_id, 2, can_idlist_h, 8, data); });

        // // 线程2：中位指令
        // threads.emplace_back([&]()
        //                      { socketcan_sendsimplecommand(right_m_id, 2, can_idlist_m, 8, data + 2); });

        // // 线程3：低位指令
        // threads.emplace_back([&]()

        //                      { socketcan_sendsimplecommand(right_l_id, 3, can_idlist_l, 8, data + 4); });

        // // 等待所有线程完成
        // for (auto &t : threads)
        // {
        //     if (t.joinable())
        //     {
        //         t.join();
        //     }
        // }

        socketcan_sendsimplecommand(right_h_id, 2, can_idlist_h, command, data);

        socketcan_sendsimplecommand(right_m_id, 2, can_idlist_m, command, data + 2);

        // 线程3：低位指令
        socketcan_sendsimplecommand(right_l_id, 3, can_idlist_l, command, data + 4);
    }
    else
    {
        uint32_t can_idlist_h[2] = {23, 24};
        uint32_t can_idlist_m[2] = {25, 26};
        uint32_t can_idlist_l[3] = {27, 28, 29};

        // std::vector<std::thread> threads;

        // threads.emplace_back([&]()
        //                      { socketcan_sendsimplecommand(left_h_id, 2, can_idlist_h, 8, data); });

        // threads.emplace_back([&]()
        //                      { socketcan_sendsimplecommand(left_m_id, 2, can_idlist_m, 8, data + 2); });

        // threads.emplace_back([&]()
        //                      { socketcan_sendsimplecommand(left_l_id, 3, can_idlist_l, 8, data + 4); });

        // for (auto &t : threads)
        // {
        //     if (t.joinable())
        //     {
        //         t.join();
        //     }
        // }

        socketcan_sendsimplecommand(left_h_id, 2, can_idlist_h, command, data);

        socketcan_sendsimplecommand(left_m_id, 2, can_idlist_m, command, data + 2);

        // 线程3：低位指令
        socketcan_sendsimplecommand(left_l_id, 3, can_idlist_l, command, data + 4);
    }
}



void set_motor_current_zero()
{
    int data[7]={0,0,0,0,0,0,0};
    set_motor(data, 28, 0);
    set_motor(data, 28, 1);
}
// {

//     int data[7]={9000,9000,9000,9000,9000,9000,9000};
//     int data2[7]={-9000,-9000,-9000,-9000,-9000,-9000,-9000};
//     int data3[7]={9000,9000,9000,9000,9000,9000,9000};
//     int data4[7]={-9000,-9000,-9000,-9000,-9000,-9000,-9000};

//      set_motor_speed(data, 1);
//       set_motor_speed(data2, 1);
//        set_motor_speed(data3, 0);
//         set_motor_speed(data4, 0);

// }

// sleep(5);

// {
//     int data[7]={1,0,0,0,0,0,0};
//     int data2[7]={-1,0,0,0,0,0,0};
//     int data3[7]={1,0,0,0,0,0,0};
//     int data4[7]={-1,0,0,0,0,0,0};

//      get_motor_speed(data, 1);
//       get_motor_speed(data2, 1);
//        get_motor_speed(data3, 0);
//         get_motor_speed(data4, 0);

//         cout << "r speed:" << data[0] << " " << data[1] << " " << data[2] << " " << data[3] << " " << data[4] << " " << data[5] << " " << data[6] << " " << endl;
//         cout << "r speed:" << data2[0] << " " << data2[1] << " " << data2[2] << " " << data2[3] << " " << data2[4] << " " << data2[5] << " " << data2[6] << " " << endl;
//         cout << "r speed:" << data3[0] << " " << data3[1] << " " << data3[2] << " " << data3[3] << " " << data3[4] << " " << data3[5] << " " << data3[6] << " " << endl;
//         cout << "r speed:" << data4[0] << " " << data4[1] << " " << data4[2] << " " << data4[3] << " " << data4[4] << " " << data4[5] << " " << data4[6] << " " << endl;

// }

// {

//     int data[7]={30,30,30,30,30,30,30};
//     int data2[7]={-30,-30,-30,-30,-30,-30,-30};
//     int data3[7]={30,30,30,30,30,30,30};
//     int data4[7]={-30,-30,-30,-30,-30,-30,-30};

//      set_motor_addspeed(data, 1);
//       set_motor_addspeed(data2, 1);
//        set_motor_addspeed(data3, 0);
//         set_motor_addspeed(data4, 0);

// }

// sleep(2);

// {
//     int data[7] = {1, 0, 0, 0, 0, 0, 0};
//     int data2[7] = {-1, 0, 0, 0, 0, 0, 0};
//     int data3[7] = {1, 0, 0, 0, 0, 0, 0};
//     int data4[7] = {-1, 0, 0, 0, 0, 0, 0};

//     get_motor_addspeed(data, 1);
//     get_motor_addspeed(data2, 1);
//     get_motor_addspeed(data3, 0);
//     get_motor_addspeed(data4, 0);

//     cout << "r speed:" << data[0] << " " << data[1] << " " << data[2] << " " << data[3] << " " << data[4] << " " << data[5] << " " << data[6] << " " << endl;
//     cout << "r speed:" << data2[0] << " " << data2[1] << " " << data2[2] << " " << data2[3] << " " << data2[4] << " " << data2[5] << " " << data2[6] << " " << endl;
//     cout << "r speed:" << data3[0] << " " << data3[1] << " " << data3[2] << " " << data3[3] << " " << data3[4] << " " << data3[5] << " " << data3[6] << " " << endl;
//     cout << "r speed:" << data4[0] << " " << data4[1] << " " << data4[2] << " " << data4[3] << " " << data4[4] << " " << data4[5] << " " << data4[6] << " " << endl;
// }

// sleep(10000);

// while (1)
// {

// int data2[7] = {0, 0, 0, 0, 0, 0};

// set_motor_position(data2, 1);
// set_motor_position(data2, 0);

//     sleep(4);

//     // int data3[7] = {static_cast<int>(-0.561221 * 180 / 3.14 * 4 * 65536/360),static_cast<int>(-0.203564 * 180 / 3.14 * 4 * 65536/360),static_cast<int>(1.73402 * 180 / 3.14 * 4 * 65536/360),
//     //     static_cast<int>(0.870989 * 180 / 3.14 * 4 * 65536/360),static_cast<int>(-1.53887 * 180 / 3.14 * 4 * 65536/360),
//     //     static_cast<int>(-0.0890188 * 180 / 3.14 * 4 * 65536/360),static_cast<int>(-0.221013* 180 / 3.14 * 4 * 65536/360)};
//     // int data4[7] = {static_cast<int>(-0.544108 * 180 / 3.14 * 4 * 65536/360),static_cast<int>(0.203472 * 180 / 3.14 * 4 * 65536/360),static_cast<int>(-1.77717 * 180 / 3.14 * 4 * 65536/360),
//     //     static_cast<int>(-0.87906 * 180 / 3.14 * 4 * 65536/360),static_cast<int>(1.5303 * 180 / 3.14 * 4 * 65536/360),
//     //     static_cast<int>(-0.0800856 * 180 / 3.14 * 4 * 65536/360),static_cast<int>(-0.2204776 * 180 / 3.14 * 4 * 65536/360)};

//     //     for(int i=0;i<7;i++)
//     //     {
//     //         cout << data3[i] <<endl;
//     //          cout << data4[i] <<endl;
//     //     }
//     //        // cin >> ahhh;
//     // set_motor_position(data3,1);
//     // set_motor_position(data4,0);

//  //     sleep(4);

//         //     return 0;
//         // }

//         Matrix<double, 1, 6> pos1, pos2, goal_last, pos3;

//         pos1 << 0.25, -0.3, -0.15, 0, 0, 0;
//         pos2 << 0.25, 0.4, -0.15, 0, 0, 0;
//         pos3 << 0.45, -0.15, -0.2, 0, 0, 0;

//         while (1)
//         {
//             int a;
//             cin >> a;
//             Taihu_r.moveLToPos(pos1, 1);
//             pos1[0] += 0.2;

//             cin >> a;

//             Taihu_r.moveLToPos(pos1, 1);

//             pos1[0] -= 0.2;

//             // Taihu_l.moveLToPos(pos2, 1);
//         }

//         // sleep(2);




void set_waist_motor_position(int *data)
{
   
        uint32_t can_idlist[3] = {4, 3,2};
      

       

       

        socketcan_sendcommand(waist_id, 3, can_idlist, 30, data ); 
  
   
}


void get_motor_waist_position(int *data)
{
  
      
        uint32_t can_idlist[3] = {4, 3, 2};


      

       
        socketcan_sendsimplecommand(waist_id, 3, can_idlist, 8, data );

    //    cout <<
    
  
}

namespace
{

constexpr double kMotorCntPerDeg = 65536.0 * 4.0 / 360.0;

double quintic_scalar(double t, double total_time, double start, double target)
{
    if (total_time <= 1e-6)
        return target;

    const double ratio = std::clamp(t / total_time, 0.0, 1.0);
    const double r2 = ratio * ratio;
    const double r3 = r2 * ratio;
    const double r4 = r3 * ratio;
    const double r5 = r4 * ratio;
    const double pos_term = 10.0 * r3 - 15.0 * r4 + 6.0 * r5;
    return start + (target - start) * pos_term;
}

} // namespace

int smooth_motor_move_deg(
    int channel,
    int motor_count,
    uint32_t *can_idlist,
    const double *target_angles_deg,
    double speed_deg_per_s,
    int dt_ms)
{
    if (motor_count <= 0 || can_idlist == nullptr || target_angles_deg == nullptr)
        return -1;
    if (speed_deg_per_s <= 1e-6)
        speed_deg_per_s = 10.0;
    if (dt_ms <= 0)
        dt_ms = 20;

    std::vector<int> current_cnt(static_cast<size_t>(motor_count), 0);
    if (socketcan_sendsimplecommand(channel, motor_count, can_idlist, 8, current_cnt.data()) != 1)
        return -1;

    std::vector<double> start_deg(static_cast<size_t>(motor_count));
    std::vector<double> target_deg(static_cast<size_t>(motor_count));
    std::vector<int> pos_data(static_cast<size_t>(motor_count));

    double max_delta_deg = 0.0;
    for (int i = 0; i < motor_count; ++i)
    {
        start_deg[static_cast<size_t>(i)] = motor_cnt_to_angle_deg(current_cnt[static_cast<size_t>(i)]);
        target_deg[static_cast<size_t>(i)] = target_angles_deg[i];
        max_delta_deg = std::max(
            max_delta_deg,
            std::abs(target_deg[static_cast<size_t>(i)] - start_deg[static_cast<size_t>(i)]));
    }

    if (max_delta_deg < 0.01)
    {
        for (int i = 0; i < motor_count; ++i)
            pos_data[static_cast<size_t>(i)] = motor_angle_deg_to_cnt(target_deg[static_cast<size_t>(i)]);
        socketcan_sendcommand(channel, motor_count, can_idlist, 30, pos_data.data());
        return 0;
    }

    const double total_time_s = max_delta_deg / speed_deg_per_s;
    const double dt_s = static_cast<double>(dt_ms) / 1000.0;

    double t = 0.0;
    while (t < total_time_s)
    {
        for (int i = 0; i < motor_count; ++i)
        {
            const double angle_deg = quintic_scalar(
                t,
                total_time_s,
                start_deg[static_cast<size_t>(i)],
                target_deg[static_cast<size_t>(i)]);
            pos_data[static_cast<size_t>(i)] = motor_angle_deg_to_cnt(angle_deg);
        }
        socketcan_sendcommand(channel, motor_count, can_idlist, 30, pos_data.data());
        usleep(static_cast<useconds_t>(dt_ms * 1000));
        t += dt_s;
    }

    for (int i = 0; i < motor_count; ++i)
        pos_data[static_cast<size_t>(i)] = motor_angle_deg_to_cnt(target_deg[static_cast<size_t>(i)]);
    socketcan_sendcommand(channel, motor_count, can_idlist, 30, pos_data.data());
    return 0;
}


void motor_speed_change()
{
    int data[7] = {};

    // set_motor_position(data, 0);
    // set_motor_position(data, 1);

    int speed_data[7] = {1, 1, 1, 1, 1, 1, 1};

    get_motor_speed(speed_data, 1);

    for (int i = 0; i < 7; i++)
    {
        cout << " " << speed_data[i];
    }
    cout << endl;

    get_motor_speed(speed_data, 0);

    for (int i = 0; i < 7; i++)
    {
        cout << " " << speed_data[i];
    }
    cout << endl;

    speed_data[0] = -1;

    get_motor_speed(speed_data, 1);

    for (int i = 0; i < 7; i++)
    {
        cout << " " << speed_data[i];
    }
    cout << endl;

    get_motor_speed(speed_data, 0);

    for (int i = 0; i < 7; i++)
    {
        cout << " " << speed_data[i];
    }
    cout << endl;

    int speed[7] = {9000, 9000, 9000, 9000, 9000, 9000, 9000};
    int speed2[7] = {-9000, -9000, -9000, -9000, -9000, -9000, -9000};

    set_motor_speed(speed, 1);
    set_motor_speed(speed2, 1);
    set_motor_speed(speed, 0);
    set_motor_speed(speed2, 0);

    speed_data[0] = 1;
    get_motor_speed(speed_data, 1);

    for (int i = 0; i < 7; i++)
    {
        cout << " " << speed_data[i];
    }
    cout << endl;

    get_motor_speed(speed_data, 0);

    for (int i = 0; i < 7; i++)
    {
        cout << " " << speed_data[i];
    }
    cout << endl;

    speed_data[0] = -1;

    get_motor_speed(speed_data, 1);

    for (int i = 0; i < 7; i++)
    {
        cout << " " << speed_data[i];
    }
    cout << endl;

    get_motor_speed(speed_data, 0);

    for (int i = 0; i < 7; i++)
    {
        cout << " " << speed_data[i];
    }
    cout << endl;

    sleep(1);

    std::cout << "------------------------------" << std::endl;
}



void rev_motor_error( int a)
{
    int data[7];

    uint8_t speed_m = 11;
   
    if (a == 1)
    {
        // 定义CAN ID列表
        uint32_t can_idlist_h[2] = {16, 17};
        uint32_t can_idlist_m[2] = {18, 19};
        uint32_t can_idlist_l[3] = {20, 21, 22};

        // 创建线程容器
        std::vector<std::thread> threads;

        // 线程1：高位指令
        threads.emplace_back([&]()
                             { socketcan_sendsimplecommand(right_h_id, 2, can_idlist_h, speed_m, data); });

        // 线程2：中位指令
        threads.emplace_back([&]()
                             { socketcan_sendsimplecommand(right_m_id, 2, can_idlist_m, speed_m, data + 2); });

        // 线程3：低位指令
        threads.emplace_back([&]()

                             { socketcan_sendsimplecommand(right_l_id, 3, can_idlist_l, speed_m, data + 4); });

        // socketcan_sendsimplecommand(right_h_id, 2, can_idlist_h, 8, data);

        // socketcan_sendsimplecommand(right_m_id, 2, can_idlist_m, 8, data + 2);

        // // 线程3：低位指令
        // socketcan_sendsimplecommand(right_l_id, 3, can_idlist_l, 8, data + 4);

        // 等待所有线程完成
        for (auto &t : threads)
        {
            if (t.joinable())
            {
                t.join();
            }
        }
    }
    else
    {
        uint32_t can_idlist_h[2] = {23, 24};
        uint32_t can_idlist_m[2] = {25, 26};
        uint32_t can_idlist_l[3] = {27, 28, 29};

        std::vector<std::thread> threads;

        threads.emplace_back([&]()
                             { socketcan_sendsimplecommand(left_h_id, 2, can_idlist_h, speed_m, data); });

        threads.emplace_back([&]()
                             { socketcan_sendsimplecommand(left_m_id, 2, can_idlist_m, speed_m, data + 2); });

        threads.emplace_back([&]()
                             { socketcan_sendsimplecommand(left_l_id, 3, can_idlist_l, speed_m, data + 4); });

        for (auto &t : threads)
        {
            if (t.joinable())
            {
                t.join();
            }
        }
    }
}



// =============================================================================
// END: src/Ti5_socketcan.cpp
// =============================================================================

// =============================================================================
// BEGIN: src/wt_client.cpp
// =============================================================================
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

// =============================================================================
// END: src/wt_client.cpp
// =============================================================================

// =============================================================================
// BEGIN: src/wt_box_tcp.cpp
// =============================================================================
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









// =============================================================================
// END: src/wt_box_tcp.cpp
// =============================================================================

// =============================================================================
// BEGIN: src/lanxincontrol.cpp
// =============================================================================
#include "lanxincontrol.h"

#include <chrono>
#include <ctime>
#include <exception>
#include <iostream>
#include <thread>

#include "client/common/client_common.h"
#include "robot_task.h"

LanxinControl* LanxinControl::g_instance_ = nullptr;

LanxinControl::LanxinControl() { g_instance_ = this; }

LanxinControl::~LanxinControl() { Disconnect(); }

bool LanxinControl::Connect(const std::string& ip, int port) {
  ip_ = ip;
  port_ = port;
  int ret = VMR_initWithServer(ip_.c_str(), port_);
  if (ret != 0) {
    std::cerr << "VMR_initWithServer failed, ret=" << ret << std::endl;
    return false;
  }

  vmr_handle_ = VMR_Handle_Create();
  if (!vmr_handle_) {
    std::cerr << "VMR_Handle_Create failed" << std::endl;
    return false;
  }

  VMR_registerBatteryCallback(static_cast<int>(vmr_handle_),
                              BatteryCallbackBridge);
  std::cout << "Connected to " << ip_ << ":" << port_ << " (VMR), apiv2 RPC port: "
            << rpc_port_ << std::endl;
  return true;
}

void LanxinControl::SetRpcPort(int port) {
  if (port <= 0) {
    return;
  }
  if (rpc_port_ != port) {
    rpc_ready_ = false;
    robot_task_handle_ = -1;
  }
  rpc_port_ = port;
}

int LanxinControl::GetRpcPort() const { return rpc_port_; }

bool LanxinControl::OpenRpcClient() {
  if (ip_.empty()) {
    std::cerr << "OpenRpcClient: ip is empty" << std::endl;
    return false;
  }
  try {
    rpc_client::ClientConfig conf;
    conf.ip = ip_;
    conf.port = rpc_port_;
    conf.connect_timeout_ms = 3000;
    conf.call_timeout_ms = 60000;
    conf.error_callback = [](const std::error_code& ec,
                             const std::string& message) {
      std::cerr << "RPC error: " << ec.message() << ", " << message
                << std::endl;
    };
    if (!rpc_client::InitClient(conf)) {
      std::cerr << "InitClient failed" << std::endl;
      rpc_ready_ = false;
      robot_task_handle_ = -1;
      return false;
    }
    auto handles = robot_task::GetAllHandleRobotTaskDev();
    if (handles.empty()) {
      std::cerr << "no robot_task handle" << std::endl;
      rpc_ready_ = false;
      robot_task_handle_ = -1;
      return false;
    }
    robot_task_handle_ = handles[0];
    rpc_ready_ = true;
    std::cout << "RPC connected " << ip_ << ":" << rpc_port_
              << " robot_task handle=" << robot_task_handle_ << std::endl;
    return true;
  } catch (const std::exception& e) {
    std::cerr << "OpenRpcClient 异常: " << e.what() << std::endl;
    rpc_ready_ = false;
    robot_task_handle_ = -1;
    return false;
  }
}

bool LanxinControl::SetTopoEndpoint(const std::string& ip, int rpc_port) {
  if (ip.empty()) {
    return false;
  }
  ip_ = ip;
  SetRpcPort(rpc_port);
  return OpenRpcClient();
}

void LanxinControl::Disconnect() {
  if (vmr_handle_) {
    VMR_Handle_Destroy(vmr_handle_);
    vmr_handle_ = 0;
  }
  rpc_ready_ = false;
  robot_task_handle_ = -1;
  has_last_charge_task_ = false;
  has_last_topo_task_ = false;
}

bool LanxinControl::IsConnected() const { return vmr_handle_ != 0; }

bool LanxinControl::WaitTaskDone(const std::string& task_id) const {
  while (true) {
    VmrTaskResult r = VMR_checkTaskStatus(vmr_handle_, task_id.c_str());
    if (r.task_result == -1) {
      std::cout << "task running..." << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
      continue;
    }
    std::cout << "task_result=" << r.task_result << std::endl;
    return r.task_result == 0;
  }
}

bool LanxinControl::MoveToPose(double x, double y, double theta) {
  VmrPose pose;
  pose.x = x;
  pose.y = y;
  pose.theta = theta;
  std::string task_id = VMR_moveTasks(vmr_handle_, pose);
  if (task_id.empty()) {
    std::cerr << "VMR_moveTasks failed" << std::endl;
    return false;
  }
  std::cout << "move task_id=" << task_id << std::endl;
  return WaitTaskDone(task_id);
}

bool LanxinControl::MoveWithNavTask(double x, double y, double theta,
                                    int move_mode, int navi_mode,
                                    bool forbid_rotation_on_start) {
  VmrNavTask task;
  task.target_pose.x = x;
  task.target_pose.y = y;
  task.target_pose.theta = theta;
  task.move_mode = static_cast<VmrMoveMode>(move_mode);
  task.navi_mode = static_cast<VmrNaviMode>(navi_mode);
  task.forbid_rotation_on_start = forbid_rotation_on_start;

  std::string task_id = VMR_moveTasks(vmr_handle_, task);
  if (task_id.empty()) {
    std::cerr << "VMR_moveTasks(VmrNavTask) failed" << std::endl;
    return false;
  }
  std::cout << "nav task_id=" << task_id << std::endl;
  return WaitTaskDone(task_id);
}

bool LanxinControl::MoveForwardOrBackward(double distance, bool forward) {
  float angle = forward ? 0.0f : 180.0f;
  std::string task_id =
      VMR_moveRelative(vmr_handle_, static_cast<float>(distance), angle);
  if (task_id.empty()) {
    std::cerr << "VMR_moveRelative failed" << std::endl;
    return false;
  }
  std::cout << (forward ? "forward" : "backward") << " task_id=" << task_id
            << std::endl;
  return WaitTaskDone(task_id);
}

bool LanxinControl::RotateInPlace(double deg) {
  std::string task_id = VMR_rotateInPlace(vmr_handle_, static_cast<float>(deg));
  if (task_id.empty()) {
    std::cerr << "VMR_rotateInPlace failed" << std::endl;
    return false;
  }
  std::cout << "rotate task_id=" << task_id << std::endl;
  return WaitTaskDone(task_id);
}

bool LanxinControl::MoveTopo(const std::string& pose_name, int32_t action) {
  if (!rpc_ready_) {
    std::cerr << "MoveTopo: call SetTopoEndpoint first (RPC not connected)"
              << std::endl;
    return false;
  }
  try {
    robot_task::TopoPose pose;
    pose.pose_name = pose_name;
    pose.action = action;
    auto future = robot_task::topo_move_task(robot_task_handle_, pose);
    last_topo_task_id_ = future.id;
    has_last_topo_task_ = true;
    std::cout << "cmd task id: " << future.id << std::endl;

    auto status = future.wait_for(std::chrono::seconds(36));
    if (status == std::future_status::ready) {
      auto result = future.get();
      std::cout << "cmd result " << result.result << std::endl;
      return result.result == 1;
    }
    std::cout << "cmd timeout" << std::endl;
   // robot_task::cancel_topo_move_task(robot_task_handle_, future.id);
    return false;
  } catch (const std::exception& e) {
    std::cerr << "MoveTopo(\"" << pose_name << "\") 异常: " << e.what()
              << std::endl;
    return false;
  }
}

bool LanxinControl::CancelTopoTask() {
  if (!rpc_ready_) {
    std::cerr << "CancelTopoTask: RPC not connected" << std::endl;
    return false;
  }
  if (!has_last_topo_task_) {
    std::cerr << "no cached topo task id, run MoveTopo first" << std::endl;
    return false;
  }
  robot_task::cancel_topo_move_task(robot_task_handle_, last_topo_task_id_);
  std::cout << "cancel_topo_move_task(" << last_topo_task_id_ << ") sent"
            << std::endl;
  return true;
}

bool LanxinControl::EnsureRpcReady() {
  if (rpc_ready_) {
    return true;
  }
  return OpenRpcClient();
}

bool LanxinControl::StartChargeTask(double x, double y, double theta) {
  if (!EnsureRpcReady()) {
    return false;
  }
  try {
    robot_task::RobotPose pose{x, y, theta};
    auto future = robot_task::charge_task(robot_task_handle_, pose);
    last_charge_task_id_ = future.id;
    has_last_charge_task_ = true;
    std::cout << "charge task id=" << future.id << std::endl;

    auto status = future.wait_for(std::chrono::seconds(120));
    if (status == std::future_status::ready) {
      auto result = future.get();
      std::cout << "charge result=" << result.result << std::endl;
      return result.result == 0;
    }
    std::cout << "charge timeout, canceling..." << std::endl;
    robot_task::cancel_charge_task(robot_task_handle_, future.id);
    return false;
  } catch (const std::exception& e) {
    std::cerr << "StartChargeTask 异常: " << e.what() << std::endl;
    return false;
  }
}

bool LanxinControl::CancelChargeTask() {
  if (!EnsureRpcReady()) {
    return false;
  }
  if (!has_last_charge_task_) {
    std::cerr << "no cached charge task id, run StartChargeTask first"
              << std::endl;
    return false;
  }
  robot_task::cancel_charge_task(robot_task_handle_, last_charge_task_id_);
  std::cout << "cancel_charge_task(" << last_charge_task_id_ << ") sent"
            << std::endl;
  return true;
}

bool LanxinControl::ChargeRelay(bool on) {
  // 新版 VMR_AMR_SDK 中已无 robot_task::charge_relay 接口，
  // 这里暂时仅给出占位实现，避免编译错误，实际控制方式需根据新版 SDK 文档更新。
  std::cerr << "ChargeRelay is not supported with current VMR_AMR_SDK "
               "headers. Please update implementation if SDK provides "
               "a new API."
            << std::endl;
  return false;
}

bool LanxinControl::SetSpeedFactor(double factor) {
  int ret = VMR_setSpeedFactor(vmr_handle_, factor);
  std::cout << "VMR_setSpeedFactor(" << factor << ") ret=" << ret << std::endl;
  return ret == 0;
}

bool LanxinControl::EnableSdkCtrlSpeed(bool enable) {
  std::string task_id = VMR_enableSdkCtrlSpeed(vmr_handle_, enable);
  std::cout << "VMR_enableSdkCtrlSpeed(" << (enable ? "true" : "false")
            << ") -> \"" << task_id << "\"" << std::endl;
  return !task_id.empty();
}

bool LanxinControl::SendTwist(double vx, double vy, double wz) {
  VmrTwistInfo tw{};
  tw.linear.x = vx;
  tw.linear.y = vy;
  tw.angular.z = wz;
  VMR_setRobotTwist(vmr_handle_, tw);
  return true;
}

bool LanxinControl::StopTwist() { return SendTwist(0.0, 0.0, 0.0); }

void LanxinControl::BatteryCallbackBridge(const VmrBatteryInfo& info) {
  if (g_instance_) {
    g_instance_->OnBattery(info);
  }
}

void LanxinControl::OnBattery(const VmrBatteryInfo& info) {
  battery_ = info;
  has_battery_ = true;
}

bool LanxinControl::GetBattery(VmrBatteryInfo& out_battery) const {
  if (!has_battery_) {
    return false;
  }
  out_battery = battery_;
  return true;
}

void LanxinControl::PrintBattery() const {
  VmrBatteryInfo b;
  if (!GetBattery(b)) {
    std::cout << "no battery callback yet, wait 1-2s and retry" << std::endl;
    return;
  }
  std::cout << "battery: " << b.percentage << "%, V=" << b.voltage
            << ", I=" << b.current << ", status="
            << static_cast<int>(b.power_supply_status)
            << " (0 unknown,1 charging,2 discharging)" << std::endl;
}


int lanxin_point_move(LanxinControl &ctrl,double *point)
{

  ctrl.MoveToPose(point[0], point[1], point[2]);

  return 0;
}

// =============================================================================
// END: src/lanxincontrol.cpp
// =============================================================================

// =============================================================================
// BEGIN: src/gripper_interface.cpp
// =============================================================================
#include "gripper_interface.hpp"

#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <unistd.h>

namespace gripper {
namespace {

constexpr int kRightMotorId = 0;
constexpr int kLeftMotorId = 1;

double clamp(double v, double lo, double hi) {
  return std::clamp(v, std::min(lo, hi), std::max(lo, hi));
}

Side sideFromMotorId(int id) {
  return id == kRightMotorId ? Side::Right : Side::Left;
}

double angleToRad(double angle_deg, const Config &cfg) {
  const double ratio =
      clamp(angle_deg / std::max(1e-6, cfg.full_close_angle_deg), 0.0, cfg.max_close_ratio);
  return cfg.open_position_rad + ratio * (cfg.close_position_rad - cfg.open_position_rad);
}

double radToAngle(double q, const Config &cfg) {
  const double denom = cfg.close_position_rad - cfg.open_position_rad;
  if (std::fabs(denom) < 1e-6) {
    return 0.0;
  }
  const double ratio = clamp((q - cfg.open_position_rad) / denom, 0.0, 1.0);
  return ratio * cfg.full_close_angle_deg;
}

bool probeMotor(const std::shared_ptr<SerialPort> &serial, int motor_id) {
  MotorCmd cmd;
  MotorData data;
  cmd.motorType = MotorType::M4010;
  cmd.id = motor_id;
  cmd.mode = queryMotorMode(cmd.motorType, MotorMode::FOC);
  data.motorType = cmd.motorType;
  usleep(200);
  return serial->sendRecv(&cmd, &data);
}

}  // namespace

struct Gripper::SideImpl {
  Side side = Side::Left;
  int motor_id = 0;
  std::string port;
  std::shared_ptr<SerialPort> serial;
  float gear_ratio = 1.0f;

  double target_q = 0.0;
  double q_cmd = 0.0;
  double q = 0.0;
  double dq = 0.0;
  double tau = 0.0;
  bool initialized = false;

  double teleop_ratio = 0.0;
  double effective_target_q = 0.0;
  bool torque_limited = false;

  // ros2_170D 软力矩路径（Soft* / setTeleopSoftRatio）
  bool soft_torque_active = false;
  bool soft_torque_limited = false;
  double q_anchor = 0.0;
  double dq_filt = 0.0;
  bool cmd_initialized = false;
};

namespace {

bool readMotorPosition(const std::shared_ptr<SerialPort> &serial, int motor_id,
                       float gear_ratio, double *q_out) {
  if (!serial || !q_out) {
    return false;
  }
  MotorCmd cmd;
  MotorData data;
  cmd.motorType = MotorType::M4010;
  cmd.id = motor_id;
  cmd.mode = queryMotorMode(cmd.motorType, MotorMode::FOC);
  data.motorType = cmd.motorType;
  if (!serial->sendRecv(&cmd, &data)) {
    return false;
  }
  *q_out = data.q / gear_ratio;
  return true;
}

}  // namespace

std::vector<std::string> scanSerialPorts() {
  std::vector<std::string> ports;
  constexpr const char *kPrefixes[] = {"/dev/ttyUSB", "/dev/ttyCH343USB"};

  if (!std::filesystem::exists("/dev")) {
    return ports;
  }

  for (const auto &entry : std::filesystem::directory_iterator("/dev")) {
    const std::string path = entry.path().string();
    for (const char *prefix : kPrefixes) {
      if (path.rfind(prefix, 0) == 0) {
        ports.push_back(path);
        break;
      }
    }
  }
  std::sort(ports.begin(), ports.end());
  return ports;
}

Gripper::Gripper(const Config &config) : config_(config) {}

Gripper::~Gripper() { stop(); }

bool Gripper::isConnected() const { return connected_.load(); }

std::vector<DetectedMotor> Gripper::detectedMotors() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<DetectedMotor> out;
  for (const auto &[side, impl] : sides_) {
    DetectedMotor m;
    m.side = side;
    m.motor_id = impl->motor_id;
    m.port = impl->port;
    out.push_back(m);
  }
  return out;
}

bool Gripper::connect() {
  if (connected_.load()) {
    return true;
  }

  const auto ports = scanSerialPorts();
  if (ports.empty()) {
    std::cerr << "[gripper] 未找到串口 (/dev/ttyUSB* 或 /dev/ttyCH343USB*)\n";
    return false;
  }

  std::cout << "[gripper] 扫描串口: ";
  for (const auto &p : ports) {
    std::cout << p << " ";
  }
  std::cout << "\n";

  std::map<int, std::pair<std::shared_ptr<SerialPort>, std::string>> found;

  for (int attempt = 0; attempt < config_.detect_retries && found.size() < 2; ++attempt) {
    for (const auto &port : ports) {
      auto serial = std::make_shared<SerialPort>(port.c_str());
      for (int id : {kRightMotorId, kLeftMotorId}) {
        if (found.count(id) > 0) {
          continue;
        }
        if (probeMotor(serial, id)) {
          found[id] = {serial, port};
          std::cout << "[gripper] 检测到 motor_id=" << id << " side="
                    << (id == kRightMotorId ? "right" : "left") << " port=" << port << "\n";
        }
      }
    }
    if (found.size() < 2) {
      usleep(50000);
    }
  }

  if (found.empty()) {
    std::cerr << "[gripper] 未检测到夹爪电机，请检查 USB 与电源\n";
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  sides_.clear();
  for (const auto &[id, info] : found) {
    const Side side = sideFromMotorId(id);
    auto impl = std::make_unique<SideImpl>();
    impl->side = side;
    impl->motor_id = id;
    impl->port = info.second;
    impl->serial = info.first;
    impl->gear_ratio = queryGearRatio(MotorType::M4010);
    double current_q = config_.open_position_rad;
    if (readMotorPosition(impl->serial, impl->motor_id, impl->gear_ratio, &current_q)) {
      impl->q = current_q;
      impl->initialized = true;
      std::cout << "[gripper] motor_id=" << id << " 当前位置 q=" << current_q << " rad\n";
    }
    impl->target_q = current_q;
    impl->q_cmd = current_q;
    impl->q_anchor = current_q;
    impl->dq_filt = 0.0;
    sides_[side] = std::move(impl);
  }

  connected_ = true;
  return true;
}

bool Gripper::start() {
  if (running_.load()) {
    return true;
  }
  if (!connected_.load() && !connect()) {
    return false;
  }

  stop_requested_ = false;
  running_ = true;
  control_thread_ = std::thread(&Gripper::controlLoop, this);
  return true;
}

void Gripper::stop() {
  if (running_.load()) {
    stop_requested_ = true;
    if (control_thread_.joinable()) {
      control_thread_.join();
    }
    running_ = false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  sides_.clear();
  connected_ = false;
}

void Gripper::setTargetRad(Side side, double target_rad) {
  std::lock_guard<std::mutex> lock(mutex_);
  teleop_mode_ = false;
  teleop_soft_mode_ = false;
  const auto it = sides_.find(side);
  if (it == sides_.end()) {
    return;
  }
  const double q =
      clamp(target_rad, config_.open_position_rad, config_.close_position_rad);
  it->second->soft_torque_active = false;
  it->second->soft_torque_limited = false;
  it->second->torque_limited = false;
  it->second->target_q = q;
  it->second->q_cmd = q;
  if (it->second->initialized) {
    it->second->q_anchor = it->second->q;
    it->second->dq_filt = it->second->dq;
  }
}

void Gripper::setSoftTargetRad(Side side, double target_rad) {
  std::lock_guard<std::mutex> lock(mutex_);
  teleop_mode_ = false;
  teleop_soft_mode_ = false;
  const auto it = sides_.find(side);
  if (it == sides_.end()) {
    return;
  }
  const double q =
      clamp(target_rad, config_.open_position_rad, config_.close_position_rad);
  it->second->soft_torque_active = true;
  it->second->soft_torque_limited = false;
  it->second->target_q = q;
  syncSoftMotionState(*it->second);
}

void Gripper::setAngle(Side side, double angle_deg) {
  setTargetRad(side, angleToRad(clamp(angle_deg, 0.0, config_.full_close_angle_deg), config_));
}

void Gripper::setBothAngles(double left_deg, double right_deg) {
  setAngle(Side::Left, left_deg);
  setAngle(Side::Right, right_deg);
}

void Gripper::open(Side side) { setTargetRad(side, config_.open_position_rad); }
void Gripper::openBoth() {
  setTargetRad(Side::Left, config_.open_position_rad);
  setTargetRad(Side::Right, config_.open_position_rad);
}

void Gripper::close(Side side, double ratio) {
  const double r = clamp(ratio, 0.0, config_.max_close_ratio);
  const double q =
      config_.open_position_rad + r * (config_.close_position_rad - config_.open_position_rad);
  setTargetRad(side, q);
}

void Gripper::closeBoth(double ratio) {
  const double r = clamp(ratio, 0.0, config_.max_close_ratio);
  const double q =
      config_.open_position_rad + r * (config_.close_position_rad - config_.open_position_rad);
  setTargetRad(Side::Left, q);
  setTargetRad(Side::Right, q);
}

bool Gripper::waitBothRad(double target_rad, double tolerance_rad,
                          std::chrono::milliseconds timeout) {
  bool wait_left = false;
  bool wait_right = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    wait_left = sides_.count(Side::Left) > 0;
    wait_right = sides_.count(Side::Right) > 0;
  }
  bool ok = true;
  if (wait_left) {
    ok &= waitForRad(Side::Left, target_rad, tolerance_rad, timeout);
  }
  if (wait_right) {
    ok &= waitForRad(Side::Right, target_rad, tolerance_rad, timeout);
  }
  return ok;
}

bool Gripper::openBothAndWait(double tolerance_rad, std::chrono::milliseconds timeout) {
  openBoth();
  return waitBothRad(config_.open_position_rad, tolerance_rad, timeout);
}

bool Gripper::closeBothAndWait(double ratio, double tolerance_rad,
                               std::chrono::milliseconds timeout) {
  closeBoth(ratio);
  const double r = clamp(ratio, 0.0, config_.max_close_ratio);
  const double q =
      config_.open_position_rad + r * (config_.close_position_rad - config_.open_position_rad);
  return waitBothRad(q, tolerance_rad, timeout);
}

bool Gripper::openAndWait(Side side, double tolerance_rad, std::chrono::milliseconds timeout) {
  open(side);
  return waitForRad(side, config_.open_position_rad, tolerance_rad, timeout);
}

bool Gripper::closeAndWait(Side side, double ratio, double tolerance_rad,
                           std::chrono::milliseconds timeout) {
  close(side, ratio);
  const double r = clamp(ratio, 0.0, config_.max_close_ratio);
  const double q =
      config_.open_position_rad + r * (config_.close_position_rad - config_.open_position_rad);
  return waitForRad(side, q, tolerance_rad, timeout);
}

void Gripper::openSoft(Side side) {
  setSoftTargetRad(side, config_.open_position_rad);
}

void Gripper::openSoftBoth() {
  setSoftTargetRad(Side::Left, config_.open_position_rad);
  setSoftTargetRad(Side::Right, config_.open_position_rad);
}

void Gripper::closeSoft(Side side, double ratio) {
  const double r = clamp(ratio, 0.0, config_.max_close_ratio);
  const double q =
      config_.open_position_rad + r * (config_.close_position_rad - config_.open_position_rad);
  setSoftTargetRad(side, q);
}

void Gripper::closeSoftBoth(double ratio) {
  const double r = clamp(ratio, 0.0, config_.max_close_ratio);
  const double q =
      config_.open_position_rad + r * (config_.close_position_rad - config_.open_position_rad);
  setSoftTargetRad(Side::Left, q);
  setSoftTargetRad(Side::Right, q);
}

bool Gripper::openSoftAndWait(Side side, double tolerance_rad,
                              std::chrono::milliseconds timeout) {
  openSoft(side);
  return waitForRad(side, config_.open_position_rad, tolerance_rad, timeout);
}

bool Gripper::openSoftBothAndWait(double tolerance_rad, std::chrono::milliseconds timeout) {
  openSoftBoth();
  return waitBothRad(config_.open_position_rad, tolerance_rad, timeout);
}

bool Gripper::closeSoftAndWait(Side side, double ratio, double tolerance_rad,
                               std::chrono::milliseconds timeout) {
  closeSoft(side, ratio);
  const double r = clamp(ratio, 0.0, config_.max_close_ratio);
  const double q =
      config_.open_position_rad + r * (config_.close_position_rad - config_.open_position_rad);
  return waitForRad(side, q, tolerance_rad, timeout);
}

bool Gripper::closeSoftBothAndWait(double ratio, double tolerance_rad,
                                   std::chrono::milliseconds timeout) {
  closeSoftBoth(ratio);
  const double r = clamp(ratio, 0.0, config_.max_close_ratio);
  const double q =
      config_.open_position_rad + r * (config_.close_position_rad - config_.open_position_rad);
  return waitBothRad(q, tolerance_rad, timeout);
}

double Gripper::resolveGraspTorque(double max_torque_nm) const {
  return max_torque_nm > 0.0 ? max_torque_nm : config_.grasp_torque_limit_nm;
}

double Gripper::resolveSoftTorque(double max_torque_nm) const {
  return max_torque_nm > 0.0 ? max_torque_nm : config_.soft_torque_limit_nm;
}

GraspFeedback Gripper::graspSideUntil(Side side, double max_torque_nm,
                                      double position_tolerance_rad,
                                      std::chrono::milliseconds timeout) {
  GraspFeedback out;
  const double torque_limit = resolveGraspTorque(max_torque_nm);
  const double close_q = config_.close_position_rad;
  const double pos_tol = std::max(0.0, position_tolerance_rad);

  close(side, 1.0);

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto fb = feedback(side);
    if (!fb.have_feedback) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    out.position_rad = fb.position_rad;
    out.torque = fb.torque;

    if (torque_limit > 0.0 && std::fabs(fb.torque) >= torque_limit) {
      setTargetRad(side, fb.position_rad);
      out.result = GraspResult::TorqueLimit;
      return out;
    }
    if (std::fabs(fb.position_rad - close_q) <= pos_tol) {
      out.result = GraspResult::PositionReached;
      return out;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  const auto fb = feedback(side);
  out.position_rad = fb.position_rad;
  out.torque = fb.torque;
  out.result = GraspResult::Timeout;
  return out;
}

GraspFeedback Gripper::graspAndWait(Side side, double max_torque_nm,
                                    std::chrono::milliseconds timeout) {
  return graspSideUntil(side, max_torque_nm, 0.2, timeout);
}

GraspFeedback Gripper::graspSoftSideUntil(Side side, double max_torque_nm,
                                          double position_tolerance_rad,
                                          std::chrono::milliseconds timeout) {
  GraspFeedback out;
  const double tau_limit = resolveSoftTorque(max_torque_nm);
  const double close_q = config_.close_position_rad;
  const double pos_tol = std::max(0.0, position_tolerance_rad);
  const double tau_contact = tau_limit > 0.0 ? tau_limit * 0.75 : 0.0;
  constexpr double kVelTol = 0.35;
  constexpr int kContactSamples = 15;

  closeSoft(side, 1.0);

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  int contact_samples = 0;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto fb = feedback(side);
    if (!fb.have_feedback) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    out.position_rad = fb.position_rad;
    out.torque = fb.torque;

    if (std::fabs(fb.position_rad - close_q) <= pos_tol) {
      out.result = GraspResult::PositionReached;
      return out;
    }

    if (tau_contact > 0.0 && fb.soft_torque_limited &&
        std::fabs(fb.torque) >= tau_contact && std::fabs(fb.velocity_rad_s) < kVelTol &&
        fb.position_rad > close_q + pos_tol) {
      ++contact_samples;
      if (contact_samples >= kContactSamples) {
        setSoftTargetRad(side, fb.position_rad);
        out.result = GraspResult::TorqueLimit;
        return out;
      }
    } else {
      contact_samples = 0;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  const auto fb = feedback(side);
  out.position_rad = fb.position_rad;
  out.torque = fb.torque;
  out.result = GraspResult::Timeout;
  return out;
}

GraspFeedback Gripper::graspSoftAndWait(Side side, double max_torque_nm,
                                        std::chrono::milliseconds timeout) {
  return graspSoftSideUntil(side, max_torque_nm, 0.2, timeout);
}

GraspFeedback Gripper::graspSoftBothAndWait(double max_torque_nm,
                                            std::chrono::milliseconds timeout) {
  GraspFeedback out;
  closeSoftBoth(1.0);

  const double tau_limit = resolveSoftTorque(max_torque_nm);
  const double close_q = config_.close_position_rad;
  constexpr double kPosTol = 0.2;
  const double tau_contact = tau_limit > 0.0 ? tau_limit * 0.75 : 0.0;
  constexpr double kVelTol = 0.35;
  constexpr int kContactSamples = 15;

  bool left_done = !hasSide(Side::Left);
  bool right_done = !hasSide(Side::Right);
  GraspFeedback left_fb;
  GraspFeedback right_fb;
  int left_contact = 0;
  int right_contact = 0;

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (!left_done && hasSide(Side::Left)) {
      const auto fb = feedback(Side::Left);
      if (fb.have_feedback) {
        left_fb.position_rad = fb.position_rad;
        left_fb.torque = fb.torque;
        if (std::fabs(fb.position_rad - close_q) <= kPosTol) {
          left_fb.result = GraspResult::PositionReached;
          left_done = true;
        } else if (tau_contact > 0.0 && fb.soft_torque_limited &&
                   std::fabs(fb.torque) >= tau_contact &&
                   std::fabs(fb.velocity_rad_s) < kVelTol &&
                   fb.position_rad > close_q + kPosTol) {
          ++left_contact;
          if (left_contact >= kContactSamples) {
            setSoftTargetRad(Side::Left, fb.position_rad);
            left_fb.result = GraspResult::TorqueLimit;
            left_done = true;
          }
        } else {
          left_contact = 0;
        }
      }
    }
    if (!right_done && hasSide(Side::Right)) {
      const auto fb = feedback(Side::Right);
      if (fb.have_feedback) {
        right_fb.position_rad = fb.position_rad;
        right_fb.torque = fb.torque;
        if (std::fabs(fb.position_rad - close_q) <= kPosTol) {
          right_fb.result = GraspResult::PositionReached;
          right_done = true;
        } else if (tau_contact > 0.0 && fb.soft_torque_limited &&
                   std::fabs(fb.torque) >= tau_contact &&
                   std::fabs(fb.velocity_rad_s) < kVelTol &&
                   fb.position_rad > close_q + kPosTol) {
          ++right_contact;
          if (right_contact >= kContactSamples) {
            setSoftTargetRad(Side::Right, fb.position_rad);
            right_fb.result = GraspResult::TorqueLimit;
            right_done = true;
          }
        } else {
          right_contact = 0;
        }
      }
    }
    if (left_done && right_done) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  if (!left_done) {
    left_fb.result = GraspResult::Timeout;
    const auto fb = feedback(Side::Left);
    left_fb.position_rad = fb.position_rad;
    left_fb.torque = fb.torque;
  }
  if (!right_done) {
    right_fb.result = GraspResult::Timeout;
    const auto fb = feedback(Side::Right);
    right_fb.position_rad = fb.position_rad;
    right_fb.torque = fb.torque;
  }

  if (left_fb.result == GraspResult::TorqueLimit ||
      right_fb.result == GraspResult::TorqueLimit) {
    out.result = GraspResult::TorqueLimit;
  } else if (left_fb.result == GraspResult::PositionReached &&
             right_fb.result == GraspResult::PositionReached) {
    out.result = GraspResult::PositionReached;
  } else {
    out.result = GraspResult::Timeout;
  }
  out.position_rad = (left_fb.position_rad + right_fb.position_rad) * 0.5;
  out.torque = std::max(std::fabs(left_fb.torque), std::fabs(right_fb.torque));
  return out;
}

bool Gripper::isSoftTorqueActive(Side side) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = sides_.find(side);
  if (it == sides_.end()) {
    return false;
  }
  return it->second->soft_torque_active;
}

GraspFeedback Gripper::graspBothAndWait(double max_torque_nm,
                                        std::chrono::milliseconds timeout) {
  GraspFeedback out;
  closeBoth(1.0);

  const double torque_limit = resolveGraspTorque(max_torque_nm);
  const double close_q = config_.close_position_rad;
  constexpr double kPosTol = 0.2;

  bool left_done = !hasSide(Side::Left);
  bool right_done = !hasSide(Side::Right);
  GraspFeedback left_fb;
  GraspFeedback right_fb;

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (!left_done && hasSide(Side::Left)) {
      const auto fb = feedback(Side::Left);
      if (fb.have_feedback) {
        left_fb.position_rad = fb.position_rad;
        left_fb.torque = fb.torque;
        if (torque_limit > 0.0 && std::fabs(fb.torque) >= torque_limit) {
          setTargetRad(Side::Left, fb.position_rad);
          left_fb.result = GraspResult::TorqueLimit;
          left_done = true;
        } else if (std::fabs(fb.position_rad - close_q) <= kPosTol) {
          left_fb.result = GraspResult::PositionReached;
          left_done = true;
        }
      }
    }
    if (!right_done && hasSide(Side::Right)) {
      const auto fb = feedback(Side::Right);
      if (fb.have_feedback) {
        right_fb.position_rad = fb.position_rad;
        right_fb.torque = fb.torque;
        if (torque_limit > 0.0 && std::fabs(fb.torque) >= torque_limit) {
          setTargetRad(Side::Right, fb.position_rad);
          right_fb.result = GraspResult::TorqueLimit;
          right_done = true;
        } else if (std::fabs(fb.position_rad - close_q) <= kPosTol) {
          right_fb.result = GraspResult::PositionReached;
          right_done = true;
        }
      }
    }
    if (left_done && right_done) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  if (!left_done) {
    left_fb.result = GraspResult::Timeout;
    const auto fb = feedback(Side::Left);
    left_fb.position_rad = fb.position_rad;
    left_fb.torque = fb.torque;
  }
  if (!right_done) {
    right_fb.result = GraspResult::Timeout;
    const auto fb = feedback(Side::Right);
    right_fb.position_rad = fb.position_rad;
    right_fb.torque = fb.torque;
  }

  if (left_fb.result == GraspResult::TorqueLimit ||
      right_fb.result == GraspResult::TorqueLimit) {
    out.result = GraspResult::TorqueLimit;
  } else if (left_fb.result == GraspResult::PositionReached &&
             right_fb.result == GraspResult::PositionReached) {
    out.result = GraspResult::PositionReached;
  } else {
    out.result = GraspResult::Timeout;
  }
  out.position_rad = (left_fb.position_rad + right_fb.position_rad) * 0.5;
  out.torque = std::max(std::fabs(left_fb.torque), std::fabs(right_fb.torque));
  return out;
}

bool Gripper::hasSide(Side side) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return sides_.count(side) > 0;
}

SideFeedback Gripper::feedback(Side side) const {
  std::lock_guard<std::mutex> lock(mutex_);
  SideFeedback fb;
  const auto it = sides_.find(side);
  if (it == sides_.end()) {
    return fb;
  }
  const auto &s = *it->second;
  fb.position_rad = s.q;
  fb.velocity_rad_s = s.dq;
  fb.torque = s.tau;
  fb.angle_deg = radToAngle(s.q, config_);
  fb.have_feedback = s.initialized;
  fb.command_rad = s.q_cmd;
  fb.soft_torque_limited = s.soft_torque_limited;
  return fb;
}

bool Gripper::waitFor(Side side, double target_deg, double tolerance_deg,
                      std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  const double tol = std::max(0.0, tolerance_deg);
  while (std::chrono::steady_clock::now() < deadline) {
    const auto fb = feedback(side);
    if (fb.have_feedback && std::fabs(fb.angle_deg - target_deg) <= tol) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return false;
}

bool Gripper::waitForRad(Side side, double target_rad, double tolerance_rad,
                         std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  const double tol = std::max(0.0, tolerance_rad);
  while (std::chrono::steady_clock::now() < deadline) {
    const auto fb = feedback(side);
    if (fb.have_feedback && std::fabs(fb.position_rad - target_rad) <= tol) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

double Gripper::ratioToRad(double ratio) const {
  const double r = clamp(ratio, 0.0, config_.max_close_ratio);
  return config_.open_position_rad + r * (config_.close_position_rad - config_.open_position_rad);
}

double Gripper::radToRatio(double q) const {
  const double denom = config_.close_position_rad - config_.open_position_rad;
  if (std::fabs(denom) < 1e-6) {
    return 0.0;
  }
  return clamp((q - config_.open_position_rad) / denom, 0.0, 1.0);
}

void Gripper::syncSoftMotionState(SideImpl &impl) {
  if (!impl.initialized) {
    return;
  }
  impl.q_anchor = impl.q;
  impl.dq_filt = impl.dq;
  impl.q_cmd = impl.q;
  impl.cmd_initialized = true;
}

void Gripper::updateMotionFilter(SideImpl &impl) {
  if (!impl.initialized) {
    return;
  }
  const double pf = clamp(config_.pos_filter, 1e-3, 1.0);
  impl.q_anchor += pf * (impl.q - impl.q_anchor);
  impl.dq_filt += pf * (impl.dq - impl.dq_filt);
}

void Gripper::applySoftTorqueLimit(SideImpl &impl) {
  const double tau_lim = config_.soft_torque_limit_nm;
  if (tau_lim <= 0.0 || config_.kp <= 1e-6 || !impl.initialized) {
    impl.soft_torque_limited = false;
    return;
  }

  updateMotionFilter(impl);

  // 大行程：只按斜率走，不钳位力矩；同步 anchor，避免夹取后张开时被旧 anchor 锁死
  const double dist_to_target = std::fabs(impl.target_q - impl.q);
  if (dist_to_target > config_.soft_coast_margin_rad) {
    impl.soft_torque_limited = false;
    impl.q_anchor = impl.q;
    return;
  }

  // 接触段用当前实测位置做参考，不用滞后 filter 的 q_anchor
  const double ref_q = impl.q;

  // ros2_170D dex1_gripper_driver_node: tau ≈ kp*(q_cmd-q) - kd*dq
  const double lo = ref_q + (-tau_lim + config_.kd * impl.dq_filt) / config_.kp;
  const double hi = ref_q + (+tau_lim + config_.kd * impl.dq_filt) / config_.kp;

  const double before = impl.q_cmd;
  impl.q_cmd = clamp(impl.q_cmd, std::min(lo, hi), std::max(lo, hi));
  impl.soft_torque_limited = std::fabs(impl.q_cmd - before) > 1e-4;
}

void Gripper::runSoftControlStep(SideImpl &impl, double dt) {
  if (!impl.cmd_initialized) {
    impl.q_cmd = impl.initialized ? impl.q : impl.target_q;
    impl.q_anchor = impl.q_cmd;
    impl.dq_filt = impl.initialized ? impl.dq : 0.0;
    impl.cmd_initialized = true;
  }

  const double dist_to_target = std::fabs(impl.target_q - impl.q);
  const double coast_margin = std::max(0.0, config_.soft_coast_margin_rad);

  if (coast_margin <= 0.0 || dist_to_target > coast_margin) {
    // 大行程：指令直接跟目标，速度与经典一致
    impl.q_cmd = impl.target_q;
  } else {
    const double max_step = std::max(0.0, config_.soft_slew_rate) * dt;
    if (max_step > 0.0) {
      const double err = impl.target_q - impl.q_cmd;
      impl.q_cmd += clamp(err, -max_step, max_step);
    } else {
      impl.q_cmd = impl.target_q;
    }
  }

  applySoftTorqueLimit(impl);
  impl.q_cmd = clamp(impl.q_cmd, config_.close_position_rad, config_.open_position_rad);
}

void Gripper::applyTeleopTarget(SideImpl &impl) {
  const double ratio = clamp(impl.teleop_ratio, 0.0, 1.0);
  double q_des = ratioToRad(ratio);

  const double tau_lim = config_.grasp_torque_limit_nm;
  if (tau_lim > 0.0 && q_des < impl.q - 1e-4 && std::fabs(impl.tau) >= tau_lim) {
    q_des = impl.q;
    impl.torque_limited = true;
  } else {
    impl.torque_limited = false;
  }

  impl.effective_target_q = q_des;
  impl.target_q = q_des;
  impl.q_cmd = q_des;
}

void Gripper::setTeleopMode(bool enabled) {
  teleop_mode_ = enabled;
  if (!enabled) {
    teleop_soft_mode_ = false;
  }
}

void Gripper::setTeleopRatio(Side side, double ratio) {
  std::lock_guard<std::mutex> lock(mutex_);
  teleop_mode_ = true;
  teleop_soft_mode_ = false;
  const auto it = sides_.find(side);
  if (it == sides_.end()) {
    return;
  }
  it->second->teleop_ratio = clamp(ratio, 0.0, 1.0);
  it->second->soft_torque_active = false;
}

void Gripper::setTeleopBoth(double left_ratio, double right_ratio) {
  std::lock_guard<std::mutex> lock(mutex_);
  teleop_mode_ = true;
  teleop_soft_mode_ = false;
  if (const auto it = sides_.find(Side::Left); it != sides_.end()) {
    it->second->teleop_ratio = clamp(left_ratio, 0.0, 1.0);
    it->second->soft_torque_active = false;
  }
  if (const auto it = sides_.find(Side::Right); it != sides_.end()) {
    it->second->teleop_ratio = clamp(right_ratio, 0.0, 1.0);
    it->second->soft_torque_active = false;
  }
}

TeleopFeedback Gripper::teleopFeedback(Side side) const {
  std::lock_guard<std::mutex> lock(mutex_);
  TeleopFeedback out;
  const auto it = sides_.find(side);
  if (it == sides_.end()) {
    return out;
  }
  const auto &s = *it->second;
  out.command_ratio = s.teleop_ratio;
  out.effective_ratio = radToRatio(s.effective_target_q);
  out.position_rad = s.q;
  out.torque = s.tau;
  out.torque_limited = s.torque_limited;
  return out;
}

void Gripper::setTeleopSoftMode(bool enabled) {
  teleop_soft_mode_ = enabled;
  if (enabled) {
    teleop_mode_ = true;
  }
}

void Gripper::setTeleopSoftRatio(Side side, double ratio) {
  std::lock_guard<std::mutex> lock(mutex_);
  teleop_mode_ = true;
  teleop_soft_mode_ = true;
  const auto it = sides_.find(side);
  if (it == sides_.end()) {
    return;
  }
  it->second->teleop_ratio = clamp(ratio, 0.0, 1.0);
  it->second->soft_torque_active = true;
  syncSoftMotionState(*it->second);
}

void Gripper::setTeleopSoftBoth(double left_ratio, double right_ratio) {
  std::lock_guard<std::mutex> lock(mutex_);
  teleop_mode_ = true;
  teleop_soft_mode_ = true;
  if (const auto it = sides_.find(Side::Left); it != sides_.end()) {
    it->second->teleop_ratio = clamp(left_ratio, 0.0, 1.0);
    it->second->soft_torque_active = true;
  }
  if (const auto it = sides_.find(Side::Right); it != sides_.end()) {
    it->second->teleop_ratio = clamp(right_ratio, 0.0, 1.0);
    it->second->soft_torque_active = true;
  }
}

TeleopFeedback Gripper::teleopSoftFeedback(Side side) const {
  std::lock_guard<std::mutex> lock(mutex_);
  TeleopFeedback out;
  const auto it = sides_.find(side);
  if (it == sides_.end()) {
    return out;
  }
  const auto &s = *it->second;
  out.command_ratio = s.teleop_ratio;
  out.effective_ratio = radToRatio(s.q_cmd);
  out.position_rad = s.q;
  out.torque = s.tau;
  out.torque_limited = s.soft_torque_limited;
  return out;
}

bool Gripper::calibrate(Side side) {
  if (!connected_.load() && !connect()) {
    return false;
  }

  std::shared_ptr<SerialPort> serial;
  int motor_id = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = sides_.find(side);
    if (it == sides_.end()) {
      std::cerr << "[gripper] 未找到 side=" << (side == Side::Left ? "left" : "right") << "\n";
      return false;
    }
    serial = it->second->serial;
    motor_id = it->second->motor_id;
  }

  const bool ok = serial->calibration(MotorType::M4010, motor_id, 0.0f, config_.calibration_limit_rad);
  if (ok) {
    std::cout << "[gripper] motor_id=" << motor_id << " 标定成功\n";
  } else {
    std::cerr << "[gripper] motor_id=" << motor_id << " 标定失败\n";
  }
  return ok;
}

bool Gripper::calibrateInteractive() {
  if (!connect()) {
    return false;
  }

  const auto motors = detectedMotors();
  if (motors.empty()) {
    return false;
  }

  std::cout << "\n=== DEX1 夹爪标定 ===\n";
  std::cout << "步骤：手动将夹爪紧闭到极限，然后输入 s 并回车。\n";
  std::cout << "参考: https://support.unitree.com/home/zh/dex1-1_gripper/dex1_1\n\n";

  int index = 1;
  for (const auto &m : motors) {
    const char *side_name = m.side == Side::Right ? "右" : "左";
    std::cout << "--- 标定 " << index << "/" << motors.size() << " ---\n";
    std::cout << "Motor ID: " << m.motor_id << "  " << side_name << "夹爪  串口: " << m.port << "\n";
    std::cout << "请手动紧闭夹爪，完成后输入 s 回车（其他键跳过）: ";
    char key = 0;
    std::cin >> key;
    if (key == 's' || key == 'S') {
      if (!calibrate(m.side)) {
        return false;
      }
    } else {
      std::cout << "已跳过 motor_id=" << m.motor_id << "\n";
    }
    ++index;
  }

  std::cout << "\n标定流程结束。\n";
  return true;
}

void Gripper::controlLoop() {
  using clock = std::chrono::steady_clock;
  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, config_.control_hz));
  auto next_tick = clock::now();

  const double kp_scale = config_.kp;
  const double kd_scale = config_.kd;

  while (!stop_requested_.load()) {
    const auto tick_start = clock::now();
    const double dt = std::max(0.0, std::chrono::duration<double>(tick_start - next_tick).count());

    std::lock_guard<std::mutex> lock(mutex_);
    const bool teleop = teleop_mode_.load();
    const bool teleop_soft = teleop_soft_mode_.load();

    for (auto &[side, impl] : sides_) {
      (void)side;
      if (!impl->serial) {
        continue;
      }

      if (teleop && teleop_soft) {
        impl->soft_torque_active = true;
        impl->target_q = ratioToRad(clamp(impl->teleop_ratio, 0.0, 1.0));
        runSoftControlStep(*impl, dt);
        impl->effective_target_q = impl->q_cmd;
        impl->torque_limited = impl->soft_torque_limited;
      } else if (teleop) {
        applyTeleopTarget(*impl);
      } else if (impl->soft_torque_active) {
        runSoftControlStep(*impl, dt);
      } else {
        const double slew_rate = config_.default_slew_rate;
        const double max_step = std::max(0.0, slew_rate) * dt;
        if (max_step > 0.0) {
          const double err = impl->target_q - impl->q_cmd;
          impl->q_cmd += clamp(err, -max_step, max_step);
        } else {
          impl->q_cmd = impl->target_q;
        }
        impl->soft_torque_limited = false;
        updateMotionFilter(*impl);
        impl->q_cmd =
            clamp(impl->q_cmd, config_.close_position_rad, config_.open_position_rad);
      }

      MotorCmd cmd;
      MotorData data;
      cmd.motorType = MotorType::M4010;
      cmd.id = impl->motor_id;
      cmd.mode = queryMotorMode(cmd.motorType, MotorMode::FOC);
      data.motorType = cmd.motorType;

      const float gr = impl->gear_ratio;
      const float gr2 = gr * gr;
      cmd.kp = static_cast<float>(kp_scale / gr2);
      cmd.kd = static_cast<float>(kd_scale / gr2);
      cmd.q = static_cast<float>(impl->q_cmd * gr);
      cmd.dq = 0.0f;
      cmd.tau = 0.0f;
      cmd.timeout = 0;

      if (impl->serial->sendRecv(&cmd, &data)) {
        impl->q = data.q / gr;
        impl->dq = data.dq / gr;
        impl->tau = data.tau * gr;
        impl->initialized = true;
      }
    }

    next_tick += std::chrono::duration_cast<clock::duration>(period);
    std::this_thread::sleep_until(next_tick);
  }
}

}  // namespace gripper

// =============================================================================
// END: src/gripper_interface.cpp
// =============================================================================

// =============================================================================
// BEGIN: src/tts_http_client.cpp
// =============================================================================
#include "tts_http_client.h"

#include <arpa/inet.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <netdb.h>
#include <cstdlib>
#include <sstream>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <unordered_map>
#include <vector>

namespace
{
    struct ParsedUrl
    {
        std::string host;
        int port = 80;
        std::string base_path = "";
    };

    bool starts_with(const std::string &s, const std::string &prefix)
    {
        return s.rfind(prefix, 0) == 0;
    }

    std::string trim(const std::string &s)
    {
        size_t b = 0;
        while (b < s.size() && (s[b] == ' ' || s[b] == '\t' || s[b] == '\r' || s[b] == '\n'))
            ++b;
        size_t e = s.size();
        while (e > b && (s[e - 1] == ' ' || s[e - 1] == '\t' || s[e - 1] == '\r' || s[e - 1] == '\n'))
            --e;
        return s.substr(b, e - b);
    }

    std::string to_lower(std::string s)
    {
        for (char &c : s)
        {
            if (c >= 'A' && c <= 'Z')
                c = static_cast<char>(c - 'A' + 'a');
        }
        return s;
    }

    bool parse_http_base_url(const std::string &base_url, ParsedUrl &out, std::string &err)
    {
        if (!starts_with(base_url, "http://"))
        {
            err = "only http:// is supported";
            return false;
        }
        std::string rest = base_url.substr(std::strlen("http://"));
        if (rest.empty())
        {
            err = "invalid base_url";
            return false;
        }

        size_t slash = rest.find('/');
        std::string hostport = (slash == std::string::npos) ? rest : rest.substr(0, slash);
        out.base_path = (slash == std::string::npos) ? "" : rest.substr(slash);
        if (!out.base_path.empty() && out.base_path.back() == '/')
            out.base_path.pop_back();

        size_t colon = hostport.rfind(':');
        if (colon == std::string::npos)
        {
            out.host = hostport;
            out.port = 80;
        }
        else
        {
            out.host = hostport.substr(0, colon);
            std::string port_str = hostport.substr(colon + 1);
            if (port_str.empty())
            {
                err = "invalid port";
                return false;
            }
            out.port = std::stoi(port_str);
        }

        if (out.host.empty())
        {
            err = "host is empty";
            return false;
        }
        return true;
    }

    std::string json_escape(const std::string &s)
    {
        std::ostringstream oss;
        for (unsigned char c : s)
        {
            switch (c)
            {
            case '\"':
                oss << "\\\"";
                break;
            case '\\':
                oss << "\\\\";
                break;
            case '\b':
                oss << "\\b";
                break;
            case '\f':
                oss << "\\f";
                break;
            case '\n':
                oss << "\\n";
                break;
            case '\r':
                oss << "\\r";
                break;
            case '\t':
                oss << "\\t";
                break;
            default:
                if (c < 0x20)
                {
                    static const char *hex = "0123456789abcdef";
                    oss << "\\u00" << hex[(c >> 4) & 0x0F] << hex[c & 0x0F];
                }
                else
                {
                    oss << static_cast<char>(c);
                }
                break;
            }
        }
        return oss.str();
    }

    bool recv_all(int fd, std::string &out, std::string &err)
    {
        char buf[8192];
        while (true)
        {
            const ssize_t n = recv(fd, buf, sizeof(buf), 0);
            if (n > 0)
            {
                out.append(buf, static_cast<size_t>(n));
                continue;
            }
            if (n == 0)
                return true;
            if (errno == EINTR)
                continue;
            err = std::string("recv failed: ") + std::strerror(errno);
            return false;
        }
    }

    bool parse_http_response(const std::string &raw,
                             int &status,
                             std::unordered_map<std::string, std::string> &headers,
                             std::string &body,
                             std::string &err)
    {
        const std::string delim = "\r\n\r\n";
        const size_t header_end = raw.find(delim);
        if (header_end == std::string::npos)
        {
            err = "invalid HTTP response: no header terminator";
            return false;
        }

        const std::string header_block = raw.substr(0, header_end);
        body = raw.substr(header_end + delim.size());

        std::istringstream hs(header_block);
        std::string status_line;
        if (!std::getline(hs, status_line))
        {
            err = "invalid HTTP response: no status line";
            return false;
        }
        if (!status_line.empty() && status_line.back() == '\r')
            status_line.pop_back();

        {
            std::istringstream sl(status_line);
            std::string http_ver;
            sl >> http_ver >> status;
            if (http_ver.empty() || status <= 0)
            {
                err = "invalid status line: " + status_line;
                return false;
            }
        }

        std::string line;
        while (std::getline(hs, line))
        {
            if (!line.empty() && line.back() == '\r')
                line.pop_back();
            if (line.empty())
                continue;
            size_t colon = line.find(':');
            if (colon == std::string::npos)
                continue;
            std::string k = to_lower(trim(line.substr(0, colon)));
            std::string v = trim(line.substr(colon + 1));
            headers[k] = v;
        }
        return true;
    }

    std::string guess_ext(const std::string &content_type)
    {
        const std::string ct = to_lower(content_type);
        if (ct.find("wav") != std::string::npos || ct.find("wave") != std::string::npos)
            return ".wav";
        if (ct.find("mpeg") != std::string::npos || ct.find("mp3") != std::string::npos)
            return ".mp3";
        if (ct.find("ogg") != std::string::npos)
            return ".ogg";
        return ".bin";
    }

    std::string find_player_cmd()
    {
        const char *path_env = std::getenv("PATH");
        const std::string path = path_env ? path_env : "";
        std::vector<std::string> dirs;
        std::stringstream ss(path);
        std::string dir;
        while (std::getline(ss, dir, ':'))
            dirs.push_back(dir);

        const std::vector<std::string> players = {"ffplay -nodisp -autoexit", "paplay", "aplay", "mpg123"};
        for (const auto &p : players)
        {
            const std::string exe = p.substr(0, p.find(' '));
            for (const auto &d : dirs)
            {
                const std::string full = d + "/" + exe;
                if (::access(full.c_str(), X_OK) == 0)
                    return p;
            }
        }
        return "";
    }
}

namespace tts
{
    namespace
    {
        std::string make_quiet_play_command(const std::string &player, const std::string &path)
        {
            const std::string qpath = "\"" + path + "\"";
            if (starts_with(player, "ffplay"))
            {
                return player + " -loglevel quiet -hide_banner " + qpath + " >/dev/null 2>&1";
            }
            if (player == "paplay" || player == "aplay")
            {
                return player + " " + qpath + " >/dev/null 2>&1";
            }
            if (starts_with(player, "mpg123"))
            {
                return player + " -q " + qpath + " >/dev/null 2>&1";
            }
            return player + " " + qpath + " >/dev/null 2>&1";
        }

        void log_tts_result(const char *scene, const TtsResult &ret)
        {
            const std::string tag = scene ? (std::string("[TTS] ") + scene) : "[TTS]";
            if (!ret.ok)
            {
                std::cerr << tag << " 播报失败: " << ret.error_message << std::endl;
            }
            else
            {
                std::cout << tag << " 播报成功" << std::endl;
            }
        }
    }

    TtsResult SynthesizeToFile(const std::string &base_url,
                               const std::string &text,
                               const std::string &output_path,
                               int timeout_sec)
    {
        TtsResult ret;
        if (output_path.empty())
        {
            ret.error_message = "output_path is empty";
            return ret;
        }

        ParsedUrl url;
        if (timeout_sec <= 0)
            timeout_sec = 30;

        if (!parse_http_base_url(base_url, url, ret.error_message))
            return ret;

        const std::string req_path = (url.base_path.empty() ? "" : url.base_path) + "/tts";
        const std::string body = std::string("{\"text\":\"") + json_escape(text) + "\"}";

        struct addrinfo hints;
        std::memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;

        struct addrinfo *res = nullptr;
        const std::string port_str = std::to_string(url.port);
        int gai = getaddrinfo(url.host.c_str(), port_str.c_str(), &hints, &res);
        if (gai != 0)
        {
            ret.error_message = std::string("getaddrinfo failed: ") + gai_strerror(gai);
            return ret;
        }

        int fd = -1;
        for (struct addrinfo *p = res; p != nullptr; p = p->ai_next)
        {
            fd = socket(p->ai_family, p->ai_socktype, p->ai_protocol);
            if (fd < 0)
                continue;

            struct timeval tv;
            tv.tv_sec = timeout_sec;
            tv.tv_usec = 0;
            setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
            setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

            if (connect(fd, p->ai_addr, p->ai_addrlen) == 0)
                break;

            close(fd);
            fd = -1;
        }
        freeaddrinfo(res);

        if (fd < 0)
        {
            ret.error_message = std::string("connect failed: ") + std::strerror(errno);
            return ret;
        }

        std::ostringstream req;
        req << "POST " << req_path << " HTTP/1.1\r\n";
        req << "Host: " << url.host << ":" << url.port << "\r\n";
        req << "Content-Type: application/json\r\n";
        req << "Content-Length: " << body.size() << "\r\n";
        req << "Connection: close\r\n";
        req << "\r\n";
        req << body;
        const std::string req_str = req.str();

        size_t sent_total = 0;
        while (sent_total < req_str.size())
        {
            ssize_t n = send(fd, req_str.data() + sent_total, req_str.size() - sent_total, 0);
            if (n > 0)
            {
                sent_total += static_cast<size_t>(n);
                continue;
            }
            if (n < 0 && errno == EINTR)
                continue;
            ret.error_message = std::string("send failed: ") + std::strerror(errno);
            close(fd);
            return ret;
        }

        std::string raw_resp;
        if (!recv_all(fd, raw_resp, ret.error_message))
        {
            close(fd);
            return ret;
        }
        close(fd);

        std::unordered_map<std::string, std::string> headers;
        std::string resp_body;
        int status = -1;
        if (!parse_http_response(raw_resp, status, headers, resp_body, ret.error_message))
            return ret;

        ret.http_status = status;
        auto it_ct = headers.find("content-type");
        if (it_ct != headers.end())
            ret.content_type = it_ct->second;

        if (ret.http_status != 200)
        {
            ret.error_message = "http status is not 200";
            return ret;
        }

        if (ret.content_type.rfind("audio/", 0) != 0)
        {
            ret.error_message = std::string("unexpected content-type: ") + ret.content_type;
            return ret;
        }

        std::ofstream ofs(output_path, std::ios::binary);
        if (!ofs.is_open())
        {
            ret.error_message = std::string("open output failed: ") + output_path;
            return ret;
        }
        ofs.write(resp_body.data(), static_cast<std::streamsize>(resp_body.size()));
        if (!ofs.good())
        {
            ret.error_message = "write output failed";
            return ret;
        }

        ret.ok = true;
        ret.output_path = output_path;
        ret.audio_bytes = resp_body.size();
        return ret;
    }

    TtsResult SpeakText(const std::string &text,
                        const std::string &base_url,
                        int timeout_sec)
    {
        const auto now = std::chrono::system_clock::now();
        const auto ts = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
        const std::string tmp_base = "/tmp/ti5_tts_" + std::to_string(ts);

        // 先写成 .bin，拿到 content-type 后再按扩展名重命名
        TtsResult ret = SynthesizeToFile(base_url, text, tmp_base + ".bin", timeout_sec);
        if (!ret.ok)
            return ret;

        const std::string ext = guess_ext(ret.content_type);
        const std::string final_path = tmp_base + ext;
        if (::rename(ret.output_path.c_str(), final_path.c_str()) == 0)
            ret.output_path = final_path;

        const std::string player = find_player_cmd();
        if (player.empty())
        {
            ret.ok = false;
            ret.error_message = "未找到可用播放器(ffplay/paplay/aplay/mpg123)";
            return ret;
        }

        const std::string cmd = make_quiet_play_command(player, ret.output_path);
        const int play_rc = std::system(cmd.c_str());
        if (play_rc != 0)
        {
            ret.ok = false;
            ret.error_message = "音频播放失败";
            return ret;
        }
        return ret;
    }

    void SpeakTextWithLog(const std::string &text,
                          const std::string &base_url,
                          int timeout_sec,
                          const char *scene)
    {
        log_tts_result(scene, SpeakText(text, base_url, timeout_sec));
    }

    void SpeakTextAsync(std::string text,
                        const std::string &base_url,
                        int timeout_sec,
                        const char *scene)
    {
        std::thread(
            [t = std::move(text), base_url = std::string(base_url), timeout_sec, scene]()
            {
                SpeakTextWithLog(t, base_url, timeout_sec, scene);
            })
            .detach();
    }

    namespace
    {
        const char kWelcomeScene[] =
            "欢迎体验爱仕达无人售卖区，多款精美钛杯随心选，快来挑选 你的心仪款吧～";
        const char kAfterOrder[] =
            "哇，眼光超棒！这款杯子超受欢迎的～ 请稍作等候，马上为你取杯";
        const char kCupTaken[] = "杯子已成功取出，颜值质感双在线，超赞的";
        const char kDeliveryDone[] =
            "你的专属钛杯已送达～ 愿这杯美好伴你日日舒心，下次再来呀！";
    }

    void SpeakWelcomeSceneAsync()
    {
        SpeakTextAsync(kWelcomeScene, "http://127.0.0.1:4990", 30, "欢迎语");
    }

    void SpeakAfterOrderAsync()
    {
        SpeakTextAsync(kAfterOrder, "http://127.0.0.1:4990", 30, "下单确认");
    }

    void SpeakCupTakenAsync()
    {
        SpeakTextAsync(kCupTaken, "http://127.0.0.1:4990", 30, "取杯");
    }

    void SpeakDeliveryDoneAsync()
    {
        SpeakTextAsync(kDeliveryDone, "http://127.0.0.1:4990", 30, "送达完成");
    }
}

// =============================================================================
// END: src/tts_http_client.cpp
// =============================================================================

// =============================================================================
// BEGIN: src/realsense_get.cpp
// =============================================================================
#include "realsense_get.h"

#include <cmath>
#include <cstring>
#include <iostream>
#include <map>
#include <stdexcept>
#include <vector>

#include <librealsense2/rs.hpp>
#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>

namespace
{

constexpr const char *kSlotKeys[] = {"head", "right_hand", "left_hand"};
constexpr int kWarmupFrames = 15;

struct SlotDevice
{
    std::string slot_id;
    std::string label;
    std::string serial;
    rs2::pipeline pipeline;
    rs2::align align{RS2_STREAM_COLOR};
    float depth_scale = 0.001f;
    std::array<double, 9> K{};
    std::array<double, 5> dist{};
    bool started = false;
};

std::string slot_key(CameraSlot slot)
{
    const auto idx = static_cast<size_t>(slot);
    if (idx >= 3)
        return {};
    return kSlotKeys[idx];
}

void fill_intrinsics(const rs2_intrinsics &intr, std::array<double, 9> &K, std::array<double, 5> &dist)
{
    K = {
        intr.fx, 0.0, intr.ppx,
        0.0, intr.fy, intr.ppy,
        0.0, 0.0, 1.0};
    for (int i = 0; i < 5; ++i)
        dist[static_cast<size_t>(i)] = intr.coeffs[i];
}

void mat_bgr_to_vector(const cv::Mat &bgr, std::vector<uint8_t> &out)
{
    CV_Assert(bgr.type() == CV_8UC3 && bgr.isContinuous());
    const size_t n = static_cast<size_t>(bgr.rows) * static_cast<size_t>(bgr.cols) * 3u;
    out.resize(n);
    std::memcpy(out.data(), bgr.data, n);
}

void mat_depth_to_vector(const cv::Mat &depth_m, std::vector<float> &out)
{
    CV_Assert(depth_m.type() == CV_32FC1 && depth_m.isContinuous());
    const size_t n = static_cast<size_t>(depth_m.rows) * static_cast<size_t>(depth_m.cols);
    out.resize(n);
    std::memcpy(out.data(), depth_m.ptr<float>(), n * sizeof(float));
}

cv::Mat depth_z16_to_meters(const rs2::frame &depth_frame, float depth_scale)
{
    const int h = depth_frame.as<rs2::video_frame>().get_height();
    const int w = depth_frame.as<rs2::video_frame>().get_width();
    const auto *raw = reinterpret_cast<const uint16_t *>(depth_frame.get_data());

    cv::Mat depth_m(h, w, CV_32FC1);
    for (int y = 0; y < h; ++y)
    {
        auto *row = depth_m.ptr<float>(y);
        for (int x = 0; x < w; ++x)
        {
            const uint16_t z = raw[y * w + x];
            row[x] = (z == 0) ? std::nanf("") : static_cast<float>(z) * depth_scale;
        }
    }
    return depth_m;
}

} // namespace

struct RealSenseMultiCam::Impl
{
    std::string yaml_path;
    int width = 1280;
    int height = 720;
    int fps = 30;
    std::map<std::string, SlotDevice> slots;
};

RealSenseMultiCam::RealSenseMultiCam(std::string yaml_path, int width, int height, int fps)
    : impl_(std::make_unique<Impl>())
{
    impl_->yaml_path = std::move(yaml_path);
    impl_->width = width;
    impl_->height = height;
    impl_->fps = fps;
}

RealSenseMultiCam::~RealSenseMultiCam()
{
    stop();
}

const char *RealSenseMultiCam::slot_name(CameraSlot slot)
{
    return slot_key(slot).c_str();
}

CameraSlot RealSenseMultiCam::slot_from_choice(int choice)
{
    switch (choice)
    {
    case 1:
        return CameraSlot::Head;
    case 2:
        return CameraSlot::RightHand;
    case 3:
        return CameraSlot::LeftHand;
    default:
        throw std::invalid_argument("invalid camera choice");
    }
}

bool RealSenseMultiCam::init(std::string &err)
{
    err.clear();
    stop();

    YAML::Node root;
    try
    {
        root = YAML::LoadFile(impl_->yaml_path);
    }
    catch (const std::exception &e)
    {
        err = std::string("读取相机配置失败: ") + e.what();
        return false;
    }

    const YAML::Node realsense = root["realsense"];
    if (!realsense || !realsense.IsMap())
    {
        err = "配置缺少 realsense 节点: " + impl_->yaml_path;
        return false;
    }

    for (const char *key : kSlotKeys)
    {
        const YAML::Node node = realsense[key];
        if (!node || !node.IsMap())
        {
            err = std::string("配置缺少相机位置: ") + key;
            return false;
        }

        SlotDevice dev;
        dev.slot_id = key;
        dev.label = node["label"] ? node["label"].as<std::string>() : key;
        dev.serial = node["serial"] ? node["serial"].as<std::string>() : "";
        if (dev.serial.empty())
        {
            err = std::string("序列号为空: ") + key;
            return false;
        }
        impl_->slots.emplace(key, std::move(dev));
    }

    for (auto &kv : impl_->slots)
    {
        auto &dev = kv.second;
        try
        {
            rs2::config cfg;
            cfg.enable_device(dev.serial);
            cfg.enable_stream(RS2_STREAM_COLOR, impl_->width, impl_->height, RS2_FORMAT_BGR8, impl_->fps);
            cfg.enable_stream(RS2_STREAM_DEPTH, impl_->width, impl_->height, RS2_FORMAT_Z16, impl_->fps);

            const rs2::pipeline_profile profile = dev.pipeline.start(cfg);
            const rs2::video_stream_profile color_sp =
                profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
            fill_intrinsics(color_sp.get_intrinsics(), dev.K, dev.dist);

            const rs2::device device = profile.get_device();
            const rs2::depth_sensor depth_sensor = device.first<rs2::depth_sensor>();
            dev.depth_scale = depth_sensor.get_depth_scale();

            for (int i = 0; i < kWarmupFrames; ++i)
                dev.pipeline.wait_for_frames();

            dev.started = true;
            std::cout << "RealSense 已启动: " << dev.label << " (" << dev.slot_id
                      << ") SN=" << dev.serial << std::endl;
        }
        catch (const std::exception &e)
        {
            err = std::string("打开相机失败 [") + dev.slot_id + "] SN=" + dev.serial + ": " + e.what();
            stop();
            return false;
        }
    }

    return true;
}

CameraFrameData RealSenseMultiCam::grab(CameraSlot slot)
{
    CameraFrameData out;
    const std::string key = slot_key(slot);
    auto it = impl_->slots.find(key);
    if (it == impl_->slots.end() || !it->second.started)
    {
        out.message = std::string("相机未初始化: ") + key;
        return out;
    }

    auto &dev = it->second;
    try
    {
        const rs2::frameset frames = dev.pipeline.wait_for_frames();
        const rs2::frameset aligned = dev.align.process(frames);
        const rs2::frame color_f = aligned.get_color_frame();
        const rs2::frame depth_f = aligned.get_depth_frame();
        if (!color_f || !depth_f)
        {
            out.message = "取帧失败: 彩色或深度为空";
            return out;
        }

        const int h = color_f.as<rs2::video_frame>().get_height();
        const int w = color_f.as<rs2::video_frame>().get_width();
        const auto *color_ptr = reinterpret_cast<const uint8_t *>(color_f.get_data());
        const cv::Mat bgr(h, w, CV_8UC3, const_cast<uint8_t *>(color_ptr));
        const cv::Mat depth_m = depth_z16_to_meters(depth_f, dev.depth_scale);

        out.width = w;
        out.height = h;
        mat_bgr_to_vector(bgr.clone(), out.color_bgr);
        mat_depth_to_vector(depth_m, out.depth_m);
        out.K = dev.K;
        out.dist = dev.dist;
        out.ok = true;
        out.message = "ok";
    }
    catch (const std::exception &e)
    {
        out.message = std::string("grab 异常: ") + e.what();
    }
    return out;
}

CameraFrameData RealSenseMultiCam::prepare_frame_for_slot(CameraFrameData frame, CameraSlot slot)
{
    if (!frame.ok || slot != CameraSlot::RightHand)
        return frame;

    const int w = frame.width;
    const int h = frame.height;
    if (w <= 0 || h <= 0)
        return frame;

    cv::Mat bgr(h, w, CV_8UC3, frame.color_bgr.data());
    cv::Mat depth(h, w, CV_32FC1, frame.depth_m.data());

    cv::Mat bgr_rot;
    cv::Mat depth_rot;
    cv::rotate(bgr, bgr_rot, cv::ROTATE_180);
    cv::rotate(depth, depth_rot, cv::ROTATE_180);

    mat_bgr_to_vector(bgr_rot, frame.color_bgr);
    mat_depth_to_vector(depth_rot, frame.depth_m);

    // 图像旋转 180° 后主点与切向畸变需同步变换
    frame.K[2] = static_cast<double>(w - 1) - frame.K[2];
    frame.K[5] = static_cast<double>(h - 1) - frame.K[5];
    frame.dist[2] = -frame.dist[2]; // p1
    frame.dist[3] = -frame.dist[3]; // p2

    return frame;
}

void RealSenseMultiCam::stop()
{
    for (auto &kv : impl_->slots)
    {
        if (kv.second.started)
        {
            try
            {
                kv.second.pipeline.stop();
            }
            catch (...)
            {
            }
            kv.second.started = false;
        }
    }
}

// =============================================================================
// END: src/realsense_get.cpp
// =============================================================================

// =============================================================================
// BEGIN: src/seg_pose_bridge.cpp
// =============================================================================
#include "seg_pose_bridge.h"

#include "function.h"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <pwd.h>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>

#include <Python.h>
#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

namespace py = pybind11;

namespace
{

std::atomic<bool> g_app_stop{false};

void on_sigint(int)
{
    _Exit(130);
}

} // namespace

void request_app_stop()
{
    g_app_stop.store(true, std::memory_order_relaxed);
}

bool app_stop_requested()
{
    return g_app_stop.load(std::memory_order_relaxed);
}

void install_app_sigint_handler()
{
    struct sigaction sa{};
    sa.sa_handler = on_sigint;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
}

void interruptible_sleep_ms(int ms)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (!app_stop_requested() && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

namespace
{

#ifndef SEG_POSE_CONDA_PREFIX
#define SEG_POSE_CONDA_PREFIX "/home/ti5robot/anaconda3/envs/human_interaction_env"
#endif

std::string resolve_user_home()
{
    const char *home = std::getenv("HOME");
    const char *sudo_user = std::getenv("SUDO_USER");
    if (sudo_user != nullptr && sudo_user[0] != '\0' &&
        (home == nullptr || std::string(home) == "/root"))
    {
        if (const passwd *pw = getpwnam(sudo_user))
            return pw->pw_dir;
    }
    return (home != nullptr) ? home : "";
}

void prepend_path_env(const char *key, const std::string &prefix)
{
    if (prefix.empty())
        return;
    const char *existing = std::getenv(key);
    std::string merged = prefix;
    if (existing != nullptr && existing[0] != '\0')
    {
        merged += ':';
        merged += existing;
    }
    setenv(key, merged.c_str(), 1);
}

/** 使用 human_interaction_env（与 seg_circle_pose/main.py 相同 conda 环境） */
void setup_embedded_python_env()
{
    const std::string home = resolve_user_home();
    if (!home.empty())
        setenv("HOME", home.c_str(), 1);

    const std::string conda = SEG_POSE_CONDA_PREFIX;
    setenv("PYTHONHOME", conda.c_str(), 1);

    const std::string site = conda + "/lib/python3." + std::to_string(PY_MAJOR_VERSION) + "." +
                             std::to_string(PY_MINOR_VERSION) + "/site-packages";
    prepend_path_env("PYTHONPATH", site);
    prepend_path_env("PATH", conda + "/bin");
}

constexpr const char *kPoseVisWindow = "pose_vis_all";
constexpr int kPanelDispW = 640;
constexpr int kPanelDispH = 480;

std::array<py::object, 3> g_vis_panel_images{py::none(), py::none(), py::none()};
bool g_vis_composite_window_ready = false;

size_t camera_slot_index(CameraSlot slot)
{
    return static_cast<size_t>(slot);
}

void store_vis_panel(CameraSlot slot, const py::object &vis_bgr)
{
    const size_t idx = camera_slot_index(slot);
    if (idx >= g_vis_panel_images.size())
        return;
    g_vis_panel_images[idx] = vis_bgr;
}

void show_pose_visualization_composite()
{
    py::module_ cv2 = py::module_::import("cv2");
    py::module_ np = py::module_::import("numpy");

    if (!g_vis_composite_window_ready)
    {
        cv2.attr("namedWindow")(kPoseVisWindow, cv2.attr("WINDOW_NORMAL"));
        cv2.attr("resizeWindow")(kPoseVisWindow, kPanelDispW * 3, kPanelDispH);
        g_vis_composite_window_ready = true;
    }

    static const char *labels[] = {"Head", "RightHand", "LeftHand"};
    py::list panels;
    for (size_t i = 0; i < g_vis_panel_images.size(); ++i)
    {
        py::object panel;
        if (!g_vis_panel_images[i].is_none())
        {
            panel = cv2.attr("resize")(
                g_vis_panel_images[i], py::make_tuple(kPanelDispW, kPanelDispH));
        }
        else
        {
            panel = np.attr("zeros")(
                py::make_tuple(kPanelDispH, kPanelDispW, 3), py::arg("dtype") = np.attr("uint8"));
        }
        cv2.attr("putText")(
            panel,
            labels[i],
            py::make_tuple(10, 28),
            cv2.attr("FONT_HERSHEY_SIMPLEX"),
            0.8,
            py::make_tuple(0, 255, 255),
            2,
            cv2.attr("LINE_AA"));
        panels.append(panel);
    }

    py::object composite = np.attr("hstack")(panels);
    cv2.attr("imshow")(kPoseVisWindow, composite);
    cv2.attr("waitKey")(1);
}

py::array bgr_vector_to_numpy(int h, int w, const std::vector<uint8_t> &bgr)
{
    const size_t expect = static_cast<size_t>(h) * static_cast<size_t>(w) * 3u;
    if (static_cast<size_t>(bgr.size()) != expect)
        throw std::runtime_error("color_bgr size mismatch");
    return py::array_t<uint8_t>(
        {h, w, 3},
        {static_cast<py::ssize_t>(w * 3), static_cast<py::ssize_t>(3), static_cast<py::ssize_t>(1)},
        bgr.data());
}

py::array depth_vector_to_numpy(int h, int w, const std::vector<float> &depth_m)
{
    const size_t expect = static_cast<size_t>(h) * static_cast<size_t>(w);
    if (static_cast<size_t>(depth_m.size()) != expect)
        throw std::runtime_error("depth_m size mismatch");
    return py::array_t<float>(
        {h, w},
        {static_cast<py::ssize_t>(w * sizeof(float)), static_cast<py::ssize_t>(sizeof(float))},
        depth_m.data());
}

py::array K_to_numpy(const std::array<double, 9> &K)
{
    py::array_t<double> arr({3, 3});
    auto buf = arr.mutable_unchecked<2>();
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            buf(r, c) = K[static_cast<size_t>(r * 3 + c)];
    return arr;
}

py::array dist_to_numpy(const std::array<double, 5> &dist)
{
    py::array_t<double> arr(5);
    auto buf = arr.mutable_unchecked<1>();
    for (int i = 0; i < 5; ++i)
        buf(i) = dist[static_cast<size_t>(i)];
    return arr;
}

std::array<double, 16> numpy_pose_to_array(const py::object &pose_obj)
{
    py::array_t<double> arr = pose_obj.cast<py::array_t<double>>();
    if (arr.ndim() != 2 || arr.shape(0) != 4 || arr.shape(1) != 4)
        throw std::runtime_error("pose_4x4 must be 4x4");

    std::array<double, 16> out{};
    auto buf = arr.unchecked<2>();
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            out[static_cast<size_t>(r * 4 + c)] = buf(r, c);
    return out;
}

void show_pose_visualization(
    const CameraFrameData &frame,
    const py::object &algorithm_output,
    CameraSlot slot)
{
    py::module_ vis_mod = py::module_::import("visualization");

    py::array rgb = bgr_vector_to_numpy(frame.height, frame.width, frame.color_bgr);
    py::object vis = vis_mod.attr("draw_results")(
        rgb,
        algorithm_output,
        K_to_numpy(frame.K),
        dist_to_numpy(frame.dist),
        false);

    store_vis_panel(slot, vis);
    show_pose_visualization_composite();
}

} // namespace

struct SegPoseBridge::Impl
{
    std::unique_ptr<py::scoped_interpreter> interpreter;
    py::object engine;
    py::object algorithm_input_cls;
};

SegPoseBridge::SegPoseBridge() : impl_(std::make_unique<Impl>()) {}

SegPoseBridge::~SegPoseBridge() = default;

bool SegPoseBridge::init(const std::string &pose_config_path, const std::string &seg_root, std::string &err)
{
    err.clear();
    try
    {
        setup_embedded_python_env();
        impl_->interpreter = std::make_unique<py::scoped_interpreter>();

        py::module_ sys = py::module_::import("sys");
        sys.attr("path").attr("insert")(0, seg_root);

        py::module_ config_mod = py::module_::import("algorithm.config_loader");
        py::module_ engine_mod = py::module_::import("algorithm.engine");
        py::module_ types_mod = py::module_::import("algorithm.types");

        py::object params = config_mod.attr("load_pose_params")(pose_config_path);
        impl_->engine = engine_mod.attr("CirclePoseEngine")(params);
        impl_->algorithm_input_cls = types_mod.attr("AlgorithmInput");

        std::cout << "CirclePoseEngine 已加载 (human_interaction_env), 配置: " << pose_config_path
                  << std::endl;
        return true;
    }
    catch (const std::exception &e)
    {
        err = std::string("SegPoseBridge 初始化失败: ") + e.what();
        impl_->engine = py::none();
        return false;
    }
}

PoseRunResult SegPoseBridge::run(
    const CameraFrameData &frame,
    int algorithm_id,
    bool show_visualization,
    CameraSlot vis_slot)
{
    PoseRunResult result;
    if (!impl_->engine || impl_->engine.is_none())
    {
        result.message = "引擎未初始化";
        return result;
    }
    if (!frame.ok || frame.color_bgr.empty() || frame.width <= 0 || frame.height <= 0)
    {
        result.message = "无效相机帧: " + frame.message;
        return result;
    }

    try
    {
        py::object depth_arg;
        if (algorithm_id == 1)
        {
            if (frame.depth_m.empty())
            {
                result.message = "算法 1 需要深度图";
                return result;
            }
            depth_arg = depth_vector_to_numpy(frame.height, frame.width, frame.depth_m);
        }
        else
        {
            depth_arg = py::none();
        }

        py::object inp = impl_->algorithm_input_cls(
            bgr_vector_to_numpy(frame.height, frame.width, frame.color_bgr),
            depth_arg,
            K_to_numpy(frame.K),
            dist_to_numpy(frame.dist),
            algorithm_id);

        py::object out = impl_->engine.attr("run")(inp);
        py::list targets = out.attr("targets");

        const py::ssize_t n = targets.size();
        result.targets.reserve(static_cast<size_t>(n));
        for (py::ssize_t i = 0; i < n; ++i)
        {
            py::object t = targets[i];
            PoseTargetResult one;
            one.class_id = t.attr("class_id").cast<int>();
            one.class_name = py::str(t.attr("class_name"));
            one.algorithm_id = t.attr("algorithm_id").cast<int>();
            one.confidence = t.attr("confidence").cast<float>();
            one.radius_m = t.attr("radius_m").cast<float>();
            one.success = t.attr("success").cast<bool>();
            one.message = py::str(t.attr("message"));
            one.pose_4x4 = numpy_pose_to_array(t.attr("pose_4x4"));
            result.targets.push_back(std::move(one));
        }

        result.ok = true;
        result.message = "ok";

        if (show_visualization)
            show_pose_visualization(frame, out, vis_slot);
    }
    catch (const py::error_already_set &e)
    {
        if (PyErr_ExceptionMatches(PyExc_KeyboardInterrupt))
        {
            PyErr_Clear();
            request_app_stop();
            result.message = "interrupted";
            return result;
        }
        result.message = std::string("run 异常: ") + e.what();
    }
    catch (const std::exception &e)
    {
        result.message = std::string("run 异常: ") + e.what();
    }
    return result;
}

void SegPoseBridge::close_visualization()
{
    g_vis_composite_window_ready = false;
    for (py::object &panel : g_vis_panel_images)
        panel = py::none();
    if (!impl_->interpreter)
        return;
    try
    {
        py::module_ cv2 = py::module_::import("cv2");
        cv2.attr("destroyAllWindows")();
    }
    catch (...)
    {
    }
}

std::string project_root_dir()
{
    if (const char *env = std::getenv("MARKET_SIMPLE_ROOT"))
        return env;

    const std::filesystem::path cwd = std::filesystem::current_path();
    if (std::filesystem::exists(cwd / "config" / "realsense_cameras.yaml"))
        return cwd.string();
    if (std::filesystem::exists(cwd / ".." / "config" / "realsense_cameras.yaml"))
        return std::filesystem::canonical(cwd / "..").string();
    return cwd.string();
}

std::string default_cameras_yaml_path()
{
    return project_root_dir() + "/config/realsense_cameras.yaml";
}

std::string default_pose_yaml_path()
{
    return project_root_dir() + "/seg_circle_pose/config/pose_params.yaml";
}

std::string default_seg_circle_pose_root()
{
    return project_root_dir() + "/seg_circle_pose";
}

std::string default_camera_to_robot_yaml_path()
{
    return project_root_dir() + "/config/camera_to_base_result.yaml";
}

std::string default_left_hand_to_robot_yaml_path()
{
    return project_root_dir() + "/config/left_hand_to_base_result.yaml";
}

std::string default_right_hand_to_robot_yaml_path()
{
    return project_root_dir() + "/config/right_hand_to_base_result.yaml";
}

namespace
{

bool parse_matrix_doubles(const std::string &text, std::array<double, 16> &out)
{
    std::vector<double> vals;
    const char *p = text.c_str();
    while (*p != '\0')
    {
        char *end = nullptr;
        const double v = std::strtod(p, &end);
        if (end == p)
        {
            ++p;
            continue;
        }
        vals.push_back(v);
        p = end;
    }
    if (vals.size() != 16)
        return false;
    for (int i = 0; i < 16; ++i)
        out[static_cast<size_t>(i)] = vals[static_cast<size_t>(i)];
    return true;
}

std::array<double, 16> mat4_mul_row_major(const std::array<double, 16> &a, const std::array<double, 16> &b)
{
    std::array<double, 16> c{};
    for (int r = 0; r < 4; ++r)
    {
        for (int ccol = 0; ccol < 4; ++ccol)
        {
            double sum = 0.0;
            for (int k = 0; k < 4; ++k)
                sum += a[static_cast<size_t>(r * 4 + k)] * b[static_cast<size_t>(k * 4 + ccol)];
            c[static_cast<size_t>(r * 4 + ccol)] = sum;
        }
    }
    return c;
}

/** yaml 标定矩阵平移 mm → m（仅平移列） */
void cam2robot_translation_mm_to_m(std::array<double, 16> &cam2robot)
{
    cam2robot[3] /= 1000.0;
    cam2robot[7] /= 1000.0;
    cam2robot[11] /= 1000.0;
}

Eigen::Matrix<double, 4, 4> row_major_pose_to_matrix4(const std::array<double, 16> &pose)
{
    Eigen::Matrix<double, 4, 4> T;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            T(r, c) = pose[static_cast<size_t>(r * 4 + c)];
    return T;
}

void print_head_target_cam4x4_robot6(
    const char *label,
    const PoseTargetResult &t,
    const std::array<double, 16> &cam2robot)
{
    std::cout << "\n[" << label << "] 相机系 pose_4x4 (row-major, m):\n";
    for (int r = 0; r < 4; ++r)
    {
        for (int c = 0; c < 4; ++c)
            std::cout << std::setw(11) << std::fixed << std::setprecision(6)
                      << t.pose_4x4[static_cast<size_t>(r * 4 + c)] << " ";
        std::cout << "\n";
    }
    const std::array<double, 16> pose_robot = mat4_mul_row_major(cam2robot, t.pose_4x4);
    const Eigen::Matrix<double, 1, 6> pos6 =
        T2PosEulerAngles(row_major_pose_to_matrix4(pose_robot));
    std::cout << "[" << label << "] 基座系 1x6 (x,y,z m; rx,ry,rz rad): "
              << std::setprecision(6) << pos6(0) << ", " << pos6(1) << ", " << pos6(2)
              << ", " << pos6(3) << ", " << pos6(4) << ", " << pos6(5) << "\n";
}

} // namespace

bool load_cam2robot_matrix(const std::string &yaml_path, std::array<double, 16> &out, std::string &err)
{
    err.clear();
    std::ifstream in(yaml_path);
    if (!in)
    {
        err = "无法打开: " + yaml_path;
        return false;
    }

    std::string line;
    if (!std::getline(in, line))
    {
        err = "文件为空: " + yaml_path;
        return false;
    }

    const auto pos = line.find("matrix:");
    if (pos == std::string::npos)
    {
        err = "未找到 matrix 字段";
        return false;
    }

    if (!parse_matrix_doubles(line.substr(pos), out))
    {
        err = "matrix 需要 16 个数";
        return false;
    }
    cam2robot_translation_mm_to_m(out);
    return true;
}

std::array<double, 16> transform_pose_cam_to_robot(
    const std::array<double, 16> &cam2robot,
    const std::array<double, 16> &det_pose_cam_m)
{
    return mat4_mul_row_major(cam2robot, det_pose_cam_m);
}

void assign_hand_targets_in_robot_frame(
    const PoseDetectionRecords &records,
    const std::array<double, 16> &cam2robot,
    double right_hand_pos[3],
    double left_hand_pos[3])
{
    auto set_invalid = [](double pos[3]) {
        pos[0] = -1.0;
        pos[1] = 0.0;
        pos[2] = 0.0;
    };
    auto set_pos = [](double pos[3], double x, double y, double z) {
        pos[0] = x;
        pos[1] = y;
        pos[2] = z;
    };

    set_invalid(right_hand_pos);
    set_invalid(left_hand_pos);

    struct Candidate
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        float confidence = 0.f;
        size_t index = 0;
        double dist_sq_cam = 0.0; // 头相机坐标系下距光心距离平方
    };

    constexpr double kMaxX = 0.8;
    constexpr double kMaxAbsY = 0.38;
    constexpr double kZoneSplit = 0.1;

    std::vector<Candidate> left_zone;
    std::vector<Candidate> right_zone;
    std::vector<Candidate> middle_zone;
    left_zone.reserve(records.size());
    right_zone.reserve(records.size());
    middle_zone.reserve(records.size());

    std::cout << "\n--- robot 基座系 → 左右手分配 (m) ---\n";

    for (size_t i = 0; i < records.size(); ++i)
    {
        const auto &t = records[i].target;
        const std::string name =
            t.class_name.empty() ? ("cls" + std::to_string(t.class_id)) : t.class_name;

        if (!t.success)
        {
            std::cout << "[" << i << "] " << name << " 跳过(FAIL: " << t.message << ")\n";
            continue;
        }

        const std::array<double, 16> pose_robot = transform_pose_cam_to_robot(cam2robot, t.pose_4x4);
        const double x = pose_robot[3];
        const double y = pose_robot[7];
        const double z = pose_robot[11];

        if (x > kMaxX || y > kMaxAbsY || y < -kMaxAbsY)
        {
            std::cout << "[" << i << "] " << name
                      << " 过滤(x=" << std::fixed << std::setprecision(4) << x
                      << " y=" << y << " z=" << z << ")\n";
            continue;
        }

        const double cx = t.pose_4x4[3];
        const double cy = t.pose_4x4[7];
        const double cz = t.pose_4x4[11];
        const double dist_sq_cam = cx * cx + cy * cy + cz * cz;
        Candidate c{x, y, z, t.confidence, i, dist_sq_cam};
        if (y > kZoneSplit)
            left_zone.push_back(c);
        else if (y < -kZoneSplit)
            right_zone.push_back(c);
        else
            middle_zone.push_back(c);

        std::cout << "[" << i << "] " << name
                  << " conf=" << std::setprecision(3) << t.confidence
                  << " x=" << std::setprecision(4) << x
                  << " y=" << y
                  << " z=" << z << "\n";
    }

    auto pick_best = [](const std::vector<Candidate> &zone, int *used_index) -> const Candidate * {
        const Candidate *best = nullptr;
        for (const Candidate &c : zone)
        {
            if (used_index != nullptr && *used_index == static_cast<int>(c.index))
                continue;
            if (best == nullptr || c.dist_sq_cam < best->dist_sq_cam)
                best = &c;
        }
        return best;
    };

    int used_index = -1;
    int right_index = -1;
    int left_index = -1;
    bool right_used_middle = false;

    if (const Candidate *pick = pick_best(right_zone, nullptr))
    {
        set_pos(right_hand_pos, pick->x, pick->y, pick->z);
        used_index = static_cast<int>(pick->index);
        right_index = used_index;
    }
    else if (const Candidate *pick = pick_best(middle_zone, nullptr))
    {
        set_pos(right_hand_pos, pick->x, pick->y, pick->z);
        used_index = static_cast<int>(pick->index);
        right_index = used_index;
        right_used_middle = true;
    }

    if (const Candidate *pick = pick_best(left_zone, nullptr))
    {
        set_pos(left_hand_pos, pick->x, pick->y, pick->z);
        left_index = static_cast<int>(pick->index);
        if (used_index < 0)
            used_index = left_index;
    }
    else if (!right_used_middle)
    {
        if (const Candidate *pick = pick_best(middle_zone, used_index >= 0 ? &used_index : nullptr))
        {
            set_pos(left_hand_pos, pick->x, pick->y, pick->z);
            left_index = static_cast<int>(pick->index);
        }
    }

    std::cout << "右手: ";
    if (right_hand_pos[0] < 0.0)
        std::cout << "无效\n";
    else
        std::cout << "x=" << std::setprecision(4) << right_hand_pos[0]
                  << " y=" << right_hand_pos[1]
                  << " z=" << right_hand_pos[2]
                  << (right_used_middle ? " (中间区)\n" : " (侧区)\n");

    std::cout << "左手: ";
    if (left_hand_pos[0] < 0.0)
        std::cout << "无效\n";
    else
        std::cout << "x=" << std::setprecision(4) << left_hand_pos[0]
                  << " y=" << left_hand_pos[1]
                  << " z=" << left_hand_pos[2] << "\n";

    std::cout << "\n--- 头部分配目标：相机 4x4 / 基座 1x6 ---\n";
    if (right_index >= 0)
        print_head_target_cam4x4_robot6("右手目标", records[static_cast<size_t>(right_index)].target, cam2robot);
    else
        std::cout << "[右手目标] 无有效分配\n";
    if (left_index >= 0)
        print_head_target_cam4x4_robot6("左手目标", records[static_cast<size_t>(left_index)].target, cam2robot);
    else
        std::cout << "[左手目标] 无有效分配\n";
}

void assign_nearest_hand_cam_target_in_robot_frame(
    const PoseDetectionRecords &records,
    const std::array<double, 16> &cam2robot,
    Eigen::Matrix<double, 1, 6> &out_pose)
{
    out_pose << -1.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    const auto *best = static_cast<const PoseDetectionRecord *>(nullptr);
    double best_dist_sq = 0.0;

    for (const PoseDetectionRecord &rec : records)
    {
        const auto &t = rec.target;
        if (!t.success)
            continue;

        const double x = t.pose_4x4[3];
        const double y = t.pose_4x4[7];
        const double z = t.pose_4x4[11];
        const double dist_sq = x * x + y * y + z * z;

        if (best == nullptr || dist_sq < best_dist_sq)
        {
            best = &rec;
            best_dist_sq = dist_sq;
        }
    }

    if (best == nullptr)
    {
        std::cout << "[hand_cam] 无有效目标\n";
        return;
    }

    const std::array<double, 16> pose_robot =
        transform_pose_cam_to_robot(cam2robot, best->target.pose_4x4);
    out_pose = T2PosEulerAngles(row_major_pose_to_matrix4(pose_robot));

    const auto &t = best->target;
    const std::string name =
        t.class_name.empty() ? ("cls" + std::to_string(t.class_id)) : t.class_name;
    std::cout << "[hand_cam] " << name
              << " conf=" << std::fixed << std::setprecision(3) << t.confidence
              << " robot=(" << std::setprecision(4) << out_pose(0) << ","
              << out_pose(1) << "," << out_pose(2) << ","
              << out_pose(3) << "," << out_pose(4) << "," << out_pose(5) << ")\n";
}

PosePipeline::PosePipeline() = default;

PosePipeline::~PosePipeline()
{
    shutdown(false);
}

bool PosePipeline::init(std::string &err)
{
    err.clear();
    root_ = project_root_dir();

    std::cout << "========== 位姿检测 ==========\n";
    std::cout << "工程根: " << root_ << "\n";

    std::cout << "\n[1] 初始化相机 ... ";
    cameras_ = std::make_unique<RealSenseMultiCam>(default_cameras_yaml_path());
    if (!cameras_->init(err))
    {
        std::cout << "失败\n";
        cameras_.reset();
        return false;
    }
    std::cout << "OK\n";

    std::cout << "[2] 初始化算法引擎 ... ";
    bridge_ = std::make_unique<SegPoseBridge>();
    if (!bridge_->init(default_pose_yaml_path(), default_seg_circle_pose_root(), err))
    {
        std::cout << "失败\n";
        cameras_->stop();
        cameras_.reset();
        bridge_.reset();
        return false;
    }
    std::cout << "OK\n";
    install_app_sigint_handler();
    return true;
}

void PosePipeline::shutdown(bool close_visualization)
{
    if (bridge_)
    {
        if (close_visualization)
            bridge_->close_visualization();
        bridge_.reset();
    }
    if (cameras_)
    {
        cameras_->stop();
        cameras_.reset();
    }
}

RealSenseMultiCam &PosePipeline::cameras()
{
    return *cameras_;
}

SegPoseBridge &PosePipeline::bridge()
{
    return *bridge_;
}

void print_pose_run_result(const PoseRunResult &result, const char *slot_label, int algorithm_id)
{
    std::cout << "\n========== " << slot_label << " | 算法 " << algorithm_id << " ==========\n";
    if (!result.ok)
    {
        std::cout << "失败: " << result.message << "\n";
        return;
    }

    if (result.targets.empty())
    {
        std::cout << "未检测到目标\n";
        return;
    }

    for (size_t i = 0; i < result.targets.size(); ++i)
    {
        const auto &t = result.targets[i];
        const double tx = t.pose_4x4[3];
        const double ty = t.pose_4x4[7];
        const double tz = t.pose_4x4[11];
        const std::string status = t.success ? "OK" : ("FAIL(" + t.message + ")");
        const std::string name = t.class_name.empty() ? ("cls" + std::to_string(t.class_id)) : t.class_name;

        std::cout << "[" << i << "] " << name
                  << " algo=" << t.algorithm_id
                  << " conf=" << std::fixed << std::setprecision(3) << t.confidence
                  << " r=" << std::setprecision(4) << t.radius_m << "m "
                  << status
                  << " pos_m=(" << std::setprecision(4) << tx << "," << ty << "," << tz << ")\n";

        std::cout << "pose_4x4 (row-major, m):\n";
        for (int r = 0; r < 4; ++r)
        {
            std::cout << "  ";
            for (int c = 0; c < 4; ++c)
                std::cout << std::setw(11) << std::setprecision(6) << t.pose_4x4[static_cast<size_t>(r * 4 + c)] << " ";
            std::cout << "\n";
        }
    }
}

int algorithm_id_for_slot(CameraSlot slot)
{
    return (slot == CameraSlot::Head) ? 1 : 0;
}

PoseDetectionRecords detect_pose_at_slot(
    RealSenseMultiCam &cameras,
    SegPoseBridge &bridge,
    CameraSlot slot,
    bool show_visualization)
{
    if (app_stop_requested())
        return {};

    const char *slot_label = RealSenseMultiCam::slot_name(slot);
    const int algorithm_id = algorithm_id_for_slot(slot);

    CameraFrameData frame = cameras.grab(slot);
    if (!frame.ok)
    {
        if (!app_stop_requested())
            std::cerr << "[" << slot_label << "] 取帧失败: " << frame.message << std::endl;
        return {};
    }

    frame = RealSenseMultiCam::prepare_frame_for_slot(std::move(frame), slot);

    const PoseRunResult result = bridge.run(frame, algorithm_id, show_visualization, slot);
    if (!result.ok)
    {
        if (!app_stop_requested())
            std::cerr << "[" << slot_label << "] 算法失败: " << result.message << std::endl;
        return {};
    }

    PoseDetectionRecords records;
    append_pose_records(records, slot_label, algorithm_id, result);
    if (records.empty())
        std::cout << "[" << slot_label << "] 未检测到目标\n";

    return records;
}

void append_pose_records(
    PoseDetectionRecords &records,
    const char *slot_name,
    int frame_algorithm_id,
    const PoseRunResult &result)
{
    if (!result.ok)
        return;

    for (const PoseTargetResult &t : result.targets)
    {
        PoseDetectionRecord rec;
        rec.slot_name = slot_name ? slot_name : "";
        rec.frame_algorithm_id = frame_algorithm_id;
        rec.target = t;
        records.push_back(std::move(rec));
    }
}

void print_pose_records_summary(const char *slot_title, const PoseDetectionRecords &records)
{
    std::cout << "\n--- " << slot_title << " (" << records.size() << " 条) ---\n";
    if (records.empty())
    {
        std::cout << "(无)\n";
        return;
    }

    for (size_t i = 0; i < records.size(); ++i)
    {
        const auto &rec = records[i];
        const auto &t = rec.target;
        const double tx = t.pose_4x4[3];
        const double ty = t.pose_4x4[7];
        const double tz = t.pose_4x4[11];
        const std::string name = t.class_name.empty() ? ("cls" + std::to_string(t.class_id)) : t.class_name;
        const std::string status = t.success ? "OK" : ("FAIL(" + t.message + ")");

        std::cout << "[" << i << "] " << name
                  << " cls=" << t.class_id
                  << " algo=" << t.algorithm_id
                  << " frame_algo=" << rec.frame_algorithm_id
                  << " conf=" << std::fixed << std::setprecision(3) << t.confidence
                  << " r=" << std::setprecision(4) << t.radius_m << "m "
                  << status
                  << " pos_m=(" << std::setprecision(4) << tx << "," << ty << "," << tz << ")\n";

        std::cout << "  pose_4x4:\n";
        for (int r = 0; r < 4; ++r)
        {
            std::cout << "    ";
            for (int c = 0; c < 4; ++c)
                std::cout << std::setw(11) << std::setprecision(6)
                          << t.pose_4x4[static_cast<size_t>(r * 4 + c)] << " ";
            std::cout << "\n";
        }
    }
}

// =============================================================================
// END: src/seg_pose_bridge.cpp
// =============================================================================

// =============================================================================
// BEGIN: src/main3.cpp
// =============================================================================
#include "realsense_get.h"
#include "seg_pose_bridge.h"

#include <iostream>
#include <string>
#include <thread>

#include "head.h"
#include "gripper_interface.hpp"
#include "wt_box_tcp.h"
#include "reals_tcp.h"
#include "waist.h"
#include "Ti5_socketcan.h"
#include "Ti5_Arm.h"
#include "wt_client.h"
#include "lanxincontrol.h"
#include "tts_http_client.h"
#ifdef USE_TI5_BODY_CAN_DRIVER
#include "ti5_can_bridge.h"
#endif

#include <array>
#include <chrono>
#include <cerrno>
#include <cstdint>
#include <fstream>
#include <functional>
#include <random>
#include <unordered_map>
#include <vector>

// true=OpenCV 可视化调试；false=仅终端打印
constexpr bool kDebugVisualize = true;

namespace
{

constexpr double kHeadXStaggerThreshold = 0.52;
constexpr double kWaistLayer3XHome = -0.132502;
constexpr double kWaistStepX = 0.06;
constexpr int kWaistAdvanceMaxSteps = 2;
constexpr double kHandDescendZ = 0.04;       // 手相机精定位后下压量
constexpr double kLiftAfterGraspZ = 0.05;    // 夹取后抬起：原 0.04 + 1cm

bool goal_z_in_range(double z)
{
    return z >= -0.50 && z <= -0.15;
}

bool goal_pos_valid_right(double x, double y, double z)
{
    return x >= 0.3 && x <= 0.8 && y >= -0.35 && y <= -0.05 && goal_z_in_range(z);
}

bool goal_pos_valid_left(double x, double y, double z)
{
    return x >= 0.3 && x <= 0.8 && y >= 0.05 && y <= 0.35 && goal_z_in_range(z);
}

bool head_hand_detected(const double hand_pos[3])
{
    return hand_pos[0] >= 0.0;
}

bool head_x_needs_stagger(double x)
{
    return x > kHeadXStaggerThreshold;
}

void fill_goal_last_from_head_assign(
    const double right_hand_pos[3],
    const double left_hand_pos[3],
    Eigen::Matrix<double, 1, 6> &goal_last_r,
    Eigen::Matrix<double, 1, 6> &goal_last_l)
{
    goal_last_r << right_hand_pos[0], right_hand_pos[1], -0.30, -90 * rad, 45 * rad, 0;
    goal_last_l << left_hand_pos[0], left_hand_pos[1], -0.30, 90 * rad, 45 * rad, 0;
    goal_last_r(0) += 0.015;
    goal_last_l(0) += 0.015;
    goal_last_r(2) += 0.065;
    goal_last_l(2) += 0.065;
}

bool head_move_allowed_right(
    const double right_hand_pos[3],
    const Eigen::Matrix<double, 1, 6> &goal_last_r)
{
    return goal_pos_valid_right(goal_last_r(0), goal_last_r(1), right_hand_pos[2]);
}

bool head_move_allowed_left(
    const double left_hand_pos[3],
    const Eigen::Matrix<double, 1, 6> &goal_last_l)
{
    return goal_pos_valid_left(goal_last_l(0), goal_last_l(1), left_hand_pos[2]);
}

void apply_hand_base_to_goal(
    bool have_r,
    const Eigen::Matrix<double, 1, 6> &base_pos_r,
    bool have_l,
    const Eigen::Matrix<double, 1, 6> &base_pos_l,
    Eigen::Matrix<double, 1, 6> &goal_last_r,
    Eigen::Matrix<double, 1, 6> &goal_last_l)
{
    if (have_r)
    {
        goal_last_r(0) = base_pos_r(0);
        goal_last_r(1) = base_pos_r(1);
    }
    if (have_l)
    {
        goal_last_l(0) = base_pos_l(0);
        goal_last_l(1) = base_pos_l(1);
    }
}

void update_hand_goal_xy_from_base(
    const Eigen::Matrix<double, 1, 6> &base_pos,
    Eigen::Matrix<double, 1, 6> &goal_last)
{
    goal_last(0) = base_pos(0);
    goal_last(1) = base_pos(1);
}

bool hand_move_allowed_right(
    const Eigen::Matrix<double, 1, 6> &goal_last_r,
    const Eigen::Matrix<double, 1, 6> &base_pos_r)
{
    return goal_pos_valid_right(goal_last_r(0), goal_last_r(1), base_pos_r(2));
}

bool hand_move_allowed_left(
    const Eigen::Matrix<double, 1, 6> &goal_last_l,
    const Eigen::Matrix<double, 1, 6> &base_pos_l)
{
    return goal_pos_valid_left(goal_last_l(0), goal_last_l(1), base_pos_l(2));
}

void log_arm_skip_right()
{
    cout << "[arm] 右手目标超范围(x:0.3~0.8, y:-0.35~-0.05, z:-0.50~-0.15)，本段不移动\n";
}

void log_arm_skip_left()
{
    cout << "[arm] 左手目标超范围(x:0.3~0.8, y:0.05~0.35, z:-0.50~-0.15)，本段不移动\n";
}

struct HeadAssignState
{
    double right_hand_pos[3] = {-1.0, 0.0, 0.0};
    double left_hand_pos[3] = {-1.0, 0.0, 0.0};
    Eigen::Matrix<double, 1, 6> goal_last_r;
    Eigen::Matrix<double, 1, 6> goal_last_l;
    bool have_r = false;
    bool have_l = false;
    bool move_r = false;
    bool move_l = false;
};

void refresh_head_move_flags(HeadAssignState &st)
{
    st.have_r = head_hand_detected(st.right_hand_pos);
    st.have_l = head_hand_detected(st.left_hand_pos);
    st.move_r = st.have_r && head_move_allowed_right(st.right_hand_pos, st.goal_last_r);
    st.move_l = st.have_l && head_move_allowed_left(st.left_hand_pos, st.goal_last_l);
}

HeadAssignState detect_and_assign_head(
    RealSenseMultiCam &cameras,
    SegPoseBridge &bridge,
    const std::array<double, 16> &cam2robot,
    bool retry_if_both_invalid)
{
    HeadAssignState st;
    PoseDetectionRecords head_records =
        detect_pose_at_slot(cameras, bridge, CameraSlot::Head, kDebugVisualize);
    assign_hand_targets_in_robot_frame(
        head_records, cam2robot, st.right_hand_pos, st.left_hand_pos);
    fill_goal_last_from_head_assign(
        st.right_hand_pos, st.left_hand_pos, st.goal_last_r, st.goal_last_l);
    refresh_head_move_flags(st);

    if (retry_if_both_invalid && !st.move_r && !st.move_l)
    {
        cout << "[head] 双手均无效，头相机重试一次\n";
        head_records =
            detect_pose_at_slot(cameras, bridge, CameraSlot::Head, kDebugVisualize);
        assign_hand_targets_in_robot_frame(
            head_records, cam2robot, st.right_hand_pos, st.left_hand_pos);
        fill_goal_last_from_head_assign(
            st.right_hand_pos, st.left_hand_pos, st.goal_last_r, st.goal_last_l);
        refresh_head_move_flags(st);
    }
    return st;
}

void run_head_approach_selective(
    Robot_Arm &arm_r,
    Robot_Arm &arm_l,
    HeadAssignState &st,
    bool move_r,
    bool move_l)
{
    arm_dual_line_move_selective(
        arm_r, st.goal_last_r, move_r, arm_l, st.goal_last_l, move_l, 0.2);
    st.goal_last_r(2) -= 0.05;
    st.goal_last_l(2) -= 0.05;
    arm_dual_line_move_selective(
        arm_r, st.goal_last_r, move_r, arm_l, st.goal_last_l, move_l, 0.2);
}

struct HandMoveState
{
    bool move_r = false;
    bool move_l = false;
};

HandMoveState run_hand_detect_and_validate(
    Robot_Arm &arm_r,
    Robot_Arm &arm_l,
    RealSenseMultiCam &cameras,
    SegPoseBridge &bridge,
    HeadAssignState &st,
    bool enable_r,
    bool enable_l,
    std::string &err)
{
    HandMoveState hs;
    if (!enable_r && !enable_l)
        return hs;

    // 手相机前：7 轴中至少 6 轴 speed==0，连续判定通过才拍
    auto wait_steady_for_hand_cam = []() -> bool {
        return wait_arms_motors_stopped(true, true, 15.0, 6);
    };

    if (!wait_steady_for_hand_cam())
    {
        log_arms_motor_running_speeds(true, true, "手相机前停稳失败(超时)");
        std::cerr << "[hand] 电机未停稳，跳过手相机拍照\n";
        return hs;
    }
    log_arms_motor_running_speeds(true, true, "手相机拍照前电机速度");
    sleep(1);

    if (enable_r)
        cout << "检测右手" << arm_get_tcp_pos(arm_r) << endl;
    if (enable_l)
        cout << "检测左手" << arm_get_tcp_pos(arm_l) << endl;

    PoseDetectionRecords right_hand_records;
    PoseDetectionRecords left_hand_records;
    if (enable_r)
        right_hand_records =
            detect_pose_at_slot(cameras, bridge, CameraSlot::RightHand, kDebugVisualize);
    if (enable_l)
        left_hand_records =
            detect_pose_at_slot(cameras, bridge, CameraSlot::LeftHand, kDebugVisualize);

    std::array<double, 16> cam2robot_r{};
    std::array<double, 16> cam2robot_l{};
    Eigen::Matrix<double, 1, 6> hand_cam_r, hand_cam_l;
    hand_cam_r << -1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    hand_cam_l << -1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    Eigen::Matrix<double, 1, 6> base_pos_r, base_pos_l;

    const bool have_cam_r =
        !enable_r ||
        load_cam2robot_matrix(default_right_hand_to_robot_yaml_path(), cam2robot_r, err);
    if (enable_r && !have_cam_r)
        std::cerr << "读取右手 cam2robot 失败: " << err << std::endl;
    else if (enable_r)
    {
        assign_nearest_hand_cam_target_in_robot_frame(
            right_hand_records, cam2robot_r, hand_cam_r);
        base_pos_r = arm_get_base_visual_pos(arm_r, hand_cam_r);
    }

    const bool have_cam_l =
        !enable_l ||
        load_cam2robot_matrix(default_left_hand_to_robot_yaml_path(), cam2robot_l, err);
    if (enable_l && !have_cam_l)
        std::cerr << "读取左手 cam2robot 失败: " << err << std::endl;
    else if (enable_l)
    {
        assign_nearest_hand_cam_target_in_robot_frame(
            left_hand_records, cam2robot_l, hand_cam_l);
        base_pos_l = arm_get_base_visual_pos(arm_l, hand_cam_l);
    }

    apply_hand_base_to_goal(
        enable_r && have_cam_r,
        base_pos_r,
        enable_l && have_cam_l,
        base_pos_l,
        st.goal_last_r,
        st.goal_last_l);

    hs.move_r = enable_r && have_cam_r && hand_move_allowed_right(st.goal_last_r, base_pos_r);
    hs.move_l = enable_l && have_cam_l && hand_move_allowed_left(st.goal_last_l, base_pos_l);

    if (!hs.move_r && enable_r && have_cam_r)
    {
        if (!wait_steady_for_hand_cam())
        {
            std::cerr << "[hand] 重试前电机未停稳，跳过右手手相机\n";
            return hs;
        }
        sleep(1);
        cout << "[hand] 右手无效，右手相机重试一次\n";
        right_hand_records =
            detect_pose_at_slot(cameras, bridge, CameraSlot::RightHand, kDebugVisualize);
        assign_nearest_hand_cam_target_in_robot_frame(
            right_hand_records, cam2robot_r, hand_cam_r);
        base_pos_r = arm_get_base_visual_pos(arm_r, hand_cam_r);
        update_hand_goal_xy_from_base(base_pos_r, st.goal_last_r);
        hs.move_r = hand_move_allowed_right(st.goal_last_r, base_pos_r);
    }

    if (!hs.move_l && enable_l && have_cam_l)
    {
        if (!wait_steady_for_hand_cam())
        {
            std::cerr << "[hand] 重试前电机未停稳，跳过左手手相机\n";
            return hs;
        }
        sleep(1);
        cout << "[hand] 左手无效，左手相机重试一次\n";
        left_hand_records =
            detect_pose_at_slot(cameras, bridge, CameraSlot::LeftHand, kDebugVisualize);
        assign_nearest_hand_cam_target_in_robot_frame(
            left_hand_records, cam2robot_l, hand_cam_l);
        base_pos_l = arm_get_base_visual_pos(arm_l, hand_cam_l);
        update_hand_goal_xy_from_base(base_pos_l, st.goal_last_l);
        hs.move_l = hand_move_allowed_left(st.goal_last_l, base_pos_l);
    }

    if (enable_r)
    {
        cout << "hand_cam_r: " << hand_cam_r << endl;
        cout << "base_pos_r: " << base_pos_r << endl;
    }
    if (enable_l)
    {
        cout << "hand_cam_l: " << hand_cam_l << endl;
        cout << "base_pos_l: " << base_pos_l << endl;
    }

    return hs;
}

void run_hand_approach_and_grasp(
    gripper::Gripper &g,
    Robot_Arm &arm_r,
    Robot_Arm &arm_l,
    HeadAssignState &st,
    const HandMoveState &hs)
{
    // 1) 手相机只更新了 xy，先平移到 xy（z 不变）
    arm_dual_line_move_selective(
        arm_r, st.goal_last_r, hs.move_r, arm_l, st.goal_last_l, hs.move_l, 0.2);
    sleep(1);

    // 2) 再下压 z，然后夹取
    if (hs.move_r)
        st.goal_last_r(2) -= kHandDescendZ;
    if (hs.move_l)
        st.goal_last_l(2) -= kHandDescendZ;
    arm_dual_line_move_selective(
        arm_r, st.goal_last_r, hs.move_r, arm_l, st.goal_last_l, hs.move_l, 0.2);
    sleep(1);

    if (hs.move_r)
    {
        const auto grasp_r = g.graspAndWait(gripper::Side::Right);
        if (grasp_r.result != gripper::GraspResult::TorqueLimit &&
            grasp_r.result != gripper::GraspResult::PositionReached)
        {
            std::cerr << "[gripper] 右手夹取异常 result=" << static_cast<int>(grasp_r.result)
                      << " q=" << grasp_r.position_rad << "\n";
        }
    }
    if (hs.move_l)
    {
        const auto grasp_l = g.graspAndWait(gripper::Side::Left);
        if (grasp_l.result != gripper::GraspResult::TorqueLimit &&
            grasp_l.result != gripper::GraspResult::PositionReached)
        {
            std::cerr << "[gripper] 左手夹取异常 result=" << static_cast<int>(grasp_l.result)
                      << " q=" << grasp_l.position_rad << "\n";
        }
    }
}

void lift_grasped_selective(
    Robot_Arm &arm_r,
    Robot_Arm &arm_l,
    HeadAssignState &st,
    bool lift_r,
    bool lift_l)
{
    if (!lift_r && !lift_l)
        return;
    if (lift_r)
        st.goal_last_r(2) += kLiftAfterGraspZ;
    if (lift_l)
        st.goal_last_l(2) += kLiftAfterGraspZ;
    arm_dual_line_move_selective(
        arm_r, st.goal_last_r, lift_r, arm_l, st.goal_last_l, lift_l, 0.2);
    sleep(1);
}

bool advance_waist_until_side_x_ok(
    WaistRobot &waist,
    Eigen::Matrix<double, 1, 6> &posup_down_3_layer,
    RealSenseMultiCam &cameras,
    SegPoseBridge &bridge,
    const std::array<double, 16> &cam2robot,
    bool wait_right,
    HeadAssignState &st)
{
    for (int step = 0; step < kWaistAdvanceMaxSteps; ++step)
    {
        posup_down_3_layer(0) -= kWaistStepX;
        cout << "[waist] x 前进 step=" << (step + 1)
             << " pos_x=" << posup_down_3_layer(0) << endl;
        waist.moveLToPos(posup_down_3_layer, 0.1);
        sleep(2); // 腰进后纯等待，不做任何操作（不拍头/手相机）

        st = detect_and_assign_head(cameras, bridge, cam2robot, false);
        cout << "goal_last_r: " << st.goal_last_r << endl;
        cout << "goal_last_l: " << st.goal_last_l << endl;

        if (wait_right)
        {
            if (!st.have_r || !st.move_r)
                return false;
            if (!head_x_needs_stagger(st.goal_last_r(0)))
                return true;
        }
        else
        {
            if (!st.have_l || !st.move_l)
                return false;
            if (!head_x_needs_stagger(st.goal_last_l(0)))
                return true;
        }
    }
    return false;
}

void restore_waist_layer3_x(WaistRobot &waist, Eigen::Matrix<double, 1, 6> &posup_down_3_layer)
{
    posup_down_3_layer(0) = kWaistLayer3XHome;
    cout << "[waist] 恢复 layer3 x=" << kWaistLayer3XHome << endl;
    waist.moveLToPos(posup_down_3_layer, 0.1);
}

} // namespace









int main()
{
    install_app_sigint_handler();

    gripper::Config cfg;
    cfg.grasp_torque_limit_nm = 4.5;
    cfg.soft_torque_limit_nm = 4.0;
    cfg.pos_filter = 0.5;
    cfg.soft_slew_rate = 100.0;
    cfg.soft_coast_margin_rad = 0.12;

    gripper::Gripper g(cfg);
    if (!g.start())
    {
        std::cerr << "夹爪启动失败\n";
        return 1;
    }

    PosePipeline pipeline;
    std::string err;
    if (!pipeline.init(err))
    {
        std::cerr << "初始化失败: " << err << std::endl;
        return 1;
    }

    RealSenseMultiCam &cameras = pipeline.cameras();
    SegPoseBridge &bridge = pipeline.bridge();

    // ---------- 腰部 / 待机位姿（与现场标定一致）----------
    Eigen::Matrix<double, 1, 6> posup_down_1_layer, posup_down_2_layer, posup_down_3_layer, yao_pos,
        put_down;

    posup_down_1_layer << -0.136861, 1.17673e-17, 0.202175, 1.5708, -1.57022, 1.06446e-13;
    posup_down_2_layer << -0.13417, 2.41252e-17, 0.3393994, -1.5708, -1.56917, 3.14159;
    posup_down_3_layer << -0.132502, 4.36259e-17, 0.622464, -1.5708, -1.56703, 3.14159;
    yao_pos << -0.199171, 3.10263e-17, 0.506697, 1.5708, -1.51744, 1.14658e-15;
    put_down << -0.199171, 3.10263e-17, 0.506697, 1.5708, -1.51744, 1.14658e-15;

    init_socketcan();
    hand_socketid_bind();

    {
        int waist_p[1] = {30 * 65536 * 4 / 360};
        uint32_t waist_canID[] = {31};
        socketcan_sendcommand(head_id, 1, waist_canID, 30, waist_p);
    }

    WaistRobot Ti5_waist;
    const std::string arm_yaml = project_root_dir() + "/config/Robot_Arm_Model.yaml";
    Robot_Arm Taihu_r("T7", "T170", "right", arm_yaml);
    Robot_Arm Taihu_l("T7", "T170", "left", arm_yaml);




    double waist_angles[1] = {0.0};
    uint32_t waist_canID[] = {1};
    smooth_motor_move_deg(waist_id, 1, waist_canID, waist_angles, 30.0);
    sleep(1);
    rev_motor_error(0);
    rev_motor_error(1);

    // sleep(8);

    // cout << Taihu_r.getJointPos() << endl;
    //  cout << Taihu_l.getJointPos() << endl;

    //  return 0;

    motor_speed_change();

    sleep(1);

    //  set_motor_current_zero();

    
 
    

    Ti5_waist.moveLToPos(posup_down_3_layer, 0.1);

    Matrix<double, 1, 6> posr, posl, goal_last, goal_last_copy;

    posr << 0.45, -0.36, -0.15, -90 * rad,0, 0;
    posl << 0.45, 0.36, -0.15, 90 * rad, 0, 0;

    {
        // 上电初始化：双臂先到待机位
        //start_pos(Taihu_r, Taihu_l);
        arm_dual_line_move(Taihu_r, posr, Taihu_l, posl, 0.2);
        
        sleep(1);
    }

    while (true) // 搬箱主循环：待机 → 识别 → 抓取 → （暂）抬起调试
    {
        // 每轮开始：双臂回待机位，转腰到识别姿态
        arm_dual_line_move(Taihu_r, posr, Taihu_l, posl, 0.2);

        double waist_angles[1] = {0.0};
        uint32_t waist_canID[] = {1};
        smooth_motor_move_deg(waist_id, 1, waist_canID, waist_angles, 30.0); // 转腰到 -90°，方便头相机看货箱


        std::array<double, 16> cam2robot{};
        if (!load_cam2robot_matrix(default_camera_to_robot_yaml_path(), cam2robot, err)) // 读头相机→基座标定矩阵
        {
            std::cerr << "读取 cam2robot 失败: " << err << std::endl;
        }
        else // 头相机标定加载成功，进入本轮抓取流程
        {

            // 开夹爪，头相机识别并分配左右手目标
            g.openAndWait(gripper::Side::Right);
            g.openAndWait(gripper::Side::Left);

            HeadAssignState st =
                detect_and_assign_head(cameras, bridge, cam2robot, true); // 拍头相机 + YOLO + 左右手目标分配（失败会重试一次）

            cout << "goal_last_r: " << st.goal_last_r << endl;
            cout << "goal_last_l: " << st.goal_last_l << endl;

            // 头部分配结果检查：未识别/超范围则跳过该侧；双手都无效则整轮跳过
            if (!st.have_r)
                cout << "[head] 未识别到右手，跳过右手手相机/抓取\n";
            if (!st.have_l)
                cout << "[head] 未识别到左手，跳过左手手相机/抓取\n";
            if (!st.move_r && st.have_r)
                log_arm_skip_right();
            if (!st.move_l && st.have_l)
                log_arm_skip_left();
            if (!st.move_r && !st.move_l)
                continue;

            // 头部分配且 xy/z 合格的手才参与手相机/抓取；stagger 表示该侧 x 太远需分侧或腰前进
            const bool enable_r_hand = st.move_r;
            const bool enable_l_hand = st.move_l;

            const bool stagger_r = st.move_r && head_x_needs_stagger(st.goal_last_r(0));
            const bool stagger_l = st.move_l && head_x_needs_stagger(st.goal_last_l(0));

            bool waist_adjusted = false; // 本轮是否动过腰，结束时要 restore
            bool grasped_r = false;
            bool grasped_l = false;
            bool lifted_r = false;
            bool lifted_l = false;

            auto finish_lift_temp = [&]() { // 暂时代码：抬起已抓侧 → 松爪 → 恢复腰 → 等键盘继续
                if (grasped_r && !lifted_r)
                    st.goal_last_r(2) += kLiftAfterGraspZ;
                if (grasped_l && !lifted_l)
                    st.goal_last_l(2) += kLiftAfterGraspZ;
                arm_dual_line_move_selective(
                    Taihu_r, st.goal_last_r, grasped_r, Taihu_l, st.goal_last_l, grasped_l, 0.2);
                sleep(1);
                if (grasped_r)
                    g.openAndWait(gripper::Side::Right);
                if (grasped_l)
                    g.openAndWait(gripper::Side::Left);
                if (waist_adjusted)
                    restore_waist_layer3_x(Ti5_waist, posup_down_3_layer);
                int duan;
                cin >> duan; // 调试暂停：输入任意数回车后再进下一轮
            };

            if (stagger_r && !stagger_l && st.move_l) // 右手 x>0.52：先抓左手，腰前进后再抓右手
            {
                cout << "[stagger] 右手 x>" << kHeadXStaggerThreshold
                     << "，先执行左手\n";
                run_head_approach_selective(Taihu_r, Taihu_l, st, false, true); // 头相机粗定位：只动左手
                HandMoveState hs_l = run_hand_detect_and_validate( // 左手相机精定位 + 有效性校验
                    Taihu_r, Taihu_l, cameras, bridge, st, false, enable_l_hand, err);
                if (!hs_l.move_l)
                {
                    log_arm_skip_left();
                    continue;
                }
                run_hand_approach_and_grasp(g, Taihu_r, Taihu_l, st, hs_l); // 左手下压 + 夹取
                grasped_l = true;
                lift_grasped_selective(Taihu_r, Taihu_l, st, false, true); // 先抓侧立即抬起，避免腰进推零件
                lifted_l = true;

                cout << "[stagger] 腰前进直到右手 x<=" << kHeadXStaggerThreshold << endl;
                waist_adjusted = true;
                if (!advance_waist_until_side_x_ok( // 腰 -=0.06 前进，重拍头部直到右手 x 够近
                        Ti5_waist, posup_down_3_layer, cameras, bridge, cam2robot, true, st))
                {
                    cout << "[stagger] 腰前进后右手仍不可用，跳过本轮\n";
                    restore_waist_layer3_x(Ti5_waist, posup_down_3_layer);
                    continue;
                }

                // 腰进完成：头相机重新识别，只为右手分配目标；左手已抓不再参与
                st = detect_and_assign_head(cameras, bridge, cam2robot, false);
                st.move_l = false;
                cout << "[stagger] 腰进后头部分配右手 goal_last_r: " << st.goal_last_r << endl;
                if (!st.move_r)
                {
                    log_arm_skip_right();
                    restore_waist_layer3_x(Ti5_waist, posup_down_3_layer);
                    continue;
                }

                cout << "[stagger] 执行右手（左手已抓，不再分配左手任务）\n";
                run_head_approach_selective(Taihu_r, Taihu_l, st, true, false); // 只动右手
                HandMoveState hs_r = run_hand_detect_and_validate(
                    Taihu_r, Taihu_l, cameras, bridge, st, true, false, err); // 只拍右手相机
                if (!hs_r.move_r)
                {
                    log_arm_skip_right();
                    restore_waist_layer3_x(Ti5_waist, posup_down_3_layer);
                    continue;
                }
                run_hand_approach_and_grasp(g, Taihu_r, Taihu_l, st, hs_r);
                grasped_r = true;
                finish_lift_temp();
                continue;
            }

            if (stagger_l && !stagger_r && st.move_r) // 左手 x>0.52：先抓右手，腰前进后再抓左手
            {
                cout << "[stagger] 左手 x>" << kHeadXStaggerThreshold
                     << "，先执行右手\n";
                run_head_approach_selective(Taihu_r, Taihu_l, st, true, false); // 头相机粗定位：只动右手
                HandMoveState hs_r = run_hand_detect_and_validate(
                    Taihu_r, Taihu_l, cameras, bridge, st, enable_r_hand, false, err);
                if (!hs_r.move_r)
                {
                    log_arm_skip_right();
                    continue;
                }
                run_hand_approach_and_grasp(g, Taihu_r, Taihu_l, st, hs_r);
                grasped_r = true;
                lift_grasped_selective(Taihu_r, Taihu_l, st, true, false); // 先抓侧立即抬起，避免腰进推零件
                lifted_r = true;

                cout << "[stagger] 腰前进直到左手 x<=" << kHeadXStaggerThreshold << endl;
                waist_adjusted = true;
                if (!advance_waist_until_side_x_ok( // 腰前进，重拍头部直到左手 x 够近
                        Ti5_waist, posup_down_3_layer, cameras, bridge, cam2robot, false, st))
                {
                    cout << "[stagger] 腰前进后左手仍不可用，跳过本轮\n";
                    restore_waist_layer3_x(Ti5_waist, posup_down_3_layer);
                    continue;
                }

                // 腰进完成：头相机重新识别，只为左手分配目标；右手已抓不再参与
                st = detect_and_assign_head(cameras, bridge, cam2robot, false);
                st.move_r = false;
                cout << "[stagger] 腰进后头部分配左手 goal_last_l: " << st.goal_last_l << endl;
                if (!st.move_l)
                {
                    log_arm_skip_left();
                    restore_waist_layer3_x(Ti5_waist, posup_down_3_layer);
                    continue;
                }

                cout << "[stagger] 执行左手（右手已抓，不再分配右手任务）\n";
                run_head_approach_selective(Taihu_r, Taihu_l, st, false, true);
                HandMoveState hs_l = run_hand_detect_and_validate(
                    Taihu_r, Taihu_l, cameras, bridge, st, false, true, err);
                if (!hs_l.move_l)
                {
                    log_arm_skip_left();
                    restore_waist_layer3_x(Ti5_waist, posup_down_3_layer);
                    continue;
                }
                run_hand_approach_and_grasp(g, Taihu_r, Taihu_l, st, hs_l);
                grasped_l = true;
                finish_lift_temp();
                continue;
            }

            if (stagger_r && stagger_l) // 双手 x 都>0.52：腰前进（最多2次）直到 x 回到范围内
            {
                cout << "[stagger] 双手 x>" << kHeadXStaggerThreshold << "，腰前进\n";
                waist_adjusted = true;
                for (int step = 0; step < kWaistAdvanceMaxSteps; ++step)
                {
                    posup_down_3_layer(0) -= kWaistStepX;
                    cout << "[waist] x 前进 step=" << (step + 1)
                         << " pos_x=" << posup_down_3_layer(0) << endl;
                    Ti5_waist.moveLToPos(posup_down_3_layer, 0.1);
                    sleep(2); // 腰进后纯等待，不做任何操作（不拍头/手相机）
                    st = detect_and_assign_head(cameras, bridge, cam2robot, false); // 腰动完重拍头部更新目标
                    refresh_head_move_flags(st);
                    cout << "goal_last_r: " << st.goal_last_r << endl;
                    cout << "goal_last_l: " << st.goal_last_l << endl;
                    if (!st.move_r && !st.move_l)
                        break;
                    // 有效侧 x 都回到 0.52 以内即可退出腰前进循环
                    const bool ok_r = !st.move_r || !head_x_needs_stagger(st.goal_last_r(0));
                    const bool ok_l = !st.move_l || !head_x_needs_stagger(st.goal_last_l(0));
                    if (ok_r && ok_l)
                        break;
                }
                if (!st.move_r && !st.move_l)
                {
                    restore_waist_layer3_x(Ti5_waist, posup_down_3_layer);
                    continue;
                }
            }

            if (stagger_r && !stagger_l && !st.move_l && st.move_r) // 仅右手有效且 x>0.52：腰前进后只抓右手
            {
                cout << "[stagger] 仅右手有效且 x>" << kHeadXStaggerThreshold
                     << "，腰前进后执行右手\n";
                waist_adjusted = true;
                if (!advance_waist_until_side_x_ok(
                        Ti5_waist, posup_down_3_layer, cameras, bridge, cam2robot, true, st))
                {
                    restore_waist_layer3_x(Ti5_waist, posup_down_3_layer);
                    continue;
                }
                st.move_l = false; // 后面正常流程不再尝试左手
            }
            else if (stagger_l && !stagger_r && !st.move_r && st.move_l) // 仅左手有效且 x>0.52：腰前进后只抓左手
            {
                cout << "[stagger] 仅左手有效且 x>" << kHeadXStaggerThreshold
                     << "，腰前进后执行左手\n";
                waist_adjusted = true;
                if (!advance_waist_until_side_x_ok(
                        Ti5_waist, posup_down_3_layer, cameras, bridge, cam2robot, false, st))
                {
                    restore_waist_layer3_x(Ti5_waist, posup_down_3_layer);
                    continue;
                }
                st.move_r = false; // 后面正常流程不再尝试右手
            }

            // 正常双臂流程：头相机粗定位（两段下压）
            run_head_approach_selective(
                Taihu_r, Taihu_l, st, st.move_r, st.move_l);

            // 手相机精定位：拍/hand 检测、换算 base 坐标、写回 goal_last；未识别侧不拍
            HandMoveState hs = run_hand_detect_and_validate(
                Taihu_r,
                Taihu_l,
                cameras,
                bridge,
                st,
                enable_r_hand,
                enable_l_hand,
                err);

            if (!hs.move_r && enable_r_hand)
                log_arm_skip_right();
            if (!hs.move_l && enable_l_hand)
                log_arm_skip_left();
            if (!hs.move_r && !hs.move_l) // 手相机阶段双手均失败，跳过本轮
            {
                if (waist_adjusted)
                    restore_waist_layer3_x(Ti5_waist, posup_down_3_layer);
                continue;
            }

            hs.move_r = hs.move_r && enable_r_hand;
            hs.move_l = hs.move_l && enable_l_hand;
            run_hand_approach_and_grasp(g, Taihu_r, Taihu_l, st, hs); // 手相机目标下压 + 力控夹取
            grasped_r = hs.move_r;
            grasped_l = hs.move_l;

            {
                // 暂时代码：本轮到此结束，不执行下方放箱流程
                finish_lift_temp();
                continue;
            }

            // 放箱流程（暂时代码 continue 后不可达）：回位 → 转腰 → 下放 → 松爪 → 回待机
            arm_dual_line_move(Taihu_r, posr, Taihu_l, posl, 0.2);

            double waist_angles[1] = {0.0};
            uint32_t waist_canID[] = {1};
            smooth_motor_move_deg(waist_id, 1, waist_canID, waist_angles, 30.0); // 转腰回 0°

            Matrix<double, 1, 6> posr_down, posl_down;

            posr_down << 0.48, -0.26, -0.25, 0, 30 * rad, 0;
            posl_down << 0.48, 0.26, -0.25, 0, 30 * rad, 0;

            arm_dual_line_move(Taihu_r, posr_down, Taihu_l, posl_down, 0.2); // 双臂下放到放货位

            g.openAndWait(gripper::Side::Right);
            g.openAndWait(gripper::Side::Left);

            arm_dual_line_move(Taihu_r, posr, Taihu_l, posl, 0.2); // 回待机位


            
        }

        sleep(2); // 每轮间隔
    }

    std::cout << "\n已退出\n";
    g.stop();
    pipeline.shutdown(kDebugVisualize);
    return 0;
}

// =============================================================================
// END: src/main3.cpp
// =============================================================================
