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