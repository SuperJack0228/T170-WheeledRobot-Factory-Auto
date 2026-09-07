#include "shopclient.h"
#include <string>
#include <map>
#include <cctype>
#include <regex>

#ifdef _WIN32
#include <windows.h>
#endif

int shop_connect()
{
#ifdef _WIN32
    // 设置控制台编码为UTF-8
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);

    // Windows系统需要初始化Winsock
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
    {
        std::cerr << "WSAStartup failed" << std::endl;
        return -1;
    }
#endif
    int clientSocket;
    struct sockaddr_in serverAddr;

    // 创建socket
    clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket == -1)
    {
        std::cerr << "Socket creation failed" << std::endl;
#ifdef _WIN32
        WSACleanup();
#endif
        return -1;
    }

    // 设置服务器地址
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(SERVER_PORT);

    // 将IP地址从字符串转换为网络字节序
    if (inet_pton(AF_INET, SERVER_IP, &serverAddr.sin_addr) <= 0)
    {
        std::cerr << "Invalid address or address not supported" << std::endl;
        close(clientSocket);
#ifdef _WIN32
        WSACleanup();
#endif
        return -1;
    }

    // 连接到服务器
    std::cout << "🔗 正在连接到Python服务器 " << SERVER_IP << ":" << SERVER_PORT << "..." << std::endl;

    if (connect(clientSocket, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
    {
        std::cerr << "连接失败" << std::endl;
        close(clientSocket);
#ifdef _WIN32
        WSACleanup();
#endif
        return -1;
    }

    std::cout << "✅ 成功连接到Python服务器!" << std::endl;

    return clientSocket;
}

std::string extractFirstName(const std::string &message)
{
    try
    {
        std::cout << "收到消息: " << message << std::endl;
        
        // 查找"name"字段
        std::string target = "\"name\":";
        size_t pos = message.find(target);
        
        if (pos == std::string::npos)
        {
            // 尝试查找'name'（没有引号）
            target = "name:";
            pos = message.find(target);
        }
        
        if (pos == std::string::npos)
        {
            std::cout << "找不到name字段" << std::endl;
            return "";
        }
        
        // 跳过"name":
        pos += target.length();
        
        // 跳过空格
        while (pos < message.length() && std::isspace(message[pos]))
        {
            pos++;
        }
        
        if (pos >= message.length())
        {
            std::cout << "超出字符串范围" << std::endl;
            return "";
        }
        
        // 检查值开始字符
        char start_char = message[pos];
        std::string result;
        
        if (start_char == '"')  // 带引号的字符串
        {
            pos++;  // 跳过开头的引号
            size_t end_pos = message.find('"', pos);
            if (end_pos == std::string::npos)
            {
                std::cout << "找不到结束引号" << std::endl;
                return "";
            }
            result = message.substr(pos, end_pos - pos);
        }
        else  // 可能是没有引号的字符串
        {
            // 找到下一个逗号、空格或结束大括号
            size_t end_pos = pos;
            while (end_pos < message.length() && 
                   message[end_pos] != ',' && 
                   message[end_pos] != '}' && 
                   message[end_pos] != ' ' && 
                   message[end_pos] != '\t' &&
                   message[end_pos] != '\n' &&
                   message[end_pos] != '\r')
            {
                end_pos++;
            }
            result = message.substr(pos, end_pos - pos);
        }
        
        // 清理结果（移除可能的空格）
        result.erase(0, result.find_first_not_of(" \t\n\r"));
        result.erase(result.find_last_not_of(" \t\n\r") + 1);
        
        std::cout << "提取到的名称: " << result << std::endl;
        return result;
    }
    catch (const std::exception &e)
    {
        std::cerr << "解析错误: " << e.what() << std::endl;
        return "";
    }
}


std::string convertToPinyin(const std::string &target, int &extractedNumber)
{
    // 初始化提取的数字为0
    extractedNumber = 0;
    
    // 创建中文到拼音的映射表
    static std::map<std::string, std::string> nameMap = {
        {"梦幻极光纯钛磁吸焖泡银杯","vacuum_bottle_silvery"},
        {"梦幻极光纯钛磁吸焖泡蓝杯","vacuum_bottle_medium_blue"},
        {"梦幻极光纯钛磁吸焖泡红杯","vacuum_bottle_pinkish_yellow"},
        {"繁花轻语纯钛茶水分离杯","vacuum_bottle_golden_lid"},
        {"繁花轻语纯钛随行杯","vacuum_bottle_white_lid"},
        {"pan_middele_blue","pan_middele_blue"}


         


       
        
        };

    // 方法1: 使用正则表达式匹配数字
    std::regex pattern("(.*?)(\\d+)$");
    std::smatch matches;
    
    if (std::regex_match(target, matches, pattern) && matches.size() == 3)
    {
        // 获取前面的文本部分
        std::string prefix = matches[1].str();
        std::string numberStr = matches[2].str();
        
        // 尝试查找前缀对应的拼音
        auto it = nameMap.find(prefix);
        if (it != nameMap.end())
        {
            // 转换数字部分
            extractedNumber = std::stoi(numberStr);
            return it->second;
        }
    }
    
    // 方法2: 如果正则匹配失败，尝试从末尾提取数字
    if (matches.size() != 3)
    {
        // 查找第一个数字的位置
        size_t digitPos = target.length();
        for (size_t i = target.length(); i > 0; --i)
        {
            if (std::isdigit(target[i-1]))
            {
                digitPos = i-1;
            }
            else
            {
                break;
            }
        }
        
        // 如果找到数字
        if (digitPos < target.length())
        {
            std::string prefix = target.substr(0, digitPos);
            std::string numberStr = target.substr(digitPos);
            
            auto it = nameMap.find(prefix);
            if (it != nameMap.end())
            {
                extractedNumber = std::stoi(numberStr);
                return it->second;
            }
        }
    }
    
    // 如果没有找到前缀匹配，尝试整个字符串匹配
    auto it = nameMap.find(target);
    if (it != nameMap.end())
    {
        return it->second;
    }

    // 如果没有找到任何匹配，返回原字符串
    return target;
}