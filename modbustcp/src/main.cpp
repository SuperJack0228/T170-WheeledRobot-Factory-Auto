#include "modbus_tcp.h"

#include <iomanip>
#include <iostream>
#include <string>

namespace {

constexpr uint16_t kReadAddr = 2100;
constexpr uint16_t kWriteAddr = 2150;
constexpr uint16_t kDefaultPort = 502;
constexpr uint8_t kDefaultUnitId = 1;
constexpr uint16_t kDefaultQuantity = 10;

bool readLine(const std::string& prompt, std::string& out) {
    std::cout << prompt;
    std::cout.flush();
    if (!std::getline(std::cin, out)) {
        return false;
    }
    const auto first = out.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        out.clear();
        return true;
    }
    const auto last = out.find_last_not_of(" \t\r\n");
    out = out.substr(first, last - first + 1);
    return true;
}

bool parseU16(const std::string& s, uint16_t& value, uint16_t def) {
    if (s.empty()) {
        value = def;
        return true;
    }
    try {
        const unsigned long v = std::stoul(s);
        if (v > 65535) {
            return false;
        }
        value = static_cast<uint16_t>(v);
        return true;
    } catch (...) {
        return false;
    }
}

// 支持十进制、0x 十六进制、以及有符号 -32768~32767。
bool parseRegisterValue(const std::string& s, uint16_t& value) {
    if (s.empty()) {
        return false;
    }
    try {
        int base = 10;
        std::string t = s;
        if (t.size() > 2 && t[0] == '0' && (t[1] == 'x' || t[1] == 'X')) {
            base = 16;
        }
        const long v = std::stol(t, nullptr, base);
        if (v < -32768 || v > 65535) {
            return false;
        }
        if (v < 0) {
            value = static_cast<uint16_t>(static_cast<int16_t>(v));
        } else {
            value = static_cast<uint16_t>(v);
        }
        return true;
    } catch (...) {
        return false;
    }
}

void printRegisters(uint16_t start_addr, const std::vector<uint16_t>& values) {
    std::cout << "\n读取成功，共 " << values.size() << " 个寄存器:\n";
    std::cout << "  地址      十进制      十六进制      有符号\n";
    std::cout << "  ---------------------------------------------\n";
    for (size_t i = 0; i < values.size(); ++i) {
        const uint16_t addr = static_cast<uint16_t>(start_addr + i);
        const uint16_t raw = values[i];
        const int16_t signed_v = static_cast<int16_t>(raw);
        std::cout << "  " << std::setw(6) << addr
                  << "  " << std::setw(8) << raw
                  << "      0x" << std::hex << std::uppercase << std::setw(4)
                  << std::setfill('0') << raw << std::dec << std::nouppercase
                  << std::setfill(' ')
                  << "      " << std::setw(6) << signed_v << "\n";
    }
    std::cout << std::endl;
}

void printMenu() {
    std::cout << "\n请选择操作:\n"
              << "  r - 读取寄存器 " << kReadAddr << " 起共 " << kDefaultQuantity << " 个\n"
              << "  w - 向寄存器 " << kWriteAddr << " 写入数值\n"
              << "  q - 退出\n";
}

}  // namespace

int main() {
    std::cout << "======== Modbus TCP 客户端 ========\n";
    std::cout << "读: 保持寄存器 " << kReadAddr << " (功能码 03)\n";
    std::cout << "写: 保持寄存器 " << kWriteAddr << " (功能码 06)\n";
    std::cout << "默认端口 " << kDefaultPort << "，从站号 " << static_cast<int>(kDefaultUnitId)
              << "\n\n";

    std::string ip;
    if (!readLine("请输入设备 IP: ", ip) || ip.empty()) {
        std::cerr << "未输入 IP，退出。\n";
        return 1;
    }

    std::string port_str;
    if (!readLine("请输入端口 [默认 502]: ", port_str)) {
        return 1;
    }
    uint16_t port = kDefaultPort;
    if (!parseU16(port_str, port, kDefaultPort) || port == 0) {
        std::cerr << "端口无效。\n";
        return 1;
    }

    ModbusTcpClient client;
    std::cout << "正在连接 " << ip << ":" << port << " ...\n";
    if (!client.connect(ip, port)) {
        std::cerr << "连接失败: " << client.lastError() << "\n";
        return 1;
    }
    std::cout << "连接成功。\n";

    while (true) {
        printMenu();
        std::string cmd;
        if (!readLine("请输入命令: ", cmd)) {
            break;
        }
        if (cmd.empty()) {
            continue;
        }
        if (cmd == "q" || cmd == "Q") {
            break;
        }

        if (!client.isConnected()) {
            std::cerr << "连接已断开。\n";
            return 1;
        }

        if (cmd == "r" || cmd == "R") {
            std::vector<uint16_t> values;
            if (!client.readHoldingRegisters(kDefaultUnitId, kReadAddr, kDefaultQuantity, values)) {
                std::cerr << "读取失败: " << client.lastError() << "\n";
                if (!client.isConnected()) {
                    return 1;
                }
                continue;
            }
            printRegisters(kReadAddr, values);
            continue;
        }

        if (cmd == "w" || cmd == "W") {
            std::string value_str;
            if (!readLine("请输入要写入寄存器 " + std::to_string(kWriteAddr) +
                              " 的数值 (0~65535，或 -32768~32767，支持 0x 十六进制): ",
                          value_str)) {
                break;
            }
            uint16_t value = 0;
            if (!parseRegisterValue(value_str, value)) {
                std::cerr << "数值无效。\n";
                continue;
            }
            if (!client.writeSingleRegister(kDefaultUnitId, kWriteAddr, value)) {
                std::cerr << "写入失败: " << client.lastError() << "\n";
                if (!client.isConnected()) {
                    return 1;
                }
                continue;
            }
            std::cout << "已向寄存器 " << kWriteAddr << " 写入 "
                      << value << " (0x" << std::hex << std::uppercase << std::setw(4)
                      << std::setfill('0') << value << std::dec << std::nouppercase
                      << std::setfill(' ') << ")\n";
            continue;
        }

        std::cout << "未知命令，请输入 r / w / q。\n";
    }

    client.disconnect();
    std::cout << "已断开连接。\n";
    return 0;
}
