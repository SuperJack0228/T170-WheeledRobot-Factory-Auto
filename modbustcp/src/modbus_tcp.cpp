#include "modbus_tcp.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <sstream>

namespace {

constexpr uint16_t kProtocolId = 0;
constexpr uint8_t kFcReadHolding = 0x03;
constexpr uint8_t kFcWriteSingle = 0x06;
constexpr uint8_t kFcWriteMultiple = 0x10;
constexpr size_t kMbapSize = 7;

void putU16(uint8_t* buf, uint16_t value) {
    buf[0] = static_cast<uint8_t>((value >> 8) & 0xFF);
    buf[1] = static_cast<uint8_t>(value & 0xFF);
}

uint16_t getU16(const uint8_t* buf) {
    return static_cast<uint16_t>((static_cast<uint16_t>(buf[0]) << 8) | buf[1]);
}

std::string errnoMessage() {
    return std::strerror(errno);
}

std::string exceptionText(uint8_t code) {
    switch (code) {
        case 0x01: return "非法功能码";
        case 0x02: return "非法数据地址";
        case 0x03: return "非法数据值";
        case 0x04: return "从站设备故障";
        default: {
            std::ostringstream oss;
            oss << "未知异常码 0x" << std::hex << static_cast<int>(code);
            return oss.str();
        }
    }
}

}  // namespace

ModbusTcpClient::ModbusTcpClient()
    : sock_(-1), transaction_id_(0) {}

ModbusTcpClient::~ModbusTcpClient() {
    disconnect();
}

void ModbusTcpClient::disconnect() {
    if (sock_ >= 0) {
        ::shutdown(sock_, SHUT_RDWR);
        ::close(sock_);
        sock_ = -1;
    }
}

bool ModbusTcpClient::isConnected() const {
    return sock_ >= 0;
}

bool ModbusTcpClient::setSocketTimeout(int timeout_ms) {
    timeval tv{};
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    if (::setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        last_error_ = std::string("设置接收超时失败: ") + errnoMessage();
        return false;
    }
    if (::setsockopt(sock_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) < 0) {
        last_error_ = std::string("设置发送超时失败: ") + errnoMessage();
        return false;
    }
    return true;
}

bool ModbusTcpClient::connect(const std::string& ip, uint16_t port, int timeout_ms) {
    disconnect();

    sock_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock_ < 0) {
        last_error_ = std::string("创建套接字失败: ") + errnoMessage();
        return false;
    }

    int nodelay = 1;
    ::setsockopt(sock_, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

    if (!setSocketTimeout(timeout_ms)) {
        disconnect();
        return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (::inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) {
        last_error_ = "IP 地址格式无效: " + ip;
        disconnect();
        return false;
    }

    if (::connect(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        last_error_ = std::string("连接失败: ") + errnoMessage();
        disconnect();
        return false;
    }

    last_error_.clear();
    return true;
}

bool ModbusTcpClient::sendAll(const uint8_t* data, size_t len) {
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = ::send(sock_, data + sent, len - sent, MSG_NOSIGNAL);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            last_error_ = std::string("发送失败: ") + errnoMessage();
            return false;
        }
        if (n == 0) {
            last_error_ = "连接已关闭";
            return false;
        }
        sent += static_cast<size_t>(n);
    }
    return true;
}

bool ModbusTcpClient::recvAll(uint8_t* data, size_t len) {
    size_t got = 0;
    while (got < len) {
        ssize_t n = ::recv(sock_, data + got, len - got, 0);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                last_error_ = "接收超时";
            } else {
                last_error_ = std::string("接收失败: ") + errnoMessage();
            }
            return false;
        }
        if (n == 0) {
            last_error_ = "对端关闭连接";
            return false;
        }
        got += static_cast<size_t>(n);
    }
    return true;
}

bool ModbusTcpClient::transact(uint8_t unit_id,
                              const std::vector<uint8_t>& req_pdu,
                              std::vector<uint8_t>& rsp_pdu) {
    rsp_pdu.clear();

    if (!isConnected()) {
        last_error_ = "尚未连接";
        return false;
    }
    if (req_pdu.empty() || req_pdu.size() > 252) {
        last_error_ = "请求 PDU 长度非法";
        return false;
    }

    ++transaction_id_;

    std::vector<uint8_t> req(kMbapSize + req_pdu.size());
    putU16(req.data() + 0, transaction_id_);
    putU16(req.data() + 2, kProtocolId);
    putU16(req.data() + 4, static_cast<uint16_t>(1 + req_pdu.size()));
    req[6] = unit_id;
    std::memcpy(req.data() + kMbapSize, req_pdu.data(), req_pdu.size());

    if (!sendAll(req.data(), req.size())) {
        disconnect();
        return false;
    }

    uint8_t mbap[kMbapSize]{};
    if (!recvAll(mbap, sizeof(mbap))) {
        disconnect();
        return false;
    }

    const uint16_t tid = getU16(mbap + 0);
    const uint16_t proto = getU16(mbap + 2);
    const uint16_t length = getU16(mbap + 4);

    if (proto != kProtocolId) {
        last_error_ = "协议标识不是 Modbus";
        disconnect();
        return false;
    }
    if (tid != transaction_id_) {
        last_error_ = "事务标识不匹配";
        disconnect();
        return false;
    }
    if (length < 2 || length > 253) {
        last_error_ = "MBAP 长度字段异常";
        disconnect();
        return false;
    }

    rsp_pdu.resize(length - 1);
    if (!recvAll(rsp_pdu.data(), rsp_pdu.size())) {
        disconnect();
        return false;
    }
    if (rsp_pdu.empty()) {
        last_error_ = "响应 PDU 为空";
        return false;
    }

    if ((rsp_pdu[0] & 0x80) != 0) {
        const uint8_t ex = rsp_pdu.size() > 1 ? rsp_pdu[1] : 0;
        last_error_ = "从站异常: " + exceptionText(ex);
        return false;
    }

    last_error_.clear();
    return true;
}

bool ModbusTcpClient::readHoldingRegisters(uint8_t unit_id,
                                           uint16_t start_addr,
                                           uint16_t quantity,
                                           std::vector<uint16_t>& values) {
    values.clear();

    if (quantity == 0 || quantity > 125) {
        last_error_ = "读取数量必须在 1~125 之间";
        return false;
    }

    std::vector<uint8_t> req(5);
    req[0] = kFcReadHolding;
    putU16(req.data() + 1, start_addr);
    putU16(req.data() + 3, quantity);

    std::vector<uint8_t> rsp;
    if (!transact(unit_id, req, rsp)) {
        return false;
    }
    if (rsp[0] != kFcReadHolding) {
        last_error_ = "功能码与请求不一致";
        return false;
    }
    if (rsp.size() < 2) {
        last_error_ = "响应过短";
        return false;
    }

    const uint8_t byte_count = rsp[1];
    if (byte_count != quantity * 2) {
        last_error_ = "字节计数与请求寄存器数量不符";
        return false;
    }
    if (rsp.size() < static_cast<size_t>(2 + byte_count)) {
        last_error_ = "寄存器数据不完整";
        return false;
    }

    values.resize(quantity);
    for (uint16_t i = 0; i < quantity; ++i) {
        values[i] = getU16(rsp.data() + 2 + i * 2);
    }
    return true;
}

bool ModbusTcpClient::writeSingleRegister(uint8_t unit_id, uint16_t addr, uint16_t value) {
    std::vector<uint8_t> req(5);
    req[0] = kFcWriteSingle;
    putU16(req.data() + 1, addr);
    putU16(req.data() + 3, value);

    std::vector<uint8_t> rsp;
    if (!transact(unit_id, req, rsp)) {
        return false;
    }
    if (rsp[0] != kFcWriteSingle) {
        last_error_ = "功能码与请求不一致";
        return false;
    }
    if (rsp.size() < 5) {
        last_error_ = "写响应过短";
        return false;
    }
    if (getU16(rsp.data() + 1) != addr || getU16(rsp.data() + 3) != value) {
        last_error_ = "写响应内容与请求不一致";
        return false;
    }
    return true;
}

bool ModbusTcpClient::writeMultipleRegisters(uint8_t unit_id,
                                             uint16_t start_addr,
                                             const std::vector<uint16_t>& values) {
    if (values.empty() || values.size() > 123) {
        last_error_ = "写入数量必须在 1~123 之间";
        return false;
    }

    const uint16_t quantity = static_cast<uint16_t>(values.size());
    const uint8_t byte_count = static_cast<uint8_t>(quantity * 2);

    std::vector<uint8_t> req(6 + byte_count);
    req[0] = kFcWriteMultiple;
    putU16(req.data() + 1, start_addr);
    putU16(req.data() + 3, quantity);
    req[5] = byte_count;
    for (uint16_t i = 0; i < quantity; ++i) {
        putU16(req.data() + 6 + i * 2, values[i]);
    }

    std::vector<uint8_t> rsp;
    if (!transact(unit_id, req, rsp)) {
        return false;
    }
    if (rsp[0] != kFcWriteMultiple) {
        last_error_ = "功能码与请求不一致";
        return false;
    }
    if (rsp.size() < 5) {
        last_error_ = "写响应过短";
        return false;
    }
    if (getU16(rsp.data() + 1) != start_addr || getU16(rsp.data() + 3) != quantity) {
        last_error_ = "写响应内容与请求不一致";
        return false;
    }
    return true;
}
