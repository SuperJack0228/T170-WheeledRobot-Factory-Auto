#ifndef MODBUS_TCP_H
#define MODBUS_TCP_H

#include <cstdint>
#include <string>
#include <vector>

// Modbus TCP 客户端：连接设备并读写保持寄存器。
class ModbusTcpClient {
public:
    ModbusTcpClient();
    ~ModbusTcpClient();

    ModbusTcpClient(const ModbusTcpClient&) = delete;
    ModbusTcpClient& operator=(const ModbusTcpClient&) = delete;

    // 连接到指定 IP 和端口，成功返回 true。
    bool connect(const std::string& ip, uint16_t port, int timeout_ms = 3000);

    void disconnect();
    bool isConnected() const;

    // 功能码 0x03：从 start_addr 起读取 quantity 个保持寄存器。
    bool readHoldingRegisters(uint8_t unit_id,
                              uint16_t start_addr,
                              uint16_t quantity,
                              std::vector<uint16_t>& values);

    // 功能码 0x06：向单个保持寄存器写入 16 位数值。
    bool writeSingleRegister(uint8_t unit_id, uint16_t addr, uint16_t value);

    // 功能码 0x10：从 start_addr 起连续写入多个保持寄存器。
    bool writeMultipleRegisters(uint8_t unit_id,
                                uint16_t start_addr,
                                const std::vector<uint16_t>& values);

    const std::string& lastError() const { return last_error_; }

private:
    bool sendAll(const uint8_t* data, size_t len);
    bool recvAll(uint8_t* data, size_t len);
    bool setSocketTimeout(int timeout_ms);
    bool transact(uint8_t unit_id,
                  const std::vector<uint8_t>& req_pdu,
                  std::vector<uint8_t>& rsp_pdu);

    int sock_;
    uint16_t transaction_id_;
    std::string last_error_;
};

#endif
