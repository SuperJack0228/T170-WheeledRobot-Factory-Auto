#ifndef AOYIHAND_H
#define AOYIHAND_H
#include <stdint.h>
#include <vector>
#include <numeric>
#include "serial/serial.h"
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <unistd.h>
#include <thread>
#include <csignal>
#include <iomanip>
#include <fcntl.h>
#include <string>
#include <memory>

#include <limits.h>  // 添加这个头文件以定义 PATH_MAX
#include <stdlib.h> 
#include <libusb-1.0/libusb.h>

using namespace std;

enum aoyi_hand
{
    right_a,
    left_a
};

// 协议定义
#define PROTOCOL_HEADER_1 0x55
#define PROTOCOL_HEADER_2 0xAA

// // 全局变量
extern serial::Serial *serial_port;
extern serial::Serial *serial_port2;

extern serial::Serial *right_port;
extern serial::Serial *left_port;

extern uint8_t aoyi_right_id;
extern  uint8_t aoyi_left_id;

// std::unique_ptr<serial::Serial> serial_port;
// std::unique_ptr<serial::Serial> serial_port2;


// std::unique_ptr<serial::Serial> right_port;
// std::unique_ptr<serial::Serial> left_port;




// 设备信息结构体
struct UsbDevice {
    std::string busId;      // e.g. "1-1.3"
    std::string devPath;    // e.g. "/dev/ttyCH341USB0"
    uint16_t vendorId;      // e.g. 0x1a86
    uint16_t productId;     // e.g. 0x7523
};

// 重置模式
enum ResetMode {
    RESET_SYSFS,    // Linux sysfs unbind/bind
    RESET_LIBUSB,   // 跨平台 libusb
    RESET_COMMAND   // 调用外部命令 (usbreset)
};


uint8_t readConfigValue(int lineNum);

void port_bind();

vector<uint8_t> verify_hand(uint8_t handID, uint8_t masterID, uint8_t command, serial::Serial *hand_port);

int aoyi_init();

vector<string> readConfigFile(const string &configPath);

string getLineFromConfig(const string &configPath, int lineNumber = 0);

uint8_t calculateLRC(const uint8_t *data, size_t length);

vector<uint8_t> sendHandCommand(uint8_t handID, uint8_t masterID, uint8_t command, const vector<uint8_t> &data, aoyi_hand side);

bool initSerial(const string &portName, int num);

void cleanup();

void aoyizhuaqu(aoyi_hand side);

void aoyisong(aoyi_hand side);

void tiger_hand(aoyi_hand side);

void grasp_hand(aoyi_hand side);

void grasp_hand2(aoyi_hand side);

void grasp_hand_angle(aoyi_hand side, uint16_t values[5]);

void aoyijiaodu(uint16_t values[6], uint8_t speed[6], aoyi_hand side);

void FINGER_CURRENT_LIMIT(uint16_t values[6], aoyi_hand side);

void FINGER_FORCE_LIMIT(uint16_t values[6], aoyi_hand side);

int get_hand_status(aoyi_hand side);

int set_hand_id(aoyi_hand side);

#endif
