#pragma once

#include <linux/can.h>

#include <cstddef>
#include <cstdint>
#include <span>

/// ROH-AP001 SerialCtrl V3.0。命令字两边一样。不收发。
/// 按出参类型重载：span<can_frame> = CAN，span<uint8_t> = 串口。组包返回写入个数，失败返回 0。拆包失败返回 false。
/// packet_complete 返回应丢掉的帧数/字节数，0 表示未齐。ack(rx, id, cmd) 认该命令的成功回包。
/// 拆包要带手 ID；单指回包还要带 finger_id。
/// 串口：完整包（55 AA + LRC），主机 ID=1。
/// CAN：同一应用层包按最多 8 字节拆成标准帧。发往手 can_id=手 ID；回包 can_id 可能是手 ID 或主机 1（payload 里 src 才是手 ID）。
/// 尾帧 dlc=剩余字节，不补满 8。回包按字节拼回再解。
/// finger_id：0 拇指 … 5 拇指根。角度=实际×100；位置 0～65535；speed 0～255。
/// 无力控（0x07/0x08/0x12/0x47/0x53/0x54）。无出厂校正、厂方数据、手册标明 ROHand 忽略的命令。

// CAN 控制
std::size_t finger_start(std::uint8_t id, std::span<can_frame> out);                                       // 0x49
std::size_t finger_stop(std::uint8_t id, std::span<can_frame> out);                                        // 0x4A
std::size_t set_finger_pos_abs(std::uint8_t id, std::uint8_t finger_id, std::uint16_t pos, std::uint8_t speed,
                              std::span<can_frame> out);                                                 // 0x4B
std::size_t set_finger_pos(std::uint8_t id, std::uint8_t finger_id, std::uint16_t pos, std::uint8_t speed,
                          std::span<can_frame> out);                                                     // 0x4C
std::size_t set_finger_angle(std::uint8_t id, std::uint8_t finger_id, std::int16_t angle, std::uint8_t speed,
                            std::span<can_frame> out);                                                   // 0x4D
std::size_t set_thumb_root_pos(std::uint8_t id, std::uint8_t preset, std::span<can_frame> out);            // 0x4E，{0,1,2}
std::size_t set_finger_pos_abs_all(std::uint8_t id, std::span<const std::uint16_t> pos, std::span<const std::uint8_t> speed,
                                  std::span<can_frame> out);                                             // 0x4F
std::size_t set_finger_pos_all(std::uint8_t id, std::span<const std::uint16_t> pos, std::span<const std::uint8_t> speed,
                              std::span<can_frame> out);                                                 // 0x50
std::size_t set_finger_angle_all(std::uint8_t id, std::span<const std::int16_t> angle, std::span<const std::uint8_t> speed,
                                std::span<can_frame> out);                                               // 0x51
std::size_t get_finger_pos_abs(std::uint8_t id, std::uint8_t finger_id, std::span<can_frame> out);         // 0x0A
std::size_t get_finger_pos(std::uint8_t id, std::uint8_t finger_id, std::span<can_frame> out);             // 0x0B
std::size_t get_finger_angle(std::uint8_t id, std::uint8_t finger_id, std::span<can_frame> out);           // 0x0C
std::size_t get_thumb_root_pos(std::uint8_t id, std::span<can_frame> out);                                 // 0x0D
std::size_t get_finger_pos_abs_all(std::uint8_t id, std::span<can_frame> out);                             // 0x0E
std::size_t get_finger_pos_all(std::uint8_t id, std::span<can_frame> out);                                 // 0x0F
std::size_t get_finger_angle_all(std::uint8_t id, std::span<can_frame> out);                               // 0x10
std::size_t get_finger_current(std::uint8_t id, std::uint8_t finger_id, std::span<can_frame> out);         // 0x06
bool finger_pos_abs(std::span<const can_frame> rx, std::uint8_t id, std::uint8_t finger_id, std::uint16_t& target,
                   std::uint16_t& current);
bool finger_pos(std::span<const can_frame> rx, std::uint8_t id, std::uint8_t finger_id, std::uint16_t& target,
               std::uint16_t& current);
bool finger_angle(std::span<const can_frame> rx, std::uint8_t id, std::uint8_t finger_id, std::int16_t& target,
                 std::int16_t& current);
bool thumb_root_pos(std::span<const can_frame> rx, std::uint8_t id, std::uint8_t& pos);
bool finger_pos_abs_all(std::span<const can_frame> rx, std::uint8_t id, std::span<std::uint16_t> target,
                       std::span<std::uint16_t> current);
bool finger_pos_all(std::span<const can_frame> rx, std::uint8_t id, std::span<std::uint16_t> target,
                   std::span<std::uint16_t> current);
bool finger_angle_all(std::span<const can_frame> rx, std::uint8_t id, std::span<std::int16_t> target,
                     std::span<std::int16_t> current);
bool finger_current(std::span<const can_frame> rx, std::uint8_t id, std::uint8_t finger_id, std::uint16_t& ma);

// CAN 配置
std::size_t set_node_id(std::uint8_t id, std::uint8_t node_id, std::span<can_frame> out);                  // 0x42
std::size_t set_finger_pid(std::uint8_t id, std::uint8_t finger_id, float p, float i, float d, float g,
                          std::span<can_frame> out);                                                     // 0x45
std::size_t set_finger_current_limit(std::uint8_t id, std::uint8_t finger_id, std::uint16_t ma,
                                    std::span<can_frame> out);                                           // 0x46
std::size_t set_finger_pos_limit(std::uint8_t id, std::uint8_t finger_id, std::uint16_t pos,
                                std::span<can_frame> out);                                               // 0x48
std::size_t set_finger_stop_params(std::uint8_t id, std::uint8_t finger_id, std::uint16_t speed,
                                  std::uint16_t stop_current_ma, std::uint16_t stop_after_ms,
                                  std::uint16_t retry_interval_ms, std::span<can_frame> out);             // 0x52
std::size_t set_self_test_level(std::uint8_t id, std::uint8_t level, std::span<can_frame> out);            // 0x60
std::size_t set_beep_switch(std::uint8_t id, std::uint8_t on, std::span<can_frame> out);                   // 0x61
std::size_t beep(std::uint8_t id, std::uint16_t period_ms, std::span<can_frame> out);                      // 0x62
std::size_t set_speed_ctrl_params(std::uint8_t id, std::uint16_t brake_distance, std::uint16_t accel_distance,
                                 std::uint16_t speed_ratio, std::span<can_frame> out);                   // 0x66
std::size_t get_finger_pid(std::uint8_t id, std::uint8_t finger_id, std::span<can_frame> out);             // 0x04
std::size_t get_finger_current_limit(std::uint8_t id, std::uint8_t finger_id, std::span<can_frame> out);   // 0x05
std::size_t get_finger_pos_limit(std::uint8_t id, std::uint8_t finger_id, std::span<can_frame> out);       // 0x09
std::size_t get_finger_stop_params(std::uint8_t id, std::uint8_t finger_id, std::span<can_frame> out);     // 0x11
std::size_t get_self_test_switch(std::uint8_t id, std::span<can_frame> out);                               // 0x20
std::size_t get_beep_switch(std::uint8_t id, std::span<can_frame> out);                                    // 0x21
std::size_t get_speed_ctrl_params(std::uint8_t id, std::span<can_frame> out);                              // 0x3D
bool finger_pid(std::span<const can_frame> rx, std::uint8_t id, std::uint8_t finger_id, float& p, float& i, float& d,
               float& g);
bool finger_current_limit(std::span<const can_frame> rx, std::uint8_t id, std::uint8_t finger_id, std::uint16_t& ma);
bool finger_pos_limit(std::span<const can_frame> rx, std::uint8_t id, std::uint8_t finger_id, std::uint16_t& pos);
bool finger_stop_params(std::span<const can_frame> rx, std::uint8_t id, std::uint8_t finger_id, std::uint16_t& speed,
                       std::uint16_t& stop_current_ma, std::uint16_t& stop_after_ms, std::uint16_t& retry_interval_ms);
bool self_test_switch(std::span<const can_frame> rx, std::uint8_t id, std::uint8_t& on);
bool beep_switch(std::span<const can_frame> rx, std::uint8_t id, std::uint8_t& on);
bool speed_ctrl_params(std::span<const can_frame> rx, std::uint8_t id, std::uint16_t& brake_distance,
                      std::uint16_t& accel_distance, std::uint16_t& speed_ratio);

// CAN 系统
std::size_t get_protocol_version(std::uint8_t id, std::span<can_frame> out);        // 0x00
std::size_t get_fw_version(std::uint8_t id, std::span<can_frame> out);              // 0x01，探活用
std::size_t get_hw_version(std::uint8_t id, std::span<can_frame> out);              // 0x02
std::size_t get_uid(std::uint8_t id, std::span<can_frame> out);                     // 0x23
std::size_t get_vendor_id(std::uint8_t id, std::span<can_frame> out);               // 0x3F
std::size_t get_motor_status(std::uint8_t id, std::span<can_frame> out);            // 0x5F 子命令 0x80
std::size_t reset_hand(std::uint8_t id, std::uint8_t mode, std::span<can_frame> out);  // 0x40，0 工作 / 1 DFU
std::size_t start_init(std::uint8_t id, std::span<can_frame> out);                  // 0x64
std::size_t packet_complete(std::span<const can_frame> frames, std::uint8_t id);
bool ack(std::span<const can_frame> rx, std::uint8_t id, std::uint8_t cmd);
bool ack(std::span<const can_frame> rx, std::uint8_t id, std::uint8_t cmd, std::uint8_t& err);
bool probe_ok(std::span<const can_frame> rx, std::uint8_t id);
bool protocol_version(std::span<const can_frame> rx, std::uint8_t id, std::uint8_t& major, std::uint8_t& minor);
bool fw_version(std::span<const can_frame> rx, std::uint8_t id, std::uint16_t& ver, std::uint8_t& major,
               std::uint8_t& minor);
bool hw_version(std::span<const can_frame> rx, std::uint8_t id, std::uint8_t& a, std::uint8_t& b, std::uint8_t& c,
               std::uint8_t& d);
bool uid(std::span<const can_frame> rx, std::uint8_t id, std::uint32_t& a, std::uint32_t& b, std::uint32_t& c);
bool vendor_id(std::span<const can_frame> rx, std::uint8_t id, char& a, char& b);
std::size_t motor_status(std::span<const can_frame> rx, std::uint8_t id, std::span<std::uint8_t> out);

// 串口 控制
std::size_t finger_start(std::uint8_t id, std::span<std::uint8_t> out);                                    // 0x49
std::size_t finger_stop(std::uint8_t id, std::span<std::uint8_t> out);                                     // 0x4A
std::size_t set_finger_pos_abs(std::uint8_t id, std::uint8_t finger_id, std::uint16_t pos, std::uint8_t speed,
                              std::span<std::uint8_t> out);                                              // 0x4B
std::size_t set_finger_pos(std::uint8_t id, std::uint8_t finger_id, std::uint16_t pos, std::uint8_t speed,
                          std::span<std::uint8_t> out);                                                  // 0x4C
std::size_t set_finger_angle(std::uint8_t id, std::uint8_t finger_id, std::int16_t angle, std::uint8_t speed,
                            std::span<std::uint8_t> out);                                                // 0x4D
std::size_t set_thumb_root_pos(std::uint8_t id, std::uint8_t preset, std::span<std::uint8_t> out);         // 0x4E，{0,1,2}
std::size_t set_finger_pos_abs_all(std::uint8_t id, std::span<const std::uint16_t> pos, std::span<const std::uint8_t> speed,
                                  std::span<std::uint8_t> out);                                          // 0x4F
std::size_t set_finger_pos_all(std::uint8_t id, std::span<const std::uint16_t> pos, std::span<const std::uint8_t> speed,
                              std::span<std::uint8_t> out);                                              // 0x50
std::size_t set_finger_angle_all(std::uint8_t id, std::span<const std::int16_t> angle, std::span<const std::uint8_t> speed,
                                std::span<std::uint8_t> out);                                            // 0x51
std::size_t get_finger_pos_abs(std::uint8_t id, std::uint8_t finger_id, std::span<std::uint8_t> out);      // 0x0A
std::size_t get_finger_pos(std::uint8_t id, std::uint8_t finger_id, std::span<std::uint8_t> out);          // 0x0B
std::size_t get_finger_angle(std::uint8_t id, std::uint8_t finger_id, std::span<std::uint8_t> out);        // 0x0C
std::size_t get_thumb_root_pos(std::uint8_t id, std::span<std::uint8_t> out);                              // 0x0D
std::size_t get_finger_pos_abs_all(std::uint8_t id, std::span<std::uint8_t> out);                          // 0x0E
std::size_t get_finger_pos_all(std::uint8_t id, std::span<std::uint8_t> out);                              // 0x0F
std::size_t get_finger_angle_all(std::uint8_t id, std::span<std::uint8_t> out);                            // 0x10
std::size_t get_finger_current(std::uint8_t id, std::uint8_t finger_id, std::span<std::uint8_t> out);      // 0x06
bool finger_pos_abs(std::span<const std::uint8_t> rx, std::uint8_t id, std::uint8_t finger_id, std::uint16_t& target,
                   std::uint16_t& current);
bool finger_pos(std::span<const std::uint8_t> rx, std::uint8_t id, std::uint8_t finger_id, std::uint16_t& target,
               std::uint16_t& current);
bool finger_angle(std::span<const std::uint8_t> rx, std::uint8_t id, std::uint8_t finger_id, std::int16_t& target,
                 std::int16_t& current);
bool thumb_root_pos(std::span<const std::uint8_t> rx, std::uint8_t id, std::uint8_t& pos);
bool finger_pos_abs_all(std::span<const std::uint8_t> rx, std::uint8_t id, std::span<std::uint16_t> target,
                       std::span<std::uint16_t> current);
bool finger_pos_all(std::span<const std::uint8_t> rx, std::uint8_t id, std::span<std::uint16_t> target,
                   std::span<std::uint16_t> current);
bool finger_angle_all(std::span<const std::uint8_t> rx, std::uint8_t id, std::span<std::int16_t> target,
                     std::span<std::int16_t> current);
bool finger_current(std::span<const std::uint8_t> rx, std::uint8_t id, std::uint8_t finger_id, std::uint16_t& ma);

// 串口 配置
std::size_t set_node_id(std::uint8_t id, std::uint8_t node_id, std::span<std::uint8_t> out);               // 0x42
std::size_t set_finger_pid(std::uint8_t id, std::uint8_t finger_id, float p, float i, float d, float g,
                          std::span<std::uint8_t> out);                                                  // 0x45
std::size_t set_finger_current_limit(std::uint8_t id, std::uint8_t finger_id, std::uint16_t ma,
                                    std::span<std::uint8_t> out);                                        // 0x46
std::size_t set_finger_pos_limit(std::uint8_t id, std::uint8_t finger_id, std::uint16_t pos,
                                std::span<std::uint8_t> out);                                            // 0x48
std::size_t set_finger_stop_params(std::uint8_t id, std::uint8_t finger_id, std::uint16_t speed,
                                  std::uint16_t stop_current_ma, std::uint16_t stop_after_ms,
                                  std::uint16_t retry_interval_ms, std::span<std::uint8_t> out);          // 0x52
std::size_t set_self_test_level(std::uint8_t id, std::uint8_t level, std::span<std::uint8_t> out);         // 0x60
std::size_t set_beep_switch(std::uint8_t id, std::uint8_t on, std::span<std::uint8_t> out);                // 0x61
std::size_t beep(std::uint8_t id, std::uint16_t period_ms, std::span<std::uint8_t> out);                   // 0x62
std::size_t set_speed_ctrl_params(std::uint8_t id, std::uint16_t brake_distance, std::uint16_t accel_distance,
                                 std::uint16_t speed_ratio, std::span<std::uint8_t> out);                // 0x66
std::size_t get_finger_pid(std::uint8_t id, std::uint8_t finger_id, std::span<std::uint8_t> out);          // 0x04
std::size_t get_finger_current_limit(std::uint8_t id, std::uint8_t finger_id, std::span<std::uint8_t> out); // 0x05
std::size_t get_finger_pos_limit(std::uint8_t id, std::uint8_t finger_id, std::span<std::uint8_t> out);    // 0x09
std::size_t get_finger_stop_params(std::uint8_t id, std::uint8_t finger_id, std::span<std::uint8_t> out);  // 0x11
std::size_t get_self_test_switch(std::uint8_t id, std::span<std::uint8_t> out);                            // 0x20
std::size_t get_beep_switch(std::uint8_t id, std::span<std::uint8_t> out);                                 // 0x21
std::size_t get_speed_ctrl_params(std::uint8_t id, std::span<std::uint8_t> out);                           // 0x3D
bool finger_pid(std::span<const std::uint8_t> rx, std::uint8_t id, std::uint8_t finger_id, float& p, float& i, float& d,
               float& g);
bool finger_current_limit(std::span<const std::uint8_t> rx, std::uint8_t id, std::uint8_t finger_id, std::uint16_t& ma);
bool finger_pos_limit(std::span<const std::uint8_t> rx, std::uint8_t id, std::uint8_t finger_id, std::uint16_t& pos);
bool finger_stop_params(std::span<const std::uint8_t> rx, std::uint8_t id, std::uint8_t finger_id, std::uint16_t& speed,
                       std::uint16_t& stop_current_ma, std::uint16_t& stop_after_ms, std::uint16_t& retry_interval_ms);
bool self_test_switch(std::span<const std::uint8_t> rx, std::uint8_t id, std::uint8_t& on);
bool beep_switch(std::span<const std::uint8_t> rx, std::uint8_t id, std::uint8_t& on);
bool speed_ctrl_params(std::span<const std::uint8_t> rx, std::uint8_t id, std::uint16_t& brake_distance,
                      std::uint16_t& accel_distance, std::uint16_t& speed_ratio);

// 串口 系统
std::size_t get_protocol_version(std::uint8_t id, std::span<std::uint8_t> out);     // 0x00
std::size_t get_fw_version(std::uint8_t id, std::span<std::uint8_t> out);           // 0x01，探活用
std::size_t get_hw_version(std::uint8_t id, std::span<std::uint8_t> out);           // 0x02
std::size_t get_uid(std::uint8_t id, std::span<std::uint8_t> out);                  // 0x23
std::size_t get_vendor_id(std::uint8_t id, std::span<std::uint8_t> out);            // 0x3F
std::size_t get_motor_status(std::uint8_t id, std::span<std::uint8_t> out);         // 0x5F 子命令 0x80
std::size_t reset_hand(std::uint8_t id, std::uint8_t mode, std::span<std::uint8_t> out);  // 0x40，0 工作 / 1 DFU
std::size_t start_init(std::uint8_t id, std::span<std::uint8_t> out);               // 0x64
std::size_t packet_complete(std::span<const std::uint8_t> raw, std::uint8_t id);
bool ack(std::span<const std::uint8_t> rx, std::uint8_t id, std::uint8_t cmd);
bool ack(std::span<const std::uint8_t> rx, std::uint8_t id, std::uint8_t cmd, std::uint8_t& err);
bool probe_ok(std::span<const std::uint8_t> rx, std::uint8_t id);
bool protocol_version(std::span<const std::uint8_t> rx, std::uint8_t id, std::uint8_t& major, std::uint8_t& minor);
bool fw_version(std::span<const std::uint8_t> rx, std::uint8_t id, std::uint16_t& ver, std::uint8_t& major,
               std::uint8_t& minor);
bool hw_version(std::span<const std::uint8_t> rx, std::uint8_t id, std::uint8_t& a, std::uint8_t& b, std::uint8_t& c,
               std::uint8_t& d);
bool uid(std::span<const std::uint8_t> rx, std::uint8_t id, std::uint32_t& a, std::uint32_t& b, std::uint32_t& c);
bool vendor_id(std::span<const std::uint8_t> rx, std::uint8_t id, char& a, char& b);
std::size_t motor_status(std::span<const std::uint8_t> rx, std::uint8_t id, std::span<std::uint8_t> out);
