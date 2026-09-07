#ifndef TI5_SOCKETCAN
#define TI5_SOCKETCAN


#include <iostream>
#include <string>
#include <cstring>
#include <array>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <string>
#include <vector>  // 添加 vector 头文件
#include <thread>  

using namespace std;

const int CAN_CHANNEL_COUNT = 6;

extern int left_h_id,left_m_id,left_l_id,right_h_id,right_m_id,right_l_id, head_id, waist_id;

extern std::array<int, CAN_CHANNEL_COUNT> can_sockets;

typedef struct {
    uint32_t AccCode;
    uint32_t AccMask;
    uint8_t Filter;
    uint32_t Bitrate;
    uint8_t Mode;
} CAN_CONFIG;

bool set_can_bitrate(const std::string &ifname, uint32_t bitrate);

bool init_can_channel(int channel, const CAN_CONFIG &config);

bool init_socketcan();

void close_can_channels();

bool send_can_frame(int channel, uint32_t can_id, const uint8_t *data, uint8_t dlc);

bool receive_can_frame_timeout(int channel, uint8_t *data);

void toIntArray(int number, uint8_t *res, int size);

int socketcan_sendcommand(int channel, int cannum, uint32_t *can_idlist, uint8_t command, int *data);

int socketcan_sendsimplecommand(int channel, int cannum, uint32_t *can_idlist, uint8_t command, int *data);

/** CAN 写失败连续 3 次，或读位置超时，置位。编排台据此退出动作循环。 */
bool can_io_faulted();
std::string can_io_fault_message();
void can_io_fault_clear();
void can_io_watch_begin();
void can_io_watch_end();

/** 网页 STOP：中止当前硬件动作并退出整段序列。 */
bool hardware_abort_requested();
void hardware_abort_request();
void hardware_abort_clear();
void hardware_abort_sleep_ms(int ms);


bool select_bind(int &hand_id, int id, string name);

void hand_socketid_bind();

void get_motor_position(int *data, int a);

/** 读左右手 7 轴当前运行速度（CAN 命令 6），a=1 右手 a=0 左手 */
void get_motor_running_speed(int *data, int a);

void set_motor_position(int *data, int a);

void get_motor_speed(int *data, int a);

void set_motor_speed(int *data, int a);


void set_motor_addspeed(int *data, int a);

void get_motor_addspeed(int *data, int a);

void set_motor(int *data, uint8_t command,int a);

void set_motor_singn_mode(int *data,uint8_t command, int a);

void set_motor_current_zero();


void set_waist_motor_position(int *data);



void get_motor_waist_position(int *data);

void motor_speed_change();

void rev_motor_error( int a);

/** 角度(度) → 电机位置计数，与 socketcan 下发公式一致 */
inline int motor_angle_deg_to_cnt(double angle_deg)
{
    return static_cast<int>(angle_deg * 65536.0 * 4.0 / 360.0);
}

/** 电机位置计数 → 角度(度) */
inline double motor_cnt_to_angle_deg(int position_cnt)
{
    return static_cast<double>(position_cnt) * 360.0 / (65536.0 * 4.0);
}

/**
 * 多电机丝滑位置运动（用法同 socketcan_sendcommand，角度在函数内转计数）。
 * @param channel           CAN 通道（如 waist_id）
 * @param motor_count       电机数量（同 socketcan_sendcommand 第2个参数）
 * @param can_idlist        电机 CAN ID 数组
 * @param target_angles_deg 目标角度数组（度），长度 = motor_count
 * @param speed_deg_per_s   平均角速度（度/秒），按最大转角估算总时间，多轴同步到达
 * @param dt_ms             控制周期（毫秒），默认 20
 * @return 0 成功，-1 参数错误或读位置失败
 */
int smooth_motor_move_deg(
    int channel,
    int motor_count,
    uint32_t *can_idlist,
    const double *target_angles_deg,
    double speed_deg_per_s,
    int dt_ms = 20);

bool init_socketcan_can2();
void close_can_channels2();





#endif