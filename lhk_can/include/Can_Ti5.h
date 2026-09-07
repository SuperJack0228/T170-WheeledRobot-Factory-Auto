#pragma once
#include <vector>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <thread>
#include <fcntl.h>
#include <sys/socket.h>
#include <iostream>
#include <filesystem>
#include <unistd.h>
#include <cstdio>
#include <unordered_map>
#include <signal.h>
#include <string>
#include <array>

inline bool running = true;
inline int stop;
inline int Can_Channels;
inline int max_id = 50;
inline std::vector<int> Can_Socks;

// ========== 仅调参（低延迟默认）：若出现偶发写失败/读不到应答，先把 Delay_After_Sending 提到 2~10 ==========
// 每次 write 成功后休眠（µs）。0 延时最低；部分硬件链路易抖动时可改为 2~10。
inline int Delay_After_Sending_Microseconds = 0;
// write/read 非 EAGAIN 失败时的重试间隔（µs）。0 最快；过高会在异常路径拉长耗时。
inline int Write_Attempt_Gap_Wait_Microseconds = 0;
inline int Write_Attempt_Times = 25;
inline int Read_Attempt_Gap_Wait_Microseconds = 0;
inline int Read_Attempt_Times = 75;
// 接收侧 EAGAIN 时 poll 超时（毫秒）。0 仅检查就绪不阻塞，利于压低 max；1 更省 CPU 但可能偶发 +1ms 长尾。
inline int Read_Poll_Timeout_Ms = 0;
// EAGAIN 后、poll 前：有限次紧读自旋，减少调度/poll 引入的长尾（略占 CPU）；可试 96~128，越高越占 CPU。
inline int Read_BusySpin_Before_Poll = 96;

// 单轴 set+get CSP：发送前是否清空 RX。关闭可减少延时（依赖按 CAN ID 匹配应答）；总线上杂帧多时可改为 true。
inline bool Flush_Rx_Before_CSP = false;
// 批量（每路通道一轮 flush + 连续 TX + 收齐）：建议在通道入口 flush 一次；false 则依赖 ID 匹配丢弃无关帧。
inline bool Flush_Rx_Before_Channel_Batch = true;

// -------------------------------------------------------------------------------------------------------------------------------------------------------------- //



///////////////////////////////////////////////////
///***** PARAMETERS *****//////////////////////////
///////////////////////////////////////////////////

int get_Motor_RunMode(uint8_t Can_ID);			 // 获取电机运行模式
int get_Current(uint8_t Can_ID);				 // 获取电流
int get_Target_Current(uint8_t Can_ID);			 // 获取目标电流
int get_Speed(uint8_t Can_ID);					 // 获取速度
int get_Target_Speed(uint8_t Can_ID);			 // 获取目标速度
int get_Position(uint8_t Can_ID);				 // 获取位置
int get_Target_Position(uint8_t Can_ID);		 // 获取目标位置
int get_Error_Status(uint8_t Can_ID);			 // 获取错误状态
int get_Speed_KP(uint8_t Can_ID);				 // 获取速度环 KP
int get_Speed_KI(uint8_t Can_ID);				 // 获取速度环 KI
int get_Position_KP(uint8_t Can_ID);			 // 获取位置环 KP
int get_Position_KD(uint8_t Can_ID);			 // 获取位置环 KD
int get_Bus_Voltage(uint8_t Can_ID);			 // 获取母线电压
int get_Max_ACC(uint8_t Can_ID);				 // 获取最大加速度
int get_Min_ACC(uint8_t Can_ID);				 // 获取最小加速度
int get_Max_Speed(uint8_t Can_ID);				 // 获取最大速度
int get_Min_Speed(uint8_t Can_ID);				 // 获取最小速度
int get_Max_Position(uint8_t Can_ID);			 // 获取最大位置
int get_Min_Position(uint8_t Can_ID);			 // 获取最小位置
int get_Motor_Temperature(uint8_t Can_ID);		 // 获取电机温度
int get_Board_Temperature(uint8_t Can_ID);		 // 获取驱动板温度
int get_Speed_KD(uint8_t Can_ID);				 // 获取速度 KD
int get_Position_KI(uint8_t Can_ID);			 // 获取位置 KI
int get_Max_Current(uint8_t Can_ID);			 // 获取最大电流
int get_Min_Current(uint8_t Can_ID);			 // 获取最小电流
int get_AbsLimit_Current(uint8_t Can_ID);		 // 获取最大电流绝对值
int get_Profile_ACC(uint8_t Can_ID);			 // 获取轮廓加速度
int get_Profile_Jerk(uint8_t Can_ID);			 // 获取轮廓加加速度
int get_Position_Offset(uint8_t Can_ID);		 // 获取位置偏移
int get_Motor_Model(uint8_t Can_ID);			 // 获取电机型号
int get_Software_Version(uint8_t Can_ID);		 // 获取软件版本号
int get_Hardware_Version(uint8_t Can_ID);		 // 获取硬件版本号
int get_Electric_Angle(uint8_t Can_ID);			 // 获取电角度
int get_Encoder_Mode(uint8_t Can_ID);			 // 获取编码器工作模式
int get_Eight_Bit_Mode(uint8_t Can_ID);			 // 获取八字节指令工作模式
int get_Encoder_Voltage(uint8_t Can_ID);		 // 获取编码器电压
int get_Inside_Encoder_Position(uint8_t Can_ID); // 获取编码器内圈位置
int get_Current_KP(uint8_t Can_ID);				 // 获取电流环 KP
int get_Current_KI(uint8_t Can_ID);				 // 获取电流环 KI
int get_Max_Voltage(uint8_t Can_ID);			 // 获取最大电压
int get_Min_Voltage(uint8_t Can_ID);			 // 获取最小电压
int get_Motor_Max_Temperature(uint8_t Can_ID);	 // 获取电机最大温度
int get_Board_Max_Temperature(uint8_t Can_ID);	 // 获取驱动板最大温度
int get_Inside_Encoder_Cycle(uint8_t Can_ID);	 // 获取内编码器单圈值
int get_Inside_Encoder_Cycles(uint8_t Can_ID);	 // 获取内编码器多圈值
int get_Outside_Encoder_Cycle(uint8_t Can_ID);	 // 获取外编码器单圈值
int get_Outside_Encoder_Cycles(uint8_t Can_ID);	 // 获取外编码器多圈值
std::vector<int> get_CSP(uint8_t Can_ID);		 // 获取电流速度位置

// -------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// -------------------------------------------------------------------------------------------------------------------------------------------------------------- //

///////////////////////////////////////////////////
///***** CONFIGURING *****/////////////////////////
///////////////////////////////////////////////////

bool set_Max_Current(uint8_t Can_ID, int Max_Current);					   // 设置最大电流
bool set_Min_Current(uint8_t Can_ID, int Min_Current);					   // 设置最小电流
bool set_Max_ACC(uint8_t Can_ID, int Max_ACC);							   // 设置最大加速度
bool set_Min_ACC(uint8_t Can_ID, int Min_ACC);							   // 设置最小加速度
bool set_Max_Speed(uint8_t Can_ID, int Max_Speed);						   // 设置最大速度
bool set_Min_Speed(uint8_t Can_ID, int Min_Speed);						   // 设置最小速度
bool set_Max_Position(uint8_t Can_ID, int Max_Position);				   // 设置最大位置
bool set_Min_Position(uint8_t Can_ID, int Min_Position);				   // 设置最小位置
bool set_Speed_KP(uint8_t Can_ID, int Speed_KP);						   // 设置速度环 KP
bool set_Speed_KI(uint8_t Can_ID, int Speed_KI);						   // 设置速度环 KI
bool set_Position_KP(uint8_t Can_ID, int Position_KP);					   // 设置位置环 KP
bool set_Position_KD(uint8_t Can_ID, int Position_KD);					   // 设置位置环 KD
bool set_Electric_Angle(uint8_t Can_ID, int Electric_Angle);			   // 设置电机电角度
bool set_Profile_ACC(uint8_t Can_ID, int Profile_ACC);					   // 设置轮廓加速度
bool set_Profile_Jerk(uint8_t Can_ID, int Profile_Jerk);				   // 设置轮廓加加速度
bool set_Position_Offset(uint8_t Can_ID, int Motor_Position_Offset);	   // 设置位置偏移
bool set_Encoder_Mode(uint8_t Can_ID, int Encoder_Mode);				   // 设置编解码器模式
bool set_Eight_Bit_Mode(uint8_t Can_ID, int Eight_Bit_Mode);			   // 设置八字节指令工作模式
bool set_Current_KP(uint8_t Can_ID, int Current_KP);					   // 设置电流环 KP
bool set_Current_KI(uint8_t Can_ID, int Current_KI);					   // 设置电流环 KI
bool set_Max_Voltage(uint8_t Can_ID, int Max_Voltage);					   // 设置最大电压
bool set_Min_Voltage(uint8_t Can_ID, int Min_Voltage);					   // 设置最小电压
bool set_Motor_Max_Temperature(uint8_t Can_ID, int Motor_Max_Temperature); // 设置电机最大温度
bool set_Board_Max_Temperature(uint8_t Can_ID, int Board_Max_Temperature); // 设置驱动板最大温度
void set_Can_ID(uint8_t Can_ID, int Can_New_ID);						   // 设置 CAN_ID
void set_Can_Baudrate(uint8_t Can_ID, int Can_Baudrate);				   // 设置CAN通信波特率
void set_Position_Feedforward(uint8_t Can_ID, int Position_Feedforward);   // 设置位置前馈
void set_Position_Limit(uint8_t Can_ID, int Motor_Position_Limit);		   // 设置位置限制
void Recover_Default_Setting_save(uint8_t Can_ID);						   // 恢复出厂设置并保存
void Save_Parameters(uint8_t Can_ID);									   // 保存参数
void Recover_Default_Setting(uint8_t Can_ID);							   // 恢复出厂设置不保存

// -------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// -------------------------------------------------------------------------------------------------------------------------------------------------------------- //

///////////////////////////////////////////////////
///***** CONTROL *****/////////////////////////////
///////////////////////////////////////////////////

void Motor_Enable(uint8_t Can_ID);									 // 使能电机
void Motor_Disable(uint8_t Can_ID);									 // 失能电机
void Clear_Error(uint8_t Can_ID);									 // 清楚错误
bool set_Current(uint8_t Can_ID, int Current);						 // 设置电流
bool set_Speed(uint8_t Can_ID, int Speed);							 // 设置速度
bool set_Position(uint8_t Can_ID, int Position);					 // 设置位置
// 单轴：可选 flush_rx（见 Flush_Rx_Before_CSP）；成功返回 3 元 CSP；失败返回空 vector。
std::vector<int> set_Current_get_CSP(uint8_t Can_ID, int Current);
std::vector<int> set_Speed_get_CSP(uint8_t Can_ID, int Speed);
std::vector<int> set_Position_get_CSP(uint8_t Can_ID, int Position);

// -------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// -------------------------------------------------------------------------------------------------------------------------------------------------------------- //

///////////////////////////////////////////////////
///***** ADVANCED CONTROL *****/////////////////////
///////////////////////////////////////////////////

int set_Current(std::vector<uint8_t> Can_ID, std::vector<int> Current);										// 设置电流
int set_Speed(std::vector<uint8_t> Can_ID, std::vector<int> Speed);											// 设置速度
int set_Position(std::vector<uint8_t> Can_ID, std::vector<int> Position);									// 设置位置
std::vector<std::vector<int>> set_Current_get_CSP(const std::vector<uint8_t> &Can_ID, const std::vector<int> &Current);
std::vector<std::vector<int>> set_Speed_get_CSP(const std::vector<uint8_t> &Can_ID, const std::vector<int> &Speed);
std::vector<std::vector<int>> set_Position_get_CSP(const std::vector<uint8_t> &Can_ID, const std::vector<int> &Position);

// -------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// -------------------------------------------------------------------------------------------------------------------------------------------------------------- //

///////////////////////////////////////////////////
///***** INITIALIZATION *****//////////////////////
///////////////////////////////////////////////////

std::string interface_state(const std::string &interface);
bool configure_can_bitrate(const std::string &interface);
int open_can_socket(const std::string &interface);
bool init_can();
void close_canDevice(int sock);
void cleanup_can_channels();
int get_channel_num();
void init_motor();

// -------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// -------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// -------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// -------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// -------------------------------------------------------------------------------------------------------------------------------------------------------------- //

struct double_buffer
{
	int data[2] = {0, 0};
	bool change = false;
	// 使用引用而不是指针
	int &read()
	{
		if (change)
		{
			swap();
			change = false;
		}
		return data[0];
	}
	int &write()
	{
		change = true;
		return data[1];
	}
	const int &read() const { return data[0]; }
	const int &write() const { return data[1]; }
	void swap() { std::swap(data[0], data[1]); }
};

struct motor
{
	int id = 0, channel_sock = -1;
	int min_Position = 0, max_Position = 0;
	int min_Speed = 0, max_Speed = 0;
	int min_Current = 0, max_Current = 0;
	bool PT = false;
	std::unordered_map<int, double_buffer> motor_para_list;
	motor() {}
};

inline std::unordered_map<int, motor> joints;
inline std::unordered_map<int, std::vector<int>> channel_id;

class Can_Motor
{
private:
	// Linux 上优先用 sigaction：行为明确、不会在交付后悄悄重置；安装早于 init，避免长初始化期间 Ctrl+C 仍指向默认/他处覆盖的 handler。
	inline static struct sigaction sa_saved_sigint_{};
	inline static struct sigaction sa_saved_sigterm_{};
	inline static int signal_handler_refcount_ = 0;

	static void install_posix_signal_handlers()
	{
		if (signal_handler_refcount_ > 0)
		{
			++signal_handler_refcount_;
			return;
		}

		struct sigaction sa{};
		sa.sa_handler = signal_handler;
		sigemptyset(&sa.sa_mask);
		sigaddset(&sa.sa_mask, SIGINT);
		sigaddset(&sa.sa_mask, SIGTERM);
		sa.sa_flags = SA_RESTART;

		if (sigaction(SIGINT, &sa, &sa_saved_sigint_) != 0)
		{
			std::perror("Can_Motor: sigaction(SIGINT)");
			return;
		}
		if (sigaction(SIGTERM, &sa, &sa_saved_sigterm_) != 0)
		{
			std::perror("Can_Motor: sigaction(SIGTERM)");
			(void)sigaction(SIGINT, &sa_saved_sigint_, nullptr);
			return;
		}
		signal_handler_refcount_ = 1;
	}

	static void restore_posix_signal_handlers()
	{
		if (signal_handler_refcount_ <= 0)
		{
			return;
		}
		--signal_handler_refcount_;
		if (signal_handler_refcount_ > 0)
		{
			return;
		}
		(void)sigaction(SIGINT, &sa_saved_sigint_, nullptr);
		(void)sigaction(SIGTERM, &sa_saved_sigterm_, nullptr);
	}

public:
	Can_Motor()
	{
		install_posix_signal_handlers();
		try
		{
			init_can();
			init_motor();
		}
		catch (...)
		{
			restore_posix_signal_handlers();
			throw;
		}
	}
	~Can_Motor()
	{
		for (const auto &kv : joints)
		{
			Motor_Disable(static_cast<uint8_t>(kv.first));
		}
		running = false;
		cleanup_can_channels();
		restore_posix_signal_handlers();
	}
	static void signal_handler(int signo)
	{
		std::cout << "Exiting..." << std::endl;
		for (const auto &kv : joints)
		{
			Motor_Disable(static_cast<uint8_t>(kv.first));
		}
		running = false;
		cleanup_can_channels();
		// 在异步信号上下文避免使用 exit()（会跑全局析构/atexit，易与已中断状态死锁）；_exit 立即终止进程。
		_exit(signo);
	}
};

