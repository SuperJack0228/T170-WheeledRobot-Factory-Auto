#pragma once
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "client/cmd_future.hpp"

// 包含公共客户端接口（单client版本）
#include "client/common/client_common.h"

namespace audio {

// ============================================
// 单 Client 版本 API（无 handle 参数）
// 用于只连接单个服务器
// ============================================

// handle（单client版本，无handle参数）
// 检查设备是否存在（单client版本）
bool HasHandleAudioDev();

// RPC/Pub函数声明（包含结构体定义）
/*
 @brief 音频播放请求
 @param std::string audio_name: 音频名称或音频文件路径; mp3 使用文件路径, modbus
 可使用 modbus:12.mp3 或 tts:文本
 @param int32_t priority: 播放优先级, 数值越大优先级越高
 @param bool loop: 是否循环播放
 @param int32_t volume: 播放音量, 范围 0~100, 小于 0 表示使用当前音量
*/
struct AudioPlayRequest {
  std::string audio_name;
  int32_t priority = 0;
  bool loop = false;
  int32_t volume = -1;
};

/*
 @brief 音频设备状态
 @param uint64_t timestamp_ns: 时间戳 ns
 @param bool playing: 是否正在播放
 @param std::string audio_name: 当前播放音频名称
 @param int32_t volume: 当前音量, 范围 0~100
 @param int32_t exception: 异常码, 0 表示正常
*/
struct AudioState {
  uint64_t timestamp_ns;
  bool playing = false;
  std::string audio_name;
  int32_t volume = 0;
  int32_t exception = 0;
};

/*
 @brief 实时获取音频设备状态
 @param cb: 回调函数, 接收 AudioState 参数
*/
void audio_state(const std::function<void(AudioState&&)>& cb);

/*
 @brief 播放音频
 @param AudioPlayRequest: 音频播放请求
*/
bool play_audio(const AudioPlayRequest& audio_request);

/*
 @brief 停止当前音频播放
*/
bool stop_audio();

/*
 @brief 设置音频音量
 @param int32_t: 音量, 范围 0~100
*/
bool set_audio_volume(int32_t volume);

/*
 @brief 查询音频音量
 @return int32_t: 当前音量, 范围 0~100
*/
int32_t query_audio_volume();

// Action 相关定义（包含结构体定义）

// 注册所有 action 完成回调处理器（单 client 版本）

}  // namespace audio
