#pragma once
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "client/cmd_future.hpp"

// 包含公共客户端接口（单client版本）
#include "client/common/client_common.h"

namespace map_manager {

// ============================================
// 单 Client 版本 API（无 handle 参数）
// 用于只连接单个服务器
// ============================================

// handle（单client版本，无handle参数）
// 检查设备是否存在（单client版本）
bool HasHandleMapManager();

// RPC/Pub函数声明（包含结构体定义）
struct ResultUploadMap {
  // 错误码，默认0表示成功
  int32_t err_code;
  // 错误消息
  std::string err_message;
  // 返回数据
  std::string data;
};

struct ResultDownloadMap {
  // 错误码，默认0表示成功
  int32_t err_code;
  // 错误消息
  std::string err_message;
  // 返回数据
  std::string data;
};

struct ResultChangeMap {
  // 错误码，默认0表示成功
  int32_t err_code;
  // 错误消息
  std::string err_message;
  // 返回数据
  std::string data;
};

struct MapInfo {
  std::string name;
};

/*
 @brief 获取地图列表
 @return 地图名称列表
*/
std::vector<std::string> get_map_list();

/*
 @brief 获取当前地图名称
 @return 地图名称
*/
std::string get_current_map_name();

/*
 @brief 删除地图
 @param map_name: 地图名称
 @return 返回失败原因，字符串空代表成功
*/
std::string delete_map(std::string map_name);

/*
 @brief 开始建图
 @param map_name: 地图名称
 @return 返回失败原因，字符串空代表无成功
*/
std::string start_mapping(const MapInfo& map_info);

/*
 @brief 结束建图
 @return 返回失败原因，字符串空代表无成功
*/
std::string stop_mapping();

/*
 @brief 启动当前地图的地图续建功能(需定位成功)
 @return 返回失败原因，字符串空代表无成功
*/
std::string start_map_extension();

/*
 @brief 停止当前地图的地图续建功能
 @return 返回失败原因，字符串空代表无成功
*/
std::string stop_map_extension();

// Action 相关定义（包含结构体定义）
/*
 @brief 上传地图
 @param map_name: 地图名称
 @param zip_data: 地图zip包
 @return 返回失败原因
*/
// 异步执行 action，返回 future
CmdFuture<ResultUploadMap> upload_map(std::string map_name,
                                      std::string zip_data);
// 异步执行 action，带回调（用于 Python 绑定，避免创建线程）
uint64_t upload_map_with_cb(std::string map_name, std::string zip_data,
                            std::function<void(ResultUploadMap)> callback);
// 取消 cmd
void cancel_upload_map(uint64_t task_id);

/*
 @brief 下载地图
 @param map_name: 地图名称
 @return 地图的zip包，如果为空，代表地图不存在
*/
// 异步执行 action，返回 future
CmdFuture<ResultDownloadMap> download_map(std::string map_name);
// 异步执行 action，带回调（用于 Python 绑定，避免创建线程）
uint64_t download_map_with_cb(std::string map_name,
                              std::function<void(ResultDownloadMap)> callback);
// 取消 cmd
void cancel_download_map(uint64_t task_id);

/*
 @brief 切换地图
 @param map_name: 地图名称
 @return 返回失败原因，字符串空代表无成功
*/
// 异步执行 action，返回 future
CmdFuture<ResultChangeMap> change_map(std::string map_name);
// 异步执行 action，带回调（用于 Python 绑定，避免创建线程）
uint64_t change_map_with_cb(std::string map_name,
                            std::function<void(ResultChangeMap)> callback);
// 取消 cmd
void cancel_change_map(uint64_t task_id);

// 注册所有 action 完成回调处理器（单 client 版本）

}  // namespace map_manager
