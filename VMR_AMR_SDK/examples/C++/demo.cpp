#include <atomic>
#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fstream>
#include <future>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <sys/stat.h>
#include <sys/types.h>

#include "audio.h"
#include "battery.h"
#if defined(HAVE_DEPTH_SEMANTIC)
#include "depth_semantic.h"
#endif
#include "exception.h"
#include "imu.h"
#include "io_manager.h"
#include "laser_2d.h"
#include "lift.h"
#include "light.h"
#include "localization.h"
#if defined(HAVE_MAP_MANAGER)
#include "map_manager.h"
#endif
#if defined(HAVE_MAP_READER)
#include "map_reader.h"
#endif
#include "nav.h"
#include "obstacle_avoidance.h"
#include "odometry.h"
#include "point_cloud.h"
#include "qrcamera.h"
#include "robot_task.h"

namespace {

// RPC 服务默认端口和 demo 默认连接 IP。
constexpr int kRpcPort = 9003;
constexpr const char* kDefaultIp = "192.168.22.244";
constexpr int kDefaultWaitSeconds = 3;

volatile std::sig_atomic_t g_stop = 0;

// 命令行参数集合。
// 默认只打开“安全 RPC 烟测”：
// 1. 不会让车执行拓扑任务、充电任务、顶升动作或 STOP 异常。
// 2. 订阅测试默认关闭，因为当前 rest_rpc client 在订阅回调活跃时继续做
//    普通 RPC 调用，可能触发 call_back 断言；需要时用 --subscriptions 单独跑。
struct Options {
  std::string ip = kDefaultIp;
  bool show_help = false;

  // 默认启用的安全 RPC 测试项。
  bool safe = true;
  bool handles = true;
  bool subscriptions = false;
  bool audio = true;
  bool qrcamera = true;
  bool light = true;
  bool map_query = true;
  bool obstacle = true;
  bool nav = true;
  bool exception_normal = true;
  bool point_cloud_loop = false;

  // 以下测试会改变真实设备状态，必须通过命令行显式打开。
  bool lift = false;
  int32_t lift_height = 0;
  bool topo = false;
  std::string topo_pose_name;
  bool charge = false;
  robot_task::RobotPose charge_pose{0.0, 0.0, 0.0};
  bool charge_relay = false;
  bool charge_relay_state = false;
  bool exception_stop = false;
  bool io_set_do = false;
  int io_handle = 0;
  uint8_t io_bit = 0;
  bool io_value = false;
  bool depth_rgb_enable = false;
  int depth_rgb_handle = 0;
  bool depth_rgb_value = false;
  bool map_delete = false;
  bool map_start_mapping = false;
  bool map_stop_mapping = false;
  bool map_start_extension = false;
  bool map_stop_extension = false;
  bool map_upload = false;
  bool map_download = false;
  bool map_download_dir = false;
  bool map_change = false;
  std::string map_name;
  std::string map_file;
  std::string map_dir;

  // 安全测试的可调参数。
  int qrcamera_location = 1;
  int wait_seconds = kDefaultWaitSeconds;
  std::string audio_name = "modbus:1.mp3";
  std::string point_cloud_file;
};

void PrintUsage(const char* program) {
  std::cout
      << "Usage: " << program << " [ip] [options]\n"
      << "\n"
      << "Default runs safe RPC smoke tests: handles, audio, qrcamera,\n"
      << "light, map query, virtual obstacle, nav zero-speed,\n"
      << "exception NORMAL.\n"
      << "\n"
      << "Options:\n"
      << "  --help                         Show this help\n"
      << "  --safe                         Run safe tests only (default)\n"
      << "  --handles                      Check all device handles\n"
      << "  --map-query, --map-list, --mq  Query current map, map list and map points\n"
      << "  --subscriptions                Register pub/sub callbacks briefly.\n"
      << "                                 It runs at the end, separate from RPC calls.\n"
      << "  --point-cloud-loop             Only subscribe point cloud and print continuously.\n"
      << "                                 Ctrl+C exits; --wait N exits after N seconds.\n"
      << "  --point-cloud-file <path>      Save point cloud as CSV in point-cloud-loop.\n"
      << "  --audio-name <name>            Audio name/path. mp3 uses file path;\n"
      << "                                 modbus uses modbus:xxx or tts:text\n"
      << "  --qrcamera-location <0|1>      QR camera location, default 1\n"
      << "  --wait <seconds>               Callback wait time, default 3\n"
      << "\n"
      << "Real robot state changing tests, run explicitly:\n"
      << "  --lift <height_mm>             Run lift_ctr and print returned height\n"
      << "  --topo <pose_name>             Run topo_move_task\n"
      << "  --charge <x> <y> <theta>       Run charge_task\n"
      << "  --charge-relay <0|1>           Set charge relay\n"
      << "  --exception-stop               Set exception level STOP for 1 second,\n"
      << "                                 then restore NORMAL\n"
      << "  --io-set-do <handle> <bit> <0|1>\n"
      << "                                 Set IO DO bit\n"
      << "  --depth-rgb-enable <handle> <0|1>\n"
      << "                                 Enable or disable depth semantic RGB read\n"
      << "  --map-start-mapping <name>     Start mapping with map name\n"
      << "  --map-stop-mapping             Stop current mapping\n"
      << "  --map-start-extension          Start current map extension\n"
      << "  --map-stop-extension           Stop current map extension\n"
      << "  --map-upload <name> <zip_file> Upload map zip data\n"
      << "  --map-download <name> <zip_file|dir>\n"
      << "                                 Download map zip data to file or dir\n"
      << "  --map-download-dir <name> <dir>\n"
      << "                                 Download map zip data to dir as <name>.zip\n"
      << "  --map-change, --map-use, --mc <name>\n"
      << "                                 Change current map\n"
      << "  --map-delete, --map-rm, --md <name>\n"
      << "                                 Delete map\n";
}

bool IsFlag(const std::string& arg) { return arg.rfind("--", 0) == 0; }

int32_t ToInt(const std::string& value) {
  return static_cast<int32_t>(std::stoi(value));
}

double ToDouble(const std::string& value) { return std::stod(value); }

void DisableDefaultTests(Options& opts) {
  opts.handles = false;
  opts.subscriptions = false;
  opts.audio = false;
  opts.qrcamera = false;
  opts.light = false;
  opts.map_query = false;
  opts.obstacle = false;
  opts.nav = false;
  opts.exception_normal = false;
}

Options ParseArgs(int argc, char** argv) {
  Options opts;
  bool ip_set = false;

  // 第一个非 -- 开头的参数作为 RPC 服务 IP，其余参数按选项解析。
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      opts.show_help = true;
    } else if (arg == "--safe") {
      opts.safe = true;
    } else if (arg == "--handles") {
      opts.handles = true;
    } else if (arg == "--map-query" || arg == "--map-list" ||
               arg == "--mq") {
      DisableDefaultTests(opts);
      opts.map_query = true;
    } else if (arg == "--subscriptions") {
      opts.subscriptions = true;
    } else if (arg == "--point-cloud-loop") {
      opts.point_cloud_loop = true;
      DisableDefaultTests(opts);
    } else if (arg == "--point-cloud-file" && i + 1 < argc) {
      opts.point_cloud_loop = true;
      DisableDefaultTests(opts);
      opts.point_cloud_file = argv[++i];
    } else if (arg == "--audio-name" && i + 1 < argc) {
      opts.audio_name = argv[++i];
    } else if (arg == "--qrcamera-location" && i + 1 < argc) {
      opts.qrcamera_location = ToInt(argv[++i]);
    } else if (arg == "--wait" && i + 1 < argc) {
      opts.wait_seconds = ToInt(argv[++i]);
    } else if (arg == "--lift" && i + 1 < argc) {
      opts.lift = true;
      opts.lift_height = ToInt(argv[++i]);
    } else if (arg == "--topo" && i + 1 < argc) {
      opts.topo = true;
      opts.topo_pose_name = argv[++i];
    } else if (arg == "--charge" && i + 3 < argc) {
      opts.charge = true;
      opts.charge_pose.x = ToDouble(argv[++i]);
      opts.charge_pose.y = ToDouble(argv[++i]);
      opts.charge_pose.theta = ToDouble(argv[++i]);
    } else if (arg == "--charge-relay" && i + 1 < argc) {
      opts.charge_relay = true;
      opts.charge_relay_state = ToInt(argv[++i]) != 0;
    } else if (arg == "--exception-stop") {
      opts.exception_stop = true;
    } else if (arg == "--io-set-do" && i + 3 < argc) {
      opts.io_set_do = true;
      opts.io_handle = ToInt(argv[++i]);
      opts.io_bit = static_cast<uint8_t>(ToInt(argv[++i]));
      opts.io_value = ToInt(argv[++i]) != 0;
    } else if (arg == "--depth-rgb-enable" && i + 2 < argc) {
      opts.depth_rgb_enable = true;
      opts.depth_rgb_handle = ToInt(argv[++i]);
      opts.depth_rgb_value = ToInt(argv[++i]) != 0;
    } else if (arg == "--map-start-mapping" && i + 1 < argc) {
      DisableDefaultTests(opts);
      opts.map_start_mapping = true;
      opts.map_name = argv[++i];
    } else if (arg == "--map-stop-mapping") {
      DisableDefaultTests(opts);
      opts.map_stop_mapping = true;
    } else if (arg == "--map-start-extension") {
      DisableDefaultTests(opts);
      opts.map_start_extension = true;
    } else if (arg == "--map-stop-extension") {
      DisableDefaultTests(opts);
      opts.map_stop_extension = true;
    } else if (arg == "--map-upload" && i + 2 < argc) {
      DisableDefaultTests(opts);
      opts.map_upload = true;
      opts.map_name = argv[++i];
      opts.map_file = argv[++i];
    } else if (arg == "--map-download" && i + 2 < argc) {
      DisableDefaultTests(opts);
      opts.map_download = true;
      opts.map_name = argv[++i];
      opts.map_file = argv[++i];
    } else if (arg == "--map-download-dir" && i + 2 < argc) {
      DisableDefaultTests(opts);
      opts.map_download = true;
      opts.map_download_dir = true;
      opts.map_name = argv[++i];
      opts.map_dir = argv[++i];
    } else if ((arg == "--map-change" || arg == "--map-use" ||
                arg == "--mc") &&
               i + 1 < argc) {
      DisableDefaultTests(opts);
      opts.map_change = true;
      opts.map_name = argv[++i];
    } else if ((arg == "--map-delete" || arg == "--map-rm" ||
                arg == "--md") &&
               i + 1 < argc) {
      DisableDefaultTests(opts);
      opts.map_delete = true;
      opts.map_name = argv[++i];
    } else if (!IsFlag(arg) && !ip_set) {
      opts.ip = arg;
      ip_set = true;
    } else {
      std::cout << "Unknown or incomplete option: " << arg << std::endl;
      opts.show_help = true;
    }
  }

  return opts;
}

std::string ReadBinaryFile(const std::string& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input.is_open()) {
    throw std::runtime_error("open file failed: " + path);
  }
  return std::string(std::istreambuf_iterator<char>(input),
                     std::istreambuf_iterator<char>());
}

void WriteBinaryFile(const std::string& path, const std::string& data) {
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output.is_open()) {
    throw std::runtime_error("open file failed: " + path);
  }
  output.write(data.data(), static_cast<std::streamsize>(data.size()));
  if (!output.good()) {
    throw std::runtime_error("write file failed: " + path);
  }
}

bool IsDirectory(const std::string& path) {
  struct stat st;
  return stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

bool LooksLikeDirectoryPath(const std::string& path) {
  return !path.empty() && (path.back() == '/' || IsDirectory(path));
}

void EnsureDirectory(const std::string& dir) {
  if (dir.empty()) {
    throw std::runtime_error("map download dir is empty");
  }

  std::string current;
  for (size_t i = 0; i < dir.size(); ++i) {
    current.push_back(dir[i]);
    if (dir[i] != '/' || current == "/") {
      continue;
    }
    if (current.size() > 1 && current.back() == '/') {
      current.pop_back();
    }
    if (!current.empty() && !IsDirectory(current) &&
        mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
      throw std::runtime_error("create dir failed: " + current + ", " +
                               std::strerror(errno));
    }
    if (!current.empty() && !IsDirectory(current)) {
      throw std::runtime_error("path exists but is not dir: " + current);
    }
    if (i < dir.size() && dir[i] == '/') {
      current.push_back('/');
    }
  }

  if (!IsDirectory(dir) && mkdir(dir.c_str(), 0755) != 0 && errno != EEXIST) {
    throw std::runtime_error("create dir failed: " + dir + ", " +
                             std::strerror(errno));
  }
  if (!IsDirectory(dir)) {
    throw std::runtime_error("path exists but is not dir: " + dir);
  }
}

std::string JoinPath(const std::string& dir, const std::string& filename) {
  if (dir.empty() || dir.back() == '/') {
    return dir + filename;
  }
  return dir + "/" + filename;
}

std::string MakeMapZipName(const std::string& map_name) {
  std::string filename;
  filename.reserve(map_name.size() + 4);
  for (unsigned char ch : map_name) {
    if (ch == '/' || ch == '\\' || ch < 32) {
      filename.push_back('_');
    } else {
      filename.push_back(static_cast<char>(ch));
    }
  }
  if (filename.empty()) {
    filename = "map";
  }
  filename += ".zip";
  return filename;
}

std::string ResolveMapDownloadPath(const Options& opts) {
  if (opts.map_download_dir) {
    EnsureDirectory(opts.map_dir);
    return JoinPath(opts.map_dir, MakeMapZipName(opts.map_name));
  }
  if (LooksLikeDirectoryPath(opts.map_file)) {
    EnsureDirectory(opts.map_file);
    return JoinPath(opts.map_file, MakeMapZipName(opts.map_name));
  }
  return opts.map_file;
}

void OnSignal(int) { g_stop = 1; }

void InitSdkClient(const std::string& ip) {
  rpc_client::ClientConfig conf;
  conf.ip = ip;
  conf.port = kRpcPort;
  conf.connect_timeout_ms = 3000;
  conf.call_timeout_ms = 60000;
  // RPC 内部异常统一打到 stdout，方便现场直接看 demo 输出。
  conf.error_callback = [](const std::error_code& ec,
                           const std::string& message) {
    std::cout << "RPC error: " << ec.message() << ", details: " << message
              << std::endl;
  };

  if (!rpc_client::InitClient(conf)) {
    std::cout << "init fail: " << ip << ":" << kRpcPort << std::endl;
    std::exit(1);
  }
}

void ExitDemo(int code) {
  // 当前 client 订阅接口没有暴露反订阅能力，demo 退出时可能仍有回调包在路上。
  // 这里不主动 CloseClient，否则 rest_rpc 关闭连接时会触发 Operation aborted
  // 日志，容易被误认为业务 RPC 失败。demo 进程退出后 socket 由系统回收。
  // 使用 _Exit 跳过静态析构，避免 rest_rpc 回调线程在收尾阶段触发断言。
  std::_Exit(code);
}

template <typename Func>
void RunTest(const std::string& name, Func&& func) {
  std::cout << "\n========== " << name << " ==========" << std::endl;
  try {
    func();
  } catch (const std::exception& e) {
    std::cout << name << " failed: " << e.what() << std::endl;
  } catch (...) {
    std::cout << name << " failed: unknown exception" << std::endl;
  }
}

void PrintHandle(const std::string& name, bool ok) {
  std::cout << name << ": " << (ok ? "available" : "missing") << std::endl;
}

template <typename T>
bool Contains(const std::vector<T>& values, T value) {
  return std::find(values.begin(), values.end(), value) != values.end();
}

template <typename T>
std::optional<T> WaitAndPrintResult(CmdFuture<T>& cmd,
                                    const std::string& name,
                                    int timeout_seconds) {
  // action 类接口统一走 CmdFuture，这里封装等待、超时取消和结果打印。
  if (cmd.wait_for(std::chrono::seconds(timeout_seconds)) !=
      std::future_status::ready) {
    std::cout << name << " timeout, cancel task id: " << cmd.id << std::endl;
    if (cmd.cancel) {
      cmd.cancel();
    }
    return std::nullopt;
  }

  const auto result = cmd.get();
  std::cout << name << " task id: " << cmd.id << std::endl;
  std::cout << "err_code: " << result.err_code
            << ", err_message: " << result.err_message << std::endl;
  return result;
}

void TestHandles() {
  // 单实例设备返回 bool，多实例设备返回 handle 数量。
  PrintHandle("audio", audio::HasHandleAudioDev());
  PrintHandle("exception", exception::HasHandleExceptionDev());
  PrintHandle("lift", lift::HasHandleLiftDev());
  PrintHandle("light", light::HasHandleLightDev());
  PrintHandle("localization", localization::HasHandleLocationDev());
#if defined(HAVE_MAP_MANAGER)
  PrintHandle("map_manager", map_manager::HasHandleMapManager());
#else
  std::cout << "map_manager: not linked" << std::endl;
#endif
#if defined(HAVE_MAP_READER)
  PrintHandle("map_reader", map_reader::HasHandleMapReader());
#else
  std::cout << "map_reader: not linked" << std::endl;
#endif
  PrintHandle("nav", nav::HasHandleNavDev());
  PrintHandle("obstacle_avoidance",
              obstacle_avoidance::HasHandleObstacleAvoidanceDev());
  PrintHandle("odometry", odometry::HasHandleOdomDev());
  PrintHandle("point_cloud", point_cloud::HasHandlePointCloudDev());
  PrintHandle("qrcamera", qrcamera::HasHandleQRCameraDev());
  PrintHandle("robot_task", robot_task::HasHandleRobotTaskDev());

  std::cout << "battery count: " << battery::GetAllHandleBatteryDev().size()
            << std::endl;
#if defined(HAVE_DEPTH_SEMANTIC)
  std::cout << "depth_semantic count: "
            << depth_semantic::GetAllHandleDepthSemanticDev().size()
            << std::endl;
#else
  std::cout << "depth_semantic: not linked" << std::endl;
#endif
  std::cout << "io_manager count: " << io_manager::GetAllHandleIoDev().size()
            << std::endl;
  std::cout << "imu count: " << imu::GetAllHandleImuDev().size()
            << std::endl;
  std::cout << "laser_2d count: " << laser_2d::GetAllHandleLaser2DDev().size()
            << std::endl;
}

void TestSubscriptions(int wait_seconds) {
  // 订阅类接口只打印第一帧，避免 demo 长时间运行时刷屏。
  // 这个测试在 main 最后执行，避免订阅回调活跃时继续做普通 RPC 调用。
  if (audio::HasHandleAudioDev()) {
    auto printed = std::make_shared<std::atomic_bool>(false);
    audio::audio_state([printed](audio::AudioState&& state) {
      if (!printed->exchange(true)) {
        std::cout << "audio_state: playing=" << state.playing
                  << ", name=" << state.audio_name
                  << ", volume=" << state.volume << std::endl;
      }
    });
  }

  if (exception::HasHandleExceptionDev()) {
    auto printed = std::make_shared<std::atomic_bool>(false);
    exception::exception_state(
        [printed](exception::ExceptionsInfo&& state) {
          if (!printed->exchange(true)) {
            std::cout << "exception_state: level=" << state.robot_exception
                      << ", count=" << state.exceptions.size() << std::endl;
          }
        });
  }

  if (lift::HasHandleLiftDev()) {
    auto printed = std::make_shared<std::atomic_bool>(false);
    lift::lift_state([printed](lift::LiftInfo&& state) {
      if (!printed->exchange(true)) {
        std::cout << "lift_state: height=" << state.lift_status << std::endl;
      }
    });
  }

  if (light::HasHandleLightDev()) {
    auto printed = std::make_shared<std::atomic_bool>(false);
    light::light_state([printed](light::LightInfo&& state) {
      if (!printed->exchange(true)) {
        std::cout << "light_state: status=" << state.light_status << std::endl;
      }
    });
  }

  if (localization::HasHandleLocationDev()) {
    auto printed = std::make_shared<std::atomic_bool>(false);
    localization::loc_state([printed](localization::LocationInfo&& state) {
      if (!printed->exchange(true)) {
        std::cout << "loc_state: x=" << state.x << ", y=" << state.y
                  << ", theta=" << state.theta
                  << ", status=" << state.status << std::endl;
      }
    });
  }

  if (odometry::HasHandleOdomDev()) {
    auto printed = std::make_shared<std::atomic_bool>(false);
    odometry::odom_state([printed](odometry::OdomInfo&& state) {
      if (!printed->exchange(true)) {
        std::cout << "odom_state: x=" << state.x << ", y=" << state.y
                  << ", theta=" << state.theta << std::endl;
      }
    });
  }

  if (point_cloud::HasHandlePointCloudDev()) {
    auto printed = std::make_shared<std::atomic_bool>(false);
    point_cloud::point_cloud_pub(
        [printed](point_cloud::PointCloud&& cloud) {
          if (!printed->exchange(true)) {
            std::cout << "point_cloud: location=" << cloud.location
                      << ", points=" << cloud.points.size() << std::endl;
          }
        });
  }

  if (qrcamera::HasHandleQRCameraDev()) {
    auto printed = std::make_shared<std::atomic_bool>(false);
    qrcamera::qrcamera_result(
        [printed](qrcamera::QRCameraResult&& result) {
          if (!printed->exchange(true)) {
            std::cout << "qrcamera_result: detected=" << result.detected
                      << ", id=" << result.qr_id << std::endl;
          }
        });
  }

  for (auto id : battery::GetAllHandleBatteryDev()) {
    // 多实例设备需要带 handle 订阅。
    auto printed = std::make_shared<std::atomic_bool>(false);
    battery::battery_state(id, [id, printed](battery::BatteryState&& state) {
      if (!printed->exchange(true)) {
        std::cout << "battery[" << id << "]: percentage="
                  << state.percentage << std::endl;
      }
    });
  }

#if defined(HAVE_DEPTH_SEMANTIC)
  for (auto id : depth_semantic::GetAllHandleDepthSemanticDev()) {
    auto printed = std::make_shared<std::atomic_bool>(false);
    depth_semantic::recognition_state(
        id, [id, printed](depth_semantic::RecognitionResult&& result) {
          if (!printed->exchange(true)) {
            std::cout << "depth_semantic[" << id
                      << "]: classes=" << result.class_names.size()
                      << ", boxes=" << result.obj_box.size()
                      << ", pcloud=" << result.pcloud.points.size()
                      << std::endl;
          }
        });
  }
#endif

  for (auto id : io_manager::GetAllHandleIoDev()) {
    auto printed = std::make_shared<std::atomic_bool>(false);
    io_manager::io_state(id, [id, printed](io_manager::IoDataReport&& state) {
      if (!printed->exchange(true)) {
        const auto di_true =
            std::count(state.di_.begin(), state.di_.end(), true);
        const auto do_true =
            std::count(state.do_.begin(), state.do_.end(), true);
        std::cout << "io_manager[" << id << "]: di=" << state.di_.size()
                  << " true=" << di_true << ", do=" << state.do_.size()
                  << " true=" << do_true << std::endl;
      }
    });
  }

  for (auto id : imu::GetAllHandleImuDev()) {
    auto printed = std::make_shared<std::atomic_bool>(false);
    imu::imu_state(id, [id, printed](imu::VmrImuInfo&& state) {
      if (!printed->exchange(true)) {
        std::cout << "imu[" << id << "]: type=" << state.type
                  << ", w=" << state.orientation.w << std::endl;
      }
    });
  }

  for (auto id : laser_2d::GetAllHandleLaser2DDev()) {
    auto printed = std::make_shared<std::atomic_bool>(false);
    laser_2d::laser_scan(
        id, [id, printed](laser_2d::BasicLaserScan&& scan) {
          if (!printed->exchange(true)) {
            std::cout << "laser_2d[" << id << "]: frame="
                      << scan.header.frame_id
                      << ", ranges=" << scan.ranges.size() << std::endl;
          }
        });
  }

  std::this_thread::sleep_for(std::chrono::seconds(wait_seconds));
}

void TestPointCloudLoop(const Options& opts) {
  // 点云持续打印模式：只订阅 point_cloud，不再混跑其他 RPC。
  // wait_seconds > 0 时打印指定秒数；wait_seconds <= 0 时按 Ctrl+C 退出。
  if (!point_cloud::HasHandlePointCloudDev()) {
    std::cout << "point_cloud missing" << std::endl;
    return;
  }

  auto output = std::make_shared<std::ofstream>();
  auto output_mutex = std::make_shared<std::mutex>();
  if (!opts.point_cloud_file.empty()) {
    output->open(opts.point_cloud_file, std::ios::out | std::ios::trunc);
    if (!output->is_open()) {
      std::cout << "open point cloud file failed: " << opts.point_cloud_file
                << std::endl;
      return;
    }
    *output << "frame,timestamp_ns,location,point_index,x,y,z\n";
    std::cout << "save point cloud csv: " << opts.point_cloud_file
              << std::endl;
  }

  std::atomic<uint64_t> count{0};
  point_cloud::point_cloud_pub([&count, output, output_mutex](
                                   point_cloud::PointCloud&& cloud) {
    const uint64_t index = ++count;
    std::cout << "point_cloud[" << index << "]"
              << " timestamp_ns=" << cloud.timestamp_ns
              << " location=" << cloud.location
              << " points=" << cloud.points.size();

    size_t non_zero_count = 0;
    const point_cloud::PointXYZ* first_valid = nullptr;
    for (const auto& p : cloud.points) {
      if (p.x != 0.0F || p.y != 0.0F || p.z != 0.0F) {
        ++non_zero_count;
        if (first_valid == nullptr) {
          first_valid = &p;
        }
      }
    }
    std::cout << " non_zero=" << non_zero_count;
    if (first_valid != nullptr) {
      std::cout << " first_valid=(" << first_valid->x << ", "
                << first_valid->y << ", " << first_valid->z << ")";
    }
    std::cout << std::endl;

    if (output->is_open()) {
      std::lock_guard<std::mutex> lock(*output_mutex);
      for (size_t i = 0; i < cloud.points.size(); ++i) {
        const auto& p = cloud.points[i];
        *output << index << ',' << cloud.timestamp_ns << ','
                << cloud.location << ',' << i << ',' << p.x << ',' << p.y
                << ',' << p.z << '\n';
      }
      output->flush();
    }
  });

  const auto start = std::chrono::steady_clock::now();
  while (g_stop == 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (opts.wait_seconds > 0 &&
        std::chrono::steady_clock::now() - start >=
            std::chrono::seconds(opts.wait_seconds)) {
      break;
    }
  }
}

void TestMapQuery() {
#if defined(HAVE_MAP_MANAGER)
  if (map_manager::HasHandleMapManager()) {
    const auto current = map_manager::get_current_map_name();
    const auto maps = map_manager::get_map_list();
    std::cout << "current map: " << current << std::endl;
    std::cout << "map list count: " << maps.size() << std::endl;
    for (size_t i = 0; i < maps.size() && i < maps.size() ; ++i) {
      std::cout << "  map[" << i << "]: " << maps[i] << std::endl;
    }
  } else {
    std::cout << "map_manager missing" << std::endl;
  }
#else
  std::cout << "map_manager not linked" << std::endl;
#endif

#if defined(HAVE_MAP_READER)
  if (map_reader::HasHandleMapReader()) {
    const auto info = map_reader::get_current_map_info();
    std::cout << "map goals: " << info.goals.size()
              << ", lines: " << info.lines.size() << std::endl;
    for (size_t i = 0; i < info.goals.size() && i < info.lines.size(); ++i) {
      const auto& goal = info.goals[i];
      std::cout << "  goal[" << i << "]: id=" << goal.id
                << ", name=" << goal.name << ", center=(" << goal.center[0]
                << ", " << goal.center[1] << ")" << std::endl;
    }
  } else {
    std::cout << "map_reader missing" << std::endl;
  }
#else
  std::cout << "map_reader not linked" << std::endl;
#endif
}

void TestMapStartMapping(const Options& opts) {
#if defined(HAVE_MAP_MANAGER)
  // 建图会改变当前地图状态，只在 --map-start-mapping 显式传入时执行。
  if (!map_manager::HasHandleMapManager()) {
    std::cout << "map_manager missing" << std::endl;
    return;
  }

  map_manager::MapInfo info;
  info.name = opts.map_name;
  std::cout << "start_mapping(" << info.name
            << "): " << map_manager::start_mapping(info) << std::endl;
#else
  std::cout << "map_manager not linked; need libclient_map_manager.so for this "
               "architecture"
            << std::endl;
#endif
}

void TestMapStopMapping() {
#if defined(HAVE_MAP_MANAGER)
  if (!map_manager::HasHandleMapManager()) {
    std::cout << "map_manager missing" << std::endl;
    return;
  }

  std::cout << "stop_mapping: " << map_manager::stop_mapping() << std::endl;
#else
  std::cout << "map_manager not linked" << std::endl;
#endif
}

void TestMapStartExtension() {
#if defined(HAVE_MAP_MANAGER)
  // 续建会改变当前地图状态，只在 --map-start-extension 显式传入时执行。
  if (!map_manager::HasHandleMapManager()) {
    std::cout << "map_manager missing" << std::endl;
    return;
  }

  std::cout << "start_map_extension: "
            << map_manager::start_map_extension() << std::endl;
#else
  std::cout << "map_manager not linked" << std::endl;
#endif
}

void TestMapStopExtension() {
#if defined(HAVE_MAP_MANAGER)
  if (!map_manager::HasHandleMapManager()) {
    std::cout << "map_manager missing" << std::endl;
    return;
  }

  std::cout << "stop_map_extension: "
            << map_manager::stop_map_extension() << std::endl;
#else
  std::cout << "map_manager not linked" << std::endl;
#endif
}

void TestMapUpload(const Options& opts) {
#if defined(HAVE_MAP_MANAGER)
  // 上传地图会改变机器人地图集合，只在 --map-upload 显式传入时执行。
  if (!map_manager::HasHandleMapManager()) {
    std::cout << "map_manager missing" << std::endl;
    return;
  }

  const auto zip_data = ReadBinaryFile(opts.map_file);
  std::cout << "upload map file bytes: " << zip_data.size() << std::endl;
  auto cmd = map_manager::upload_map(opts.map_name, zip_data);
  const auto result = WaitAndPrintResult(cmd, "upload_map", 300);
  if (result) {
    std::cout << "data: " << result->data << std::endl;
  }
#else
  std::cout << "map_manager not linked" << std::endl;
#endif
}

void TestMapDownload(const Options& opts) {
#if defined(HAVE_MAP_MANAGER)
  if (!map_manager::HasHandleMapManager()) {
    std::cout << "map_manager missing" << std::endl;
    return;
  }

  auto cmd = map_manager::download_map(opts.map_name);
  const auto result = WaitAndPrintResult(cmd, "download_map", 300);
  if (result) {
    const auto output_path = ResolveMapDownloadPath(opts);
    WriteBinaryFile(output_path, result->data);
    std::cout << "download map file bytes: " << result->data.size()
              << ", path: " << output_path << std::endl;
  }
#else
  std::cout << "map_manager not linked; need libclient_map_manager.so for this "
               "architecture"
            << std::endl;
#endif
}

void TestMapChange(const Options& opts) {
#if defined(HAVE_MAP_MANAGER)
  // 切图会改变当前业务地图，只在 --map-change 显式传入时执行。
  if (!map_manager::HasHandleMapManager()) {
    std::cout << "map_manager missing" << std::endl;
    return;
  }

  auto cmd = map_manager::change_map(opts.map_name);
  const auto result = WaitAndPrintResult(cmd, "change_map", 300);
  if (result) {
    std::cout << "data: " << result->data << std::endl;
  }
#else
  std::cout << "map_manager not linked" << std::endl;
#endif
}

void TestMapDelete(const Options& opts) {
#if defined(HAVE_MAP_MANAGER)
  // 删除地图不可逆，只在 --map-delete 显式传入时执行。
  if (!map_manager::HasHandleMapManager()) {
    std::cout << "map_manager missing" << std::endl;
    return;
  }

  std::cout << "delete_map(" << opts.map_name
            << "): " << map_manager::delete_map(opts.map_name) << std::endl;
#else
  std::cout << "map_manager not linked" << std::endl;
#endif
}

void TestAudio(const Options& opts) {
  // audio_name 为普通文件路径时走 mp3 播放；
  // 以 modbus: 或 tts: 开头时走 modbus 喇叭控制。
  if (!audio::HasHandleAudioDev()) {
    std::cout << "audio missing" << std::endl;
    return;
  }

  std::cout << "query volume: " << audio::query_audio_volume() << std::endl;
  std::cout << "set volume: " << audio::set_audio_volume(60) << std::endl;

  audio::AudioPlayRequest request;
  request.audio_name = opts.audio_name;
  request.priority = 1;
  request.loop = false;
  request.volume = -1;
  std::cout << "play_audio(" << request.audio_name
            << "): " << audio::play_audio(request) << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(1));
  std::cout << "stop_audio: " << audio::stop_audio() << std::endl;
}

void TestQRCamera(const Options& opts) {
  // 二维码相机开关测试：短暂打开后立即关闭，避免影响后续业务。
  if (!qrcamera::HasHandleQRCameraDev()) {
    std::cout << "qrcamera missing" << std::endl;
    return;
  }

  qrcamera::QRCameraSwitchRequest request;
  request.location = opts.qrcamera_location;
  request.enable = true;
  std::cout << "qrcamera enable: " << qrcamera::set_qrcamera_enable(request)
            << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  request.enable = false;
  std::cout << "qrcamera disable: " << qrcamera::set_qrcamera_enable(request)
            << std::endl;
}

void TestLight() {
  // 灯带测试只下发一个空闲蓝灯状态。
  if (!light::HasHandleLightDev()) {
    std::cout << "light missing" << std::endl;
    return;
  }

  light::LightInfo info{};
  info.light_status = light::BLUE_IDLE;
  light::ctr_light(info);
  std::cout << "ctr_light BLUE_IDLE sent" << std::endl;
}

void TestVirtualObstacle() {
  // 使用固定测试 ID，新增后查询，再删除，尽量不留下虚拟障碍物。
  if (!obstacle_avoidance::HasHandleObstacleAvoidanceDev()) {
    std::cout << "obstacle_avoidance missing" << std::endl;
    return;
  }

  obstacle_avoidance::VirtualObstacleInfo obstacle{};
  obstacle.obstacle_id = 900001;
  obstacle.top_left = {0.5F, 0.5F};
  obstacle.bottom_right = {0.8F, 0.8F};
  std::cout << "add_virtual_obstacle: "
            << obstacle_avoidance::add_virtual_obstacle(obstacle)
            << std::endl;
  const auto list = obstacle_avoidance::query_virtual_obstacle();
  std::cout << "query_virtual_obstacle count: "
            << list.virtual_obstacle_list.size() << std::endl;
  std::cout << "remove_virtual_obstacle: "
            << obstacle_avoidance::remove_virtual_obstacle(
                   obstacle.obstacle_id)
            << std::endl;
}

void TestNav() {
  // 只切入第三方速度控制并下发 0 速度，然后退出控制。
  if (!nav::HasHandleNavDev()) {
    std::cout << "nav missing" << std::endl;
    return;
  }

  nav::Twist zero{};
  std::cout << "set_speed_factor(1.0): " << nav::ctrl_speed_factor(1.0)
            << std::endl;
  std::cout << "set_third_speed_control: "
            << nav::set_third_speed_ctrl() << std::endl;
  std::cout << "set_third_expected_speed zero: "
            << nav::send_third_expected_speed(zero) << std::endl;
  std::cout << "close_third_speed_control: "
            << nav::close_third_speed_ctrl() << std::endl;
}

void TestExceptionNormal() {
  // 默认只设置 NORMAL，作为 exception RPC 的安全连通性测试。
  if (!exception::HasHandleExceptionDev()) {
    std::cout << "exception missing" << std::endl;
    return;
  }

  std::cout << "set_exception_level NORMAL: "
            << exception::set_exception_level(exception::NORMAL)
            << std::endl;
}

void TestExceptionStop() {
  // STOP 会影响真实车体状态，因此只在 --exception-stop 显式传入时执行。
  if (!exception::HasHandleExceptionDev()) {
    std::cout << "exception missing" << std::endl;
    return;
  }

  std::cout << "set_exception_level STOP: "
            << exception::set_exception_level(exception::STOP) << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(1));
  std::cout << "restore NORMAL: "
            << exception::set_exception_level(exception::NORMAL)
            << std::endl;
}

void TestLift(const Options& opts) {
  // 顶升会动作真实机构，只在 --lift <height_mm> 显式传入时执行。
  if (!lift::HasHandleLiftDev()) {
    std::cout << "lift missing" << std::endl;
    return;
  }

  lift::LiftInfo info{};
  info.lift_status = opts.lift_height;
  auto cmd = lift::lift_ctr(info);
  const auto result = WaitAndPrintResult(cmd, "lift_ctr", 60);
  if (result) {
    std::cout << "returned lift height: " << result->data << std::endl;
  }
}

void TestTopoMove(const Options& opts) {
  // 拓扑任务会让车移动，只在 --topo <pose_name> 显式传入时执行。
  if (!robot_task::HasHandleRobotTaskDev()) {
    std::cout << "robot_task missing" << std::endl;
    return;
  }

  robot_task::TopoPose pose;
  pose.pose_name = opts.topo_pose_name;
  pose.action = 0;
  auto cmd = robot_task::topo_move_task(pose);
  WaitAndPrintResult(cmd, "topo_move_task", 300);
}

void TestChargeTask(const Options& opts) {
  // 充电任务会让车移动到指定坐标，只在 --charge 显式传入时执行。
  if (!robot_task::HasHandleRobotTaskDev()) {
    std::cout << "robot_task missing" << std::endl;
    return;
  }

  auto cmd = robot_task::charge_task(opts.charge_pose);
  WaitAndPrintResult(cmd, "charge_task", 300);
}

void TestChargeRelay(const Options& opts) {
  // 充电继电器会改变硬件输出，只在 --charge-relay <0|1> 显式传入时执行。
  if (!robot_task::HasHandleRobotTaskDev()) {
    std::cout << "robot_task missing" << std::endl;
    return;
  }

  std::cout << "charge_relay(" << opts.charge_relay_state
            << "): " << robot_task::charge_relay(opts.charge_relay_state)
            << std::endl;
}

void TestIoSetDo(const Options& opts) {
  // IO 输出会改变真实硬件状态，只在 --io-set-do 显式传入时执行。
  const auto handles = io_manager::GetAllHandleIoDev();
  if (!Contains(handles, opts.io_handle)) {
    std::cout << "io_manager handle missing: " << opts.io_handle << std::endl;
    return;
  }

  io_manager::set_do_bit(opts.io_handle, opts.io_bit, opts.io_value);
  std::cout << "set_do_bit handle=" << opts.io_handle
            << ", bit=" << static_cast<int>(opts.io_bit)
            << ", value=" << opts.io_value << " sent" << std::endl;
}

void TestDepthRgbEnable(const Options& opts) {
#if defined(HAVE_DEPTH_SEMANTIC)
  // RGB 开关会改变深度语义模块的数据读取状态，只在显式传参时执行。
  const auto handles = depth_semantic::GetAllHandleDepthSemanticDev();
  if (!Contains(handles, opts.depth_rgb_handle)) {
    std::cout << "depth_semantic handle missing: " << opts.depth_rgb_handle
              << std::endl;
    return;
  }

  depth_semantic::set_rgb_enable(opts.depth_rgb_handle, opts.depth_rgb_value);
  std::cout << "set_rgb_enable handle=" << opts.depth_rgb_handle
            << ", value=" << opts.depth_rgb_value << " sent" << std::endl;
#else
  std::cout << "depth_semantic not linked" << std::endl;
#endif
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGPIPE, SIG_IGN);
  std::signal(SIGINT, OnSignal);
  std::signal(SIGTERM, OnSignal);

  const auto opts = ParseArgs(argc, argv);
  if (opts.show_help) {
    PrintUsage(argv[0]);
    return 0;
  }

  InitSdkClient(opts.ip);

  if (opts.point_cloud_loop) {
    RunTest("point_cloud_loop",
            [&opts] { TestPointCloudLoop(opts); });
    ExitDemo(0);
  }

  // 默认执行安全烟测；危险动作只有对应参数为 true 时才会进入。
  if (opts.handles) {
    RunTest("handles", TestHandles);
  }
  if (opts.audio) {
    RunTest("audio", [&opts] { TestAudio(opts); });
  }
  if (opts.qrcamera) {
    RunTest("qrcamera", [&opts] { TestQRCamera(opts); });
  }
  if (opts.light) {
    RunTest("light", TestLight);
  }
  if (opts.map_query) {
    RunTest("map_query", TestMapQuery);
  }
  if (opts.map_start_mapping) {
    RunTest("map_start_mapping",
            [&opts] { TestMapStartMapping(opts); });
  }
  if (opts.map_stop_mapping) {
    RunTest("map_stop_mapping", TestMapStopMapping);
  }
  if (opts.map_start_extension) {
    RunTest("map_start_extension", TestMapStartExtension);
  }
  if (opts.map_stop_extension) {
    RunTest("map_stop_extension", TestMapStopExtension);
  }
  if (opts.map_upload) {
    RunTest("map_upload", [&opts] { TestMapUpload(opts); });
  }
  if (opts.map_download) {
    RunTest("map_download", [&opts] { TestMapDownload(opts); });
  }
  if (opts.map_change) {
    RunTest("map_change", [&opts] { TestMapChange(opts); });
  }
  if (opts.map_delete) {
    RunTest("map_delete", [&opts] { TestMapDelete(opts); });
  }
  if (opts.obstacle) {
    RunTest("virtual_obstacle", TestVirtualObstacle);
  }
  if (opts.nav) {
    RunTest("nav", TestNav);
  }
  if (opts.exception_normal) {
    RunTest("exception_normal", TestExceptionNormal);
  }
  if (opts.exception_stop) {
    RunTest("exception_stop", TestExceptionStop);
  }
  if (opts.lift) {
    RunTest("lift", [&opts] { TestLift(opts); });
  }
  if (opts.topo) {
    RunTest("topo_move_task", [&opts] { TestTopoMove(opts); });
  }
  if (opts.charge) {
    RunTest("charge_task", [&opts] { TestChargeTask(opts); });
  }
  if (opts.charge_relay) {
    RunTest("charge_relay", [&opts] { TestChargeRelay(opts); });
  }
  if (opts.io_set_do) {
    RunTest("io_set_do", [&opts] { TestIoSetDo(opts); });
  }
  if (opts.depth_rgb_enable) {
    RunTest("depth_rgb_enable", [&opts] { TestDepthRgbEnable(opts); });
  }
  if (opts.subscriptions) {
    RunTest("subscriptions",
            [&opts] { TestSubscriptions(opts.wait_seconds); });
  }

  ExitDemo(0);
}
