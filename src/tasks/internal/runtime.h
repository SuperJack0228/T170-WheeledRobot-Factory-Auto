#pragma once
#include "t170/config/access.h"
#include "t170/skills/types.h"
// Transitional host bridges between task implementations. No vendor SDK headers.
#include "t170/tasks/context.h"
#include "t170/tasks/factory.h"
#include "t170/tasks/operations.h"
#include "t170/common/announcements.h"
#include "t170/integration/dispatch/messages.h"
#include "t170/integration/dispatch/inbox.h"
#include "t170/telemetry/factory_status.h"
#include "t170/skills/legacy/profiles.h"
#include "t170/skills/legacy/runtime.h"
#include "t170/hardware/device_bus.h"
#include <filesystem>
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>
namespace orch {
using Chassis=t170::hardware::Chassis;
inline t170::integration::dispatch::Gateway &dispatch_gateway(){return t170::tasks::services().dispatch;}
inline constexpr int kVisionQrFail = 10;
inline constexpr int kBeltPlaceOccupied = 16;
inline constexpr float kBeltPartMergeIou = 0.5f;
inline constexpr float kFallenClearlyHigher = 0.10f;
inline constexpr double kBeltGraspWinXMinM = -0.08;
inline constexpr double kBeltGraspWinXMaxM = 0.10;
inline constexpr double kBeltGraspWinYMinM = 0.09;
inline constexpr double kBeltGraspWinYMaxM = 0.14;
inline constexpr int kBeltGraspEmpty = 11;
inline constexpr int kBeltNoPartVision = 12;
inline constexpr int kBeltFallen = 13;
inline constexpr int kBeltGripperNotReady = 15;
inline constexpr int kBeltHandSkip = 14;
inline constexpr int kEmptyRight = 1;
inline constexpr int kEmptyLeft = 2;
inline constexpr int kDispatchApplyTimeoutMs = 20000;
inline constexpr int kDispatchNextMissionWaitMs = 20000;
inline constexpr int kDispatchHoldStay = 9;
inline constexpr int kDispatchNoPartStay = 13;
inline constexpr int kDispatchGripperStay = 17;
inline constexpr int kDispatchTaskSkip = 18;
inline constexpr int kDispatchOffline = 19;
inline constexpr int kDispatchReportFailed = 20;
inline constexpr int kDispatchPointsMissing = 7;
extern HwSession *g_hw;
extern int g_load_parts_seen;
extern bool g_grasp_departed;
extern std::function<void()> g_on_grasp_depart;
extern bool g_unload_empty_known;
extern bool g_dispatch_stay_at_tray2;
extern std::string g_photo_ready_station;
extern bool g_tray2_place_prep_done;
extern int g_unload_empty_right;
extern int g_unload_empty_left;
extern std::string g_dispatch_nav_fault;
extern std::chrono::steady_clock::time_point g_station_arrived_at;
extern bool g_station_arrived;
extern std::thread g_belt_depart_th;
extern std::atomic<int> g_belt_depart_rc;
extern bool g_belt_depart_started;
extern std::thread g_tray2_waist_th;
extern std::atomic<int> g_tray2_waist_rc;
extern bool g_tray2_waist_started;
extern bool g_body_disabled_for_charge;
int chassis_goto_station(
    const std::string &station,
    bool watch_obstacle = false,
    const std::function<void()> &on_station_name = {});
struct ChassisTravel
{
    std::thread th;
    std::atomic<int> rc{0};

    ChassisTravel(const std::string &station, bool watch_obstacle = false,
                  const std::function<void()> &on_station_name = {})
    {
        th = std::thread([this, station, watch_obstacle, on_station_name]() {
            try
            {
                rc = chassis_goto_station(station, watch_obstacle, on_station_name);
            }
            catch (const std::exception &ex)
            {
                std::cerr << "[orch] 底盘导航失败: " << ex.what() << "\n";
                rc = -6;
            }
        });
    }

    ChassisTravel(const ChassisTravel &) = delete;
    ChassisTravel &operator=(const ChassisTravel &) = delete;

    int join()
    {
        if (th.joinable())
            th.join();
        return rc.load();
    }

    ~ChassisTravel()
    {
        if (th.joinable())
            th.join();
    }
};
struct BeltPartMark
{
    double x = 0;
    double y = 0;
    double z = 0;
    bool have_marker = false;
    int class_id = -1;
    float conf = 0.f;
    float x0 = -1.f;
    float y0 = -1.f;
    float x1 = -1.f;
    float y1 = -1.f;
};
struct BeltPartList
{
    bool ok = false;
    std::string message;
    std::vector<BeltPartMark> parts;
};
struct MarkerQuery
{
    double x = 0;
    double y = 0;
    const char *name = "";
};
struct BeltGraspWindow
{
    int count = 0;
    bool right = false;
    bool left = false;
};
struct TaskPoints
{
    std::string raw;
    std::string well;
    std::string load;
    std::string unload;
};
struct StationOverride
{
    ChassisConfig saved;
    explicit StationOverride(const ChassisConfig &now) {
        std::lock_guard lock(t170::config::access_mutex());saved=now;
    }
    ~StationOverride() {
        std::lock_guard lock(t170::config::access_mutex());g_move_cfg.chassis=saved;
    }
};
enum class DispatchNote
{
    PlaceHand,
    PickHand,
    Mission
};
int waist_rotate_deg(double yaw_deg, double speed_deg_s);
bool apply_fixed_belt_origin_z(BeltDetectResult &det, const ConveyorStationConfig &c, const char *tag);
int conveyor_waist_and_head(const ConveyorStationConfig &c, const char *tag);
const char *belt_yolo_class_zh(int class_id);
void log_gripper_qs(const char *tag);
bool station_name_matches(const std::string &got, const std::string &want);
Chassis &chassis_seer();
bool ensure_chassis();
bool stage_dispatch_map_name();
bool read_pose_before_dispatch(double &x, double &y, double &angle);
bool finish_dispatch_map(DispatchClient &client, double x, double y, double angle);
std::string chassis_location_station();
void publish_dispatch_pose(double x, double y, double angle, const std::string &station = {});
void publish_dispatch_battery(double level);
void start_belt_depart_after_right(const std::string &station, const char *hand_zh = "右手");
void start_tray2_waist_after_right_arrived();
int take_belt_depart();
int take_tray2_waist();
BeltPartList detect_belt_parts_full(
    SegPoseBridge &bridge,
    const CameraFrameData &frame,
    const ConveyorStationConfig &c,
    const BeltDetectResult &det,
    const char *tag,
    const PoseRunResult *precomputed = nullptr);
bool belt_part_in_grasp_window(const BeltPartMark &part);
void log_belt_used_box(const char *name, const char *role, int index, const BeltPartMark &part);
BeltGraspWindow classify_belt_grasp_window(
    const std::vector<BeltPartMark> &parts, const char *tag);
std::vector<int> match_parts_to_queries(
    const std::vector<MarkerQuery> &queries,
    const std::vector<BeltPartMark> &parts,
    double tol_m);
int skill_legacy_code(const t170::skills::Result &result);
int vision_grasp_with_engine(SegEngineId engine_id, bool adaptive_approach_rpy = false,
                            bool depart_after_grasp = true, bool clear_tray_when_stay = true);
TaskPoints read_task_points();
bool task_points_ready(const std::string &task, const TaskPoints &p);
void dispatch_note_state(const std::string &state);
std::string dispatch_idle_station();
bool dispatch_apply(
    const std::string &request, const std::string &grant, const std::string &what,
    const std::function<bool()> &call);
int dispatch_wait_system(DispatchClient &client, const std::string &state);
std::string dispatch_exception_state();
int dispatch_report(DispatchClient &client, DispatchNote note, const char *what);
int dispatch_report_now(DispatchClient &client, DispatchNote note, const char *what);
void dispatch_send(DispatchClient &client, const char *name, bool sent);
int confirm_belt_place_not_fallen(
    DispatchClient &client, const ConveyorStationConfig &c, const char *tag,
    bool do_right, bool do_left, const char *place_what);
// 0：操作员选了重试。跳过、下线、发送失败分别返回上面三个码。
int dispatch_report_ex(DispatchClient &client, const char *zh);
// 0：再拍一次。否则把跳过、下线或发送失败原样交回去。
int dispatch_belt_vision_again(DispatchClient &client, int &vision_retry, const char *what);
int dispatch_place_belt(
    DispatchClient &client, bool (*apply_place)(DispatchClient &, int),
    const char *request_state, const char *grant_state,
    const ConveyorStationConfig &c, const char *tag,
    bool hold_right, bool hold_left, const char *place_what,
    bool skip_ready = false, bool already_granted = false);
bool apply_pick_half(DispatchClient &client, int timeout_ms);
bool apply_pick_well(DispatchClient &client, int timeout_ms);
bool belt_ready_to_pick(const std::string &station, const ConveyorStationConfig &c, const char *what);
int drive_belt_ready(const std::string &station, const ConveyorStationConfig &c, const char *tag);
int dispatch_pick_pair(
    DispatchClient &client, bool (*apply_pick)(DispatchClient &, int),
    const std::string &request_state, const std::string &grant_state,
    const ConveyorStationConfig &c, const std::string &station, const char *tag,
    bool do_left, bool do_right, const char *left_what, const char *right_what,
    const std::string &depart_after_right = {}, bool skip_apply = false);
int place_while_driving(
    DispatchClient &client, bool (*apply_place)(DispatchClient &, int),
    const char *request_state, const char *grant_state,
    const std::string &station, const ConveyorStationConfig &c, const char *tag,
    bool hold_right, bool hold_left, const char *place_what);
int set_body_motors(bool enable);
std::string resolve_charge_station();
int run_charge_flow(DispatchClient *client, bool stop_when_charging);
int dispatch_charge(DispatchClient &client);
int dispatch_load(DispatchClient &client);
int dispatch_transfer(DispatchClient &client);
int dispatch_unload(DispatchClient &client);
int go_belt_ready(
    Arm &arm_r, Arm &arm_l, const ConveyorStationConfig &c, const char *tag);
int go_belt_place(
    Arm &arm_r, Arm &arm_l, const ConveyorStationConfig &c, const char *tag);
int go_belt_grasp_rpy(
    Arm &arm_r, Arm &arm_l, const ConveyorStationConfig &c, const char *tag);
int go_belt_station(
    const ConveyorStationConfig &c, const char *tag,
    const std::function<int(bool is_right)> &on_hand_placed = {},
    bool do_right = true, bool do_left = true,
    bool skip_ready = false, int photo_after_arrival_ms = -1);
int go_belt_grasp(
    bool from_place, const ConveyorStationConfig &c, const std::string &station, const char *tag,
    int only_hand = -1, const std::function<int(bool is_right)> &on_grasped = {},
    bool photo_settle = true, const std::string &depart_after_right = {},
    const std::function<void(bool is_right)> &on_lift = {},
    bool no_empty_retry = false);
void stop_chassis_status_reporter();
}
