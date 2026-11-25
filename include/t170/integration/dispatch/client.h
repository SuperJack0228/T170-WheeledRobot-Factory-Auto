#pragma once
#include <memory>
#include <string>

namespace t170::integration::dispatch {
enum class Fault { CameraDisconnected, InitFailed, LoadPortBlocked, MotorDisconnected,
                   QrUnrecognized, UnloadPointEmpty, WorkpieceFallen, GripperNotReady };
// 1404 是本机新增的机器人异常。已安装的调度 SDK 枚举只到 1403，report_exception 接收整数。
inline constexpr int kGripperNotReadyCode = 1404;
struct Snapshot {
    bool is_map_updated=false;
    std::string raw_material_point,well_done_point,standby_point,charge_point;
    std::string sequence_one_port_one,sequence_one_port_two,sequence_two_port_one,sequence_two_port_two;
};
class Client {
public:
    virtual ~Client()=default;
    virtual std::string apply_mission()=0;
    virtual bool apply_place_raw_material(int timeout_ms)=0;
    virtual bool apply_pick_up_half_done(int timeout_ms)=0;
    virtual bool apply_place_half_done(int timeout_ms)=0;
    virtual bool apply_pick_up_well_done(int timeout_ms)=0;
    virtual bool mission_done()=0;
    virtual bool pick_up_done()=0;
    virtual bool place_done()=0;
    virtual bool raw_material_empty()=0;
    virtual bool well_done_material_full()=0;
    virtual bool report_exception(int native_code)=0;
    // Disconnect first, then join the task receiver, then destroy this wrapper.
    virtual void close() noexcept=0;
};
class Gateway {
public:
    virtual ~Gateway()=default;
    virtual std::unique_ptr<Client> connect(const std::string &robot_type,const std::string &serial)=0;
    virtual Snapshot snapshot() const=0;
    virtual int exception_code(Fault) const=0;
    virtual void update_pose(double x,double y,double yaw,const std::string &station)=0;
    virtual void update_battery(int percent)=0;
    virtual void mark_map_current()=0;
};
std::unique_ptr<Gateway> make_robot_client_gateway();
} // namespace t170::integration::dispatch
