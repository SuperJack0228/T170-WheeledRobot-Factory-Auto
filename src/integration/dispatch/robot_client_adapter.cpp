#include "t170/integration/dispatch/client.h"
#include "robot_client.hpp"
#include <mutex>
namespace t170::integration::dispatch {
namespace {
class RobotClientAdapter final : public Client {
    std::mutex mutex_;
    std::unique_ptr<RobotClient> sdk_;
    RobotClient *sdk() {std::lock_guard<std::mutex> lock(mutex_);return sdk_.get();}
public:
    RobotClientAdapter(const std::string &type,const std::string &serial):sdk_(std::make_unique<RobotClient>(type,serial)){}
    std::string apply_mission() override {auto *p=sdk();return p?p->apply_mission():std::string();}
    bool apply_place_raw_material(int ms) override{auto *p=sdk();return p && p->apply_place_raw_material(ms);}
    bool apply_pick_up_half_done(int ms) override{auto *p=sdk();return p && p->apply_pick_up_half_done(ms);}
    bool apply_place_half_done(int ms) override{auto *p=sdk();return p && p->apply_place_half_done(ms);}
    bool apply_pick_up_well_done(int ms) override{auto *p=sdk();return p && p->apply_pick_up_well_done(ms);}
    bool mission_done() override{auto *p=sdk();return p && p->mission_done();}
    bool pick_up_done() override{auto *p=sdk();return p && p->pick_up_done();}
    bool place_done() override{auto *p=sdk();return p && p->place_done();}
    bool raw_material_empty() override{auto *p=sdk();return p && p->raw_material_empty();}
    bool well_done_material_full() override{auto *p=sdk();return p && p->well_done_material_full();}
    bool report_exception(int code) override{auto *p=sdk();return p && p->report_exception(code);}
    void close() noexcept override {
        // Retain the field SDK's destruction-before-join unblock order. The
        // adapter stays alive until the receiving thread has returned.
        std::unique_ptr<RobotClient> old;
        {std::lock_guard<std::mutex> lock(mutex_);old=std::move(sdk_);}
        old.reset();
    }
};
class RobotClientGateway final : public Gateway {
public:
    std::unique_ptr<Client> connect(const std::string &type,const std::string &serial) override{
        return std::make_unique<RobotClientAdapter>(type,serial);
    }
    Snapshot snapshot() const override {
        Snapshot s;s.is_map_updated=RobotClient::is_map_updated;
        s.raw_material_point=RobotClient::raw_material_point;s.well_done_point=RobotClient::well_done_point;
        s.standby_point=RobotClient::standby_point;s.charge_point=RobotClient::charge_point;
        s.sequence_one_port_one=RobotClient::sequence_one_port_one;s.sequence_one_port_two=RobotClient::sequence_one_port_two;
        s.sequence_two_port_one=RobotClient::sequence_two_port_one;s.sequence_two_port_two=RobotClient::sequence_two_port_two;
        return s;
    }
    int exception_code(Fault f) const override {
        switch(f) {
        case Fault::CameraDisconnected:return RobotClient::CameraDisconnected;
        case Fault::InitFailed:return RobotClient::InitFailed;
        case Fault::LoadPortBlocked:return RobotClient::LoadPortBlocked;
        case Fault::MotorDisconnected:return RobotClient::MotorDisconnected;
        case Fault::QrUnrecognized:return RobotClient::QrUnrecognized;
        case Fault::UnloadPointEmpty:return RobotClient::UnloadPointEmpty;
        case Fault::WorkpieceFallen:return RobotClient::WorkpieceFallen;
        }return 0;
    }
    void update_pose(double x,double y,double yaw,const std::string &station) override{
        RobotClient::x=static_cast<float>(x);RobotClient::y=static_cast<float>(y);RobotClient::angle=static_cast<float>(yaw);
        if(!station.empty())RobotClient::current_point=station;
    }
    void update_battery(int value) override{RobotClient::power_percent=value;}
    void mark_map_current() override{RobotClient::is_map_updated=false;}
};
}
std::unique_ptr<Gateway> make_robot_client_gateway(){return std::make_unique<RobotClientGateway>();}
} // namespace t170::integration::dispatch
