#pragma once
// Inert ports for offline tests. Not part of any production include path.
#include "t170/hardware/chassis.h"
#include "t170/integration/dispatch/client.h"
namespace factory_test {
class Chassis : public t170::hardware::Chassis {
public:
    bool IsConnected() const override{return true;}
    void SetHost(const std::string &) override{}
    bool Connect() override{return true;}
    std::string LastMessage() const override{return "synthetic";}
    int GetLocation(double &,double &,double &,std::string &,std::string &,double &,int &,bool &) override{return -1;}
    int GetBattery(double &,bool &,double &,double &,double &,int &) override{return -1;}
    int GetNavStatus(int &,int &,std::string &,double &,double &,double &,std::vector<std::string> &,std::vector<std::string> &,double &) override{return -1;}
    int GetBlocked(bool &,bool &,double &,double &,std::vector<std::string> &) override{return -1;}
    int GoToStation(const std::string &) override{return -1;}
    int CancelNav() override{return 0;}
    int GetMapInfo(std::string &,std::vector<std::string> &,std::string &,std::vector<std::string> &,std::vector<std::string> &,std::vector<std::string> &) override{return -1;}
    int UploadAndLoadMap(const std::string &) override{return -1;}
    int DoRelocate(double,double,double) override{return -1;}
    int StopCharging() override{return 0;}
};
class Client : public t170::integration::dispatch::Client {
public:
    std::string apply_mission() override{return {};}
    bool apply_place_raw_material(int) override{return true;}
    bool apply_pick_up_half_done(int) override{return true;}
    bool apply_place_half_done(int) override{return true;}
    bool apply_pick_up_well_done(int) override{return true;}
    bool mission_done() override{return true;}
    bool pick_up_done() override{return true;}
    bool place_done() override{return true;}
    bool raw_material_empty() override{return true;}
    bool well_done_material_full() override{return true;}
    bool report_exception(int) override{return true;}
    void close() noexcept override{}
};
class Gateway : public t170::integration::dispatch::Gateway {
public:
    t170::integration::dispatch::Snapshot state;
    std::unique_ptr<t170::integration::dispatch::Client> connect(const std::string &,const std::string &) override{return std::make_unique<Client>();}
    t170::integration::dispatch::Snapshot snapshot() const override{return state;}
    int exception_code(t170::integration::dispatch::Fault) const override{return 0;}
    void update_pose(double,double,double,const std::string &) override{}
    void update_battery(int) override{}
    void mark_map_current() override{state.is_map_updated=false;}
};
}
