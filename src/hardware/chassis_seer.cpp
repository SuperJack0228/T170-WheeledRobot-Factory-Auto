#include "t170/hardware/chassis.h"
#include "Ti5_Seer.hpp"
#include <mutex>
#include <utility>
namespace t170::hardware {
namespace {
class SeerAdapter final : public Chassis {
    std::string host_;
    mutable std::mutex creation_;
    mutable std::unique_ptr<Seer> sdk_;
    Seer &sdk() const {
        std::lock_guard<std::mutex> lock(creation_);
        if(!sdk_) sdk_=std::make_unique<Seer>(host_);
        return *sdk_;
    }
public:
    explicit SeerAdapter(std::string host):host_(std::move(host)){}
    bool IsConnected() const override{return sdk().IsConnected();}
    void SetHost(const std::string &host) override{sdk().SetHost(host);}
    bool Connect() override{return sdk().Connect();}
    std::string LastMessage() const override{return sdk().LastMessage();}
    int GetLocation(double &x,double &y,double &a,std::string &cur,std::string &last,double &confidence,int &state,bool &forbidden) override{
        return sdk().GetLocation(x,y,a,cur,last,confidence,state,forbidden);
    }
    int GetBattery(double &level,bool &charging,double &voltage,double &current,double &temperature,int &cycles) override{
        return sdk().GetBattery(level,charging,voltage,current,temperature,cycles);
    }
    int GetNavStatus(int &state,int &type,std::string &target,double &x,double &y,double &a,
                     std::vector<std::string> &finished,std::vector<std::string> &unfinished,double &distance) override{
        return sdk().GetNavStatus(state,type,target,x,y,a,finished,unfinished,distance);
    }
    int GetBlocked(bool &blocked,bool &slowed,double &x,double &y,std::vector<std::string> &obstacles) override{
        return sdk().GetBlocked(blocked,slowed,x,y,obstacles);
    }
    int GoToStation(const std::string &station) override{return sdk().GoToStation(station);}
    int CancelNav() override{return sdk().CancelNav();}
    int GetMapInfo(std::string &current,std::vector<std::string> &maps,std::string &md5,
                   std::vector<std::string> &names,std::vector<std::string> &sizes,std::vector<std::string> &times) override{
        return sdk().GetMapInfo(current,maps,md5,names,sizes,times);
    }
    int UploadAndLoadMap(const std::string &body) override{return sdk().UploadAndLoadMap(body);}
    int DoRelocate(double x,double y,double yaw) override{return sdk().DoRelocate(x,y,yaw);}
    int StopCharging() override{return sdk().SetChargingRelay("{\"status\":false}");}
};
}
std::unique_ptr<Chassis> make_seer_chassis(std::string host){return std::make_unique<SeerAdapter>(std::move(host));}
} // namespace t170::hardware
