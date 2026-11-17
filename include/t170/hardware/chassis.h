#pragma once
#include <memory>
#include <string>
#include <vector>

namespace t170::hardware {
// The subset used by factory tasks. No SEER header or socket protocol escapes this port.
class Chassis {
public:
    virtual ~Chassis() = default;
    virtual bool IsConnected() const = 0;
    virtual void SetHost(const std::string &) = 0;
    virtual bool Connect() = 0;
    virtual std::string LastMessage() const = 0;
    virtual int GetLocation(double &,double &,double &,std::string &,std::string &,double &,int &,bool &) = 0;
    virtual int GetBattery(double &,bool &,double &,double &,double &,int &) = 0;
    virtual int GetNavStatus(int &,int &,std::string &,double &,double &,double &,
                             std::vector<std::string> &,std::vector<std::string> &,double &) = 0;
    virtual int GetBlocked(bool &,bool &,double &,double &,std::vector<std::string> &) = 0;
    virtual int GoToStation(const std::string &) = 0;
    virtual int CancelNav() = 0;
    virtual int GetMapInfo(std::string &,std::vector<std::string> &,std::string &,
                          std::vector<std::string> &,std::vector<std::string> &,std::vector<std::string> &) = 0;
    virtual int UploadAndLoadMap(const std::string &) = 0;
    virtual int DoRelocate(double,double,double) = 0;
    virtual int StopCharging() = 0;
};
// Creates a lazy adapter. Construction does not connect or start navigation.
std::unique_ptr<Chassis> make_seer_chassis(std::string host);
} // namespace t170::hardware
