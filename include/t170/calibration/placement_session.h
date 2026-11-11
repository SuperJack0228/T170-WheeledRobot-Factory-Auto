#pragma once
#include <json/json.h>
#include <atomic>
#include <memory>
namespace orch { struct HwSession; }
namespace t170::calibration {
// Calls which touch hardware are serialized by the factory's hardware mutex.
class PlacementSession {
public:
    PlacementSession();
    ~PlacementSession();
    bool active() const;
    Json::Value status() const; // cached state only; never reads hardware
    void interrupt(); // asynchronous stop notification; no motor writes
    Json::Value execute(const Json::Value &request,orch::HwSession &hardware);
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}
