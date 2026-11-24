#include "t170/config/conveyor_heights.h"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <tuple>

namespace t170::config {
ConveyorHeightCatalog load_conveyor_heights(const std::string &path) {
    const auto node=YAML::LoadFile(path);ConveyorHeightCatalog out;
    if(node["schema_version"].as<int>()!=1)throw std::runtime_error("Unsupported conveyor height schema");
    out.enabled=node["enabled"].as<bool>();out.reference_machine=node["reference_machine"].as<std::string>();
    std::set<std::string> names,businesses;std::set<int> info_ids,locate_ids;
    if(!node["machines"].IsSequence())throw std::runtime_error("conveyor heights machines must be a sequence");
    for(const auto &row:node["machines"]) {
        ConveyorMachineHeight m;m.machine_id=row["machine_id"].as<std::string>();
        m.business_id=row["business_id"].as<std::string>();m.operation_profile=row["operation_profile"].as<std::string>();
        m.locate_id=row["locate_id"].as<int>();m.info_id=row["info_id"].as<int>();
        m.qr_plane_height_m=row["qr_plane_height_cm"].as<double>()*.01;
        if(m.machine_id.empty() || m.business_id.empty() || !names.insert(m.machine_id).second ||
           !businesses.insert(m.business_id).second || !info_ids.insert(m.info_id).second ||
           m.info_id<0 || m.info_id>=1000 || m.locate_id<0 || m.locate_id>=1000 || m.info_id==m.locate_id ||
           (m.operation_profile!="conveyor" && m.operation_profile!="conveyor2") ||
           !std::isfinite(m.qr_plane_height_m) || m.qr_plane_height_m<=0)
            throw std::runtime_error("Invalid/duplicate conveyor height record: "+m.machine_id);
        locate_ids.insert(m.locate_id);out.machines.push_back(m);
    }
    for(int id:info_ids)if(locate_ids.count(id))throw std::runtime_error("Information ID conflicts with location ID");
    if(!names.count(out.reference_machine))throw std::runtime_error("Unknown conveyor reference machine");
    return out;
}

bool same_conveyor_identity(const ConveyorHeightCatalog &a,const ConveyorHeightCatalog &b) {
    auto identity=[](const ConveyorHeightCatalog &catalog) {
        std::vector<std::tuple<std::string,std::string,std::string,int,int>> keys;
        for(const auto &m:catalog.machines)keys.emplace_back(m.machine_id,m.business_id,m.operation_profile,m.locate_id,m.info_id);
        std::sort(keys.begin(),keys.end());return keys;
    };
    return a.enabled==b.enabled && identity(a)==identity(b);
}

bool match_conveyor_height(const ConveyorHeightCatalog &catalog,const std::vector<int> &ids,
    const std::string &profile,double reference_z,double reference_waist,double waist,
    ConveyorHeightMatch &out,std::string &error) {
    error.clear();out={};
    if(!catalog.enabled){error="传送带高度表未启用";return false;}
    if(!std::isfinite(reference_z)||!std::isfinite(reference_waist)||!std::isfinite(waist)){
        error="传送带参考高度或腰高无效";return false;
    }
    const ConveyorMachineHeight *selected=nullptr,*reference=nullptr;
    for(const auto &m:catalog.machines) {
        if(m.machine_id==catalog.reference_machine)reference=&m;
        if(std::find(ids.begin(),ids.end(),m.info_id)!=ids.end()) {
            if(selected){error="传送带观测包含多个信息码，无法唯一绑定高度";return false;}
            selected=&m;
        }
    }
    if(!reference || !selected){error="未识别已登记的传送带信息码；公共定位码不能确定高度";return false;}
    if(selected->operation_profile!=profile){error="识别到的机床类型与当前传送带任务不匹配: "+selected->machine_id;return false;}
    out.machine=*selected;out.reference_height_m=reference->qr_plane_height_m;
    out.delta_height_m=selected->qr_plane_height_m-reference->qr_plane_height_m;
    out.plane_z_m=reference_z-(waist-reference_waist)+out.delta_height_m;
    return std::isfinite(out.plane_z_m);
}
}
