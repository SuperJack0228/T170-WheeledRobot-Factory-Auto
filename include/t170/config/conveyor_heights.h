#pragma once
#include <string>
#include <vector>

namespace t170::config {
struct ConveyorMachineHeight {
    std::string machine_id,business_id,operation_profile;
    int locate_id=500,info_id=-1;
    double qr_plane_height_m=0;
};
struct ConveyorHeightCatalog {
    bool enabled=false;
    std::string reference_machine;
    std::vector<ConveyorMachineHeight> machines;
};
struct ConveyorHeightMatch {
    ConveyorMachineHeight machine;
    double reference_height_m=0,delta_height_m=0,plane_z_m=0;
};
ConveyorHeightCatalog load_conveyor_heights(const std::string &path);
bool same_conveyor_identity(const ConveyorHeightCatalog &,const ConveyorHeightCatalog &);
bool match_conveyor_height(const ConveyorHeightCatalog &,const std::vector<int> &observed_ids,
    const std::string &operation_profile,double reference_plane_z_m,double reference_waist_z_m,
    double waist_z_m,ConveyorHeightMatch &,std::string &error);
}
