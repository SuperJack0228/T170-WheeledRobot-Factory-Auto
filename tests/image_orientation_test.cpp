#include "t170/vision/image_orientation.h"
#include <iostream>
#include <cmath>
#include <stdexcept>

void check(bool ok,const char *why){if(!ok)throw std::runtime_error(why);}
int main(){try {
    check(vision::default_image_rotation(CameraSlot::LeftHand)==0 &&
          vision::default_image_rotation(CameraSlot::RightHand)==180,"standard defaults reversed");
    CameraFrameData raw;raw.ok=true;raw.width=2;raw.height=2;
    raw.color_bgr={1,2,3,4,5,6,7,8,9,10,11,12};raw.depth_m={1,2,3,4};
    raw.K={100,0,.3,0,110,.4,0,0,1};raw.dist={.1,.2,.03,.04,.5};
    const auto rotated=vision::orient_frame(raw,180);
    check(rotated.color_bgr==std::vector<std::uint8_t>({10,11,12,7,8,9,4,5,6,1,2,3}),"BGR channels or pixels reversed incorrectly");
    check(rotated.depth_m==std::vector<float>({4,3,2,1}),"depth not aligned");
    check(std::abs(rotated.K[2]-.7)<1e-12 && std::abs(rotated.K[5]-.6)<1e-12 &&
          rotated.dist[2]==-.03 && rotated.dist[3]==-.04,"intrinsics not rotated");
    check(vision::orient_frame(rotated,180).color_bgr==rotated.color_bgr,"double preparation rotates twice");
    check(vision::orient_frame(rotated,0).color_bgr==raw.color_bgr,"return to raw axes failed");
    bool rejected=false;try{vision::orient_frame(raw,90);}catch(const std::invalid_argument&){rejected=true;}
    check(rejected,"invalid rotation accepted");
    std::cout<<"image orientation passed\n";return 0;
}catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}}
