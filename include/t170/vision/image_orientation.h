#pragma once
#include "t170/common/frame.h"
#include <algorithm>
#include <stdexcept>

namespace vision {
inline int default_image_rotation(CameraSlot slot) { return slot==CameraSlot::RightHand ? 180 : 0; }
inline const char *image_convention(int rotation) { return rotation==180 ? "rgb_rotated_180" : "raw_rgb"; }

// A prepared frame declares its axes, so applying the same policy twice is safe.
inline CameraFrameData orient_frame(CameraFrameData frame,int rotation) {
    if(rotation!=0 && rotation!=180)throw std::invalid_argument("image_rotation_deg must be 0 or 180");
    if(!frame.ok || frame.image_rotation_deg==rotation)return frame;
    const auto count=static_cast<std::size_t>(frame.width)*static_cast<std::size_t>(frame.height);
    if(frame.width<=0 || frame.height<=0 || frame.color_bgr.size()!=count*3)
        throw std::invalid_argument("Invalid color frame size");
    for(std::size_t i=0;i<count/2;++i)
        for(int channel=0;channel<3;++channel)std::swap(frame.color_bgr[3*i+channel],frame.color_bgr[3*(count-1-i)+channel]);
    if(frame.depth_m.size()==count)std::reverse(frame.depth_m.begin(),frame.depth_m.end());
    frame.K[2]=frame.width-1-frame.K[2];frame.K[5]=frame.height-1-frame.K[5];
    frame.dist[2]*=-1;frame.dist[3]*=-1;
    frame.image_rotation_deg=rotation;
    return frame;
}
}
