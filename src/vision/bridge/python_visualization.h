#pragma once
#include "t170/common/frame.h"
#include <pybind11/pybind11.h>
#include <cstddef>
namespace vision_bridge_detail {
std::size_t camera_slot_index(CameraSlot slot);
void show_pose_visualization(const CameraFrameData &,const pybind11::object &,int);
void show_aruco_visualization(const CameraFrameData &,const pybind11::object &,int);
void close_visualization(bool have_interpreter);
}
