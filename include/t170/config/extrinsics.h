#pragma once
#include <array>
#include <string>

// Hand-eye YAML stores translation in millimetres. The returned row-major
// camera-to-parent transform uses metres; callers must not convert it again.
// Failure leaves the caller's last valid transform unchanged.
bool load_cam2robot_matrix(const std::string &path, std::array<double,16> &out, std::string &error);
