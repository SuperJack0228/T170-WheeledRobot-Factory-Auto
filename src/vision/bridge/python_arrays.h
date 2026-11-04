#pragma once
#include "t170/common/frame.h"
#include <pybind11/numpy.h>
#include <stdexcept>
namespace vision_bridge_detail {
namespace py = pybind11;
inline py::array bgr_vector_to_numpy(int h, int w, const std::vector<uint8_t> &bgr)
{
    const size_t expect = static_cast<size_t>(h) * static_cast<size_t>(w) * 3u;
    if (static_cast<size_t>(bgr.size()) != expect)
        throw std::runtime_error("color_bgr size mismatch");
    return py::array_t<uint8_t>(
        {h, w, 3},
        {static_cast<py::ssize_t>(w * 3), static_cast<py::ssize_t>(3), static_cast<py::ssize_t>(1)},
        bgr.data());
}

inline py::array depth_vector_to_numpy(int h, int w, const std::vector<float> &depth_m)
{
    const size_t expect = static_cast<size_t>(h) * static_cast<size_t>(w);
    if (static_cast<size_t>(depth_m.size()) != expect)
        throw std::runtime_error("depth_m size mismatch");
    return py::array_t<float>(
        {h, w},
        {static_cast<py::ssize_t>(w * sizeof(float)), static_cast<py::ssize_t>(sizeof(float))},
        depth_m.data());
}

inline py::array K_to_numpy(const std::array<double, 9> &K)
{
    py::array_t<double> arr({3, 3});
    auto buf = arr.mutable_unchecked<2>();
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            buf(r, c) = K[static_cast<size_t>(r * 3 + c)];
    return arr;
}

inline py::array dist_to_numpy(const std::array<double, 5> &dist)
{
    py::array_t<double> arr(5);
    auto buf = arr.mutable_unchecked<1>();
    for (int i = 0; i < 5; ++i)
        buf(i) = dist[static_cast<size_t>(i)];
    return arr;
}

}
