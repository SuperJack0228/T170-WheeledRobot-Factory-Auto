#pragma once
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cstdint>
namespace t170::calibration {
// Reproducibility fingerprint, not a cryptographic signature.
inline std::string fingerprint(const std::filesystem::path &path) {
    std::ifstream in(path,std::ios::binary);
    if(!in) throw std::runtime_error("无法读取标定依赖: "+path.string());
    std::uint64_t h=14695981039346656037ull;char c;
    while(in.get(c)) {h^=static_cast<unsigned char>(c);h*=1099511628211ull;}
    std::ostringstream out;out<<std::hex<<h;return out.str();
}
}
