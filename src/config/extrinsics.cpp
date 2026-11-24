#include "t170/config/extrinsics.h"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <stdexcept>

bool load_cam2robot_matrix(const std::string &path,std::array<double,16> &out,std::string &error) {
    error.clear();
    try {
        const auto root=YAML::LoadFile(path);
        const auto matrix=root["a. matrix"]?root["a. matrix"]:root["matrix"];
        if(!matrix || !matrix.IsSequence())throw std::runtime_error("缺少 matrix 或 a. matrix 数组");
        std::array<double,16> candidate{};
        if(matrix.size()==4) {
            for(int r=0;r<4;++r) {
                if(!matrix[r].IsSequence() || matrix[r].size()!=4)
                    throw std::runtime_error("matrix 必须是 4×4 数组或 16 项数组");
                for(int c=0;c<4;++c)candidate[r*4+c]=matrix[r][c].as<double>();
            }
        } else if(matrix.size()==16) {
            for(int i=0;i<16;++i)candidate[i]=matrix[i].as<double>();
        } else throw std::runtime_error("matrix 必须是 4×4 数组或 16 项数组");
        for(double v:candidate)if(!std::isfinite(v))throw std::runtime_error("matrix 含非有限数值");
        for(int i=12;i<16;++i)if(std::abs(candidate[i]-(i==15?1.:0.))>1e-6)
            throw std::runtime_error("matrix 最后一行应为 [0, 0, 0, 1]");
        // Preserve the existing file contract and rotation values. Newline and
        // scalar formatting changes must not change physical calibration.
        for(int i:{3,7,11})candidate[i]/=1000.;
        out=candidate;return true;
    } catch(const std::exception &e) {
        error="手眼配置读取失败 ["+path+"]: "+e.what();return false;
    }
}
