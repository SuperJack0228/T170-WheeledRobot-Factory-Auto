#pragma once
#include "t170/common/tray_types.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <vector>

namespace t170::calibration {
// Physical order, independent of the printed ArUco row/column numbering.
// Robot base: +X forward, +Y left. Row 1 is farthest; column 1 is leftmost.
inline wrist_servo::TrayAnchor teaching_hole(const std::vector<wrist_servo::TrayAnchor> &holes,int hand) {
    if(hand!=0 && hand!=1) throw std::runtime_error("标定手臂编号无效");
    std::map<int,std::vector<wrist_servo::TrayAnchor>> rows;
    for(const auto &h:holes) if(h.base.allFinite()) rows[h.row].push_back(h);
    if(rows.size()!=6) throw std::runtime_error("需识别完整六行网格以确定离机器人最近的第二排");
    std::vector<std::pair<double,int>> order;
    for(const auto &[row,items]:rows) {
        double x=0;for(const auto &h:items)x+=h.base.x();order.emplace_back(x/items.size(),row);
    }
    std::sort(order.rbegin(),order.rend());
    auto fifth=rows.at(order[4].second);
    if(fifth.size()!=6) throw std::runtime_error("第五行需有完整六孔网格");
    std::sort(fifth.begin(),fifth.end(),[](const auto &a,const auto &b){return a.base.y()>b.base.y();});
    const auto target=fifth[hand?4:1];
    if(target.class_id!=1) throw std::runtime_error("指定标定孔不是空孔，请清空第五行后重新识别");
    return target;
}
inline bool valid_teach_jog(double dx,double dy) {
    return std::isfinite(dx) && std::isfinite(dy) && std::hypot(dx,dy)>0 && std::hypot(dx,dy)<=.00500001;
}
}
