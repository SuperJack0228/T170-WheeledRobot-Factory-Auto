#pragma once
#include <cmath>
#include <string>
namespace vision {
struct BeltCircleConfig {
    bool enabled=false;
    double radius_m=0; // 0: bounded radius search; fill measured OUTER radius for stronger rejection
    double min_radius_m=.006,max_radius_m=.035,radius_tolerance=.20;
    double feature_above_final_m=0,search_radius_m=.025,residual_m=.001;
    double pixels_per_m=2000,max_time_ms=60;
    int min_coverage_bins=8,min_inliers=35,ransac_trials=320,saturation_threshold=250;
};
inline bool validate_belt_circle(const BeltCircleConfig &c,std::string &error) {
    const bool ok=std::isfinite(c.radius_m)&&c.radius_m>=0 && std::isfinite(c.feature_above_final_m)&&std::abs(c.feature_above_final_m)<=.15 &&
      std::isfinite(c.min_radius_m)&&std::isfinite(c.max_radius_m)&&c.min_radius_m>=.002&&c.max_radius_m>c.min_radius_m&&c.max_radius_m<=.08 &&
      (c.radius_m==0||(c.radius_m>=c.min_radius_m&&c.radius_m<=c.max_radius_m)) &&
      std::isfinite(c.radius_tolerance)&&c.radius_tolerance>0&&c.radius_tolerance<=.4 &&
      std::isfinite(c.search_radius_m)&&c.search_radius_m>0&&c.search_radius_m<=.05 &&
      std::isfinite(c.residual_m)&&c.residual_m>0&&c.residual_m<=.003 &&
      std::isfinite(c.pixels_per_m)&&c.pixels_per_m>=500&&c.pixels_per_m<=4000 &&
      std::isfinite(c.max_time_ms)&&c.max_time_ms>0&&c.max_time_ms<=100 &&
      c.min_coverage_bins>=6&&c.min_coverage_bins<=16&&c.min_inliers>=15&&c.min_inliers<=1000 &&
      c.ransac_trials>=32&&c.ransac_trials<=2000&&c.saturation_threshold>=200&&c.saturation_threshold<=255;
    if(!ok)error="invalid belt2_circle bounds/geometry";return ok;
}
}
