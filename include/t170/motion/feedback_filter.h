#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
namespace t170::motion {
// Entire MODEL-angle frames are accepted atomically. A zero angle is legal;
// only nonfinite/out-of-range/physically discontinuous frames are rejected.
class JointFeedbackFilter {
public:
    using Joints=Eigen::Matrix<double,1,7>;
    bool accept(const Joints &q,double time,const Joints &low,const Joints &high,double velocity) {
        if(!q.allFinite() || !std::isfinite(time) || !std::isfinite(velocity) || velocity<=0 ||
           (q.array()<low.array()-.02).any() || (q.array()>high.array()+.02).any())return false;
        if(valid_) {
            const double dt=time-time_;
            if(dt<0 || (q-last_).cwiseAbs().maxCoeff()>.02+2*velocity*dt)return false;
        }
        last_=q;time_=time;valid_=true;return true;
    }
private:
    Joints last_=Joints::Zero();double time_=0;bool valid_=false;
};
}
