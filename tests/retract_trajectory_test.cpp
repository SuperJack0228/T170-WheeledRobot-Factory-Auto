#include "servo/trajectory.h"
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace wrist_servo;
void require(bool ok,const char *message) {
    if(!ok) throw std::runtime_error(message);
}
void bounded_samples(const Eigen::MatrixXd &q,double dt,const ConnectorLimits &limits) {
    Eigen::RowVectorXd v=Eigen::RowVectorXd::Zero(q.cols()),a=v;
    for(int i=1;i<q.rows();++i) {
        const Eigen::RowVectorXd next_v=(q.row(i)-q.row(i-1))/dt;
        const Eigen::RowVectorXd next_a=(next_v-v)/dt;
        if(i>1) require(next_a.cwiseAbs().maxCoeff()<=limits.acceleration,"sample acceleration unbounded");
        if(i>2) require(((next_a-a)/dt).cwiseAbs().maxCoeff()<=limits.jerk,"sample jerk unbounded");
        require(next_v.cwiseAbs().maxCoeff()<=limits.speed,"sample velocity unbounded");
        v=next_v; a=next_a;
    }
}
int main() {
    try {
        constexpr double dt=.005;
        ConnectorLimits limits;
        const Eigen::RowVectorXd held=Eigen::RowVectorXd::Zero(7);
        Eigen::MatrixXd path=Eigen::MatrixXd::Zero(4,7);
        path.col(0).setConstant(.4*std::acos(-1)/180);
        const auto connected=connect_held_start(path,held,dt,limits);
        require(connected.rows()>path.rows(),"sub-degree jump was silently accepted");
        require((connected.row(0)-held).norm()<1e-12,"start does not match hold");
        require((connected.bottomRows(1)-path.bottomRows(1)).norm()<1e-12,"goal changed");
        require((connected.row(1)-held).cwiseAbs().maxCoeff()<1e-4,"first command still jumps");
        bounded_samples(connected,dt,limits);

        const auto unchanged=connect_held_start(Eigen::MatrixXd::Zero(4,7),held,dt,limits);
        require(unchanged.rows()==4,"aligned SDK start gained an unnecessary pause");
        Eigen::MatrixXd native=Eigen::MatrixXd::Zero(8,7);
        for(int i=0;i<native.rows();++i) native(i,0)=.5*12*(i*dt)*(i*dt);
        require(connect_held_start(native,held,dt,limits).rows()==native.rows(),
                "clean SDK-native start was unnecessarily retimed by connector acceleration limits");
        // Mimic the old forced-first-row patch hiding the jump in row one.
        path.row(0)=held;
        const auto prefix=connect_held_start(path,held,dt,limits);
        require(prefix.rows()>4,"hidden row-one branch jump was not repaired");
        bounded_samples(prefix,dt,limits);

        JointBoundary from{held,held,held},to{held,held,held};
        from.v(0)=.03; from.a(0)=.05;
        to.q(0)=.01; to.v(0)=.04; to.a(0)=-.04;
        const auto blend=connect_joint_boundaries(from,to,dt,limits);
        require(blend.rows()>3,"moving-boundary connector failed");
        require((blend.row(0)-from.q).norm()<1e-12 &&
                (blend.bottomRows(1)-to.q).norm()<1e-12,"connector endpoints changed");
        // Six-point one-sided derivatives are exact for a degree-five curve.
        // First-order differences at 5ms would measure a different instant and
        // incorrectly label a valid quintic boundary as an acceleration jump.
        auto derivative=[&](bool end,bool second) -> Eigen::RowVectorXd {
            const double first_weights[]={-137,300,-300,200,-75,12};
            const double second_weights[]={45,-154,214,-156,61,-10};
            Eigen::RowVectorXd value=held;
            for(int k=0;k<6;++k) value+=(second ? second_weights[k] : first_weights[k])*
                blend.row(end ? blend.rows()-1-k : k);
            return value/(second ? 12*dt*dt : (end ? -60 : 60)*dt);
        };
        require(blend.rows()>=6,"too few samples for endpoint verification");
        require((derivative(false,false)-from.v).norm()<1e-8 &&
                (derivative(true,false)-to.v).norm()<1e-8,"velocity seam mismatch");
        require((derivative(false,true)-from.a).norm()<1e-7 &&
                (derivative(true,true)-to.a).norm()<1e-7,"acceleration seam mismatch");
        require(connect_joint_boundaries(from,to,dt,limits,[](const auto &){return false;}).rows()==0,
                "clearance rejection ignored");
        path.col(0).setConstant(.06);
        require(connect_held_start(path,held,dt,limits).rows()==0,"stale start accepted");
        path(0,0)=std::numeric_limits<double>::quiet_NaN();
        require(connect_held_start(path,held,dt,limits).rows()==0,"NaN accepted");
        require(connect_held_start(Eigen::MatrixXd::Zero(4,7),held,0,limits).rows()==0,"zero dt accepted");
        limits.speed=std::numeric_limits<double>::quiet_NaN();
        require(connect_held_start(Eigen::MatrixXd::Zero(4,7),held,dt,limits).rows()==0,"NaN limits accepted");
        std::cout << "retract_trajectory: sub-degree handoff, hidden prefix jump, C2 boundaries, derivative bounds and rejection passed\n";
    } catch(const std::exception &e) {
        std::cerr << "retract_trajectory FAIL: " << e.what() << '\n'; return 1;
    }
}
