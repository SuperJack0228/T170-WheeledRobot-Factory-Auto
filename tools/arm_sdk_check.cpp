// Calculation only: this executable has NO device/CAN/camera dependency.
#include "t170/motion/arm.h"
#include <Eigen/Geometry>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <random>
#include <thread>
#include <atomic>

namespace {
Eigen::Matrix4d reference_flange(const t170::motion::ArmDefinition &d,const t170::motion::ArmJoints &q) {
    Eigen::Matrix4d t=Eigen::Matrix4d::Identity();
    for(int i=0;i<7;++i) {
        const double al=d.mdh(i,0),a=d.mdh(i,1),di=d.mdh(i,2),th=q(i)+d.mdh(i,3);
        Eigen::Matrix4d m;m<<cos(th),-sin(th),0,a,sin(th)*cos(al),cos(th)*cos(al),-sin(al),-sin(al)*di,
            sin(th)*sin(al),cos(th)*sin(al),cos(al),cos(al)*di,0,0,0,1;t*=m;
    }return t;
}
Eigen::Matrix<double,1,6> pose(const Eigen::Matrix4d &t) {
    Eigen::Matrix<double,1,6> p;p.head<3>()=t.topRightCorner<3,1>().transpose();
    const auto r=t.topLeftCorner<3,3>();p(4)=std::atan2(-r(2,0),std::hypot(r(0,0),r(1,0)));
    p(3)=std::atan2(r(2,1),r(2,2));p(5)=std::atan2(r(1,0),r(0,0));return p;
}
}
int main(int argc,char **argv) {
    try {
        if(argc!=2)throw std::runtime_error("Usage: t170_arm_sdk_check /resolved/runtime/config (cwd must be that runtime)");
        const auto model=std::filesystem::path(argv[1])/"Robot_Model.yaml";
        const auto first=t170::motion::load_arm_definition(model.string(),"right");
        std::cout<<"# calculation_only=1 backend=Ti5_Robotics model="<<first.model_id<<" joint_coordinates=sdk_mdh_model_rad\n";
        std::cout<<"side,axis,delta_m,code,solve_ms,position_error_m,rotation_error_rad,max_joint_change_rad\n"<<std::setprecision(12);
        int failures=0;
        for(const std::string side:{"left","right"}) {
            Arm arm("T7_170",first.model_id,side,model.string());const auto &d=arm.definition();
            const t170::motion::ArmJoints seed=(d.minimum+d.maximum)*.5;
            std::mt19937 random(124);std::uniform_real_distribution<double> uniform(0,1);
            double maximum_fk_error=0;
            for(int sample=0;sample<100;++sample) {
                t170::motion::ArmJoints q;for(int j=0;j<7;++j)q(j)=d.minimum(j)+uniform(random)*(d.maximum(j)-d.minimum(j));
                const auto reference=reference_flange(d,q);
                maximum_fk_error=std::max(maximum_fk_error,(arm.Flange_Transform(q)-reference).cwiseAbs().maxCoeff());
                maximum_fk_error=std::max(maximum_fk_error,(arm.Forward_Kinematics(q)-reference*t170::motion::arm_pose_transform(d.tool)).cwiseAbs().maxCoeff());
            }
            std::cout<<"# side="<<side<<" fk_and_flange_max_matrix_error="<<maximum_fk_error<<'\n';
            if(maximum_fk_error>1e-9)++failures;
            const auto origin=arm.Forward_Kinematics(seed);
            for(int axis=0;axis<3;++axis)for(double delta:{-.005,.005}) {
                auto target=origin;target(axis,3)+=delta;auto p=pose(target);Eigen::RowVectorXd q=seed;
                const auto began=std::chrono::steady_clock::now();
                const int rc=arm.Inverse_Kinematics_Numeric(p,q);
                const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
                const auto actual=arm.Forward_Kinematics(q);
                const double xyz=(actual.topRightCorner<3,1>()-target.topRightCorner<3,1>()).norm();
                const double angle=Eigen::AngleAxisd(target.topLeftCorner<3,3>()*actual.topLeftCorner<3,3>().transpose()).angle();
                std::cout<<side<<','<<axis<<','<<delta<<','<<rc<<','<<ms<<','<<xyz<<','<<angle<<','<<(q-seed).cwiseAbs().maxCoeff()<<'\n';
                if(rc!=0||xyz>.0005||angle>.005)++failures;
            }
            auto same=pose(origin);Eigen::MatrixXd path;
            if(arm.Line_Trajectory(seed,same,path)!=0 || path.rows()<2 || (path.row(0)-seed).norm()>1e-12)++failures;
        }
        Arm left("T7_170",first.model_id,"left",model.string()),right("T7_170",first.model_id,"right",model.string());
        std::atomic<bool> concurrent_ok{true};
        auto repeat=[&](Arm &arm) {
          try {
            const auto &d=arm.definition();const t170::motion::ArmJoints q=(d.minimum+d.maximum)*.5;
            const auto expected=arm.Forward_Kinematics(q);
            for(int i=0;i<1000;++i)if((arm.Forward_Kinematics(q)-expected).cwiseAbs().maxCoeff()>1e-12)concurrent_ok=false;
          } catch(const std::exception &) {concurrent_ok=false;}
        };
        std::thread l([&]{repeat(left);}),r([&]{repeat(right);});l.join();r.join();
        std::cout<<"# concurrent_dual_instance_fk="<<(concurrent_ok?"pass":"fail")<<'\n';
        if(!concurrent_ok)++failures;
        return failures?1:0;
    }catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 2;}
}
