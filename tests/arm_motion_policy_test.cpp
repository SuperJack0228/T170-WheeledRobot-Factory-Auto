#include "support/reference_arm.h"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <stdexcept>
#include <unistd.h>
using Pose=Eigen::Matrix<double,1,6>;
void require(bool ok,const std::string &message){if(!ok)throw std::runtime_error(message);}
Pose pose(const Eigen::Matrix4d &t) {
    Pose p;p.head<3>()=t.topRightCorner<3,1>().transpose();
    p(4)=atan2(-t(2,0),hypot(t(0,0),t(1,0)));p(3)=atan2(t(2,1),t(2,2));p(5)=atan2(t(1,0),t(0,0));return p;
}
void qp_test() {
    std::mt19937 rng(170);std::normal_distribution<double> normal;
    for(int trial=0;trial<100;++trial) {
        Eigen::Matrix<double,7,7> a;Eigen::Matrix<double,7,1> g;
        for(int i=0;i<7;++i){g(i)=3*normal(rng);for(int j=0;j<7;++j)a(i,j)=normal(rng);}
        Eigen::Matrix<double,7,7> h=a.transpose()*a+Eigen::Matrix<double,7,7>::Identity()*.1;
        ArmJoints low=ArmJoints::Constant(-.2),high=ArmJoints::Constant(.3),x;
        if(trial%3==0)low(1)=high(1)=0;
        require(t170::motion::solve_box_qp(h,g,low,high,x),"box QP failed");
        const Eigen::Matrix<double,7,1> gradient=h*x.transpose()+g;
        for(int j=0;j<7;++j) {
            require(x(j)>=low(j)-1e-9 && x(j)<=high(j)+1e-9,"QP violated bounds");
            if(high(j)-low(j)<1e-12)continue;
            require(x(j)<=low(j)+1e-8?gradient(j)>=-1e-7:
                x(j)>=high(j)-1e-8?gradient(j)<=1e-7:std::abs(gradient(j))<1e-7,"QP KKT optimality");
        }
    }
}
void bind_limits(Arm &arm) {
    const auto &d=arm.definition();const ArmJoints a=arm.to_device(d.minimum),b=arm.to_device(d.maximum);
    ArmJoints low=a.cwiseMin(b),high=a.cwiseMax(b);
    if(d.side=="left")low(6)=std::max(low(6),-M_PI/2);
    arm.install_device_limits(low,high);
    const auto bounds=arm.effective_limits();
    require(bounds.device_read,"driver bounds not installed");
    require((bounds.minimum.array()>=d.minimum.array()).all() && (bounds.maximum.array()<=d.maximum.array()).all(),"limits enlarged model range");
    if(d.side=="left")require(std::abs(bounds.minimum(6)+M_PI/2)<1e-12,"left J7 driver bound not intersected");
    bool rejected=false;try{arm.install_device_limits(high,low);}catch(const std::runtime_error &){rejected=true;}
    require(rejected,"invalid bounds accepted");
    require(arm.effective_limits().minimum==bounds.minimum,"failed update changed limits");
}
void jacobian_test(Arm &arm) {
    const auto &d=arm.definition();ArmJoints q=d.home;
    for(int sample=0;sample<5;++sample) {
        q=d.home;for(int j=0;j<7;++j)q(j)+=.03*sin((sample+1)*(j+1));
        const auto t=arm.Forward_Kinematics(q);const auto jac=arm.Jacobian(q);
        for(int j=0;j<7;++j) {
            auto next=q;next(j)+=1e-7;const auto n=arm.Forward_Kinematics(next);
            const Eigen::Vector3d linear=(n.topRightCorner<3,1>()-t.topRightCorner<3,1>())/1e-7;
            const Eigen::AngleAxisd aa(n.topLeftCorner<3,3>()*t.topLeftCorner<3,3>().transpose());
            require((jac.block<3,1>(0,j)-linear).norm()<1e-6,"MDH analytic translation Jacobian mismatch");
            require((jac.block<3,1>(3,j)-aa.angle()*aa.axis()/1e-7).norm()<1e-6,"MDH analytic rotation Jacobian mismatch");
        }
    }
}
void field_line(Arm &arm,const std::string &name,const ArmJoints &degrees,const Eigen::Vector3d &xyz,bool hold_rotation=true) {
    const ArmJoints start=degrees*M_PI/180;Pose target=pose(arm.Forward_Kinematics(start));target.head<3>()=xyz.transpose();
    if(!hold_rotation)target.tail<3>().setZero();
    Eigen::MatrixXd path;arm.Cart_Linear_Velocity=.2;const int rc=arm.Line_Trajectory(start,target,path);
    std::cout<<name<<" rc="<<rc<<" rows="<<path.rows()<<'\n';require(rc==0,"field regression: "+name);
    require((path.row(0)-start).norm()<1e-12,"field path changed initial joints");
    for(int i=0;i<path.rows();++i)require(!arm.Check_Joint_Limit(path.row(i)),"field trajectory outside effective range");
    const auto end=arm.Forward_Kinematics(path.bottomRows(1));
    require((end.topRightCorner<3,1>()-xyz).norm()<.0005,"field path endpoint translation");
    require(Eigen::AngleAxisd(end.topLeftCorner<3,3>()*t170::motion::arm_pose_transform(target).topLeftCorner<3,3>().transpose()).angle()<.005,"field path endpoint rotation");
}
int main(int argc,char **argv) {
 try {
    require(argc==3,"runtime/repository required");qp_test();
    const std::filesystem::path runtime=argv[1],root=argv[2];
    for(const std::string id:{"t170c_v2_3kg","t170c_v2_5kg"}) {
        const auto temp=std::filesystem::temp_directory_path()/("t170-policy-test-"+id+"-"+std::to_string(getpid()));
        std::filesystem::create_directories(temp);
        auto model=YAML::LoadFile((root/"config/robot_models"/id/"Robot_Model.yaml").string());
        const auto actual=YAML::LoadFile((runtime/"config/Robot_Model.yaml").string());
        for(const auto side:{"left","right"}) {
            if(id=="t170c_v2_3kg")model["T7_170"][side]["tool"]=YAML::Clone(actual["T7_170"][side]["tool"]);
            else {model["T7_170"][side]["tool"]=std::vector<double>{.26,0,0,0,0,0};}
        }
        auto binding=YAML::LoadFile((runtime/"config/arm_binding.yaml").string());binding["model_id"]=id;
        binding["motion_policy"]=YAML::LoadFile((root/"config/robot_models"/id/"motion_policy.yaml").string());
        {std::ofstream out(temp/"Robot_Model.yaml");out<<model;}
        {std::ofstream out(temp/"arm_binding.yaml");out<<binding;}
        auto dr=t170::motion::load_arm_definition((temp/"Robot_Model.yaml").string(),"right");
        auto dl=t170::motion::load_arm_definition((temp/"Robot_Model.yaml").string(),"left");
        Arm right(dr,std::make_shared<ReferenceBackend>(dr)),left(dl,std::make_shared<ReferenceBackend>(dl));
        bind_limits(right);bind_limits(left);jacobian_test(right);jacobian_test(left);
        for(Arm *arm:{&right,&left}) {
            const auto &d=arm->definition();const auto home=arm->Forward_Kinematics(d.home);
            Eigen::Vector3d xyz(.45,d.hand==1?-.35:.35,-.1);
            // 3 kg Home J2 is 8 deg more outward, so the nominal TCP shifts with it.
            if(id=="t170c_v2_3kg") xyz<<0.44899191,d.hand==1?-0.36509325:0.36509325,-0.07686968;
            require((home.topRightCorner<3,1>()-xyz).norm()<1e-4,"Home configured FK mismatch");
            require(arm->elbow_position(d.home).z()<-.20,"Home elbow must be below shoulder");
            for(double delta:{-.10,.10}) {
                ArmJoints start=d.home;start(0)+=delta;start(3)-=.1;Eigen::MatrixXd path;
                require(arm->Joint_Trajectory(start,d.home,path)==0,"joint Home planning");
                require((path.bottomRows(1)-d.home).norm()<1e-12,"Home endpoint depends on starting pose");
            }
            Pose ready;ready<<.45,d.hand==1?-.30:.30,-.25,0,55*M_PI/180,(d.hand==1?40:-40)*M_PI/180;
            Eigen::MatrixXd approach;arm->Cart_Linear_Velocity=.20;
            require(arm->Line_Trajectory(d.home,ready,approach)==0,"Home to grasp-ready line failed");
            std::cout<<id<<' '<<d.side<<" home_to_ready_s="<<(approach.rows()-1)*.005<<'\n';
            require((approach.rows()-1)*.005<15,"normal preparation timing regression");
        }
        // Equal reflected task targets, deliberately different starting seeds.
        for(double z:{-.10,-.20,-.25}) {
            Pose pr;pr<<.45,-.30,z,0,.6,.4;Pose pl=pr;pl(1)=-pr(1);pl(3)=-pr(3);pl(5)=-pr(5);
            Eigen::RowVectorXd qr=dr.home,ql=dl.home;qr(2)+=.25;ql(0)+=.15;
            require(right.Inverse_Kinematics(pr,qr)==0 && left.Inverse_Kinematics(pl,ql)==0,"mirrored task inverse failed");
            Eigen::Vector3d er=right.elbow_position(qr),el=left.elbow_position(ql);el.y()=-el.y();
            std::cout<<id<<" mirrored_elbow_difference_m="<<(er-el).norm()<<'\n';
            require((er-el).norm()<.035,"mirrored task chose inconsistent elbow branches");
            require(er.z()<-.05 && el.z()<-.05,"low industrial task flipped elbow up");
        }
        if(id=="t170c_v2_3kg") {
            const ArmJoints rstart=(ArmJoints()<<54.3,-61.3,-42.3,73.4,20.6,-45.3,68.4).finished()*M_PI/180;
            const ArmJoints lstart=(ArmJoints()<<81,63,44.8,66.9,3.6,-53.6,-89.9).finished()*M_PI/180;
            for(auto entry:{std::pair<Arm *,ArmJoints>{&right,rstart},{&left,lstart}}) {
                Eigen::MatrixXd home_path;
                require(entry.first->Joint_Trajectory(entry.second,entry.first->definition().home,home_path)==0,"field-start fixed joint Home failed");
                std::cout<<"field_joint_home "<<entry.first->definition().side<<" nominal_s="<<(home_path.rows()-1)*.005<<'\n';
                require((home_path.rows()-1)*.005<8,"field Home timing regression");
                require((home_path.bottomRows(1)-entry.first->definition().home).norm()<1e-12,"field Home selected another endpoint");
            }
            field_line(right,"belt_xy_right",(ArmJoints()<<56.9,-64,-39.4,82,21.1,-57.4,65.3).finished(),{.45,-.25,-.2031});
            field_line(left,"belt_xy_left",(ArmJoints()<<52.2,60.6,30.6,82.2,-10.1,-58.8,-73.1).finished(),{.45,.25,-.2030});
            field_line(left,"home_lift_left",(ArmJoints()<<52.9,60.8,30.8,82.2,-10,-59,-73.5).finished(),{.4513,.2992,-.1});
            field_line(left,"home_z_left_driver_bound",(ArmJoints()<<55.1,63.8,32.3,73.1,-9.2,-49.5,-78.1).finished(),{.45,.35,-.1});
            field_line(left,"home_rotate_left",(ArmJoints()<<81,63,44.8,66.9,3.6,-53.6,-89.9).finished(),{.4713,.3778,-.1},false);
        }
        std::filesystem::remove_all(temp);
    }
    std::cout<<"Bound QP KKT, effective limits, analytic Jacobian, fixed Home, mirrored elbows and field line replays passed; reference FK only\n";
 }catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
}
