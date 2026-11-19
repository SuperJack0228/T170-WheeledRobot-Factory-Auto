#include "support/reference_arm.h"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

using t170::motion::ArmJoints;
void check(bool v,const char *s){if(!v)throw std::runtime_error(s);}
Eigen::Matrix<double,1,6> pose(const Eigen::Matrix4d &t) {
    Eigen::Matrix<double,1,6> p;p.head<3>()=t.topRightCorner<3,1>().transpose();
    p(4)=atan2(-t(2,0),hypot(t(0,0),t(1,0)));p(3)=atan2(t(2,1),t(2,2));p(5)=atan2(t(1,0),t(0,0));return p;
}
int main(int argc,char **argv) {
    try {
        check(argc==3,"runtime and repository paths required");
        const auto runtime=std::filesystem::path(argv[1]),root=std::filesystem::path(argv[2]);
        for(const std::string id:{"t170c_v2_3kg","t170c_v2_5kg"})for(const std::string side:{"left","right"}) {
            const auto temp=std::filesystem::temp_directory_path()/("t170-model-test-"+id+side);
            std::filesystem::create_directories(temp);
            std::filesystem::copy_file(root/"config/robot_models"/id/"Robot_Model.yaml",temp/"Robot_Model.yaml",std::filesystem::copy_options::overwrite_existing);
            auto binding=YAML::LoadFile((runtime/"config/arm_binding.yaml").string());binding["model_id"]=id;
            {std::ofstream out(temp/"arm_binding.yaml");out<<binding;}
            auto d=t170::motion::load_arm_definition((temp/"Robot_Model.yaml").string(),side);
            auto backend=std::make_shared<ReferenceBackend>(d);Arm arm(d,backend);
            const ArmJoints seed=(d.minimum+d.maximum)*.5;
            check(arm.to_model(arm.to_device(seed)).isApprox(seed,1e-12),"device/model roundtrip");
            check(!arm.Check_Joint_Limit(seed),"model limits");
            const auto t=arm.Forward_Kinematics(seed);
            auto same=pose(t);Eigen::RowVectorXd q=seed;
            check(arm.Inverse_Kinematics_Numeric(same,q)==0 && (q-seed).norm()<1e-10,"same pose changed branch");
            for(int axis=0;axis<3;++axis)for(double delta:{-.005,.005}) {
                auto target=t;target(axis,3)+=delta;auto p=pose(target);q=seed;
                check(arm.Inverse_Kinematics_Numeric(p,q)==0,"seeded 5mm IK did not converge");
                check((arm.Forward_Kinematics(q).topRightCorner<3,1>()-target.topRightCorner<3,1>()).norm()<.0005,"IK endpoint error");
                Eigen::MatrixXd path;check(arm.Line_Trajectory(seed,p,path)==0,"continuous line failed");
                check((path.row(0)-seed).norm()<1e-12,"trajectory start not preserved");
                check(((path.bottomRows(path.rows()-1)-path.topRows(path.rows()-1))/d.period).cwiseAbs().maxCoeff()<=d.velocity*1.001,"velocity bound");
            }
            if(!d.ideal_t7) {
                q=seed;check(arm.Inverse_Kinematics_Analytic(same,q,seed(1))==0,"nonideal fixed-J2 fallback");
                check(backend->inverse_calls==0,"nonideal geometry used ideal analytic IK");
            }
            auto bad=same;bad(0)=10;q=seed;
            check(arm.Inverse_Kinematics(bad,q)!=0 && (q-seed).norm()<1e-12,"failed/positive native result committed invalid joints");
            std::filesystem::remove_all(temp);
        }
        // Exercise the actual resolved No.124 tool and taught model angles,
        // not only the zero-tool templates and midpoint seeds above.
        const auto taught=YAML::LoadFile((root/"config/robots/TH-T170C-0124/taught_poses.yaml").string());
        for(const std::string side:{"left","right"}) {
            auto d=t170::motion::load_arm_definition((runtime/"config/Robot_Model.yaml").string(),side);
            auto backend=std::make_shared<ReferenceBackend>(d);
            Arm arm(d,backend);
            check(std::abs(d.tool(0)-.26)<1e-12,"resolved tool translation");
            check(std::abs(d.tool(3))>.002,"3kg exact flange rotation missing");
            ArmJoints seed;
            const auto values=taught["retract_motion"]["ready_"+side+"_deg"];
            for(int i=0;i<7;++i)seed(i)=values[i].as<double>()*M_PI/180;
            check(!arm.Check_Joint_Limit(seed),"actual taught pose outside selected model limits");
            ArmJoints nearby=seed;nearby(3)+=.003;
            auto goal=pose(arm.Forward_Kinematics(nearby));Eigen::RowVectorXd q=seed;
            check(arm.Inverse_Kinematics_Numeric(goal,q)==0,"actual tool/ready pose seeded IK");
            Eigen::MatrixXd path;
            check(arm.Line_Trajectory(seed,goal,path)==0,"actual tool/ready pose continuous line");
            check((path.row(0)-seed).norm()<1e-12,"actual ready trajectory start");
        }
        std::cout<<"Both arm variants: model coordinates, seeded IK, exact trajectory start and failure isolation passed; reference backend only\n";
    }catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
}
