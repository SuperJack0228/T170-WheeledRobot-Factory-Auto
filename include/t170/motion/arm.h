#pragma once
#include <Eigen/Dense>
#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace t170::motion {
using ArmJoints = Eigen::Matrix<double,1,7>;
struct ArmDefinition {
    int hand = 1;
    std::string model_id, side, config_path;
    std::vector<int> can_ids;
    ArmJoints motor_direction = ArmJoints::Ones();
    ArmJoints motor_offset = ArmJoints::Zero();
    ArmJoints minimum = ArmJoints::Constant(-3.14), maximum = ArmJoints::Constant(3.14);
    Eigen::Matrix<double,7,4> mdh = Eigen::Matrix<double,7,4>::Zero();
    Eigen::Matrix<double,1,6> tool = Eigen::Matrix<double,1,6>::Zero();
    double period = .005, velocity = .9, acceleration = 2.5, jerk = 20;
    bool ideal_t7 = false;
};
// SDK implementation is private to vendor_arm.cpp. Test backends are explicitly
// injected; the production constructor always creates the vendor backend.
class ArmBackend {
public:
    virtual ~ArmBackend() = default;
    virtual Eigen::Matrix4d forward(const ArmJoints &) const = 0;
    virtual Eigen::Matrix4d flange(const ArmJoints &) const = 0;
    virtual int inverse(const Eigen::Matrix4d &, ArmJoints &, std::optional<double>) = 0;
};
ArmDefinition load_arm_definition(const std::string &model_file,const std::string &side);
const ArmDefinition &arm_definition(int hand);
ArmJoints model_to_device(int hand,const ArmJoints &q);
ArmJoints device_to_model(int hand,const ArmJoints &q);
Eigen::Matrix4d arm_pose_transform(const Eigen::Matrix<double,1,6> &pose);
}

// Application-owned arm adapter. ALL public joint arguments/results are SDK
// MODEL radians, including IK seeds, trajectories, limits and held references.
// Method spellings are retained at existing call sites; this is not the legacy
// vendor class and has no binary/API dependency on Robot_Arm/Ti5_Arm_2204.so.
class Arm {
public:
    Arm(const std::string &sdk_model,const std::string &robot_model,const std::string &side,
        const std::string &model_file = "");
    Arm(t170::motion::ArmDefinition,std::shared_ptr<t170::motion::ArmBackend>);
    Arm(const Arm &) = delete;
    Arm &operator=(const Arm &) = delete;
    std::vector<int> can_id_list;
    double Cart_Linear_Velocity = .10;
    const t170::motion::ArmDefinition &definition() const { return definition_; }
    t170::motion::ArmJoints to_model(const t170::motion::ArmJoints &q) const {
        return (q-definition_.motor_offset).cwiseQuotient(definition_.motor_direction);
    }
    t170::motion::ArmJoints to_device(const t170::motion::ArmJoints &q) const {
        return q.cwiseProduct(definition_.motor_direction)+definition_.motor_offset;
    }
    Eigen::Matrix4d Forward_Kinematics(const Eigen::Ref<const Eigen::RowVectorXd> &q);
    Eigen::Matrix4d Flange_Transform(const Eigen::Ref<const Eigen::RowVectorXd> &q);
    Eigen::MatrixXd Jacobian(const Eigen::Ref<const Eigen::RowVectorXd> &q);
    bool Check_Joint_Limit(const Eigen::Ref<const Eigen::RowVectorXd> &q);
    int Inverse_Kinematics(Eigen::Matrix<double,1,6> &target,Eigen::Ref<Eigen::RowVectorXd> q);
    int Inverse_Kinematics_Numeric(Eigen::Matrix<double,1,6> &target,Eigen::Ref<Eigen::RowVectorXd> q);
    int Inverse_Kinematics_Analytic(const Eigen::Matrix<double,1,6> &target,Eigen::Ref<Eigen::RowVectorXd> q,double j2);
    int Joint_Trajectory(const Eigen::MatrixXd &start,const Eigen::MatrixXd &end,Eigen::MatrixXd &out);
    int Line_Trajectory(const Eigen::MatrixXd &start,Eigen::Matrix<double,1,6> &target,
                        Eigen::MatrixXd &out,std::optional<double> j2=std::nullopt);
private:
    int local_inverse(const Eigen::Matrix4d &,t170::motion::ArmJoints &,std::optional<double>);
    bool valid_solution(const Eigen::Matrix4d &,const t170::motion::ArmJoints &);
    t170::motion::ArmDefinition definition_;
    std::shared_ptr<t170::motion::ArmBackend> backend_;
};
