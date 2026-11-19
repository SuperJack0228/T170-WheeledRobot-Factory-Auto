/// 交付自检：正逆解往返、不可达、点头补偿符号。
/// 通过：退出码 0。失败：退出码 1。
#include "LowerBody.h"

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>

namespace {

bool JointsClose(const LowerBody::Joints5& a, const LowerBody::Joints5& b, double eps = 1e-9)
{
    if (!a.allFinite() || !b.allFinite()) {
        return false;
    }
    for (int i = 0; i < LowerBody::Dof; ++i) {
        if (std::abs(LowerBody::WrapPi(a(i) - b(i))) > eps) {
            return false;
        }
    }
    return true;
}

LowerBody::Joints5 Q(double ankle, double knee, double hip, double roll, double yaw)
{
    LowerBody::Joints5 q;
    q << ankle, knee, hip, roll, yaw;
    return q;
}

}  // namespace

int main()
{
    std::srand(1);
    const LowerBody kin;
    const LowerBody::Params p = kin.params();

    struct Case {
        const char* name;
        LowerBody::Joints5 q;
    };

    const Case cases[] = {
        {"站立零位", Q(0, 0, 0, 0, 0)},
        {"只下蹲", Q(0.8, -1.5, LowerBody::WrapPi(-0.8 + 1.5), 0, 0)},
        {"只前点头", Q(0, 0, -M_PI / 9, 0, 0)},
        {"只后仰", Q(0, 0, M_PI / 12, 0, 0)},
        {"只侧倾", Q(0, 0, 0, M_PI / 18, 0)},
        {"只转腰", Q(0, 0, 0, 0, 5 * M_PI / 36)},
        {"下蹲+前点头", Q(0.7, -1.3, LowerBody::WrapPi(-0.7 + 1.3 - M_PI / 9), 0, 0)},
        {"下蹲+点头+侧倾+转腰",
         Q(0.6, -1.2, LowerBody::WrapPi(-0.6 + 1.2 - M_PI / 12), -2 * M_PI / 45, M_PI / 9)},
    };

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "L1=" << p.L1 << " L2=" << p.L2 << " d2=" << p.d2 << " m\n\n";

    int n_fail = 0;
    for (const auto& c : cases) {
        const Eigen::Matrix4d T = kin.Forward_Kinematics(c.q);
        const LowerBody::Joints5 q_ik = kin.Inverse_Kinematics(T);
        const Eigen::Matrix4d T2 = kin.Forward_Kinematics(q_ik);
        const bool ok = JointsClose(c.q, q_ik) && (T2 - T).norm() < 1e-12;
        if (!ok) {
            ++n_fail;
        }
        std::cout << (ok ? "[OK] " : "[FAIL] ") << c.name << "\n  q(rad) ";
        for (int i = 0; i < LowerBody::Dof; ++i) {
            std::cout << c.q(i) << (i + 1 < LowerBody::Dof ? " " : "");
        }
        std::cout << "\n  IK  ";
        for (int i = 0; i < LowerBody::Dof; ++i) {
            std::cout << q_ik(i) << (i + 1 < LowerBody::Dof ? " " : "");
        }
        std::cout << "\n  T.p=[" << T(0, 3) << ", " << T(1, 3) << ", " << T(2, 3)
                  << "]  ||T-T2||=" << (T2 - T).norm() << "\n\n";
    }

    {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T(2, 3) = p.L1 + p.L2 + 0.5;
        const auto q = kin.Inverse_Kinematics(T);
        const bool nan_ok = !LowerBody::Reachable(q);
        if (!nan_ok) {
            ++n_fail;
        }
        std::cout << (nan_ok ? "[OK] " : "[FAIL] ") << "超长不可达\n";
    }

    {
        int n = 0;
        double max_q = 0.0, max_T = 0.0;
        for (int i = 0; i < 200; ++i) {
            const double qa = 0.15 + 0.9 * (std::rand() / double(RAND_MAX));
            const double qk = -(0.3 + 1.4 * (std::rand() / double(RAND_MAX)));
            const double pitch = (std::rand() / double(RAND_MAX) - 0.5) * 0.8;
            const double roll = (std::rand() / double(RAND_MAX) - 0.5) * 0.4;
            const double yaw = (std::rand() / double(RAND_MAX) - 0.5) * 0.8;
            const LowerBody::Joints5 q =
                Q(qa, qk, LowerBody::WrapPi(-qa - qk - pitch), roll, yaw);
            if (kin.Forward_Hip(qa, qk).x() > 0.02) {
                continue;
            }
            const Eigen::Matrix4d T = kin.Forward_Kinematics(q);
            const LowerBody::Joints5 qi = kin.Inverse_Kinematics(T);
            if (!qi.allFinite()) {
                ++n_fail;
                break;
            }
            for (int k = 0; k < LowerBody::Dof; ++k) {
                max_q = std::max(max_q, std::abs(LowerBody::WrapPi(q(k) - qi(k))));
            }
            max_T = std::max(max_T, (kin.Forward_Kinematics(qi) - T).norm());
            ++n;
        }
        const bool ok = max_q < 1e-9 && max_T < 1e-12;
        if (!ok) {
            ++n_fail;
        }
        std::cout << (ok ? "[OK] " : "[FAIL] ") << "随机" << n << "组 最大角误差=" << max_q
                  << " 最大T误差=" << max_T << "\n";
    }

    {
        const LowerBody::Joints5 q = Q(0, 0, -M_PI / 9, 0, 0);
        const Eigen::Matrix4d T = kin.Forward_Kinematics(q);
        double cx = 0.0, cz = 0.0;
        kin.Inverse_Waist(T.block<3, 3>(0, 0), cx, cz);
        const bool sign_ok = cx > 0.0 && cz < 0.0;
        if (!sign_ok) {
            ++n_fail;
        }
        std::cout << (sign_ok ? "[OK] " : "[FAIL] ") << "前点头 cx=" << cx * 1e3
                  << " mm  cz=" << cz * 1e3 << " mm\n";
    }

    std::cout << "\n==== " << (n_fail == 0 ? "全部通过" : "有失败") << " fail=" << n_fail
              << " ====\n";
    return n_fail == 0 ? 0 : 1;
}
