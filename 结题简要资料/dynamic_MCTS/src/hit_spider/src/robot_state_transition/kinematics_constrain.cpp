//六足机器人运动学约束的实现

#include "hit_spider/robot_state_transition/kinematics_constrain.hh"
#include "hit_spider/hexapod.hh"

namespace Robot_State_Transition
{
    namespace
    {
        /**
         * @ :默单腿fixJifoot约束foot的约束矩阵
         * @description:
         * @return {*}
         */
        MatrixXX init_A_Ji_foot()
        {
            MatrixXX A(7, 3);
            A.setZero();

            A(0, 2) = 1;

            A(1, 0) = -1;
            A(1, 1) = 1.428148006742115;

            A(2, 0) = -1;
            A(2, 1) = -1.428148006742115;

            A(3, 0) = 3.171594802363213;
            A(3, 1) = -1;
            A(3, 2) = -25.769207769201120;

            A(4, 0) = 3.171594802363213;
            A(4, 1) = 1;
            A(4, 2) = -25.769207769201120;

            A(5, 0) = 3.171594802363213;
            A(5, 1) = 1;
            A(5, 2) = -2.634036361284702;

            A(6, 0) = 3.171594802363213;
            A(6, 1) = -1;
            A(6, 2) = -2.634036361284702;

            return A;
        }

        /**
         * @ :默单腿fixJifoot约束foot的右边值
         * @description:
         * @return {*}
         */
        VectorX init_b_Ji_foot()
        {
            VectorX b(7);
            b.setZero();
            b(0) = -0.28;
            b(3) = 24.480747380741064;
            b(4) = 24.480747380741064;
            b(5) = 4.353148255853780;
            b(6) = 4.353148255853780;

            return b;
        }

        /**
         * @ :默单腿foot约束fixJi的约束矩阵
         * @description:
         * @return {*}
         */
        MatrixXX init_A_foot_Ji()
        {
            MatrixXX A(7, 3);
            A.setZero();

            A(0, 2) = 1;

            A(1, 0) = 1.428148006742115;
            A(1, 1) = 1;

            A(2, 0) = -1.428148006742115;
            A(2, 1) = 1;

            A(3, 0) = 1;
            A(3, 1) = -3.171594802363213;
            A(3, 2) = -25.769207769201120;

            A(4, 0) = -1;
            A(4, 1) = -3.171594802363213;
            A(4, 2) = -25.769207769201120;

            A(5, 0) = 1;
            A(5, 1) = -3.171594802363213;
            A(5, 2) = -2.634036361284702;

            A(6, 0) = -1;
            A(6, 1) = -3.171594802363213;
            A(6, 2) = -2.634036361284702;

            return A;
        }

        /**
         * @ : 单腿foot约束fixJi的右边值
         * @description:
         * @return {*}
         */
        VectorX init_b_foot_Ji()
        {
            VectorX b(7);
            b.setZero();
            b(0) = -0.28;
            b(3) = 24.480747380741064;
            b(4) = 24.480747380741064;
            b(5) = 4.353148255853780;
            b(6) = 4.353148255853780;

            return b;
        }

    }

    const MatrixXX A_Ji_foot = init_A_Ji_foot();
    const VectorX b_Ji_foot = init_b_Ji_foot();

    const MatrixXX A_foot_Ji = init_A_foot_Ji();
    const VectorX b_foot_Ji = init_b_foot_Ji();

    std::pair<MatrixXX, VectorX> get_kinematics_con_cog_foot(const hit_spider::hexapod_Base_Pose &base_pose)
    {
        MatrixXX A_output(42, 3);
        VectorX b_output(42);

        A_output.setZero();
        b_output.setZero();

        // base描述world的齐次变换矩阵
        Eigen::Isometry3d T_world_base = getTrans_W_B(base_pose);
        Matrix3 base_world_R = T_world_base.matrix().block(0, 0, 3, 3).transpose();
        Vector3 base_world_p = -base_world_R * T_world_base.matrix().block(0, 3, 3, 1);

        for (int i = 0; i < 6; ++i)
        {
            // fixJi坐标系描述base
            Matrix3 Ji_base_R = TransMatrix_FixJi_Body[i].matrix().block(0, 0, 3, 3);
            Vector3 Ji_base_p = TransMatrix_FixJi_Body[i].matrix().block(0, 3, 3, 1);

            A_output.block(7 * i, 0, 7, 3) = A_Ji_foot * Ji_base_R * base_world_R;
            b_output.block(7 * i, 0, 7, 1) = b_Ji_foot - A_Ji_foot * (Ji_base_R * base_world_p + Ji_base_p);
        }

        return std::make_pair(A_output, b_output);
    }

    std::pair<MatrixXX, VectorX> get_kinematics_con_foot_cog(const hit_spider::hexapod_Base_Pose &base_pose,const std::vector<int> &support_leg, const MatrixX3 &contactPoints, const MatrixX3 &contactNormals)
    {
        assert(((int)support_leg.size() == (int)contactPoints.rows()) && ((int)support_leg.size() == (int)contactNormals.rows()));

        MatrixX3 A_output(7 * support_leg.size(), 3);
        VectorX b_output(7 * support_leg.size());

        A_output.setZero();
        b_output.setZero();

        // 世界系下的机器人下一步位姿，用于判断下一步fixJi的位置
        Eigen::Isometry3d T_W_B = getTrans_W_B(base_pose);
        const double base_yaw = base_pose.orientation.yaw;

        for (int i = 0; i < (int)support_leg.size(); ++i)
        {
            // 计算foot固定坐标系
            Matrix3 R_W_foot;
            R_W_foot.col(2) = -1 * contactNormals.row(i);
            R_W_foot.col(0) << cos(body_fixJi_theta[support_leg[i] - 1] + base_yaw), sin(body_fixJi_theta[support_leg[i] - 1] + base_yaw), 0;
            R_W_foot.col(0) << -1 * R_W_foot.col(0).cross(Vector3(0, 0, 1));
            R_W_foot.col(1) = R_W_foot.col(2).cross(R_W_foot.col(0));
            R_W_foot.colwise().normalize(); // 归一化

            Matrix3 R_foot_W = R_W_foot.transpose();
            Vector3 foot_W_p = -1 * R_foot_W * contactPoints.row(i).transpose();

            // 获取base坐标系下质心到fixJi的偏移
            Vector3 base_fixJi_translation = TransMatrix_Body_2_fixJi[support_leg[i] - 1].matrix().block<3, 1>(0, 3);

            // 转换为世界坐标系下质心到fixJi的偏移（只有旋转矩阵作用，没有平移！）
            Vector3 W_base_fixJi_translation = T_W_B.matrix().block<3, 3>(0, 0) * base_fixJi_translation;

            A_output.block(7 * i, 0, 7, 3) = A_foot_Ji * R_foot_W;
            b_output.block(7 * i, 0, 7, 1) = b_foot_Ji - A_foot_Ji * R_foot_W * W_base_fixJi_translation - A_foot_Ji * foot_W_p;
        }

        return std::make_pair(A_output, b_output);
    }
}
