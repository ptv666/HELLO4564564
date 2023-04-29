// 规划过程中使用的函数定义

#include "hit_spider/MCTS/planning.hh"

namespace planning
{
    namespace
    {
        /**
         * @ : 0.初始化支撑状态列表，2^6 = 64种，减去少于三条腿支撑的情况一共：42种可能支撑状态
         * @description:应该放到private中的，因为只是用一次来初始化“已知的”支撑状态集合
         * @return {*}
         */
        hexapod_SupportState_List createInitialSupportList()
        {
            hexapod_SupportState_List support_state_list(42, 6);
            hexapod_SupportState support_state;
            int ros_index = 0;
            for (int leg1 = 0; leg1 < 2; leg1++)
            {
                for (int leg2 = 0; leg2 < 2; leg2++)
                {
                    for (int leg3 = 0; leg3 < 2; leg3++)
                    {
                        for (int leg4 = 0; leg4 < 2; leg4++)
                        {
                            for (int leg5 = 0; leg5 < 2; leg5++)
                            {
                                for (int leg6 = 0; leg6 < 2; leg6++)
                                {
                                    if (leg1 + leg2 + leg3 + leg4 + leg5 + leg6 > 2)
                                    {
                                        support_state(0) = leg1;
                                        support_state(1) = leg2;
                                        support_state(2) = leg3;
                                        support_state(3) = leg4;
                                        support_state(4) = leg5;
                                        support_state(5) = leg6;
                                        support_state_list.row(ros_index++) = support_state;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            // debug
            assert(ros_index == 42);

            return support_state_list;
        }

        bool in_polygon(const hit_spider::hexapod_Base_Pose &pose_now, const MatrixXX &CWC_inputs)
        {
            MatrixXX A;
            VectorX b;
            if (!Robot_State_Transition::vertices_to_H(CWC_inputs, A, b))
            {
                return false;
            }

            point_Planar p;
            p << pose_now.position.x, pose_now.position.y;
            VectorX result = A * p;
            if ((result.array() <= b.array()).all())
            {
                return true;
            }

            return false;
        }

        void my_round_4(MatrixXX &points)
        {
            double num;
            for (int i = 0; i < points.rows(); ++i)
            {
                for (int j = 0; j < points.cols(); ++j)
                {
                    num = round(points(i, j) * 1e4);
                    points(i, j) = num / (double)1e4;
                }
            }
        }
    }

    const hexapod_SupportState_List initialSupportList = createInitialSupportList(); // 42个支撑状态构成集合

    hexapod_SupportState_List findAvailable_SupportStates(const hit_spider::hexapod_State &hexapodState)
    {
        //------支撑腿数量大于3，容错腿不能是支撑腿------
        hexapod_SupportState faultLeg;
        for (int i = 0; i < 6; ++i)
        {
            faultLeg(i) = hexapodState.faultLeg_State_Now[i]; // 当前的错误退信息决定了Next的支撑可行状态
        }

        VectorXi can_support(initialSupportList.rows());
        can_support.setZero();

        for (int i = 0; i < initialSupportList.rows(); ++i)
        {
            if (initialSupportList.row(i) * faultLeg == 0) // 容错腿不能当做支撑腿
            {
                can_support(i) = 1;
            }
        }

        hexapod_SupportState_List SupportList(can_support.sum(), 6);
        int row_index = 0;
        for (int i = 0; i < initialSupportList.rows(); ++i)
        {
            if (can_support(i) == 1)
            {
                SupportList.row(row_index++) = initialSupportList.row(i);
            }
        }

        // debug
        assert(row_index == can_support.sum());

        const hexapod_SupportState_List SupportList_temp = SupportList;
        can_support.resize(SupportList_temp.rows(), 1);
        can_support.setZero();

        //------判断哪些支撑状态是稳定的(运动学约束不用考虑，因为移动过程中考虑了运动学约束)------
        for (int i = 0; i < SupportList_temp.rows(); ++i)
        {
            int support_leg_num = 0; // 支撑腿数量
            for (int j = 0; j < 6; ++j)
            {
                support_leg_num += SupportList_temp(i, j);
            }

            std::vector<int> support_leg;                 // 支撑腿标号
            MatrixX3 contact_Points(support_leg_num, 3);  // 支撑腿接触点
            MatrixX3 contact_normals(support_leg_num, 3); // 接触点法向量
            contact_normals.setZero();
            contact_normals.col(2).setOnes(); // 先考虑简单情况

            row_index = 0;
            for (int j = 0; j < 6; ++j)
            {
                if (SupportList_temp(i, j) == 1)
                {
                    contact_Points.row(row_index++) << hexapodState.feetPositionNow.foot[j].x,
                        hexapodState.feetPositionNow.foot[j].y,
                        hexapodState.feetPositionNow.foot[j].z;
                    support_leg.push_back(j + 1);
                }
            }

            // 判断当前质心位置是否在下一步CWC内
            if (support_leg_num == 3)
            {
                // 判断三个支撑点是否共线
                MatrixX3 temp_contact_Points = contact_Points;
                temp_contact_Points.col(2).setOnes();
                double S = temp_contact_Points.determinant();
                if (S < 1e-5)
                {
                    // debug
                    //  ROS_ERROR("三个支撑点共线");
                    continue;
                }
            }
            MatrixXX CWC_A = Robot_State_Transition::comput_friction_region(contact_Points, contact_normals, 0.5, Robot_State_Transition::Mass);
            if (!in_polygon(hexapodState.base_Pose_Now, CWC_A))
            {
                continue;
            }

            // 判断当前质心位置是否满足关节力矩约束
            Vector3 c;
            c << hexapodState.base_Pose_Now.position.x, hexapodState.base_Pose_Now.position.y, hexapodState.base_Pose_Now.position.z;
            MatrixX3 joints(support_leg_num, 3);
            if ((support_leg_inverse_kin(hexapodState.base_Pose_Now, hexapodState.feetPositionNow, support_leg, joints)) && (Robot_State_Transition::is_meet_dynamic_con(c, support_leg_num, contact_Points, joints)))
            {
                can_support(i) = 1;
            }
        }

        // 更新可行支撑状态集合
        SupportList.resize(can_support.sum(), 6);
        row_index = 0;
        for (int i = 0; i < SupportList_temp.rows(); ++i)
        {
            if (can_support(i) == 1)
            {
                SupportList.row(row_index++) = SupportList_temp.row(i);
            }
        }

        // debug
        assert(row_index == can_support.sum());
        assert(SupportList.rows() == can_support.sum());

        return SupportList;
    }

    double get_CWC_base_length(const hit_spider::hexapod_State &hexapodState)
    {
        int support_leg_num = 0;
        for (int i = 0; i < 6; ++i)
        {
            support_leg_num += hexapodState.support_State_Next[i];
        }

        MatrixX3 contact_Points(support_leg_num, 3);
        MatrixX3 contact_normals(support_leg_num, 3);
        contact_normals.setZero();
        contact_normals.col(2).setOnes(); // 先考虑简单情况

        int row_index = 0;
        for (int i = 0; i < 6; ++i)
        {
            if (hexapodState.support_State_Next[i] == 1)
            {
                contact_Points.row(row_index++) << hexapodState.feetPositionNow.foot[i].x,
                    hexapodState.feetPositionNow.foot[i].y,
                    hexapodState.feetPositionNow.foot[i].z;
            }
        }

        MatrixXX CWC_A = Robot_State_Transition::comput_friction_region(contact_Points, contact_normals, 0.5, Robot_State_Transition::Mass);

        // 求解多边形对应的约束
        MatrixXX A;
        VectorX b;
        if (!Robot_State_Transition::vertices_to_H(CWC_A, A, b))
        {
            my_round_4(CWC_A);
            if (!Robot_State_Transition::vertices_to_H(CWC_A, A, b))
            {
                // debug
                ROS_ERROR("cdd不稳定!给它一次机会还是不稳定,请输入一个字符继续");
                std::cout << contact_Points << std::endl;
                std::cout << CWC_A << std::endl;
                getchar();
                return 0;
            }
        }

        // 优化求解最大CWC约束的最大步长
        MatrixXX D(1, 2);
        VectorX d(1);
        D << 0, 1;
        d << 0;
        VectorX g(2), minBound, maxBound;
        g << 1, 0;
        double CWC_reslut_length = 0.0;
        Robot_State_Transition::solve_LP_GLPK(A, b, D, d, g, minBound, maxBound, true, nullptr, &CWC_reslut_length);
        return (CWC_reslut_length - hexapodState.base_Pose_Now.position.x);
    }

    double get_kin_length(const hit_spider::hexapod_State &hexapodState)
    {
        int support_leg_num = 0;
        for (int i = 0; i < 6; ++i)
        {
            support_leg_num += hexapodState.support_State_Next[i];
        }

        std::vector<int> support_leg;
        MatrixX3 contact_Points(support_leg_num, 3);
        MatrixX3 contact_normals(support_leg_num, 3);
        contact_normals.setZero();
        contact_normals.col(2).setOnes(); // 先考虑简单情况

        int row_index = 0;
        for (int i = 0; i < 6; ++i)
        {
            if (hexapodState.support_State_Next[i] == SUPPORT)
            {
                contact_Points.row(row_index++) << hexapodState.feetPositionNow.foot[i].x,
                    hexapodState.feetPositionNow.foot[i].y,
                    hexapodState.feetPositionNow.foot[i].z;
                support_leg.push_back(i + 1);
            }
        }

        // 当前支撑腿对质心的运动约束
        auto feet_con_cog_Ab = Robot_State_Transition::get_kinematics_con_foot_cog(hexapodState.base_Pose_Now, support_leg, contact_Points, contact_normals);

        MatrixXX A(7 * support_leg_num, 3);
        VectorX b(7 * support_leg_num);
        A.setZero();
        b.setZero();
        A.block(0, 0, 7 * support_leg_num, 3) = feet_con_cog_Ab.first;
        b.block(0, 0, 7 * support_leg_num, 1) = feet_con_cog_Ab.second;

        // 优化求解最大kin约束的最大步长
        MatrixXX D(2, 3);
        VectorX d(2);
        D << 0, 1, 0, // y =0
            0, 0, 1;  // z=0.5
        d << 0, 0.5;
        VectorX g(3), minBound, maxBound;
        g << 1, 0, 0;

        double kin_reslut_length = 0.0;
        Robot_State_Transition::solve_LP_GLPK(A, b, D, d, g, minBound, maxBound, true, nullptr, &kin_reslut_length);
        return (kin_reslut_length - hexapodState.base_Pose_Now.position.x);
    }

    double get_dynamic_length(const hit_spider::hexapod_State &hexapodState)
    {
        // 不考虑joint扭矩约束的最大步长
        double length_ = 0.9 * std::min(get_CWC_base_length(hexapodState), get_kin_length(hexapodState));
        if (length_ < 1e-3)
        {
            length_ = 0;
            return length_;
        }

        int support_leg_num = 0;
        for (int i = 0; i < 6; ++i)
        {
            support_leg_num += hexapodState.support_State_Next[i];
        }
        MatrixX3 contact_Points(support_leg_num, 3);
        std::vector<int> support_leg;
        int row_index = 0;
        for (int i = 0; i < 6; ++i)
        {
            if (hexapodState.support_State_Next[i] == SUPPORT)
            {
                contact_Points.row(row_index++) << hexapodState.feetPositionNow.foot[i].x,
                    hexapodState.feetPositionNow.foot[i].y,
                    hexapodState.feetPositionNow.foot[i].z;
                support_leg.push_back(i + 1);
            }
        }

        Vector3 c(0, 0, 0.5);
        hit_spider::hexapod_Base_Pose base_pose = hexapodState.base_Pose_Now;
        MatrixX3 joints(support_leg_num, 3);

        // 离散为10个点
        int i = 0;
        while (1)
        {
            for (i = 0; i < 10; ++i)
            {
                c(0) = hexapodState.base_Pose_Now.position.x + (double)(i + 1) * length_ / (double)10;
                base_pose.position.x = c(0);

                if ((!support_leg_inverse_kin(base_pose, hexapodState.feetPositionNow, support_leg, joints)) || (!Robot_State_Transition::is_meet_dynamic_con(c, support_leg_num, contact_Points, joints)))
                {
                    length_ = (c(0) - hexapodState.base_Pose_Now.position.x) * 0.9;

                    break;
                }
            }

            if (i == 10)
            {
                break;
            }
        }

        return length_;
    }

    hexapod_SupportState_List findAvailable_SupportStates_and_maxLength(const hit_spider::hexapod_State &hexapodState, std::vector<double> &max_length)
    {
        std::vector<double> temp_vec_destroy;
        max_length.swap(temp_vec_destroy); // 清空内存

        //------支撑腿数量大于3，容错腿不能是支撑腿------
        hexapod_SupportState faultLeg;
        for (int i = 0; i < 6; ++i)
        {
            faultLeg(i) = hexapodState.faultLeg_State_Now[i]; // 当前的错误退信息决定了Next的支撑可行状态
        }
        VectorXi can_support(initialSupportList.rows());
        can_support.setZero();
        for (int i = 0; i < initialSupportList.rows(); ++i)
        {
            if (initialSupportList.row(i) * faultLeg == 0) // 容错腿不能当做支撑腿
            {
                can_support(i) = 1;
            }
        }
        hexapod_SupportState_List SupportList(can_support.sum(), 6);
        int row_index = 0;
        for (int i = 0; i < initialSupportList.rows(); ++i)
        {
            if (can_support(i) == 1)
            {
                SupportList.row(row_index++) = initialSupportList.row(i);
            }
        }
        const hexapod_SupportState_List SupportList_temp = SupportList;
        can_support.resize(SupportList_temp.rows(), 1);
        can_support.setZero();

        //------判断哪些支撑状态是稳定的(运动学约束不用考虑，因为移动过程中考虑了运动学约束)------
        for (int i = 0; i < SupportList_temp.rows(); ++i)
        {
            // 数据准备
            int support_leg_num = 0; // 支撑腿数量
            for (int j = 0; j < 6; ++j)
            {
                support_leg_num += SupportList_temp(i, j);
            }
            std::vector<int> support_leg;                 // 支撑腿标号
            MatrixX3 contact_Points(support_leg_num, 3);  // 支撑腿接触点
            MatrixX3 contact_normals(support_leg_num, 3); // 接触点法向量
            contact_normals.setZero();
            contact_normals.col(2).setOnes(); // 先考虑简单情况
            row_index = 0;
            for (int j = 0; j < 6; ++j)
            {
                if (SupportList_temp(i, j) == 1)
                {
                    contact_Points.row(row_index++) << hexapodState.feetPositionNow.foot[j].x,
                        hexapodState.feetPositionNow.foot[j].y,
                        hexapodState.feetPositionNow.foot[j].z;
                    support_leg.push_back(j + 1);
                }
            }

            // debug
            //  std::cout << contact_Points << std::endl;

            // 1.判断当前质心位置是否在下一步CWC内
            if (support_leg_num == 3)
            {
                // 判断三个支撑点是否共线
                MatrixX3 temp_contact_Points = contact_Points;
                temp_contact_Points.col(2).setOnes();
                double S = temp_contact_Points.determinant();
                if (S < 1e-5)
                {
                    // debug
                    //  ROS_ERROR("三个支撑点共线");
                    continue;
                }
            }

            MatrixXX CWC_A = Robot_State_Transition::comput_friction_region(contact_Points, contact_normals, 0.5, Robot_State_Transition::Mass);
            MatrixXX A;
            VectorX b;
            // cdd求解约束方程
            if (!Robot_State_Transition::vertices_to_H(CWC_A, A, b))
            {
                my_round_4(CWC_A);
                if (!Robot_State_Transition::vertices_to_H(CWC_A, A, b))
                {
                    // debug
                    // ROS_ERROR("cdd不稳定!给它一次机会还是不稳定"); // 认为此支撑状态是不行的
                    continue;
                }
                // debug
                //  std::cout << "cdd虽然有数值问题,但是减小精度就行了" << std::endl;
            }
            // 判断是否在多边形内部
            point_Planar c_xy;
            c_xy << hexapodState.base_Pose_Now.position.x, hexapodState.base_Pose_Now.position.y;

            if (!((A * c_xy).array() <= b.array()).all()) // 当前质心不在CWC多边形内部
            {
                continue;
            }

            //  2.判断当前质心位置是否满足关节力矩约束，
            Vector3 c;
            c << hexapodState.base_Pose_Now.position.x, hexapodState.base_Pose_Now.position.y, hexapodState.base_Pose_Now.position.z;
            MatrixX3 joints(support_leg_num, 3);
            // 当前质心位置是否满足支撑腿的关节力矩约束
            if ((support_leg_inverse_kin(hexapodState.base_Pose_Now, hexapodState.feetPositionNow, support_leg, joints)) && (Robot_State_Transition::is_meet_dynamic_con(c, support_leg_num, contact_Points, joints)))
            {
                can_support(i) = 1;
            }
            else
            {
                continue;
            }

            // 3.求解CWC最大步长
            MatrixXX D(1, 2);
            VectorX d(1);
            D << 0, 1;
            d << 0;
            VectorX g(2), minBound, maxBound;
            g << 1, 0;
            double CWC_reslut_length = 0.0;
            Robot_State_Transition::solve_LP_GLPK(A, b, D, d, g, minBound, maxBound, true, nullptr, &CWC_reslut_length);
            CWC_reslut_length -= hexapodState.base_Pose_Now.position.x;

            // 4.求解kin最大步长
            // 当前支撑腿对质心的运动约束
            auto feet_con_cog_Ab = Robot_State_Transition::get_kinematics_con_foot_cog(hexapodState.base_Pose_Now, support_leg, contact_Points, contact_normals);

            A.resize(7 * support_leg_num, 3);
            b.resize(7 * support_leg_num);
            A.setZero();
            b.setZero();
            A.block(0, 0, 7 * support_leg_num, 3) = feet_con_cog_Ab.first;
            b.block(0, 0, 7 * support_leg_num, 1) = feet_con_cog_Ab.second;

            // 优化求解最大kin约束的最大步长
            D.resize(2, 3);
            d.resize(2, 1);
            D << 0, 1, 0,
                0, 0, 1;
            d << 0, 0.5;
            g.resize(3, 1);
            g << 1, 0, 0;

            double kin_reslut_length = 0.0;
            Robot_State_Transition::solve_LP_GLPK(A, b, D, d, g, minBound, maxBound, true, nullptr, &kin_reslut_length);
            kin_reslut_length -= hexapodState.base_Pose_Now.position.x;

            // 5.求解dynamic最大步长
            double length_ = 0.9 * std::min(CWC_reslut_length, kin_reslut_length);
            if (length_ < 1e-5)
            {
                length_ = 0;
                max_length.push_back(length_);
                continue; // 继续判断下一个支撑状态
            }

            c << 0, 0, 0.5;
            hit_spider::hexapod_Base_Pose base_pose = hexapodState.base_Pose_Now;
            joints.setZero();

            // 离散为10个点
            int ii = 0;
            while (1)
            {
                for (ii = 0; ii < 10; ++ii)
                {
                    c(0) = hexapodState.base_Pose_Now.position.x + (double)(ii + 1) * length_ / (double)10;
                    base_pose.position.x = c(0);

                    if ((!support_leg_inverse_kin(base_pose, hexapodState.feetPositionNow, support_leg, joints)) || (!Robot_State_Transition::is_meet_dynamic_con(c, support_leg_num, contact_Points, joints)))
                    {
                        length_ = (c(0) - hexapodState.base_Pose_Now.position.x) * 0.9;
                        break;
                    }
                }

                if (ii == 10)
                {
                    break;
                }
            }
            max_length.push_back(length_);
        }

        // 更新可行支撑状态集合
        SupportList.resize(can_support.sum(), 6);
        row_index = 0;
        for (int i = 0; i < SupportList_temp.rows(); ++i)
        {
            if (can_support(i) == 1)
            {
                SupportList.row(row_index++) = SupportList_temp.row(i);
            }
        }

        // debug
        assert(row_index == can_support.sum());
        assert(SupportList.rows() == can_support.sum());
        assert((int)max_length.size() == row_index);

        return SupportList;
    }

    Footholds getAvailableFootholds(const hit_spider::hexapod_State &hexapodState)
    {
        Footholds output_feet_position;

        // 环境可落足点集合
        MatrixX3 map_feasible_position = Static_Information::get_now_Feasible_foot_position(hexapodState.base_Pose_Next);

        // base约束foot
        auto Ab = Robot_State_Transition::get_kinematics_con_cog_foot(hexapodState.base_Pose_Next);

        for (int i = 0; i < 6; ++i)
        {
            if (hexapodState.support_State_Next[i] == SWING)
            {
                MatrixX3 A = Ab.first.block(7 * i, 0, 7, 3);
                VectorX b = Ab.second.block(7 * i, 0, 7, 1);

                // 判断环境中的点是否在kin质心约束foot的约束范围内
                // for (int j = 0; j < (int)map_feasible_position.rows(); ++j)
                // {
                //     VectorX temp = A * map_feasible_position.row(j).transpose();
                //     if ((temp.array() <= b.array()).all())
                //     {
                //         output_feet_position.leg[i].push_back(map_feasible_position.row(j).transpose());
                //     }
                // }
                MatrixXX temp = A * map_feasible_position.transpose();
                for (int j = 0; j < temp.cols(); ++j)
                {
                    if ((temp.col(j).array() <= b.array()).all())
                    {
                        output_feet_position.leg[i].push_back(map_feasible_position.row(j).transpose());
                    }
                }
            }
        }
        return output_feet_position;
    }

    void swingLegFoot_position_Expert(hit_spider::hexapod_State &hexapodState)
    {
        Footholds feasible_positions = getAvailableFootholds(hexapodState);
        for (int i = 0; i < 6; ++i)
        {
            if (hexapodState.support_State_Next[i] == SWING)
            {
                if (feasible_positions.leg[i].size() != 0)
                {
                    // 寻找x最大的落足点
                    auto max_iter = std::max_element(feasible_positions.leg[i].begin(), feasible_positions.leg[i].end(),
                                                     [](const Vector3 &v1, const Vector3 &v2)
                                                     { return v1(0) < v2(0); });
                    hexapodState.feetPositionNext.foot[i].x = max_iter->x();
                    hexapodState.feetPositionNext.foot[i].y = max_iter->y();
                    hexapodState.feetPositionNext.foot[i].z = max_iter->z();
                }
                else
                {
                    hexapodState.faultLeg_State_Next[i] = FAULT_LEG;
                    hexapodState.feetPositionNext.foot[i].x = hexapodState.base_Pose_Next.position.x + Hexapod_defaultFoothold[i].x();
                    hexapodState.feetPositionNext.foot[i].y = hexapodState.base_Pose_Next.position.y + Hexapod_defaultFoothold[i].y();
                    hexapodState.feetPositionNext.foot[i].z = hexapodState.base_Pose_Next.position.z + Hexapod_defaultFoothold[i].z() + 0.3f;
                }
            }
            else
            {
                hexapodState.feetPositionNext.foot[i] = hexapodState.feetPositionNow.foot[i]; // 支撑腿的位置不变
            }
        }
    }

    void get_nextState_Expert(hit_spider::hexapod_State &hexapodState)
    {
        hit_spider::hexapod_State temp_hexapodState = hexapodState;

        // 获取可行支撑状态
        std::vector<double> max_length;
        const hexapod_SupportState_List feasible_support_state = findAvailable_SupportStates_and_maxLength(hexapodState, max_length);

        // debug
        // std::cout << "支撑状态有:" << std::endl
        //           << feasible_support_state << std::endl;
        // std::cout << "前进距离为:" << std::endl;
        // for (int i = 0; i < (int)max_length.size(); ++i)
        // {
        //     std::cout << max_length[i] << '\n';
        // }
        // std::cout << std::endl;

        auto iter = std::max_element(max_length.begin(), max_length.end());
        int support_index = iter - max_length.begin();

        // 专家法选择最大前进距离对应的支撑状态作为下一步的支撑状态
        hexapodState.base_Pose_Next = hexapodState.base_Pose_Now;
        hexapodState.base_Pose_Next.position.x += *iter;
        for (int j = 0; j < 6; ++j)
        {
            hexapodState.support_State_Next[j] = feasible_support_state(support_index, j);
        }

        // 专家法为摆动腿选择落足点
        swingLegFoot_position_Expert(hexapodState);
    }

    void get_nextState_Rondom(hit_spider::hexapod_State &hexapodState)
    {
        hit_spider::hexapod_State temp_hexapodState = hexapodState;

        // 获取可行支撑状态
        std::vector<double> max_length;
        const hexapod_SupportState_List feasible_support_state = findAvailable_SupportStates_and_maxLength(hexapodState, max_length);

        // debug
        // std::cout << "支撑状态有:" << std::endl
        //           << feasible_support_state << std::endl;
        // std::cout << "前进距离为:" << std::endl;
        // for (int i = 0; i < (int)max_length.size(); ++i)
        // {
        //     std::cout << max_length[i] << '\n';
        // }
        // std::cout << std::endl;

        int temp_num = rand_customization(0, max_length.size() - 1);

        // 随机选择支撑状态
        hexapodState.base_Pose_Next = hexapodState.base_Pose_Now;
        hexapodState.base_Pose_Next.position.x += max_length[temp_num];
        for (int j = 0; j < 6; ++j)
        {
            hexapodState.support_State_Next[j] = feasible_support_state(temp_num, j);
        }

        // 专家法为摆动腿选择落足点
        swingLegFoot_position_Expert(hexapodState);
    }

    std::vector<hit_spider::hexapod_State> get_NextState_list(const hit_spider::hexapod_State &hexapodState)
    {
        hit_spider::hexapod_State temp_hexapodState = hexapodState;

        // 确定可行支撑状态
        std::vector<double> max_length;
        const hexapod_SupportState_List feasible_support_state = findAvailable_SupportStates_and_maxLength(hexapodState, max_length);

        // 输出的备选状态集
        std::vector<hit_spider::hexapod_State> state_list;
        state_list.reserve(feasible_support_state.rows());

        for (int i = 0; i < (int)feasible_support_state.rows(); ++i)
        {
            // debug
            //  ROS_INFO("第%d个支撑状态为:", i + 1);
            //  std::cout << feasible_support_state.row(i) << std::endl;

            // 确定每一个支撑状态的前进距离
            for (int j = 0; j < 6; ++j)
            {
                temp_hexapodState.support_State_Next[j] = feasible_support_state(i, j);
            }
            double temp_length = max_length[i];
            temp_hexapodState.base_Pose_Next = hexapodState.base_Pose_Now;
            temp_hexapodState.base_Pose_Next.position.x += temp_length;

            // 确定摆动腿落足点
            swingLegFoot_position_Expert(temp_hexapodState);
            temp_hexapodState.base_Pose_Now = temp_hexapodState.base_Pose_Next;           // 机体位置
            temp_hexapodState.feetPositionNow = temp_hexapodState.feetPositionNext;       // 落足点位置
            temp_hexapodState.support_State_Now = temp_hexapodState.support_State_Next;   // 支撑状态
            temp_hexapodState.faultLeg_State_Now = temp_hexapodState.faultLeg_State_Next; // 容错腿状态

            state_list.push_back(temp_hexapodState);
        }

        return state_list;
    }
}