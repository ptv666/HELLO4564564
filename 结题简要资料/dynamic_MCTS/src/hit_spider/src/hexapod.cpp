#include "hit_spider/hexapod.hh"
#include <functional> //std::bind()函数

//------Class Leg------
Leg::Leg() : legNum(0), theta1(0), theta2(0), theta3(0)
{
}

Leg::~Leg()
{
}

void Leg::set_num(const int &num)
{
    this->legNum = num;
}

void Leg::IK(const hit_spider::hexapod_Base_Pose &Wolrd_BasePose, const geometry_msgs::Point &World_foot)
{
    // 计算足端目标机器人坐标系到世界坐标系的旋转平移矩阵T
    Eigen::Isometry3d T_W_B = getTrans_W_B(Wolrd_BasePose);

    // 计算足端到固定基节坐标系下的坐标
    Eigen::Vector3d p1_ = Eigen::Vector3d(World_foot.x, World_foot.y, World_foot.z);
    Eigen::Vector3d foot_FixJi;
    foot_FixJi = TransMatrix_FixJi_Body[legNum - 1] * T_W_B.inverse() * p1_;

    float tmp_theta1 = atan2(foot_FixJi.y(), foot_FixJi.x());
    float N = foot_FixJi.z();
    float M = 0.0f;
    if (fabs(foot_FixJi.y()) < 0.00000000001f)
    {
        M = foot_FixJi.x() - 0.18f;
    }
    else
    {
        M = foot_FixJi.y() / sin(tmp_theta1) - 0.18f;
    }
    if (sqrt(M * M + N * N) > 0.5 + 0.5)
    {
        // debug
        std::cout << "foot_FixJi = " << foot_FixJi << std::endl;
        ROS_ERROR("out of range,M:%f,N:%f", M, N);
    }
    float tmp_acos = acos((M * M + N * N) / sqrt(M * M + N * N));
    float tmp_theta2 = atan2(N, M) + tmp_acos;
    float tmp_theta3 = atan2(N - 0.5 * sin(tmp_theta2), M - 0.5f * cos(tmp_theta2)) - tmp_theta2;

    tmp_theta1 = -tmp_theta1;
    tmp_theta2 = -tmp_theta2;
    tmp_theta3 = tmp_theta3;

    // 更新单腿关节角度
    this->theta1 = tmp_theta1;
    this->theta2 = tmp_theta2;
    this->theta3 = tmp_theta3;
}

//------Class Leg------

//------Class Hexapod------
Hexapod::Hexapod(ros::NodeHandle &nh)
{
    this->legs = new Leg[6];
    for (int i = 0; i < 6; ++i)
    {
        this->legs[i].set_num(i + 1);
    }

    // joint_states话题初始化
    this->joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("/joint_states", 50);
    this->joint_state.name.resize(18);
    this->joint_state.position.resize(18);
    for (size_t i = 0; i < 18; ++i) // 设置18个关节的名字
    {
        this->joint_state.name[i] = HEXAPOD_JOINT_STATE_NAME[i];
    }

    // 设置坐标变换的名字
    this->odom_2_trunck.header.frame_id = "odom";     // 基坐标系（世界坐标系）
    this->odom_2_trunck.child_frame_id = "link_base"; // 子坐标系
    odom_2_trunck.transform.rotation.w = 1;           // 初始化一下

    //------发布fixGu相对于fixJi节的静态坐标变换，用于绘图使用，静态坐标变换也需要spin
    tf2_ros::StaticTransformBroadcaster stb;
    std::string str_header1, str_header2;
    str_header1 = "fixJi_"; // urdf提供了这个连杆坐标系
    str_header1 = "fixGu_";
    tf2::Quaternion q;
    q.setEuler(-PI / 2, 0, 0);
    for (int i = 0; i < 6; ++i)
    {
        geometry_msgs::TransformStamped tfs;
        tfs.header.stamp = ros::Time::now();
        tfs.header.frame_id = str_header1 + std::to_string(i + 1);
        tfs.child_frame_id = str_header2 + std::to_string(i + 1);
        tfs.transform.rotation.w = q.getW();
        tfs.transform.rotation.x = q.getX();
        tfs.transform.rotation.y = q.getY();
        tfs.transform.rotation.z = q.getZ();
        tfs.transform.translation.x = 0.18;
        tfs.transform.translation.y = 0;
        tfs.transform.translation.z = 0;
        stb.sendTransform(tfs);
    }
}

Hexapod::~Hexapod()
{
    delete[] legs;
}

void Hexapod::IK_Robot(const hit_spider::hexapod_State &p)
{
    // 设置odom姿态
    this->odom_2_trunck.transform = getTransform_W_B(p.base_Pose_Now);

    // 求解单腿逆运动学，设置18个关节的关节角度
    for (int i = 0; i < 6; ++i)
    {
        this->legs[i].IK(p.base_Pose_Now, p.feetPositionNow.foot[i]);
    }

    // 更新发布的关机角度信息
    joint_state.position[0] = -(this->legs[0].theta1); // 第一条腿 == lf
    joint_state.position[1] = -(this->legs[0].theta2);
    joint_state.position[2] = (this->legs[0].theta3) + PI / 2;

    joint_state.position[3] = -(this->legs[1].theta1); // 第二条腿 == lm
    joint_state.position[4] = -(this->legs[1].theta2);
    joint_state.position[5] = (this->legs[1].theta3) + PI / 2;

    joint_state.position[6] = -(this->legs[2].theta1); // 第三条腿 == lh
    joint_state.position[7] = -(this->legs[2].theta2);
    joint_state.position[8] = (this->legs[2].theta3) + PI / 2;

    joint_state.position[9] = -(this->legs[5].theta1); // 第六条腿 == rh
    joint_state.position[10] = -(this->legs[5].theta2);
    joint_state.position[11] = (this->legs[5].theta3) + PI / 2;

    joint_state.position[12] = -(this->legs[4].theta1); // 第五条腿 == rm
    joint_state.position[13] = -(this->legs[4].theta2);
    joint_state.position[14] = (this->legs[4].theta3) + PI / 2;

    joint_state.position[15] = -(this->legs[3].theta1); // 第四条腿 == rm
    joint_state.position[16] = -(this->legs[3].theta2);
    joint_state.position[17] = (this->legs[3].theta3) + PI / 2;

    // 广播坐标变换，机器人Trunk新的位姿
    this->odom_2_trunck.header.stamp = ros::Time::now();
    this->odom_broadcaster.sendTransform(this->odom_2_trunck);

    // 广播18个关节角度
    this->joint_state.header.stamp = ros::Time::now();
    this->joint_state_pub_.publish(this->joint_state);
}

//------Class Hexapod------

Matrix63 whole_body_inverse_kin(const hit_spider::hexapod_Base_Pose &Wolrd_BasePose, const hit_spider::FeetPosition &World_feet)
{
    Matrix63 Joint_angles;

    // 计算足端目标机器人坐标系到世界坐标系的旋转平移矩阵T
    Eigen::Isometry3d T_W_B = getTrans_W_B(Wolrd_BasePose);

    // 六条腿一个一个求解
    for (int i = 0; i < 6; ++i)
    {
        Eigen::Vector3d p1_ = Eigen::Vector3d(World_feet.foot[i].x, World_feet.foot[i].y, World_feet.foot[i].z);
        Eigen::Vector3d foot_FixJi;
        foot_FixJi = TransMatrix_FixJi_Body[i] * T_W_B.inverse() * p1_; // fixGu坐标系下描述足端位置

        float tmp_theta1 = atan2(foot_FixJi.y(), foot_FixJi.x());
        float N = foot_FixJi.z();
        float M = 0.0f;
        if (fabs(foot_FixJi.y()) < 0.00000000001f)
        {
            M = foot_FixJi.x() - 0.18f;
        }
        else
        {
            M = foot_FixJi.y() / sin(tmp_theta1) - 0.18f;
        }
        if (sqrt(M * M + N * N) > 0.5 + 0.5)
        {
            ROS_ERROR("foot out of kinematics range,M:%f,N:%f", M, N);
        }
        float tmp_acos = acos((M * M + N * N) / sqrt(M * M + N * N));
        float tmp_theta2 = atan2(N, M) + tmp_acos;
        float tmp_theta3 = atan2(N - 0.5 * sin(tmp_theta2), M - 0.5f * cos(tmp_theta2)) - tmp_theta2;

        Joint_angles(i, 0) = tmp_theta1;
        Joint_angles(i, 1) = tmp_theta2;
        Joint_angles(i, 2) = tmp_theta3 + PI / 2;
    }

    return Joint_angles;
}

bool support_leg_inverse_kin(const hit_spider::hexapod_Base_Pose &Wolrd_BasePose, const hit_spider::FeetPosition &World_feet, const std::vector<int> &support_leg, MatrixX3 &joints_output)
{
    joints_output.resize(support_leg.size(), 3);

    // 计算足端目标机器人坐标系到世界坐标系的旋转平移矩阵T
    Eigen::Isometry3d T_W_B = getTrans_W_B(Wolrd_BasePose);

    int row_index = 0;
    // 一个腿一个腿求解
    for (int i = 0; i < 6; ++i)
    {
        auto iter = std::find(support_leg.begin(), support_leg.end(), i + 1);

        if (iter != support_leg.end())
        {
            Eigen::Vector3d p1_ = Eigen::Vector3d(World_feet.foot[i].x, World_feet.foot[i].y, World_feet.foot[i].z);
            Eigen::Vector3d foot_FixJi;
            foot_FixJi = TransMatrix_FixJi_Body[i] * T_W_B.inverse() * p1_; // fixGu坐标系下描述足端位置

            float tmp_theta1 = atan2(foot_FixJi.y(), foot_FixJi.x());
            float N = foot_FixJi.z();
            float M = 0.0f;
            if (fabs(foot_FixJi.y()) < 0.00000000001f)
            {
                M = foot_FixJi.x() - 0.18f;
            }
            else
            {
                M = foot_FixJi.y() / sin(tmp_theta1) - 0.18f;
            }
            if (sqrt(M * M + N * N) > 0.5 + 0.5)
            {
                // debug
                // ROS_WARN("foot out of kinematics range,foot_FixJi.y = :%f,foot_FixJi.z = :%f", foot_FixJi.x(), foot_FixJi.z());

                return false;
            }
            float tmp_acos = acos((M * M + N * N) / sqrt(M * M + N * N));
            float tmp_theta2 = atan2(N, M) + tmp_acos;
            float tmp_theta3 = atan2(N - 0.5 * sin(tmp_theta2), M - 0.5f * cos(tmp_theta2)) - tmp_theta2;

            joints_output(row_index, 0) = tmp_theta1;
            joints_output(row_index, 1) = tmp_theta2;
            joints_output(row_index, 2) = tmp_theta3 + PI / 2;
            ++row_index;
        }
    }

    return true;
}