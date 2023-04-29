// util中接口函数的实现、全局变量的定义

#include "hit_spider/util.hh"
#include <tf2/LinearMath/Quaternion.h>

//------共享全局变量定义------
namespace
{
    // 初始化固定的坐标变换FixJi
    Eigen::Isometry3d init_TransMatrix_FixJi_Body(const int &num)
    {
        Eigen::Isometry3d T;

        T.matrix().row(0) << cos(body_fixJi_theta[num]), sin(body_fixJi_theta[num]), 0, -0.4; //-R
        T.matrix().row(1) << -sin(body_fixJi_theta[num]), cos(body_fixJi_theta[num]), 0, 0;
        T.matrix().row(2) << 0, 0, 1, 0;
        T.matrix().row(3) << 0, 0, 0, 1;

        return T;
    }

    // 初始化固定的坐标变换FixGu
    Eigen::Isometry3d init_TransMatrix_Body_2_fixJi(const int &num)
    {
        Eigen::Isometry3d T = init_TransMatrix_FixJi_Body(num);

        return T.inverse();
    }

    // 机体base坐标系下六个默认落足点
    Vector3 init_defaultFoothold(const int &num)
    {
        Vector3 p;
        p.x() = 1.08 * cos(body_fixJi_theta[num]);
        p.y() = 1.08 * sin(body_fixJi_theta[num]);
        p.z() = -0.5;

        return p;
    }

}

const float body_fixJi_theta[6] = {PI * 2 / 3, PI, PI * 4 / 3, PI * 5 / 3, 0, PI * 1 / 3};
const std::vector<std::string> HEXAPOD_JOINT_STATE_NAME{"joint_lf_1", "joint_lf_2", "joint_lf_3",
                                                        "joint_lm_1", "joint_lm_2", "joint_lm_3",
                                                        "joint_lh_1", "joint_lh_2", "joint_lh_3",
                                                        "joint_rf_1", "joint_rf_2", "joint_rf_3",
                                                        "joint_rm_1", "joint_rm_2", "joint_rm_3",
                                                        "joint_rh_1", "joint_rh_2", "joint_rh_3"};
const Eigen::Isometry3d TransMatrix_FixJi_Body[6] = {init_TransMatrix_FixJi_Body(0), init_TransMatrix_FixJi_Body(1), init_TransMatrix_FixJi_Body(2),
                                                     init_TransMatrix_FixJi_Body(3), init_TransMatrix_FixJi_Body(4), init_TransMatrix_FixJi_Body(5)};
const Eigen::Isometry3d TransMatrix_Body_2_fixJi[6] = {init_TransMatrix_Body_2_fixJi(0), init_TransMatrix_Body_2_fixJi(1), init_TransMatrix_Body_2_fixJi(2),
                                                       init_TransMatrix_Body_2_fixJi(3), init_TransMatrix_Body_2_fixJi(4), init_TransMatrix_Body_2_fixJi(5)};
const Vector3 Hexapod_defaultFoothold[6] = {init_defaultFoothold(0), init_defaultFoothold(1), init_defaultFoothold(2),
                                            init_defaultFoothold(3), init_defaultFoothold(4), init_defaultFoothold(5)};

//------常用函数定义------

Matrix3 crossMatrix(const Vector3 &x)
{
    Matrix3 res;
    res.setZero();
    res(0, 1) = -x(2);
    res(0, 2) = x(1);
    res(1, 0) = x(2);
    res(1, 2) = -x(0);
    res(2, 0) = -x(1);
    res(2, 1) = x(0);
    return res;
}

int rand_customization(int minValue, int maxValue)
{
    maxValue = maxValue + 1;
    return rand() % (maxValue - minValue) + minValue;
}

Eigen::Isometry3d getTrans_W_B(const hit_spider::hexapod_Base_Pose &pose)
{
    tf2::Quaternion tf2_q;
    tf2_q.setEuler(pose.orientation.roll, pose.orientation.pitch, pose.orientation.yaw);
    Eigen::Quaterniond q(tf2_q.getW(), tf2_q.getX(), tf2_q.getY(), tf2_q.getZ());
    Vector3 t(pose.position.x, pose.position.y, pose.position.z);

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(q);
    T.pretranslate(t);
    return T;
}

geometry_msgs::Transform getTransform_W_B(const hit_spider::hexapod_Base_Pose &pose)
{
    geometry_msgs::Transform trans;
    tf2::Quaternion tf2_q;
    tf2_q.setEuler(pose.orientation.roll, pose.orientation.pitch, pose.orientation.yaw);
    trans.rotation.w = tf2_q.getW();
    trans.rotation.x = tf2_q.getX();
    trans.rotation.y = tf2_q.getY();
    trans.rotation.z = tf2_q.getZ();
    trans.translation.x = pose.position.x;
    trans.translation.y = pose.position.y;
    trans.translation.z = pose.position.z;
    return trans;
}

void init_hexapodState(hit_spider::hexapod_State &hexapodState)
{

    // 当前机体位姿 与 下一状态机体位姿
    hexapodState.base_Pose_Now.position.x = hexapodState.base_Pose_Now.position.y = 0.0;
    hexapodState.base_Pose_Now.position.z = 0.5;
    hexapodState.base_Pose_Now.orientation.roll = hexapodState.base_Pose_Now.orientation.yaw = hexapodState.base_Pose_Now.orientation.pitch = 0.0;
    hexapodState.base_Pose_Next = hexapodState.base_Pose_Now;

    // 六条腿的默认落足点
    for (int i = 0; i < 6; ++i)
    {
        // 当前落足点
        hexapodState.feetPositionNow.foot[i].x = Hexapod_defaultFoothold[i].x();
        hexapodState.feetPositionNow.foot[i].y = Hexapod_defaultFoothold[i].y();
        hexapodState.feetPositionNow.foot[i].z = 0;

        // 当前支撑状态
        hexapodState.support_State_Now[i] = SUPPORT;
        // 当前容错状态
        hexapodState.faultLeg_State_Now[i] = NORMAL_LEG;
    }
    // 下一步落足点
    hexapodState.feetPositionNext = hexapodState.feetPositionNow;
    // 下一步支撑状态和容错状态
    hexapodState.support_State_Next = hexapodState.support_State_Now;
    hexapodState.faultLeg_State_Next = hexapodState.faultLeg_State_Now;

    hexapodState.move_Direction.x = 1;
    hexapodState.move_Direction.y = 0;
    hexapodState.move_Direction.z = 0;
}
