// 六足机器人对象

#ifndef HIT_SPIDER_HEXAPOD_H_
#define HIT_SPIDER_HEXAPOD_H_

#include "hit_spider/util.hh"

struct Leg
{
    Leg();
    ~Leg();
    void set_num(const int &num);
    /**
     * @ : 单腿逆运动学，由落足位置更新单腿的三个关节角度
     * @description:
     * @param {Point} foot_Word：世界坐标系下的足端位置
     * @param {Transform} robotW：世界系下机器人Trunk机体的坐标变换T
     * @return {*}：更新leg对象中的三个关节角度
     */
    void IK(const hit_spider::hexapod_Base_Pose &Wolrd_BasePose, const geometry_msgs::Point &World_foot);

    int legNum;
    // 单腿三个关节角度
    float theta1;
    float theta2;
    float theta3;
};

class Hexapod
{
public:
    Hexapod(ros::NodeHandle &nh); // 发布gu节相对于ji节的静态坐标变换
    ~Hexapod();

    /**
     * @ : 计算整机逆运动学，发布坐标变换、关节角度信息，同时绘图
     * @description:
     * @param {hexapod_State} &p
     * @return {*}
     */
    void IK_Robot(const hit_spider::hexapod_State &p);

private:
    Leg *legs;

    // 18个关节角度
    ros::Publisher joint_state_pub_; // 关节角度信息发布者
    sensor_msgs::JointState joint_state;

    // odom到机器人base_link的坐标变换
    tf2_ros::TransformBroadcaster odom_broadcaster; // 动态坐标变换发布者
    geometry_msgs::TransformStamped odom_2_trunck;

private:
};

/**
 * @ : 根据机体位姿和落足点求解18个关节角度
 * @description:
 * @param {hexapod_Base_Pose} &base_pose：机体位姿
 * @param {FeetPosition} &feet_position：落足点
 * @return {*}：六条腿的关节角度，每一行是一条腿的关节角度
 */
Matrix63 whole_body_inverse_kin(const hit_spider::hexapod_Base_Pose &Wolrd_BasePose, const hit_spider::FeetPosition &Wolrd_feet);

//返回指定腿(支撑腿)的关节角度
bool support_leg_inverse_kin(const hit_spider::hexapod_Base_Pose &Wolrd_BasePose, const hit_spider::FeetPosition &World_feet, const std::vector<int> &support_leg, MatrixX3 &joints_output);

#endif
