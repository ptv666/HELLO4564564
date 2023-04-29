// 接触序列规划完成后，规划足端轨迹、机体轨迹

#ifndef MCTS_TRAJECTORY_HH
#define MCTS_TRAJECTORY_HH

#include "hit_spider/util.hh"

namespace trajectory
{
    // 输入六足机器人初末状态，计算足端轨迹的六次多项式系数
    Eigen::Matrix<float, 7, 18> solution_leg(const hit_spider::hexapod_State &state1, const hit_spider::hexapod_State &state2);
    // 输入六足机器人初末状态，计算机体质心轨迹的五次多项式系数
    Eigen::Matrix<float, 6, 3> solution_body(const hit_spider::hexapod_Base_Pose &robotNow, const hit_spider::hexapod_Base_Pose &robotNext);
    // 输入六足机器人初末状态，计算机体姿态轨迹的五次多项式系数
    Eigen::Matrix<float, 6, 3> solution_body_rotation(const hit_spider::hexapod_Base_Pose &robotNow, const hit_spider::hexapod_Base_Pose &robotNext);

    // 根据时间计算足端具体位置
    void leg_assignment(const hit_spider::hexapod_State &state, hit_spider::FeetPosition &feetNow, const Eigen::Matrix<float, 7, 18> &solution, const float t);
    // 根据时间计算具体的机体位姿
    void body_assignment(hit_spider::hexapod_Base_Pose &robotNext, const Eigen::Matrix<float, 6, 3> &solution, const Eigen::Matrix<float, 6, 3> &solutionbodyRotate, const float t);
}
#endif