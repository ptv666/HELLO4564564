// 接触序列规划完成后，规划足端轨迹、机体轨迹具体实现

#include "hit_spider/MCTS/trajectory.hh"

using namespace std;

namespace trajectory
{

    // 腿部运动参数计算  7：每条腿每个方向的7个解析参数  18：每条腿三个方向*6条腿，还能修改摆动腿的步高
    Eigen::Matrix<float, 7, 18> solution_leg(hit_spider::hexapod_State state1, hit_spider::hexapod_State state2)
    {
        Eigen::Matrix<float, 7, 7> mat_leg, inversemat_leg;
        mat_leg << 1, 0, 0, 0, 0, 0, 0,
            1, 0.5, pow(0.5, 2), pow(0.5, 3), pow(0.5, 4), pow(0.5, 5), pow(0.5, 6),
            1, 1, 1, 1, 1, 1, 1,
            0, 1, 0, 0, 0, 0, 0,
            0, 1, 2, 3, 4, 5, 6,
            0, 0, 2, 0, 0, 0, 0,
            0, 0, 2, 6, 12, 20, 30;
        inversemat_leg = mat_leg.inverse(); // 矩阵求逆 Eigen库的用法
        // vector<float>  solution_leg(3, 0);

        Eigen::Matrix<float, 7, 18> solutionxyz = Eigen::MatrixXf::Zero(7, 18); // 矩阵初始化
        Eigen::Matrix<float, 7, 18> knownquantity = Eigen::MatrixXf::Zero(7, 18);
        for (int i = 0; i < 6; i++)
        {

            if (state2.support_State_Now[i] == 0) // 判断摆动腿 0：Swing
            {

                knownquantity(0, 3 * i) = state1.feetPositionNow.foot[i].x;
                knownquantity(0, 3 * i + 1) = state1.feetPositionNow.foot[i].y;
                knownquantity(0, 3 * i + 2) = state1.feetPositionNow.foot[i].z;
                knownquantity(2, 3 * i) = state2.feetPositionNow.foot[i].x;
                knownquantity(2, 3 * i + 1) = state2.feetPositionNow.foot[i].y;
                knownquantity(2, 3 * i + 2) = state2.feetPositionNow.foot[i].z;
                float h = state2.feetPositionNow.foot[i].z + 0.3; // 摆动腿步高 ////////////////////////////////
                knownquantity(1, 3 * i) = (state1.feetPositionNow.foot[i].x + state2.feetPositionNow.foot[i].x) / 2.0f;
                knownquantity(1, 3 * i + 1) = (state1.feetPositionNow.foot[i].y + state2.feetPositionNow.foot[i].y) / 2.0f;
                knownquantity(1, 3 * i + 2) = h;
            }
        }

        solutionxyz = inversemat_leg * knownquantity;
        return solutionxyz;
    }

    // 腿部数据赋值
    void leg_assignment(hit_spider::hexapod_State state, hit_spider::FeetPosition &feetNow, Eigen::Matrix<float, 7, 18> solution, float t)
    {
        for (int j = 0; j < 6; j++)
        {
            if (state.support_State_Now[j] == 0)
            {
                feetNow.foot[j].x = solution(0, 3 * j) + solution(1, 3 * j) * t + solution(2, 3 * j) * pow(t, 2) + solution(3, 3 * j) * pow(t, 3) + solution(4, 3 * j) * pow(t, 4) + solution(5, 3 * j) * pow(t, 5) + solution(6, 3 * j) * pow(t, 6);
                feetNow.foot[j].y = solution(0, 3 * j + 1) + solution(1, 3 * j + 1) * t + solution(2, 3 * j + 1) * pow(t, 2) + solution(3, 3 * j + 1) * pow(t, 3) + solution(4, 3 * j + 1) * pow(t, 4) + solution(5, 3 * j + 1) * pow(t, 5) + solution(6, 3 * j + 1) * pow(t, 6);
                feetNow.foot[j].z = solution(0, 3 * j + 2) + solution(1, 3 * j + 2) * t + solution(2, 3 * j + 2) * pow(t, 2) + solution(3, 3 * j + 2) * pow(t, 3) + solution(4, 3 * j + 2) * pow(t, 4) + solution(5, 3 * j + 2) * pow(t, 5) + solution(6, 3 * j + 2) * pow(t, 6);
            }
        }
    }

    // 机体参数计算  6：机体每个方向的6个解析参数  3：xyz三个方向
    Eigen::Matrix<float, 6, 3> solution_body(hit_spider::hexapod_Base_Pose robotNow, hit_spider::hexapod_Base_Pose robotNext)
    {
        Eigen::Matrix<float, 6, 6> mat_body, inversemat_body;
        mat_body << 1, 0, 0, 0, 0, 0,
            1, 1, 1, 1, 1, 1,
            0, 1, 0, 0, 0, 0,
            0, 1, 2, 3, 4, 5,
            0, 0, 2, 0, 0, 0,
            0, 0, 2, 6, 12, 20;
        inversemat_body = mat_body.inverse(); // 矩阵求逆 Eigen库的用法
        Eigen::Matrix<float, 6, 3> solutionxyz_body;
        Eigen::Matrix<float, 6, 3> knownquantity_body = Eigen::MatrixXf::Zero(6, 3); // 矩阵初始化

        knownquantity_body(0, 0) = robotNow.position.x;
        knownquantity_body(0, 1) = robotNow.position.y;
        knownquantity_body(0, 2) = robotNow.position.z;
        knownquantity_body(1, 0) = robotNext.position.x;
        knownquantity_body(1, 1) = robotNext.position.y;
        knownquantity_body(1, 2) = robotNext.position.z;
        solutionxyz_body = inversemat_body * knownquantity_body;
        return solutionxyz_body;
    }

    Eigen::Matrix<float, 6, 3> solution_body_rotation(hit_spider::hexapod_Base_Pose robotNow, hit_spider::hexapod_Base_Pose robotNext)
    {
        Eigen::Matrix<float, 6, 6> mat_body, inversemat_body;
        mat_body << 1, 0, 0, 0, 0, 0,
            1, 1, 1, 1, 1, 1,
            0, 1, 0, 0, 0, 0,
            0, 1, 2, 3, 4, 5,
            0, 0, 2, 0, 0, 0,
            0, 0, 2, 6, 12, 20;
        inversemat_body = mat_body.inverse(); // 矩阵求逆 Eigen库的用法
        Eigen::Matrix<float, 6, 3> solutionxyz_body;
        Eigen::Matrix<float, 6, 3> knownquantity_body = Eigen::MatrixXf::Zero(6, 3); // 矩阵初始化

        knownquantity_body(0, 0) = robotNow.orientation.roll;
        knownquantity_body(0, 1) = robotNow.orientation.pitch;
        knownquantity_body(0, 2) = robotNow.orientation.yaw;
        knownquantity_body(1, 0) = robotNext.orientation.roll;
        knownquantity_body(1, 1) = robotNext.orientation.pitch;
        knownquantity_body(1, 2) = robotNext.orientation.yaw;
        solutionxyz_body = inversemat_body * knownquantity_body;
        return solutionxyz_body;
    }

    // 机体数据赋值
    void body_assignment(hit_spider::hexapod_Base_Pose &robotNext, Eigen::Matrix<float, 6, 3> solution, Eigen::Matrix<float, 6, 3> solutionbodyRotate, float t)
    {
        robotNext.position.x = solution(0, 0) + solution(1, 0) * t + solution(2, 0) * pow(t, 2) + solution(3, 0) * pow(t, 3) + solution(4, 0) * pow(t, 4) + solution(5, 0) * pow(t, 5);
        robotNext.position.y = solution(0, 1) + solution(1, 1) * t + solution(2, 1) * pow(t, 2) + solution(3, 1) * pow(t, 3) + solution(4, 1) * pow(t, 4) + solution(5, 1) * pow(t, 5);
        robotNext.position.z = solution(0, 2) + solution(1, 2) * t + solution(2, 2) * pow(t, 2) + solution(3, 2) * pow(t, 3) + solution(4, 2) * pow(t, 4) + solution(5, 2) * pow(t, 5);

        robotNext.orientation.roll = solutionbodyRotate(0, 0) + solutionbodyRotate(1, 0) * t + solutionbodyRotate(2, 0) * pow(t, 2) + solutionbodyRotate(3, 0) * pow(t, 3) + solutionbodyRotate(4, 0) * pow(t, 4) + solutionbodyRotate(5, 0) * pow(t, 5);
        robotNext.orientation.pitch = solutionbodyRotate(0, 1) + solutionbodyRotate(1, 1) * t + solutionbodyRotate(2, 1) * pow(t, 2) + solutionbodyRotate(3, 1) * pow(t, 3) + solutionbodyRotate(4, 1) * pow(t, 4) + solutionbodyRotate(5, 1) * pow(t, 5);
        robotNext.orientation.yaw = solutionbodyRotate(0, 2) + solutionbodyRotate(1, 2) * t + solutionbodyRotate(2, 2) * pow(t, 2) + solutionbodyRotate(3, 2) * pow(t, 3) + solutionbodyRotate(4, 2) * pow(t, 4) + solutionbodyRotate(5, 2) * pow(t, 5);
    }

}
