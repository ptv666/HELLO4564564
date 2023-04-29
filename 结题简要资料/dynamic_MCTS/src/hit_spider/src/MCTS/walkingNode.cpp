// 接收规划出的接触状态序列，发布机器人行走中的信息

#include <ros/ros.h>
#include <Eigen/Core>
#include "hit_spider/hexapod_RPY.h"       //机体姿态
#include "hit_spider/hexapod_Base_Pose.h" //机体位姿
#include "hit_spider/FeetPosition.h"      //六个腿落足点
#include "hit_spider/hexapod_State.h"     //机器人状态State
#include "hit_spider/MCTS/trajectory.hh"
#include "hit_spider/hexapod.hh"
#include <Eigen/Dense>

bool msgFlag = false;
std::vector<hit_spider::hexapod_State> statesList;

void supportStateCallback(const hit_spider::hexapod_State::ConstPtr &msg)
{
  if (msgFlag == false)
  {

    statesList.push_back(*msg);

    hit_spider::hexapod_State tmp((*msg));
    std::cout << "Received" << std::endl;
    ROS_INFO("当前statesList的存储值为:%d", (int)statesList.size());
    ROS_INFO("接收到消息,质心位置为:%.4f,%.4f,%.4f", msg->base_Pose_Now.position.x, msg->base_Pose_Now.position.y, msg->base_Pose_Now.position.z);
    ROS_INFO("接收到消息,第一条腿的位置:%.4f,%.4f,%.4f", msg->feetPositionNow.foot[0].x, msg->feetPositionNow.foot[0].y, msg->feetPositionNow.foot[0].z);
    ROS_INFO("接收到消息,当前腿的支撑状态:%d,%d,%d,%d,%d,%d", msg->support_State_Now[0], msg->support_State_Now[1], msg->support_State_Now[2], msg->support_State_Now[3], msg->support_State_Now[4], msg->support_State_Now[5]);

    if ((*msg).remarks.data == "end_flag")
    {
      msgFlag = true;
      std::cout << std::endl
                << std::endl;
    }
  }
}

// 生成行走轨迹,且发送出去
void publishWalkingTrajctory(Hexapod &hexapod, std::vector<hit_spider::hexapod_State> statesL_, visualization_msgs::Marker *footTrajectory, ros::Publisher &footTrajectory_marker_pub)
{
  for (int j = 0; j < (int)statesL_.size() - 1; ++j)
  {
    // 计算插值曲线的参数
    Eigen::Matrix<float, 7, 18> solutionleg;
    solutionleg = trajectory::solution_leg(statesL_[j], statesL_[j + 1]);
    Eigen::Matrix<float, 6, 3> solutionbody;
    solutionbody = trajectory::solution_body(statesL_[j].base_Pose_Now, statesL_[j + 1].base_Pose_Now);
    Eigen::Matrix<float, 6, 3> solutionbodyRotate;
    solutionbodyRotate = trajectory::solution_body_rotation(statesL_[j].base_Pose_Now, statesL_[j + 1].base_Pose_Now);

    float framNum = 100;
    for (float i = 0.1; i < framNum + 1; i++) // 一次运动中插入多少帧 此处选取100
    {
      hit_spider::hexapod_State hexapodState = statesL_[j + 1];
      trajectory::body_assignment(hexapodState.base_Pose_Now, solutionbody, solutionbodyRotate, i / framNum); // 用i来充当时间
      trajectory::leg_assignment(hexapodState, hexapodState.feetPositionNow, solutionleg, i / framNum);

      for (int num_leg = 0; num_leg < 6; ++num_leg)
      {
        footTrajectory[num_leg].points.push_back(hexapodState.feetPositionNow.foot[num_leg]);
        footTrajectory_marker_pub.publish(footTrajectory[num_leg]);
      }

      hexapod.IK_Robot(hexapodState);

      ros::Duration(0.015).sleep();
    }
    ros::Duration(0.5).sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "walkingNode");

  ros::NodeHandle node;
  ros::Subscriber stateSub = node.subscribe("supportStateTopic", 100, supportStateCallback);
  ros::Publisher footTrajectory_marker_pub = node.advertise<visualization_msgs::Marker>("footTrajectory", 30);
  // 创建足端轨迹Marker
  visualization_msgs::Marker footTrajectory[6];
  for (int i = 0; i < 6; ++i)
  {
    footTrajectory[i].header.frame_id = "/odom";
    footTrajectory[i].ns = std::string("footTrajectory") + std::to_string(i + 1);
    footTrajectory[i].action = visualization_msgs::Marker::ADD;
    footTrajectory[i].pose.orientation.w = 1.0;
    footTrajectory[i].type = visualization_msgs::Marker::LINE_STRIP;
    footTrajectory[i].lifetime = ros::Duration(2.5);
    footTrajectory[i].scale.x = 0.02;
    footTrajectory[i].color.r = 0.99;
    footTrajectory[i].color.g = 0.0;
    footTrajectory[i].color.b = 0.0;
    footTrajectory[i].color.a = 1.0;
  }

  int rosRate = 100; // 这个得快一点，才能及时更新作动器是否在运动
  ros::Rate rate(rosRate);

  Hexapod hexapod(node);

  while (node.ok())
  {

    // 步态运动
    if (msgFlag == true)
    {

      publishWalkingTrajctory(hexapod, statesList, footTrajectory, footTrajectory_marker_pub);

      statesList.clear();
      msgFlag = false;
    }

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
};
