// 数据类型定义、一些常量、常用接口函数的声明

#ifndef HIT_SPIDER_UTIL_HH
#define HIT_SPIDER_UTIL_HH

#include <iostream>
#include <string>
#include <fstream>   //文件操作
#include <sstream>   //字符串操作,搭配fstream读取文件数据
#include <vector>    //STL容器(可以尝试使用list保存备选节点)
#include <algorithm> //STL算法
#include <utility>   //std::pair
#include <memory>    //shared智能指针动态内存分配

#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>            //三维点
#include <geometry_msgs/Transform.h>        //坐标变换
#include <geometry_msgs/TransformStamped.h> //用于广播的坐标变换(带时间戳)
#include <sensor_msgs/JointState.h>         //关节角度

#include <std_msgs/ColorRGBA.h>        //颜色,初始化Marker
#include <visualization_msgs/Marker.h> //Marker可视化

#include <tf2_ros/buffer.h>                       //动态坐标变换缓存
#include <tf2_ros/transform_listener.h>           //坐标变换接收者
#include <tf2_ros/transform_broadcaster.h>        //动态坐标变换发布
#include <tf2_ros/static_transform_broadcaster.h> //静态坐标变换发布
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  //tf2消息转换为geometry_msgsg消息
#include <tf2/LinearMath/Quaternion.h>            //欧拉角转四元数

#include <grid_map_ros/grid_map_ros.hpp> //grid_map地图
#include <grid_map_msgs/GridMap.h>

#include "hit_spider/hexapod_RPY.h"       //机体姿态
#include "hit_spider/hexapod_Base_Pose.h" //机体位姿
#include "hit_spider/FeetPosition.h"      //六个腿落足点
#include "hit_spider/hexapod_State.h"     //机器人状态State

#define PI 3.14159265358979323846

//------数据类型的定义------
typedef Eigen::Matrix<double, 2, 1> point_Planar;          // 二维平面上的点
typedef Eigen::Matrix<double, 3, 1> Vector3;               // 三维点
typedef Eigen::Matrix<double, 6, 1> Vector6;               // 六维状态
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorX;  // 动态维数向量
typedef Eigen::Matrix<int, Eigen::Dynamic, 1> VectorXi;
typedef Eigen::Matrix<double, 3, 3> Matrix3;               // 3×3矩阵
typedef Eigen::Matrix<double, 6, 3> Matrix63;              // 6×3矩阵，六条腿的关节角度
typedef Eigen::Matrix<double, Eigen::Dynamic, 3> MatrixX3; // 三维向量按行排列的，可用于表示接触点、法向量序列（不知道有多少接触点）、雅克比矩阵序列
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXX;

Matrix3 crossMatrix(const Vector3 &x); // 求三维向量对应的反对称矩阵

//------机器人相关常量,全局共享变量声明------
#define FAULT_LEG 1
#define NORMAL_LEG 0
#define SUPPORT 1
#define SWING 0

extern const float body_fixJi_theta[6];                         // 六条腿在Truck分布的角度
extern const std::vector<std::string> HEXAPOD_JOINT_STATE_NAME; // 18个关节名称
extern const Eigen::Isometry3d TransMatrix_FixJi_Body[6];       // FixJi描述base的齐次变换矩阵(FixJi就是根关节默认位姿)
extern const Eigen::Isometry3d TransMatrix_Body_2_fixJi[6];     // base描述FixGu的齐次变换矩阵(FixGu就是扇形坐标系)
extern const Vector3 Hexapod_defaultFoothold[6];                // 默认落足位置(相对于base坐标系)

//------常用函数
/**
 * @ : 生成区间内的一个随机整数
 * @description:
 * @param {int} minValue：区间下界
 * @param {int} maxValue：区间上界
 * @return {*}：生成的随机整数
 */
int rand_customization(int minValue, int maxValue);

/**
 * @ : 获取世界系下机器人Truck机体的齐次变换矩阵
 * @description:
 * @param {hit_spider::hexapod_Base_Pose} &pose：机器人位姿，欧拉角和位置
 * @return {Eigen::Isometry3d}：EIgen的4×4齐次变换矩阵
 */
Eigen::Isometry3d getTrans_W_B(const hit_spider::hexapod_Base_Pose &pose);

geometry_msgs::Transform getTransform_W_B(const hit_spider::hexapod_Base_Pose &pose);

/**
 * @ : 初始化树节点的状态，质心位姿、六条腿落足点均是默认位置、所有腿均是支撑腿、没有废腿、节点访问次数0
 * @description:
 * @param {MCTS_State} &hexapodState
 * @return {*}
 */
void init_hexapodState(hit_spider::hexapod_State &hexapodState);

#endif