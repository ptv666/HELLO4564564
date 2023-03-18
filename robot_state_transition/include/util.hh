/*
 * @Author: ptw 1515901920@qq.com
 * @Date: 2023-03-15 11:11:28
 * @LastEditors: ptw 1515901920@qq.com
 * @LastEditTime: 2023-03-17 14:54:53
 * @FilePath: /robot_state_transition/include/util.hh
 * @Description: 状态转移过程中可能需要用到的数据结构
 */

#ifndef ROBOT_STATE_TRANSITION_UTIL_HH
#define ROBOT_STATE_TRANSITION_UTIL_HH

#include <iostream>
#include <vector>
#include <map>
#include <utility>
#include <eigen3/Eigen/Dense>

namespace Robot_State_Transition
{
#define PI 3.14159265358979323846	/* pi */
#define PI_2 1.57079632679489661923 /* pi/2 */
#define PI_4 0.78539816339744830962 /* pi/4 */

	typedef double value_type;

	typedef Eigen::Matrix<double, 2, 1> point_Planar; // 二维平面上的点
	typedef Eigen::Matrix<value_type, 3, 1> Vector3;  // 三维点
	typedef Eigen::Matrix<value_type, 4, 1> Vector4;  // 四维列向量，用于表示齐次坐标

	// 想把三维力和六维Wrench写在一个更泛化的函数中
	typedef Eigen::Matrix<value_type, Eigen::Dynamic, 1> VectorX;
	typedef Eigen::Matrix<value_type, Eigen::Dynamic, Eigen::Dynamic> MatrixXX;
	typedef std::vector<MatrixXX, Eigen::aligned_allocator<MatrixXX>> vector_MatrixXX;

	typedef Eigen::Matrix<value_type, 3, 3> Matrix3;								   // 3×3矩阵，用于Jacobian矩阵
	typedef Eigen::Matrix<value_type, Eigen::Dynamic, 3> MatrixX3;					   // 三维向量按行排列的，可用于表示接触点、法向量序列（不知道有多少接触点）、雅克比矩阵序列
	typedef Eigen::Matrix<value_type, Eigen::Dynamic, 6> MatrixX6;					   // 可用于表示Wrench
	typedef std::vector<Matrix3, Eigen::aligned_allocator<Matrix3>> vector_Matrix3;	   // 用于多条腿的关节角
	typedef std::vector<MatrixX3, Eigen::aligned_allocator<MatrixX3>> vector_MatrixX3; // 用于接触点生成的线性摩擦锥的Vertices顶点集合
	typedef std::vector<MatrixX6, Eigen::aligned_allocator<MatrixX6>> vector_MatrixX6; // 用于接触点Wrench的Vertices顶点集合

}

#endif // FEASIBLE_WRENCH_POLYTOPE_UTIL_HH