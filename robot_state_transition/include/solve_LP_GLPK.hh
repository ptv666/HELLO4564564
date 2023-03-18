/*
 * @Author: ptw 1515901920@qq.com
 * @Date: 2023-03-15 10:58:41
 * @LastEditors: ptw 1515901920@qq.com
 * @LastEditTime: 2023-03-17 09:30:18
 * @FilePath: /robot_state_transition/include/solve_LP_GLPK.hh
 * @Description:使用GLPK求解LP问题的接口头文件
 */
#include <iostream>
#include <glpk.h>
#include "util.hh"
#ifndef ROBOT_STATE_TRANSITION_SOLVE_LP_GLPK_HH
#define ROBOT_STATE_TRANSITION_SOLVE_LP_GLPK_HH
namespace Robot_State_Transition
{

	/**
	 * min g^T * x;
	 * s.t.
	 * Ax<=b;
	 * Dx=d;
	 * @description: 为GLPK设置的求解器接口，能输入Eigen矩阵形式的LP问题
	 * @param A:线性不等式约束的系数矩阵
	 * @param b:线性不等式约束的右边值
	 * @param D:线性等式约束的系数矩阵
	 * @param d:线性等式约束的右边值
	 * @param g:线性目标值
	 * @param vec_opt:优化的最优决策变量
	 * @return bool：优化是否成功
	 */
	bool solve_LP_GLPK(const MatrixXX &A, const VectorX &b,
					   const MatrixXX &D, const VectorX &d,
					   const VectorX &g,
					   VectorX &vec_opt);

	/**
	 * min(max) g^T * x;
	 * s.t.
	 * Ax<=b;
	 * Dx=d;
	 * @description: 为GLPK设置的求解器接口，能输入Eigen矩阵形式的LP问题
	 * @param A:线性不等式约束的系数矩阵
	 * @param b:线性不等式约束的右边值
	 * @param D:线性等式约束的系数矩阵
	 * @param d:线性等式约束的右边值
	 * @param g:线性目标值
	 * @param minBounds:决策变量的上界
	 * @param maxBounds:决策变量的下界
	 * @param direction:优化的方向，默认是最小值
	 * @param *result_x:接受变量的指针
	 * @param *direction:接受最优值的指针
	 * @return bool：求解器是否求解成功、目标函数值、决策变量值
	 */
	bool solve_LP_GLPK(const MatrixXX &A, const VectorX &b,
					   const MatrixXX &D, const VectorX &d,
					   const VectorX &g,
					   const VectorX &minBounds, const VectorX &maxBounds,
					   bool direction = false,
					   VectorX *result_x = nullptr,
					   double *result_optimization = nullptr);
}
#endif