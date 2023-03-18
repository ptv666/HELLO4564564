/*
 * @Author: ptw 1515901920@qq.com
 * @Date: 2023-03-15 14:09:33
 * @LastEditors: ptw 1515901920@qq.com
 * @LastEditTime: 2023-03-17 19:00:12
 * @FilePath: /robot_state_transition/a.cpp
 * @Description:
 */
#include <iostream>
#include <vector>

#include <eigen3/Eigen/Dense>
typedef Eigen::Matrix<double, 2, 1> point_Planar;
typedef Eigen::Matrix<double, 3, 1> Vector3;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorX;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXX;

int main(int argc, char *argv[])
{
    VectorX a(5, 1);
    VectorX b(6);
    a.setRandom();
    b.setRandom();
    std::cout << a.rows() << '\t' << a.cols() << '\n'
              << a.transpose() << std::endl;
    std::cout << b.rows() << '\t' << b.cols() << '\n'
              << b.transpose() << std::endl;

    return 0;
}
