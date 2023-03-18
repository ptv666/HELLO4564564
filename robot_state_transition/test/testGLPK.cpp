/*
 * @Author: ptw 1515901920@qq.com
 * @Date: 2023-03-15 15:56:27
 * @LastEditors: ptw 1515901920@qq.com
 * @LastEditTime: 2023-03-15 16:32:06
 * @FilePath: /robot_state_transition/test/testGLPK.cpp
 * @Description: 测试
 */
#define BOOST_TEST_MODULE transition
#include <boost/test/included/unit_test.hpp>
#include "solve_LP_GLPK.hh"

BOOST_AUTO_TEST_SUITE(flat_ground)

BOOST_AUTO_TEST_CASE(test_GLP_FR)
{
    Robot_State_Transition::MatrixXX A(3, 2), D;
    Robot_State_Transition::VectorX b(3, 1), d, g(2, 1);
    Robot_State_Transition::VectorX minbounds, maxbounds, result_x;
    double result_optimization = -1;
    A << 1, 1,
        -1, 1,
        0, -1;
    b << 1, 1, 0;
    g << 0, 1;
    bool flag = Robot_State_Transition::solve_LP_GLPK(A, b, D, d, g, minbounds, maxbounds, true, &result_x, &result_optimization);

    if (flag)
    {
        std::cout << "求解成功\n"
                  << "最优值为:\t" << result_optimization << std::endl
                  << "决策变量为:\n"
                  << result_x.transpose() << std::endl;
    }

    Robot_State_Transition::VectorX result_true(2);
    result_true << 0, 1;
    BOOST_CHECK(flag); // 优化成功还是失败
    BOOST_CHECK(result_x == result_true);
    BOOST_CHECK(result_optimization == 1);
}

BOOST_AUTO_TEST_CASE(test_GLP_LO)
{
    Robot_State_Transition::MatrixXX A(3, 3), D;
    Robot_State_Transition::VectorX b(3, 1), d, g(3, 1);
    Robot_State_Transition::VectorX minbounds(3), maxbounds, result_x;
    minbounds.setZero();
    double result_optimization;
    A << 1, 1, 1,
        10, 4, 5,
        2, 2, 6;
    b << 100, 600, 300;
    g << 10, 6, 4;
    bool flag = Robot_State_Transition::solve_LP_GLPK(A, b, D, d, g, minbounds, maxbounds, true, &result_x, &result_optimization);
    if (flag)
    {
        std::cout << "求解成功\n"
                  << "最优值为:\t" << result_optimization << std::endl
                  << "决策变量为:\n"
                  << result_x.transpose() << std::endl;
    }
}

BOOST_AUTO_TEST_CASE(test_GLP_UNBND)
{
    Robot_State_Transition::MatrixXX A(3, 3), D;
    Robot_State_Transition::VectorX b(3, 1), d, g(3, 1);
    Robot_State_Transition::VectorX minbounds, maxbounds, result_x;
    double result_optimization;
    A << 1, 1, 1,
        10, 4, 5,
        2, 2, 6;
    b << 100, 600, 300;
    g << 10, 6, 4;
    bool flag = Robot_State_Transition::solve_LP_GLPK(A, b, D, d, g, minbounds, maxbounds, true, &result_x, &result_optimization);
    if (flag)
    {
        std::cout << "求解成功\n"
                  << "最优值为:\t" << result_optimization << std::endl
                  << "决策变量为:\n"
                  << result_x.transpose() << std::endl;
    }
}

BOOST_AUTO_TEST_SUITE_END()