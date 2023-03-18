/*
 * @Author: ptw 1515901920@qq.com
 * @Date: 2023-03-17 19:07:42
 * @LastEditors: ptw 1515901920@qq.com
 * @LastEditTime: 2023-03-18 12:23:14
 * @FilePath: /robot_state_transition/test/test_CWC_region.cpp
 * @Description:
 */
#define BOOST_TEST_MODULE transition

#include <boost/test/included/unit_test.hpp>
#include "Bretl.hh"
#include "solve_LP_GLPK.hh"
#include <time.h>  //时间
#include <iomanip> //规范化输出信息

BOOST_AUTO_TEST_SUITE(test_CWC)

BOOST_AUTO_TEST_CASE(test_CWC_4_1)
{
    clock_t start1, end1;
    start1 = clock();

    Robot_State_Transition::MatrixXX Contacts(4, 3), Normals(4, 3);
    Contacts << 0.3, 0.2, -0.4,
        0.3, -0.2, -0.4,
        -0.3, 0.2, -0.4,
        -0.3, -0.2, -0.4;
    Normals << 0, 0, 1,
        0, 0, 1,
        0, 0, 1,
        0, 0, 1;
    double mass = 45;
    double mu = 0.5;

    auto Vertices = Robot_State_Transition::comput_friction_region(Contacts, Normals, mu, mass);

    end1 = clock(); // 计算第一阶段的时间
    std::cout << "------ ------ ------ ------ ------ ------" << std::endl;
    std::cout << "四个简单接触点用时:\t" << std::setprecision(8) << (double)(end1 - start1) / CLOCKS_PER_SEC << std::endl;
    std::cout << Vertices << std::endl;
    std::cout << "四个法向量分别为:" << std::endl;
    std::cout << Normals << std::endl;


    BOOST_CHECK(4 == 4);
}

BOOST_AUTO_TEST_CASE(test_CWC_4_2)
{
    clock_t start1, end1;
    start1 = clock();

    Robot_State_Transition::MatrixXX Contacts(4, 3), Normals(4, 3);
    Contacts << 0.3, 0.2, -0.4,
        0.3, -0.2, -0.4,
        -0.3, 0.2, -0.4,
        -0.3, -0.2, -0.4;
    Normals << 0, (double)-1 / 2, sqrt(3) / 2,
        0, 0, 1,
        0, (double)-1 / 2, sqrt(3) / 2,
        0, 0, 1;
    double mass = 45;
    double mu = 0.5;

    auto Vertices = Robot_State_Transition::comput_friction_region(Contacts, Normals, mu, mass);

    end1 = clock(); // 计算第一阶段的时间

    std::cout << "------ ------ ------ ------ ------ ------" << std::endl;
    std::cout << "四个稍微复杂接触点用时:\t" << std::setprecision(8) << (double)(end1 - start1) / CLOCKS_PER_SEC << std::endl;
    std::cout << Vertices << std::endl;
    std::cout << "四个法向量分别为:" << std::endl;
    std::cout << Normals << std::endl;


    BOOST_CHECK(0 == 0);
}

BOOST_AUTO_TEST_CASE(test_CWC_4_2_2)
{
    clock_t start1, end1;
    start1 = clock();

    Robot_State_Transition::MatrixXX Contacts(4, 3), Normals(4, 3);
    Contacts << 0.3, 0.2, -0.4,
        0.3, -0.2, -0.4,
        -0.3, 0.2, -0.4,
        -0.3, -0.2, -0.4;

    Normals << 0, (double)-1 / 2, sqrt(3) / 2,
        0, (double)1 / 2, sqrt(3) / 2,
        0, (double)-1 / 2, sqrt(3) / 2,
        0, (double)1 / 2, sqrt(3) / 2;

    double mass = 45;
    double mu = 0.5;

    auto Vertices = Robot_State_Transition::comput_friction_region(Contacts, Normals, mu, mass);

    end1 = clock(); // 计算第一阶段的时间

    std::cout << "------ ------ ------ ------ ------ ------" << std::endl;
    std::cout << "四个稍微复杂接触点用时:\t" << std::setprecision(8) << (double)(end1 - start1) / CLOCKS_PER_SEC << std::endl;
    std::cout << Vertices << std::endl;
    std::cout << "四个法向量分别为:" << std::endl;
    std::cout << Normals << std::endl;


    BOOST_CHECK(0 == 0);
}

BOOST_AUTO_TEST_CASE(test_CWC_4_3_0)
{
    srand(10);
    clock_t start1, end1;
    start1 = clock();

    Robot_State_Transition::MatrixXX Contacts(4, 3), Normals(4, 3);
    Contacts << 0.3, 0.2, -0.4,
        0.3, -0.2, -0.4,
        -0.3, 0.2, -0.4,
        -0.3, -0.2, -0.4;
    Normals.setRandom();
    Normals.rowwise().normalize();
    for (int i = 0; i < Normals.rows(); ++i)
    {
        if (Normals(i, 2) < 0)
        {
            Normals.row(i) = -Normals.row(i);
        }
    }

    double mass = 45;
    double mu = 0.5;

    auto Vertices = Robot_State_Transition::comput_friction_region(Contacts, Normals, mu, mass);

    end1 = clock(); // 计算第一阶段的时间

    std::cout << "------ ------ ------ ------ ------ ------" << std::endl;
    std::cout << "四个落足点，随机法向量，用时:\t" << std::setprecision(8) << (double)(end1 - start1) / CLOCKS_PER_SEC << std::endl;
    std::cout << Vertices << std::endl;
    std::cout << Vertices.transpose() << std::endl;
    std::cout << "四个法向量分别为:" << std::endl;
    std::cout << Normals << std::endl;


    BOOST_CHECK(0 == 0);
}

BOOST_AUTO_TEST_CASE(test_CWC_4_3)
{
    srand(11);
    clock_t start1, end1;
    start1 = clock();

    Robot_State_Transition::MatrixXX Contacts(4, 3), Normals(4, 3);
    Contacts << 0.3, 0.2, -0.4,
        0.3, -0.2, -0.4,
        -0.3, 0.2, -0.4,
        -0.3, -0.2, -0.4;
    Normals.setRandom();
    Normals.rowwise().normalize();
    for (int i = 0; i < Normals.rows(); ++i)
    {
        if (Normals(i, 2) < 0)
        {
            Normals.row(i) = -Normals.row(i);
        }
    }

    double mass = 45;
    double mu = 0.5;

    auto Vertices = Robot_State_Transition::comput_friction_region(Contacts, Normals, mu, mass);

    end1 = clock(); // 计算第一阶段的时间

    std::cout << "------ ------ ------ ------ ------ ------" << std::endl;
    std::cout << "四个落足点，随机法向量，用时:\t" << std::setprecision(8) << (double)(end1 - start1) / CLOCKS_PER_SEC << std::endl;
    std::cout << Vertices << std::endl;
    std::cout << Vertices.transpose() << std::endl;

    std::cout << "四个法向量分别为:" << std::endl;
    std::cout << Normals << std::endl;


    BOOST_CHECK(0 == 0);
}

BOOST_AUTO_TEST_CASE(test_CWC_4_4)
{
    srand(20);
    clock_t start1, end1;
    start1 = clock();

    Robot_State_Transition::MatrixXX Contacts(4, 3), Normals(4, 3);
    Contacts << 0.3, 0.2, -0.4,
        0.3, -0.2, -0.4,
        -0.3, 0.2, -0.4,
        -0.3, -0.2, -0.4;
    Normals.setRandom();
    Normals.rowwise().normalize();
    for (int i = 0; i < Normals.rows(); ++i)
    {
        if (Normals(i, 2) < 0)
        {
            Normals.row(i) = -Normals.row(i);
        }
    }


    double mass = 45;
    double mu = 0.5;

    auto Vertices = Robot_State_Transition::comput_friction_region(Contacts, Normals, mu, mass);

    end1 = clock(); // 计算第一阶段的时间

    std::cout << "------ ------ ------ ------ ------ ------" << std::endl;
    std::cout << "四个落足点，随机法向量，用时:\t" << std::setprecision(8) << (double)(end1 - start1) / CLOCKS_PER_SEC << std::endl;
    std::cout << Vertices << std::endl;
    std::cout << "四个法向量分别为:" << std::endl;
    std::cout << Normals << std::endl;


    BOOST_CHECK(0 == 0);
}

BOOST_AUTO_TEST_CASE(test_CWC_6_1)
{
    srand(20);
    clock_t start1, end1;
    start1 = clock();

    Robot_State_Transition::MatrixXX Contacts(6, 3), Normals(6, 3);
    Contacts << 0.3, 0.2, -0.4,
        0.3, -0.2, -0.4,
        -0.3, 0.2, -0.4,
        -0.3, -0.2, -0.4,
        0, 0.35, -0.4,
        0, -0.35, -0.4;
    Normals << 0, 0, 1,
        0, 0, 1,
        0, 0, 1,
        0, 0, 1,
        0, 0, 1,
        0, 0, 1;

    double mass = 45;
    double mu = 0.5;

    auto Vertices = Robot_State_Transition::comput_friction_region(Contacts, Normals, mu, mass);

    end1 = clock(); // 计算第一阶段的时间

    std::cout << "------ ------ ------ ------ ------ ------" << std::endl;
    std::cout << "六个落足点，随机法向量，用时:\t" << std::setprecision(8) << (double)(end1 - start1) / CLOCKS_PER_SEC << std::endl;
    std::cout << Vertices << std::endl;
    std::cout << "六个法向量分别为:" << std::endl;
    std::cout << Normals << std::endl;


    BOOST_CHECK(0 == 0);
}

BOOST_AUTO_TEST_CASE(test_CWC_6_2)
{
    srand(30);
    clock_t start1, end1;
    start1 = clock();

    Robot_State_Transition::MatrixXX Contacts(6, 3), Normals(6, 3);
    Contacts << 0.3, 0.2, -0.4,
        0.3, -0.2, -0.4,
        -0.3, 0.2, -0.4,
        -0.3, -0.2, -0.4,
        0, 0.35, -0.4,
        0, -0.35, -0.4;
    Normals.setRandom();
    Normals.rowwise().normalize();
    for (int i = 0; i < Normals.rows(); ++i)
    {
        if (Normals(i, 2) < 0)
        {
            Normals.row(i) = -Normals.row(i);
        }
    }

    double mass = 45;
    double mu = 0.5;

    auto Vertices = Robot_State_Transition::comput_friction_region(Contacts, Normals, mu, mass);

    end1 = clock(); // 计算第一阶段的时间

    std::cout << "------ ------ ------ ------ ------ ------" << std::endl;
    std::cout << "六个落足点，随机法向量，用时:\t" << std::setprecision(8) << (double)(end1 - start1) / CLOCKS_PER_SEC << std::endl;
    std::cout << Vertices << std::endl;
    std::cout << "六个法向量分别为:" << std::endl;
    std::cout << Normals << std::endl;


    BOOST_CHECK(0 == 0);
}

BOOST_AUTO_TEST_CASE(test_CWC_6_3)
{
    srand(40);
    clock_t start1, end1;
    start1 = clock();

    Robot_State_Transition::MatrixXX Contacts(6, 3), Normals(6, 3);
    Contacts << 0.3, 0.2, -0.4,
        0.3, -0.2, -0.4,
        -0.3, 0.2, -0.4,
        -0.3, -0.2, -0.4,
        0, 0.35, -0.4,
        0, -0.35, -0.4;
    Normals.setRandom();
    Normals.rowwise().normalize();
    for (int i = 0; i < Normals.rows(); ++i)
    {
        if (Normals(i, 2) < 0)
        {
            Normals.row(i) = -Normals.row(i);
        }
    }

    double mass = 45;
    double mu = 0.5;

    auto Vertices = Robot_State_Transition::comput_friction_region(Contacts, Normals, mu, mass);

    end1 = clock(); // 计算第一阶段的时间

    std::cout << "------ ------ ------ ------ ------ ------" << std::endl;
    std::cout << "六个落足点，随机法向量，用时:\t" << std::setprecision(8) << (double)(end1 - start1) / CLOCKS_PER_SEC << std::endl;
    std::cout << Vertices << std::endl;
    std::cout << "六个法向量分别为:" << std::endl;
    std::cout << Normals << std::endl;


    BOOST_CHECK(0 == 0);
}

BOOST_AUTO_TEST_CASE(test_CWC_6_4)
{
    srand(50);
    clock_t start1, end1;
    start1 = clock();

    Robot_State_Transition::MatrixXX Contacts(6, 3), Normals(6, 3);
    Contacts << 0.3, 0.2, -0.4,
        0.3, -0.2, -0.4,
        -0.3, 0.2, -0.4,
        -0.3, -0.2, -0.4,
        0, 0.35, -0.4,
        0, -0.35, -0.4;
    Normals.setRandom();
    Normals.rowwise().normalize();
    for (int i = 0; i < Normals.rows(); ++i)
    {
        if (Normals(i, 2) < 0)
        {
            Normals.row(i) = -Normals.row(i);
        }
    }

    double mass = 45;
    double mu = 0.5;

    auto Vertices = Robot_State_Transition::comput_friction_region(Contacts, Normals, mu, mass);

    end1 = clock(); // 计算第一阶段的时间

    std::cout << "------ ------ ------ ------ ------ ------" << std::endl;
    std::cout << "六个落足点，随机法向量，用时:\t" << std::setprecision(8) << (double)(end1 - start1) / CLOCKS_PER_SEC << std::endl;
    std::cout << Vertices << std::endl;
    std::cout << "六个法向量分别为:" << std::endl;
    std::cout << Normals << std::endl;


    BOOST_CHECK(0 == 0);
}

BOOST_AUTO_TEST_CASE(test_CWC_6_5)
{
    srand(60);
    clock_t start1, end1;
    start1 = clock();

    Robot_State_Transition::MatrixXX Contacts(6, 3), Normals(6, 3);
    Contacts << 0.3, 0.2, -0.4,
        0.3, -0.2, -0.4,
        -0.3, 0.2, -0.4,
        -0.3, -0.2, -0.4,
        0, 0.35, -0.4,
        0, -0.35, -0.4;
    Normals.setRandom();
    Normals.rowwise().normalize();
    for (int i = 0; i < Normals.rows(); ++i)
    {
        if (Normals(i, 2) < 0)
        {
            Normals.row(i) = -Normals.row(i);
        }
    }

    double mass = 45;
    double mu = 0.5;

    auto Vertices = Robot_State_Transition::comput_friction_region(Contacts, Normals, mu, mass);

    end1 = clock(); // 计算第一阶段的时间

    std::cout << "------ ------ ------ ------ ------ ------" << std::endl;
    std::cout << "六个落足点，随机法向量，用时:\t" << std::setprecision(8) << (double)(end1 - start1) / CLOCKS_PER_SEC << std::endl;
    std::cout << Vertices << std::endl;
    std::cout << "六个法向量分别为:" << std::endl;
    std::cout << Normals << std::endl;


    BOOST_CHECK(0 == 0);
}

BOOST_AUTO_TEST_CASE(test_CWC_6_6)
{
    srand(31);
    clock_t start1, end1;
    start1 = clock();

    Robot_State_Transition::MatrixXX Contacts(6, 3), Normals(6, 3);
    Contacts << 0.3, 0.2, -0.4,
        0.3, -0.2, -0.4,
        -0.3, 0.2, -0.4,
        -0.3, -0.2, -0.4,
        0, 0.35, -0.4,
        0, -0.35, -0.4;
    Normals.setRandom();
    Normals.rowwise().normalize();
    for (int i = 0; i < Normals.rows(); ++i)
    {
        if (Normals(i, 2) < 0)
        {
            Normals.row(i) = -Normals.row(i);
        }
    }

    double mass = 45;
    double mu = 0.5;

    auto Vertices = Robot_State_Transition::comput_friction_region(Contacts, Normals, mu, mass);

    end1 = clock(); // 计算第一阶段的时间

    std::cout << "------ ------ ------ ------ ------ ------" << std::endl;
    std::cout << "六个落足点，随机法向量，用时:\t" << std::setprecision(8) << (double)(end1 - start1) / CLOCKS_PER_SEC << std::endl;
    std::cout << Vertices << std::endl;
    std::cout << "六个法向量分别为:" << std::endl;
    std::cout << Normals << std::endl;


    BOOST_CHECK(0 == 0);
}

BOOST_AUTO_TEST_CASE(test_CWC_6_7)
{
    srand(31);
    clock_t start1, end1;
    start1 = clock();

    Robot_State_Transition::MatrixXX Contacts(6, 3), Normals(6, 3);
    Contacts << 0.3, 0.2, -0.4,
        0.3, -0.2, -0.4,
        -0.3, 0.2, -0.4,
        -0.3, -0.2, -0.4,
        0, 0.35, -0.4,
        0, -0.35, -0.4;
    Normals.setRandom();
    Normals.rowwise().normalize();
    for (int i = 0; i < Normals.rows(); ++i)
    {
        if (Normals(i, 2) < 0)
        {
            Normals.row(i) = -Normals.row(i);
        }
    }

    double mass = 45;
    double mu = 0.5;

    auto Vertices = Robot_State_Transition::comput_friction_region(Contacts, Normals, mu, mass);

    end1 = clock(); // 计算第一阶段的时间

    std::cout << "------ ------ ------ ------ ------ ------" << std::endl;
    std::cout << "六个落足点，随机法向量，用时:\t" << std::setprecision(8) << (double)(end1 - start1) / CLOCKS_PER_SEC << std::endl;
    std::cout << Vertices << std::endl;
    std::cout << "六个法向量分别为:" << std::endl;
    std::cout << Normals << std::endl;


    BOOST_CHECK(0 == 0);
}

BOOST_AUTO_TEST_SUITE_END()