#include "Bretl.hh"
#define BOOST_TEST_MODULE transition
#include <boost/test/included/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(Bretl)

BOOST_AUTO_TEST_CASE(test_01)
{
    const int n = 10;
    const int p = 2;

    Robot_State_Transition::MatrixXX A, D, E;
    Robot_State_Transition::VectorX b, d, f;

    // Ax <= b
    A.resize(2 * n, n);
    A.block<n, n>(0, 0) = Eigen::Matrix<double, n, n>::Identity();
    A.block<n, n>(n, 0) = -1 * Eigen::Matrix<double, n, n>::Identity();

    b.resize(2 * n, 1);
    b.setOnes();

    // Dx = d
    D.resize(1, n);
    D.setOnes();
    d.resize(1, 1);
    d.setZero();

    E.resize(p, n);
    E.setZero();
    E(0, 0) = 1;
    E(1, 1) = 1;
    f.resize(p, 1);
    f.setZero();

    Robot_State_Transition::MatrixXX vertices = Robot_State_Transition::project_polytope_bretl(E, f, A, b, D, d);

    std::cout << vertices << std::endl;
}

BOOST_AUTO_TEST_SUITE_END()