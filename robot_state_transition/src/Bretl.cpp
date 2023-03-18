/*
 * @Author: ptw 1515901920@qq.com
 * @Date: 2023-03-16 21:11:07
 * @LastEditors: ptw 1515901920@qq.com
 * @LastEditTime: 2023-03-18 12:14:51
 * @FilePath: /robot_state_transition/src/Bretl.cpp
 * @Description: pypoman.Bretl.py的复现
 */
#include "Bretl.hh"
namespace Robot_State_Transition
{
    // Vertex_Planar部分
    Vertex_Planar::Vertex_Planar()
    {
        this->point.setZero();
        next = nullptr;
        expanded = false;
    }

    Vertex_Planar::Vertex_Planar(point_Planar point_input)
    {
        this->point = point_input;
        next = nullptr;
        expanded = false;
    }

    Vertex_Planar *Vertex_Planar::expand(VectorX &g,
                                         const MatrixXX &A, const VectorX &b,
                                         const MatrixXX &D, const VectorX &d)
    {
        // 优化方向
        //  v1 = this
        Vertex_Planar *v2 = this->next;
        point_Planar direction;

        direction(0) = (v2->point)(1) - (this->point)(1); // 拓展方向(y,-x)垂直于edge
        direction(1) = this->point(0) - v2->point(0);
        direction.normalize();

        // 进行优化
        point_Planar point_boundary;
        bool flag = optimize_direction(direction, g, A, b, D, d, point_boundary);
        if (!flag)
        {
            this->expanded = true;
            return nullptr;
        }

        // 是否添加添加新得到的边界点
        point_Planar p1 = point_boundary - this->point;
        point_Planar p2 = v2->point - this->point;

        double dis_expand = abs(p1(0) * p2(1) - p1(1) * p2(0));
        if (dis_expand < Threshold_Expand)
        {
            this->expanded = true;
            return nullptr;
        }
        else
        {
            Vertex_Planar *vnew = new Vertex_Planar(point_boundary);
            // vnew连接原来的两个点
            vnew->next = this->next;
            this->next = vnew;
            this->expanded = false;
            return vnew;
        }
    }

    // Polygon_Planar部分

    Polygon_Planar::Polygon_Planar(Vertex_Planar *v1, Vertex_Planar *v2, Vertex_Planar *v3)
    {
        v1->next = v2;
        v2->next = v3;
        v3->next = v1;
        vertices.push_back(v1);
        vertices.push_back(v2);
        vertices.push_back(v3);
    }

    Polygon_Planar::Polygon_Planar(const Vertex_Planar &v1, const Vertex_Planar &v2, const Vertex_Planar &v3)
    {
        Vertex_Planar *pv1 = new Vertex_Planar(v1);
        Vertex_Planar *pv2 = new Vertex_Planar(v2);
        Vertex_Planar *pv3 = new Vertex_Planar(v3);

        pv1->next = pv2;
        pv2->next = pv3;
        pv3->next = pv1;

        this->vertices.push_back(pv1);
        this->vertices.push_back(pv2);
        this->vertices.push_back(pv3);
    }

    Polygon_Planar::~Polygon_Planar()
    {
        for (auto iter = this->vertices.begin(); iter != this->vertices.end(); ++iter)
        {
            delete *iter;
        }
    }

    bool Polygon_Planar::all_expanded() const
    {
        for (auto iter = vertices.begin(); iter != vertices.end(); ++iter)
        {
            if (!(*iter)->expanded)
            {
                return false;
            }
        }
        return true;
    }

    void Polygon_Planar::iter_expand(VectorX g,
                                     const MatrixXX &A, const VectorX &b,
                                     const MatrixXX &D, const VectorX &d,
                                     const int &max_iter)
    {
        int nb_iter = 0;
        Vertex_Planar *v = this->vertices.at(0);
        while (!(this->all_expanded()) && nb_iter < max_iter)
        {
            if (v->expanded)
            {
                v = v->next;
                continue;
            }
            Vertex_Planar *vnew = v->expand(g, A, b, D, d);
            if (vnew == nullptr)
            {
                continue;
            }
            this->vertices.push_back(vnew);
            ++nb_iter;
        }
    }

    void Polygon_Planar::sort_vertices()
    {
        double minsd = 1e10;
        auto ibottom = this->vertices.cbegin();
        Vertex_Planar *v;
        for (auto iter = this->vertices.cbegin(); iter != this->vertices.cend(); ++iter)
        {
            v = *iter;
            if (v->point(1) + v->next->point(1) < minsd)
            {
                ibottom = iter;
                minsd = v->point(1) + v->next->point(1);
            }
        }

        std::vector<Vertex_Planar *> newVertices;
        v = *ibottom;
        do
        {
            newVertices.push_back(v);
            v = v->next;
        } while (v != *ibottom);

        std::reverse(newVertices.begin(), newVertices.end()); // 反序
        newVertices.insert(newVertices.begin(), *(newVertices.end() - 1));
        newVertices.pop_back();

        this->vertices = newVertices;
    }

    MatrixXX Polygon_Planar::export_vertices(double min_dist) const
    {
        std::vector<Vertex_Planar *> newVertices;
        newVertices.push_back(*(this->vertices.begin()));
        for (auto iter = (this->vertices.cbegin() + 1); iter != (this->vertices.cend() - 1); ++iter)
        {
            Vertex_Planar *vcur = *iter;
            Vertex_Planar *vlast = *(newVertices.end() - 1);
            if ((vcur->point - vlast->point).norm() > min_dist)
            {
                newVertices.push_back(vcur);
            }
        }
        newVertices.push_back(*(this->vertices.end() - 1));

        // 转换为矩阵
        MatrixXX output_matrix;
        output_matrix.resize(2, newVertices.size());
        int num_col = 0;
        for (auto iter = newVertices.begin(); iter != newVertices.end(); ++iter, ++num_col)
        {
            output_matrix.col(num_col) = (*iter)->point;
        }

        return output_matrix;
    }

    // 这里面会改变优化目标g
    bool optimize_direction(const point_Planar &vdir, VectorX &g,
                            const MatrixXX &A, const VectorX &b,
                            const MatrixXX &D, const VectorX &d,
                            point_Planar &result_opt)
    {
        int g_rows = g.rows();
        g(g_rows - 2) = -vdir(0); // x
        g(g_rows - 1) = -vdir(1); // y
        VectorX vec_opt;
        bool flag = solve_LP_GLPK(A, b, D, d, g, vec_opt);
        if (flag)
        {
            result_opt << vec_opt(g_rows - 2), vec_opt(g_rows - 1);
        }

        return flag;
    }

    bool optimize_angle(const double theta, VectorX &g,
                        const MatrixXX &A, const VectorX &b,
                        const MatrixXX &D, const VectorX &d,
                        point_Planar &result_opt)
    {
        point_Planar vdir;
        vdir(0) = cos(theta);
        vdir(1) = sin(theta);

        bool flag = optimize_direction(vdir, g, A, b, D, d, result_opt);
        return flag;
    }

    Polygon_Planar compute_polygon(VectorX &g,
                                   const MatrixXX &A, const VectorX &b,
                                   const MatrixXX &D, const VectorX &d,
                                   double *init_angle,
                                   int max_iter)
    {
        bool flag_angle = false;
        if (init_angle == nullptr)
        {
            flag_angle = true;
            init_angle = new double((double)rand() / RAND_MAX * PI);
        }
        double theta = *init_angle;
        double step_angle = (double)2 / 3 * PI;

        MatrixXX init_vertices;
        init_vertices.resize(2, 3); // 三个初始点
        int init_num = 0;
        point_Planar point;

        // 第一个初始点
        while (true)
        {
            bool flag = optimize_angle(theta, g, A, b, D, d, point);
            if (flag)
            {
                init_vertices.col(init_num++) = point;
                break;
            }
        }

        // debug
        // std::cout << init_vertices << std::endl;

        // 另外两个初始点
        while (init_num < 3 && max_iter > 0)
        {
            theta += step_angle;
            if (theta >= 2 * PI)
            {
                step_angle *= 0.25 + 0.5 * (double)random() / RAND_MAX;
                theta += step_angle - 2 * PI;
            }

            bool flag = optimize_angle(theta, g, A, b, D, d, point);
            if (flag)
            {

                // debug
                // std::cout << "优化出的点为:\n"
                //           << point << std::endl;

                // 防止初始点之间距离过小
                if (init_num == 1)
                {
                    if ((point - init_vertices.col(0)).norm() < Threshold_Init)
                    {
                        continue;
                    }
                    init_vertices.col(init_num++) = point;

                    // debug
                    // std::cout << init_vertices << std::endl;

                    --max_iter;
                }
                else if (init_num == 2)
                {
                    if ((point - init_vertices.col(0)).norm() < Threshold_Init || (point - init_vertices.col(1)).norm() < Threshold_Init)
                    {
                        continue;
                    }
                    init_vertices.col(init_num++) = point;

                    // debug
                    // std::cout << init_vertices << std::endl;

                    --max_iter;
                }
            }
        }

        assert(init_num == 3);
        Vertex_Planar *v1 = new Vertex_Planar(init_vertices.col(0));
        Vertex_Planar *v2 = new Vertex_Planar(init_vertices.col(1));
        Vertex_Planar *v3 = new Vertex_Planar(init_vertices.col(2));

        Polygon_Planar polygon(v1, v2, v3);
        polygon.iter_expand(g, A, b, D, d, max_iter);

        if (flag_angle)
        {
            delete init_angle;
        }

        return polygon;
    }

    //------
    MatrixXX project_polytope_bretl(const MatrixXX &E, const VectorX &f,
                                    const MatrixXX &A, const VectorX &b,
                                    const MatrixXX &D, const VectorX &d,
                                    double *init_angle,
                                    int max_iter, const double max_radius)
    {
        // 投影到二维平面上
        assert(E.rows() == 2 && f.rows() == 2);

        // 加入范围不等式约束
        // Inequality constraints: A_ext * [ x  u  v ] <= b_ext iff
        // (1) A * x <= b and (2) |u|, |v| <= max_radius
        MatrixXX A_ext;
        const int rows_A = A.rows(); // 需要是const才能在下面使用
        const int cols_A = A.cols();

        A_ext.resize(rows_A + 4, cols_A + 2);
        A_ext.setZero();
        A_ext.block(0, 0, rows_A, cols_A) = A;
        A_ext(rows_A, cols_A) = 1;
        A_ext(rows_A + 1, cols_A) = -1;
        A_ext(rows_A + 2, cols_A + 1) = 1;
        A_ext(rows_A + 3, cols_A + 1) = -1;

        VectorX b_ext;
        b_ext.resize(rows_A + 4);
        b_ext.setZero();
        b_ext.block(0, 0, rows_A, 1) = b;
        b_ext.block(rows_A, 0, 4, 1).array() = max_radius;

        // 加入等式约束
        // Equality constraints: C_ext * [ x  u  v ] == d_ext iff
        // (1) C * x == d and (2) [ u  v ] == E * x + f
        MatrixXX D_ext;
        const int rows_D = D.rows();
        const int cols_D = D.cols();
        D_ext.resize(rows_D + 2, cols_D + 2);
        D_ext.setZero();
        D_ext.block(0, 0, rows_D, cols_D) = D;
        D_ext.block(rows_D, 0, 2, cols_D) = E;
        D_ext.block(rows_D, cols_D, 2, 2) = -1 * Eigen::Matrix<double, 2, 2>::Identity();

        VectorX d_ext;
        d_ext.resize(rows_D + 2, 1);
        d_ext.setZero();
        d_ext.block(0, 0, rows_D, 1) = d;
        d_ext.block(rows_D, 0, 2, 1) = -1 * f;

        VectorX g;
        g.resize(cols_A + 2, 1);
        g.setZero();

        // // debug
        // std::cout << "g = \n"
        //           << g << std::endl;
        // std::cout << "A_ext = \n"
        //           << A_ext << std::endl;
        // std::cout << "b_ext = \n"
        //           << b_ext << std::endl;
        // std::cout << "D_ext = \n"
        //           << D_ext << std::endl;
        // std::cout << "d_ext = \n"
        //           << d_ext << std::endl;

        Polygon_Planar polygon = compute_polygon(g, A_ext, b_ext, D_ext, d_ext, init_angle, max_iter);

        polygon.sort_vertices();
        MatrixXX point_set = polygon.export_vertices();

        return point_set;
    }

    namespace
    {
        Matrix3 crossMatrix(const Vector3 &x)
        {
            Matrix3 res;
            res.setZero();
            res(0, 1) = -x(2);
            res(0, 2) = x(1);
            res(1, 0) = x(2);
            res(1, 2) = -x(0);
            res(2, 0) = -x(1);
            res(2, 1) = x(0);
            return res;
        }
    }

    MatrixXX comput_friction_region(const MatrixX3 &contactPoints, const MatrixX3 &contactNormals_input, const double frictionCoefficient, double m)
    {
        double g = 9.81;
        // 接触点数量、法向量数量需要保持一致
        assert(contactPoints.rows() == contactNormals_input.rows());
        MatrixX3 contactNormals = contactNormals_input.rowwise().normalized(); // 支撑点法向量归一化

        const int contact_Num = contactPoints.rows();
        const int cone_edge_number = 4;
        const double delta_theta = 2 * (double)M_PI / cone_edge_number; // 线性化摩擦锥需要的变化角度

        // CWC计算
        //------一个接触点一个接触点进行计算------
        //  变量定义参考质心动力学方程
        MatrixXX A1(6, 3 * contact_Num); // 质心动力学约束矩阵
        A1.setZero();
        VectorX u(6, 1);
        u.setZero();
        u(2, 0) = m * g; // 重力

        MatrixXX B(4 * contact_Num, 3 * contact_Num); // 线性摩擦锥约束矩阵
        B.setZero();

        MatrixXX A1_temp(6, 3); // A = [I3;Skew_symmetric_Matrix(接触点p)]
        A1_temp.setZero();
        A1_temp.block<3, 3>(0, 0).setIdentity();

        MatrixXX B_temp(3, 4);

        for (size_t i = 0; i < contact_Num; ++i)
        {
            Vector3 N = contactNormals.row(i);
            A1_temp.block<3, 3>(3, 0) = crossMatrix(contactPoints.row(i).transpose());

            // 计算与法向量垂直的切向量
            Vector3 T1, T2;
            T1 = N.cross(Vector3::UnitY());
            if (T1.norm() < 1e-5)
            {
                T1 = N.cross(Vector3::UnitX());
            }
            T2 = N.transpose().cross(T1);

            T1.normalize();
            T2.normalize();

            // debug
            // std::cout << "T1 = \n"
            //           << T1 << std::endl;
            // std::cout << "T2 = \n"
            //           << T2 << std::endl;

            // 四棱锥的线性约束
            B_temp.col(0) = T1 - frictionCoefficient * N;
            B_temp.col(1) = T2 - frictionCoefficient * N;
            B_temp.col(2) = -1 * T1 - frictionCoefficient * N;
            B_temp.col(3) = -1 * T2 - frictionCoefficient * N;

            A1.block(0, 3 * i, 6, 3) = A1_temp;
            B.block(4 * i, 3 * i, 4, 3) = B_temp.transpose();
        }

        // c_xy = Ex + f
        MatrixXX E(2, 3 * contact_Num);
        VectorX f(2);
        E.row(0) = A1.row(4) / (-1 * m * g); // x坐标
        E.row(1) = A1.row(3) / (m * g);      // y坐标
        f.setZero();

        // Ax == b
        MatrixXX A(4, 3 * contact_Num);
        VectorX b(4);
        A.block(0, 0, 3, 3 * contact_Num) = A1.block(0, 0, 3, 3 * contact_Num);
        A.block(3, 0, 1, 3 * contact_Num) = A1.block(5, 0, 1, 3 * contact_Num);
        b.setZero();
        b(2) = m * g;

        // Dx <= d
        // MatrixXX D = B
        VectorX d(4 * contact_Num);
        d.setZero();

        // // debug
        // std::cout << "E = \n"
        //           << E << std::endl;
        // std::cout << "f = \n"
        //           << f << std::endl;
        // std::cout << "A = \n"
        //           << A << std::endl;
        // std::cout << "b = \n"
        //           << b << std::endl;
        // std::cout << "D = \n"
        //           << B << std::endl;
        // std::cout << "d = \n"
        //           << d << std::endl;

        srand(10);  //确保初始init_angle的角度相同，可以复现实验
        return project_polytope_bretl(E, f, B, d, A, b);
    }
}