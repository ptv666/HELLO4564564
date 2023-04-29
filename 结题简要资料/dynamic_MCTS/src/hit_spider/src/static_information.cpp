// 静态消息定义

#include "hit_spider/static_information.hh"

namespace Static_Information
{
    namespace
    {
        grid_map::GridMap init_grid_map(const std::string &file_name)
        {
            setlocale(LC_ALL, ""); // 可输出中文

            // grid_map对象,地图大小设置,坐标系设置
            grid_map::GridMap mapData;
            mapData.setGeometry(grid_map::Length(30, 20), 0.1);
            mapData.setFrameId("odom");

            // 添加图层
            mapData.add("elevation");                                                                      // 高程图
            mapData.add("3D_feeling", grid_map::Matrix::Zero(mapData.getSize()(0), mapData.getSize()(1))); // 显示3d感觉的图层

            for (grid_map::GridMapIterator iterator(mapData); !iterator.isPastEnd(); ++iterator)
            {
                mapData.at("3D_feeling", *iterator) = -0.6;
            }

            // 1.从文件读取地图数据,加入到elevation图层中
            std::ifstream fin(file_name);
            std::string line_info, input_result;
            std::vector<float> vectorData;
            if (fin) // 有该文件
            {
                std::cout << "*************************************************" << std::endl;
                std::cout << "Find map file: " << file_name << std::endl;
                std::cout << "*************************************************" << std::endl;
                while (getline(fin, line_info)) // line中不包括每行的换行符
                {
                    std::stringstream inputDATA(line_info);
                    while (inputDATA >> input_result)
                    {
                        vectorData.push_back(atof(input_result.c_str()));
                    }
                }
                for (int i = 0; i < (int)vectorData.size(); i += 2)
                {
                    grid_map::Position gm_position;
                    gm_position << vectorData[i], vectorData[i + 1];
                    grid_map::Index gm_index;
                    mapData.getIndex(gm_position, gm_index);
                    mapData.at("elevation", gm_index) = 0;
                    mapData.at("3D_feeling", gm_index) = 0;

                    // debug
                    // std::cout << vectorData[i] << "," << vectorData[i + 1] << std::endl; // 输出地形坐标
                }
            }
            else // 没有该文件
            {
                ROS_FATAL("没有指定地图文件,请检查路径");
                ros::shutdown();
                exit(1);
            }

            // 2.把机器人初始(默认)落足点加入到elevation图层中
            for (int i = 0; i < 6; ++i)
            {
                grid_map::Index gm_index;
                grid_map::Position gm_position;
                gm_position.x() = Hexapod_defaultFoothold[i].x();
                gm_position.y() = Hexapod_defaultFoothold[i].y();
                mapData.getIndex(gm_position, gm_index);
                mapData.at("elevation", gm_index) = 0;
            }

            // 3.打印消息
            ROS_INFO("创建的grid_map地图大小为 %d x %d m (%d x %d cells).", (int)mapData.getLength().x(), (int)mapData.getLength().y(), (int)mapData.getSize()(0), (int)mapData.getSize()(1));

            return mapData;
        }

        MatrixX3 init_Feasible_foot_position()
        {
            MatrixX3 AA;
            int num = 0;

            for (grid_map::GridMapIterator it(mapData); !it.isPastEnd(); ++it)
            {
                grid_map::Position position;
                mapData.getPosition(*it, position);
                if (mapData.at("elevation", *it) == 0)
                {
                    ++num;
                }
            }

            AA.resize(num, 3);
            int row_index = 0;

            for (grid_map::GridMapIterator it(mapData); !it.isPastEnd(); ++it)
            {
                grid_map::Position position;
                mapData.getPosition(*it, position);
                if (mapData.at("elevation", *it) == 0)
                {
                    AA.row(row_index++) << position.x(), position.y(), 0;
                }
            }

            return AA;
        }
    }

    MatrixX3 get_now_Feasible_foot_position(const hit_spider::hexapod_Base_Pose &pose)
    {
        MatrixX3 AA;
        int num = 0;

        grid_map::Position position;
        grid_map::Position center(pose.position.x, pose.position.y);
        double radius = 1.5;

        for (grid_map::CircleIterator it(mapData, center, radius); !it.isPastEnd(); ++it)
        {
            mapData.getPosition(*it, position);
            if (mapData.at("elevation", *it) == 0)
            {
                ++num;
            }
        }

        AA.resize(num, 3);
        int row_index = 0;

        for (grid_map::CircleIterator it(mapData, center, radius); !it.isPastEnd(); ++it)
        {
            grid_map::Position position;
            mapData.getPosition(*it, position);
            if (mapData.at("elevation", *it) == 0)
            {
                AA.row(row_index++) << position.x(), position.y(), 0;
            }
        }

        return AA;
    }

    const grid_map::GridMap mapData = init_grid_map(terrain_file_name);
    const MatrixX3 Feasible_foot_position = init_Feasible_foot_position();
}
