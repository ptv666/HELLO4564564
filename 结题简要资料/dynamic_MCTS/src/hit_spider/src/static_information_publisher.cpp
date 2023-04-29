// 静态信息发布者对象

#include "hit_spider/static_information.hh"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "static_information_publisher");
    ros::NodeHandle nh;

    //------静态地图相关------
    ros::Publisher gridMapPub = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true); // 地图锁存住

    ROS_INFO("话题发布的的grid_map地图大小为 %d x %d m (%d x %d cells).",
             (int)Static_Information::mapData.getLength().x(), (int)Static_Information::mapData.getLength().y(),
             (int)Static_Information::mapData.getSize()(0), (int)Static_Information::mapData.getSize()(1));

    grid_map_msgs::GridMap gm_message;                                                 // 创建grid_map消息
    grid_map::GridMapRosConverter::toMessage(Static_Information::mapData, gm_message); // 把grid_map地图转换为ros消息格式

    gridMapPub.publish(gm_message);

    //------ros回头------
    ros::spin();

    return 0;
}
