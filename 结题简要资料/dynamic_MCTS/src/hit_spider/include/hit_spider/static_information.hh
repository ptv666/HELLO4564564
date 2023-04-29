// 静态数据：地图信息、静态坐标变换

#include "hit_spider/util.hh"

// #define terrain_file_name "/home/ptw/dynamic_MCTS/src/hit_spider/config/terrain/随机大块.txt"
// #define terrain_file_name "/home/ptw/dynamic_MCTS/src/hit_spider/config/terrain/连续壕沟.txt"
#define terrain_file_name "/home/ptw/dynamic_MCTS/src/hit_spider/config/terrain/中间空.txt"

namespace Static_Information
{
    MatrixX3 get_now_Feasible_foot_position(const hit_spider::hexapod_Base_Pose &pose);

    extern const grid_map::GridMap mapData;
    extern const MatrixX3 Feasible_foot_position;
}