// 双线性变换库的用户接口

#include "hit_spider/util.hh"
#include <cdd/setoper.h>
#include <cdd/cdd.h>

namespace Robot_State_Transition
{
    // 顶点转换为不等式
    bool vertices_to_H(const MatrixXX &vertices_set, MatrixXX &H_output, VectorX &h_output);
}