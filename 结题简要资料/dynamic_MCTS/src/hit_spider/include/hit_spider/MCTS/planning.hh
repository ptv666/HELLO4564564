// 规划过程中使用的函数声明

#ifndef HIT_SPIDER_PLANNING_H_
#define HIT_SPIDER_PLANNING_H_

#include "hit_spider/util.hh"                                        //数据类型
#include "hit_spider/hexapod.hh"                                     //求解逆运动学关节角
#include "hit_spider/static_information.hh"                          //寻找环境中可落足点
#include "hit_spider/robot_state_transition/Bretl.hh"                //CWC
#include "hit_spider/robot_state_transition/my_cdd.hh"               //双线性变换库
#include "hit_spider/robot_state_transition/kinematics_constrain.hh" //运动学约束
#include "hit_spider/robot_state_transition/dynamic_constrain.hh"    //动力学约束

typedef Eigen::Matrix<int, 6, 1> hexapod_SupportState;                   // 支撑状态（容错状态）
typedef Eigen::Matrix<int, Eigen::Dynamic, 6> hexapod_SupportState_List; // 支撑状态集合（容错腿状态集合）

// 备选落足点集合
struct Footholds
{
    std::vector<Vector3, Eigen::aligned_allocator<Vector3>> leg[6];
};

namespace planning
{
    extern const hexapod_SupportState_List initialSupportList; // 42个支撑状态构成集合

   //根据当前废腿信息确定下一步的支撑腿状态集合
    hexapod_SupportState_List findAvailable_SupportStates(const hit_spider::hexapod_State &hexapodState);
    hexapod_SupportState_List findAvailable_SupportStates_and_maxLength(const hit_spider::hexapod_State &hexapodState, std::vector<double> &max_length);

    // Now到Next的步长计算
    // 考虑静态稳定性约束的步长
    double get_CWC_base_length(const hit_spider::hexapod_State &hexapodState);
    // 考虑运动学约束的步长
    double get_kin_length(const hit_spider::hexapod_State &hexapodState);
    // 考虑动力学约束的步长
    double get_dynamic_length(const hit_spider::hexapod_State &hexapodState);

    // Next摆动腿可行落足点集合
    Footholds getAvailableFootholds(const hit_spider::hexapod_State &hexapodState);
    // 专家法选择摆动腿落足点
    void swingLegFoot_position_Expert(hit_spider::hexapod_State &hexapodState);
    // 专家法计算下一个state
    void get_nextState_Expert(hit_spider::hexapod_State &hexapodState);
    // 随机法计算下一个state
    void get_nextState_Rondom(hit_spider::hexapod_State &hexapodState);
    // 确定备选状态集——专家法（专家法确定步长、摆动腿落足点）
    std::vector<hit_spider::hexapod_State> get_NextState_list(const hit_spider::hexapod_State &hexapodState);

}

#endif
