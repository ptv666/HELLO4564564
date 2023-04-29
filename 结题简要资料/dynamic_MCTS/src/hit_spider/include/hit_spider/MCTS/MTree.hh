// 蒙特卡洛搜索树类头文件——声明类和函数

#ifndef HIT_SPIDER_DBM_MTREE_H
#define HIT_SPIDER_DBM_MTREE_H

#include "hit_spider/util.hh"
#include "hit_spider/MCTS/planning.hh"

struct MNode;                             // MTCS树节点声明
typedef std::shared_ptr<MNode> MNode_ptr; // MCTS树节点指针

struct BestChiledParameter
{
    // UCB的权重分配并没有考虑静态稳定裕度
    float C = 0.0;  // UCB
    float w1 = 0.0; // simLengthAverage，向后模拟仿真的平均拓展步长
    float w2 = 0.0; // steplengthAverage，根节点到该节点的平均拓展步长
    float w3 = 0.0; // disP2ME，父节点到该节点的距离
    float w4 = 0.0;
    float w5 = 0.0;
    int simStepNum = 0;
};

struct MNode
{
    // 构造函数：make_shared创建共享指针时会调用构造函数
    MNode() : Parent(nullptr), nodeDepth(0), visitTimes(0),
              disParentToMe(0), simMaxDis(0), average_step_length(0)
    {
    }

    MNode(const hit_spider::hexapod_State &state_input) : Parent(nullptr), nodeDepth(0), visitTimes(0),
                                                          disParentToMe(0), simMaxDis(0), average_step_length(0)
    {
        this->element = state_input;
    }

    hit_spider::hexapod_State element;                        // 节点对应的状态
    std::vector<hit_spider::hexapod_State> alternativeStates; // 备选状态集，主要是下一步的支撑状态、质心位姿、摆动腿落足点

    MNode_ptr Parent;             // 父节点指针
    std::set<MNode_ptr> children; // 节点的儿子节点构成的集合

    int nodeDepth;  // 节点的深度
    int visitTimes; // 节点访问次数

    double disParentToMe;       // 到父节点的距离
    double simMaxDis;           // 向后仿真出来的最远位置x坐标
    double average_step_length; // 从"最根节点"到该节点当前位置的平均步长
};

class MTree
{
public:
    MTree(MNode_ptr &root_input); // 构造函数，设置根节点和它的depth
    void Set_PAR(const BestChiledParameter &PAR);
    void Set_PAR2(const BestChiledParameter &PAR2);

    /**
     * @ : 加入一个新的儿子节点
     * @description:
     * @param {MNode} *node：儿子节点指针
     * @param {MNode} *parent：对应的父节点指针
     * @return {*}
     */
    void addNodeChild(MNode_ptr &node, MNode_ptr &parent);

    /**
     * @ : 获取当前根节点的最大拓展深度
     * @description:
     * @return {*}
     */
    int getMaxDepth();

    //------MCTS搜索树算法------

    /**
     * @ :寻找当前滑动根节点的下一层最优子节点
     * @description:
     * @param {BestChiledParameter} PAR
     * @return {*}
     */
    MNode_ptr findBestChild_Expand(const MNode_ptr &node_findBestChild);
    MNode_ptr findBestChild_Sliding_Root();

    /**
     * @ : 拓展阶段
     * @description:
     * @param {MNode_ptr} &node_expand
     * @return {*}
     */
    MNode_ptr MCTS_Expansion(MNode_ptr &node_expand);

    /**
     * @ : 选择和拓展阶段，选择一个节点，拓展它
     * @description:
     * @return {*}
     */
    MNode_ptr MCTS_SelectionAndExpansion();

    std::vector<hit_spider::hexapod_State> MCTS_Simulation(MNode_ptr &node_sim);

    /**
     * @ : 对得分进行反向传播
     * @description:
     * @param {MNode_ptr} &node_back
     * @return {*}
     */
    bool MCTS_Backpropagation(MNode_ptr &node_back);

    // private:
    MNode_ptr root;             // “最”根节点
    MNode_ptr Sliding_New_root; // 滑动根节点

    BestChiledParameter PAR;  // selectionAndExpansion时使用的参数
    BestChiledParameter PAR2; // sliding根节点时使用的参数
};

#endif