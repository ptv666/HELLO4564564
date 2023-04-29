// MCTS搜索树类具体定义

#include "hit_spider/MCTS/MTree.hh"

//---构造与初始化
MTree::MTree(MNode_ptr &root_input) : root(root_input), Sliding_New_root(root_input)
{
    this->root->visitTimes = 0;
    this->root->nodeDepth = 1;
    this->root->Parent = nullptr;
    this->root->disParentToMe = 0;
    this->root->simMaxDis = 0;
    this->root->average_step_length = 0;
}

void MTree::Set_PAR(const BestChiledParameter &PAR)
{
    this->PAR = PAR;
}
void MTree::Set_PAR2(const BestChiledParameter &PAR2)
{
    this->PAR2 = PAR2;
}

// 拓展树的操作
void MTree::addNodeChild(MNode_ptr &node, MNode_ptr &parent)
{
    node->Parent = parent;
    node->nodeDepth = parent->nodeDepth + 1;
    parent->children.insert(node);
}

// 获取树的深度
int getDepth(const MNode_ptr &node)
{
    if (node == nullptr) // 树还没有被使用过
    {
        return 0;
    }

    int res = 1;
    std::set<MNode_ptr> sons = node->children;
    for (auto iter = sons.cbegin(); iter != sons.cend(); ++iter)
    {
        res = std::max(res, getDepth(*iter) + 1);
    }
    return res;
}
int MTree::getMaxDepth()
{
    return getDepth(this->root);
}

//------MCTS搜索树核心------
MNode_ptr MTree::findBestChild_Expand(const MNode_ptr &node_findBestChild)
{
    auto best_iter = node_findBestChild->children.end(); // 指向一个set集合最后一个元素的下一个位置
    float best_Value = -1;                               // 最优儿子对应的UCB值

    for (auto iter = node_findBestChild->children.begin(); iter != node_findBestChild->children.end(); ++iter)
    {
        // UCB计算：为什么要开三次方根呢?   ->   为了归一化
        float tmpValue = this->PAR.w1 * pow(((*iter)->simMaxDis) / (double)this->PAR.simStepNum, (double)1 / 3.0f) +
                         this->PAR.w2 * pow(((*iter)->average_step_length), (double)1 / 3.0f) +
                         this->PAR.C * sqrt(log(float(this->Sliding_New_root->visitTimes)) / float((*iter)->visitTimes));

        if (tmpValue > best_Value) // 寻找UCB的最大值
        {
            best_Value = tmpValue;
            best_iter = iter;
        }
    }

    // 代表返回的值为正常节点
    if (best_iter != node_findBestChild->children.end())
    {
        return *best_iter;
    }
    else // 代表返回为被忽略的节点,所有的儿子节点全被忽略了
    {
        return node_findBestChild;
    }
}

MNode_ptr MTree::findBestChild_Sliding_Root()
{
    auto best_iter = this->Sliding_New_root->children.end(); // 指向一个set集合最后一个元素的下一个位置
    float best_Value = -1;                                   // 最优儿子对应的UCB值

    for (auto iter = this->Sliding_New_root->children.begin(); iter != this->Sliding_New_root->children.end(); ++iter)
    {
        // UCB计算：为什么要开三次方根呢?   ->   为了归一化
        float tmpValue = this->PAR2.w1 * pow(((*iter)->simMaxDis) / (float)this->PAR.simStepNum, (double)1 / 3.0f) +
                         this->PAR2.w2 * pow(((*iter)->average_step_length), (double)1 / 3.0f);

        if (tmpValue > best_Value) // 寻找UCB的最大值
        {
            best_Value = tmpValue;
            best_iter = iter;
        }
    }

    // 代表返回的值为正常节点
    if (best_iter != this->Sliding_New_root->children.end())
    {
        return *best_iter;
    }
    else // 代表返回为被忽略的节点,所有的儿子节点全被忽略了
    {
        return this->Sliding_New_root;
    }
}

MNode_ptr MTree::MCTS_SelectionAndExpansion()
{
    MNode_ptr root_ = this->Sliding_New_root; // 从搜索树的滑动根节点开始搜索

    // 死循环，一定要找到一个拓展节点出来！
    while (1)
    {
        // 节点未全部展开，那UCB最大值一定是未展开节点上出现
        if (root_->alternativeStates.size() != 0)
        {
            // 拓展一次该节点，返回拓展节点的指针
            MNode_ptr returnNode = this->MCTS_Expansion(root_);

            return returnNode; // 返回新拓展出的节点
        }

        // ！！！滑动后的序列并不好，一定要避免，出现这种情况就是UCB参数不合理！！！
        // 希望这个不要发生，否则滑动根节点就没有任何意义了!!!
        // 当前节点已经全部展开，但是所有备选儿子都是卡死的，没有一个真正的被加入到树中，所以该节点也是卡死节点
        if (root_->children.size() == 0)
        {
            // 将该节点从它的父节点中的子节点list中删除
            if (root_->Parent == nullptr) // 该节点就是根节点
            {
                ROS_ERROR("!!!蒙特卡洛搜索树初始就是卡死的!!!\n 请输入一个字符以继续:");
                getchar(); // 等到输入一个字符
            }
            // 并不是最根节点，还有希望求解，把该节点从它的父节点的children集合中删除
            root_->Parent->children.erase(root_);

            ROS_ERROR("滑动根节点并不好\n请输入一个字符以继续:"); // 期望这个不要发生，否则滑动根节点就没有任何意义了!!!

            root_ = root_->Parent; // 蒙特卡洛搜索树向前回溯一个节点
            getchar();
            continue;
        }
        else
        {
            //------当前滑动根节点节点已经全部展开，寻找它的最佳儿子节点，对最佳儿子节点再进行expandsion
            root_ = this->findBestChild_Expand(root_);
        }
    }
}

MNode_ptr MTree::MCTS_Expansion(MNode_ptr &node_expand)
{
    // 在备选状态中随机选择一个状态
    int index = rand_customization(0, node_expand->alternativeStates.size() - 1);
    MNode_ptr node = std::make_shared<MNode>(node_expand->alternativeStates[index]);
    node_expand->alternativeStates.erase(node_expand->alternativeStates.begin() + index); // 从父节点的备选状态中删除
    node->alternativeStates = planning::get_NextState_list(node->element);                // 新拓展节点的备选状态集合

    this->addNodeChild(node, node_expand); // 把新拓展的节点添加到树中

    // 当前节点距离它的父节点距离
    node->disParentToMe = node->element.base_Pose_Now.position.x - node_expand->element.base_Pose_Now.position.x;

    node->average_step_length = (node->element.base_Pose_Now.position.x - this->Sliding_New_root->element.base_Pose_Now.position.x) / (double)(node->nodeDepth - this->Sliding_New_root->nodeDepth);

    return node;
}

std::vector<hit_spider::hexapod_State> MTree::MCTS_Simulation(MNode_ptr &node_sim)
{
    std::vector<hit_spider::hexapod_State> smi_state_squence;   // 仿真序列
    hit_spider::hexapod_State hexapodState = node_sim->element; // 向后仿真的state

    int stepCount = 0; // 当前仿真步数

    while (1)
    {
        ++stepCount;

        planning::get_nextState_Rondom(hexapodState); // 随机获取下一个支撑状态，专家法获取落足点

        // 机器人向后走
        hexapodState.base_Pose_Now = hexapodState.base_Pose_Next;
        hexapodState.feetPositionNow = hexapodState.feetPositionNext;
        hexapodState.faultLeg_State_Now = hexapodState.faultLeg_State_Next;
        hexapodState.support_State_Now = hexapodState.support_State_Next;

        smi_state_squence.push_back(hexapodState); // 存入仿真序列中

        if (stepCount >= this->PAR.simStepNum) // 仿真步数到了
        {
            // 仿真出来的最远距离
            node_sim->simMaxDis = hexapodState.base_Pose_Now.position.x;
            return smi_state_squence;
        }
    }
}

bool MTree::MCTS_Backpropagation(MNode_ptr &node_back)
{
    node_back->visitTimes += 1; // simulation后的叶节点访问次数加一
                                // 传播其余分值信息

    // 传播访问次数信息
    MNode_ptr node_ = node_back;
    while (1)
    {
        node_ = node_->Parent; // 不断向根节点方向传播

        if (node_ == nullptr) // 已经全部传播完了
        {
            break;
        }

        node_->visitTimes += 1; // 父节点的访问次数增加1
    }

    // 新拓展出的节点是否卡死
    if (node_back->alternativeStates.size() == 0)
    {
        // debug
        ROS_WARN("新拓展出来的节点是卡死的");
        node_back->Parent->children.erase(node_back);
        return false;
    }

    // 传播最远仿真距离
    node_ = node_back;
    while (1)
    {
        node_ = node_->Parent; // 不断向根节点方向传播

        if (node_ == nullptr) // 已经全部传播完了
        {
            break;
        }

        double yuanlai = this->PAR2.w1 * pow((node_->simMaxDis) / (double)this->PAR.simStepNum, (double)1 / 3.0f) + this->PAR2.w2 * pow(node_back->average_step_length, (double)1 / 3.0f);
        double tmpValue = this->PAR2.w1 * pow((node_back->simMaxDis) / (double)this->PAR.simStepNum, (double)1 / 3.0f) + this->PAR2.w2 * pow(node_back->average_step_length, (double)1 / 3.0f);

        if (tmpValue > yuanlai)
        {
            node_->simMaxDis = node_back->simMaxDis;
            node_->average_step_length = node_back->average_step_length;
        }
    }

    return true;
}