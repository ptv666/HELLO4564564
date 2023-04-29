// 蒙特卡洛搜索树主程序

#include "hit_spider/MCTS/MTree.hh"
#define num_Search_One_Step 500

static void print_once()
{
    static bool has_done = false;
    if (!has_done)
    {

        std::cout << std::endl;
        std::cout << "  N:"
                  << "当前根节点访问的次数 / 滑动根节点前的剩余访问次数" << std::endl;
        std::cout << "  Dx:"
                  << "滑动根节点的位置" << std::endl;
        std::cout << "  |->Ex:"
                  << "当前根节点最远拓展距离（相对于根节点）" << std::endl;
        std::cout << "  MaxN:"
                  << "搜索树最远拓展位置" << std::endl;
        std::cout << "  MaxS:"
                  << "搜索树最远仿真距离" << std::endl;
        std::cout << "  x:"
                  << "当前模拟节点的位置" << std::endl;
        std::cout << "  sim:"
                  << "当前节点向后仿真距离" << std::endl;

        has_done = true;
    }
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "main_planning_node");
    ros::NodeHandle nh_;
    ros::Duration(0.5).sleep(); // 等待其他的程序启动

    // 发布消息以绘制支撑状态
    ros::Publisher supportStatePub = nh_.advertise<hit_spider::hexapod_State>("supportStateTopic", 500, true); // supportStateTopic

    // ------可调参赛数
    // rand函数的随机发生器  --->  随机选择初始state的可复现性
    srand((unsigned)time(NULL));
    // srand(10);
    static int depth_count = num_Search_One_Step;
    // ------可调参赛数

    // 初始化一个state
    hit_spider::hexapod_State hexapodState;
    init_hexapodState(hexapodState);

    // 初始化蒙特卡洛搜索树对象
    MNode_ptr nodeStart = std::make_shared<MNode>();
    nodeStart->element = hexapodState;
    nodeStart->alternativeStates = planning::get_NextState_list(nodeStart->element); // 为初始节点添加备选状态

    // 初始化蒙特卡洛搜索树
    MTree tree(nodeStart);

    BestChiledParameter PAR;
    PAR.C = 1.0f;
    PAR.w1 = 3.0f; // 向后模拟仿真距离的权重
    PAR.w2 = 1.0f; // 平均步长的权重
    PAR.simStepNum = 20;
    BestChiledParameter PAR2;
    PAR2.C = 0.0f;
    PAR2.w1 = PAR.w1;
    PAR2.w2 = PAR.w2;
    tree.Set_PAR(PAR);
    tree.Set_PAR2(PAR2);

    while (1)
    {
        // 1.MCTS蒙特卡洛搜索树选择阶段，选择一个节点并进行拓展
        MNode_ptr tmpNode = tree.MCTS_SelectionAndExpansion();

        // 没有有效节点可以扩展，机器人已经卡死了
        if (tmpNode == nodeStart) // nodeStart是根节点
        {
            ROS_ERROR("机器人初始位置就是卡死的,输入一个字符结束吧");
            getchar();
        }

        // 2.对选择的节点进行模拟仿真，获得节点得分，获得最大仿真距离
        std::vector<hit_spider::hexapod_State> sequenceState = tree.MCTS_Simulation(tmpNode);

        // 3.反向传播得分情况、节点访问次数情况
        bool Flag_ = tree.MCTS_Backpropagation(tmpNode);

        // 更新最远位置信息（在外部就存放了最远仿真距离，那在节点内部还需要存放这个值吗）
        static float maxSimX = 0.0f;  // MCTS搜索到的最远位置
        static float maxNodeX = 0.0f; // MCTS搜索树当前拓展到的最远位置
        if (Flag_)
        {
            if (maxSimX < tmpNode->simMaxDis)
            {
                maxSimX = tmpNode->simMaxDis;
            }
            if (maxNodeX < tmpNode->element.base_Pose_Now.position.x)
            {
                maxNodeX = tmpNode->element.base_Pose_Now.position.x;
            }
        }

        print_once();
        std::cout << "\r"
                  << "N:" << std::setw(5) << tree.root->visitTimes << "/" << --depth_count << "  Dx:" << std::setw(5) << tree.Sliding_New_root->element.base_Pose_Now.position.x << "  |->Ex:" << std::setw(10) << maxNodeX - tree.Sliding_New_root->element.base_Pose_Now.position.x
                  << "  MaxN:" << std::setw(10) << maxNodeX << "  MaxS:" << std::setw(10) << maxSimX << " x:" << std::setw(10) << tmpNode->element.base_Pose_Now.position.x << " sim:" << std::setw(10) << tmpNode->simMaxDis << std::flush;

        // 移动根节点：当前根节点访问次数超过了depth_count 或者  搜索树当前拓展的及最远节点距离滑动根节点超过0.8m
        if ((depth_count <= 0) || (maxNodeX - tree.Sliding_New_root->element.base_Pose_Now.position.x > 0.8f))
        {
            // 找出最佳子节点，然后移动根节点到该子节点

            MNode_ptr sliding_node = tree.findBestChild_Sliding_Root();

            // 剪枝，最优子节点的父节点其他的儿子节点全部舍弃
            sliding_node->Parent->children.clear();
            sliding_node->Parent->children.insert(sliding_node);

            tree.Sliding_New_root = sliding_node; // 移动根节点
            depth_count = num_Search_One_Step;
        }

        // 结束的终止条件：当前根节点最大访问次数超过，不用终止，一直向后寻找就行
        // if (((tree.root->element.visitTimes > 100) && (maxSimX - maxNodeX < 0.2)) || (stopFlag_ == false) || maxNodeX > 1)
        if (maxNodeX > 7.2) //     10.2m   6.99m 2.99m
        {
            ROS_WARN("找到结果了!!!\n输入一个字符继续");
            getchar();
            // 记录解序列
            std::vector<hit_spider::hexapod_State> solutionStates;
            while (1)
            {
                // 向前不断寻找父节点
                solutionStates.push_back(tmpNode->element);
                std::cout << tmpNode << std::endl;
                tmpNode = tmpNode->Parent;
                if (tmpNode == nullptr) // 到达最根节点
                {
                    break;
                }
            }
            std::cout << "求解出来的简化前的序列长度:\t" << solutionStates.size() << std::endl;

            std::reverse(solutionStates.begin(), solutionStates.end());
            (solutionStates.rbegin())->remarks.data = "end_flag";

            while (1)
            {
                std::cout << "press button to pubulish:" << std::endl;
                getchar();
                for (int i = 0; i < (int)solutionStates.size(); i++)
                {
                    supportStatePub.publish(solutionStates[i]);
                    std::cout << "x:" << solutionStates[i].base_Pose_Now.position.x << std::endl;
                    ros::Duration(0.05).sleep();
                }

                std::cout << "是否还想再运行一次?(NO退出)" << std::endl;
                std::string str;
                std::cin >> str;
                if (str == "NO")
                {
                    std::cout << "结束了，只能重新运行了" << std::endl;
                    break;
                }
            }
            return 0;
        }
    }

    return 1;
}
