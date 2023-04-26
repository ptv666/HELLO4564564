classdef humanoid_robot<Gurobi_Interface
    properties
        two_feet_distance = 0.5;        %定义两脚之间的距离
        
        left_foot_init  = [0,0.25,0]';  %初始状态
        right_foot_init = [0,-0.25,0]';
        
        left_foot_goal  = [];   %目标状态   %3×1列向量
        right_foot_goal = [];
        
        %相关代价的权重
        weight_g = diag([100,100,20]);    %最后一步距离goal的权重
        weight_r = diag([10,10,5]);      %两步之间的权重--------%与goal相差大一点，这样确保能走到目标点
        weight_trim = 20;                  %trim的代价值
%         weight_r_final = diag([0,0,0]);   %走最后一步的代价--------MIT定义的两步之间的权重特殊之处
        
        N = -1;                 %步数，必须为偶数   %在构造函数中确定
        min_steps = -1;
        feet_state_list = [];   %落足状态集合--------%是待优化项
        
        %*****先定义一个全局尺寸，之后再改****
        square_l = 0.8;     %一步工作空间尺寸，正方形
%         rectangle_a = 0.4;  %长度
%         rectangle_b = 0.2;  %宽度
        delta_theta = 0.0;  %角度
        ground_size = 10;   %落足点的上下限
        
    end
    methods
        %实际上obj.N = obj.vars.Feet_State_List.size(2) = size(obj.vars.Feet_State_List.Index,2) = obj.vars.Trim.size(2)
        %构造函数，输入目标点，目标点是两腿中点，输入形式为:[x;y;theta]
        function obj=humanoid_robot(goal)
            obj = obj@Gurobi_Interface();
            
            %根据输入目标点提前确定可能的步数，创建足端序列变量
            obj.left_foot_goal  = [goal(1) - 0.5 * obj.two_feet_distance*sin(goal(3)), goal(2) + 0.5 * obj.two_feet_distance*cos(goal(3)), goal(3)]';
            obj.right_foot_goal = [goal(1) + 0.5 * obj.two_feet_distance*sin(goal(3)), goal(2) - 0.5*obj.two_feet_distance*cos(goal(3)), goal(3)]';
            
            %%验证中的东西，后期必然改动
            obj.N = norm([goal(1),goal(2)])/0.4;   %********--------0.15是估计的保守的平均移动步长，验证使用
            obj.N = 20;     %先手动输入一波，看看效果如何,效果还行
%             obj.N = ceil(obj.N*1.5);
            obj.min_steps = 2;

            if mod(obj.N,2)     %双足机器人的步数为偶数更容易处理
                obj.N = obj.N + 1;
            end
%             obj.delta_theta = ((goal(3)-0)/obj.N) * 1.02;     %********--------验证使用，让yaw角几乎每一步都要走选择最大值，但是还留有一丝余地，这样设置是可以加速优化时间，但是不能避障
            obj.delta_theta = pi/6;
            obj.N = obj.N + 2;  %创建落足状态序列，2+N加入初始状态
            obj.min_steps = obj.min_steps + 2;  %最小步数也加2
            obj.addVariable('Feet_State_List', 'C', [3,obj.N], -obj.ground_size, obj.ground_size);
            
            %约束初始状态
%             left_init_index  = obj.vars.Feet_State_List.Index(:,1);                 %先获取初始状态在决策变量矩阵中的索引值
%             right_init_index = obj.vars.Feet_State_List.Index(:,2);
            obj.vars.Feet_State_List.lb(:,1)  = obj.left_foot_init;     %修改初始状态的lb和ub间接控制住初始状态
            obj.vars.Feet_State_List.ub(:,1)  = obj.left_foot_init;
            obj.vars.Feet_State_List.lb(:,2) = obj.right_foot_init;
            obj.vars.Feet_State_List.ub(:,2) = obj.right_foot_init;
        end
        
        %添加最后一步至目标点的代价值:(x_end-x_g)'Q(x_end-x_g)
        function obj=set_Goal_cost(obj)
            %获得最后状态向量在决策变量矩阵中的索引
            last_state_index_left  = obj.vars.Feet_State_List.Index(:,end-1);
            last_state_index_right = obj.vars.Feet_State_List.Index(:,end);
            %x'Qx
            Q = zeros(obj.num_vars, obj.num_vars);   %二次目标矩阵Q需要完全构建，再相加
            Q(last_state_index_left, last_state_index_left)   = obj.weight_g;   %3×3矩阵
            Q(last_state_index_right, last_state_index_right) = obj.weight_g;
            %-2x'Qxg
            c = zeros(obj.num_vars, 1);
            c(last_state_index_left)  = -2*obj.weight_g*obj.left_foot_goal;     %3×1矩阵
            c(last_state_index_right) = -2*obj.weight_g*obj.right_foot_goal;
            %xg'Qxg
            alpha = obj.left_foot_goal'*obj.weight_g*obj.left_foot_goal + obj.right_foot_goal'*obj.weight_g*obj.right_foot_goal; %常数
            obj.addCost(Q, c, alpha);   %添加序列末状态和目标状态之间的差值
        end
        
        %添加两步之间的代价值(x2-x1)'Q(x2-x1)
        function obj=set_Run_cost(obj)
            for i=3:obj.N
                Q = zeros(obj.num_vars, obj.num_vars);
                %x1'Qx1
                index_x1 = obj.vars.Feet_State_List.Index(:,i);
                Q(index_x1, index_x1) = obj.weight_r;
                %x2'Qx2
                index_x2 = obj.vars.Feet_State_List.Index(:,i-2);
                Q(index_x2, index_x2) = obj.weight_r;
                %-2x2'Qx1
                Q(index_x1, index_x2) = -obj.weight_r;
                Q(index_x2, index_x1) = -obj.weight_r;
                
                obj.addCost(Q, [], []); %for循环内
            end
        end
        
        %添加单腿的xy工作空间约束
        %左脚限制右脚运动空间：中心：[xl+0.5sin(t_left),yl-0.5cos(t_left)]
        %右脚限制左脚运动空间：中心：[xl-0.5sin(t_right),yl+0.5cos(t_right)]
        %可行点到中心距离delta_x=a/2*cos(t_foot),delta_y=a/2*sin(t_foot)
        function obj=set_Leg_Workspace_constrain(obj)
            %先添加sin和cos，约束住了
            obj.addVariable('sin_t', 'C', [1 obj.N], -1, 1)	%每个落足点添加sin和cos
            obj.addVariable('cos_t', 'C', [1 obj.N], -1, 1)
            theta_index = reshape(obj.vars.Feet_State_List.Index(end,:),[],1);
            sin_t_index = reshape(obj.vars.sin_t.Index,[], 1);	%转成列向量
            cos_t_index = reshape(obj.vars.cos_t.Index,[], 1);
            for i=1:obj.N
                obj.addSinConstraint(theta_index(i), sin_t_index(i));
                obj.addCosConstraint(theta_index(i), cos_t_index(i));
            end
            
            %计算可行空间中心,要避免建立某些无用变量，或者需要很好的处理它。
            obj.addVariable('foot_feasible_center', 'C', [2 obj.N + 1], -obj.ground_size, obj.ground_size);	%------这里加1是为了把最后一步决定的虚拟下一步中心确定出来
            obj.vars.foot_feasible_center.lb(:,1) = reshape(obj.left_foot_init(1:2), [], 1);	%初始状态不需要计算可行空间，不允许Gurobi对其求解
            obj.vars.foot_feasible_center.ub(:,1) = reshape(obj.left_foot_init(1:2), [], 1);  
            obj.vars.foot_feasible_center.lb(:,2) = reshape(obj.right_foot_init(1:2),[], 1); 
            obj.vars.foot_feasible_center.ub(:,2) = reshape(obj.right_foot_init(1:2),[], 1); 
            
            center_index_x = reshape(obj.vars.foot_feasible_center.Index(1,:), [], 1);	%可行域中心索引，转成列向量
            center_index_y = reshape(obj.vars.foot_feasible_center.Index(2,:), [], 1);
            feet_index_x  = obj.vars.Feet_State_List.Index(1,:);        %落足点中心索引
            feet_index_y  = obj.vars.Feet_State_List.Index(2,:);
            %这一步的状态决定下一步的落足中心
            for i=3:obj.N + 1   %这里为了让最后一步也能约束上一步，所以加1计算虚拟中心
                Aeq = zeros(2, obj.num_vars);
                beq = zeros(2, 1);
                j = mod(i, 2);	%奇数代表左脚的可行域中心，偶数代表右脚可行域中心
                Aeq(1, center_index_x(i)) = -1;	%中心x的位置
                Aeq(1, sin_t_index(i-1))  = obj.two_feet_distance * (-1)^j;
                Aeq(1, feet_index_x(i-1)) = 1;
                beq(1, 1) = 0;
                Aeq(2, center_index_y(i)) = -1;	%中心y的位置
                Aeq(2, cos_t_index(i-1))  = obj.two_feet_distance * (-1)^(j+1);
                Aeq(2, feet_index_y(i-1)) = 1;
                beq(2, 1) = 0;
                temp_name = ['foot_feasible_center_con', num2str(i)];
                obj.addLinearConstraints_equal(Aeq, beq, temp_name);
            end
            
            %单腿运动空间限制为正方形 abs(x-x_center)<=正方形边长/2
            %循环1：这一步约束下一步，下一步在这一步形成的正方形内
            for i=3:obj.N
                A = zeros(2, obj.num_vars);
                b = zeros(2, 1);
                A(1, feet_index_x(i)) = 1;
                A(1, center_index_x(i)) = -1;
                b(1,1) = 0.5*obj.square_l;
                A(2, feet_index_y(i)) = 1;
                A(2, center_index_y(i)) = -1;
                b(2,1) = 0.5*obj.square_l;
                obj.addLinearConstraints_unequal(A,  b, 'WorkSpace_1_con');    %绝对值的一侧
                obj.addLinearConstraints_unequal(-A, b, 'WorkSpace_2_con');   %绝对值的另一侧
            end
            %循环2：下一步同时也约束这一步，这一步在下一步形成的正方形内
            for i=2:obj.N-1
                A = zeros(2, obj.num_vars);
                b = zeros(2,1);
                A(1, feet_index_x(i)) = 1;
                A(1, center_index_x(i+2)) = -1;  %下一步确定的下下一步落足可行域中心，该可行域同时也要约束当前这一步
                b(1,1) = 0.5*obj.square_l;
                A(2, feet_index_y(i)) = 1;
                A(2, center_index_y(i+2)) = -1;
                b(2,1) = 0.5*obj.square_l;
                obj.addLinearConstraints_unequal(A,  b, 'WorkSpace_1_con');    %绝对值的一侧
                obj.addLinearConstraints_unequal(-A, b, 'WorkSpace_2_con');   %绝对值的另一侧
            end
            
            %加入yaw角 theta_index已经在上面定义好了
            for i=3:obj.N
                A = zeros(1, obj.num_vars);
                b = zeros(1,1);
                A(1, theta_index(i))   = 1;
                A(1, theta_index(i-1)) = -1;
                b(1,1) = obj.delta_theta;
                obj.addLinearConstraints_unequal(A, b, 'yaw_1_con');    %绝对值的一侧
                obj.addLinearConstraints_unequal(-A, b, 'yaw_2_con');   %绝对值的另一侧
            end
        end
        
        %添加Trim解决步数不确定导致估计步数过多存在冗余的情况
        %只有合适的trim才能起到很好的剪枝效果，过大的trim可以使机器人以最激进的方式前进，trim过小则剪枝效果不好，实际上都是代价值在起作用
        function obj=set_Trim_flag(obj)
            obj.addVariable('Trim', 'B', [1 obj.N], 0, 1);
            trim_Index = reshape(obj.vars.Trim.Index, [], 1);   %获取Trim的索引供后面使用，转成列向量
            %--------trim这里需要判断步数是否超过3，如果没有超过3那么就可以直接return了，为了代码工程的完备性（能很好的处理goal和init重合的情况），对于其他的约束是否也需要呢？
            
            obj.vars.Trim.lb(1,1:2) = 0;        %第一步的Trim固定住
            obj.vars.Trim.ub(1,1:2) = 0;
            obj.vars.Trim.lb(1,end-1:end) = 1;  %最后一步的Trim固定住
            obj.vars.Trim.ub(1,end-1:end) = 1;
            
            %--------trim数组的特点：当前trim=1，则后面所有的trim全部为1，也就是说前面的trim小于后面的trim，1后面不能出现0
            
            for i=3:obj.N-2
                A = zeros(1, obj.num_vars);
                b = zeros(1,1);
                A(trim_Index(i)) = 1;
                A(trim_Index(i+1)) = -1;
                obj.addLinearConstraints_unequal(A, b, 'Trim_attribute_math')
            end
            
            %--------trim存在if的含义，为了用数学工具表示，一定要把if处理为通用的情形
            %为trim添加实际物理意义：trim=1的时候，该步和最后一步的状态相同
            feet_Index = obj.vars.Feet_State_List.Index;
            M = 999999;   %很大的数，trim=0就几乎没有约束，trim=1就约束紧
            for i=3:obj.N-2    %init和last已经被固定住了,它们对应的落足点坐标也不需要再被约束，尤其是init的落足点坐标 更是已经被确定了
                A = zeros(3, obj.num_vars);
                b = size(3,1);
                if mod(i,2) 
                    j = size(trim_Index, 1)-1;	%左脚
                else
                    j = size(trim_Index, 1);	%右脚
                end
                
                A(1, feet_Index(1,i)) = 1;      %对x
                A(1, feet_Index(1,j)) = -1;
               
                A(2, feet_Index(2,i)) = 1;  	%对y
                A(2, feet_Index(2,j)) = -1;

                A(3, feet_Index(3,i)) = 1;    	%对yaw
                A(3, feet_Index(3,j)) = -1;
                A(1:3, trim_Index(i)) = 1 * M;	%当trim=1，则abs(delta)<=0，也就是和末状态是相同的
                b(1:3,1) = 1 * M;
                obj.addLinearConstraints_unequal(A, b, 'Trim_attribute_physics_Fir');
                A = -A;
                A(1:3, trim_Index(i)) = 1 * M;
                obj.addLinearConstraints_unequal(A, b, 'Trim_attribute_physics_Sec');
            end
            %--------Trim与最小步数存在关系，机器人min_steps是对步数最激进的估计，那么min_steps就反过来对Trim有一个保守的约束（Trim存在一个最大值）
            
            A = zeros(1, obj.num_vars);
            b = zeros(1,1);
            A(1, trim_Index) = 1;
            b(1,1) = obj.N - obj.min_steps; %最大值设置是合理的，最少走min_steps步，总共有N步，所以trim最大值为N-min_step步
            obj.addLinearConstraints_unequal(A, b, 'Trim_con');
            
            %--------添加Trim的代价值，Trim=1的数量越多说明去除冗余步数的效果越好，那么期望该sum(Trim)大一些，目标值是求解最小值，所以对sum(Trim)取负数
            c = zeros(obj.num_vars, 1);
            c(trim_Index) = -obj.weight_trim;
            alpha = obj.N * obj.weight_trim;    %添加这个代价常量是为了使得Trim带来的总代价是正数
            obj.addCost([], c, alpha);
        end
        
        %添加落足点的region
        function obj=set_foot_region(obj,rectangle_region_con,init_region_num)
            % @rectangle_region_con是由creat_rectangle_region_con函数生成的结构体数组，结构体数组的行数表示了region的数量
            
            %添加控制落足点region的变量
            num_region = size(rectangle_region_con,1);
            obj.addVariable('H', 'B', [num_region obj.N], 0, 1);
            H_index = obj.vars.H.Index; %大小为num_region×obj.N
            feet_Index = obj.vars.Feet_State_List.Index;
            
            %--------把起点约束住，由于最后一步不一定能到达终点，所以不对它施加约束
            if length(init_region_num)==1   %初始时左脚和右脚可以不在同一个region中
                init_region_num(2) = init_region_num(1);
            end
            obj.vars.H.lb(:,1) = 0; obj.vars.H.lb(init_region_num(1),1) = 1;    %对init_left
            obj.vars.H.ub(:,1) = 0; obj.vars.H.ub(init_region_num(1),1) = 1;
            obj.vars.H.lb(:,2) = 0; obj.vars.H.lb(init_region_num(2),2) = 1;    %对init_right
            obj.vars.H.ub(:,2) = 0; obj.vars.H.ub(init_region_num(2),2) = 1;
            
            %--------添加H矩阵的物理意义1：每一步只能落在一个region
            for i=1:obj.N
                Aeq = zeros(1, obj.num_vars);
                beq = zeros(1,1);
                Aeq(1,H_index(:,i)) = 1;    %H矩阵每一列和为1
                beq(1,1) = 1;
                obj.addLinearConstraints_equal(Aeq, beq, 'H_Physic_Fir');
            end
            
            %--------添加H矩阵的物理意义2：落在region就会收到region区域的几何约束
            M = 999999;
            for i=3:obj.N   %不对初始状态的region进行实际的物理约束，所以初始落足点的region任意给定都可以，但是为了未来拓展更多接口不要任意给，要准确的给定
                for j=1:num_region
                    %对H矩阵中每一个元素分别进行实际物理约束：如果H(j,i)=1，则第i步落在了第j个region中
                    %对于creat_rectangle_region_con函数，对每一个region它会输出四组不等式，前两组是x的，后两组是y的
                    A = zeros(4, obj.num_vars);
                    b = zeros(4,1);
                    A(1:2, feet_Index(1,i)) = rectangle_region_con(j).A(1:2);	%对x的约束
                    A(3:4, feet_Index(2,i)) = rectangle_region_con(j).A(3:4);	%对y的约束
                    A(1:4, H_index(j,i)) = M;
                    b = rectangle_region_con(j).b + M;
                    obj.addLinearConstraints_unequal(A, b, 'H_Physic_Sec');
                end                
            end
        end
        
    end
end