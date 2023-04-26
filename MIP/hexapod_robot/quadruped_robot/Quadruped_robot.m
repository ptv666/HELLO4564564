classdef Quadruped_robot<Gurobi_Interface
    properties
        %定义脚的顺序为：1-左前、2-左后、3-右后、4-右前
        body_length = 0.8;  %四足机器人尺寸
        body_width  = 0.5;
%         body_height = 0.4;
        Leg_length = 0.5;	%body中心到落足可行域中心的距离
        theta_offset = [0.5586, pi-0.5586, -pi+0.5586, -0.5586];    %四只脚相对body中心的角度偏移
        
        feet_init = [];     %四条腿的初始状态
        feet_goal = [];     %四条腿的目标状态
        
        weight_g = diag([1000,100,100,20]);	%最后一步距离goal的权重
        weight_r = diag([10,10,10,5]);     	%两步之间的权重--------%与goal相差大一点，这样确保能走到目标点
        weight_trim = 10;                   %trim的代价值，Trim和两步之间的代价值进行对抗
        weight_time = 1.0;                  %落足时刻代价值，使机器人倾向于对角步态,这个代价值没有和其他的代价存在耦合关系，所以只要有就行，不需要很大
        weight_Equilibrium = 10;            %稳定性约束，根据落足点的数量
        weight_swing_leg_num_in_danger = 100;   %为delta_z超过阈值的时刻的摆动腿的总数增加二次代价值
        
        N = -1;         %总步数的估计值
        square_l = 0.5;	%正方形的大小
        delta_z = 0.05; %高度z变化差值0.05
        delta_yaw = 0.05;  %--------yaw角之间的变化差值
        delta_z_threshold = 0.02*0.999;     %高度变化阈值，危险指标，切换步态，乘以0.999是为了解决Gurobi没有严格的小于的问题，也就是解决等于threshold不定的问题
        
        ground_size = 10;   %决策变量上下界限制
        min_steps = -1;     %估计的最小步数，用于确定的Trim的上界
    end
    
    methods
        %输入目标值，通过目标值计算至少需要走多少步，创建落足点序列
        function obj=Quadruped_robot(goal)
            % @goal为4×1，目标位置body的x,y,z,yaw
            goal = reshape(goal,[],1);
            obj = obj@Gurobi_Interface();   %调用父类的构造函数
            
            %初始化四只脚起始位置
            obj.feet_init = zeros(4,4);    %每一列代表一只脚的目标状态
            for i=1:4
                obj.feet_init(1:2,i) = [0;0] + obj.Leg_length*[cos(obj.theta_offset(i));sin(obj.theta_offset(i))];
                obj.feet_init(3:4,i) = 0;
            end
            
            %确定四只脚的目标位置
            obj.feet_goal = zeros(4,4);    %每一列代表一只脚的目标状态
            for i=1:4
                obj.feet_goal(1:2,i) = goal(1:2) + obj.Leg_length*[cos(obj.theta_offset(i));sin(obj.theta_offset(i))];
                obj.feet_goal(3,i) = goal(3);
                obj.feet_goal(4,i) = goal(4);
            end
            
            %确定初始步数
%             obj.N = norm([goal(1),goal(2)])/0.1;
            obj.N = 32;     %50
%             temp_distance = norm(goal(1:2));
%             obj.min_steps = floor(temp_distance/obj.square_l);
            obj.min_steps = 12;
            
            %把步数处理为4的倍数
            i = mod(obj.N,4);
            if i~=0                    
                obj.N = obj.N + 4 - i;
            end
            obj.N = obj.N + 4;  %加入初始的足端状态
            
            %创建决策变量
            obj.addVariable('Feet_State_List', 'C', [4,obj.N], -obj.ground_size,obj.ground_size);
            
            %约束初始状态
            for i=1:4
                 obj.vars.Feet_State_List.lb(:,i) = obj.feet_init(:,i);  %修改初始状态的lb和ub间接控制住初始状态
                 obj.vars.Feet_State_List.ub(:,i) = obj.feet_init(:,i);
            end
            
            %约束yaw
            obj.vars.Feet_State_List.lb(4,5:obj.N) = -pi;
            obj.vars.Feet_State_List.ub(4,5:obj.N) = pi;
        end
        
        %添加最后一步至目标点的代价值:(x_end-x_g)'Q(x_end-x_g)
        function obj=set_Goal_cost(obj)
            %获得最后状态向量在决策变量矩阵中的索引
            last_feet_states = obj.vars.Feet_State_List.Index(:,end-3:end);
            for i=1:4
                %x'Qx
                Q = zeros(obj.num_vars, obj.num_vars);                           	%二次目标矩阵Q需要完全构建，再相加
                Q(last_feet_states(:,i),  last_feet_states(:,i))  = obj.weight_g;   %4×4矩阵
                
                %-2x'Qxg
                c = zeros(obj.num_vars, 1);
                c(last_feet_states(:,i))  = -2*obj.weight_g*obj.feet_goal(:,i);     %4×1矩阵
                
                %xg'Qxg
                alpha = obj.feet_goal(:,i)'*obj.weight_g*obj.feet_goal(:,i);        %优化目标常数
                obj.addCost(Q, c, alpha);   %添加序列末状态和目标状态之间的差值
            end
        end
        
        %添加两步之间的代价值(x2-x1)'Q(x2-x1)
        function obj=set_Run_cost(obj)
            for i=5:obj.N
                Q = zeros(obj.num_vars, obj.num_vars);
                %x1'Qx1
                index_x1 = obj.vars.Feet_State_List.Index(:,i);
                Q(index_x1, index_x1) = obj.weight_r;
                %x2'Qx2
                index_x2 = obj.vars.Feet_State_List.Index(:,i-4);
                Q(index_x2, index_x2) = obj.weight_r;
                %-2x2'Qx1
                Q(index_x1, index_x2) = -obj.weight_r;
                Q(index_x2, index_x1) = -obj.weight_r;
                %添加Gurobi约束
                obj.addCost(Q, [], []);   %for循环内
            end
        end 
        
        %添加单腿工作空间约束
        function obj=set_Leg_Workspace_constrain(obj)
             Feet_State_Index = obj.vars.Feet_State_List.Index;
             
            %--------每一组腿的yaw值相同，yaw1 = yaw2 = yaw3 = yaw4，添加成功
            
            for i=1:4:obj.N     
                Aeq = zeros(3, obj.num_vars);
                beq = zeros(3,1);
                Aeq(1, Feet_State_Index(4,i)) = 1;
                Aeq(1, Feet_State_Index(4,i+1)) = -1;
                Aeq(2, Feet_State_Index(4,i)) = 1;
                Aeq(2, Feet_State_Index(4,i+2)) = -1;
                Aeq(3, Feet_State_Index(4,i)) = 1;
                Aeq(3, Feet_State_Index(4,i+3)) = -1;
                beq(1:3,1) = 0;
                obj.addLinearConstraints_equal(Aeq, beq, 'yaw_equal');
            end
            
            %--------添加sin、cos约束，添加成功
            
            obj.addVariable('sin_yaw', 'C', [1 obj.N], -1, 1) 	%每个落足点添加sin和cos
            obj.addVariable('cos_yaw', 'C', [1 obj.N], -1, 1)
            sin_yaw_index = reshape(obj.vars.sin_yaw.Index,[], 1);	%转成列向量
            cos_yaw_index = reshape(obj.vars.cos_yaw.Index,[], 1);
            for i=1:obj.N
                obj.addSinConstraint(Feet_State_Index(4,i), sin_yaw_index(i,1));
                obj.addCosConstraint(Feet_State_Index(4,i), cos_yaw_index(i,1));
            end
            
            %--------添加质心位置，添加成功
            obj.addVariable('COM', 'C', [2 obj.N], -obj.ground_size,obj.ground_size);
            COM_Index = obj.vars.COM.Index;
            group_index = 0;
            for i=1:4:obj.N
                group_index = group_index + 1;
                Aeq = zeros(8,obj.num_vars);
                beq = zeros(8,1);
                
                %前两个计算1-左前
                Aeq(1,Feet_State_Index(1,i:i+3)) = 1; %质心x坐标
                Aeq(1,COM_Index(1,i)) = -4;
                Aeq(2,Feet_State_Index(2,i:i+3)) = 1; %质心y坐标
                Aeq(2,COM_Index(2,i)) = -4;
                beq(1:2,1) = 0;
                
                %后六个相等
                Aeq(3:2:8,COM_Index(1,i)) = -1;     %质心x
                Aeq(4:2:8,COM_Index(2,i)) = -1;     %质心y
                
                Aeq(3,COM_Index(1,i+1)) = 1;        %2-左后
                Aeq(4,COM_Index(2,i+1)) = 1;
                Aeq(5,COM_Index(1,i+2)) = 1;      	%3-右后
                Aeq(6,COM_Index(2,i+2)) = 1;
                Aeq(7,COM_Index(1,i+3)) = 1;        %4-右前
                Aeq(8,COM_Index(2,i+3)) = 1;
                beq(3:8,1) = 0;
                
                obj.addLinearConstraints_equal(Aeq,beq,'COM_compute');
            end
            
            %--------添加单腿工作空间约束，之前没有添加成功，现在添加成功了
            for i=5:obj.N
                foot = mod(i,4);        %foot计算出来为腿的编号1-4
                if foot==0
                    foot = 4;
                end
                A = zeros(2,obj.num_vars);
                b = zeros(2,1);
                
                %落足点x：abs{x - COM_x - L_leg*cos(yaw)*cos(offset) + L_leg*sin(yaw)*sin(offset)}<square_l
                A(1,Feet_State_Index(1,i)) = 1;
                A(1,COM_Index(1,i-4)) = -1;
                A(1,cos_yaw_index(i-4)) = -obj.Leg_length*cos(obj.theta_offset(foot));
                A(1,sin_yaw_index(i-4)) =  obj.Leg_length*sin(obj.theta_offset(foot));
                %落足点y：abs{y - COM_y - L_leg*sin(yaw)*cos(offset) - L_leg*cos(yaw)*sin(offset)}<square_l
                A(2,Feet_State_Index(2,i)) = 1;
                A(2,COM_Index(2,i-4)) = -1;
                A(2,sin_yaw_index(i-4)) = -obj.Leg_length*cos(obj.theta_offset(foot));
                A(2,cos_yaw_index(i-4)) = -obj.Leg_length*sin(obj.theta_offset(foot));
                
                b(1:2,1) = 0.5*obj.square_l;    %可行域是一个正方形，最大步长也是正方形的一半
                obj.addLinearConstraints_unequal(A,b,'leg_con');
                
                %绝对值的另外一侧
                A = -A;
                b(1:2,1) = 0.5*obj.square_l;
                obj.addLinearConstraints_unequal(A,b,'leg_con');
            end
            
            %--------添加对应脚两步之间的delta_z，加上了
            for i=5:obj.N
                A = zeros(1,obj.num_vars);
                b = zeros(1,1);
                A(1,Feet_State_Index(3,i)) = 1;
                A(1,Feet_State_Index(3,i-4)) = -1;
                b(1,1) = obj.delta_z;
                obj.addLinearConstraints_unequal(A,b,'delta_z');
                %绝对值的另外一侧
                A = -A;
                obj.addLinearConstraints_unequal(A,b,'delta_z');
            end
            
            %--------添加对应脚两步之间的delta_yaw，加上了
            for i=5:obj.N
                A = zeros(1,obj.num_vars);
                b = zeros(1,1);
                A(1,Feet_State_Index(4,i)) = 1;
                A(1,Feet_State_Index(4,i-4)) = -1;
                b(1,1) = obj.delta_yaw;
                obj.addLinearConstraints_unequal(A,b,'delta_yaw');
                %绝对值的另外一侧
                 A = -A;
                 obj.addLinearConstraints_unequal(A,b,'delta_yaw');
            end
            
            
        end
        
        %添加剪枝Trim，去除冗余步数，似乎没有加上去，现在加上去了
        function obj=set_Trim_flag(obj)
            obj.addVariable('Trim', 'B', [1 obj.N], 0, 1);
            trim_Index = reshape(obj.vars.Trim.Index, [], 1);   %获取Trim的索引供后面使用，转成列向量
            
            %--------trim这里需要判断步数是否超过5，如果没有超过5那么就可以直接return了，为了代码工程的完备性（能很好的处理goal和init重合的情况），对于其他的约束是否也需要呢？
            
            if obj.N<=5
                return;
            end
            
            obj.vars.Trim.lb(1,1:4) = 0;        %第一步的Trim固定住
            obj.vars.Trim.ub(1,1:4) = 0;
            obj.vars.Trim.lb(1,end-3:end) = 1;  %最后一步的Trim固定住
            obj.vars.Trim.ub(1,end-3:end) = 1;
            
            %--------trim数组的特点：当前trim=1，则后面所有的trim全部为1，也就是说前面的trim小于后面的trim
            
            for i=5:obj.N-4
                A = zeros(1, obj.num_vars);
                b = zeros(1,1);
                A(trim_Index(i,1)) = 1;
                A(trim_Index(i+1)) = -1;
                b(1,1) = 0;
                obj.addLinearConstraints_unequal(A, b, 'Trim_Seq_relation')
            end
            
            %修改一下Trim的序列关系，约束单腿序列而非所有腿的序列
            
%             for i=5:obj.N-4
%                 A = zeros(1, obj.num_vars);
%                 b = zeros(1,1);
%                 A(trim_Index(i,1)) = 1;
%                 A(trim_Index(i+4)) = -1;
%                 b(1,1) = 0;
%                 obj.addLinearConstraints_unequal(A, b, 'Trim_Seq_relation')
%             end
            
            %--------trim存在if的含义，为了用数学工具表示，一定要把if处理为通用的情形
            %为trim添加实际物理意义：trim=1的时候，该步和最后一步的状态相同
            Feet_State_Index = obj.vars.Feet_State_List.Index;
            M = 9999;   %很大的数，trim=0就几乎没有约束，trim=1就约束紧
            for i=5:obj.N-4    %init和last已经被固定住了,它们对应的落足点坐标也不需要再被约束，尤其是init的落足点坐标 更是已经被确定了
                A = zeros(4,obj.num_vars);
                b = size(4,1);
                foot = mod(i,4);
                if foot==1
                    j = obj.N - 3;	%1-左前
                elseif foot==2
                    j = obj.N - 2; 	%2-左后
                elseif foot==3
                    j = obj.N - 1; 	%2-右后
                else
                    j = obj.N;      %4-右前th 3rw aehhhbn
                end
                
                A(1, Feet_State_Index(1,i)) = 1;	%对x
                A(1, Feet_State_Index(1,j)) = -1;
               
                A(2, Feet_State_Index(2,i)) = 1;	%对y
                A(2, Feet_State_Index(2,j)) = -1;
                
                A(3, Feet_State_Index(3,i)) = 1;	%对z
                A(3, Feet_State_Index(3,j)) = -1;

                A(4, Feet_State_Index(4,i)) = 1; 	%对yaw
                A(4, Feet_State_Index(4,j)) = -1;
                
                A(1:4, trim_Index(i,1)) = 1 * M;	%当trim=1，则abs(delta)<=0，也就是和末状态是相同的
                b(1:4,1) = 1 * M;
                obj.addLinearConstraints_unequal(A, b, 'Trim_attribute_physics');
                A = -A;
                A(1:4,trim_Index(i,1)) = 1 * M;
                obj.addLinearConstraints_unequal(A, b, 'Trim_attribute_physics');
            end
            
            %--------Trim与最小步数存在关系，机器人min_steps是对步数最激进的估计，那么min_steps就反过来对Trim有一个保守的约束（Trim存在一个最大值）
            
            A = zeros(1, obj.num_vars);
            b = zeros(1,1);
            A(1, trim_Index) = 1;
            b(1,1) = obj.N - obj.min_steps; %最大值设置是合理的，最少走min_steps步，总共有N步，所以trim最大值为N-min_step步
            obj.addLinearConstraints_unequal(A, b, 'Trim_sum_con');
            
            %--------添加Trim的代价值，Trim=1的数量越多说明去除冗余步数的效果越好，那么期望该sum(Trim)大一些，目标值是求解最小值，所以对sum(Trim)取负数
            c = zeros(obj.num_vars, 1);
            trim_Index = reshape(obj.vars.Trim.Index, [], 1);   %再获取一次，难道中间没有被改变了吗？？？
            c(trim_Index,1) = -obj.weight_trim;
            alpha = obj.N * obj.weight_trim;
            obj.addCost([], c, alpha);
        end 
        
        %添加区域Region约束
        function obj=set_foot_region(obj,QingXie_rectangle_region_con, init_region_num)
            % @rectangle_region_con是由creat_rectangle_region_con函数生成的结构体数组，结构体数组的行数表示了region的数量
            
            %添加控制落足点region的变量
            num_region = size(QingXie_rectangle_region_con, 1);
            obj.addVariable('H', 'B', [num_region obj.N], 0, 1);
            H_index = obj.vars.H.Index;     %大小为num_region×obj.N
            Feet_State_Index = obj.vars.Feet_State_List.Index;
            
            %--------把起点约束住，由于最后一步不一定能到达终点，所以不对它施加约束
            if length(init_region_num)==1   %初始时四条腿可以不在同一个region中
                repmat(init_region_num,1,4);
            end
            obj.vars.H.lb(:,1) = 0; obj.vars.H.lb(init_region_num(1), 1) = 1;	%对1-左前腿
            obj.vars.H.ub(:,1) = 0; obj.vars.H.ub(init_region_num(1), 1) = 1;
            obj.vars.H.lb(:,2) = 0; obj.vars.H.lb(init_region_num(2), 2) = 1;	%对2-左后腿
            obj.vars.H.ub(:,2) = 0; obj.vars.H.ub(init_region_num(2), 2) = 1;
            obj.vars.H.lb(:,3) = 0; obj.vars.H.lb(init_region_num(3), 3) = 1;	%对3-右后腿
            obj.vars.H.ub(:,3) = 0; obj.vars.H.ub(init_region_num(3), 3) = 1;
            obj.vars.H.lb(:,4) = 0; obj.vars.H.lb(init_region_num(4), 4) = 1;	%对4-右前腿
            obj.vars.H.ub(:,4) = 0; obj.vars.H.ub(init_region_num(4), 4) = 1;
            
            %--------添加H矩阵的物理意义1：每一步只能落在一个region
            for i=1:obj.N
                Aeq = zeros(1, obj.num_vars);
                beq = zeros(1,1);
                Aeq(1, H_index(:,i)) = 1;    %H矩阵每一列和为1
                beq(1,1) = 1;
                obj.addLinearConstraints_equal(Aeq,beq,'H_Physic_Fir');
            end
            
            %--------添加H矩阵的物理意义2：落在region就会收到region区域的几何约束
            M = 9999;
            for i=5:obj.N
                for j=1:num_region
                    %对H矩阵中每一个元素分别进行实际物理约束：如果H(j,i)=1，则第i步落在了第j个region中
                    %对于creat_rectangle_region_con函数，对每一个region它会输出六组不等式，前两组是x的，再两组是y的，后两组是z的
                    A = zeros(6, obj.num_vars);
                    b = zeros(6,1);
                    A(1:2, Feet_State_Index(1,i)) = QingXie_rectangle_region_con(j).A(1:2);     %对x的约束
                    A(3:4, Feet_State_Index(2,i)) = QingXie_rectangle_region_con(j).A(3:4);     %对y的约束
                    A(5:6, Feet_State_Index(3,i)) = QingXie_rectangle_region_con(j).A(5:6);     %对z的约束
                    A(5,Feet_State_Index(2,i))    =-QingXie_rectangle_region_con(j).A(end);  	%z和y之间的tan(theta)关系
                    A(5,Feet_State_Index(2,i))    = QingXie_rectangle_region_con(j).A(end);   	%z和y之间的tan(theta)关系
                    A(1:6,H_index(j,i)) = M;
                    b = QingXie_rectangle_region_con(j).b + M;
                    obj.addLinearConstraints_unequal(A, b, 'H_Physic_Sec');
                end                
            end
        end
         
        %添加步态矩阵T，为每一步添加时刻
        function obj=set_Gait_Matrix(obj)
            %添加步态矩阵T，T_i,j表示第j时刻第i个落足状态被执行了
            obj.addVariable('T', 'B', [obj.N obj.N], 0, 1);
            
            %添加初始状态约束，只是约束住而已，并没有什么实际物理意义，因为初始状态不需要T来表示
            obj.vars.T.lb(1:4,1:4) = eye(4,4);  %初始状态对应的T无意义，任取满足“条件”的即可
            obj.vars.T.ub(1:4,1:4) = eye(4,4);
            obj.vars.T.lb(5:end,1:4) = 0;       %前4列为初始状态的时刻，其余的落足点不需要定义，约束住，不需要Gurobi对其求解
            obj.vars.T.ub(5:end,1:4) = 0;
            
            obj.vars.T.lb(5,5) = 1;             %约束第一个运动步
            obj.vars.T.ub(5,5) = 1;
            
            T_Index = obj.vars.T.Index;         %获取T的索引
            
            %--------添加T的约束1：（基于数学）各行之和为1
            for i=1:obj.N
                Aeq = zeros(1,obj.num_vars);
                beq = zeros(1,1);
                Aeq(1,T_Index(i,:)) = 1;
                beq(1,1) = 1;
                obj.addLinearConstraints_equal(Aeq, beq, 'T_row_sum');
            end
            
            %--------添加T的约束2：（基于物理）当前轮次执行的时间在上一轮次之后
            %Gurobi不能定义严格的小于，所以手动添加一个阈值
            %引入中间变量timing
            obj.addVariable('Timing', 'C', [1,obj.N], 1, obj.N);
            Timing_Index = reshape(obj.vars.Timing.Index,[],1); %获取Trim索引，并转换为列向量
            for i=1:obj.N       %为Timing添加实际物理意义，Timing表示对应落足点在哪一个时刻被执行
                Aeq = zeros(1,obj.num_vars);
                beq = zeros(1,1);
                Aeq(1,T_Index(i,:)) = 1:obj.N;
                Aeq(1,Timing_Index(i,1)) = -1;
                beq(1,1) = 0;
                obj.addLinearConstraints_equal(Aeq,beq,'Timing_compute');
            end
            
            for i=5:obj.N       %当前轮次执行的时间在上一轮次之后，耗时0.2s左右
                foot = mod(i,4);
                A = zeros(4,obj.num_vars);
                b = zeros(4,1);
                temp_Index = zeros(4,1); 	%上一轮次执行时间的位置索引
                
                if foot==1      %1-左前
                    temp_Index = [i-1, i-2, i-3, i-4]; 
                elseif foot==2  %2-左后
                    temp_Index = [i-2, i-3, i-4, i-5];
                elseif foot==3  %3-右后
                    temp_Index = [i-3, i-4, i-5, i-6];
                else         	%4-右前
                    temp_Index = [i-4, i-5, i-6, i-7];
                end
                for j=1:4       %上一周期的运动时刻索引
                    A(j,Timing_Index(temp_Index(j))) = 1;
                end
                A(1:4,Timing_Index(i,1)) = -1;	%当前周期的运动时刻索引
                b(1:4,1) = -0.1;                %设置一个小阈值，解决Gurobi不能添加严格的“小于”约束问题，最终变为：上一轮时刻+0.1<=这一轮的时刻
                obj.addLinearConstraints_unequal(A, b, 'Timing_round'); %这一轮次的脚执行时刻在上一轮次之后
            end
            
            %--------添加T的约束3：（基于物理）避免不可能步态，顺撇与奔跑步态，耗时3.6s左右
            for i=1:4:obj.N
                for j=1:obj.N
                    A = zeros(4,obj.num_vars);
                    b = zeros(4,1);
                    %顺撇步态：1-左前 和 2-左后，3-右后 和 4-右前 不能同时发生
                    A(1,T_Index(i,j)) = 1;
                    A(1,T_Index(i+1,j)) = 1;
                    A(2,T_Index(i+2,j)) = 1;
                    A(2,T_Index(i+3,j)) = 1;
                    b(1:2,1) = 1;
                    %奔跑步态：1-左前 和 4-右前，2-左后 和 3-右后 不能同时发生
                    A(3,T_Index(i,j)) = 1;
                    A(3,T_Index(i+3,j)) = 1;
                    A(4,T_Index(i+1,j)) = 1;
                    A(4,T_Index(i+2,j)) = 1;
                    b(3:4,1) = 1;
                    obj.addLinearConstraints_unequal(A,b,'T3'); %添加约束
                end
            end
            
            %--------添加T的约束4：（基于实验）：同一条腿不能在相邻的两个时刻运动->本质上是质心的计算没有和T矩阵联系在一起，其次是目前的数学问题本身存在这一可行解
            %*******--------这个约束后期必须要删除，不能从实验结果上看出来不行再添加相应的约束，一定是想好了约束添加后求解出结果，不能从结果反推问题
            for i=5:obj.N
                A = zeros(1,obj.num_vars);
                b = zeros(1,1);
                A(1,Timing_Index(i,1)) = -1;	%上一步运动的时刻一定小于这一步的运动时刻，所以不是绝对值约束，就是不等式约束
                A(1,Timing_Index(i-4)) = 1;
                b(1,1) = -2; %t2-t1>=2
                obj.addLinearConstraints_unequal(A,b,'adjoint_Timing_con');
            end
            
            %--------添加代价值，使机器人倾向于对角步态行走
            c = zeros(obj.num_vars,1);
            c(Timing_Index(:),1) = obj.weight_time;
            obj.addCost([],c,[]);
        end
        
        %添加稳定性约束，期望每一次状态转移发生是有更多的落足点支撑住。
        %这样添加的代价一定是0，不能添加一次代价；需要添加二次代价，这样添加二次代价也没有用，徒劳的
        %最终通过添加T矩阵“列和”的二次代价值即可解决该问题。<-通过想法反推方法
        function obj=set_Equilibrium_cost(obj)
            obj.addVariable('T_column_sum', 'C', [1 obj.N], 0, 2);          %添加一个辅助变量来计算平方项
            T_column_sum_Index = reshape(obj.vars.T_column_sum.Index,[],1);	%获取T矩阵列和变量的索引，并排成列向量
            T_Index = obj.vars.T.Index;                                     %获取T矩阵的下标索引，用于计算列和
            for i=1:obj.N
                Aeq = zeros(1,obj.num_vars);
                beq = zeros(1,1);
                Aeq(1,T_Index(:,i)) = 1;
                Aeq(1,T_column_sum_Index(i,1)) = -1;
                beq(1,1) = 0;
                obj.addLinearConstraints_equal(Aeq,beq,'comput_T_column_sum');
            end
            
            for i=1:obj.N   %T矩阵每一列代价
                Q = zeros(obj.num_vars,obj.num_vars);
                alpha = 0;
                Q(T_column_sum_Index(i,1),T_column_sum_Index(i,1)) = obj.weight_Equilibrium;
                alpha = -1*obj.weight_Equilibrium;    
                obj.addCost(Q,[],alpha);
            end
        end
        
        %根据高度落足点高度变化及时的切换步态
        function obj=set_Gait_switch(obj)
            Feet_State_Index = obj.vars.Feet_State_List.Index;      %获取落足点状态索引，供后面使用
            T_Index = obj.vars.T.Index;                             %获取T矩阵的索引，供后面使用
            
            %--------计算两步之间的delta_z
            
            obj.addVariable('delta_z','C',[1 obj.N],-obj.ground_size,obj.ground_size)
            obj.vars.delta_z.lb(1,1:4) = 0;                         %约束住初始状态的delta_z，它之前不存在状态，所以就是0
            obj.vars.delta_z.ub(1,1:4) = 0;
            Delta_z_Index = reshape(obj.vars.delta_z.Index,[],1);   %获取delta_z中间变量的索引值，并排成列向量
            
            for i=5:obj.N   %计算delta_z耗时0.1s
                Aeq = zeros(1,obj.num_vars);
                beq = zeros(1,1);
                Aeq(1,Feet_State_Index(3,i)) = 1;
                Aeq(1,Feet_State_Index(3,i-4)) = -1;
                Aeq(1,Delta_z_Index(i,1)) = -1;
                beq(1,1) = 0;
                obj.addLinearConstraints_equal(Aeq,beq,'compute_delta_z');
            end
            
            %--------根据delta_z的情况确定是否超过高度变化的阈值
            
            obj.addVariable('delta_z_flag', 'B', [2 obj.N], 0, 1);	%第一行代表delta_z是否大于Threshold，第二行代表delta_z是否小于 -Threshold
            obj.vars.delta_z_flag.lb(2,1:4) = 0;                    %初始状态没有越限，约束住
            obj.vars.delta_z_flag.ub(2,1:4) = 0;
            delta_z_Flag_Index = obj.vars.delta_z_flag.Index;       %获取delta_z_flag索引矩阵
            
            M = 9999;   %delta_z确定Flag耗时1.3s
            for i=5:obj.N   %确定delta_z是否大于 Threshold，为flag第一行赋值
                A = zeros(2,obj.num_vars);
                b = zeros(2,1);
                
                %(0.1-delta_z)+FlagM<=M
                A(1,Delta_z_Index(i,1)) = -1;
                A(1,delta_z_Flag_Index(1,i)) = M;
                b(1,1) = M - obj.delta_z_threshold;
                
                %(delta_z-0.1)-FlagM<=0
                A(2,Delta_z_Index(i,1)) = 1;
                A(2,delta_z_Flag_Index(1,i)) = -1 * M;
                b(2,1) = obj.delta_z_threshold;
                
                obj.addLinearConstraints_unequal(A,b,'delta_z_FLAG_Fir');
            end
            for i=5:obj.N   %确定delta_z是否小于-Threshold，为flag第二行赋值
                A = zeros(2,obj.num_vars);
                b = zeros(2,1);
                
                %(delta_z+0.1)+FlagM<=M
                A(1,Delta_z_Index(i,1)) = 1;
                A(1,delta_z_Flag_Index(2,i)) = M;
                b(1,1) = M - obj.delta_z_threshold;
                
                %-(delta_z+0.1)-FlagM<=0
                A(2,Delta_z_Index(i,1)) = -1;
                A(2,delta_z_Flag_Index(2,i)) = -1 * M;
                b(2,1) = obj.delta_z_threshold;
                
                obj.addLinearConstraints_unequal(A,b,'delta_z_FLAG_Sec');
            end
            
            %--------计算Flag_Index的列和
            
            obj.addVariable('Sum_Column_delta_z_Flag', 'B', [1 obj.N], 0, 1);               %不需要给前四列再定义住上下限了，因为可以通过前面定义好的变量值计算出来
            Sum_delta_z_Flag_Index = reshape(obj.vars.Sum_Column_delta_z_Flag.Index,[],1);  %获取索引值，并转换为列向量
            
            for i=1:obj.N   %耗时0.1s
                Aeq = zeros(1,obj.num_vars);
                beq = zeros(1,1);
                Aeq(1,delta_z_Flag_Index(:,i)) = 1;
                Aeq(1,Sum_delta_z_Flag_Index(i,1)) = -1;
                beq(1,1) = 0;
                obj.addLinearConstraints_equal(Aeq,beq,'compute_sum_Flag');
            end
            
            %--------计算T_new，标记危险的步态转移,方法是用每一个step的delta_z_flag去乘以T矩阵的每一行
            
            obj.addVariable('T_plus_delta_z_flag', 'B', [obj.N obj.N], 0, 1);   %Flag直接计算sum(T的列)很难，但是计算一个0,1变量却很容易
            obj.vars.T_plus_delta_z_flag.lb(:,1:4) = 0;                         %对初始的前四个时刻约束为0，只需计算后面的时刻的状态转移
            obj.vars.T_plus_delta_z_flag.ub(:,1:4) = 0;
            obj.vars.T_plus_delta_z_flag.lb(1:4,:) = 0;
            obj.vars.T_plus_delta_z_flag.ub(1:4,:) = 0;
            T_new_Index = obj.vars.T_plus_delta_z_flag.Index;                   %获取T_new的索引矩阵
            
            obj.addVariable('Sum_Column_New_T', 'C', [1 obj.N], 0, 2);          %计算moment_delta_z_flag时就根据T_new的列和，如果列和大于0.5，那么就认为这个moment是危险的
            obj.vars.Sum_Column_New_T.lb(1,1:4) = 0;                            %下面的代码没有对前四个元素的计算，所以在此约束住
            obj.vars.Sum_Column_New_T.ub(1,1:4) = 0;
            Column_Sum_New_T_Index = reshape(obj.vars.Sum_Column_New_T.Index,[],1); %获取T_new列和变量索引值，并转换为列向量
            
            %用每一步的Flag去乘以T矩阵的这一行获得T_new，再计算T_new的列和，当然了这里的“乘”只是数学意义上的，添加到约束中自然需要转换为大M法
            M = 9999;       %耗时60s
            for j=5:obj.N       %每一个时刻
                Aeq = zeros(1,obj.num_vars);
                beq = zeros(1,1);
                for i=5:obj.N   %每一个step
                    A = zeros(3,obj.num_vars);
                    b = zeros(3,1);
                    
                    %T_new(i,j)<=T(i,j)，如果T(i,j)=0，那么T_new(i,j)必为0,；如果T(i,j)=1，那么T_new(i,j)值不定
                    A(1,T_new_Index(i,j)) = 1;
                    A(1,T_Index(i,j)) = -1;
                    b(1,1) = 0;
                    
                    %abs(T_new(i,j)-Flag(i))<=M-T(i,j)M
                    A(2,T_new_Index(i,j)) = 1;
                    A(2,Sum_delta_z_Flag_Index(i,1)) = -1;
                    A(2,T_Index(i,j)) = 1.0 * M;
                    b(2,1) = 1.0 * M;
                    
                    %绝对值的另外一侧
                    A(3,:) = -A(2,:);
                    A(3,T_Index(i,j)) = 1.0 * M;
                    b(3,1) = 1.0 * M;
                    
                    obj.addLinearConstraints_unequal(A, b, 'compute_T_new');
                end
                Aeq(1,T_new_Index(:,j)) = 1;
                Aeq(1,Column_Sum_New_T_Index(j,1)) = -1;
                beq(1,1) = 0;
                obj.addLinearConstraints_equal(Aeq, beq, 'compute_new_T_column_sum');
            end
            
            %--------通过T_new的列和计算每一个时刻的moment_delta_z_flag，
            
            obj.addVariable('moment_delta_z_flag', 'B', [1 obj.N], 0, 1);   %把steps的高度z变化是否超过阈值转变为每一个时刻的高度z变化是否超过阈值
            moment_Flag_Index = reshape(obj.vars.moment_delta_z_flag.Index,[],1);   %获取moment_flag的索引并转换为列向量的形式
            
            %用New_T的列和计算moment_delta_z_flag
            for i=1:obj.N
                A = zeros(2,obj.num_vars);
                b = zeros(2,1);
                %(Sum_star - 0.5) - M*new_Flag<=0
                A(1,Column_Sum_New_T_Index(i,1)) = 1;
                A(1,moment_Flag_Index(i,1)) = -1 * M;
                b(1,1) = 0.5;
                %(0.5 - Sum_star) + M*new_Flag<=M
                A(2,Column_Sum_New_T_Index(i,1)) = -1;
                A(2,moment_Flag_Index(i,1)) = M;
                b(2,1) = M - 0.5;
                
                obj.addLinearConstraints_unequal(A, b, 'compue_time_delta_z_flag');
            end
            
            %**用moment_delta_z_flag计算T的列和*该代码，把矩阵的索引全部改成(x,y)的形式
            obj.addVariable('sum_of_column_T', 'C', [1 obj.N], 0, 2);
            obj.addVariable('wing_leg_num_in_danger', 'C' , [1 obj.N], 0, 2);
            
            T_column_sum_Index = reshape(obj.vars.sum_of_column_T.Index,[],1);      %提取T列和变量的索引值，并转换为列向量
            Swing_Leg_num_Index = reshape(obj.vars.wing_leg_num_in_danger.Index,[],1);  %提取危险状态下摆动腿的数量变量，并转换为列向量
            
            for i=1:obj.N   %先计算T的列和
                Aeq = zeros(1,obj.num_vars);
                beq = zeros(1,1);
                Aeq(1,T_Index(:,i)) = 1;
                Aeq(1,T_column_sum_Index(i,1)) = -1;
                beq(1,1) = 0;
                obj.addLinearConstraints_equal(Aeq,beq,'compute_the_sum_of_T_column_');
            end
            
            for i=1:obj.N   %根据moment_delta_z_flag提取T的列和，这个有问题
                A = zeros(3,obj.num_vars);
                b = zeros(3,1);
                %abs(Sum - Sum_compute)<=(1 - Flag)M
                A(1,T_column_sum_Index(i,1)) = 1;
                A(1,Swing_Leg_num_Index(i,1)) = -1;
                A(1,moment_Flag_Index(i,1)) = M;
                b(1,1) = M;
                
                %绝对值的另外一侧
                A(2,T_column_sum_Index(i,1)) = -1;
                A(2,Swing_Leg_num_Index(i,1)) = 1;
                A(2,moment_Flag_Index(i,1)) = M;
                b(2,1) = M;
                
                %Sum_compute <= Flag*M
                A(3,Swing_Leg_num_Index(i,1)) = 1;
                A(3,moment_Flag_Index(i,1)) = -1 * M;
                b(3,1) = 0;
                
                obj.addLinearConstraints_unequal(A, b, 'compute_swing_leg_in_danger');
            end
            

            %**核心：如果delta_z超过了阈值，就对该状态转移的 “时刻” 摆动腿总数施加惩罚**
            %为wing_leg_num_in_danger添加二次代价值，并且在加入代价的时候，把摆动腿的数量减去1，否则in_danger时即便只有一条腿最终也会有很高的代价值
            %(wing_leg_num_in_danger - 1)'Q(wing_leg_num_in_danger - 1)
            
            for i=1:obj.N
                Q = zeros(obj.num_vars,obj.num_vars);
                c = zeros(obj.num_vars,1);
                alpha = 0.0;
                Q(Swing_Leg_num_Index(i),Swing_Leg_num_Index(i)) = obj.weight_swing_leg_num_in_danger;
                c(Swing_Leg_num_Index(i)) = -2*obj.weight_swing_leg_num_in_danger;
                alpha = obj.weight_swing_leg_num_in_danger;
                
                obj.addCost(Q,c,alpha);
            end      
        end
        
    end
end