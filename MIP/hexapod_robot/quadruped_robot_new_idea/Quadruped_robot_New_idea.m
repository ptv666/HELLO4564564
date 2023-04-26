classdef Quadruped_robot_New_idea<Gurobi_Interface
    properties
        %定义脚的顺序为：1-左前、2-左后、3-右后、4-右前
        body_length = 0.8;  %四足机器人尺寸
        body_width  = 0.5;
%         body_height = 0.4;
        Leg_length = 0.5;	%body中心到落足可行域中心的距离
        theta_offset = [0.5586, pi-0.5586, -pi+0.5586, -0.5586];    %四只脚相对body中心的角度偏移
        
        COM_init =[0;0;0;0];
        COM_goal = [];
        feet_init = [];     %四条腿的初始状态，每一列对应一个腿
        feet_goal = [];     %四条腿的目标状态，每一列对应一个腿
        
        weight_g = diag([100,100,100,100]);	%最后一步距离goal的权重
        weight_r = diag([10,10,10,10]);   	%两步之间的权重--------%与goal相差大一点，这样确保能走到目标点
        Leg_Rate_Com = 0.8;
        weight_trim = 2;                    %trim的代价值，Trim和两步之间的代价值进行对抗
        weight_time = 20;                  %落足时刻代价值，使机器人倾向于对角步态,这个代价值没有和其他的代价存在耦合关系，所以只要有就行，不需要很大
        weight_Equilibrium = 10;            %稳定性约束，根据落足点的数量
        weight_swing_leg_num_in_danger = 10;   %为delta_z超过阈值的时刻的摆动腿的总数增加二次代价值
        
        N = -1;             %总步数的估计值
        square_l = 0.5;     %正方形的大小
        delta_z = 0.12;     %高度z变化差值0.05
        delta_yaw = 0.05;       %--------yaw角之间的变化差值
        delta_z_threshold = 0.02;     %高度变化阈值，危险指标，切换步态，乘以0.999是为了解决Gurobi没有严格的小于的问题，也就是解决等于threshold不定的问题
        
        ground_size = 10;   %决策变量上下界限制
        min_steps = -1;     %估计的最小步数，用于确定的Trim的上界
    end
    
    methods
        %输入目标值，通过目标值计算至少需要走多少步，创建落足点序列
        function obj=Quadruped_robot_New_idea(goal)
            % @goal为4×1，目标位置body的x,y,z,yaw
            goal = reshape(goal,[],1);
            obj = obj@Gurobi_Interface();   %调用父类的构造函数
            obj.COM_goal = goal;
            
            %初始化四只脚起始位置
            obj.feet_init = zeros(4,4);    %每一列代表一只脚的目标状态
            for i=1:4
                obj.feet_init(1:2,i) = obj.COM_init(1:2,1) + obj.Leg_length*[cos(obj.theta_offset(i));sin(obj.theta_offset(i))];
                obj.feet_init(3:4,i) = obj.COM_init(3:4,1);
            end
            
            %确定四只脚的目标位置
            obj.feet_goal = zeros(4,4);    %每一列代表一只脚的目标状态
            for i=1:4
                obj.feet_goal(1:2,i) = goal(1:2) + obj.Leg_length*[cos(obj.theta_offset(i));sin(obj.theta_offset(i))];
                obj.feet_goal(3,i) = goal(3);
%                 obj.feet_goal(4,i) = goal(4);
                obj.feet_goal(4,i) = 0;
            end
            
            %确定初始步数------先人为给定来验证想法
            obj.N = 40; %20
            obj.min_steps = 4;
            
            %加入初始COM状态
            obj.N = obj.N + 1;
            
            %创建决策变量
            obj.addVariable('COM_state', 'C', [4,obj.N], -obj.ground_size,obj.ground_size); %每一列是真正的COM的(x,y,z,yaw)状态，这个是真正的代价变量
            obj.addVariable('T', 'B', [4,obj.N], 0,1);                  %步态矩阵T
            obj.addVariable('Leg_state', 'C', [16,obj.N], -2*obj.ground_size,2*obj.ground_size);   %只是COM_consist的附属产品，如果把COM_consist元素表示为cell，那么Leg_state完全可以删除
            
            %给变量初始值
            obj.vars.T.lb(1:4,1) = 1; %T矩阵第一列不需要Gurobi求解
            obj.vars.T.ub(1:4,1) = 1; 
            obj.vars.COM_state.lb(1:4,1) = [0;0;0;0];   %COM_state第一列不需要Gurobi求解
            obj.vars.COM_state.ub(1:4,1) = [0;0;0;0];
            for j=1:4  %Leg_state第一列不需要Gurobi求解
                temp_index = (4*j-3):(4*j);
                obj.vars.Leg_state.lb(temp_index,1) = obj.feet_init(:,j);
                obj.vars.Leg_state.ub(temp_index,1) = obj.feet_init(:,j);
            end
            
            %先不让机体发生转动，一直朝向前方，因为四只脚都有yaw角，无法计算COM的yaw角
            temp_index = 4:4:16;
            obj.vars.Leg_state.lb(temp_index,2:end) = 0.0;   
            obj.vars.Leg_state.ub(temp_index,2:end) = 0.0;
            
        end
        
        %设置变量之间的数学、物理关系
        function obj=set_vars_relation(obj)
            COM_state_Index = obj.vars.COM_state.Index;
            T_Index = obj.vars.T.Index;
            Leg_state_Index = obj.vars.Leg_state.Index;
            
            %添加COM_state和Leg_state的从属关系，Leg_state只是要表达COM_state的具体元素罢了
            for j=2:obj.N
                Aeq = zeros(4,obj.num_vars);
                beq = zeros(4,1);
               
                Aeq(1,Leg_state_Index(1:4:16,j)) = 1;     %x
                Aeq(1,COM_state_Index(1,j)) = -4;
                Aeq(2,Leg_state_Index(2:4:16,j)) = 1;     %y
                Aeq(2,COM_state_Index(2,j)) = -4;
                Aeq(3,Leg_state_Index(3:4:16,j)) = 1;     %z
                Aeq(3,COM_state_Index(3,j)) = -4;
                Aeq(4,Leg_state_Index(4:4:16,j)) = 1;     %yaw
                Aeq(4,COM_state_Index(4,j)) = -4;
                beq(1:4,1) = 0;
                
                obj.addLinearConstraints_equal(Aeq,beq,'COM_state_AND_Leg_state');
            end
            
            %添加T矩阵的约束，只约束危险步态
            for j=2:obj.N
                i = 1;
                A = zeros(4,obj.num_vars);
                b = zeros(4,1);
                
                %顺撇步态：1-左前 和 2-左后
                A(1,T_Index(i,j)) = 1;
                A(1,T_Index(i+1,j)) = 1;
                %顺撇步态：3-右后 和 4-右前 不能同时发生
                A(2,T_Index(i+2,j)) = 1;
                A(2,T_Index(i+3,j)) = 1;
                b(1:2,1) = 1;

                %奔跑步态：1-左前 和 4-右前
                A(3,T_Index(i,j)) = 1;
                A(3,T_Index(i+3,j)) = 1;
                %奔跑步态：2-左后 和 3-右后 不能同时发生
                A(4,T_Index(i+1,j)) = 1;
                A(4,T_Index(i+2,j)) = 1;
                b(3:4,1) = 1;
                
                obj.addLinearConstraints_unequal(A,b,'T_self_con');	%添加约束
            end
            
            %两步不要在相邻的时刻运动，加速求解
            for j=2:obj.N-1
                for i=1:4
                    A = zeros(1,obj.num_vars);
                    b = zeros(1,1);
                    A(1,T_Index(i,j:j+1)) = 1;
                    b(1,1) = 1;
                    obj.addLinearConstraints_unequal(A,b,'adjoint_timing');
                end
            end
            
            %T矩阵和COM计算之间的关系，它们之间是一列一列产生对应关系
            %如果T(i,j) = 0,则COM_state第i条腿的的当前状态和上一时刻是相同的
            %如果T(i,j) = 1,则COM_state第i条腿添加正方形约束
            %T矩阵一列的4个元素与Leg_state_Index一列的16个约束产生映射
            M = 999999;
            cos_yaw = 1;
            sin_yaw = 0;
            for j=2:obj.N   %j代表时刻
                for i=1:4   %i代表对四条腿中的哪一个
                    
                    temp_index = (4*i-3):(4*i-1); %H(i,j)对应的是Leg_state_Index(4i-3:4i,j)，但是不考虑yaw角，那么只有Leg_state_Index(4i-3:4i-1,j)，也就H(i,j)映射的xyz
                    
                    %对第i条腿的x
                    A_x = zeros(4,obj.num_vars);  
                    b_x = zeros(4,1);
                    %等式约束：abs(x(j)-x(j-1))<=M*T(i,j)
                    A_x(1,Leg_state_Index(temp_index(1),j)) = 1;
                    A_x(1,Leg_state_Index(temp_index(1),j-1)) = -1;
                    A_x(2,:) = -A_x(1,:);
                    A_x(1:2,T_Index(i,j)) = -1*M;
                    b_x(1:2,1) = 0;
                    %x正方形约束：abs{x - COM_x - L_leg*cos(yaw)*cos(offset) + L_leg*sin(yaw)*sin(offset)}<square_l + (1-T(i,j))*M
                    A_x(3,Leg_state_Index(temp_index(1),j)) = 1;
                    A_x(3,COM_state_Index(1,j-1)) = -1;
                    A_x(4,:) = -A_x(3,:);
                    A_x(3:4,T_Index(i,j)) = M;
                    b_x(3,1) =  cos_yaw * obj.Leg_length * cos(obj.theta_offset(i)) - sin_yaw * obj.Leg_length * sin(obj.theta_offset(i)) + 0.5*obj.square_l + M;
                    b_x(4,1) = -cos_yaw * obj.Leg_length * cos(obj.theta_offset(i)) + sin_yaw * obj.Leg_length * sin(obj.theta_offset(i)) + 0.5*obj.square_l + M;
                    
                    %对第i条腿的y
                    A_y = zeros(4,obj.num_vars);  
                    b_y = zeros(4,1);
                    %等式约束：abs(y(j)-y(j-1))<=M*T(i,j)
                    A_y(1,Leg_state_Index(temp_index(2),j)) = 1;
                    A_y(1,Leg_state_Index(temp_index(2),j-1)) = -1;
                    A_y(2,:) = -A_y(1,:);
                    A_y(1:2,T_Index(i,j)) = -1*M;
                    b_y(1:2,1) = 0;
                    
                    %y正方形约束：abs{y - COM_y - L_leg*sin(yaw)*cos(offset) - L_leg*cos(yaw)*sin(offset)}<square_l + (1-T(i,j))*M
                    A_y(3,Leg_state_Index(temp_index(2),j)) = 1;
                    A_y(3,COM_state_Index(2,j-1)) = -1;
                    A_y(4,:) = -A_y(3,:);
                    A_y(3:4,T_Index(i,j)) = M;
                    b_y(3,1) =  sin_yaw * obj.Leg_length * cos(obj.theta_offset(i)) + cos_yaw * obj.Leg_length * sin(obj.theta_offset(i)) + 0.5*obj.square_l + M;
                    b_y(4,1) = -sin_yaw * obj.Leg_length * cos(obj.theta_offset(i)) - cos_yaw * obj.Leg_length * sin(obj.theta_offset(i)) + 0.5*obj.square_l + M;
                    
                    %z的约束
                    A_z = zeros(4,obj.num_vars);
                    b_z = zeros(4,1);
                    %等式约束：abs(z(j)-z(j-1))<=M*T(i,j)
                    A_z(1,Leg_state_Index(temp_index(3),j)) = 1;
                    A_z(1,Leg_state_Index(temp_index(3),j-1)) = -1;
                    A_z(2,:) = -A_z(1,:);
                    A_z(1:2,T_Index(i,j)) = -1*M;
                    b_z(1:2,1) = 0;
                    %不等式约束：abs(z(j)-z(j-1))<=delta_z + (1-T(i,j))*M
                    A_z(3,Leg_state_Index(temp_index(3),j)) = 1;
                    A_z(3,Leg_state_Index(temp_index(3),j-1)) = -1;
                    A_z(4,:) = -A_z(3,:);
                    A_z(3:4,T_Index(i,j)) = M;
                    b_z(3:4,1) = obj.delta_z + M;
                    
                    temp_name = ['(',num2str(i),',',num2str(j),')'];
                    x_name = ['T',temp_name,'_AND_Leg_state__x'];
                    y_name = ['T',temp_name,'_AND_Leg_state__y'];
                    z_name = ['T',temp_name,'_AND_Leg_state__z'];
                    
                    obj.addLinearConstraints_unequal(A_x,b_x,x_name);
                    obj.addLinearConstraints_unequal(A_y,b_y,y_name);
                    obj.addLinearConstraints_unequal(A_z,b_z,z_name);
                end
            end
        end
        
        %添加COM的目标代价值:(x_end-x_g)'Q(x_end-x_g)
        function obj=set_COM_Goal_cost(obj)
            %获得最后一个COM的索引
            COM_last_state_Index = obj.vars.COM_state.Index(:,end);
            %x'Qx
            Q = zeros(obj.num_vars, obj.num_vars); 
            Q(COM_last_state_Index,COM_last_state_Index) = obj.weight_g;
            %-2x'Qxg
            c = zeros(obj.num_vars, 1);
            c(COM_last_state_Index,1) = -2 * obj.weight_g * obj.COM_goal;
            %xg'Qxg,优化目标常数
            alpha = obj.COM_goal' * obj.weight_g * obj.COM_goal;	
            obj.addCost(Q, c, alpha);
        end
        
        %添加COM的运行代价值(x2-x1)'Q(x2-x1)
        function obj=set_COM_Run_cost(obj)
            COM_state_Index = obj.vars.COM_state.Index;
            for j=2:obj.N
                Q = zeros(obj.num_vars, obj.num_vars);
                %x2'Qx2
                Q(COM_state_Index(:,j), COM_state_Index(:,j)) = obj.weight_r;
                %x1'Qx1
                Q(COM_state_Index(:,j-1), COM_state_Index(:,j-1)) = obj.weight_r;
                %-2x2'Qx1
                Q(COM_state_Index(:,j-1), COM_state_Index(:,j)) = -obj.weight_r;
                Q(COM_state_Index(:,j), COM_state_Index(:,j-1)) = -obj.weight_r;
                %添加Gurobi约束
                obj.addCost(Q, [], []);   %for循环内
            end
        end 
        
        %添加Step目标代价值(x_end-x_g)'Q(x_end-x_g)
        function obj=set_Step_Goal_cost(obj)
            for i=1:4
                temp_index = (4*i-3):(4*i);
                %获得最后一个Step的索引
                Leg_last_state_Index = obj.vars.Leg_state.Index(temp_index,end);
                %x'Qx
                Q = zeros(obj.num_vars, obj.num_vars);
                Q(Leg_last_state_Index,Leg_last_state_Index) = obj.Leg_Rate_Com * obj.weight_g;
                %-2x'Qxg
                c = zeros(obj.num_vars, 1);
                c(Leg_last_state_Index,1) = -2 * (obj.Leg_Rate_Com * obj.weight_g) * obj.feet_goal(:,i);
                %xg'Qxg,优化目标常数
                alpha = obj.feet_goal(:,i)' * (obj.Leg_Rate_Com * obj.weight_g) * obj.feet_goal(:,i);	
                obj.addCost(Q, c, alpha);
            end            
        end
        
        %添加Step运行代价值(x2-x1)'Q(x2-x1)
        function obj=set_Step_Run_cost(obj)
            Leg_state_Index = obj.vars.Leg_state.Index;
            for j=2:obj.N
                for i=1:4
                    temp_index = (4*i-3):(4*i);
                    Q = zeros(obj.num_vars, obj.num_vars);
                    %x2'Qx2
                    Q(Leg_state_Index(temp_index,j), Leg_state_Index(temp_index,j)) = obj.Leg_Rate_Com * obj.weight_r;
                    %x1'Qx1
                    Q(Leg_state_Index(temp_index,j-1), Leg_state_Index(temp_index,j-1)) = obj.Leg_Rate_Com * obj.weight_r;
                    %-2x2'Qx1
                    Q(Leg_state_Index(temp_index,j-1), Leg_state_Index(temp_index,j)) = -obj.Leg_Rate_Com * obj.weight_r;
                    Q(Leg_state_Index(temp_index,j), Leg_state_Index(temp_index,j-1)) = -obj.Leg_Rate_Com * obj.weight_r;
                    %添加Gurobi约束
                    obj.addCost(Q, [], []);   %for循环内 
                end
            end
        end
        
         %添加对COM的剪枝Trim，去除冗余步数
        function obj=set_Trim_flag(obj)
            obj.addVariable('Trim', 'B', [1 obj.N], 0, 1);
            trim_Index = obj.vars.Trim.Index;
            
            %--------trim这里需要判断步数是否超过5，如果没有超过5那么就可以直接return了，为了代码工程的完备性（能很好的处理goal和init重合的情况），对于其他的约束是否也需要呢？
            
            if obj.N<=2
                return;
            end
            
            obj.vars.Trim.lb(1,1) = 0;      %第一步的Trim固定住
            obj.vars.Trim.ub(1,1) = 0;
%             obj.vars.Trim.lb(1,end) = 1;	%最后一步的Trim固定住
%             obj.vars.Trim.ub(1,end) = 1;
            
            %--------trim数组的特点：当前trim=1，则后面所有的trim全部为1，也就是说前面的trim小于后面的trim
            
            for j=2:obj.N-1
                A = zeros(1,obj.num_vars);
                b = zeros(1,1);
                A(trim_Index(1,j)) = 1;
                A(trim_Index(1,j+1)) = -1;
                b(1,1) = 0;
                obj.addLinearConstraints_unequal(A, b, 'Trim_Seq_relation')
            end
            
            %--------为trim添加实际物理意义：trim=1的时候，该步和最后一步的状态相同
            T_Index = obj.vars.T.Index;
            M = 999999;
            for j=2:obj.N
                A = zeros(4,obj.num_vars);
                b = zeros(4,1);
                for i=1:4
                    A(i,T_Index(i,j)) = 1;
                end
                A(1:4,trim_Index(1,j)) = 1*M;
                b(1:4,1) = M;
                obj.addLinearConstraints_unequal(A,b,'Trim_attribute_physics');
            end
%             COM_state_Index = obj.vars.COM_state.Index;
%             M = 999999;
%             for j=2:obj.N-1     %对COM的约束，只约束COM会造成腿乱动的情况,试一下！
%                 A = zeros(4,obj.num_vars);
%                 b = size(4,1);
%                 
%                 %abs(x-x(end))<=(1-Trim)*M
%                 A(1, COM_state_Index(1,j)) = 1;     %对x
%                 A(1, COM_state_Index(1,end)) = -1;
%                
%                 A(2, COM_state_Index(2,j)) = 1;     %对y
%                 A(2, COM_state_Index(2,end)) = -1;
%                 
%                 A(3, COM_state_Index(3,j)) = 1;     %对z
%                 A(3, COM_state_Index(3,end)) = -1;
% 
%                 A(4, COM_state_Index(4,j)) = 1;     %对yaw
%                 A(4, COM_state_Index(4,end)) = -1;
%                 
%                 A(1:4, trim_Index(1,j)) = 1 * M;
%                 b(1:4,1) = 1 * M;
%                 obj.addLinearConstraints_unequal(A, b, 'Trim_attribute_physics');
%                 A = -A;
%                 A(1:4,trim_Index(1,i)) = 1 * M;
%                 obj.addLinearConstraints_unequal(A, b, 'Trim_attribute_physics');
%             end
            
            %--------Trim与最小步数存在关系，机器人min_steps是对步数最激进的估计，那么min_steps就反过来对Trim有一个保守的约束（Trim存在一个最大值）
            
            A = zeros(1,obj.num_vars);
            b = zeros(1,1);
            A(1,trim_Index) = 1;
            b(1,1) = obj.N - obj.min_steps;
            obj.addLinearConstraints_unequal(A, b, 'Trim_sum_con');
            
            %--------添加Trim的代价值，Trim=1的数量越多说明去除冗余步数的效果越好，那么期望该sum(Trim)大一些，目标值是求解最小值，所以对sum(Trim)取负数
            c = zeros(obj.num_vars, 1);
            c(trim_Index,1) = -obj.weight_trim;
            alpha = (obj.N-1) * obj.weight_trim;
            obj.addCost([], c, alpha);
        end 
        
        %添加落足点的区域Region约束
        function obj=set_foot_region(obj,rectangle_region_con_has_z)
            % @rectangle_region_con是由creat_rectangle_region_con_has_z函数生成的结构体数组，结构体数组的行数表示了region的数量
            
            %添加控制落足点region的变量
            num_region = size(rectangle_region_con_has_z, 1);
            obj.addVariable('H', 'B', [4*num_region obj.N], 0, 1); %行对应四组！
            H_index = obj.vars.H.Index;                 %H矩阵Index：4*num_region×N（4代表4个腿）
            Leg_state_Index = obj.vars.Leg_state.Index; %落足点Index：16×N
            
            %--------对初始状态的落足区域任意取一个值即可
            obj.vars.H.lb(:,1) = 0;
            obj.vars.H.ub(:,1) = 0;
            
            %--------添加H矩阵的物理意义1：每一步只能落在一个region
            for j=2:obj.N
                for i=1:4
                    temp_index = ((i-1)*num_region + 1):(i * num_region);
                    Aeq = zeros(1, obj.num_vars);
                    beq = zeros(1,1);
                    Aeq(1, H_index(temp_index,j)) = 1;  %每一只脚只能落在唯一的一个region中
                    beq(1,1) = 1;
                    obj.addLinearConstraints_equal(Aeq,beq,'H_Physic_Fir');
                end
            end
            
            %--------添加H矩阵的物理意义2：落在region就会受到region区域的几何约束
            M = 999999;
            for j=2:obj.N
                for i=1:4
                    leg_temp_index = (4*i-3):(4*i);
                    for k=1:num_region
                        %对于creat_rectangle_region_con函数，对每一个region它会输出六组不等式，前两组是x的，再两组是y的，后两组是z的
                        region_temp_index = (i-1)*num_region + k;
                        A = zeros(6, obj.num_vars);
                        b = zeros(6,1);
                        A(1:2, Leg_state_Index(leg_temp_index(1),j)) = rectangle_region_con_has_z(k).A(1:2);	%对x的约束
                        A(3:4, Leg_state_Index(leg_temp_index(2),j)) = rectangle_region_con_has_z(k).A(3:4);	%对y的约束
                        A(5:6, Leg_state_Index(leg_temp_index(3),j)) = rectangle_region_con_has_z(k).A(5:6);  	%对z的约束
                        A(1:6,H_index(region_temp_index,j)) = M;
                        b = rectangle_region_con_has_z(k).b + M;
                        obj.addLinearConstraints_unequal(A, b, 'H_Physic_Sec');
                    end
                end                
            end
        end     

        %添加稳定性约束，期望每一次状态转移发生是有更多的落足点支撑住，这个约束只是实验使用的，真正使用的是下面的高度变化代价值切换步态
        function obj=set_Equilibrium_cost(obj)
            obj.addVariable('T_column_sum', 'C', [1 obj.N], 0, 2);          %添加一个辅助变量来计算平方项
            obj.vars.T_column_sum.lb(1,1) = 0;
            obj.vars.T_column_sum.ub(1,1) = 0;
            
            T_column_sum_Index = obj.vars.T_column_sum.Index;
            T_Index = obj.vars.T.Index;
            for j=2:obj.N
                Aeq = zeros(1,obj.num_vars);
                beq = zeros(1,1);
                Aeq(1,T_Index(:,j)) = 1;
                Aeq(1,T_column_sum_Index(1,j)) = -1;
                beq(1,1) = 0;
                obj.addLinearConstraints_equal(Aeq,beq,'comput_T_column_sum');
            end
            
            for i=2:obj.N   %T矩阵每一列代价temp_init，不统计初始状态的
                Q = zeros(obj.num_vars,obj.num_vars);
                alpha = 0;
                Q(T_column_sum_Index(1,i),T_column_sum_Index(1,i)) = obj.weight_Equilibrium;
                alpha = -1*obj.weight_Equilibrium;    
                obj.addCost(Q,[],alpha);
            end
        end
        
        %根据高度落足点高度变化及时的切换步态
        function obj=set_Gait_switch(obj)
            COM_state_Index = obj.vars.COM_state.Index;
            T_Index = obj.vars.T.Index;
            Leg_state_Index = obj.vars.Leg_state.Index;
            
            %--------计算两步之间的delta_z
            
            obj.addVariable('delta_z','C',[1 obj.N],-obj.ground_size,obj.ground_size)
            obj.vars.delta_z.lb(1,1) = 0;
            obj.vars.delta_z.ub(1,1) = 0;
            Delta_z_Index = obj.vars.delta_z.Index;
            
            for j=2:obj.N
                Aeq = zeros(1,obj.num_vars);
                beq = zeros(1,1);
                Aeq(1,COM_state_Index(3,j)) = 1;
                Aeq(1,COM_state_Index(3,j-1)) = -1;
                Aeq(1,Delta_z_Index(1,j)) = -1;
                beq(1,1) = 0;
                obj.addLinearConstraints_equal(Aeq,beq,'compute_delta_z');
            end
            
            %--------根据delta_z的情况确定是否超过高度变化的阈值
            
            obj.addVariable('delta_z_flag', 'B', [2 obj.N], 0, 1);	%第一行代表delta_z是否大于Threshold，第二行代表delta_z是否小于 -Threshold
            obj.vars.delta_z_flag.lb(1:2,1) = 0;
            obj.vars.delta_z_flag.ub(1:2,1) = 0;
            delta_z_Flag_Index = obj.vars.delta_z_flag.Index;     	%获取delta_z_flag索引矩阵
            
            M = 999999;
            for j=2:obj.N   %确定delta_z是否大于 Threshold，为flag第一行赋值
                A = zeros(2,obj.num_vars);
                b = zeros(2,1);
                
                %(0.1-delta_z)+FlagM<=M
                A(1,Delta_z_Index(1,j)) = -1;
                A(1,delta_z_Flag_Index(1,j)) = M;
                b(1,1) = M - obj.delta_z_threshold;
                
                %(delta_z-0.1)-FlagM<=0
                A(2,Delta_z_Index(1,j)) = 1;
                A(2,delta_z_Flag_Index(1,j)) = -1 * M;
                b(2,1) = obj.delta_z_threshold;
                
                obj.addLinearConstraints_unequal(A,b,'delta_z_FLAG_Fir');
            end
            for j=2:obj.N   %确定delta_z是否小于-Threshold，为flag第二行赋值
                A = zeros(2,obj.num_vars);
                b = zeros(2,1);
                
                %(delta_z+0.1)+FlagM<=M
                A(1,Delta_z_Index(1,j)) = 1;
                A(1,delta_z_Flag_Index(2,j)) = M;
                b(1,1) = M - obj.delta_z_threshold;
                
                %-(delta_z+0.1)-FlagM<=0
                A(2,Delta_z_Index(1,j)) = -1;
                A(2,delta_z_Flag_Index(2,j)) = -1 * M;
                b(2,1) = obj.delta_z_threshold;
                
                obj.addLinearConstraints_unequal(A,b,'delta_z_FLAG_Sec');
            end
            
            %--------计算Flag_Index的列和
            
            obj.addVariable('Sum_Column_delta_z_Flag', 'B', [1 obj.N], 0, 1);
            Sum_delta_z_Flag_Index = obj.vars.Sum_Column_delta_z_Flag.Index;
            
            for j=1:obj.N
                Aeq = zeros(1,obj.num_vars);
                beq = zeros(1,1);
                Aeq(1,delta_z_Flag_Index(:,j)) = 1;
                Aeq(1,Sum_delta_z_Flag_Index(1,j)) = -1;
                beq(1,1) = 0;
                obj.addLinearConstraints_equal(Aeq,beq,'compute_sum_Flag');
            end
            
            %--------计算T_new，标记危险的步态转移,方法是用每一个step的delta_z_flag去乘以T矩阵的每一行
            
            obj.addVariable('T_multiplication_delta_z_flag', 'B', [4 obj.N], 0, 1);	%Flag直接计算sum(T的列)很难，但是计算一个0,1变量却很容易
            obj.vars.T_multiplication_delta_z_flag.lb(1:4,1) = 0;
            obj.vars.T_multiplication_delta_z_flag.ub(1:4,1) = 0;
            T_new_Index = obj.vars.T_multiplication_delta_z_flag.Index;
            
            obj.addVariable('Sum_Column_New_T', 'C', [1 obj.N], 0, 2);
            obj.vars.Sum_Column_New_T.lb(1,1) = 0;
            obj.vars.Sum_Column_New_T.ub(1,1) = 0;
            Column_Sum_New_T_Index = obj.vars.Sum_Column_New_T.Index;
            
            %用每一步的Flag去乘以T矩阵的这一行获得T_new，再计算T_new的列和，当然了这里的“乘”只是数学意义上的，添加到约束中自然需要转换为大M法
            M = 999999;
            for j=2:obj.N
                Aeq = zeros(1,obj.num_vars);
                beq = zeros(1,1);
                
                for i=1:4
                    A = zeros(3,obj.num_vars);
                    b = zeros(3,1);
                    
                    %T_new(i,j)<=T(i,j)，如果T(i,j)=0，那么T_new(i,j)必为0,；如果T(i,j)=1，那么T_new(i,j)值不定
                    A(1,T_new_Index(i,j)) = 1;
                    A(1,T_Index(i,j)) = -1;
                    b(1,1) = 0;
                    
                    %abs(T_new(i,j)-Flag(i))<=M-T(i,j)M
                    A(2,T_new_Index(i,j)) = 1;
                    A(2,Sum_delta_z_Flag_Index(1,j)) = -1;
                    A(2,T_Index(i,j)) = 1.0 * M;
                    b(2,1) = 1.0 * M;
                    
                    %绝对值的另外一侧
                    A(3,:) = -A(2,:);
                    A(3,T_Index(i,j)) = 1.0 * M;
                    b(3,1) = 1.0 * M;
                    
                    obj.addLinearConstraints_unequal(A, b, 'compute_T_new');
                end
                Aeq(1,T_new_Index(:,j)) = 1;
                Aeq(1,Column_Sum_New_T_Index(1,j)) = -1;
                beq(1,1) = 0;
                obj.addLinearConstraints_equal(Aeq, beq, 'compute_new_T_column_sum');
            end
            
            for j = 2:obj.N     %为T_new的列和作为代价值
                Q = zeros(obj.num_vars,obj.num_vars);
                Q(Column_Sum_New_T_Index(1,j),Column_Sum_New_T_Index(1,j)) = obj.weight_swing_leg_num_in_danger;
                obj.addCost(Q,[],[]);
            end
            
        end
        
    end
end