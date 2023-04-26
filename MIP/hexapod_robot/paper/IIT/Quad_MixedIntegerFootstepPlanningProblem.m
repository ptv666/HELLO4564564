classdef Quad_MixedIntegerFootstepPlanningProblem < Quad_MixedIntegerConvexProgram
    %它从混合整数规划类中继承而来，所以涵盖了所有关于gurobi中对数学表达式的定义操作
% Structure for Quadruped footstep planning through Mixed Integer Convex Optimization
% Currently implemented with numerical variables for constraints in reachability and 
% robot shape.只使用了数值变量并没有使用符号变量
%
% Developed by Bernardo Aceituno-C (Mechatronics Group, USB C Laboratory).
%
% As presented in the 2017 paper "A Generalized Mixed-Integer Convex Model
% For Multilegged Footstep Planning on Uneven Terrain" by Bernardo Aceituno-Cabezas,
% J. Cappelletto, J. Grieco, and G. Fernarndez (USB)
%
% Based on MixedIntegerFootstepPlanningProblem by Robin Deits (Robot Locomotion Group, MIT CSAIL).

  properties
    Quadruped;      %四足机器人对象
    n_legs = 4;     %四条腿
    nsteps;         %行走的步数，提前确定好N_f
    seed_plan;      %plan结构对象，包含了接触状态序列
    plan_gait = false;  %是否添加了步态约束
    L_leg = 0.0991; %腿的长度？
    offset = [0.8339, -0.8339, pi - 0.8339, -pi + 0.8339];  %左前、右前、左后、右后
    d_lim = 0.01;   %单腿运动空间的方形的边长
    l_bnd = 0.1;    %单腿运动步长
    gamma_z = 0.005;    %落足点高度变化的限制0.005m
    dz = 0.05;      %运动一步z变化最大值
    weights;        %优化目标的权值
    max_distance = 500; %一个很大的值
    pose_indices = [1,2,3,6];   %pos中有六个项x,y,z,roll,pitch,yaw，只取其中的x,y,z位置项和yaw角
  end

  methods
      %初始化操作，定义决策变量，也就是落足点序列需要被优化，但是没搞懂weight->是一个结构体，里面包含了不同类别的权值信息
      function obj = Quad_MixedIntegerFootstepPlanningProblem(Quadruped, seed_plan)
          %主要定义出N_f个决策变量用于处理落足点
        % Constructs the optimization problem and declares the variables for each footstep
        % @param Quadruped a Quadruped.
        % @param seed_plan a blank footstep plan, provinding the structure of the
        %                  desired plan. Probably generated with
        %                  FootstepPlan.blank_plan()，既然是空的plan，那为什么可以设置上下限？

        %检查参数的属性
        typecheck(Quadruped, 'Quadruped');
        typecheck(seed_plan, 'Quad_FootstepPlan');
        
        %MATLAB类继承的用法，右侧的obj是父类对象，调用父类的构造函数
        obj = obj@Quad_MixedIntegerConvexProgram();%构造混合整数求解对象，这是一个与gurobi好用的数学接口
        obj.Quadruped = Quadruped;  %构造四足机器人
        obj.seed_plan = seed_plan;  %传入plan对象，定义着规划的接口
        obj.nsteps = length(obj.seed_plan.footsteps);   %确定步数，给定的
        obj.n_legs = 4; %腿的数量

        assert(mod(obj.nsteps,obj.n_legs) == 0) %足端序列长度一定是腿个数的整数倍，也就是4的倍数
        
        %***************-----定义优化三个目标各自的权重
        obj.weights = obj.Quadruped.getFootstepOptimizationWeights();
        
        %落足点序列初值，序列列数是N_f，行数是3表示(x,y,z)
        seed_steps = [seed_plan.footsteps.pos]; 
        min_yaw = -pi;  %设置机体的yaw角限制
        max_yaw = pi;
        
        %初始位姿一定要先给定！否则模型无法求解
        %设置上下限，形式为[pos;yaw]的4×n矩阵，4代表每一步，n代表总步数；设置的上下限都十分的“广”
        %lb和ub的大小为4×N_f，4中三个是x,y,z,一个是yaw
        lb = [repmat(seed_steps(1:3,1) - obj.max_distance, 1, obj.nsteps);
              min_yaw + zeros(1, obj.nsteps)];
        ub = [repmat(seed_steps(1:3,1) + obj.max_distance, 1, obj.nsteps);
              max_yaw + zeros(1, obj.nsteps)];
        
        %把第一步限制死，机体的默认位置和初始支撑点是不需要优化的
        %限制这四个决策变量上下限是相同的，那么它就是一个常数了，这里也要注意了，我们需要手动确定初始状态并赋值！！
        lb(:,1) = seed_steps(obj.pose_indices, 1);
        lb(:,2) = seed_steps(obj.pose_indices, 2);
        lb(:,3) = seed_steps(obj.pose_indices, 3);
        lb(:,4) = seed_steps(obj.pose_indices, 4);

        ub(:,1) = seed_steps(obj.pose_indices, 1);
        ub(:,2) = seed_steps(obj.pose_indices, 2);
        ub(:,3) = seed_steps(obj.pose_indices, 3);
        ub(:,4) = seed_steps(obj.pose_indices, 4);
        
        %一共有4×n个变量，分别代表了x,y,z,yaw,共N_f个，由于定义好了“gurobi”的接口，所以这里就会自动的创建对应的A矩阵，Aeq矩阵，Q矩阵，Qc矩阵，c向量，q向量
        obj = obj.addVariable('footsteps', 'C', [obj.n_legs, obj.nsteps], lb, ub);
        %footsteps变量是4×N_f大小的
      end
      
      % sin和cos的近似，又引入了新的决策变量
      function obj = addSinCosLinearEquality(obj)
        % Adds a linear approximation of the sin and cos functions for eacg footstep yaw 
        % as presented in the 2014 paper "Footstep Planning on Uneven Terrain with
        % Mixed-Integer Convex Optimization" by Robin Deits and Russ Tedrake (MIT)

        yaw0 = obj.seed_plan.footsteps(1).pos(6);
        min_yaw = yaw0 - pi/8 * floor((obj.nsteps-4)/4);
        max_yaw = yaw0 + pi/8 * ceil((obj.nsteps-4)/4);

        if max_yaw > pi
          max_yaw = pi;
        end

        if min_yaw < pi
          min_yaw = -pi;
        end

        cos_boundaries = reshape(bsxfun(@plus, [min_yaw:pi:max_yaw; min_yaw:pi:max_yaw], [-(pi/2-1); (pi/2-1)]), 1, []);
        sin_boundaries = reshape(bsxfun(@plus, [min_yaw:pi:max_yaw; min_yaw:pi:max_yaw], [-1; 1]), 1, []);

        obj = obj.addVariableIfNotPresent('cos_yaw', 'C', [1, obj.nsteps], -1, 1);
        obj = obj.addVariableIfNotPresent('sin_yaw', 'C', [1, obj.nsteps], -1, 1);
        obj = obj.addVariable('cos_sector', 'B', [length(cos_boundaries)-1, obj.nsteps], 0, 1);
        obj = obj.addVariable('sin_sector', 'B', [length(sin_boundaries)-1, obj.nsteps], 0, 1);
        obj = obj.addInitialSinCosConstraints();

        obj.vars.footsteps.lb(4,7:end) = min_yaw;
        obj.vars.footsteps.ub(4,7:end) = max_yaw;

        for j = 5:4:obj.nsteps-4
          %fix all legs to always point in the same direction
          n1 = j;
          n2 = j+1;
          n3 = j+2;
          n4 = j+3;

          Aeq = sparse(9, obj.nv);
          beq = zeros(9, 1);

          Aeq(1,obj.vars.footsteps.i(4,n1)) = 1;
          Aeq(1,obj.vars.footsteps.i(4,n2)) = -1;

          Aeq(2,obj.vars.footsteps.i(4,n1)) = 1;
          Aeq(2,obj.vars.footsteps.i(4,n3)) = -1;

          Aeq(3,obj.vars.footsteps.i(4,n1)) = 1;
          Aeq(3,obj.vars.footsteps.i(4,n4)) = -1;

          Aeq(4,obj.vars.cos_yaw.i(n1)) = 1;
          Aeq(4,obj.vars.cos_yaw.i(n2)) = -1;

          Aeq(5,obj.vars.cos_yaw.i(n1)) = 1;
          Aeq(5,obj.vars.cos_yaw.i(n3)) = -1;

          Aeq(6,obj.vars.cos_yaw.i(n1)) = 1;
          Aeq(6,obj.vars.cos_yaw.i(n4)) = -1;

          Aeq(7,obj.vars.sin_yaw.i(n1)) = 1;
          Aeq(7,obj.vars.sin_yaw.i(n2)) = -1;

          Aeq(8,obj.vars.sin_yaw.i(n1)) = 1;
          Aeq(8,obj.vars.sin_yaw.i(n3)) = -1;

          Aeq(9,obj.vars.sin_yaw.i(n1)) = 1;
          Aeq(9,obj.vars.sin_yaw.i(n4)) = -1;

          obj = obj.addLinearConstraints([], [], Aeq, beq); 
        end

        obj = obj.addVariable('unit_circle_slack', 'C', [1,1], norm([pi/4;pi/4]), norm([pi/4;pi/4]));
        Aeq_s = sparse(obj.nsteps, obj.nv);
        Aeq_c = zeros(obj.nsteps, obj.nv);
        beq = ones(size(Aeq_s, 1), 1);
        
        for j = 1:obj.nsteps
          Aeq_c(j, obj.vars.cos_sector.i(:,j)) = 1;
          Aeq_s(j, obj.vars.sin_sector.i(:,j)) = 1;
        end

        obj = obj.addLinearConstraints([], [], [Aeq_s; Aeq_c], [beq; beq]);
        obj = obj.addPolyConesByIndex([repmat(obj.vars.unit_circle_slack.i, 1, obj.nsteps-4); obj.vars.cos_yaw.i(5:end); obj.vars.sin_yaw.i(5:end)], 8);

        M = 2*pi;
        Ai = sparse((obj.nsteps-4) * (length(cos_boundaries)-1 + length(sin_boundaries)-1) * 4, obj.nv);
        bi = zeros(size(Ai, 1), 1);
        offset = 0;
        expected_offset = size(Ai, 1);

        for s = 1:length(cos_boundaries)-1
          th0 = cos_boundaries(s);
          th1 = cos_boundaries(s+1);

          th = (th0 + th1)/2;
          cos_slope = -sin(th);
          cos_intercept = cos(th) - (cos_slope * th);

          for j = 5:obj.nsteps
            % -yaw(j) <= -th0 + M(1-cos_sector(s,j))
            Ai(offset+1, obj.vars.footsteps.i(4,j)) = -1;
            Ai(offset+1, obj.vars.cos_sector.i(s,j)) = M;
            bi(offset+1) = -th0 + M;
            % yaw(j) <= th1 + M(1-cos_sector(s,j))
            Ai(offset+2, obj.vars.footsteps.i(4,j)) = 1;
            Ai(offset+2, obj.vars.cos_sector.i(s,j)) = M;
            bi(offset+2) = th1 + M;
            offset = offset + 2;

            % cos_yaw(j) <= cos_slope * yaw(j) + cos_intercept + M(1-cos_sector(s,j))
            Ai(offset+1, obj.vars.cos_yaw.i(j)) = 1;
            Ai(offset+1, obj.vars.footsteps.i(4,j)) = -cos_slope;
            Ai(offset+1, obj.vars.cos_sector.i(s,j)) = M;
            bi(offset+1) = cos_intercept + M;
            % cos_yaw(j) >= cos_slope * yaw(j) + cos_intercept - M(1-cos_sector(s,j))
            Ai(offset+2, obj.vars.cos_yaw.i(j)) = -1;
            Ai(offset+2, obj.vars.footsteps.i(4,j)) = cos_slope;
            Ai(offset+2, obj.vars.cos_sector.i(s,j)) = M;
            bi(offset+2) = -cos_intercept + M;
            offset = offset + 2;
          end

        end

        for s = 1:length(sin_boundaries)-1
          th0 = sin_boundaries(s);
          th1 = sin_boundaries(s+1);

          th = (th0 + th1)/2;
          sin_slope = cos(th);
          sin_intercept = sin(th) - (sin_slope * th);

          for j = 5:obj.nsteps
            % -yaw(j) <= -th0 + M(1-sin_sector(s,j))
            Ai(offset+1, obj.vars.footsteps.i(4,j)) = -1;
            Ai(offset+1, obj.vars.sin_sector.i(s,j)) = M;
            bi(offset+1) = -th0 + M;
            % yaw(j) <= th1 + M(1-sin_sector(s,j))
            Ai(offset+2, obj.vars.footsteps.i(4,j)) = 1;
            Ai(offset+2, obj.vars.sin_sector.i(s,j)) = M;
            bi(offset+2) = th1 + M;
            offset = offset + 2;

            % sin_yaw(j) <= sin_slope * yaw(j) + sin_intercept + M(1-sin_sector(s,j))
            Ai(offset+1, obj.vars.sin_yaw.i(j)) = 1;
            Ai(offset+1, obj.vars.footsteps.i(4,j)) = -sin_slope;
            Ai(offset+1, obj.vars.sin_sector.i(s,j)) = M;
            bi(offset+1) = sin_intercept + M;
            % sin_yaw(j) >= sin_slope * yaw(j) + sin_intercept - M(1-sin_sector(s,j))
            Ai(offset+2, obj.vars.sin_yaw.i(j)) = -1;
            Ai(offset+2, obj.vars.footsteps.i(4,j)) = sin_slope;
            Ai(offset+2, obj.vars.sin_sector.i(s,j)) = M;
            bi(offset+2) = -sin_intercept + M;
            offset = offset + 2;
          end
        end
        assert(offset == expected_offset);
        obj = obj.addLinearConstraints(Ai, bi, [], []);

        % Consistency between sin and cos sectors
        Ai = sparse((obj.nsteps-4) * obj.vars.sin_sector.size(1) * 2, obj.nv);
        bi = zeros(size(Ai, 1), 1);
        offset = 0;
        expected_offset = size(Ai, 1);
        nsectors = obj.vars.sin_sector.size(1);
        for k = 1:nsectors
          for j = 5:obj.nsteps
            Ai(offset+1, obj.vars.cos_sector.i(k,j)) = 1;
            Ai(offset+1, obj.vars.sin_sector.i(max(1,k-1):min(k+1,nsectors),j)) = -1;
            Ai(offset+2, obj.vars.sin_sector.i(k,j)) = 1;
            Ai(offset+2, obj.vars.cos_sector.i(max(1,k-1):min(k+1,nsectors),j)) = -1;
            offset = offset + 2;
          end
        end
        assert(offset == expected_offset);
        obj = obj.addLinearConstraints(Ai, bi, [], []);

        % Transitions between sectors
        Ai = sparse((obj.nsteps-4) * (nsectors-1) * 2, obj.nv);
        bi = zeros(size(Ai, 1), 1);
        offset = 0;
        expected_offset = size(Ai, 1);
        for j = 5:obj.nsteps
          if mod(j,2)
            for k = 1:nsectors - 1
              Ai(offset+1, obj.vars.cos_sector.i(k,j-3)) = 1;
              Ai(offset+1, obj.vars.cos_sector.i(k:k+1,j)) = -1;
              Ai(offset+2, obj.vars.sin_sector.i(k,j-3)) = 1;
              Ai(offset+2, obj.vars.sin_sector.i(k:k+1,j)) = -1;
              offset = offset + 2;
            end
          else
            for k = 2:nsectors
              Ai(offset+1, obj.vars.cos_sector.i(k,j-1)) = 1;
              Ai(offset+1, obj.vars.cos_sector.i(k-1:k,j)) = -1;
              Ai(offset+2, obj.vars.sin_sector.i(k,j-1)) = 1;
              Ai(offset+2, obj.vars.sin_sector.i(k-1:k,j)) = -1;
              offset = offset + 2;
            end
          end
        end
        assert(offset == expected_offset);
        obj = obj.addLinearConstraints(Ai, bi, [], []);
      end
      % 初始sin和cos约束
      function obj = addInitialSinCosConstraints(obj)
        % Constrain the values of sin and cos for the current poses of the feet
        obj.vars.cos_yaw.lb(1) = cos(obj.seed_plan.footsteps(1).pos(6));
        obj.vars.cos_yaw.ub(1) = cos(obj.seed_plan.footsteps(1).pos(6));
        obj.vars.cos_yaw.lb(2) = cos(obj.seed_plan.footsteps(2).pos(6));
        obj.vars.cos_yaw.ub(2) = cos(obj.seed_plan.footsteps(2).pos(6));
        obj.vars.sin_yaw.lb(3) = sin(obj.seed_plan.footsteps(3).pos(6));
        obj.vars.sin_yaw.ub(3) = sin(obj.seed_plan.footsteps(3).pos(6));
        obj.vars.sin_yaw.lb(4) = sin(obj.seed_plan.footsteps(4).pos(6));
        obj.vars.sin_yaw.ub(4) = sin(obj.seed_plan.footsteps(4).pos(6));
      end
      
      %定义了一个虚拟的“落足中心”，落足点需要被约束在该中心的正方形内部，“虚拟落足中心”的计算存在质疑，并不是根据当前支撑状态决定的质心位置来计算的，而是一个周期一个周期计算的
      %单腿的运动可达的位置约束
      function obj = addReachabilityConstraints(obj)
        % Adds a set of linear constraints on the reachability of each leg and the 
        % geometry of the robot.
        %
        % as presented in the 2017 paper "A Generalized Mixed-Integer Convex Model
        % For Multilegged Footstep Planning on Uneven Terrain" by Bernardo Aceituno-Cabezas,
        % J. Cappelletto, J. Grieco, and G. Fernarndez (USB)
        %把单腿的工作空间限制在方形之内，方形有质心位置定义
        % Adds a linear a constraint to matain each footstep in a squared region centered on its reference 
        % feet position with respect to the Center of Contacts (CoC) with a side d_lim.

        % defines the variable r_nom for the reference position of each step
        % measured from the center of contacts of each configuration
        obj = obj.addVariable('r_nom', 'C', [2, obj.nsteps], -10*ones(2,obj.nsteps), 10*ones(2,obj.nsteps));
        
        k = 1;
        
        %这里认为机体的中心是一轮中的四个落足点的中心
        for j = 1:obj.nsteps
          if mod(k,4) == 0
            k = 0;
            n1 = j-3;
            n2 = j-2;
            n3 = j-1;
            n4 = j;
            correct = obj.offset(4);
          elseif mod(k,3) == 0
            n1 = j-2;
            n2 = j-1;
            n3 = j;
            n4 = j+1;
            correct = obj.offset(3);
          elseif mod(k,2) == 0
            n1 = j-1;
            n2 = j;
            n3 = j+1;
            n4 = j+2;
            correct = obj.offset(2);
          else
            n1 = j;
            n2 = j+1;
            n3 = j+2;
            n4 = j+3;
            correct = obj.offset(1);
          end

          %setting the values
          Aeq = sparse(2,obj.nv);
          beq = zeros(2,1);
            
          Aeq(1,obj.vars.footsteps.i(1,n1)) = 1;
          Aeq(1,obj.vars.footsteps.i(1,n2)) = 1;
          Aeq(1,obj.vars.footsteps.i(1,n3)) = 1;
          Aeq(1,obj.vars.footsteps.i(1,n4)) = 1;
          Aeq(1,obj.vars.r_nom.i(1,j)) = -4;
          %yaw角又一个周期四步中的第一步决定
          Aeq(1,obj.vars.cos_yaw.i(n1)) = 4*obj.L_leg*cos(correct);     %论文中的公式cos(yaw + correct)
          Aeq(1,obj.vars.sin_yaw.i(n1)) = -4*obj.L_leg*sin(correct);

          Aeq(2,obj.vars.footsteps.i(2,n1)) = 1;
          Aeq(2,obj.vars.footsteps.i(2,n2)) = 1;
          Aeq(2,obj.vars.footsteps.i(2,n3)) = 1;
          Aeq(2,obj.vars.footsteps.i(2,n4)) = 1;
          Aeq(2,obj.vars.r_nom.i(2,j)) = -4;
          Aeq(2,obj.vars.cos_yaw.i(n1)) = 4*obj.L_leg*sin(correct);     %论文中的公式sin(yaw + correct)
          Aeq(2,obj.vars.sin_yaw.i(n1)) = 4*obj.L_leg*cos(correct);

          k = k + 1;

          obj = obj.addLinearConstraints([],[],Aeq,beq);
        end
        k = 1;

        %we now have that f(i) must be in a square of side d_lim from the step
        
        %正方形约束
        %polytopic relaxation of the geometric contraint
        for j = 5:obj.nsteps
          Ai = sparse(4,obj.nv);
          bi = zeros(4,1);

          Ai(1,obj.vars.footsteps.i(1,j)) = 1;  %x
          Ai(1,obj.vars.r_nom.i(1,j)) = -1;
          bi(1,1) = obj.d_lim;

          Ai(2,obj.vars.footsteps.i(1,j)) = -1;
          Ai(2,obj.vars.r_nom.i(1,j)) = 1;
          bi(2,1) = obj.d_lim;

          Ai(3,obj.vars.footsteps.i(2,j)) = 1;  %y
          Ai(3,obj.vars.r_nom.i(2,j)) = -1;
          bi(3,1) = obj.d_lim;

          Ai(4,obj.vars.footsteps.i(2,j)) = -1;
          Ai(4,obj.vars.r_nom.i(2,j)) = 1;
          bi(4,1) = obj.d_lim;

          obj = obj.addLinearConstraints(Ai,bi,[],[]);
        end

        % Constraints the reachability of each leg for the robot configuration and its successor
        % Adds linear constraints such that every footstep lies on a square of side l_bnd 
        % centered on the previous nominal leg position.
        
        %delta_x与delta_y不能超过0.1m
        for j = 5:obj.nsteps
          %defines the linear constraint on reachability on x
          Ai = sparse(2,obj.nv);
          bi = zeros(2,1);

          Ai(1,obj.vars.footsteps.i(1,j)) = 1;
          Ai(1,obj.vars.r_nom.i(1,j-4)) = -1;
          bi(1,1) = obj.l_bnd;

          Ai(2,obj.vars.footsteps.i(1,j)) = -1;
          Ai(2,obj.vars.r_nom.i(1,j-4)) = 1;
          bi(2,1) = obj.l_bnd;

          %adds the constraints
          obj = obj.addLinearConstraints(Ai,bi,[],[]);
          
          %defines the linear constraint on reachability on y
          Ai = sparse(2,obj.nv);
          bi = zeros(2,1);

          Ai(1,obj.vars.footsteps.i(2,j)) = 1;
          Ai(1,obj.vars.r_nom.i(2,j-4)) = -1;
          bi(1,1) = obj.l_bnd;
          
          Ai(2,obj.vars.footsteps.i(2,j)) = -1;
          Ai(2,obj.vars.r_nom.i(2,j-4)) = 1;
          bi(2,1) = obj.l_bnd;

          %adds the constraints
          obj = obj.addLinearConstraints(Ai,bi,[],[]);
          
          %defines the linear constraint on reachability of the yaw
          Ai = sparse(2,obj.nv);
          bi = zeros(2,1);

          Ai(1,obj.vars.footsteps.i(4,j)) = 1;
          Ai(1,obj.vars.footsteps.i(4,j-4)) = -1;
          bi(1,1) = pi/8;

          Ai(2,obj.vars.footsteps.i(4,j)) = -1;
          Ai(2,obj.vars.footsteps.i(4,j-4)) = 1;
          bi(2,1) = pi/8;

          %adds the constraints
          obj = obj.addLinearConstraints(Ai,bi,[],[]);
        end
        
        % Limits the difference in z between footsteps
        Ai = sparse((obj.nsteps-4)*2, obj.nv);
        bi = zeros(size(Ai, 1), 1);
        offset = 0;
        expected_offset = size(Ai, 1);
        
        %约束delta_z小于0.05m
        for j = 5:obj.nsteps
          % obj.symbolic_constraints = [obj.symbolic_constraints,...
          %   -obj.seed_plan.params.nom_downward_step <= x(3,j) - x(3,j-1) <= obj.seed_plan.params.nom_upward_step];
          Ai(offset+1, obj.vars.footsteps.i(3,j)) = -1;
          Ai(offset+1, obj.vars.footsteps.i(3,j-4)) = 1;
          bi(offset+1) = obj.dz;
          Ai(offset+2, obj.vars.footsteps.i(3,j)) = 1;
          Ai(offset+2, obj.vars.footsteps.i(3,j-4)) = -1;
          bi(offset+2) = obj.dz;
          offset = offset + 2;
        end

        assert(offset == expected_offset);
        obj = obj.addLinearConstraints(Ai, bi, [], []);
      end
      
      %目标代价值
      function obj = addQuadraticGoalObjective(obj, goal_pose, step_indices)
        % For each index j in step_indices, add a caudratic cost of the form:
        % weights(j) * (footstep(j) - xgoal(6 - N +j))' * Qfoal * (footstep(j) - xgoal(6 - N +j)
        
        k = 1;
        w_goal = diag(obj.weights.goal(obj.pose_indices));  %提取x,y,z,yaw的代价值[1000;1000;0;100]
        
        %step_indices代表了在N_f中的序号
        for j = step_indices %对传入的step_indices循环添加代价值
          if mod(k,4) == 0  %第四条腿
            xg = goal_pose.Foot4(obj.pose_indices); %获取腿的目标值
          elseif mod(k,3) == 0  %第三条腿
            xg = goal_pose.Foot3(obj.pose_indices);
          elseif mod(k,2) == 0  %第二条腿
            xg = goal_pose.Foot2(obj.pose_indices);
          else  %第一条腿
            xg = goal_pose.Foot1(obj.pose_indices);
          end
          %即使是只使用一步的代价，也需要定义出所有变量大小的矩阵！这就是计算机优化！
          %这些矩阵到model中会进行相加的
          Qi = sparse([], [], [], obj.nv, obj.nv, 4);   %nv：number of variable,4代表预留4个非零位置；矩阵大小是nv×nv，只有四个值不为零
          ci = zeros(obj.nv, 1);    %定义大小：nv×1，列向量，表示优化目标的线性列向量
          
          %这个就是误差的二次型形式，因为当前状态蕴含在了决策变量中，目标状态给定为x_g，所以确实是这样定义的:把(x-x_g)'Qgoal(x-x_g)展开就是以下的形式
          Qi(obj.vars.footsteps.i(:,j), obj.vars.footsteps.i(:,j)) = w_goal;    %对step_indices（大概率是最后一步）序号处的腿的状态给予设置目标代价值
          ci(obj.vars.footsteps.i(:,j)) = -2 * w_goal * xg;                     %二次型展开的形式
          objcon_i = xg' * w_goal * xg;
          
          %添加代价值
          obj = obj.addCost(Qi, ci, objcon_i);
          
          k = k + 1;
        end
      end    
        
      %每两步之间添加代价值-这里单腿的运动的代价值用到了addReachabilityConstraints中定义的变量！！
      %两步之间的代价
      function obj = addQuadraticRelativeObjective(obj)
        % Add a quadratic cost on the transfer distance between configurations
        % in the form:
        % (pi - pi-n)'Qr(pi - pi-n)
        % Where Qr is chosen to matain the footsteps displacement on its nominal position
        %定义式子为：weights * (pi - pi-n)' * Qr * (pi - pi-n)
        
        % defines the center of contacts as the geometric center of each n_leg configuration
    
        %先约束每一步的机体质心位置
        %存在“超前”计算的问题，第一条腿动完，后面三条腿没有动，但是却使用了它们的位置，也是合理的，因为要选择下一个落足点
        %变量p代表着base质心位置，默认求解方式是计算四条腿的中心来求解，当足端接触状态序列确定或，变量p也就自动确定了
        obj = obj.addVariable('p', 'C', [3, obj.nsteps], -10*ones(3,obj.nsteps), 10*ones(3 ,obj.nsteps));
        k = 1;
        %给每一步都添加质心位置约束，质心位置变量并没有用到
        for j = 1:obj.nsteps
          if mod(k,4) == 0
            k = 0;
            n1 = j-3;
            n2 = j-2;
            n3 = j-1;
            n4 = j;
          elseif mod(k,3) == 0
            n1 = j-2;
            n2 = j-1;
            n3 = j;
            n4 = j+1;
          elseif mod(k,2) == 0
            n1 = j-1;
            n2 = j;
            n3 = j+1;
            n4 = j+2;
          else
            n1 = j;
            n2 = j+1;
            n3 = j+2;
            n4 = j+3;
          end
          %setting the values
          Aeq = sparse(3,obj.nv);
          beq = zeros(3,1);
            
          %质心x坐标
          Aeq(1,obj.vars.footsteps.i(1,n1)) = 1;
          Aeq(1,obj.vars.footsteps.i(1,n2)) = 1;
          Aeq(1,obj.vars.footsteps.i(1,n3)) = 1;
          Aeq(1,obj.vars.footsteps.i(1,n4)) = 1;
          Aeq(1,obj.vars.p.i(1,j)) = -4;
          
          %质心y坐标
          Aeq(2,obj.vars.footsteps.i(2,n1)) = 1;
          Aeq(2,obj.vars.footsteps.i(2,n2)) = 1;
          Aeq(2,obj.vars.footsteps.i(2,n3)) = 1;
          Aeq(2,obj.vars.footsteps.i(2,n4)) = 1;
          Aeq(2,obj.vars.p.i(2,j)) = -4;

          %质心z坐标
          Aeq(3,obj.vars.footsteps.i(3,n1)) = 1;
          Aeq(3,obj.vars.footsteps.i(3,n2)) = 1;
          Aeq(3,obj.vars.footsteps.i(3,n3)) = 1;
          Aeq(3,obj.vars.footsteps.i(3,n4)) = 1;
          Aeq(3,obj.vars.p.i(3,j)) = -4;
            
          k = k + 1;

          obj = obj.addLinearConstraints([],[],Aeq,beq);
        end
        
        % adds a quadratic cost in the relative distance between configurations
        % defined as the transfer distance or T_d(j) = p(j) - p(j-n_legs)
        % and a quadratic cost in the distance between each step and the
        % center of each safe region（添加落足点到安全区域中心的约束）
        %代价的形式为：
        
        %设置优化目标的代价值->每两步之间的代价值
        Qr = 1; %两步之间的代价值，[1;1;1;0;0;0]
        
        for j = 5:obj.nsteps
          %----------------------------
          %首先是关于质心移动的代价值
          %质心x方向的二次代价值(x-x_g)'*Q_r*(x-x_g)=x^2*Q_r + x_g^2*Q_r - 2*x*x_g*Q_r
          Qi = sparse(obj.nv, obj.nv);
          Qi(obj.vars.p.i(1,j), obj.vars.p.i(1,j)) = Qr;
          Qi(obj.vars.p.i(1,j), obj.vars.p.i(1,j-4)) = -Qr;
          Qi(obj.vars.p.i(1,j-4), obj.vars.p.i(1,j)) = -Qr;
          Qi(obj.vars.p.i(1,j-4), obj.vars.p.i(1,j-4)) = Qr;
          obj = obj.addCost(Qi, [], []);
          
          %质心y方向的二次代价值
          Qi = sparse(obj.nv, obj.nv);
          Qi(obj.vars.p.i(2,j), obj.vars.p.i(2,j)) = Qr;
          Qi(obj.vars.p.i(2,j), obj.vars.p.i(2,j-4)) = -Qr;
          Qi(obj.vars.p.i(2,j-4), obj.vars.p.i(2,j)) = -Qr;
          Qi(obj.vars.p.i(2,j-4), obj.vars.p.i(2,j-4)) = Qr;
          obj = obj.addCost(Qi, [], []);
          %-------------------------------
          %每两步之间yaw角的二次代价值
          Qi = sparse(obj.nv, obj.nv);
          Qi(obj.vars.footsteps.i(4,j), obj.vars.footsteps.i(4,j)) = Qr;
          Qi(obj.vars.footsteps.i(4,j), obj.vars.footsteps.i(4,j-4)) = -Qr;
          Qi(obj.vars.footsteps.i(4,j-4), obj.vars.footsteps.i(4,j)) = -Qr;
          Qi(obj.vars.footsteps.i(4,j-4), obj.vars.footsteps.i(4,j-4)) = Qr;
          obj = obj.addCost(Qi, [], []);
          %-------------------------------
          %这两个关于x、y的代价形式为何不同？->必有一个出错了，y出错了，这个代价的原意应该是当前这一步与当前这一步的落足中心的关系，当前落足中心已经定义好了
          %落足点到安全区域中心的x距离代价
          Qi = sparse(obj.nv, obj.nv);
          Qi(obj.vars.footsteps.i(1,j), obj.vars.footsteps.i(1,j)) = Qr;
          Qi(obj.vars.footsteps.i(1,j), obj.vars.r_nom.i(1,j)) = -Qr;
          Qi(obj.vars.r_nom.i(1,j), obj.vars.footsteps.i(1,j)) = -Qr;
          Qi(obj.vars.r_nom.i(1,j), obj.vars.r_nom.i(1,j)) = Qr;
          obj = obj.addCost(Qi, [], []);
          
          %落足点到安全区域中心的y距离代价
          Qi = sparse(obj.nv, obj.nv);
          Qi(obj.vars.footsteps.i(2,j), obj.vars.footsteps.i(2,j)) = Qr;
          Qi(obj.vars.footsteps.i(2,j), obj.vars.r_nom.i(2,j-4)) = -Qr;
          Qi(obj.vars.r_nom.i(2,j-4), obj.vars.footsteps.i(2,j)) = -Qr;
          Qi(obj.vars.r_nom.i(2,j-4), obj.vars.r_nom.i(2,j-4)) = Qr;
          obj = obj.addCost(Qi, [], []);
        end
      end
      
      %添加了trim，约束有：后一个t不小于前一个t，所以当一条腿停下来时，整个机器人都要停下来；其次对步数的一个估计
      %又赋予trim实际物理意义，即当trim=1时，腿不能动，和上一个状态要保持一致，这些约束有:(nsteps-4)*8个
      function obj = addTrimToFinalPoses(obj)
        % Add a binary variable for each footstep which, if true, forces that footstep to the
        % final pose in the footstep plan. This allows us to trim it out of the footstep plan later.
        % A linear objective placed on those trim variables lets us tune the number of footsteps
        % in the plan.
        
        %添加nsteps个二进制变量
        obj = obj.addVariable('trim', 'B', [1, obj.nsteps], 0, 1);
        %对trim的奖励权重
        w_trim = 1;
        min_num_steps = max([obj.seed_plan.params.min_num_steps + 6, 7]);   %最少的步数为啥这样计算？
        
        %*********---------限制最后四个变量为1代表停止运动，这里要腿固定不动？？？
        obj.vars.trim.lb(end-3:end) = 1;
        obj.vars.trim.ub(end-3:end) = 1;
        
        %限制开始四个变量为0代表开始运动
        obj.vars.trim.lb(1:4) = 0;
        obj.vars.trim.ub(1:4) = 0;
        
        %没看懂Ai所代表的一次约束不等式的“数量”，不应该是减8，而应该是减4，减去初始值
        Ai = sparse(obj.nsteps-4 + 4 + max(obj.nsteps-8, 0) * 8, obj.nv);
        bi = zeros(size(Ai, 1), 1);
        offset_ = 0;
        expected_offset = size(Ai, 1);
        
        %***************-------------------------------------
        %这里面有nstep个约束，表示trim变量t前后关系，sum(t)与步数的关系
        for j = 2:obj.nsteps
          %添加约束： trim(j) >= trim(j-1)，有nsteps-1个这样的约束
          Ai(offset_+1, obj.vars.trim.i(j)) = -1;
          Ai(offset_+1, obj.vars.trim.i(j-1)) = 1;
          offset_ = offset_ + 1;
        end     %到这里offset_=nsteps-1
        %添加一个约束 最后四步默认是不动的，所以有一个4，min_num_steps是估计需要走的最少步数，sum需要小于N_f - 估计最小步数 + 4 
        %sum(trim) <= obj.nsteps - (min_num_steps - 4)
        Ai(offset_+1, obj.vars.trim.i) = 1;
        bi(offset_+1) = obj.nsteps - (min_num_steps - 4);
        offset_ = offset_ + 1;  %再向下偏移一个
        M = obj.max_distance;   %M是一个很大的值
        %***************-------------------------------------
        %到这里offset_=nsteps
        
        %从初始状态的前四步之后：
        %定义t=0是可以运动的，t=1则不能运动的约束！
        for j = 5:obj.nsteps %每执行一次for循环就添加8个约束
          % x(:,j) - x(:,k) <= M(1-trim(j))
          Ai(offset_+(1:4), obj.vars.footsteps.i(:,j)) = speye(4);
          Ai(offset_+(1:4), obj.vars.footsteps.i(:,j-4)) = -speye(4);
          Ai(offset_+(1:4), obj.vars.trim.i(j)) = M;
          bi(offset_+(1:4)) = M;
          offset_ = offset_ + 4;    %向后偏移4

          % x(:,j) - x(:,k) >= -M(1-trim(j))
          %由于接口类只允许定义<，所以需要转换为<的格式输入进去
          Ai(offset_+(1:4), obj.vars.footsteps.i(:,j)) = -speye(4);
          Ai(offset_+(1:4), obj.vars.footsteps.i(:,j-4)) = speye(4);
          Ai(offset_+(1:4), obj.vars.trim.i(j)) = M;
          bi(offset_+(1:4)) = M;
          offset_ = offset_ + 4; 
        end

        obj = obj.addLinearConstraints(Ai, bi, [], []);
        assert(offset_ == expected_offset);  %判断添加的约束数量和A_i设置的行数是否一致！！

        c = zeros(obj.nv, 1);
        c(obj.vars.trim.i) = -w_trim;
        
        %添加t变量，通过调节它的权重来自适应步数
        obj = obj.addCost([], c, w_trim * obj.nsteps);  
      end
      
      function obj = addTerrainRegions(obj, safe_regions)
        % Add mixed-integer constraints that require that 
        % each footstep lie within one of those safe regions described by A and b.
        % such that for footstep i H(i,:) implies that A*fi < b
        % where H is a binary matrix with sum(H(i)) == 1
        
        if isempty(safe_regions)
          safe_regions = obj.seed_plan.safe_regions;
        end
        nr = length(safe_regions);  %nr表示安全区域的数量
        obj = obj.addVariable('region', 'B', [nr, obj.nsteps], 0, 1);   %H矩阵，nr行，N_f列
        
        obj = obj.addVariable('cent_region', 'C', [3, obj.nsteps], 0, 1);   %这个没啥用

        Ai = sparse((obj.nsteps-4) * sum(cellfun(@(x) size(x, 1) + 2, {safe_regions.A})), obj.nv);
        bi = zeros(size(Ai, 1), 1);
        offset_ineq = 0;
        Aeq = sparse(obj.nsteps-4, obj.nv);
        beq = ones(obj.nsteps-4, 1);
        offset_eq = 0;

        for r = 1:nr
          A = safe_regions(r).A;
          b = safe_regions(r).b;
          Ar = [A(:,1:2), sparse(size(A, 1), 1), 0*A(:,3);
                safe_regions(r).normal', 0;
                -safe_regions(r).normal', 0];
          br = [b;
                safe_regions(r).normal' * safe_regions(r).point;
                -safe_regions(r).normal' * safe_regions(r).point];
          s = size(Ar, 1);
          M = obj.max_distance;
          for j = 5:obj.nsteps  %H矩阵的物理意义
            Ai(offset_ineq + (1:s), obj.vars.footsteps.i(:,j)) = Ar;
            Ai(offset_ineq + (1:s), obj.vars.region.i(r,j)) = M;
            bi(offset_ineq + (1:s)) = br + M;
            offset_ineq = offset_ineq + s;
          end
        end
        assert(offset_ineq == size(Ai, 1));
        for j = 5:obj.nsteps    %H矩阵列和为1
          Aeq(offset_eq + 1, obj.vars.region.i(:,j)) = 1;
          offset_eq = offset_eq + 1;
        end
        assert(offset_eq == size(Aeq, 1));
        obj = obj.addLinearConstraints(Ai, bi, Aeq, beq);
      end
      
      %添加步态约束：T矩阵在此定义，T矩阵每一行只能有一个1，后一个落足点一定要在它之前的四个落足点都结束后才能落下，约束了危险步态；为T矩阵每个位置分配了时间timing但是还没有使用到
      %定义了复杂环境中切换步态是根据delta_z
      function obj = addGaitConstraints(obj)
        % Add mixed-integer constraints that require that assign each
        % leg movement to a timeslot between 1 and nsteps
        % such that for footstep i T(i,j) implies that leg i moves in timestep j
        % where T is a binary matrix with sum(T(i)) == 1

        obj.plan_gait = true;   %添加了步态约束
        
        %去除起始 ***还是终止***的四个状态，T矩阵有obj.nsteps-4行，代表中着接触状态，obj.nsteps-4列代表着可能有obj.nsteps-4时刻，但是会删减
        %上下限为[0,1]
        obj = obj.addVariable('seq', 'B', [obj.nsteps-4, obj.nsteps-4], 0, 1);
        %运动时刻，obj.nsteps-4个变量，上下限为[1,obj.nsteps]，timming代表着在整个序列中运动的先后
        obj = obj.addVariable('timming', 'C', [1, obj.nsteps-4], 1, obj.nsteps);
        
        %指定第一个时刻让第一条腿运动
        obj.vars.seq.lb(1,1) = 1;
        obj.vars.seq.ub(1,1) = 1;
        
        %约束住每一个足端状态只能到达一次，并且为每一个落足点分配运动时刻timming
        for i = 1:obj.nsteps-4
          %为每条 落足状态 分配一个timming
          % assigns each leg movement to a time variable
          Aeq = sparse(1,obj.nv);
          beq = 0;
          Aeq(1,obj.vars.seq.i(i,:)) = 1:obj.nsteps-4;
          Aeq(1,obj.vars.timming.i(1,i)) = -1;
          obj = obj.addLinearConstraints([], [], Aeq, beq);
        
          %sum(seq一行)=1，说明一个落足状态只允许到达一次
          % constrains that that each step is taken in only
          % one timminglot
          Aeq = sparse(1,obj.nv);
          beq = 1;
          Aeq(1,obj.vars.seq.i(i,:)) = 1;
          obj = obj.addLinearConstraints([], [], Aeq, beq);
        end
        
        %四条腿为一个循环，一个循环一个循环的进行,t(i-1,2,3,4)<t(i)，当前一个循环四条腿的运动时刻大于上一个循环的四条腿运动时刻：16个不等式
        %这四行大于上面四行
        % constrains that each leg moves after the previous configuration has
        % completely transfered to the current configuration
        k = 1;
        for i = 5:obj.nsteps-4
          if k > 4
            k = 1;
          end
          Ai = sparse(1,obj.nv);
          bi = -1;
          Ai(1,obj.vars.timming.i(1,i-3-k)) = 1;
          Ai(1,obj.vars.timming.i(1,i)) = -1;
          obj = obj.addLinearConstraints(Ai, bi, [], []);

          Ai = sparse(1,obj.nv);
          bi = -1;
          Ai(1,obj.vars.timming.i(1,i-2-k)) = 1;
          Ai(1,obj.vars.timming.i(1,i)) = -1;
          obj = obj.addLinearConstraints(Ai, bi, [], []);

          Ai = sparse(1,obj.nv);
          bi = -1;
          Ai(1,obj.vars.timming.i(1,i-1-k)) = 1;
          Ai(1,obj.vars.timming.i(1,i)) = -1;
          obj = obj.addLinearConstraints(Ai, bi, [], []);

          Ai = sparse(1,obj.nv);
          bi = -1;
          Ai(1,obj.vars.timming.i(1,i-k)) = 1;
          Ai(1,obj.vars.timming.i(1,i)) = -1;
          obj = obj.addLinearConstraints(Ai, bi, [], []);

          Ai = sparse(1,obj.nv);
          bi = -2;
          Ai(1,obj.vars.timming.i(1,i-4)) = 1;
          Ai(1,obj.vars.timming.i(1,i)) = -1;
          obj = obj.addLinearConstraints(Ai, bi, [], []);

          k = k + 1;
        end
        
        %约束住危险步态：奔跑步态和顺撇步态
        %对H矩阵的约束
        % constrains the possible configurations in the gait
        for i = 1:4:obj.nsteps-7
          for j = 1:obj.nsteps-4
            bi = 1;
            
            %约束两个前腿不能同时摆动（两左腿顺撇步态）
            Ai = sparse(1,obj.nv);
            Ai(1,obj.vars.seq.i(i,j)) = 1;
            Ai(1,obj.vars.seq.i(i+1,j)) = 1;
            obj = obj.addLinearConstraints(Ai, bi, [], []);
            
            %约束两个后腿不能同时摆动（两右腿顺撇步态）（两后腿奔跑）（四足奔跑步态）
            Ai = sparse(1,obj.nv);
            Ai(1,obj.vars.seq.i(i+2,j)) = 1;
            Ai(1,obj.vars.seq.i(i+3,j)) = 1;
            obj = obj.addLinearConstraints(Ai, bi, [], []);
            
            %左侧两条腿不能同时摆动（顺撇步态）
            Ai = sparse(1,obj.nv);
            Ai(1,obj.vars.seq.i(i,j)) = 1;
            Ai(1,obj.vars.seq.i(i+2,j)) = 1;
            obj = obj.addLinearConstraints(Ai, bi, [], []);
            
            %右侧两条腿不能同时摆动（顺撇步态）
            Ai = sparse(1,obj.nv);
            Ai(1,obj.vars.seq.i(i+1,j)) = 1;
            Ai(1,obj.vars.seq.i(i+3,j)) = 1;
            obj = obj.addLinearConstraints(Ai, bi, [], []);
          end
        end

        %***************----------------------
        
        %对高度的限制没有读懂！对高度进行了约束，但是并没有施加惩罚呀！
        % defines variables for the height deviation on each configuration
        % and for the binary approximation of the variable
        gamma_z = obj.gamma_z;  %落足点的高度变化情况
        %为什么使用大M法我没有特别理解，绝对值约束直接是两个不等式不就好了吗
        M = 100; % Large height difference for big-M formulation，大M法中M的定义
        obj = obj.addVariable('desv', 'C', [1, obj.nsteps-4], -inf, inf);   %这一步与上一步之间的高度差
        obj = obj.addVariable('dh', 'B', [2, obj.nsteps-4], 0, 1);          %高度变化过多的标志变量
        obj = obj.addVariable('delta_z', 'C', [1, obj.nsteps-4], -inf, inf);%惩罚项标志，实际上设置为B更加合适一些
        for j = 5:obj.nsteps
          %计算落足点高度变化信息：desv
          % computes the height deviation for each configuration
          Aeq = sparse(1,obj.nv);
          beq = 0;
          Aeq(1,obj.vars.desv.i(1,j-4)) = -1;
          Aeq(1,obj.vars.footsteps.i(3,j)) = 10;        %这里的十倍是单位之间的转换吧
          Aeq(1,obj.vars.footsteps.i(3,j-4)) = -10;
          obj = obj.addLinearConstraints([], [], Aeq, beq);
          
          % makes a dissagregate approximation of the deviation magnitude
          %如果足端高度变化大于gamma_z，则二进制变量为1，否则为0
          Ai = sparse(2,obj.nv);
          bi = [M - gamma_z; gamma_z];
          Ai(1, obj.vars.desv.i(1,j-4)) = -1;
          Ai(2, obj.vars.desv.i(1,j-4)) = 1;
          Ai(1, obj.vars.dh.i(1,j-4)) = M;
          Ai(2, obj.vars.dh.i(1,j-4)) = -M;
          obj = obj.addLinearConstraints(Ai, bi, [], []);
            
          %如果足端高度变化小于-gamma_z，则二进制变量为1，否则为0
          Ai = sparse(2,obj.nv);
          bi = [M - gamma_z; gamma_z];
          Ai(1, obj.vars.desv.i(1,j-4)) = 1;
          Ai(2, obj.vars.desv.i(1,j-4)) = -1;
          Ai(1, obj.vars.dh.i(2,j-4)) = M;
          Ai(2, obj.vars.dh.i(2,j-4)) = -M;
          obj = obj.addLinearConstraints(Ai, bi, [], []);
            
          %获得delta_z变量要么是0要么是1，0代表满足高度转移的限制，1代表不满足
          Aeq = sparse(1,obj.nv);
          beq = 0;
          Aeq(1, obj.vars.dh.i(1,j-4)) = 1;
          Aeq(1, obj.vars.dh.i(2,j-4)) = 1;
          Aeq(1, obj.vars.delta_z.i(1,j-4)) = -1;
          obj = obj.addLinearConstraints([], [], Aeq, beq);
        end
        
        %***************----------------------
        
        %添加代价值：如果高度值变化很大，那么对这一步状态转移发生时有多少条腿，对该腿的总数进行惩罚。
        %对超过高度限制的进行惩罚
        obj = obj.addVariable('s', 'C', [obj.nsteps-4, obj.nsteps-4], -inf, inf);   %设置为0，1二进制数更合适！足端位置发生变化时，高度z是否变化过大
        obj = obj.addVariable('cost_desv', 'C', [1, obj.nsteps-4], -inf, inf);      %确定同一时刻上有多少足端位置的高度z变化过大的,为其添加二次代价值，迫使遇到这种情况下不走对角步态而走波动步态
        
        q_d = 100;  %两步之间的代价值才只是1，goal的二次代价值为1000，所以这里设置100的代价值，几乎是“强制”四足机器人的步态自适应地形
        M = 1000000;
        %在不平坦的地形中the sum of legs的 成本 
        % adds a quadratic cost on the sum of legs in transfer on
        % the unevenness of the terrain
        for j = 1:obj.nsteps-4
          Qi = sparse(obj.nv, obj.nv);
          ci = sparse(obj.nv, 1);

          for i = 1:obj.nsteps-4
            % slack_ij = seq(i,j)*dh(i)
            Ain = sparse(4, obj.nv);
            bin = zeros(4,1);
            
            Ain(1,obj.vars.seq.i(i,j)) = -M;
            Ain(1,obj.vars.s.i(i,j)) = 1;
            bin(1,1) = 0; 

            Ain(2,obj.vars.seq.i(i,j)) = -M;
            Ain(2,obj.vars.s.i(i,j)) = -1;
            bin(2,1) = 0;

            Ain(3,obj.vars.seq.i(i,j)) = M;
            Ain(3,obj.vars.s.i(i,j)) = 1;
            Ain(3,obj.vars.delta_z.i(1,i)) = -1;
            bin(3,1) = M;

            Ain(4,obj.vars.seq.i(i,j)) = M;
            Ain(4,obj.vars.s.i(i,j)) = -1;
            Ain(4,obj.vars.delta_z.i(1,i)) = 1;
            bin(4,1) = M;

            obj = obj.addLinearConstraints(Ain, bin, [], []);
          end

          Aeq = sparse(1, obj.nv);
          beq = zeros(1,1);

          Aeq(1,obj.vars.cost_desv.i(1,j)) = -1;
          Aeq(1,obj.vars.s.i(:,j)) = 1;
          obj = obj.addLinearConstraints([], [], Aeq, beq);

          Qi(obj.vars.cost_desv.i(1,j), obj.vars.cost_desv.i(1,j)) = q_d;
          obj = obj.addCost(Qi, [], []);
        end
        
        %--------添加时间优化目标，让机器人的timing尽量的小从而减小代价
        % adds a cost on the duration of the plan
        q_t = 2;
        ci = zeros(obj.nv, 1);
        for j = 1:obj.nsteps-4
          ci(obj.vars.timming.i(1,j),1) = q_t;
        end
        obj = obj.addCost([], ci, []);

       
      end

      
      function obj = fixRotation(obj)
        % Fix the rotations of every step to the initial value (no rotations allowed)
        k = 1;
        yaw = obj.seed_plan.footsteps(1).pos(6);
        for j = 5:obj.nsteps
          obj.seed_plan.footsteps(j).pos(6) = yaw;
          obj.vars.footsteps.lb(4,j) = yaw - 0.1;
          obj.vars.footsteps.ub(4,j) = yaw + 0.1;
          k = k + 1;
        end

        seed_steps = [obj.seed_plan.footsteps.pos];
        yaw = seed_steps(6,:);
        
        obj = obj.addVariable('cos_yaw', 'C', [1, obj.nsteps], cos(yaw), cos(yaw));
        obj = obj.addVariable('sin_yaw', 'C', [1, obj.nsteps], sin(yaw), sin(yaw));
      end
      
      function plan = getFootstepPlan(obj)
        % Solve the problem if needed and retrieve a footstep plan with the corresponding solution.
        if ~isfield(obj.vars.footsteps, 'value')
          obj = obj.solve();
        end
        steps = zeros(6, obj.nsteps);
        steps(obj.pose_indices, :) = obj.vars.footsteps.value;
        plan = obj.seed_plan;
        for j = 1:obj.nsteps
          plan.footsteps(j).pos = steps(:,j);
        end

        for j = 5:obj.nsteps
          region_ndx = find(obj.vars.region.value(:,j));
          assert(length(region_ndx) == 1, 'Got no (or multiple) region assignments for this footstep. This indicates an infeasibility or bad setup in the mixed-integer program');
          plan.region_order(j) = region_ndx;
        end
        
        if obj.plan_gait
          %obj.vars.desv.value
          %obj.vars.dh.value
          gait_init = int8(obj.vars.seq.value);
          gait_idx  = int8(obj.vars.timming.value);
          idx = max(gait_idx);
          gait_fin = zeros(4,idx);
          for i = 1:4:length(gait_idx)-3
            gait_fin(1,int8(gait_idx(i)))   =  obj.vars.seq.value(i,int8(gait_idx(i)));
            gait_fin(2,int8(gait_idx(i+1))) =  obj.vars.seq.value(i+1,int8(gait_idx(i+1)));
            gait_fin(3,int8(gait_idx(i+2))) =  obj.vars.seq.value(i+2,int8(gait_idx(i+2)));
            gait_fin(4,int8(gait_idx(i+3))) =  obj.vars.seq.value(i+3,int8(gait_idx(i+3)));
          end
          i = 1;
          while 1
            if gait_fin(1:4,i) == zeros(4,1)
              for j = i:length(gait_fin)-1
                gait_fin(1:4,j) = gait_fin(1:4,j+1);
              end
              gait_fin = gait_fin(:,1:end-1);
              i = i - 1;
            end
            if i == size(gait_fin,2)
              break;
            else
              i = i + 1;
            end
          end
          plan.gait = gait_fin;
          plan.timming = gait_idx;
        else
          gait_ex = [1 0 0 0;0 0 1 0;0 0 0 1;0 1 0 0];
          plan.gait = repmat(gait_ex,1,(obj.nsteps-4)/4);
          plan.timming = 1:(obj.nsteps-4);
        end

        %plan = plan.trim_duplicates();
      end
    end
end