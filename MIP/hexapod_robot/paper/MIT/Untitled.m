classdef MixedIntegerFootstepPlanningProblem < MixedIntegerConvexProgram
% A general structure for various footstep planning approaches. For example implementations,
% see footstepPlanner.humanoids2014, footstepPlanner.linearUnitCircle, and footstepPlanner.fixedRotation
  properties
    biped;
    nsteps;     %初始时的总步数
    seed_plan;
    weights;
    max_distance = 30;  %场地的大小，是决策变量的上下限
    pose_indices = [1,2,3,6];   %从 [x,y,z,r,p,y]中拿出位置和yaw角
  end

  methods      
    %构造函数，设置初始步数N，添加落足点决策变量，约束住初始状态的上下限，不让Gurobi对其求解
    function obj = MixedIntegerFootstepPlanningProblem(biped, seed_plan, has_symbolic)%使用到了
      % Construct a new problem, optionally with internal symbolic representations of
      % all variables. For more info on the symbolic vars, see MixedIntegerConvexProgram
      % @param biped a Biped.
      % @param seed_plan a blank footstep plan, provinding the structure of the
      %                  desired plan. Probably generated with
      %                  FootstepPlan.blank_plan()
      % @param has_symbolic whether to keep symbolic variables

      typecheck(biped, 'Biped');
      typecheck(seed_plan, 'FootstepPlan');

      obj = obj@MixedIntegerConvexProgram(has_symbolic);

      obj.biped = biped;
      obj.seed_plan = seed_plan;
      obj.nsteps = length(obj.seed_plan.footsteps);
      %obj.weights = obj.biped.getFootstepOptimizationWeights();     %获取权重。
      obj.weights = struct('relative', [1;1;1;0;0;0.05],...
                       'relative_final', [10;10;10;0;0;1],...
                       'goal', [1000;1000;0;0;0;100]);

      seed_steps = [seed_plan.footsteps.pos];
      min_yaw = pi * floor(seed_steps(6,1) / pi - 1);
      max_yaw = pi * ceil(seed_steps(6,1) / pi + 1);

      lb = [repmat(seed_steps(1:3,1) - obj.max_distance, 1, obj.nsteps);
            min_yaw + zeros(1, obj.nsteps)];
      ub = [repmat(seed_steps(1:3,1) + obj.max_distance, 1, obj.nsteps);
            max_yaw + zeros(1, obj.nsteps)];
      lb(:,1) = seed_steps(obj.pose_indices, 1);
      ub(:,1) = seed_steps(obj.pose_indices, 1);
      lb(:,2) = seed_steps(obj.pose_indices, 2);
      ub(:,2) = seed_steps(obj.pose_indices, 2);
      obj = obj.addVariable('footsteps', 'C', [4, obj.nsteps], lb, ub);
    end
    
    %添加到达goal的二次目标代价值，没有固定写序列的最后一步，猜测可能是在优化步数时这个会有变动
    function obj = addQuadraticGoalObjective(obj, goal_pose, step_indices, relative_weights, use_symbolic) %使用到了
      % For each index j in step_indices, add a cost of the form:
      % relative_weights(j) * (footsteps(:,j) - xgoal)' * w_goal * (footsteps(:,j) - xgoal)
      w_goal = diag(obj.weights.goal(obj.pose_indices));

      if use_symbolic
        assert(obj.has_symbolic);
        for i = 1:length(step_indices)
          j = step_indices(i);
          if obj.seed_plan.footsteps(j).frame_id == obj.biped.foot_frame_id.right
            xg = goal_pose.right(obj.pose_indices);
          else
            xg = goal_pose.left(obj.pose_indices);
          end
          err = obj.vars.footsteps.symb(:,j) - xg;
          obj.symbolic_objective = obj.symbolic_objective + relative_weights(i) * err' * w_goal * err; 
        end
      else
        for j = step_indices
          if obj.seed_plan.footsteps(j).frame_id == obj.biped.foot_frame_id.right
            xg = goal_pose.right(obj.pose_indices);
          else
            xg = goal_pose.left(obj.pose_indices);
          end
          Qi = sparse([], [], [], obj.nv, obj.nv, 4);
          ci = zeros(obj.nv, 1);
          Qi(obj.vars.footsteps.i(:,j), obj.vars.footsteps.i(:,j)) = w_goal;
          ci(obj.vars.footsteps.i(:,j)) = -2 * w_goal * xg;
          objcon_i = xg' * w_goal * xg;

          obj = obj.addCost(Qi, ci, objcon_i);
        end
      end
    end
    
    function obj = addSinCosLinearEquality(obj, use_symbolic)   %使用到了
      % Add mixed-integer linear constraints implementing the piecewise linear relationship
      % between yaw and cos(yaw), sin(yaw) described in "Footstep Planning on
      % Uneven Terrain with Mixed-Integer Convex Optimization" by Robin Deits and
      % Russ Tedrake (Humanoids 2014)
      yaw0 = obj.seed_plan.footsteps(1).pos(6);
      min_yaw = pi * floor(yaw0 / pi - 1);  %0
      max_yaw = pi * ceil(yaw0 / pi + 1);   %2*pi
      cos_boundaries = reshape(bsxfun(@plus, [min_yaw:pi:max_yaw; min_yaw:pi:max_yaw], [-(pi/2-1); (pi/2-1)]), 1, []);
      sin_boundaries = reshape(bsxfun(@plus, [min_yaw:pi:max_yaw; min_yaw:pi:max_yaw], [-1; 1]), 1, []);

      obj = obj.addVariableIfNotPresent('cos_yaw', 'C', [1, obj.nsteps], -1, 1);    %sin
      obj = obj.addVariableIfNotPresent('sin_yaw', 'C', [1, obj.nsteps], -1, 1);    %cos
      obj = obj.addVariable('cos_sector', 'B', [length(cos_boundaries)-1, obj.nsteps], 0, 1);   %5段
      obj = obj.addVariable('sin_sector', 'B', [length(sin_boundaries)-1, obj.nsteps], 0, 1);
      obj = obj.addInitialSinCosConstraints();
      
      obj.vars.footsteps.lb(4,3:end) = min_yaw;
      obj.vars.footsteps.ub(4,3:end) = max_yaw;

      
        %正常使用的
        obj = obj.addVariable('unit_circle_slack', 'C', [1,1], norm([pi/4;pi/4]), norm([pi/4;pi/4]));
        Aeq_s = zeros(obj.nsteps, obj.nv);
        Aeq_c = zeros(obj.nsteps, obj.nv);
        beq = ones(size(Aeq_s, 1), 1);
        for j = 1:obj.nsteps
          Aeq_c(j, obj.vars.cos_sector.i(:,j)) = 1; %5段中只能有一个成立，也就是五个0,1变量中只能有一个是1
          Aeq_s(j, obj.vars.sin_sector.i(:,j)) = 1;
        end
        obj = obj.addLinearConstraints([], [], [Aeq_s; Aeq_c], [beq; beq]); %5段中只有一个成立的条件添加进去
        %添加线性锥约束，变量的Index传入的是：N-2个1.1107，cosyaw和sinyaw，,cos、sin和1.1107组成一列用八棱锥近似圆锥
        obj = obj.addPolyConesByIndex([repmat(obj.vars.unit_circle_slack.i, 1, obj.nsteps-2); obj.vars.cos_yaw.i(3:end); obj.vars.sin_yaw.i(3:end)], 8);

        M = 2*pi;
        Ai = zeros((obj.nsteps-2) * (length(cos_boundaries)-1 + length(sin_boundaries)-1) * 4, obj.nv);
        bi = zeros(size(Ai, 1), 1);
        offset = 0;
        expected_offset = size(Ai, 1);
        for s = 1:length(cos_boundaries)-1
          th0 = cos_boundaries(s);
          th1 = cos_boundaries(s+1);

          th = (th0 + th1)/2;
          cos_slope = -sin(th);
          cos_intercept = cos(th) - (cos_slope * th);

          for j = 3:obj.nsteps
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

          for j = 3:obj.nsteps
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
        Ai = zeros((obj.nsteps-2) * obj.vars.sin_sector.size(1) * 2, obj.nv);
        bi = zeros(size(Ai, 1), 1);
        offset = 0;
        expected_offset = size(Ai, 1);
        nsectors = obj.vars.sin_sector.size(1);
        for k = 1:nsectors
          for j = 3:obj.nsteps
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
        Ai = zeros((obj.nsteps-2) * (nsectors-1) * 2, obj.nv);
        bi = zeros(size(Ai, 1), 1);
        offset = 0;
        expected_offset = size(Ai, 1);
        for j = 3:obj.nsteps
          if obj.seed_plan.footsteps(j).frame_id == obj.biped.foot_frame_id.left
            for k = 1:nsectors - 1
              Ai(offset+1, obj.vars.cos_sector.i(k,j-1)) = 1;
              Ai(offset+1, obj.vars.cos_sector.i(k:k+1,j)) = -1;
              Ai(offset+2, obj.vars.sin_sector.i(k,j-1)) = 1;
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

    function obj = addInitialSinCosConstraints(obj) %使用到了
      % Constrain the values of sin and cos for the current poses of the feet
      obj.vars.cos_yaw.lb(1) = cos(obj.seed_plan.footsteps(1).pos(6));
      obj.vars.cos_yaw.ub(1) = cos(obj.seed_plan.footsteps(1).pos(6));
      obj.vars.cos_yaw.lb(2) = cos(obj.seed_plan.footsteps(2).pos(6));
      obj.vars.cos_yaw.ub(2) = cos(obj.seed_plan.footsteps(2).pos(6));
      obj.vars.sin_yaw.lb(1) = sin(obj.seed_plan.footsteps(1).pos(6));
      obj.vars.sin_yaw.ub(1) = sin(obj.seed_plan.footsteps(1).pos(6));
      obj.vars.sin_yaw.lb(2) = sin(obj.seed_plan.footsteps(2).pos(6));
      obj.vars.sin_yaw.ub(2) = sin(obj.seed_plan.footsteps(2).pos(6));
    end
    
    %为高度z，角度yaw添加两步之间的绝对值约束，绝对值展开会有两个子约束
    function obj = addZAndYawReachability(obj, use_symbolic)%使用到了
      % Add basic limits on delta z and delta yaw between footsteps
      % 实际上很简单，就是delta z的绝对值小于一个上限，绝对值不等式会被拆分为两个子不等式
      % 出去初始的两步，总共有obj.nsteps-2个z和yaw的变化需要被约束，(obj.nsteps-2)*2个绝对值不等式，因此一共有(obj.nsteps-2)*4个不等式
      if use_symbolic
        assert(obj.has_symbolic);
        x = obj.vars.footsteps.symb;
        yaw = x(4,:);
        for j = 3:obj.nsteps
          if obj.seed_plan.footsteps(j).frame_id == obj.biped.foot_frame_id.left
            obj.symbolic_constraints = [obj.symbolic_constraints,...
              -obj.seed_plan.params.max_inward_angle <= yaw(j) - yaw(j-1) <= obj.seed_plan.params.max_outward_angle];
          else
            obj.symbolic_constraints = [obj.symbolic_constraints,...
              -obj.seed_plan.params.max_outward_angle <= yaw(j) - yaw(j-1) <= obj.seed_plan.params.max_inward_angle];
          end
          obj.symbolic_constraints = [obj.symbolic_constraints,...
            -obj.seed_plan.params.nom_downward_step <= x(3,j) - x(3,j-1) <= obj.seed_plan.params.nom_upward_step];
        end
      else
        Ai = zeros((obj.nsteps-2)*4, obj.nv);   %
        bi = zeros(size(Ai, 1), 1);
        offset = 0;
        expected_offset = size(Ai, 1);
        for j = 3:obj.nsteps
          if obj.seed_plan.footsteps(j).frame_id == obj.biped.foot_frame_id.left
            % obj.symbolic_constraints = [obj.symbolic_constraints,...
            %   -obj.seed_plan.params.max_inward_angle <= yaw(j) - yaw(j-1) <= obj.seed_plan.params.max_outward_angle];
            Ai(offset+1, obj.vars.footsteps.i(4,j)) = -1;
            Ai(offset+1, obj.vars.footsteps.i(4,j-1)) = 1;
            bi(offset+1) = obj.seed_plan.params.max_inward_angle;
            Ai(offset+2, obj.vars.footsteps.i(4,j)) = 1;
            Ai(offset+2, obj.vars.footsteps.i(4,j-1)) = -1;
            bi(offset+2) = obj.seed_plan.params.max_outward_angle;
            offset = offset + 2;
          else
            % obj.symbolic_constraints = [obj.symbolic_constraints,...
            %   -obj.seed_plan.params.max_outward_angle <= yaw(j) - yaw(j-1) <= obj.seed_plan.params.max_inward_angle];
            Ai(offset+1, obj.vars.footsteps.i(4,j)) = -1;
            Ai(offset+1, obj.vars.footsteps.i(4,j-1)) = 1;
            bi(offset+1) = obj.seed_plan.params.max_outward_angle;
            Ai(offset+2, obj.vars.footsteps.i(4,j)) = 1;
            Ai(offset+2, obj.vars.footsteps.i(4,j-1)) = -1;
            bi(offset+2) = obj.seed_plan.params.max_inward_angle;
            offset = offset + 2;
          end
          % obj.symbolic_constraints = [obj.symbolic_constraints,...
          %   -obj.seed_plan.params.nom_downward_step <= x(3,j) - x(3,j-1) <= obj.seed_plan.params.nom_upward_step];
          Ai(offset+1, obj.vars.footsteps.i(3,j)) = -1;
          Ai(offset+1, obj.vars.footsteps.i(3,j-1)) = 1;
          bi(offset+1) = obj.seed_plan.params.nom_downward_step;
          Ai(offset+2, obj.vars.footsteps.i(3,j)) = 1;
          Ai(offset+2, obj.vars.footsteps.i(3,j-1)) = -1;
          bi(offset+2) = obj.seed_plan.params.nom_upward_step;
          offset = offset + 2;
        end
        assert(offset == expected_offset);
        obj = obj.addLinearConstraints(Ai, bi, [], []);
      end
    end

    function obj = addXYReachabilityCircles(obj, use_symbolic)%使用到了
      % Add quadratic constraints to restrict the relative foot displacements in X and Y. This
      % is the reachability method described in "Footstep Planning on
      % Uneven Terrain with Mixed-Integer Convex Optimization" by Robin Deits and
      % Russ Tedrake
      if use_symbolic
        assert(obj.has_symbolic);
        x = obj.vars.footsteps.symb;
        cos_yaw = obj.vars.cos_yaw.symb;
        sin_yaw = obj.vars.sin_yaw.symb;
        for j = 3:obj.nsteps
          [rel_foci, radii] = obj.biped.getReachabilityCircles(obj.seed_plan.params, obj.seed_plan.footsteps(j-1).frame_id);
          for k = 1:size(rel_foci, 2)
            obj.symbolic_constraints = [obj.symbolic_constraints, ...
              cone(x(1:2,j-1) + [cos_yaw(j-1), -sin_yaw(j-1); sin_yaw(j-1), cos_yaw(j-1)] * rel_foci(:,k) - x(1:2,j), radii(k))];
          end
        end
      else
        ncons = 2 * (obj.nsteps-2);
        quadcon = struct('Qc', repmat({sparse(obj.nv, obj.nv)}, 1, ncons), 'q', repmat({zeros(obj.nv, 1)}, 1, ncons), 'rhs', repmat({0}, 1, ncons));
        offset = 0;
        expected_offset = length(quadcon);
        for j = 2:obj.nsteps-1
          [rel_foci, radii] = obj.biped.getReachabilityCircles(obj.seed_plan.params, obj.seed_plan.footsteps(j).frame_id);
          assert(size(rel_foci, 2) == 2, 'I have hard-coded the number of reachability circles in this code. You can set use_symbolic=true if you need more than two');

          for k = 1:size(rel_foci, 2)
            quadcon(offset+1).rhs = radii(k)^2;

            Qc_elementwise = [...
              obj.vars.footsteps.i(1:2,j+1), obj.vars.footsteps.i(1:2,j+1), [1;1];
              obj.vars.footsteps.i(1:2,j), obj.vars.footsteps.i(1:2,j), [1;1];
              obj.vars.footsteps.i(1:2,j+1), obj.vars.footsteps.i(1:2,j), [-1; -1];
              obj.vars.footsteps.i(1:2,j), obj.vars.footsteps.i(1:2,j+1), [-1; -1];
              obj.vars.footsteps.i(1,j+1), obj.vars.cos_yaw.i(j), -1 * rel_foci(1,k);
              obj.vars.cos_yaw.i(j), obj.vars.footsteps.i(1,j+1), -1 * rel_foci(1,k);
              obj.vars.footsteps.i(1,j+1), obj.vars.sin_yaw.i(j), 1 * rel_foci(2,k);
              obj.vars.sin_yaw.i(j), obj.vars.footsteps.i(1,j+1), 1 * rel_foci(2,k);
              obj.vars.footsteps.i(2,j+1), obj.vars.sin_yaw.i(j), -1 * rel_foci(1,k);
              obj.vars.sin_yaw.i(j), obj.vars.footsteps.i(2,j+1), -1 * rel_foci(1,k);
              obj.vars.footsteps.i(2,j+1), obj.vars.cos_yaw.i(j) -1 * rel_foci(2,k);
              obj.vars.cos_yaw.i(j), obj.vars.footsteps.i(2,j+1), -1 * rel_foci(2,k);
              obj.vars.footsteps.i(1,j), obj.vars.cos_yaw.i(j), 1 * rel_foci(1,k);
              obj.vars.cos_yaw.i(j), obj.vars.footsteps.i(1,j), 1 * rel_foci(1,k);
              obj.vars.footsteps.i(1,j), obj.vars.sin_yaw.i(j), -1 * rel_foci(2,k);
              obj.vars.sin_yaw.i(j), obj.vars.footsteps.i(1,j), -1 * rel_foci(2,k);
              obj.vars.footsteps.i(2,j), obj.vars.sin_yaw.i(j), 1 * rel_foci(1,k);
              obj.vars.sin_yaw.i(j), obj.vars.footsteps.i(2,j), 1 * rel_foci(1,k);
              obj.vars.footsteps.i(2,j) obj.vars.cos_yaw.i(j), 1 * rel_foci(2,k);
              obj.vars.cos_yaw.i(j), obj.vars.footsteps.i(2,j), 1 * rel_foci(2,k);
              obj.vars.cos_yaw.i(j), obj.vars.cos_yaw.i(j), rel_foci(1,k)^2 + rel_foci(2,k)^2;
              obj.vars.sin_yaw.i(j), obj.vars.sin_yaw.i(j), rel_foci(1,k)^2 + rel_foci(2,k)^2;
              ];
            quadcon(offset+1).Qc = sparse(Qc_elementwise(:,1), Qc_elementwise(:,2), Qc_elementwise(:,3), obj.nv, obj.nv);

            quadcon(offset+1).q = zeros(obj.nv, 1);
            offset = offset + 1;
          end
        end
        assert(offset == expected_offset);
        obj = obj.addQuadcon(quadcon);
      end
    end

    %加入Trim，三个约束，数学意义一个Trim=1后的所有Trim全是1；最大值估计；大M法添加物理约束
    function obj = addTrimToFinalPoses(obj, use_symbolic) %使用到了
      % Add a binary variable for each footstep which, if true, forces that footstep to the
      % final pose in the footstep plan. This allows us to trim it out of the footstep plan later.
      % A linear objective placed on those trim variables lets us tune the number of footsteps
      % in the plan.
      if obj.nsteps <= 3
        % Only one step to take, no point in trimming
        return
      end
      obj = obj.addVariable('trim', 'B', [1, obj.nsteps], 0, 1);
      w_trim = obj.weights.relative(1) * (obj.seed_plan.params.nom_forward_step^2);
      min_num_steps = max([obj.seed_plan.params.min_num_steps + 2, 3]);

      if use_symbolic
        assert(obj.has_symbolic);
        trim = obj.vars.trim.symb;
        x = obj.vars.footsteps.symb;
        obj.symbolic_constraints = [obj.symbolic_constraints,...
          trim(end-1:end) == 1,...
          trim(1:2) == 0,...
          trim(2:end) >= trim(1:end-1),...
          sum(trim) <= obj.nsteps - (min_num_steps - 2)];
        for j = 3:obj.nsteps-2
          if mod(obj.nsteps-j, 2)
            obj.symbolic_constraints = [obj.symbolic_constraints, implies(trim(j), x(:,j) == x(:,end-1))];
          else
            obj.symbolic_constraints = [obj.symbolic_constraints, implies(trim(j), x(:,j) == x(:,end))];
          end
        end
        obj.symbolic_objective = obj.symbolic_objective - w_trim * sum(trim) + w_trim * obj.nsteps;
      else
        %1.初始和最后固定住
        obj.vars.trim.lb(end-1:end) = 1;    %最后一步的trim就是1
        obj.vars.trim.ub(end-1:end) = 1;
        obj.vars.trim.lb(1:2) = 0;          %初始的一步trim一定是0（除非目标点就是初始点）
        obj.vars.trim.ub(1:2) = 0;

        Ai = zeros(obj.nsteps-1 + 1 + max(obj.nsteps-4, 0) * 8, obj.nv);    %注意行数！
        bi = zeros(size(Ai, 1), 1);
        offset = 0;
        expected_offset = size(Ai, 1);  %最大偏移量
        
        %2. trim=1代表停下来了，当前一步停下来那么之后所有的步数都要停下来
        for j = 2:obj.nsteps   
          % trim(j) >= trim(j-1)，
          Ai(offset+1, obj.vars.trim.i(j)) = -1;
          Ai(offset+1, obj.vars.trim.i(j-1)) = 1;
          offset = offset + 1;
        end     %一共建立了obj.nsteps-1个不等式，此时offset=obj.nsteps-1
        
        
        %3.从min_steps获得sum(trim)一个上界
        % sum(trim) <= obj.nsteps - (min_num_steps - 2)，min_num_stes应该是最激进的估计，每一步都能走最远，2代表两个初始状态
        Ai(offset+1, obj.vars.trim.i) = 1;
        bi(offset+1) = obj.nsteps - (min_num_steps - 2);
        offset = offset + 1;    %此时一共建立了obj.nsteps个不等式，此时offset=obj.nsteps
        
        %4.赋予trim实际物理意义，即:动还是动；大M法表达if语句。
        M = obj.max_distance;
        for j = 3:obj.nsteps-2  %这个for循环实际上就是未trim赋予真正的物理意义，那就是如果trim=1，那么这一步就与最后一步的状态是相同的
          if mod(obj.nsteps-j, 2)
            % obj.symbolic_constraints = [obj.symbolic_constraints, implies(trim(j), x(:,j) == x(:,end-1))];
            k = obj.nsteps-1;   %左脚
          else
            k = obj.nsteps;     %右脚
          end
          % x(:,j) - x(:,k) <= M(1-trim(j))，trim=1的时候才是重要的
          Ai(offset+(1:4), obj.vars.footsteps.i(:,j)) = speye(4);   %******------奇怪的是(x,y,z)与yaw相加并没有什么实际的物理意义啊；
          Ai(offset+(1:4), obj.vars.footsteps.i(:,k)) = -speye(4);
          Ai(offset+(1:4), obj.vars.trim.i(j)) = M;
          bi(offset+(1:4)) = M;
          offset = offset + 4;

          % x(:,j) - x(:,k) >= -M(1-trim(j))，trim=1的时候才是重要的
          Ai(offset+(1:4), obj.vars.footsteps.i(:,j)) = -speye(4);
          Ai(offset+(1:4), obj.vars.footsteps.i(:,k)) = speye(4);
          Ai(offset+(1:4), obj.vars.trim.i(j)) = M;
          bi(offset+(1:4)) = M;
          offset = offset + 4;
        end     %该循环添加了(obj.nsteps-4)*8个不等式约束，到此一共添加了obj.nsteps + (obj.nsteps-4)*8个不等式约束，与一开始为Ai设计的行数相同了
        obj = obj.addLinearConstraints(Ai, bi, [], []);
        assert(offset == expected_offset);
        
        %5.加入代价值，weight_trim是负数，让gurobi最小化目标值的时候让sum(trim)变小
        c = zeros(obj.nv, 1);
        c(obj.vars.trim.i) = -w_trim;
        obj = obj.addCost([], c, w_trim * obj.nsteps);
      end
    end
    
    %添加两步之间的约束，特点在于最后一步的代价值比其他两步之间的 大
    function obj = addQuadraticRelativeObjective(obj, use_symbolic)%使用到了
      % Add a quadratic cost on the relative displacement between footsteps
      if use_symbolic
        assert(obj.has_symbolic);
        x = obj.vars.footsteps.symb;
        for j = 3:obj.nsteps
          R = [obj.vars.cos_yaw.symb(j-1), -obj.vars.sin_yaw.symb(j-1); 
               obj.vars.sin_yaw.symb(j-1), obj.vars.cos_yaw.symb(j-1)];
          if j == obj.nsteps
            w_rel = diag(obj.weights.relative_final(obj.pose_indices));
          else
            w_rel = diag(obj.weights.relative(obj.pose_indices));
          end
          if obj.seed_plan.footsteps(j-1).frame_id == obj.biped.foot_frame_id.right
            nom = [0; obj.seed_plan.params.nom_step_width];
          else
            nom = [0; -obj.seed_plan.params.nom_step_width];
          end
          err = x(:,j) - [x(1:2,j-1) + R * nom; 
                          x(3:4,j-1)];
          obj.symbolic_objective = obj.symbolic_objective + err' * w_rel * err;
        end
      else
        Qi = sparse(obj.nv, obj.nv);
        for j = 3:obj.nsteps
          if j == obj.nsteps
            w_rel = obj.weights.relative_final(obj.pose_indices);
          else
            w_rel = obj.weights.relative(obj.pose_indices);
          end
%           'relative', [1;1;1;0;0;0.05],
%           'relative_final', [10;10;10;0;0;1],
          
          if obj.seed_plan.footsteps(j-1).frame_id == obj.biped.foot_frame_id.right
            nom = [0; obj.seed_plan.params.nom_step_width];     %正常的步长？
          else
            nom = [0; -obj.seed_plan.params.nom_step_width];
          end
          assert(nom(1) == 0, 'I have hard-coded the assumption that nom(1) == 0. You can set use_symbolic=true if you want a non-zero value');
          Qnew = [...
            obj.vars.footsteps.i(1:2,j), obj.vars.footsteps.i(1:2,j), w_rel(1:2);       %x2,y2的二次代价
            obj.vars.footsteps.i(1:2,j-1), obj.vars.footsteps.i(1:2,j-1), w_rel(1:2);   %x1,y1的二次代价
            obj.vars.footsteps.i(1:2,j), obj.vars.footsteps.i(1:2,j-1), -w_rel(1:2);    %-2x1'Qx2
            obj.vars.footsteps.i(1:2,j-1), obj.vars.footsteps.i(1:2,j), -w_rel(1:2);
            
            obj.vars.sin_yaw.i(j-1), obj.vars.sin_yaw.i(j-1), w_rel(1) * nom(2)^2;      %对sin(yaw)值也约束
            obj.vars.cos_yaw.i(j-1), obj.vars.cos_yaw.i(j-1), w_rel(2) * nom(2)^2;      %对cos(yaw)值也约束
            
            %不理解这些代价代表的含义是什么，难道是单腿xy空间的约束造成的？
            obj.vars.footsteps.i(1,j), obj.vars.sin_yaw.i(j-1), w_rel(1) * nom(2);      %x2与sin(yaw1)
            obj.vars.sin_yaw.i(j-1), obj.vars.footsteps.i(1,j), w_rel(1) * nom(2);
            obj.vars.footsteps.i(2,j), obj.vars.cos_yaw.i(j-1), -w_rel(2) * nom(2);     %y2与cos(yaw1)
            obj.vars.cos_yaw.i(j-1), obj.vars.footsteps.i(2,j), -w_rel(2) * nom(2);
            obj.vars.footsteps.i(1,j-1), obj.vars.sin_yaw.i(j-1), -w_rel(1) * nom(2);   %x1与sin(yaw1)
            obj.vars.sin_yaw.i(j-1), obj.vars.footsteps.i(1,j-1), -w_rel(1) * nom(2);
            obj.vars.footsteps.i(2,j-1), obj.vars.cos_yaw.i(j-1), w_rel(2) * nom(2);    %y1与sin(yaw1)
            obj.vars.cos_yaw.i(j-1), obj.vars.footsteps.i(2,j-1), w_rel(2) * nom(2);
            
            
            obj.vars.footsteps.i(3:4,j), obj.vars.footsteps.i(3:4,j), w_rel(3:4);       %z和yaw的相对代价值
            obj.vars.footsteps.i(3:4,j-1), obj.vars.footsteps.i(3:4,j), -w_rel(3:4);
            obj.vars.footsteps.i(3:4,j), obj.vars.footsteps.i(3:4,j-1), -w_rel(3:4);
            obj.vars.footsteps.i(3:4,j-1), obj.vars.footsteps.i(3:4,j-1), w_rel(3:4);
            ];

          Qnew = sparse(Qnew(:,1), Qnew(:,2), Qnew(:,3), obj.nv, obj.nv);   %创建稀疏矩阵，x索引，y索引，值
          for k = 1:10
            x1 = rand(4,1);
            x2 = rand(4,1);
            sy = rand();
            cy = rand();
            X = zeros(obj.nv, 1);
            X(obj.vars.footsteps.i(:,j-1)) = x1;
            X(obj.vars.footsteps.i(:,j)) = x2;
            X(obj.vars.sin_yaw.i(j-1)) = sy;
            X(obj.vars.cos_yaw.i(j-1)) = cy;
            err = (x2 - [x1(1:2) + [cy, -sy; sy, cy] * nom; x1(3:4)]);
            O_des = err' * diag(w_rel) * err;
            O_actual = X' * Qnew * X;
            valuecheck(O_des, O_actual);
          end


          Qi = Qi + Qnew;
        end
        obj = obj.addCost(Qi, [], []);
      end
    end

    function obj = addTerrainRegions(obj, safe_regions, use_symbolic)%使用到了
      % Add regions of safe terrain and mixed-integer constraints which require that 
      % each footstep lie within one of those safe regions.
      if isempty(safe_regions)
        safe_regions = obj.seed_plan.safe_regions;
      end
      nr = length(safe_regions);
      obj = obj.addVariable('region', 'B', [nr, obj.nsteps], 0, 1);     %创建H矩阵
      obj.vars.region.lb(1,1:2) = 1;    %约束住初始位置在region_1
      obj.vars.region.ub(1,1:2) = 1;
      obj.vars.region.lb(2:end,1:2) = 0;
      obj.vars.region.ub(2:end,1:2) = 0;

      if use_symbolic
        assert(obj.has_symbolic)
        region = obj.vars.region.symb;
        x = obj.vars.footsteps.symb;
        obj.symbolic_constraints = [obj.symbolic_constraints,...
          sum(region, 1) == 1];
        for r = 1:nr
          A = safe_regions(r).A;
          b = safe_regions(r).b;
          Ar_ineq = [A(:,1:2), zeros(size(A, 1), 1), A(:,3)];
          br_ineq = b;
          Ar_eq = [safe_regions(r).normal', 0];
          br_eq = safe_regions(r).normal' * safe_regions(r).point;

          for j = 3:obj.nsteps
            obj.symbolic_constraints = [obj.symbolic_constraints,...
              implies(region(r,j), [Ar_ineq * x(:,j) <= br_ineq,...
                                    Ar_eq * x(:,j) == br_eq])];
          end
        end
      else
        Ai = zeros((obj.nsteps-2) * sum(cellfun(@(x) size(x, 1) + 2, {safe_regions.A})), obj.nv);
        bi = zeros(size(Ai, 1), 1);
        offset_ineq = 0;
        Aeq = zeros(obj.nsteps-2, obj.nv);
        beq = ones(obj.nsteps-2, 1);
        offset_eq = 0;

        for r = 1:nr
          A = safe_regions(r).A;
          b = safe_regions(r).b;
          Ar = [A(:,1:2), zeros(size(A, 1), 1), A(:,3);
                safe_regions(r).normal', 0;
                -safe_regions(r).normal', 0];
          br = [b;
                safe_regions(r).normal' * safe_regions(r).point;
                -safe_regions(r).normal' * safe_regions(r).point];
          s = size(Ar, 1);  %代表区域Ar存在多少个约束
          M = obj.max_distance;
          for j = 3:obj.nsteps
            Ai(offset_ineq + (1:s), obj.vars.footsteps.i(:,j)) = Ar;
            Ai(offset_ineq + (1:s), obj.vars.region.i(r,j)) = M;
            bi(offset_ineq + (1:s)) = br + M;
            offset_ineq = offset_ineq + s;
          end
        end
        assert(offset_ineq == size(Ai, 1));
        for j = 3:obj.nsteps
          Aeq(offset_eq + 1, obj.vars.region.i(:,j)) = 1;
          offset_eq = offset_eq + 1;
        end
        assert(offset_eq == size(Aeq, 1));
        obj = obj.addLinearConstraints(Ai, bi, Aeq, beq);
      end
    end

    function plan = getFootstepPlan(obj)%使用到了
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

      for j = 1:obj.nsteps
        region_ndx = find(obj.vars.region.value(:,j));
        assert(length(region_ndx) == 1, 'Got no (or multiple) region assignments for this footstep. This indicates an infeasibility or bad setup in the mixed-integer program');
        plan.region_order(j) = region_ndx;
      end
      plan = plan.trim_duplicates();

    end
  end
end

