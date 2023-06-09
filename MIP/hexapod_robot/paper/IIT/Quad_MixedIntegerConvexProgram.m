%该类实际上是一个数学接口，调用该类的函数可以很方便的处理gurobi的数据格式
classdef Quad_MixedIntegerConvexProgram
    %先看看如何定义这些约束的，具体求解的时候再把约束传递给gurobi对应的变量中即可，例如变量名和变量数据类型，gurobi是分开存储的model.varnames
    %= names;model.vtype = 'B';这在写泛型的时候并不是很方便，因此按照作者的思路来写约束
% This class is meant to represent general mixed-integer linear, quadratic, quadratically-constrained, and second-order-cone programs. 
% It allows you to define symbolic constraints using Yalmip, which are typically easy to prototype,符号约束可以使用Yalmip求解器
% or to define low-level constraints by directly constructing the A, b, Aeq, beq, etc. matrices, which is typically much faster. 
% For an example of usage, see the MixedIntegerFootstepPlanningProblem subclass. 
  properties
    %变量结构体 
    vars = struct();

    %变量的数目
    nv = 0;

    %优化目标的线性项c'x
    c = zeros(0, 1);    % the linear cost vector c'x

    %优化目标的二次项
    Q = sparse(0, 0);   % the quadratic cost matrix x'Qx

    %lp不等式约束，约束矩阵A与约束右侧值向量b，注意通过该类只能定义出小于等于
    A = zeros(0, 0);    % linear inequalities Ax <= b
    b = zeros(0, 1);    

    %lp等式约束，约束矩阵A与约束右侧值向量b
    Aeq = zeros(0, 0);  % linear equalities Ax == b
    beq = zeros(0, 1);

    %二次约束形式，Qc为二次约束矩阵，q为一次约束向量
    quadcon = struct('Qc', {}, 'q', {}, 'rhs', {}); % quadratic constraints x'Qc x + q' x <= rhs

    %优化目标的常数项
    objcon = 0; % constant term in the objective

    % indices of second-order cones (see http://www.gurobi.com/documentation/5.6/reference-manual/matlab_gurobi)
    cones = struct('index', {});

    % indices of polygonal approximations of second-order cones. 
    %The structure of these constraints are designed to mimic the second-order constraints in obj.cones, 
    %but they use a polygonal linear outer approximation of the conic constraint. The number of pieces in each approximation is set by N.
    polycones = struct('index', {}, 'N', {});

    %符号约束项，结构需要使用Yalmip
    symbolic_constraints;   % a list of symbolic constraints constructed with yalmip

    % a symbolic objective term constructed in yalmip
    symbolic_objective = 0;
    
    %结果存储
    x_sol % The solution of all variables
    
    %求解器选择
    solver % either 'gurobi' or 'mosek'. @default is 'gurobi'
  end

  properties (SetAccess = protected)
    has_symbolic = false;   %是否含有符号约束
    symbolic_vars = [];     %符号约束变量
  end

  methods
    
    %构造函数，确定是否存在符号约束
    function obj = Quad_MixedIntegerConvexProgram(has_symbolic)
      % Construct a new mixed-integer convex program.
      % @param has_symbolic whether to create symbolic variables in yalmip corresponding to
      %                     all of the variables in the problem. If obj.has_symbolic is true,
      %                     you can use both symbolic and non-symbolic (and thus faster) 
      %                     constraints as you wish. If obj.has_symbolic is not true, then
      %                     you cannot, and you must instead construct all of your constraint
      %                     and objective matrices directly.
      if nargin < 1
        has_symbolic = false;
      end
      if has_symbolic
        checkDependency('yalmip');
        obj.symbolic_constraints = lmi();
      end

      obj.has_symbolic = has_symbolic;
      
      obj.solver = 'gurobi';
    end
    
    %添加变量
    function obj = addVariable(obj, name, type_, size_, lb, ub, start_)
      % Build a struct to hold the sizes and indices of our decision variables
      % This is a new approach that I'm experimenting with, which should offer
      % a mix of the advantages of symbolic and matrix-based optimization
      % frameworks. The idea is that we have a single Matlab struct (named just
      % 'vars' for convenience) and each variable in the optimization has a
      % corresponding named field in vars. For each variable, we have subfields as
      % follows:
      % type: 'B', 'I', or 'C' for binary, integer, or continuous variables
      % size: a 2x1 vector describing the shape of the variable
      % i: the indices corresponding to the variable, as a matrix of the same size as 
      % the 'size' field above. 
      % lb: lower bound, as a matrix of the same size as 'size'
      % ub: upper bound, a matrix
      % start:  the initial values as a matrix of the same size as 'size'
      %
      % After optimization, there will be an additional field added to each variable, called
      % 'value', which will contain the final values after optimization.
      % 
      % The 'i' field of indices is useful because when
      % we actually set up the problem in gurobi or another solver, all of the
      % optimization variables are combined into one long vector. This index
      % field lets us easily address parts of that vector. For example, to set
      % the entry in a constraint matrix A corresponding to the jth row and kth column 
      % of variable 'foo' to 1, we can do the following:
      % A(1, v.foo.i(j,k)) = 1;
      
      %变量之间不能重名
      if isfield(obj.vars, name)
        error('Drake:MixedIntegerConvexProgram:DuplicateVariableName', 'Cannot add a variable with the same name as an existing one');
      end
      
      %添加新变量，设置name，type，size，i索引
      obj.vars.(name) = struct();   %创建一个名为name的变量
      obj.vars.(name).type = type_; %设置变量类型
      obj.vars.(name).size = size_; %设置变量的大小，size是2×1的，可能代表很多个变量
      
      %vars.(name).i的索引值从nv(上一此的变量数目)+1开始，大小和vars本身大小一致
      obj.vars.(name).i = reshape(obj.nv + (1:prod(obj.vars.(name).size)), obj.vars.(name).size);   %为新加入的变量设置索引值
      
      %设置lb和ub上下限
      if isscalar(lb)   %如果是标量，说明新加入的所有变量都是这个限制，所以重复这个值即可
        lb = repmat(lb, obj.vars.(name).size);
      end
      if isscalar(ub)
        ub = repmat(ub, obj.vars.(name).size);
      end
      obj.vars.(name).lb = lb;
      obj.vars.(name).ub = ub;
      
      %设置变量初值
      if nargin < 7
        start_ = [];
      end
      obj.vars.(name).start = nan(obj.vars.(name).size);
      if size(start_, 1) > obj.vars.(name).size(1)
        start_ = start_(1:obj.vars.(name).size(1),:,:);
      end
      if size(start_, 2) > obj.vars.(name).size(2)
        start_ = start_(:,1:obj.vars.(name).size(2),:);
      end
      obj.vars.(name).start(1:size(start_, 1), 1:size(start_, 2), 1:size(start_, 3)) = start_;

      % Add symbolic variables if we're doing that
      if obj.has_symbolic
        size_cell =num2cell(obj.vars.(name).size);
        if strcmp(obj.vars.(name).type, 'B')
          obj.vars.(name).symb = binvar(size_cell{:}, 'full');
        elseif strcmp(obj.vars.(name).type, 'I')
          obj.vars.(name).symb = intvar(size_cell{:}, 'full');
        else
          obj.vars.(name).symb = sdpvar(size_cell{:}, 'full');
        end
        if isempty(obj.symbolic_vars)
          obj.symbolic_vars = reshape(obj.vars.(name).symb, [], 1);
        else
          obj.symbolic_vars = [obj.symbolic_vars; reshape(obj.vars.(name).symb, [], 1)];
        end
      end
      
      %确定新增变量的个数
      num_new_vars = prod(obj.vars.(name).size);
      
      %更新目前变量的数目
      obj.nv = obj.nv + num_new_vars;
        
      %扩大优化目标线性向量的维度c'x
      obj.c = [obj.c; zeros(num_new_vars, 1)];
      %扩大优化目标二次项的维度x'Qx，行与列都变大了
      obj.Q = [obj.Q, sparse(size(obj.Q, 1), num_new_vars);
               sparse(num_new_vars, num_new_vars + size(obj.Q, 2))];
      %扩大lp约束矩阵的维度，只扩大列，行没有扩大说明lp约束的个数没有发生变化
      obj.A = [obj.A, zeros(size(obj.A, 1), num_new_vars)];
      obj.Aeq = [obj.Aeq, zeros(size(obj.Aeq, 1), num_new_vars)];
      
      %二次约束的个数不建议超过10个，超过十个建议添加“辅助变量”
      if length(obj.quadcon) > 10
        obj.Quadruped.warning_manager.warnOnce('Drake:MixedIntegerConvexProgram:ReallocatingQuadraticConstraints', 'Reallocating matrices for many quadratic constraints. This may be inefficient. If possible, try to finish adding new variables before you start adding quadratic constraints');
      end
      %扩大二次约束的维度
      for j = 1:length(obj.quadcon)
        obj.quadcon(j).Qc = [obj.quadcon(j).Qc, sparse(size(obj.quadcon(j).Qc, 1), num_new_vars);
          sparse(num_new_vars, num_new_vars + size(obj.quadcon(j).Qc, 2))];
        obj.quadcon(j).q = [obj.quadcon(j).q; zeros(num_new_vars, 1)];
      end
    end
    
    %另一种添加变量的接口
    function obj = addVariableIfNotPresent(obj, varargin)
      name = varargin{1};
      if isfield(obj.vars, name)
        return
      else
        obj = obj.addVariable(varargin{:});
      end
    end
    
    %添加线性约束，也就是约束矩阵A和右侧向量b维数扩大了，线性约束这样做是可以的
    function obj = addLinearConstraints(obj, A, b, Aeq, beq)
      obj.A = [obj.A; A];
      obj.b = [obj.b; b];
      obj.Aeq = [obj.Aeq; Aeq];
      obj.beq = [obj.beq; beq];
    end
    
    %添加圆锥约束？
    function obj = addCones(obj, cones)
      obj.cones = [obj.cones, cones];
    end
    function obj = addConesByIndex(obj, idx)
      obj = obj.addCones(struct('index', mat2cell(idx, size(idx, 1), ones(1, size(idx, 2)))));
    end
    function obj = addPolyCones(obj, polycones)
      % Add polygonal approximations of second-order cones
      obj.polycones = [obj.polycones, polycones];
    end
    function obj = addPolyConesByIndex(obj, idx, N)
      % Polycones only support approximations of cones with two variables on the left-hand side 
      % and one on the right-hand side. That is, we can only approximate the constraint that 
      % norm([x2, x3]) <= x1 
      % with the linear constraints
      % A[x2; x3] <= b
      sizecheck(idx, [3, nan]); 

      if length(N) == 1
        N = repmat(N, 1, size(idx, 2));
      else
        assert(length(N) == size(idx, 2));
      end
      obj = obj.addPolyCones(struct('index', mat2cell(idx, size(idx, 1), ones(1, size(idx, 2))), 'N', num2cell(N)));
    end
    function obj = addConesOrPolyConesByIndex(obj, idx, N)
      if nargin < 3 || isempty(N)
        N = 0;
      end
      if all(N == 0)
        obj = obj.addConesByIndex(idx);
      else
        assert(all(N ~= 0), 'Cannot mix cones and polycones in the same call');
        obj = obj.addPolyConesByIndex(idx, N);
      end
    end
    
    %添加二次约束，加入的是结构体，所以使用结构体数组，结构体数组中每一个子结构体代表了一个二次约束
    function obj = addQuadcon(obj, quadcon)
      obj.quadcon = [obj.quadcon, quadcon];
    end
    
    %设置线性目标值c'x
    function obj = setLinearCost(obj, c)
      obj.c = c;
    end
    
    %为指定变量设置线性目标值c(i)*x(i)
    function obj = setLinearCostEntries(obj, idx, val)
      obj.c(idx) = val;
    end
    
    %设置二次目标项QP问题，还能同时设置LP问题的c'x，优化目标的常数项
    function obj = addCost(obj, Q, c, objcon)
      if ~isempty(Q)
        obj.Q = obj.Q + sparse(Q);
      end
      if ~isempty(c)
        obj.c = obj.c + c;
      end
      if ~isempty(objcon)
        obj.objcon = obj.objcon + objcon;
      end
    end

    %添加符号约束项
    function obj = addSymbolicConstraints(obj, expr)
      assert(obj.has_symbolic);
      obj.symbolic_constraints = [obj.symbolic_constraints, expr];
    end
    function obj = addSymbolicCost(obj, expr)
      assert(obj.has_symbolic);
      obj = obj.addSymbolicObjective(expr);
    end
    function obj = addSymbolicObjective(obj, expr)
      assert(obj.has_symbolic);
      obj.symbolic_objective = obj.symbolic_objective + expr;
    end

    function obj = convertPolyCones(obj)
      % Build linear constraints for our polygonal cone approximations
      nconstraints = sum([obj.polycones.N]);
      A = zeros(nconstraints, obj.nv);
      b = zeros(size(A, 1), 1);
      offset = 0;
      for j = 1:length(obj.polycones)
        assert(size(obj.polycones(j).index, 1) == 3, 'polygonal cone approximation only valid for cones with 3 entries (approximates x1 <= norm([x2; x3]))')
        N = obj.polycones(j).N;
        for k = 1:N
          th = (2*pi) / N * (k-1);
          ai = rotmat(th) * [1;0];
          A(offset+1, obj.polycones(j).index(2)) = ai(1);
          A(offset+1, obj.polycones(j).index(3)) = ai(2);
          A(offset+1, obj.polycones(j).index(1)) = -1;
          offset = offset+1;
        end
      end
      assert(offset == nconstraints);

      obj = obj.addLinearConstraints(A, b, [], []);
      obj.polycones = struct('index', {}, 'N', {});
    end

    function [obj, solvertime, objval] = solve(obj)
      if obj.has_symbolic
        [obj, solvertime, objval] = obj.solveYalmip();
      else
        if(strcmp(obj.solver,'gurobi'))
          [obj, solvertime, objval] = obj.solveGurobi();
        elseif(strcmp(obj.solver,'mosek'))
          [obj, solvertime, objval] = obj.solveMosek();
        end
      end
    end
    
    %使用Gurobi求解
    function [obj, solvertime, objval] = solveGurobi(obj, params)
      checkDependency('gurobi');
      if nargin < 2
        params = struct();
      end
      params = applyDefaults(params, struct('outputflag', 0));  %关闭控制台输出
      model = obj.getGurobiModel(); %获取Gurobi模型
      result = gurobi(model, params);   %Gurobi求解
      ok = ~( strcmp(result.status, 'INFEASIBLE') || strcmp(result.status, 'INF_OR_UNBD') );  %模型是否是可求解的，status返回INFEASIBLE与INF_OR_UNBD均是不可求解的
      if ~ok
        error('Drake:MixedIntegerConvexProgram:InfeasibleProblem', 'The mixed-integer problem is infeasible.');
      end
      objval = result.objval;
      solvertime = result.runtime;
      obj = obj.extractResult(result.x);
    end
    
    %转换为Gurobi模型
    function model = getGurobiModel(obj)
      obj = obj.convertPolyCones();

      var_names = fieldnames(obj.vars);     %获取vars中存储了多少“种”变量

      model = struct();
      model.A = sparse([obj.A; obj.Aeq]);
      model.rhs = [obj.b; obj.beq];
      model.sense = [repmat('<', size(obj.A, 1), 1); repmat('=', size(obj.Aeq, 1), 1)]; %所有的一次约束全是小于，所有的等式约束全是等于
      
      model.obj = obj.c;    %一次目标值
      model.Q = obj.Q;      %二次目标值
      if ~isempty(obj.quadcon)
        model.quadcon = obj.quadcon;
      end
      model.objcon = obj.objcon;
      if ~isempty(obj.cones)
        model.cones = obj.cones;
      end

      %这里先定义出Gurobi的变量的形式，之后再把把变量名、变量数据类型、变量的上下限、变量初值复制过来
      % Set up defaults so we can fill them in from v
      model.vtype = repmat('C', obj.nv, 1);
      model.start = nan(obj.nv, 1);
      model.lb = -inf(obj.nv, 1);
      model.ub = inf(obj.nv, 1);
      
      %这里可以为变量命名的，为了后期方便观察
      for j = 1:length(var_names)   %按照变量不同的“种类”分别进行初始化与约束操作
        name = var_names{j};
        i = reshape(obj.vars.(name).i, [], 1);
        model.vtype(i) = obj.vars.(name).type;
        model.lb(i) = reshape(obj.vars.(name).lb, [], 1);
        model.ub(i) = reshape(obj.vars.(name).ub, [], 1);
        model.start(i) = reshape(obj.vars.(name).start, [], 1);
      end
    end
    %传递 解 结果给对象
    function obj = extractResult(obj, x)
      obj.x_sol = x;
      var_names = fieldnames(obj.vars);
      %提取每个变量的值到value属性中
      % Extract the solution
      for j = 1:length(var_names)
        name = var_names{j};
        i = reshape(obj.vars.(name).i, [], 1);  %把变量索引转换为1列
        if obj.vars.(name).type == 'I' 
          obj.vars.(name).value = reshape(round(x(i)), obj.vars.(name).size);   %把整数变量值进行圆整
        elseif obj.vars.(name).type == 'B'
          obj.vars.(name).value = reshape(logical(round(x(i))), obj.vars.(name).size);  %把逻辑变量进行圆整
        else
          obj.vars.(name).value = reshape(x(i), obj.vars.(name).size);  %其余数据类型的变量就不用再圆整了
        end
      end
    end

    function [obj, solvertime, objval] = solveYalmip(obj, params)
      checkDependency('gurobi');
      constraints = obj.symbolic_constraints;
      objective = obj.symbolic_objective;

      if nargin < 2 || isempty(params)
        params = sdpsettings('solver', 'gurobi', 'verbose', 0);
      end

      % Now add in any constraints or objectives which were declared non-symbolically
      objective = objective + obj.symbolic_vars' * obj.Q * obj.symbolic_vars + obj.c' * obj.symbolic_vars + obj.objcon;
      constraints = [constraints,...
        obj.Aeq * obj.symbolic_vars == obj.beq,...
        obj.A * obj.symbolic_vars <= obj.b,...
        ];
      var_names = fieldnames(obj.vars);
      for j = 1:length(var_names)
        name = var_names{j};
        constraints = [constraints,...
         obj.vars.(name).lb <= obj.vars.(name).symb,...
         obj.vars.(name).symb <= obj.vars.(name).ub];
       end
      for j = 1:length(obj.quadcon)
        constraints = [constraints,...
          obj.symbolic_vars' * obj.quadcon(j).Qc * obj.symbolic_vars + obj.quadcon(j).q' * obj.symbolic_vars <= obj.quadcon(j).rhs];
      end
      for j = 1:length(obj.cones)
        constraints = [constraints,...
          cone(obj.symbolic_vars(obj.cones(j).index(2:end)), obj.symbolic_vars(obj.cones(j).index(1)))];
      end
      for j = 1:length(obj.polycones)
        constraints = [constraints,...
          polycone(obj.symbolic_vars(obj.polycones(j).index(2:end)), obj.symbolic_vars(obj.polycones(j).index(1)), obj.polycones(j).N)];
      end

      diagnostics = optimize(constraints, objective, params);
      ok = diagnostics.problem == 0 || diagnostics.problem == -1;
      if ~ok
        error('Drake:MixedIntegerConvexProgram:InfeasibleProblem', 'The mixed-integer problem is infeasible.');
      end
      objval = double(objective);
      solvertime = diagnostics.solvertime;
      obj = obj.extractResult(double(obj.symbolic_vars));
    end
    
    function [obj,solvertime,objval] = solveMosek(obj)
      checkDependency('mosek');
      prob = obj.getMosekModel();
      params = struct();
      start_time = clock();
      [r,res] = mosekopt('minimize',prob,params);
      end_time = clock();
      solvertime = etime(end_time,start_time);
      if(isfield(prob,'ints') && ~isempty(prob.ints))
        if(~strcmp(res.sol.int.prosta,'PRIMAL_FEASIBLE'))
          error('Drake:MixedIntegerConvexProgram:INFEASIBLE','The mixed-integer problem is not feasible');
        end
        obj = obj.extractResult(res.sol.int.xx);
        objval = res.sol.int.pobjval;
      else
        if(strcmp(res.sol.itr.prosta,'PRIMAL_INFEASIBLE'))
          error('Drake:MixedIntegerConvexProgram:PrimalInfeasible','The problem is primal infeasible');
        elseif(strcmp(res.sol.itr.prosta,'DUAL_INFEASIBLE'))
          error('Drake:MixedIntegerConvexProgram:DualInfeasible','The problem is dual infeasible');
        elseif(strcmp(res.sol.itr.prosta,'PRIMAL_AND_DUAL_INFEASIBLE'))
          error('Drake:MixedIntegerConvexProgram:PrimalDualInfeasible','The problem is primal and dual infeasible');
        elseif(strcmp(res.sol.itr.prosta,'UNKNOWN'))
          warning('Drake:MixedIntegerConvexProgram:Unknown','The problem solution is unknown');
        end
        obj = obj.extractResult(res.sol.itr.xx);
        objval = res.sol.itr.pobjval;
      end
    end
    
    function prob = getMosekModel(obj)
      prob.c = obj.c;
      [prob.qosubi,prob.qosubj,prob.qoval] = find(2*obj.Q);
      lower_idx = prob.qosubi>=prob.qosubj;
      prob.qosubi = prob.qosubi(lower_idx);
      prob.qosubj = prob.qosubj(lower_idx);
      prob.qoval = prob.qoval(lower_idx);
      prob.a = sparse([obj.A;obj.Aeq]);
      prob.blc = [-inf(size(obj.b));obj.beq];
      prob.buc = [obj.b;obj.beq];
      var_names = fieldnames(obj.vars);
      prob.blx = zeros(obj.nv,1);
      prob.bux = zeros(obj.nv,1);
      integer_idx = [];
      prob.blx = -inf(obj.nv,1);
      prob.bux = inf(obj.nv,1);
      for j = 1:length(var_names)
        name = var_names{j};
        i = reshape(obj.vars.(name).i,[],1);
        prob.blx(i) = obj.vars.(name).lb(:);
        prob.bux(i) = obj.vars.(name).ub(:);
        if(obj.vars.(name).type == 'B' || obj.vars.(name).type == 'I')
          integer_idx = [integer_idx reshape(obj.vars.(name).i,1,[])];
        end
      end
      if(~isempty(integer_idx))
        prob.ints.sub = integer_idx;
      end
      if(~isempty(obj.cones))
        [r,res] = mosekopt('symbcon');
        prob.cones.type = res.symbcon.MSK_CT_QUAD*ones(1,length(obj.cones));
        prob.cones.sub = vertcat(obj.cones.index)';
        prob.cones.subptr = cumsum(cellfun(@(x) length(x),{obj.cones.index}));
        prob.cones.subptr = [1 prob.cones.subptr(1:end-1)+1];
      end
    end
    %设置求解器：gurobi还是mosek
    function obj = setSolver(obj,solver)
      if(strcmpi(solver,'gurobi'))
        checkDependency('gurobi');
      elseif(strcmpi(solver,'mosek'))
        checkDependency('mosek');
      else
        error('Drake:MixedIntegerConvexProgram:UnsupportedSolver','Solver not supported yet');
      end
      obj.solver = solver;
    end
  end
end