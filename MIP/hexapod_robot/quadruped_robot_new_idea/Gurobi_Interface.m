classdef Gurobi_Interface < handle
%该类定义与Gurobi求解器的接口，使得添加变量、约束、目标优化对象更加的简单
%Gurobi优化目标形式为：见refman P641
%最小化:x'Qx + c'x + alpha
%Ax < b                     线性约束lp问题
%Ax = b                     线性约束lp问题
%lowBound<= x <= upBound    变量上下界约束
%some x_j integral          整数约束，MIP问题
%x'Qcx + q'x <= rhs         二次约束，QCP问题

  properties
    %变量结构体，结构体每一个字段代表着一个变量集合
    vars = struct();

    %变量的数目
    num_vars = 0;

    %优化目标的二次项，Q需要是稀疏矩阵形式，最后再转换
    Q = zeros(0, 0);
    
    %优化目标的线性项c'x
    c = zeros(0, 0);
    
    %优化目标的常数项
    obj_const = 0;

    %lp不等式约束，Ax<b，A需要是稀疏矩阵，最后再转换
    A = zeros(0, 0);
    b = zeros(0, 1);
    LP_unequal_names = struct('names',{},'number',{});
    
    %lp等式约束，Aeq*x=beq
    Aeq = zeros(0, 0);
    beq = zeros(0, 1);
    LP_equal_names = struct('names',{},'number',{})
    
    %二次约束形式:x'Qc x + q' x <= rhs，Qc为二次约束矩阵，q为一次约束向量
    %不同的二次约束需要定义不同的Qc矩阵，所以使用结构体数据类型
    %Qc为sparse矩阵，q为正常数组，rhs也是正常数组
    quadcon = struct('Qc', {}, 'q', {}, 'rhs', {});
    
    %结果存储
    x_sol;
    
    %创建三角函数约束存储对象:genconsin
    sin_con = [];   %     obj.sin_con = struct('xvar', {}, 'yvar', {});
    cos_con = [];   %     obj.cos_con = struct('xvar', {}, 'yvar', {});
    
  end
  methods
    
    %构造函数，没啥要构造的，全都在properties中定义好了
    function obj = Gurobi_Interface()
        warning('注意求最大值还是最小值。Class_Gurobi_Interface_Line_52');
        obj.x_sol = [];
    end
    
    %添加变量，输入：变量名，数据类型，上限，下限，初始值(可选)
    function obj = addVariable(obj, name, type_input, size_input, lb, ub, start_input)
      %name：变量名不能与已有变量名重复
      %type：有三种类型：'C','I','B'分别代表连续型，整数型，二进制型变量，变量矩阵所有的变量都是这个数据类型
      %size：1×2的数组，代表了变量矩阵的形状
      %lb：变量的下限，设置时需要与变量矩阵形状一致，或者为一个常数（表示大家都一样），不能传入空数组
      %ub：变量的上限，设置时需要与变量矩阵形状一致，或者为一个常数（表示大家都一样），不能传入空数组
      %start：变量的初值，用于Gurobi求解时的初值选择
      %
      %设置变量是提供Index，用于快捷的索引变量矩阵中的元素
            
      %判断变量名是否已经在vars中了
      if isfield(obj.vars, name)
        error('变量不能重名。Class_Gurobi_Interface_Line_69');
      end
      if (size(size_input,1)~=1) || (size(size_input,2)~=2)
        error('变量的大小需要是1×2的，例如[2 3]。Class_Gurobi_Interface_Line_72');
      end
      
      %添加新变量，设置name，type，size，Index索引
      obj.vars.(name) = struct();
      obj.vars.(name).type = type_input;
      obj.vars.(name).size = size_input;
      
      %vars.(name).Index的索引值从num_vars + 1开始，大小和vars本身大小一致
      temp = obj.num_vars + (1:size_input(1)*size_input(2));
      obj.vars.(name).Index = reshape(temp, obj.vars.(name).size);   %为新加入的变量设置索引值
      
      %设置lb和ub上下限
      %如果是标量按照变量矩阵的形状复制给每一个变量约束上
      if isscalar(lb)
        lb = repmat(lb, obj.vars.(name).size);
      end
      if isscalar(ub)
        ub = repmat(ub, obj.vars.(name).size);
      end
      obj.vars.(name).lb = lb;
      obj.vars.(name).ub = ub;
      
      %设置变量初值
      if nargin < 7
        start_input = [];
      end
     
      %允许只对部分变量赋初值
      if size(start_input, 1) > obj.vars.(name).size(1)
        error('初值行数量大于变量行数。Class_Gurobi_Interface_Line_102');
      end
      if size(start_input, 2) > obj.vars.(name).size(2)
        error('初值列数量大于变量列数。Class_Gurobi_Interface_Line_105');
      end
      
      %没有初值的变量初值设置nan也是可行的，见refman P658
      obj.vars.(name).start = nan(obj.vars.(name).size);
      obj.vars.(name).start(1:size(start_input, 1), 1:size(start_input, 2)) = start_input;
      
      %确定新增变量的个数
      num_new_vars = prod(obj.vars.(name).size);
      %更新目前变量的数目
      obj.num_vars = obj.num_vars + num_new_vars;
        
      %扩大优化目标线性向量的维度c'x
      obj.c = [obj.c; zeros(num_new_vars, 1)];
      %扩大优化目标二次项的维度x'Qx，行与列都变大了
      obj.Q = [obj.Q, zeros(size(obj.Q, 1), num_new_vars);
               zeros(num_new_vars, num_new_vars + size(obj.Q, 2))];
      %扩大lp约束矩阵的维度
      obj.A = [obj.A, zeros(size(obj.A, 1), num_new_vars)];
      obj.Aeq = [obj.Aeq, zeros(size(obj.Aeq, 1), num_new_vars)];
      
      %二次约束的个数不建议超过10个，超过十个建议添加“辅助变量”
      if length(obj.quadcon) > 10
        warning('二次约束的个数太多，可能会无解，请尝试引入中间变量来减小二次约束个数。Class_Gurobi_Interface_Line_148');
      end
      %扩大二次约束的维度
      for i = 1:length(obj.quadcon)
        obj.quadcon(i).Qc = [obj.quadcon(i).Qc, zeros(size(obj.quadcon(i).Qc, 1), num_new_vars);
          zeros(num_new_vars, num_new_vars + size(obj.quadcon(i).Qc, 2))];
        obj.quadcon(i).q = [obj.quadcon(i).q; zeros(num_new_vars, 1)];
      end
    end %end of function addVariable
        
    %添加线性约束，也就是约束矩阵A和右侧向量b维数扩大了，线性约束这样做是可以的
    function obj = addLinearConstraints_unequal(obj, A, b, name)
      if nargin<4
          error('请为线性约束命名')
      end
      obj.A = [obj.A; A];
      obj.b = [obj.b; b];
      
      if size(obj.LP_unequal_names,1)~=0
          if strcmp(name,obj.LP_unequal_names(end).names)
            obj.LP_unequal_names(end).number = obj.LP_unequal_names(end).number + size(A,1);
            return;
          end
      end
      temp = struct('names',name,'number',size(A,1));   %根据A的行数来定义
      obj.LP_unequal_names = [obj.LP_unequal_names;temp];
    end
    
    %添加线性约束，也就是约束矩阵A和右侧向量b维数扩大了，线性约束这样做是可以的
    function obj = addLinearConstraints_equal(obj, Aeq, beq, name)
      if nargin<4
          error('请为线性约束命名')
      end
      obj.Aeq = [obj.Aeq; Aeq];
      obj.beq = [obj.beq; beq];
      
      if size(obj.LP_equal_names,1)~=0
          if strcmp(name,obj.LP_equal_names(end).names)
              obj.LP_equal_names(end).number = obj.LP_equal_names(end).number + size(Aeq,1);
              return;
          end
      end
      temp = struct('names',name,'number',size(Aeq,1));
      obj.LP_equal_names = [obj.LP_equal_names;temp];
    end
    
    %添加二次约束，结构体数组中每一个子结构体代表了一个二次约束
    function obj = addQuadcon(obj, quadcon)
      obj.quadcon = [obj.quadcon, quadcon]; %拼接结构体，前提是结构体字段都要相同
    end
    
     %添加y=sin(x)约束
    function obj = addSinConstraint(obj,x_index,y_index)
        %x_index1为x变量矩阵中的索引，y_index为y变量矩阵中的索引
      if (numel(x_index)~=1)|| (numel(y_index)~=1)
          error('添加y=sin(x)约束时，x和y的变量数目要相同.Class_Gurobi_Interface_Line_155');
      end
      new_sin_con = struct('xvar',x_index, 'yvar', y_index);
      obj.sin_con = [obj.sin_con,new_sin_con];
    end

    %添加y=cos(x)约束
    function obj = addCosConstraint(obj,x_index,y_index)
        %x_index1为x变量矩阵中的索引，y_index为y变量矩阵中的索引
      if (numel(x_index)~=1)|| (numel(y_index)~=1)
          error('添加y=cos(x)约束时，x和y的变量数目要相同.Class_Gurobi_Interface_Line_165');
      end
      new_cos_con = struct('xvar',x_index, 'yvar', y_index);
      obj.cos_con = [obj.cos_con,new_cos_con];
    end
    
    %增加线性目标值c'x，注意c为列向量
    function obj = addtLinearCost(obj, c)
      obj.c = obj.c + c;
    end
        
    %设置一般目标值，含有二次目标项QP问题和LP问题，见该文件第二行
    function obj = addCost(obj, Q, c, alpha)
      if ~isempty(Q)
        obj.Q = obj.Q + sparse(Q);
      end
      if ~isempty(c)
        obj.c = obj.c + c;
      end
      if ~isempty(alpha)
        obj.obj_const = obj.obj_const + alpha;
      end
    end
    
    %转换为Gurobi模型
    function model = getGurobiModel(obj)
      var_names = fieldnames(obj.vars);     %获取vars中存放了多少个变量矩阵
      model = struct();                     %Gurobi求解的数学模型
      
      %转换LP约束!!!!!!!!!!!!!!!!!!!!!!!!!!****************------------------
      model.A = sparse([obj.A; obj.Aeq]);   %线性约束矩阵A需要是sparse形式的
      model.rhs = [obj.b; obj.beq];         %线性约束的右侧值是一维向量，不需要是sparse形式的
      model.sense = [repmat('<', size(obj.A, 1), 1); repmat('=', size(obj.Aeq, 1), 1)];
      model.constrnames = cell(size(model.A,1),1);
      constrnames_count = 0;
      for i=1:size(obj.LP_unequal_names,1)
          for j = 1:obj.LP_unequal_names(i,1).number
              constrnames_count = constrnames_count + 1;
              temp_name = [obj.LP_unequal_names(i,1).names, '_', num2str(j)];
              model.constrnames{constrnames_count} = temp_name;
          end
      end
      for i=1:size(obj.LP_equal_names,1)
          for j = 1:obj.LP_equal_names(i,1).number
              constrnames_count = constrnames_count + 1;
              temp_name = [obj.LP_equal_names(i,1).names, '_', num2str(j)];
              model.constrnames{constrnames_count} = temp_name;
          end
      end
      
      %设置quadcon约束
      for i=1:length(obj.quadcon)
          obj.quadcon(i).Qc = sparse(obj.quadcon(i).Qc);  %转换为系数矩阵
      end
      if ~isempty(obj.quadcon)
        model.quadcon = obj.quadcon;
      end
      model.objcon = obj.obj_const;
      
      %设置sin和cos约束
      if ~isempty(obj.sin_con)  %如果为空添加sin和cos约束会报错
          model.genconsin = obj.sin_con;
      end
      if ~isempty(obj.sin_con)
          model.genconcos = obj.cos_con;
      end
      
      %设置优化目标
      model.obj = obj.c;            %一次目标值
      model.Q = sparse(obj.Q);      %二次目标值
      
      %设置Gurobi变量
      model.varnames = cell(obj.num_vars,1);        %变量名
      model.vtype = repmat('C', obj.num_vars, 1);   %变量类型
      model.start = nan(obj.num_vars, 1);           %变量迭代初值，若不定义Gurobi自己寻找
      model.lb = -inf(obj.num_vars, 1);             %变量下限
      model.ub = inf(obj.num_vars, 1);              %变量上限
      for i = 1:length(var_names)   %按照变量不同的“种类”分别进行操作
        name = var_names{i};
        index_vars = reshape(obj.vars.(name).Index,[],1);
        for j=index_vars(1):index_vars(end)
            new_name = j - index_vars(1) + 1;
            new_name = [name,'_',num2str(new_name)];    %变量命名规则：name_1，name_2，name_3...
            model.varnames{j} = new_name;
        end
        model.vtype(index_vars) = obj.vars.(name).type;
        model.lb(index_vars) = reshape(obj.vars.(name).lb,[],1);
        model.ub(index_vars) = reshape(obj.vars.(name).ub,[],1);
        model.start(index_vars) = reshape(obj.vars.(name).start,[],1);
      end
    end
    
    %使用Gurobi求解
    function [obj, solvertime, objval] = Gurobi_solve(obj, params,write_flag)
      if nargin < 2
        params = struct();
        write_flag = 0;
      elseif nargin<3
        write_flag = 0;
      end
      params.('outputflag') = 1;        %开启/关闭控制台输出 1/0
      model = obj.getGurobiModel();     %转换为Gurobi模型
      result = gurobi(model, params);   %调用Gurobi求解
      ok = ~( strcmp(result.status, 'INFEASIBLE') || strcmp(result.status, 'INF_OR_UNBD') );  %模型是否是可求解的，status返回INFEASIBLE与INF_OR_UNBD均是不可求解的
      if ~ok
        error('问题不可解。Class_Gurobi_Interface_Line_247');
      end
      objval = result.objval;       %返回优化目标值
      solvertime = result.runtime;  %返回Gurobi优化运行时间
      obj = obj.extractResult(result.x);    %提取决策变量的值到Gurobi接口对象中
      
      %生成lp文件并 保存在当前工作目录
      if write_flag
        file_name = datestr(now,'yyyy-mmm-dd--HH-MM-SS');
        file_name = [file_name,'.lp'];
        gurobi_write(model,file_name);
      end
    end
    
    %传递 解 结果给接口对象
    function obj = extractResult(obj, x)
      obj.x_sol = x;    %把所有变量的值传给了x_sol中
      var_names = fieldnames(obj.vars); %获取vars中存放了多少个变量矩阵
      
      %提取每个变量的值到value属性中
      for i = 1:length(var_names)
        name = var_names{i};
        index_vars = reshape(obj.vars.(name).Index,[],1);   %把变量索引转换为1列
        if obj.vars.(name).type == 'I' 
          obj.vars.(name).value = reshape(round(x(index_vars)), obj.vars.(name).size);   %把整数变量值进行圆整
        elseif obj.vars.(name).type == 'B'
          obj.vars.(name).value = reshape(logical(round(x(index_vars))), obj.vars.(name).size);  %把逻辑变量进行圆整
        else
          obj.vars.(name).value = reshape(x(index_vars), obj.vars.(name).size);  %其余数据类型的变量就不用再圆整了
        end
      end
    end
    
  end
end