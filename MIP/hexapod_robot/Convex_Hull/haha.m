function c = haha(a,b)
%计算两个凸包a,b的闵可夫斯基和
%输入：凸包a,b的极点序列，按行排列
%输出：cc=a+b（闵式和）
    if size(a,2)~=size(b,2)
        error('输入的两个凸包不是同一个维度的')
    end
    [La,n] = size(a);
    Lb = size(b,1);
    %定义Gurobi求解对象
    Problem = Gurobi_Interface();
    Problem.addVariable('alpha','C',[La 1],0,1)
    Problem.addVariable('beta','C',[Lb 1],0,1)
    %第一个约束和极点有关，需要动态计算改变
    Aeq = zeros(n,Problem.num_vars);
    beq = zeros(n,1);
    Aeq(1:n,Problem.vars.alpha.Index) = a';
    Aeq(1:n,Problem.vars.beta.Index) = b';
    beq(end-1:end,1) = 1;
    Problem.addLinearConstraints_equal(Aeq,beq,'线性组合约束');
    %对系数和的约束，不变的约束
    Aeq = zeros(2,Problem.num_vars);
    beq = zeros(2,1);
    Aeq(1,Problem.vars.alpha.Index) = 1;
    Aeq(2,Problem.vars.beta.Index) = 1;
    beq(:,1) = 1;
    Problem.addLinearConstraints_equal(Aeq,beq,'组合系数之和为1的约束');
    %添加目标值，注意目标是求最大值，但是求解器默认是求最小值
    cc = -100*ones(Problem.num_vars,1);
    alpha = -1*2;
    Problem.addCost([],cc,alpha);
    %转换为Gurobi模型
    model = Problem.getGurobiModel();
    gurobi_write(model,'1.lp');
    %开始迭代一个一个求解计算
    b_start_0 = Problem.vars.alpha.Index(end);
    aPlusb = [];
    cc = -100*zeros(La+Lb,n+1);
    %Gurobi参数设置
    params.OutputFlag = 0;
    count = 0;
    for i=1:La
        for j=1:Lb
            count = count + 1;
            aPlusb = a(i,:) + b(j,:);
            cc(count,1:n) = aPlusb;
            %改变线性组合约束的右边值rhs
            model.rhs(1:n,1) = aPlusb;
            model.obj(:,1) = 0;
            model.obj(i,1) = 1;
            model.obj(b_start_0 + j,1) = 1; 
            result = gurobi(model,params);
            if strcmp(result.status,'OPTIMAL')
%                 if result.objval>-1.99
%                     cc(La*(i-1)+j,end) = 1;
%                 end
                cc(count,end) = -result.objval;
            end
        end
    end
    c = [];
    for i=1:size(cc,1)
        if cc(i,3)==0
            c(end+1,:) = cc(i,1:2);
        end
    end
end