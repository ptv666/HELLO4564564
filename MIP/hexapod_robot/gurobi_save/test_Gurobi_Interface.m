%test Gurobi接口
%% LP问题
G = Gurobi_Interface();
G.addVariable('x','C',[1,3],0,Inf);
A = [1 1 0;0 1 1];
b = [1;1];
G.addLinearConstraints_unequal(A,b,'con');
c = -1*[1 2 3]';    %求解最大值，需要手动加一个负号
G.addtLinearCost(c);
G.Gurobi_solve();

%% MIP问题
G = Gurobi_Interface();
G.addVariable('x','B',[1,3],0,1);
A = [1 2 3;-1 -1 0];
b = [4;-1];
G.addLinearConstraints_unequal(A,b,'hahah');
c = -1*[1 1 2]';    %求解最大值，需要手动加一个负号
G.addtLinearCost(c);
G.Gurobi_solve();
%best objective为-3，因为加了一个负号所以是正确的
%% QP问题
G = Gurobi_Interface();
G.addVariable('x','B',[1,3],0,Inf);
A = -1*[1 2 3;1 1 0];
b = -1*[4;1];
G.addLinearConstraints_unequal(A,b,'yueshu');
c = [2 0 0]';
Q = [1 0.5 0;0.5 1 0.5;0 0.5 1];
G.addCost(Q,c,[]);
G.Gurobi_solve();
%把三个变量设置为'C'：Optimal objective 2.11111111e+00
%把三个变量设置为'B'：Best objective 3.000000000000e+00
%% QCP问题
G = Gurobi_Interface();
G.addVariable('x','C',[1,3],0,Inf);
Aeq = [1 1 1];
beq = 1;
G.addLinearConstraints_equal(Aeq,beq,'LPdengshi');
c = -1*[1 0 0]';
quadcon1.Qc = [1 0 0;0 1 0;0 0 -1];
quadcon1.q = zeros(3,1);quadcon1.rhs = 0.0;
quadcon2.Qc = [1 0 0;0 0 -1;0 0 0];
quadcon2.q = zeros(3,1);quadcon2.rhs = 0.0;
G.addQuadcon(quadcon1);
G.addQuadcon(quadcon2);
G.addtLinearCost(c);
G.Gurobi_solve();
%Optimal objective -3.26992304e-01

%% billinear问题
G = Gurobi_Interface();
G.addVariable('x','C',[1,3],0,Inf);
A = [1 1 1];
b = 10;
G.addLinearConstraints_unequal(A,b,'LPbudengshi');
c = -1*[1 0 0]';
quadcon1.Qc = [0 1 0;0 0 0;0 0 0];
quadcon1.q = zeros(3,1);quadcon1.rhs = 2.0;

%把等式转换为两个不等式
quadcon2.Qc = [0 0 1;0 0 1;0 0 0];
quadcon2.q = zeros(3,1);quadcon2.rhs = 1.0;
quadcon3.Qc = -1*[0 0 1;0 0 1;0 0 0];
quadcon3.q = zeros(3,1);quadcon3.rhs = -1*1.0;

G.addQuadcon(quadcon1);
G.addQuadcon(quadcon2);
G.addQuadcon(quadcon3);
G.addtLinearCost(c);

params.NonConvex = 2;
G.Gurobi_solve(params);
%Best objective -9.898979482305e+00, best bound -9.899367715053e+00, gap 0.0039%

%% sin cos约束
G = Gurobi_Interface();
G.addVariable('x','C',[1,2],0,pi/2);
G.addVariable('y','C',[1,2],0,1);
%x1+x2=pi/2
Aeq = [1 -1 0 0];
beq = [0];
G.addLinearConstraints_equal(Aeq,beq,'LPds');
%f1=sin(x1),f2=sin(x2)
x_index = G.vars.x.Index(1,1);
y_index = G.vars.y.Index(1,1);
G.addSinConstraint(x_index,y_index);
x_index = G.vars.x.Index(1,2);
y_index = G.vars.y.Index(1,2);
G.addCosConstraint(x_index,y_index);
%最大化sin(x1) + cos(x2)
c = -1*[0 0 1 1]'; 
G.addtLinearCost(c);
G.Gurobi_solve();

%%  测试Gurobi的高斯函数的效果
G = Gurobi_Interface();
sigma = 0.05; inv_sigma = 1/sigma;
G.addVariable('x','C',[1,1],-1,1);
G.addVariable('y','C',[1,1],-1,1);
G.addVariable('X','C',[1,1],-1,1);
G.addVariable('Y','C',[1,1],-1,1);
G.addVariable('Temp','C',[1,1],0,Inf);
G.addVariable('Z','C',[1,1],0,1);
%X = x^2/sigma
x_index = G.vars.x.Index(1,1);y_index = G.vars.y.Index(1,1);
X_index = G.vars.X.Index(1,1);Y_index = G.vars.Y.Index(1,1);Z_index = G.vars.Z.Index(1,1);
Temp_index = G.vars.Temp.Index(1,1);

G.addPolyConstraint(x_index,X_index,[-0.5*inv_sigma 0 0])
G.addPolyConstraint(y_index,Y_index,[-0.5*inv_sigma 0 0])

Aeq = zeros(1,6);
beq = zeros(1,1);
Aeq(1,X_index) = 1;Aeq(1,Y_index) = 1;Aeq(1,Temp_index) = -1;
beq(1,1) = 0;
G.addLinearConstraints_equal(Aeq,beq,'eq')
G.addExpConstraint(Temp_index,Z_index)
c = zeros(6,1);
c(Z_index) = 1;
G.addtLinearCost(c);

model = G.getGurobiModel();
gurobi_write(model,'1.lp')

G.Gurobi_solve();