%% 先测试只有一个接触点的力（倒四棱锥）约束
G = Gurobi_Interface();
G.addVariable('x','C',[1,3],-Inf,Inf);
%添加线性FFP约束
H = [   -4.71405          -0          -1
         -0    -4.71405          -1
    4.71405 1.67477e-15          -1
         -0     4.71405          -1
         -0          -0           1];
h = [
 0
 0
 0
 0
10];
G.addLinearConstraints_unequal(H,h,'LP');
%目标函数
% Q = -eye(3,3);
Q = zeros(3,3);
% c = [0 0 0]';
c = -[1 1 1]';
alpha = [];
G.addCost(Q,c,alpha);
model = G.getGurobiModel();
gurobi_write(model,'1.lp')
parameter.NonConvex = 2;
G.Gurobi_solve(parameter);

%% 三个接触点构成的Wrench大凸包
G = Gurobi_Interface();
G.addVariable('x','C',[1,6],-Inf,Inf);
H = [
     4.71405     -4.71405           -2            1           -1      4.71405
-6.76833e-14     -17.6621            1       4.7467    1.821e-14  8.44429e-14
-1.41043e-13  -1.3145e-13           -1 -2.04281e-14           -1    1.112e-13
-1.48603e-14 -1.45661e-14           -1           -1  4.52971e-15  5.68434e-15
    -4.71405      4.71405     -5.55694     -4.55694            1      4.71405
    -4.71405      4.71405           -2            1           -1     -4.71405
     -29.849           -0     -7.33192           -1           -0           -0
    -93.6029     -4.71405     -18.5931      3.26312            1     -4.71405
    -8.83105     -8.83105            1      6.62005      1.87335     -8.83105
    -6.20302     -7.51888       -2.595           -1     -1.31586      6.20302
    -4.71405     -4.71405     -2.31586     -1.31586           -1      4.71405
-2.76014e-14 -1.08776e-14 -6.42116e-15            1            1  9.63174e-15
     9.33301     -9.33301            1      2.97983      1.97983     -9.33301
     6.20302      4.88716     -2.31586           -1     -1.03672     -6.20302
     84.1748     -4.71405     -16.5931      1.26312            1     -4.71405
     4.71405      4.71405     -2.31586     -1.31586           -1     -4.71405
 -8.6596e-14      2.90894     -1.61708           -1  1.09889e-14  6.30287e-14
     4.71405      4.71405     -5.24108     -2.24108            1      4.71405
      29.849 -1.02044e-13     -7.33192           -1  2.85926e-14  2.65369e-14
          -0           -0           -0           -1           -1           -0
          -0           -0            1           -0            1           -0
           0            0            1            1            0            0];
h = [
           0
 -5.7502e-13
 3.63265e-14
-1.15818e-13
-7.99943e-14
 6.82121e-14
-4.84469e-14
-1.42109e-13
-5.83339e-13
-1.33704e-14
-1.08615e-14
-3.12639e-14
-1.12381e-12
-3.00401e-13
-2.65813e-12
-2.21287e-13
 1.08614e-15
-7.23232e-14
-8.90576e-13
          20
          20
          20];
G.addLinearConstraints_unequal(H,h,'LP');
%目标函数
% Q = -eye(6,6);
Q = zeros(6,6);
% c = [0 0 0]';
c = -[1 1 1 1 1 1]';
alpha = [];
G.addCost(Q,c,alpha);
model = G.getGurobiModel();
gurobi_write(model,'1.lp')
parameter = [];
% parameter.NonConvex = 2;
G.Gurobi_solve(parameter);
%% 一个简单的LP问题
G = Gurobi_Interface();
G.addVariable('x','C',[1,2],-Inf,Inf);
A = [-1,0;
    1,1;
    1,-1];
b = [-1;2;2];
G.addLinearConstraints_unequal(A,b,'LP');
Q = zeros(2,2);
% c = [0 0 0]';
c = [10 1]';
alpha = [];
G.addCost(Q,c,alpha);
model = G.getGurobiModel();
gurobi_write(model,'2.lp')
parameter = [];
G.Gurobi_solve(parameter);