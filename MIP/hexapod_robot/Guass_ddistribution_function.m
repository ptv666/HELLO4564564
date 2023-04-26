%创建二维高斯分布数学函数
x0 = [0,2]; y0 = [0,2];
sigma_2 = 0.3338;
[X,Y] = meshgrid(-5:0.1:5,-5:0.1:5);

X1 = X-x0(1); Y1 = Y - y0(1);
Z1 = X1.^2 /sigma_2 + Y1.^2/sigma_2;
Z1 = exp(-0.5*Z1);

X2 = X-x0(2); Y2 = Y - y0(2);
Z2 = X2.^2 /sigma_2 + Y2.^2/sigma_2;
Z2 = exp(-0.5*Z2);
figure
hold on
surface(X,Y,Z1+Z2)
% surface(X,Y,Z2)
view(3)
hold off
%% 
fsurf(@(x,y) -10*exp(-0.5*(x.^2+y.^2)/0.01))

%%  测试Gurobi的高斯函数的效果
G = Gurobi_Interface();
discrete_points = [0.1193,0.2502;
                   0.1367,-0.2478;
                   0.3065,-0.2502;
                   0.3065,0.2494;
                   0.8467,0.2502;
                   0.8776,-0.2494;
                   0.9895,0.2494;
                   1.0338,-0.2494;
                   1.1457,0.2494];
a = norm(discrete_points(1,:)-discrete_points(2,:))/4;
a = a^2; b = log(100);
% sigma_2 = a/b;
sigma_2 = 0.0001;
LL = 9;

start_x = 0.3;
start_y = -0.2;
   
G.addVariable('x','C',[1,1],-2,2,start_x);
G.addVariable('y','C',[1,1],-2,2,start_y);

G.addVariable('delta_x','C',[1,LL],-2,2);
G.addVariable('delta_y','C',[1,LL],-2,2);

G.addVariable('X','C',[1,LL],-Inf,Inf);
G.addVariable('Y','C',[1,LL],-Inf,Inf);

G.addVariable('Temp','C',[1,LL],-Inf,Inf);
G.addVariable('Z','C',[1,LL],-Inf,Inf);

x_index = G.vars.x.Index; y_index = G.vars.y.Index;
delta_x_index = G.vars.delta_x.Index; delta_y_index = G.vars.delta_y.Index;
X_index = G.vars.X.Index; Y_index = G.vars.Y.Index;
Z_index = G.vars.Z.Index; Temp_index = G.vars.Temp.Index;
for j=1:LL
    Aeq = zeros(2,G.num_vars);
    beq = zeros(2,1);
    Aeq(1,x_index(1,1)) = 1; %计算delta_x
    Aeq(1,delta_x_index(1,j)) = -1;
    beq(1,1) = discrete_points(j,1);

    Aeq(2,y_index(1,1)) = 1; %计算delta_y
    Aeq(2,delta_y_index(1,j)) = -1;
    beq(2,1) = discrete_points(j,2);
    G.addLinearConstraints_equal(Aeq,beq,'comput_delta_x_y');
end

for i=1:LL
    G.addPolyConstraint(delta_x_index(1,i),X_index(1,i),[1 0 0])
    G.addPolyConstraint(delta_y_index(1,i),Y_index(1,i),[1 0 0])
end

for i=1:LL
    Aeq = zeros(1,G.num_vars);
    beq = zeros(1,1);
    Aeq(1,X_index(1,i)) = 1;Aeq(1,Y_index(1,i)) = 1;Aeq(1,Temp_index(1,i)) = sigma_2;
    beq(1,1) = 0;
    G.addLinearConstraints_equal(Aeq,beq,'eq')
end

for i=1:LL
    G.addExpConstraint(Temp_index(1,i),Z_index(1,i))
end


c = zeros(G.num_vars,1);
c(Z_index(1,:),1) = -1;
G.addtLinearCost(c);

model = G.getGurobiModel();
% model.setObjective = 0;
gurobi_write(model,'1.lp')

G.Gurobi_solve();

%%  测试Gurobi的高斯函数的效果_最初版本
G = Gurobi_Interface();
sigma = 0.1; inv_sigma = 1/sigma;
G.addVariable('x','C',[1,1],-10,1);
G.addVariable('y','C',[1,1],-1,1);
G.addVariable('X','C',[1,1],-Inf,Inf);
G.addVariable('Y','C',[1,1],-Inf,Inf);
G.addVariable('Temp','C',[1,1],-Inf,Inf);
G.addVariable('Z','C',[1,1],-Inf,Inf);
%X = x^2/sigma
x_index = G.vars.x.Index(1,1);y_index = G.vars.y.Index(1,1);
X_index = G.vars.X.Index(1,1);Y_index = G.vars.Y.Index(1,1);Z_index = G.vars.Z.Index(1,1);
Temp_index = G.vars.Temp.Index(1,1);

G.addPolyConstraint(x_index,X_index,[1 0 0])
G.addPolyConstraint(y_index,Y_index,[1 0 0])

Aeq = zeros(1,G.num_vars);
beq = zeros(1,1);
Aeq(1,X_index) = 1;Aeq(1,Y_index) = 1;Aeq(1,Temp_index) = 2*sigma;
beq(1,1) = 0;
G.addLinearConstraints_equal(Aeq,beq,'eq')
G.addExpConstraint(Temp_index,Z_index(1,1))
c = zeros(G.num_vars,1);
c(Z_index(1,1)) = -1;
G.addtLinearCost(c);

model = G.getGurobiModel();
gurobi_write(model,'1.lp')

G.Gurobi_solve();