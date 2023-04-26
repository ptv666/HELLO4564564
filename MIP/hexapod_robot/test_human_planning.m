%% test_human_planning  %在E:\bc\matlab\motion_planning\hexapod_robot目录下运行才可以
clear,clc,
close all
% load('.\离散点\直线前进_均匀_纵向冗余_126点.mat')
load('.\离散点\直线决策前进_241点.mat')  
human_robot = humanoid_robot([4,0,0]',XY); 
human_robot.set_Goal_cost();
% human_robot.set_Run_cost();
human_robot.set_Leg_Workspace_constrain();
% human_robot.set_Trim_flag();
tic
human_robot.set_discrete_points_con();
toc

%% 添加区域约束作为对照实验
region_center = [1,0.25; 1,-0.25];
length_x = [2; 2];
length_y = [0.2; 0.2];
rectangle_region_con=creat_rectangle_region_con(region_center,length_x,length_y);
human_robot.set_foot_region(rectangle_region_con,[1 1])

%% 施加软约束
%施加软约束并不是很成功，理论看似可行但是实际收敛很慢
% tic
% human_robot.set_soft_con();
% toc

%% 先不加入离散落足点，可以先使用连续的region，规划出结果，再以这个结果作为start进行精确求解
% [~,solve_time,obj_val] = human_robot.Gurobi_solve();
clear,clc
load('初始解.mat')
vars = human_robot.vars;
tic
human_robot.set_discrete_points_con();
toc
names = fieldnames(vars);
% for i=1:size(names,1)     %156个变量
%     human_robot.vars.(names{i,1}).start = vars.(names{i,1}).value;
% end
for j=1:human_robot.N       %5412个变量
    distance = [];
    for i=1:length(human_robot.discrete_points)
        d = norm(vars.Feet_State_List.value(1:2,j) - human_robot.discrete_points(i,1:2));
        distance(end+1) = d;
    end
    [~,k] = min(distance);
    human_robot.vars.Y.start(:,j) = 0;
    human_robot.vars.Y.start(k,j) = 1;
%     human_robot.vars.Feet_State_List.start(1,j) = human_robot.discrete_points(k,1);
%     human_robot.vars.Feet_State_List.start(2,j) = human_robot.discrete_points(k,2);
end
%% 精确求解
params = [];
% params.StartNumber = -1;
%误差：All constraints must be satisfied to a tolerance of FeasibilityTol. Tightening this tolerance can produce smaller
%constraint violations, but for numerically challenging models it can sometimes lead to much larger iteration counts.
% params.FeasibilityTol = 1e-2;   %连续变量的误差允许限度
% params.IntFeasTol = 1e-1;       %整数变量的误差允许限度
% params.StartNodeLimit = -2;
% param.TimeLimit = 45;
[~,solve_time,obj_val] = human_robot.Gurobi_solve(params)

%% 获取数学模型对应的lp文件
model = human_robot.getGurobiModel();
gurobi_write(model,'3.lp');
gurobi(model)

%% 动画
figure
hold on
for i=1:length(human_robot.discrete_points)
    plot(human_robot.discrete_points(i,1),human_robot.discrete_points(i,2),'ro')
end

pause_time = 0.1;
hold on
i = 1;
quiver(human_robot.vars.Feet_State_List.value(1,i),human_robot.vars.Feet_State_List.value(2,i),...
       cos(human_robot.vars.Feet_State_List.value(3,i)),sin(human_robot.vars.Feet_State_List.value(3,i)),0.2,...
       'o','MarkerSize',5,'MarkerEdgeColor',[0.9 0.7 0.1],'MarkerFaceColor',[0.9 0.7 0.1],'Color',[0.9 0.7 0.1],'ShowArrowHead','On')   %左脚
quiver(human_robot.vars.Feet_State_List.value(1,i+1),human_robot.vars.Feet_State_List.value(2,i+1),...
       cos(human_robot.vars.Feet_State_List.value(3,i+1)),sin(human_robot.vars.Feet_State_List.value(3,i+1)),0.2,...
       'o','MarkerSize',5,'MarkerEdgeColor',[0.3 0.9 0.2],'MarkerFaceColor',[0.3 0.9 0.2],'Color',[0.3 0.9 0.2],'ShowArrowHead','On')   %右脚
scatter((human_robot.vars.Feet_State_List.value(1,i) + human_robot.vars.Feet_State_List.value(1,i+1))/2,...
        (human_robot.vars.Feet_State_List.value(2,i) + human_robot.vars.Feet_State_List.value(2,i+1))/2,...
        30,'MarkerEdgeColor',[1 0 0],'MarkerFaceColor',[1 0 0])   %质心
hold on
for i = 3:2:human_robot.vars.Feet_State_List.size(2)
    %左脚
    quiver(human_robot.vars.Feet_State_List.value(1,i),human_robot.vars.Feet_State_List.value(2,i),...
       cos(human_robot.vars.Feet_State_List.value(3,i)),sin(human_robot.vars.Feet_State_List.value(3,i)),0.2,...
       'o','MarkerSize',5,'MarkerEdgeColor',[0.9 0.7 0.1],'MarkerFaceColor',[0.9 0.7 0.1],'Color',[0.9 0.7 0.1],'ShowArrowHead','On')
    %右脚
    pause(pause_time);
    quiver(human_robot.vars.Feet_State_List.value(1,i+1),human_robot.vars.Feet_State_List.value(2,i+1),...
       cos(human_robot.vars.Feet_State_List.value(3,i+1)),sin(human_robot.vars.Feet_State_List.value(3,i+1)),0.2,...
       'o','MarkerSize',5,'MarkerEdgeColor',[0.3 0.9 0.2],'MarkerFaceColor',[0.3 0.9 0.2],'Color',[0.3 0.9 0.2],'ShowArrowHead','On')
    drawnow
    %质心 
    pause(pause_time);
     scatter((human_robot.vars.Feet_State_List.value(1,i) + human_robot.vars.Feet_State_List.value(1,i+1))/2,...
        (human_robot.vars.Feet_State_List.value(2,i) + human_robot.vars.Feet_State_List.value(2,i+1))/2,...
        30,'MarkerEdgeColor',[1 0 0],'MarkerFaceColor',[1 0 0])   %质心
    drawnow
    pause(pause_time);
end
legend('左脚','右脚','质心')
%在中间加入text注释
text((human_robot.left_foot_goal(1) + human_robot.right_foot_goal(1))/2.5,(human_robot.left_foot_goal(2) + human_robot.right_foot_goal(2))/4,[num2str(human_robot.N),'步'])
if isfield(human_robot.vars,'Trim')
    text((human_robot.left_foot_goal(1) + human_robot.right_foot_goal(1))/6,(human_robot.left_foot_goal(2) + human_robot.right_foot_goal(2))/4,...
                ['sum(Trim) = ',num2str(sum(human_robot.vars.Trim.value))])
end
hold off
axis padded


%% 用零目标值获得初始可行解_高斯模型中有使用到它
%先使用零目标获得可行解作为一个初始解，再添加目标值进行优化，从而降低Gurobi寻找
params = [];
% params.NonConvex = 2;
% Q = human_robot.Q;
% c = human_robot.c;
% obj_const = human_robot.obj_const;
% human_robot.Q = zeros(size(human_robot.Q));
% human_robot.c = zeros(size(human_robot.c));
% human_robot.obj_const = 0;
% [~,solve_time,obj_val] = human_robot.Gurobi_solve()
% human_robot.Q = Q;
% human_robot.c = c;
% human_robot.obj_const = obj_const;
[~,solve_time,obj_val] = human_robot.Gurobi_solve(params)


%% 绘制每步的步长
figure
hold on
plot_delta_XY(human_robot.vars.Feet_State_List.value)
hold off
pause(1);

%% 绘制region区域矩阵H
figure
hold

for i=1:human_robot.N
    for j=1:human_robot.vars.H.size(1)
        if human_robot.vars.H.value(j,i)~=0
            if mod(i,2)
                color = [0.9 0.7 0.1];
            else
                color = [0.3 0.9 0.2];
            end
            scatter(i,j,'MarkerEdgeColor',color,'MarkerFaceColor',color,'LineWidth',2.0);
        end
    end
end
for i=1:human_robot.vars.H.size(1)
    line([0,human_robot.N],[i,i])
end
axis padded
grid on
grid minor
title('H-region落足点的落足区域选择')
hold off