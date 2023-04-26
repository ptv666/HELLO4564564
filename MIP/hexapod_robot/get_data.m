%% 获取一些有用数据
clear,clc,
close all
load('E:\bc\matlab\motion_planning\hexapod_robot\离散点\直线决策前进_83点.mat')

human_robot = humanoid_robot([2,0,0]',XY); 
human_robot.set_Goal_cost();
human_robot.set_Run_cost();
human_robot.set_Leg_Workspace_constrain();
a = length(human_robot.A) + length(human_robot.Aeq);
tic
human_robot.set_discrete_points()
toc
b = length(human_robot.A) + length(human_robot.Aeq);
b-a
%% 添加区域约束
%% 加入region
% region_center = [0,0;...
%     0.4,0.25; 0.8,-0.25; 1.2,0.25; 1.6,-0.25; 2.8,0.25; 3.2,-0.25; 3.6,0.25;...
%     4,0;...
%     2,1.25];
region_center = [0,0;...
    0.4,0.25; 0.8,-0.25; 1.2,0.25; 1.6,-0.25; 2.0,0.25; 2.4,-0.25; 2.8,0.25; 3.2,-0.25; 3.6,0.25;...
    4,0;...
    2,1.25];
length_x = ones(size(region_center,1),1) * 0.3;
length_y = ones(size(region_center,1),1) * 0.2;
length_x(1) = 0.4;  length_x(end-1) = 0.4;  length_x(end) = 4.5;
length_y(1) = 0.75; length_y(end-1) = 0.75; length_y(end) = 0.19;
rectangle_region_con=creat_rectangle_region_con(region_center,length_x,length_y);
human_robot.set_foot_region(rectangle_region_con,[1 1])
b = length(human_robot.A) + length(human_robot.Aeq);
b-a
%% 
[~,solve_time,obj_val] = human_robot.Gurobi_solve()
%% 
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

