%% test_human_planning_has_z
clear,clc
% close all
% human_robot = humanoid_robot_has_z([5,5,1.2,-pi/2]'); %水平向前行走，走5m，最大步长为0.4m，需要走13步
human_robot = humanoid_robot_has_z([4,0,0,0]');    %目标值不能超过上下限啊！
human_robot.set_Goal_cost();
human_robot.set_Run_cost();
human_robot.set_Leg_Workspace_constrain();
human_robot.set_Trim_flag();
%% 加入region
% region_center = [0,0;...
%     0.4,0.25; 0.8,-0.25; 1.2,0.25; 1.6,-0.25; 2.8,0.25; 3.2,-0.25; 3.6,0.25;...
%     4,0;...
%     2,1.25];
region_center = [0,0;...
    0.4,0.25; 0.8,-0.25; 1.2,0.25; 1.6,-0.25; 2.0,0.25; 2.4,-0.25; 2.8,0.25; 3.2,-0.25; 3.6,0.25;...
    4,0;...
    2,1.25];
region_center(:,3) = 0;
length_x = ones(size(region_center,1),1) * 0.3;
length_y = ones(size(region_center,1),1) * 0.1;
length_x(1) = 0.4;  length_x(end-1) = 0.4;  length_x(end) = 4.5;
length_y(1) = 0.75; length_y(end-1) = 0.75; length_y(end) = 0.19;
rectangle_region_con=creat_rectangle_region_con_has_z(region_center,length_x,length_y);

human_robot.set_foot_region(rectangle_region_con,[1 1])
%% Gurobi优化求解
params.NonConvex = 2;
% params.TimeLimit = 20;
[~,solve_time,obj_val] = human_robot.Gurobi_solve(params)
% [~,solve_time,obj_val] = human_robot.Gurobi_solve()

%% 动画
pause_time = 0.1;
% figure
hold on
% view(-30,45)
view(0,90)
grid on

hold on
for i = 1:2:human_robot.vars.Feet_State_List.size(2)
    %左脚
    quiver3(human_robot.vars.Feet_State_List.value(1,i),human_robot.vars.Feet_State_List.value(2,i),human_robot.vars.Feet_State_List.value(3,i),...
       cos(human_robot.vars.Feet_State_List.value(4,i)),sin(human_robot.vars.Feet_State_List.value(4,i)),0,0.2,...
       'o','MarkerSize',5,'MarkerEdgeColor',[0.9 0.7 0.1],'MarkerFaceColor',[0.9 0.7 0.1],'Color',[0.9 0.7 0.1],'ShowArrowHead','On',...
       'LineWidth',2.5,'ShowArrowHead',true,'AutoScaleFactor',2)   %左脚
    %右脚
    pause(pause_time);
    quiver3(human_robot.vars.Feet_State_List.value(1,i+1),human_robot.vars.Feet_State_List.value(2,i+1),human_robot.vars.Feet_State_List.value(3,i+1),...
       cos(human_robot.vars.Feet_State_List.value(4,i+1)),sin(human_robot.vars.Feet_State_List.value(4,i+1)),0,0.2,...
       'o','MarkerSize',5,'MarkerEdgeColor',[0.3 0.9 0.2],'MarkerFaceColor',[0.3 0.9 0.2],'Color',[0.3 0.9 0.2],'ShowArrowHead','On',...
       'LineWidth',2.5,'ShowArrowHead',true,'AutoScaleFactor',2)   %右脚
    drawnow
    %质心 
    pause(pause_time);
    scatter3((human_robot.vars.Feet_State_List.value(1,i) + human_robot.vars.Feet_State_List.value(1,i+1))/2,...
        (human_robot.vars.Feet_State_List.value(2,i) + human_robot.vars.Feet_State_List.value(2,i+1))/2,...
        (human_robot.vars.Feet_State_List.value(3,i) + human_robot.vars.Feet_State_List.value(3,i+1))/2,...
        30,'MarkerEdgeColor',[1 0 0],'MarkerFaceColor',[1 0 0])   %质心
    drawnow
    pause(pause_time);
end
legend('左脚','右脚','质心','Location','best')
%在中间加入text注释
text((human_robot.left_foot_goal(1) + human_robot.right_foot_goal(1))/2.5,(human_robot.left_foot_goal(2) + human_robot.right_foot_goal(2))/4,[num2str(human_robot.N),'步'])
if isfield(human_robot.vars,'Trim')
    text((human_robot.left_foot_goal(1) + human_robot.right_foot_goal(1))/6,(human_robot.left_foot_goal(2) + human_robot.right_foot_goal(2))/4,...
                ['sum(Trim) = ',num2str(sum(human_robot.vars.Trim.value))])
end
hold off
xlabel('x'),ylabel('y'),zlabel('z')
axis padded
%% 绘制每步的步长
figure
hold on
plot_delta_XY_has_Z(human_robot.vars.Feet_State_List.value)
hold off
pause(1);
%% 绘制H,落足点落在了哪些region中
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