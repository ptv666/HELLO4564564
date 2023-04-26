%% 初始化
clear,clc
% close all
% human_robot = humanoid_robot_has_z([5,5,1.2,-pi/2]'); %水平向前行走，走5m，最大步长为0.4m，需要走13步
human_robot = humanoid_robot_has_z([4,0,0,0]');    %目标值不能超过上下限啊！
human_robot.set_Goal_cost();
human_robot.set_Run_cost();
human_robot.set_Leg_Workspace_constrain();
human_robot.set_Trim_flag();
%% 设置地形，选择一个地形，取消它的注释
% 直接走上去
% region_center = [0,0,0; 0,2.5,0.02; 4,2,0.05; 4,-2,0.08; 4,0,0.12];
% length_x = [4.98; 5; 2.7; 2.7; 2.7];
% length_y = [3.98; 1; 2; 2; 1.5];
% rectangle_region_con=creat_rectangle_region_con_has_z(region_center,length_x,length_y);
% xlim([-5,8]),ylim([-5,5]),zlim([0,0.3])

%先右再左
region_center = [0,0,0; 0,2.5,0.02; 4,2,0.05; 4,-2,0.15; 4,0,0.3];
length_x = [4.98; 5; 2.5; 2.5; 1];
length_y = [3.98; 1; 2; 2; 1.5];
rectangle_region_con=creat_rectangle_region_con_has_z(region_center,length_x,length_y);

%先左再右
% region_center = [0,0,0; 0,2.5,0.08; 4,2,0.2; 4,-2,0.10; 4,0,0.3];
% length_x = [4.98; 5; 2.2; 2.2; 1.2];
% length_y = [3.98; 1; 2; 2; 1.5];
% rectangle_region_con=creat_rectangle_region_con_has_z(region_center,length_x,length_y);

human_robot.set_foot_region(rectangle_region_con,[1 1])

%% Gurobi优化求解
params.NonConvex = 2;
% params.TimeLimit = 20;
[~,solve_time,obj_val] = human_robot.Gurobi_solve(params)
%% 绘图，也选择想要的实验，取消其注释
close all
% rectangle_region_con=creat_rectangle_region_con_has_z(region_center,length_x,length_y);
% xlim([-4,7]),ylim([-5,4.5]),zlim([0,0.3])
% daspect([1 1 0.12])
% view(-40,30)

% rectangle_region_con=creat_rectangle_region_con_has_z(region_center,length_x,length_y);
% xlim([-4,7]),ylim([-4.5,4.5]),zlim([0,0.3])
% daspect([1 1 0.13])
%view(-40,30)

rectangle_region_con=creat_rectangle_region_con_has_z(region_center,length_x,length_y);
xlim([-4,7]),ylim([-4.5,5]),zlim([0,0.3])
daspect([1 1 0.15])

view(-40,35)
pause_time = 0.1;

hold on
grid on

hold on
for i = 1:2:human_robot.vars.Feet_State_List.size(2)
    %左脚
    LL = quiver3(human_robot.vars.Feet_State_List.value(1,i),human_robot.vars.Feet_State_List.value(2,i),human_robot.vars.Feet_State_List.value(3,i),...
       cos(human_robot.vars.Feet_State_List.value(4,i)),sin(human_robot.vars.Feet_State_List.value(4,i)),0,0.2,...
       'o','MarkerSize',2.5,'MarkerEdgeColor',[0.9 0.7 0.1],'MarkerFaceColor',[0.9 0.7 0.1],'Color',[0.9 0.7 0.1],'ShowArrowHead','On',...
       'LineWidth',2.5,'ShowArrowHead',true,'AutoScaleFactor',2)   %左脚
    %右脚
    pause(pause_time);
    RR = quiver3(human_robot.vars.Feet_State_List.value(1,i+1),human_robot.vars.Feet_State_List.value(2,i+1),human_robot.vars.Feet_State_List.value(3,i+1),...
       cos(human_robot.vars.Feet_State_List.value(4,i+1)),sin(human_robot.vars.Feet_State_List.value(4,i+1)),0,0.2,...
       'o','MarkerSize',2.5,'MarkerEdgeColor',[0.3 0.9 0.2],'MarkerFaceColor',[0.3 0.9 0.2],'Color',[0.3 0.9 0.2],'ShowArrowHead','On',...
       'LineWidth',2.5,'ShowArrowHead',true,'AutoScaleFactor',2)   %右脚
    drawnow
    %质心 
    pause(pause_time);
    CC = scatter3((human_robot.vars.Feet_State_List.value(1,i) + human_robot.vars.Feet_State_List.value(1,i+1))/2,...
        (human_robot.vars.Feet_State_List.value(2,i) + human_robot.vars.Feet_State_List.value(2,i+1))/2,...
        (human_robot.vars.Feet_State_List.value(3,i) + human_robot.vars.Feet_State_List.value(3,i+1))/2,...
        30,'MarkerEdgeColor',[1 0 0],'MarkerFaceColor',[1 0 0])   %质心
    drawnow
    pause(pause_time);
end
%legend('左脚','右脚','质心','Location','best')
legend([LL,RR,CC],{'左脚','右脚','质心'},'Location','best')
%在中间加入text注释
text(0,-2,[num2str(human_robot.N),'步'])
if isfield(human_robot.vars,'Trim')
    text(-1.5,-2.5,...
                ['sum(Trim) = ',num2str(sum(human_robot.vars.Trim.value))])
end
hold off
xlabel('x'),ylabel('y'),zlabel('z')