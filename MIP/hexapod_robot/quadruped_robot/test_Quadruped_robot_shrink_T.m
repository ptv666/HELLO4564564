%test_Quadruped_robot_shrink_T.m
clear,clc
% close all
tic
quad_robot = Quadruped_robot_shrink_T([3.3,0,0,0]');
quad_robot.set_Goal_cost();                 %加入目标goal二次代价值
quad_robot.set_Run_cost();                  %加入两步之间的代价值
quad_robot.set_Leg_Workspace_constrain();   %单腿工作空间约束
% quad_robot.set_Trim_flag();

region_center = [0,0,0; 0.8,0,0.04; 1.45,0,0.08; 2.05,0,0.04; 2.75,0,0];
length_x = [0.95; 0.65; 0.6; 0.55; 0.8]*0.99;
length_y = ones(size(region_center,1),1)*0.99;
theta	 = [0; 0; 0; 0; 0]/180*pi;
QingXie_rectangle_region_con=creat__QingXie_rectangle_region_con(region_center,length_x,length_y,theta);
quad_robot.set_foot_region(QingXie_rectangle_region_con,[1 1 1 1])

sprintf('添加一般的约束：')
toc
tic
quad_robot.set_Gait_Matrix(); 
sprintf('添加Gait约束：')	%目前并没有代价值的对抗
toc

% quad_robot.set_Equilibrium_cost();

tic
quad_robot.set_Gait_switch();	%步态切换，先计算delta_z_Flag，再计算moment_Flag
sprintf('添加Switch约束：')
toc
%% Gurobi求解 
param.TimeLimit = 60;
[~,solvertime, objval] = quad_robot.Gurobi_solve(param,1)
% [~,solvertime, objval] = quad_robot.Gurobi_solve([])
% sum(quad_robot.vars.Trim.value)
%% 可视化步长
Plot_Steps_Length(quad_robot)

%% 动画播放
% figure
plot_Leg_region = [0,0,0,0];
grid on
hold on
pause_time = 0.01;
%正方形工作空间角点相对正方形中心的坐标
square_points = [0.5*quad_robot.square_l,0.5*quad_robot.square_l;0.5*quad_robot.square_l,-0.5*quad_robot.square_l;...
                     -0.5*quad_robot.square_l,-0.5*quad_robot.square_l;-0.5*quad_robot.square_l,0.5*quad_robot.square_l];
square_points = [square_points;square_points(1,:)];
for i = 1:4:quad_robot.vars.Feet_State_List.size(2)
    
    %1-左前，浅红色[0.8 0.4 0.4]
    h1 = quiver3(quad_robot.vars.Feet_State_List.value(1,i),quad_robot.vars.Feet_State_List.value(2,i),quad_robot.vars.Feet_State_List.value(3,i),...
       cos(quad_robot.vars.Feet_State_List.value(4,i)),sin(quad_robot.vars.Feet_State_List.value(4,i)),0,0.2,...
       'o','MarkerSize',5,'MarkerEdgeColor',[0.8 0.4 0.4],'MarkerFaceColor',[0.8 0.4 0.4],'Color',[0.8 0.4 0.4],'ShowArrowHead','On',...
       'LineWidth',2.5,'ShowArrowHead',true,'AutoScaleFactor',2); 	%1-左前
    pause(pause_time);
    
    %2-左后，棕色[0.9 0.7 0.1]
    h2 = quiver3(quad_robot.vars.Feet_State_List.value(1,i+1),quad_robot.vars.Feet_State_List.value(2,i+1),quad_robot.vars.Feet_State_List.value(3,i+1),...
       cos(quad_robot.vars.Feet_State_List.value(4,i+1)),sin(quad_robot.vars.Feet_State_List.value(4,i+1)),0,0.2,...
       'o','MarkerSize',5,'MarkerEdgeColor',[0.9 0.7 0.1],'MarkerFaceColor',[0.9 0.7 0.1],'Color',[0.9 0.7 0.1],'ShowArrowHead','On',...
       'LineWidth',2.5,'ShowArrowHead',true,'AutoScaleFactor',2);	%2-右脚
    drawnow
    pause(pause_time);
    
    %3-右后，粉红色[1 0 1]
    h3 = quiver3(quad_robot.vars.Feet_State_List.value(1,i+2),quad_robot.vars.Feet_State_List.value(2,i+2),quad_robot.vars.Feet_State_List.value(3,i+2),...
       cos(quad_robot.vars.Feet_State_List.value(4,i+2)),sin(quad_robot.vars.Feet_State_List.value(4,i+2)),0,0.2,...
       'o','MarkerSize',5,'MarkerEdgeColor',[1 0 1],'MarkerFaceColor',[1 0 1],'Color',[1 0 1],'ShowArrowHead','On',...
       'LineWidth',2.5,'ShowArrowHead',true,'AutoScaleFactor',2);	%3-右后
    drawnow
    pause(pause_time);
    
    %4-右前，浅绿色 [0.5 1.0 0.1]
    h4 = quiver3(quad_robot.vars.Feet_State_List.value(1,i+3),quad_robot.vars.Feet_State_List.value(2,i+3),quad_robot.vars.Feet_State_List.value(3,i+3),...
       cos(quad_robot.vars.Feet_State_List.value(4,i+3)),sin(quad_robot.vars.Feet_State_List.value(4,i+3)),0,0.2,...
       'o','MarkerSize',5,'MarkerEdgeColor',[0.5 1.0 0.1],'MarkerFaceColor',[0.5 1.0 0.1],'Color',[0.5 1.0 0.1],'ShowArrowHead','On',...
       'LineWidth',2.5,'ShowArrowHead',true,'AutoScaleFactor',2);	%4-右前
    drawnow
    pause(pause_time);
    
    %质心 
    h5 = scatter3((quad_robot.vars.Feet_State_List.value(1,i) + quad_robot.vars.Feet_State_List.value(1,i+1) + quad_robot.vars.Feet_State_List.value(1,i+2) + quad_robot.vars.Feet_State_List.value(1,i+3))/4,...
        (quad_robot.vars.Feet_State_List.value(2,i) + quad_robot.vars.Feet_State_List.value(2,i+1) + quad_robot.vars.Feet_State_List.value(2,i+2) + quad_robot.vars.Feet_State_List.value(2,i+3))/4,...
        (quad_robot.vars.Feet_State_List.value(3,i) + quad_robot.vars.Feet_State_List.value(3,i+1) + quad_robot.vars.Feet_State_List.value(3,i+2) + quad_robot.vars.Feet_State_List.value(3,i+3))/4,...
        30,'MarkerEdgeColor',[1 0 0],'MarkerFaceColor',[1 0 0]); 	%质心
    drawnow
    pause(pause_time);
    
    %绘制支撑多边形
    feet_points = [];
    for j=0:3
        temp_point = reshape(quad_robot.vars.Feet_State_List.value(1:3,i+j),1,[]);
        feet_points = [feet_points; temp_point];
    end
    feet_points = [feet_points; feet_points(1,:)];
    plot3(feet_points(:,1), feet_points(:,2), feet_points(:,3),'k')
    drawnow
    pause(pause_time);
    
    
    COM_now = quad_robot.vars.COM.value(:,i)';
    %绘制下一步可行域
    if plot_Leg_region(1)
        theta_1 = quad_robot.vars.Feet_State_List.value(4,i) + quad_robot.theta_offset(1);
        center_1 = COM_now + quad_robot.Leg_length*[cos(theta_1),sin(theta_1)];
        square_1 = square_points + center_1;
        plot(square_1(:,1),square_1(:,2),'Color',[0.8 0.4 0.4])
        drawnow
        pause(pause_time);
    end    

    if plot_Leg_region(2)
        theta_2 = quad_robot.vars.Feet_State_List.value(4,i) + quad_robot.theta_offset(2);
        center_2 = COM_now + quad_robot.Leg_length*[cos(theta_2),sin(theta_2)];
        square_2 = square_points + center_2;
        plot(square_2(:,1),square_2(:,2),'Color',[0.9 0.7 0.1])
        drawnow
        pause(pause_time);
    end
    
    if plot_Leg_region(3)
        theta_3 = quad_robot.vars.Feet_State_List.value(4,i) + quad_robot.theta_offset(3); 
        center_3 = COM_now + quad_robot.Leg_length*[cos(theta_3),sin(theta_3)];
        square_3 = square_points + center_3;
        plot(square_3(:,1),square_3(:,2),'Color',[1 0 1])
        drawnow
        pause(pause_time);
    end
    
    if  plot_Leg_region(4)
        theta_4 = quad_robot.vars.Feet_State_List.value(4,i) + quad_robot.theta_offset(4); 
        center_4 = COM_now + quad_robot.Leg_length*[cos(theta_4),sin(theta_4)];
        square_4 = square_points + center_4;
        plot(square_4(:,1),square_4(:,2),'Color',[0.5 1.0 0.1])
        drawnow
    end
    
end

% legend(h1, '1-左前', h2, '2-左后', h3, '3-右后', h4, '4-右前', h5, '质心', 'Location', 'best')
%在中间加入text注释
text((quad_robot.feet_goal(1,1) + quad_robot.feet_goal(1,2))/6,(quad_robot.feet_goal(2,1) + quad_robot.feet_goal(2,2))/5.5,[num2str(quad_robot.N),'步'])

if isfield(quad_robot.vars,'Trim')
    text((quad_robot.feet_goal(1,1) + quad_robot.feet_goal(1,2))/6,(quad_robot.feet_goal(2,1) + quad_robot.feet_goal(2,2))/4,...
                ['sum(Trim) = ',num2str(sum(quad_robot.vars.Trim.value))])
end

hold off
xlabel('x'),ylabel('y'),zlabel('z')
% axis equal
axis padded

%% 可视化步态
figure
hold on
for i=1:4:quad_robot.N
    
    t1 = quad_robot.vars.Timing.value(1,i);
    t2 = quad_robot.vars.Timing.value(1,i+1);
    t3 = quad_robot.vars.Timing.value(1,i+2);
    t4 = quad_robot.vars.Timing.value(1,i+3);
    
    start_time = min([t1,t2,t3,t4]);
    line([start_time, start_time], [0, 5],'LineStyle', '--')
    
    scatter(t1, 4, 'MarkerEdgeColor', [0.8 0.4 0.4], 'MarkerFaceColor', [0.8 0.4 0.4])              %1-左前
    scatter(t2, 3, 'MarkerEdgeColor', [0.9 0.7 0.1], 'MarkerFaceColor', [0.9 0.7 0.1])	%2-右脚
    scatter(t3, 2, 'MarkerEdgeColor', [0.5 1.0 0.1], 'MarkerFaceColor', [0.5 1.0 0.1])	%3-右后
    scatter(t4, 1, 'MarkerEdgeColor', [1 0 1], 'MarkerFaceColor', [1 0 1])             	%4-右前
    
end
text(2,4,'1-左前')
text(2,3,'2-左后')
text(2,2,'3-右后')
text(2,1,'4-右前')