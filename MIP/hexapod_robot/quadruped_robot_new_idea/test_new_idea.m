%test_new_idea.m，yaw角全部都为0，不考虑yaw角
clear,clc
new_quad = Quadruped_robot_New_idea([3.2,0,0,0]);
new_quad.set_vars_relation();
new_quad.set_COM_Goal_cost();
new_quad.set_COM_Run_cost();
new_quad.set_Step_Goal_cost();
new_quad.set_Step_Run_cost();
new_quad.set_Gait_switch();
% new_quad.set_Equilibrium_cost();
%% 添加Trim
new_quad.set_Trim_flag();
%% 添加区域约束
region_center = [0,0,0; 1.075,0,0.1; 1.715,0,0.2; 2.23,0,0.1; 3.1,0,0];
length_x = [1.4; 0.7; 0.5; 0.5; 1.2];
length_y = [0.8; 0.8; 0.8; 0.8; 0.8];
rectangle_region_con_has_z = creat_rectangle_region_con_has_z(region_center,length_x,length_y);
new_quad.set_foot_region(rectangle_region_con_has_z);

%% Gurobi求解
% params.NonConvex = 2;
params.TimeLimit = 30;
[~,time_solve,objval] = new_quad.Gurobi_solve(params,0)

%% 获取数学模型对应的lp文件
model = new_quad.getGurobiModel();
gurobi_write(model,'3.lp');

%% 可视化步态
square_points = [0.5,0.4; 0.5,-0.4; -0.5,-0.4; -0.5,0.4];
square_points = [square_points;square_points(1,:)];
figure
hold on
for j=2:new_quad.N
    for i=1:4
        new_square = square_points + [j,5-i];
        if new_quad.vars.T.value(i,j) == 1
            patch(new_square(:,1),new_square(:,2),'g')
        else
            patch(new_square(:,1),new_square(:,2),'k')
        end
    end
end
axis equal
ylim([0 5])
xlim([0 inf])
hold off
title('步态可视化')
text(0.2,4,'1-左前')
text(0.2,3,'2-左后')
text(0.2,2,'3-右后')
text(0.2,1,'4-右前')

%% 绘制步长
new_Steps_Length_plot(new_quad);

%% 落足区域可视化
rectangle_region_con_has_z = creat_rectangle_region_con_has_z(region_center,length_x,length_y);
hold on
pause_time = 1;
color = ['g','b','y','m'];
for j=1:new_quad.N
   quiver3(new_quad.vars.COM_state.value(1,j), new_quad.vars.COM_state.value(2,j), new_quad.vars.COM_state.value(3,j),...
            cos(new_quad.vars.COM_state.value(4,j)), sin(new_quad.vars.COM_state.value(4,j)),0,'o','MarkerSize',3.5,'MarkerEdgeColor','r','MarkerFaceColor','r','ShowArrowHead','On');
   s = [];
   for i=1:4
       temp_index = (4*i-3):(4*i-1);
       feet = new_quad.vars.Leg_state.value(temp_index,j);
       s(i) = scatter3(feet(1),feet(2),feet(3),'MarkerEdgeColor','k','MarkerFaceColor',color(i));
       drawnow
   end
   pause(pause_time)
   delete(s)
end
hold off

%% 根据落足点创建新的region
region_center = [0,0,0; 1.075,0,0.1; 1.715,0,0.2; 2.23,0,0.1; 3.1,0,0];
length_x = [1.4; 0.7; 0.5; 0.5; 1.2];
length_y = [0.8; 0.8; 0.8; 0.8; 0.8];
rectangle_region_con_has_z = creat_rectangle_region_con_has_z(region_center,length_x,length_y);
color = ['g','b','y','m'];
hold on
pause_time = 0.25;
for j=1:new_quad.N
   quiver3(new_quad.vars.COM_state.value(1,j), new_quad.vars.COM_state.value(2,j), new_quad.vars.COM_state.value(3,j),...
            cos(new_quad.vars.COM_state.value(4,j)), sin(new_quad.vars.COM_state.value(4,j)),0,'o','MarkerSize',3.5,'MarkerEdgeColor','r','MarkerFaceColor','r','ShowArrowHead','On');
   
   s = [];
   for i=1:4
       temp_index = (4*i-3):(4*i-1);
       feet = new_quad.vars.Leg_state.value(temp_index,j);
       s(i) = scatter3(feet(1),feet(2),feet(3),'MarkerEdgeColor','k','MarkerFaceColor',color(i));
       drawnow
   end
   pause(pause_time)
end
hold off