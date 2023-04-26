%% 减少闪烁，直接运行就能跑的版本，选择mat文件，一个有trim剪枝，一个没有trim剪枝
load('中期_无剪枝视频.mat')
% load('步态切换2_联合Trim_600s.mat')
region_center = [0,0,0; 1.075,0,0.1; 1.715,0,0.2; 2.23,0,0.1; 3.3,0,0];
length_x = [1.4; 0.7; 0.5; 0.5; 1.6];
length_y = [0.8; 0.8; 0.8; 0.8; 0.8];
rectangle_region_con_has_z = creat_rectangle_region_con_has_z(region_center,length_x,length_y);

%% 可视化步态
square_points = [0.5,0.4; 0.5,-0.4; -0.5,-0.4; -0.5,0.4];
square_points = [square_points;square_points(1,:)];
figure
hold on
for j=2:new_quad.N
    for i=1:4
        new_square = square_points +[j,7-i];
        if new_quad.vars.T.value(i,j) == 1
            patch(new_square(:,1),new_square(:,2),'g')
        else
            patch(new_square(:,1),new_square(:,2),'k')
        end
    end
end
axis equal
ylim([0 8])
xlim([-4 inf])
hold off
title('步态可视化')
text(-1.5,6,'1-左前')
text(-1.5,5,'2-左后')
text(-1.5,4,'3-右后')
text(-1.5,3,'4-右前')

% text(-2.5,6,'1-左前')
% text(-2.5,5,'2-左后')
% text(-2.5,4,'3-右后')
% text(-2.5,3,'4-右前')

%% 运动动画制作
rectangle_region_con_has_z = creat_rectangle_region_con_has_z(region_center,length_x,length_y);
hold on
pause_time = 1;
color = ['g','b','y','m'];
s = [];
s_T = zeros(4,1);

%初始状态绘制
j = 1;
quiver3(new_quad.vars.COM_state.value(1,j), new_quad.vars.COM_state.value(2,j), new_quad.vars.COM_state.value(3,j),...
            cos(new_quad.vars.COM_state.value(4,j)), sin(new_quad.vars.COM_state.value(4,j)),0,'o','MarkerSize',3.5,'MarkerEdgeColor','r','MarkerFaceColor','r','ShowArrowHead','On');
for i=1:4
    temp_index = (4*i-3):(4*i-1);
    feet = new_quad.vars.Leg_state.value(temp_index,j);
    s(i) = scatter3(feet(1),feet(2),feet(3),'MarkerEdgeColor','k','MarkerFaceColor',color(i));
    drawnow
end

for j=2:new_quad.N
   quiver3(new_quad.vars.COM_state.value(1,j), new_quad.vars.COM_state.value(2,j), new_quad.vars.COM_state.value(3,j),...
            cos(new_quad.vars.COM_state.value(4,j)), sin(new_quad.vars.COM_state.value(4,j)),0,'o','MarkerSize',3.5,'MarkerEdgeColor','r','MarkerFaceColor','r','ShowArrowHead','On');
   for i=1:4
       temp_index = (4*i-3):(4*i-1);
       feet = new_quad.vars.Leg_state.value(temp_index,j);
       if new_quad.vars.T.value(i,j) == 1
           delete(s(i))
           s(i) = scatter3(feet(1),feet(2),feet(3),'MarkerEdgeColor','k','MarkerFaceColor',color(i));
           drawnow
       end
   end
   pause(pause_time)
   
end
hold off