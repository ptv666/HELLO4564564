function rectangle_region_con_has_z=creat_rectangle_region_con_has_z(region_center,length_x,length_y)
%创建矩形的region平面约束，输入的可以是多组数据，但是需要排成有序的 *列向量*
%根据输入中心点的高度确定为整个平面的高度
% @region_center：矩形region的中心，n×3，在xy矩形约束的基础上添加了高度z，
% @length_x：矩形的x方向边长，n×1
% @length_y：矩形y方向的边长，n×1
%输出的约束形式为：注意该函数输出中并不含M，需要自己在添加Gurobi约束的时候自己添加，添加也简单，就是在H处加一个M，在b处加一个M即可
% x+HM<Lx+x0+M;
% -x+HM<Lx-x0+M;
% y+HM<Ly+y0+M;
% -y+HM<Ly-y0+M;
% z = z0;转变为大M法的不等式约束就是：
% z+HM<z0+M
% -z+HM<-z0+M
    a0 = size(region_center,1);
    a01 = size(region_center,2);
    a1 = size(length_x,1);
    a2 = size(length_y,1);
    if a01~=3
        error('创建region_constraint的中心点数据不是按行排列的。creat_rectangle_region_con_Line_20')
    end
    if a0~=a1 || a0~=a2
        error('创建region_constraint的数据长度不一致。creat_rectangle_region_con_Line_23')
    end
    
    %创建约束
    rectangle_region_con_has_z = struct('A',{},'b',{});
    for i = 1:a0
        x0 = region_center(i,1);
        y0 = region_center(i,2);
        z0 = region_center(i,3);
        Lx = length_x(i);
        Ly = length_y(i);
        A = [1;-1;1;-1;1;-1];
        b = [0.5*Lx + x0; 0.5*Lx - x0; 0.5*Ly + y0; 0.5*Ly - y0; z0; -z0];
        temp = struct('A',A,'b',b);
        rectangle_region_con_has_z = [rectangle_region_con_has_z;temp];
    end
    
    %顺便绘制个图像
    figure
    hold on
    for i=1:a0
        center = region_center(i,:);
        Lx = length_x(i,1);
        Ly = length_y(i,1);
        point1 = [center(1) - Lx/2, center(2) - Ly/2, center(3)];
        point2 = [center(1) - Lx/2, center(2) + Ly/2, center(3)];
        point3 = [center(1) + Lx/2, center(2) + Ly/2, center(3)];
        point4 = [center(1) + Lx/2, center(2) - Ly/2, center(3)];
        points = [point1;point2;point3;point4;point1];
        L = plot3(points(:,1),points(:,2),points(:,3));     %绘制矩形
        color = L.Color;
        if center(3)~=0
            for j=1:4
                plot3([points(j,1),points(j,1)],[points(j,2),points(j,2)],[points(j,3),0],'--','color',color)
            end
        end
        points(:,3) = 0;
        plot3(points(:,1),points(:,2),points(:,3),'--','color',color);
        text(center(1), center(2) + 0.55*Ly, center(3), ['region', num2str(i)]);
    end
    view(-30,45)
    axis padded
    grid on
    xlabel('x'),ylabel('y'),zlabel('z')
    hold off
end