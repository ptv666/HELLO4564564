function rectangle_region_con=creat_rectangle_region_con(region_center,length_x,length_y)
%创建矩形的region约束，输入的可以是多组数据，但是需要排成有序的 *列向量*
% @region_center：矩形region的中心，n×2
% @length_x：矩形的x方向边长，n×1
% @length_y：矩形y方向的边长，n×1
%输出的约束形式为：注意该函数输出中并不含M，需要自己在添加Gurobi约束的时候自己添加，添加也简单，就是在H处加一个M，在b处加一个M即可
% x+HM<Lx+x0+M;
% -x+HM<Lx-x0+M;
% y+HM<Ly+y0+M;
% -y+HM<Ly-y0+M;
    a1 = size(region_center,1);
    a11 = size(region_center,2);
    a2 = size(length_x,1);
    a3 = size(length_y,1);
    if a11~=2
        error('创建region_constraint的中心点数据不是按行排列的。creat_rectangle_region_con_Line_16')
    end
    if a1~=a2 || a1~=a3 || a2~=a3
        error('创建region_constraint的数据长度不一致。creat_rectangle_region_con_Line_19')
    end
    
    rectangle_region_con = struct('A',{},'b',{});
    for i = 1:a1
        x0 = region_center(i,1);
        y0 = region_center(i,2);
        Lx = length_x(i);
        Ly = length_y(i);
        A = [1;-1;1;-1];
        b = [0.5*Lx + x0; 0.5*Lx - x0; 0.5*Ly + y0; 0.5*Ly - y0];
        temp = struct('A',A,'b',b);
        rectangle_region_con = [rectangle_region_con;temp];
    end
    %顺便绘制个图像
    figure
    hold on
    for i=1:a1
        center = region_center(i,:);
        Lx = length_x(i,1);
        Ly = length_y(i,1);
        point1 = [center(1) - Lx/2,center(2) - Ly/2];
        point2 = [center(1) - Lx/2,center(2) + Ly/2];
        point3 = [center(1) + Lx/2,center(2) + Ly/2];
        point4 = [center(1) + Lx/2,center(2) - Ly/2];
        points = [point1;point2;point3;point4;point1];
        plot(points(:,1),points(:,2))
        text(center(1),center(2) + 0.55*Ly,['region',num2str(i)]);
    end
    hold off
end
%% 测试的
% region_center = [2.25,0.25;2.25,-0.25];
% length_x = [4.5;4.5];
% length_y = [0.2;0.2];
% rectangle_region_con=creat_rectangle_region_con(region_center,length_x,length_y);