%% 单腿工作空间求解文件
%%
%%先求解单腿的工作空间
% 已知足端相对关节2坐标系的位置坐标 已知T_W_2，就能计算出足端相对世界的坐标

% 验证足端是否在LEG的工作空间范围内
point3 = [];
for t1 = -2/9*pi:0.1: 2/9*pi
    for t2 = -pi/2:0.1:pi/2
        for t3 = -5/6*pi:0.1:0
            point3(end+1,:) = compute_P(t1,t2,t3);
        end
    end
end
%% 三维绘图
figure
hold on
scatter3(point3(:,1),point3(:,2),point3(:,3))
view(0,45)
axis equal
%% 二维绘图
figure
axis equal
point = [];
t1 = 0;
for t2 = -pi/2:0.01:pi/2
    for t3 = -5/6*pi:0.01:0
        point(end+1,:) = compute_P(t1,t2,t3);
    end
end
hold on
scatter(point(:,1),point(:,3))
yline(0,'--')
xline(0,'--')
scatter(0.18,-0.5,'ro')
axis equal