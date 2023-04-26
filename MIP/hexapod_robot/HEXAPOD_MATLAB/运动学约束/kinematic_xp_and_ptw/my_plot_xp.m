%% 单腿工作空间求解文件
%%先求解单腿的工作空间
% 已知足端相对关节2坐标系的位置坐标 已知T_W_2，就能计算出足端相对世界的坐�?

% 验证足端是否在LEG的工作空间范围内
clear;
close all;
point3 = [];
% buchang = 0.05;
buchang = 0.1;
for t1 = -2/9*pi:buchang: 2/9*pi
    for t2 = -pi/2:buchang:pi/2
        for t3 = -5/6*pi:buchang:0
            point3(end+1,:) = compute_P(t1,t2,t3);
        end
    end
end
%%三维绘图
figure
hold on
scatter3(point3(:,1),point3(:,2),point3(:,3),".")   %三维点云
view(0,45)
axis equal
xlabel('X');
ylabel('Y');
zlabel('Z');
%% 绘制扇形
R = 0.7;
lw = 1.5;
x =  [0.18,0.18 + R*cosd(45)];
y =  [0,R*sind(45)];
z = [-0.5,-0.5];
plot3(x,y,z,'color','g','LineWidth',lw)
for theta = linspace(pi/4,-pi/4,100)
    x = [x(2),0.18+R * cos(theta)];
    y = [y(2),R * sin(theta)];
    plot3(x,y,z,'color','g','LineWidth',lw)
end
x =  [0.18,0.18 + R*cosd(-45)];
y =  [0,R*sind(-45)];
z = [-0.5,-0.5];
plot3(x,y,z,'color','g','LineWidth',lw)
%% 1.
theta = 35/180*pi
x = [0 1.14*cos(theta) 0.65*cos(theta) 0 0];  % 定义等边三角形每个角的x坐标
y = [0 1.14*sin(theta) 0.65*sin(theta) 0 0];  % 定义等边三角形每个角的y坐标
z = [-0.28 -0.28 -0.87 -0.95 -0.28];  % 定义等边三角形每个角的y坐标
s1 = patch(x, y, z, 'red'); % 绘制蓝色等边三角形
s1.FaceVertexAlphaData = 0.3;
s1.FaceAlpha = 'flat' ; 
%% 2.
theta1 = 35/180*pi
theta2 = 0/180*pi
x = [1.14*cos(theta1) 1.14*cos(theta2) 0.65*cos(theta2) 0.65*cos(theta1) 1.14*cos(theta1)];  % 定义等边三角形每个角的x坐标
y = [1.14*sin(theta1) 1.14*sin(theta2) 0.65*sin(theta2) 0.65*sin(theta1) 1.14*sin(theta1)];  % 定义等边三角形每个角的y坐标
z = [-0.28 -0.28 -0.87 -0.87 -0.28];  % 定义等边三角形每个角的y坐标
s2 = patch(x, y, z, 'red'); % 绘制蓝色等边三角形
s2.FaceVertexAlphaData = 0.3;
s2.FaceAlpha = 'flat' ; 

%% 3.
theta = -35/180*pi
x = [0 1.14*cos(theta) 0.65*cos(theta) 0 0];  % 定义等边三角形每个角的x坐标
y = [0 1.14*sin(theta) 0.65*sin(theta) 0 0];  % 定义等边三角形每个角的y坐标
z = [-0.28 -0.28 -0.87 -0.95 -0.28];  % 定义等边三角形每个角的y坐标
s3 = patch(x, y, z, 'red'); % 绘制蓝色等边三角形
s3.FaceVertexAlphaData = 0.3;
s3.FaceAlpha = 'flat' ; 
%% 4.
theta1 = -35/180*pi
theta2 = 0/180*pi
x = [1.14*cos(theta1) 1.14*cos(theta2) 0.65*cos(theta2) 0.65*cos(theta1) 1.14*cos(theta1)];  % 定义等边三角形每个角的x坐标
y = [1.14*sin(theta1) 1.14*sin(theta2) 0.65*sin(theta2) 0.65*sin(theta1) 1.14*sin(theta1)];  % 定义等边三角形每个角的y坐标
z = [-0.28 -0.28 -0.87 -0.87 -0.28];  % 定义等边三角形每个角的y坐标
s4 = patch(x, y, z, 'red'); % 绘制蓝色等边三角形
s4.FaceVertexAlphaData = 0.3;
s4.FaceAlpha = 'flat' ; 

%% 5.
% 凸包封顶
theta1 = -35/180*pi
theta2 = 0/180*pi
theta3 = 35/180*pi
x = [0 1.14*cos(theta1) 1.14*cos(theta2) 1.14*cos(theta3) 0];  % 定义等边三角形每个角的x坐标
y = [0 1.14*sin(theta1) 1.14*sin(theta2) 1.14*sin(theta3) 0];  % 定义等边三角形每个角的y坐标
z = [-0.28 -0.28 -0.28 -0.28 -0.28];  % 定义等边三角形每个角的y坐标
s5 = patch(x, y, z, 'red'); % 绘制蓝色等边三角形
s5.FaceVertexAlphaData = 0.3;
s5.FaceAlpha = 'flat' ; 

%% 6.底部1
theta1 = -35/180*pi
theta2 = 0/180*pi
theta3 = 35/180*pi
x = [0 0.65*cos(theta1) 0.65*cos(theta2) 0];  % 定义等边三角形每个角的x坐标
y = [0 0.65*sin(theta1) 0.65*sin(theta2) 0];  % 定义等边三角形每个角的y坐标
z = [-0.95 -0.87 -0.87 -0.95];  % 定义等边三角形每个角的y坐标
s6 = patch(x, y, z, 'red'); % 绘制蓝色等边三角形
s6.FaceVertexAlphaData = 0.3;
s6.FaceAlpha = 'flat' ; 

%% 7.底部2
theta1 = -35/180*pi
theta2 = 0/180*pi
theta3 = 35/180*pi
x = [0 0.65*cos(theta2) 0.65*cos(theta3) 0];  % 定义等边三角形每个角的x坐标
y = [0 0.65*sin(theta2) 0.65*sin(theta3) 0];  % 定义等边三角形每个角的y坐标
z = [-0.95 -0.87 -0.87 -0.95];  % 定义等边三角形每个角的y坐标
s7 = patch(x, y, z, 'red'); % 绘制蓝色等边三角形
s7.FaceVertexAlphaData = 0.3;
s7.FaceAlpha = 'flat' ; 
axis equal;  % 设置坐标轴刻度相等，以便看到等边三角形的实际形状
alpha(0.5)