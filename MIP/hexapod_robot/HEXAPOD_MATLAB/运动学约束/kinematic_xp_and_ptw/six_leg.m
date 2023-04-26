x = cell(7,1);
y = cell(7,1);
z = cell(7,1);
%% 1.
theta = 35/180*pi;
x{1} = [0 1.14*cos(theta) 0.65*cos(theta) 0 0];  % 定义等边三角形每个角的x坐标
y{1} = [0 1.14*sin(theta) 0.65*sin(theta) 0 0];  % 定义等边三角形每个角的y坐标
z{1} = [-0.28 -0.28 -0.87 -0.95 -0.28];  % 定义等边三角形每个角的y坐标

%% 2.
theta1 = 35/180*pi;
theta2 = 0/180*pi;
x{2} = [1.14*cos(theta1) 1.14*cos(theta2) 0.65*cos(theta2) 0.65*cos(theta1) 1.14*cos(theta1)];  % 定义等边三角形每个角的x坐标
y{2} = [1.14*sin(theta1) 1.14*sin(theta2) 0.65*sin(theta2) 0.65*sin(theta1) 1.14*sin(theta1)];  % 定义等边三角形每个角的y坐标
z{2} = [-0.28 -0.28 -0.87 -0.87 -0.28];  % 定义等边三角形每个角的y坐标

%% 3.
theta = -35/180*pi;
x{3} = [0 1.14*cos(theta) 0.65*cos(theta) 0 0];  % 定义等边三角形每个角的x坐标
y{3} = [0 1.14*sin(theta) 0.65*sin(theta) 0 0];  % 定义等边三角形每个角的y坐标
z{3} = [-0.28 -0.28 -0.87 -0.95 -0.28];  % 定义等边三角形每个角的y坐标

%% 4.
theta1 = -35/180*pi;
theta2 = 0/180*pi;
x{4} = [1.14*cos(theta1) 1.14*cos(theta2) 0.65*cos(theta2) 0.65*cos(theta1) 1.14*cos(theta1)];  % 定义等边三角形每个角的x坐标
y{4} = [1.14*sin(theta1) 1.14*sin(theta2) 0.65*sin(theta2) 0.65*sin(theta1) 1.14*sin(theta1)];  % 定义等边三角形每个角的y坐标
z{4} = [-0.28 -0.28 -0.87 -0.87 -0.28];  % 定义等边三角形每个角的y坐标

%% 5.
% 凸包封顶
theta1 = -35/180*pi;
theta2 = 0/180*pi;
theta3 = 35/180*pi;
x{5} = [0 1.14*cos(theta1) 1.14*cos(theta2) 1.14*cos(theta3) 0];  % 定义等边三角形每个角的x坐标
y{5} = [0 1.14*sin(theta1) 1.14*sin(theta2) 1.14*sin(theta3) 0];  % 定义等边三角形每个角的y坐标
z{5} = [-0.28 -0.28 -0.28 -0.28 -0.28];  % 定义等边三角形每个角的y坐标

%% 6.底部1
theta1 = -35/180*pi;
theta2 = 0/180*pi;
theta3 = 35/180*pi;
x{6} = [0 0.65*cos(theta1) 0.65*cos(theta2) 0];  % 定义等边三角形每个角的x坐标
y{6} = [0 0.65*sin(theta1) 0.65*sin(theta2) 0];  % 定义等边三角形每个角的y坐标
z{6} = [-0.95 -0.87 -0.87 -0.95];  % 定义等边三角形每个角的y坐标


%% 7.底部2
theta1 = -35/180*pi
theta2 = 0/180*pi
theta3 = 35/180*pi
x{7} = [0 0.65*cos(theta2) 0.65*cos(theta3) 0];  % 定义等边三角形每个角的x坐标
y{7} = [0 0.65*sin(theta2) 0.65*sin(theta3) 0];  % 定义等边三角形每个角的y坐标
z{7} = [-0.95 -0.87 -0.87 -0.95];  % 定义等边三角形每个角的y坐标


%% 
deg = 60;
for i=1:6
    deg = deg +60;
    R = rotz(deg);
    p = [0.4*cos(deg/180*pi);0.4*sin(deg/180*pi);0.5];
    T  = eye(4,4);
    T(1:3,1:3) = R;
    T(1:3,4) = p;
    for j = 1:7
        p_temp = [x{j};y{j};z{j}];
        p_temp(end+1,:) = 1;
        p_temp = T * p_temp;
        p_temp(end,:) = [];
        patch(p_temp(1,:), p_temp(2,:), p_temp(3,:), 'red'); % 绘制蓝色等边三角形
    end
end
alpha(0.3)
view(0,45)
axis equal