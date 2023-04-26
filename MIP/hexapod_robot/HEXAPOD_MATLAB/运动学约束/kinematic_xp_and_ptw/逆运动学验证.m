%% 验证逆运动学
%% 1.面1竖直
clear,close all,clc
point1 = [0;0;-0.28];
point2 = [0;0;-0.95];
delta = point2 - point1;
theta = zeros(3,1);
t1= [];t2=[];t3=[];

for i=linspace(0,1,1000)
    point = point1 + i*delta;
    theta = inverse_FixJi(point);
    if abs(theta(1)) > 40/180*pi || abs(theta(2)) > pi/2 || theta(3) > 0 || theta(3) < -5*pi/6
        input('不在运动学空间内')
    end
    t1(end+1) = theta(1)*180/pi;    t2(end+1) = theta(2)*180/pi;    t3(end+1) = theta(3)*180/pi;
end
figure
hold on
plot(t1),plot(t2),plot(t3)
line([0,1000],[-90,-90])
legend('t1','t2','t3','-90°')

%% 2.
clear,close all,clc
theta = 35/180*pi;
point1 = [0;0;-0.95];
point2 = [0.65*cos(theta);0.65*sin(theta);-0.87];
delta = point2 - point1;
theta = zeros(3,1);
t1= [];t2=[];t3=[];

for i=linspace(0,1,1000)
    point = point1 + i*delta;
    theta = inverse_FixJi(point);
    if abs(theta(1)) > 40/180*pi || abs(theta(2)) > pi/2 || theta(3) > 0 || theta(3) < -5*pi/6
        input('不在运动学空间内')
    end
    t1(end+1) = theta(1)*180/pi;    t2(end+1) = theta(2)*180/pi;    t3(end+1) = theta(3)*180/pi;
end
figure
hold on
plot(t1),plot(t2),plot(t3)
legend('t1','t2','t3')

%% 3.
clear,close all,clc
theta = -35/180*pi;
point1 = [0;0;-0.95];
point2 = [0.65*cos(theta);0.65*sin(theta);-0.87];
delta = point2 - point1;
theta = zeros(3,1);
t1= [];t2=[];t3=[];

for i=linspace(0,1,1000)
    point = point1 + i*delta;
    theta = inverse_FixJi(point);
    if abs(theta(1)) > 40/180*pi || abs(theta(2)) > pi/2 || theta(3) > 0 || theta(3) < -5*pi/6
        input('不在运动学空间内')
    end
    t1(end+1) = theta(1)*180/pi;    t2(end+1) = theta(2)*180/pi;    t3(end+1) = theta(3)*180/pi;
end
figure
hold on
plot(t1),plot(t2),plot(t3)
legend('t1','t2','t3')

%% 4.
clear,close all,clc
theta = 0/180*pi;
point1 = [0;0;-0.95];
point2 = [0.65*cos(theta);0.65*sin(theta);-0.87];
delta = point2 - point1;
theta = zeros(3,1);
t1= [];t2=[];t3=[];

for i=linspace(0,1,1000)
    point = point1 + i*delta;
    theta = inverse_FixJi(point);
    if abs(theta(1)) > 40/180*pi || abs(theta(2)) > pi/2 || theta(3) > 0 || theta(3) < -5*pi/6
        input('不在运动学空间内')
    end
    t1(end+1) = theta(1)*180/pi;    t2(end+1) = theta(2)*180/pi;    t3(end+1) = theta(3)*180/pi;
end
figure
hold on
plot(t1),plot(t2),plot(t3)
legend('t1','t2','t3')

%% 5.
clear,close all,clc
theta = 0/180*pi;
point1 = [0.65*cos(theta);0.65*sin(theta);-0.87];
theta = 35/180*pi;
point2 = [0.65*cos(theta);0.65*sin(theta);-0.87];
delta = point2 - point1;
theta = zeros(3,1);
t1= [];t2=[];t3=[];

for i=linspace(0,1,1000)
    point = point1 + i*delta;
    theta = inverse_FixJi(point);
    if abs(theta(1)) > 40/180*pi || abs(theta(2)) > pi/2 || theta(3) > 0 || theta(3) < -5*pi/6
        input('不在运动学空间内')
    end
    t1(end+1) = theta(1)*180/pi;    t2(end+1) = theta(2)*180/pi;    t3(end+1) = theta(3)*180/pi;
end
figure
hold on
plot(t1),plot(t2),plot(t3)
legend('t1','t2','t3')

%% 6.
clear,close all,clc
theta = 0/180*pi;
point1 = [0.65*cos(theta);0.65*sin(theta);-0.87];
theta = -35/180*pi;
point2 = [0.65*cos(theta);0.65*sin(theta);-0.87];
delta = point2 - point1;
theta = zeros(3,1);
t1= [];t2=[];t3=[];

for i=linspace(0,1,1000)
    point = point1 + i*delta;
    theta = inverse_FixJi(point);
    if abs(theta(1)) > 40/180*pi || abs(theta(2)) > pi/2 || theta(3) > 0 || theta(3) < -5*pi/6
        input('不在运动学空间内')
    end
    t1(end+1) = theta(1)*180/pi;    t2(end+1) = theta(2)*180/pi;    t3(end+1) = theta(3)*180/pi;
end
figure
hold on
plot(t1),plot(t2),plot(t3)
legend('t1','t2','t3')

%% 7.
clear,close all,clc
point1 = [0;0;-0.28];
theta = -35/180*pi;
point2 = [1.14*cos(theta);1.14*sin(theta);-0.28];
delta = point2 - point1;
theta = zeros(3,1);
t1= [];t2=[];t3=[];

for i=linspace(0,1,1000)
    point = point1 + i*delta;
    theta = inverse_FixJi(point);
    if abs(theta(1)) > 40/180*pi || abs(theta(2)) > pi/2 || theta(3) > 0 || theta(3) < -5*pi/6
        input('不在运动学空间内')
    end
    t1(end+1) = theta(1)*180/pi;    t2(end+1) = theta(2)*180/pi;    t3(end+1) = theta(3)*180/pi;
end
figure
hold on
plot(t1),plot(t2),plot(t3)
line([0,1000],[-150,-150])
line([0,1000],[0,0])
legend('t1','t2','t3','-150°','0°')

%% 8.
clear,close all,clc
point1 = [0;0;-0.28];
theta = 35/180*pi;
point2 = [1.14*cos(theta);1.14*sin(theta);-0.28];
delta = point2 - point1;
theta = zeros(3,1);
t1= [];t2=[];t3=[];

for i=linspace(0,1,1000)
    point = point1 + i*delta;
    theta = inverse_FixJi(point);
    if abs(theta(1)) > 40/180*pi || abs(theta(2)) > pi/2 || theta(3) > 0 || theta(3) < -5*pi/6
        input('不在运动学空间内')
    end
    t1(end+1) = theta(1)*180/pi;    t2(end+1) = theta(2)*180/pi;    t3(end+1) = theta(3)*180/pi;
end
figure
hold on
plot(t1),plot(t2),plot(t3)
line([0,1000],[-150,-150])
legend('t1','t2','t3','-150°')

%% 9.
clear,close all,clc
theta = 0/180*pi;
point1 = [1.14*cos(theta);1.14*sin(theta);-0.28];
theta = 35/180*pi;
point2 = [1.14*cos(theta);1.14*sin(theta);-0.28];
delta = point2 - point1;
theta = zeros(3,1);
t1= [];t2=[];t3=[];

for i=linspace(0,1,1000)
    point = point1 + i*delta;
    theta = inverse_FixJi(point);
    if abs(theta(1)) > 40/180*pi || abs(theta(2)) > pi/2 || theta(3) > 0 || theta(3) < -5*pi/6
        input('不在运动学空间内')
    end
    t1(end+1) = theta(1)*180/pi;    t2(end+1) = theta(2)*180/pi;    t3(end+1) = theta(3)*180/pi;
end
figure
hold on
plot(t1),plot(t2),plot(t3)
legend('t1','t2','t3')

%% 10.
clear,close all,clc
theta = 0/180*pi;
point1 = [1.14*cos(theta);1.14*sin(theta);-0.28];
theta = -35/180*pi;
point2 = [1.14*cos(theta);1.14*sin(theta);-0.28];
delta = point2 - point1;
theta = zeros(3,1);
t1= [];t2=[];t3=[];

for i=linspace(0,1,1000)
    point = point1 + i*delta;
    theta = inverse_FixJi(point);
    if abs(theta(1)) > 40/180*pi || abs(theta(2)) > pi/2 || theta(3) > 0 || theta(3) < -5*pi/6
        input('不在运动学空间内')
    end
    t1(end+1) = theta(1)*180/pi;    t2(end+1) = theta(2)*180/pi;    t3(end+1) = theta(3)*180/pi;
end
figure
hold on
plot(t1),plot(t2),plot(t3)
legend('t1','t2','t3')

%% 11.
clear,close all,clc
theta = 35/180*pi;
point1 = [1.14*cos(theta);1.14*sin(theta);-0.28];
point2 = [0.65*cos(theta);0.65*sin(theta);-0.87];
delta = point2 - point1;
theta = zeros(3,1);
t1= [];t2=[];t3=[];

for i=linspace(0,1,1000)
    point = point1 + i*delta;
    theta = inverse_FixJi(point);
    if abs(theta(1)) > 40/180*pi || abs(theta(2)) > pi/2 || theta(3) > 0 || theta(3) < -5*pi/6
        input('不在运动学空间内')
    end
    t1(end+1) = theta(1)*180/pi;    t2(end+1) = theta(2)*180/pi;    t3(end+1) = theta(3)*180/pi;
end
figure
hold on
plot(t1),plot(t2),plot(t3)
legend('t1','t2','t3')

%% 12.
clear,close all,clc
theta = -35/180*pi;
point1 = [1.14*cos(theta);1.14*sin(theta);-0.28];
point2 = [0.65*cos(theta);0.65*sin(theta);-0.87];
delta = point2 - point1;
theta = zeros(3,1);
t1= [];t2=[];t3=[];

for i=linspace(0,1,1000)
    point = point1 + i*delta;
    theta = inverse_FixJi(point);
    if abs(theta(1)) > 40/180*pi || abs(theta(2)) > pi/2 || theta(3) > 0 || theta(3) < -5*pi/6
        input('不在运动学空间内')
    end
    t1(end+1) = theta(1)*180/pi;    t2(end+1) = theta(2)*180/pi;    t3(end+1) = theta(3)*180/pi;
end
figure
hold on
plot(t1),plot(t2),plot(t3)
legend('t1','t2','t3')

%% 13.
clear,close all,clc
theta = 0/180*pi;
point1 = [1.14*cos(theta);1.14*sin(theta);-0.28];
point2 = [0.65*cos(theta);0.65*sin(theta);-0.87];
delta = point2 - point1;
theta = zeros(3,1);
t1= [];t2=[];t3=[];

for i=linspace(0,1,1000)
    point = point1 + i*delta;
    theta = inverse_FixJi(point);
    if abs(theta(1)) > 40/180*pi || abs(theta(2)) > pi/2 || theta(3) > 0 || theta(3) < -5*pi/6
        input('不在运动学空间内')
    end
    t1(end+1) = theta(1)*180/pi;    t2(end+1) = theta(2)*180/pi;    t3(end+1) = theta(3)*180/pi;
end
figure
hold on
plot(t1),plot(t2),plot(t3)
legend('t1','t2','t3')