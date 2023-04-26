%用鼠标取点构造地图的落足点
clear;
clc;
close all;

figure;

hold on;

axis([-5,5,-5,5]);
axis equal;
grid on;


% footStepData(1,:) = [-0.540000000000000;0.935307436087194;-0.500000000000000]';
% footStepData(2,:) = [-1.08000000000000;0;-0.500000000000000]';
% footStepData(3,:) = [-0.540000000000000;-0.935307436087194;-0.500000000000000]';
% footStepData(4,:) = [0.540000000000000;-0.935307436087194;-0.500000000000000]';
% footStepData(5,:) = [1.08000000000000;0;-0.500000000000000]';
% footStepData(6,:) = [0.540000000000000;0.935307436087194;-0.500000000000000]';
% % load('C:\Users\10025\Documents\WorkFile\HexapodMatlab\FootstepDesign.mat');
% 
% plot(footStepData(:,1),footStepData(:,2),'bo-','MarkerFaceColor','b');
aa = 0;
for i=1:1:100000


    [x,y,button] = ginput(1);
    %鼠标左键
    if(button == 1)
        plot(x,y,'bo-','MarkerFaceColor','b');
        
        footStepData(aa+i,1) = x;
        footStepData(aa+i,2) = y;
        footStepData(aa+i,3) = -0.5;
    end
    %鼠标右键
    if(button == 3)
        break;
    end
end

disp(footStepData);


