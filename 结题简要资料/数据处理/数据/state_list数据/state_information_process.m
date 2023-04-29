%%处理state_information，3维质心位置、6个足端三维力(18个值)、18个关节角度、18个关节力
%% 读取数据
clear,clc,close all
%定义数据
COG = zeros(0,3);
feet_force = zeros(0,6,3);
joints_angles = zeros(0,6,3);
joints_torque = zeros(0,6,3);
%打开文件

% fidin=fopen('E:\_本科毕业设计\结题\数据处理\数据\state_list数据\沟濠_好的结果_十个点一轮.txt');
% fidin=fopen('E:\_本科毕业设计\结题\数据处理\数据\state_list数据\沟濠_CWC超限结果_十个点一轮.txt');
% fidin=fopen('E:\_本科毕业设计\结题\数据处理\数据\state_list数据\随机大块_好的结果_十个点一轮.txt');
% fidin=fopen('E:\_本科毕业设计\结题\数据处理\数据\state_list数据\随机大块_CWC超限_十个点一轮.txt');
% fidin=fopen('E:\_本科毕业设计\结题\数据处理\数据\state_list数据\中间空_只为运动_很意外_好的结果_十个点一轮.txt');    %这个的cwc需要手动剔除坏点
% fidin=fopen('E:\_本科毕业设计\结题\数据处理\数据\state_list数据\中间空_只为运动_CWC超限_十个点一轮.txt');
fidin=fopen('E:\_本科毕业设计\结题\数据处理\数据\state_list数据\中间空_满足约束_好的结果_十个点一轮.txt');      %这个的cwc需要手动剔除坏点
% fidin=fopen('E:\_本科毕业设计\结题\数据处理\数据\state_list数据\中间空_完整约束_CWC_超限_十个点一轮.txt');

fidout=fopen('mkmatlab.txt','w');                       % 创建MKMATLAB.txt文件
tline=fgetl(fidin);
while ~feof(fidin)                                      % 判断是否为文件末尾               
    tline=fgetl(fidin);                                 % 从文件读行   
    fprintf(fidout,'%s\n',tline);
end
fclose(fidout);
MK=importdata('MKMATLAB.txt');
state_index = 1;
data_index = 0;
for line = 1:size(MK,1)
% for line = 1:8
    data_index = data_index + 1;
    temp = str2num(MK{line});
    switch data_index
        case 1  %质心位置
%             disp('质心位置')
            COG(end+1,:) = temp;
            
        case 2  %足地力
%             disp('足地力')
            feet_force(end+1,:) = 0;
            for i = 1:6
                temp2 = temp(3*i-2:3*i);
                feet_force(end,i,:) = temp2;
            end
            
        case 3  %关节角度
%             disp('关节角度')
            joints_angles(end+1,:) = 0;
            for i = 1:6
                temp2 = temp(3*i-2:3*i);
                joints_angles(end,i,:) = temp2;
            end
            
        case 4  %关节力矩
            data_index = 0;
%             disp('关节力矩')
            joints_torque(end+1,:) = 0;
            for i = 1:6
                temp2 = temp(3*i-2:3*i);
                joints_torque(end,i,:) = temp2;
            end
    end
end

%% 绘图
close all
%%绘制COG位置图
figure
hold on
plot(COG(:,1))  %质心x位置
plot(COG(:,3))  %质心z位置

%%绘制cwc
%求解法向力和切向力
Ratio_N_T = zeros(size(feet_force,1),6); %切向力和法向力的比值
N = [-sind(15);0;cosd(15)];

for i = 1:size(feet_force,1)
    for j = 1:6
%         sprintf('j = %d',j)
        force = reshape(feet_force(i,j,:),1,3); %足地力，1×3
        temp = force*N;             %法向力的大小
        force_N = temp*N';          %法向力
        force_T = force - force_N;  %切向力
        if temp < 0.01
            continue;
        end
        Ratio_N_T(i,j) = norm(force_T) / temp;  %切向力大小/法向力大小
    end
end
figure
scatter_size = 30;
for i = 1:6
    subplot(2,3,i)
    hold on
    yline(0.5,'--')
    temp = 1:size(Ratio_N_T,1);
    scatter(temp,Ratio_N_T(:,i),scatter_size, 'MarkerEdgeColor','r', 'MarkerFaceColor','r', 'MarkerFaceAlpha',0.5)
%     plot(Ratio_N_T(:,i))
    hold off
end

%%绘制关节角度 (有问题！！！)
figure
sgtitle('关机角度约束','FontSize',18)
for i = 1:6
    angles_plot = zeros(size(joints_angles,1),3);
    for j = 1:size(joints_angles,1)
        angles_plot(j,:) = joints_angles(j,i,:) *180 /pi;
    end
    
    subplot(6,3,3*i-2)  %根关节
    hold on
    yline(-40,'--')
    yline(40,'--')
    for j = 1:size(angles_plot,1)
        if angles_plot(j,1) > 40
            angles_plot(j,1) = 39;
        end
        if angles_plot(j,1) < -40
            angles_plot(j,1) = -39;
        end
    end
    plot(angles_plot(:,1))
    subplot(6,3,3*i-1)  %髋关节
    hold on
    yline(-90,'--')
    yline(90,'--')
    plot(angles_plot(:,2))
    subplot(6,3,3*i)    %膝关节
    hold on
    yline(-60,'--')
    yline(90,'--')
    plot(angles_plot(:,3))
end

%%绘制关节力矩
figure
sgtitle('关机力矩约束','FontSize',18)
for i = 1:6
    toque_plot = zeros(size(joints_torque,1),3);
    for j = 1:size(joints_torque,1)
        toque_plot(j,:) = joints_torque(j,i,:);
    end
    
    subplot(6,3,3*i-2)  %根关节
    hold on
    yline(-640.64,'--')
    yline(640.64,'--')
    plot(toque_plot(:,1))
    subplot(6,3,3*i-1)  %髋关节
    hold on
    yline(-1673.28,'--')
    yline(1673.28,'--')
    plot(toque_plot(:,2))
    subplot(6,3,3*i)    %膝关节
    hold on
    yline(-1327.04,'--')
    yline(1327.04,'--')
    plot(toque_plot(:,3))
end