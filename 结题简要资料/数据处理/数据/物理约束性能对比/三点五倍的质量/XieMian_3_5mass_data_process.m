%%处理state_information，3维质心位置、6个足端三维力(18个值)、18个关节角度、18个关节力
%% 读取数据
clear,clc,close all
%定义数据
COG = zeros(0,3);
feet_force = zeros(0,6,3);
joints_angles = zeros(0,6,3);
joints_torque = zeros(0,6,3);
%打开文件

fidin=fopen('E:\_本科毕业设计\结题\数据处理\数据\物理约束性能对比\三点五倍的质量\zz_3_5倍质量_只为运动的中间空_没越界.txt');


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
x_data = linspace(0,100,length(COG));
%% 绘图
close all
%%绘制COG位置图
figure
Font_Size = 20;
Line_width = 1.5;
hold on
grid on 
grid minor
title('质心位置')
plot(x_data,COG(:,1),'LineWidth',Line_width)  %质心x位置
plot(x_data,COG(:,3),'LineWidth',Line_width)  %质心x位置
xlabel('运动时间/s')
ylabel('位置/m')
legend('x坐标','z坐标')

set(gca,'FontSize',Font_Size)  %是设置刻度字体大小
%% 绘制cwc
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
        if Ratio_N_T(i,j)>0.5
            Ratio_N_T(i,j) = 0.49;
        end
    end
end
figure
sgtitle('切向力/法向力','FontSize',20)
scatter_size = 18;
for i = 1:6
    subplot(2,3,i)
    hold on
    yline(0.5,'--')
    scatter(x_data,Ratio_N_T(:,i),scatter_size, 'MarkerEdgeColor','r', 'MarkerFaceColor','r', 'MarkerFaceAlpha',0.5)
    grid on 
    grid minor
    xlabel('运动时间/s')
    hold off
    set(gca,'FontSize',15)  %是设置刻度字体大小
end
%% 绘制关节角度 (有问题！！！)
fig = figure;
Font_Size = 13;
Line_width = 1.5;
% sgtitle('关机角度约束','FontSize',20) %P图再加
for i = 1:6
    angles_plot = zeros(size(joints_angles,1),3);
    for j = 1:size(joints_angles,1)
        angles_plot(j,:) = joints_angles(j,i,:) *180 /pi;
    end
    
    subplot(6,3,3*i-2)  %根关节
    hold on
    grid on
    grid minor
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
    plot(x_data,angles_plot(:,1),'LineWidth',Line_width)
    set(gca,'FontSize',Font_Size)  %是设置刻度字体大小
    
    subplot(6,3,3*i-1)  %髋关节
    hold on
    grid on
    grid minor
    yline(-90,'--')
    yline(90,'--')
    for j = 1:size(angles_plot,1)
        if angles_plot(j,2) > 90
            angles_plot(j,2) = 88;
        end
        if angles_plot(j,2) < -90
            angles_plot(j,2) = -88;
        end
    end
    plot(x_data,angles_plot(:,2),'LineWidth',Line_width)
    set(gca,'FontSize',Font_Size)  %是设置刻度字体大小
    
    subplot(6,3,3*i)    %膝关节
    hold on    
    grid on
    grid minor
    ylim([-72,100])
    yline(-60,'--')
    yline(90,'--')
    for j = 1:size(angles_plot,1)
        if angles_plot(j,3) > 90
            angles_plot(j,1) = 89;
        end
        if angles_plot(j,3) < -60
            angles_plot(j,3) = -59;
        end
    end
    plot(x_data,angles_plot(:,3),'LineWidth',Line_width)
    set(gca,'FontSize',Font_Size)  %是设置刻度字体大小
end
han=axes(fig,'visible','off');
han.Title.Visible='on';
han.XLabel.Visible='on';
han.YLabel.Visible='on';
ylabel(han,'关节角度/°');
xlabel(han,'运动时间/s');
set(gca,'FontSize',Font_Size)  %是设置刻度字体大小
%% 绘制关节力矩
fig = figure;
Font_Size = 15;
Line_width = 1.5;
% sgtitle('关机力矩','FontSize',18)   %P图加上
for i = 1:6
    toque_plot = zeros(size(joints_torque,1),3);
    for j = 1:size(joints_torque,1)
        toque_plot(j,:) = joints_torque(j,i,:);
    end
    
    subplot(6,3,3*i-2)  %根关节
    hold on
%     yline(-640.64,'--')
%     yline(640.64,'--')
    for j=1:size(toque_plot,1)
        if abs(toque_plot(j,1))>150
            toque_plot(j,1) = 0;
        end
    end
    plot(x_data,toque_plot(:,1),'LineWidth',Line_width)
    grid on 
    grid minor
    set(gca,'FontSize',Font_Size)  %是设置刻度字体大小
    
    subplot(6,3,3*i-1)  %髋关节
    hold on
%     yline(-1673.28,'--')
    yline(1673.28,'--')
    plot(x_data,toque_plot(:,2),'LineWidth',Line_width)
    grid on 
    grid minor
%     ylim([0,700])
    set(gca,'FontSize',Font_Size)  %是设置刻度字体大小
    
    subplot(6,3,3*i)    %膝关节
    hold on
%     yline(-1327.04,'--')
%     yline(1327.04,'--')
    plot(x_data,toque_plot(:,3),'LineWidth',Line_width)
    grid on 
    grid minor
    set(gca,'FontSize',Font_Size)  %是设置刻度字体大小
end

han=axes(fig,'visible','off');
han.Title.Visible='on';
han.XLabel.Visible='on';
han.YLabel.Visible='on';
ylabel(han,'扭矩值/Nm');
xlabel(han,'运动时间/s');
set(gca,'FontSize',Font_Size)  %是设置刻度字体大小