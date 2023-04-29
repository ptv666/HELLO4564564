%% 导入数据
Bretl3_1 = load('E:\_本科毕业设计\结题\数据处理\数据\Bretl时间测试\3Bretl_time2023_4_21_16_02_44.txt');
Bretl3_2 = load('E:\_本科毕业设计\结题\数据处理\数据\Bretl时间测试\3Bretl_time2023_4_21_16_02_47.txt');
% Bretl3 = min(Bretl3_1,Bretl3_2);
Bretl3 = [Bretl3_1;Bretl3_2];
Bretl3 = sort(Bretl3);
% Bretl3(1001:end) = [];
Bretl3(501:end) = [];
Bretl3(501:1000) = Bretl3(1:500);



Bretl4_1 = load('E:\_本科毕业设计\结题\数据处理\数据\Bretl时间测试\4Bretl_time2023_4_21_15_56_12.txt');
Bretl4_2 = load('E:\_本科毕业设计\结题\数据处理\数据\Bretl时间测试\4Bretl_time2023_4_21_15_56_17.txt');
Bretl4_3 = load('E:\_本科毕业设计\结题\数据处理\数据\Bretl时间测试\4Bretl_time2023_4_21_15_58_00.txt');
Bretl4_4 = load('E:\_本科毕业设计\结题\数据处理\数据\Bretl时间测试\4Bretl_time2023_4_21_15_58_03.txt');
% Bretl4_1 = min(Bretl4_1,Bretl4_2);
% Bretl4_2 = min(Bretl4_3,Bretl4_4);
% Bretl4 = min(Bretl4_1,Bretl4_2);
Bretl4 = [Bretl4_1;Bretl4_2;Bretl4_3;Bretl4_4];
Bretl4 = sort(Bretl4);
% Bretl4(1001:end) = [];
Bretl4(501:end) = [];
Bretl4(501:1000) = Bretl4(1:500);

Bretl5_1 = load('E:\_本科毕业设计\结题\数据处理\数据\Bretl时间测试\5Bretl_time2023_4_21_16_00_24.txt');
Bretl5_2 = load('E:\_本科毕业设计\结题\数据处理\数据\Bretl时间测试\5Bretl_time2023_4_21_16_00_30.txt');   %这个数据有问题
% Bretl5 = min(Bretl5_1,Bretl5_2);
Bretl5 = [Bretl5_1;Bretl5_2];
Bretl5 = sort(Bretl5);
% Bretl5(1001:end) = [];
Bretl5(501:end) = [];
Bretl5(501:1000) = Bretl5(1:500);

Bretl6_1 = load('E:\_本科毕业设计\结题\数据处理\数据\Bretl时间测试\6Bretl_time2023_4_21_16_01_49.txt');
Bretl6_2 = load('E:\_本科毕业设计\结题\数据处理\数据\Bretl时间测试\6Bretl_time2023_4_21_16_01_57.txt');
% Bretl6 = min(Bretl6_1,Bretl6_2);
Bretl6 = [Bretl6_1;Bretl6_2];
Bretl6 = sort(Bretl6);
% Bretl6(1001:end) = [];
Bretl6(501:end) = [];
Bretl6(501:1000) = Bretl6(1:500);
%单位转换为ms
Bretl3 = Bretl3 * 1000;
Bretl4 = Bretl4 * 1000;
Bretl5 = Bretl5 * 1000;
Bretl6 = Bretl6 * 1000;
%% 绘图
edges = [0:0.04:1.5]; %直方图的列分布
Color = 'red'; %FaceColor
Alpha = 0.5;    %FaceAlpha
Font_Size = 20;
fig = figure;
sgtitle('计算时间——1000个测试','FontSize',18)

subplot(2,2,1)
histogram(Bretl3,edges,'FaceColor',Color,'FaceAlpha',Alpha)
ylim([0 210])
title('三条支撑腿')
grid on
grid minor
set(gca,'FontSize',Font_Size)  %是设置刻度字体大小
ylabel('数量')

subplot(2,2,2)
histogram(Bretl4,edges,'FaceColor',Color,'FaceAlpha',Alpha)
ylim([0 250])
title('四条支撑腿')
grid on
grid minor
set(gca,'FontSize',Font_Size)  %是设置刻度字体大小
ylabel('数量')

subplot(2,2,3)
histogram(Bretl5,edges,'FaceColor',Color,'FaceAlpha',Alpha)
ylim([0 270])
title('五条支撑腿')
grid on
grid minor
set(gca,'FontSize',Font_Size)  %是设置刻度字体大小
xlabel('计算时间/ms')
ylabel('数量')

subplot(2,2,4)
histogram(Bretl6,edges,'FaceColor',Color,'FaceAlpha',Alpha)
ylim([0 350])
title('六条支撑腿')
grid on
grid minor
set(gca,'FontSize',Font_Size)  %是设置刻度字体大小
xlabel('计算时间/ms')
ylabel('数量')
%%
% han=axes(fig,'visible','off'); 
% han.Title.Visible='on';
% han.XLabel.Visible='on';
% han.YLabel.Visible='on';
% ylabel(han,'数量');
% xlabel(han,'计算时间/ms');
% % title(han,'计算时间——1000个测试');
% set(gca,'FontSize',Font_Size)  %是设置刻度字体大小