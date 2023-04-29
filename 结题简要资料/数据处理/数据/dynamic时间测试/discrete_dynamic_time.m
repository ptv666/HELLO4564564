%%离散动力学判断的时间性能
%% 数据准备
consume_time = [0.001226	0.001221	0.001212	0.001248	0.001232    %2
                0.002002	0.001969	0.001985	0.002075	0.001891    %5
                0.002526	0.002536	0.002534	0.002486	0.002395    %7
                0.003169	0.003404	0.003308	0.003204	0.003193    %10
                0.005738	0.005874	0.006065	0.005653	0.005952    %20
                0.013157	0.013797	0.013097	0.013734	0.013445    %50
                0.018434	0.018429	0.018646	0.018214	0.0188      %70
                0.026719	0.026357	0.027391	0.026646	0.026525];  %100
distrete_points = [2; 5; 7; 10; 20; 50; 70; 100];
consume_time = mean(consume_time, 2) * 1000;
sucess_rate = log(distrete_points);   %待写，尝试使用log对数函数
sucess_rate = [0.3495;0.6505;0.8495;0.9225;0.966;0.977;0.988;0.999];
%% 绘图
figure
yyaxis left
hold on
l0 = xline(10.0,'--');
% 绘制计算时间 [0,0.45,0.74]
l1 = plot(distrete_points, consume_time, '-o','Color', [0, 114, 189]/255, 'LineWidth', 2.0, 'MarkerSize', 6, 'MarkerFaceColor', 'red', 'MarkerEdgeColor', 'red');
ylabel('计算时间/ms')
% 绘制成功率 [0.85,0.33,0.1]
yyaxis right
l2 = plot(distrete_points,sucess_rate,'-go','Color',  [217, 83, 25]/255, 'LineWidth', 2.0, 'MarkerSize', 6, 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');
ylim([0,1])
ylabel('成功率')
set(gca,'FontSize',20)  %是设置刻度字体大小
legend('123','456','789')
legend([l1,l2],{'计算时间','成功率'},'location','best')
xlabel('离散点数量')
title('离散判断驱动约束方法性能')