%% 对比直接求逆法和凸包近似法速度大小绘图
%点的数量
points_num = [441, 961, 1600, 2500, 3721,4900,6400];
A = points_num;
A(end + 1,:) = 0;
A = A.';
%求逆法时间
times_1 = [0.010284,0.022446,0.037013,0.056134,0.083282,0.109366,0.144116];
ab1 = A\(1000 * times_1.');
%凸包法时间
times_2 = [0.000331,0.000715,0.001173,0.001882,0.002779,0.003708,0.004778];
ab2 = A\(1000 * times_2.');
%绘图
f = figure
hold on
l1 = plot(points_num, A * ab1, 'LineWidth', 1.5);
l2 = plot(points_num, A * ab2, 'LineWidth', 1.5);

s1 = plot(points_num,times_1 * 1000, 'o','MarkerFaceColor','y', 'MarkerSize',12);
s2 = plot(points_num,times_2 * 1000, 'o','MarkerFaceColor','g', 'MarkerSize',12);

legend([l1 l2],{'直接求逆法','凸包近似法'},'location','best')
xlabel('计算点数')
ylabel('花费时间/ms')
xlim([0 6600])
set(gca,'FontSize',20)  %是设置刻度字体大小
% title('摆动腿运动可达算法时间性能对比')








