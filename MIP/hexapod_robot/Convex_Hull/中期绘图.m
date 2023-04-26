%% 中期绘图，绘制一个二维空间的闵式和可视化结果
A = [0,0;1,1;-1,1];
B = [0,0;1,2;2,0];
tic
C = haha(A,B);
toc
%% 把凸包C的顶点逆时针排序
CX = C(:,1);
CY = C(:,2);
cx = mean(CX);
cy = mean(CY);
a = atan2(CY - cy, CX - cx);
[~, order] = sort(a);
C(:,1) = CX(order);
C(:,2) = CY(order);
%% 绘图
AA = A;
AA(end+1,:) = A(1,:);
BB = B;
BB(end+1,:) = B(1,:);
CC = C;
CC(end+1,:) = C(1,:);

figure
axis equal
hold on
fill(CC(:,1),CC(:,2),'g','FaceAlpha',0.3)
plot(CC(:,1),CC(:,2),'LineWidth',2)
plot(AA(:,1),AA(:,2),'--','LineWidth',2.5)
plot(BB(:,1),BB(:,2),'--','LineWidth',2.5)
% fill(BB(:,1),BB(:,2),'r','FaceAlpha',0.3)
% fill(AA(:,1),AA(:,2),'cyan','FaceAlpha',0.3)
% legend('闵式和P3','凸包P1','凸包P2')
% title('二维凸包闵式和')
xlim([-1.2,3.2])
ylim([-0.2,3.2])
hold off