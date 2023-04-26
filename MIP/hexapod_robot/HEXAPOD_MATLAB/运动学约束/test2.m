L1 = LEG(2);
point = [];
V = [];
for t1 = -2/9*pi:0.1: 2/9*pi
    for t2 = -pi/2:0.1:pi/2
        for t3 = -5/6*pi:0.1:0
            L1 = L1.setTheta(t1,t2,t3);
            point(end+1,:) = L1.p4_B;
            V(end+1) = hexahedron_V(t1,t2,t3);  %计算每一个足端位置处的六面体体积
        end
    end
end
V = V.';
% m = mean(V);
% FLAG1 = ~(V>300);
% FLAG2 = V>35;
% FLAG = FLAG1 .* FLAG2;
% sum(FLAG)
% save data.txt -ascii point
%% 三维绘图
figure
hold on
S = 10;%标记点的大小
% scatter3(point(FLAG,1)+0.58,point(FLAG,2),point(FLAG,3))
scatter3(point(:,1)+0.58,point(:,2),point(:,3),[],V)
view(0,45)
axis equal
xlabel('x'),ylabel('y'),zlabel('z')