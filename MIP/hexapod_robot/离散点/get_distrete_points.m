%% 直线前进_均匀_横向冗余_82点
X = 0:0.05:2;
Y1 = ones(size(X)) * 0.25;
Y2 = ones(size(X)) * -0.25;
XY = [[X',Y1'];[X',Y2']];
%% 直线前进_均匀_横纵向冗余_246点
X = 0:0.05:2;
Y1 = ones(size(X)) * 0.25;
Y2 = ones(size(X)) * -0.25;
XY = [[X',Y1'];[X',Y2']];
XY1 = XY; XY2 = XY;
XY1(:,2) = XY1(:,2) + 0.05;
XY2(:,2) = XY2(:,2) - 0.05;
XY = [XY;XY1;XY2];