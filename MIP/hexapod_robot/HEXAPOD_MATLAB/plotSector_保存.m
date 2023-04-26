%绘制单腿扇形区域
%这里为什么要进行旋转变换？
%而且rotz函数需要传入角度而不是弧度呀
function line = plotSector_SAVE()%theta,R,footP
    footP = [0,0,-0.5]; %程序中所有的落足点都设置在z=-0.5的位置上
    R = 1;  %单腿工作空间的扇形半径为1m
    theta = 30/180*pi;
%     theta = 30;
    Rz = rotz(theta);
    
    %扇形圆弧
    aplha= pi/4:pi/40:3*pi/4;   %扇形张角为90°，从pi/4到3/4pi
    x=footP(1) + R*cos(aplha);
    y=footP(2) + R*sin(aplha);
    %扇形两段直线
    x = [footP(1),x,footP(1)];
    y = [footP(2),y,footP(1)];
    z = ones(1,size(x,2)) * footP(3);
    data = Rz*[x;y;z]; %P2坐标系下的坐标
    plot3(data(1,:),data(2,:),data(3,:),'b-');
    hold on;
    axis equal;

end


