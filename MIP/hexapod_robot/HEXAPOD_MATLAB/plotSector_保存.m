%���Ƶ�����������
%����ΪʲôҪ������ת�任��
%����rotz������Ҫ����Ƕȶ����ǻ���ѽ
function line = plotSector_SAVE()%theta,R,footP
    footP = [0,0,-0.5]; %���������е�����㶼������z=-0.5��λ����
    R = 1;  %���ȹ����ռ�����ΰ뾶Ϊ1m
    theta = 30/180*pi;
%     theta = 30;
    Rz = rotz(theta);
    
    %����Բ��
    aplha= pi/4:pi/40:3*pi/4;   %�����Ž�Ϊ90�㣬��pi/4��3/4pi
    x=footP(1) + R*cos(aplha);
    y=footP(2) + R*sin(aplha);
    %��������ֱ��
    x = [footP(1),x,footP(1)];
    y = [footP(2),y,footP(1)];
    z = ones(1,size(x,2)) * footP(3);
    data = Rz*[x;y;z]; %P2����ϵ�µ�����
    plot3(data(1,:),data(2,:),data(3,:),'b-');
    hold on;
    axis equal;

end


