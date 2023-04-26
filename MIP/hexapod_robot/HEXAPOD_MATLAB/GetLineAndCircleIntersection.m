function [P1,P2] = GetLineAndCircleIntersection(R,x0,y0,n)
%���ֱ�ߺ�Բ�Ľ���
    %��λ����
    e = n/norm(n);
    b = [x0,y0];
    c = b + e;
    a = [0,0];%Բ������
    base = c-b;
    r = dot(base,a-b)/norm(base);
    
    pr = b + (base*r);%Բ�ĵ�ֱ���ϵ�ͶӰ
%     plot(pr(1),pr(2),'r^');

    base = sqrt(R^2 - norm(pr-a)^2 );

    
    
    
    P1 = pr + e*base;

end