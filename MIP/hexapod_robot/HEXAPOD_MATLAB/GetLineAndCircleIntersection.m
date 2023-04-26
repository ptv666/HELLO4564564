function [P1,P2] = GetLineAndCircleIntersection(R,x0,y0,n)
%获得直线和圆的交点
    %求单位向量
    e = n/norm(n);
    b = [x0,y0];
    c = b + e;
    a = [0,0];%圆心坐标
    base = c-b;
    r = dot(base,a-b)/norm(base);
    
    pr = b + (base*r);%圆心到直线上的投影
%     plot(pr(1),pr(2),'r^');

    base = sqrt(R^2 - norm(pr-a)^2 );

    
    
    
    P1 = pr + e*base;

end