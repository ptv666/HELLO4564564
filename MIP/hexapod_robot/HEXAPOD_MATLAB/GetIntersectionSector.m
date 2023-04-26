%% 线与扇形的交点， 计算运动学稳定裕度
%依赖于函数GetLineAndCircleIntersection求解直线和圆相交
%依赖于函数GetTwoVectorIntersection求解两直线相交
function resultP =  GetIntersectionSector(R,x0,y0,n)
%输入：扇形的半径，圆形坐标，前进方向
    p_foot = [x0,y0];
    p1 = [R*cos(3*pi/4),R*sin(3*pi/4)];
    p2 = [R*cos(pi/4),R*sin(pi/4)];
    
    theta1 = atan2(p1(2)-y0,p1(1)-x0);
    if (theta1<0), theta1 = theta1 + 2*pi; end;
    
    theta2 = atan2(p2(2)-y0,p2(1)-x0);
    if (theta2<0), theta2 = theta2 + 2*pi; end;
    
    theta3 = atan2(0-y0,0-x0);
    if (theta3<0), theta3 = theta3 + 2*pi; end;
    
    theta = atan2(n(2) ,n(1));
    if (theta<0), theta = theta + 2*pi; end;
    
    if(y0 <= sqrt(2)/2*R) %与扇形的角度有关
        
        if (theta > theta2 && theta < theta1) %与圆弧相交
            resultP = GetLineAndCircleIntersection(R,x0,y0,n);
        elseif (theta >= theta1 && theta < theta3)
            resultP = GetTwoVectorIntersection(p_foot,p_foot+n,[0,0],p1);
        else
            resultP = GetTwoVectorIntersection(p_foot,p_foot+n,[0,0],p2);
        end
    else

        if (theta >= theta1 && theta < theta3)
            resultP = GetTwoVectorIntersection(p_foot,p_foot+n,[0,0],p1);
        elseif (theta >= theta3 && theta <= theta2)
            resultP = GetTwoVectorIntersection(p_foot,p_foot+n,[0,0],p2);
        else
            resultP = GetLineAndCircleIntersection(R,x0,y0,n);
        end
    end

    
end



