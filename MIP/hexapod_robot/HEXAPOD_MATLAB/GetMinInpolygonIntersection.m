%% 获取点到多边形内部的最短距离,用于计算支撑多边形的稳定裕度
function [point2,margin] = GetMinInpolygonIntersection(IntersectionData,center)
%     static tempD ;
    x = IntersectionData(1,:);
    y = IntersectionData(2,:);
    if( inpolygon(center(1),center(2),x,y) == 1 )
        for i=1:1:size(x,2)
            p1 = [x(i),y(i)];
            if i == size(x,2)
                p2 = [x(1),y(1)];
            else
                p2 = [x(i+1),y(i+1)];
            end
            base = p1-p2;
            r = dot(base,center-p2)/norm(base);
            e = base/norm(base);
            pr = p2 + (e*r);%垂直投影点

            data(i) = sqrt(norm(center-p2)^2 - norm(r)^2);
            dataP(i,:) = pr;
        end

        [margin,num] = min(data);

        point2 = dataP(num,:);


    else
        aaaaa=1;
%          error("out of support polygon");
    end
    
end