%获得T_B_leg，单腿根关节相对base的T
function D = matrixBody_Leg_SAVE(theta)
%机器人base正六边形边长为0.4m
    D = [
        cos(theta), -sin(theta),0,cos(theta)*0.4;
        sin(theta), cos(theta),0,sin(theta)*0.4;
        0, 0, 1 , 0;
        0 , 0 , 0, 1;
        ];
end