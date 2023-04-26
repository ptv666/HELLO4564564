function point = computer_P(t1,t2,t3)
%计算足端位置相对于根关节的位置（旧版model）
    point = zeros(3,1);
    point(1) = cos(t1)*(0.18 + 0.5*cos(t2 + t3) + 0.5*cos(t2));
    point(2) = sin(t1)*(0.18 + 0.5*cos(t2 + t3) + 0.5*cos(t2));
    point(3) = 0.5*sin(t2 + t3) + 0.5*sin(t2);
end