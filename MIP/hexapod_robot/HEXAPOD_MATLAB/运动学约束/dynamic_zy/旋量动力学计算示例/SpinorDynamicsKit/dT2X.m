function [dX]=dT2X(T,dT)
if size(T)~=[4 4]    
    return;
end
R=T(1:3,1:3);
p=T(1:3,4);
dR=dT(1:3,1:3);
dp=dT(1:3,4);
px=[0 -p(3) p(2);
    p(3) 0 -p(1);
    -p(2) p(1) 0];
dpx=[0 -dp(3) dp(2);
    dp(3) 0 -dp(1);
    -dp(2) dp(1) 0];
dX=[dR zeros(3);
    dpx*R+px*dR dR];
end