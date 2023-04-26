function [dT]=dS2T(S,theta)
if size(S)~= [6 1]
    return
end
omega=[S(1) S(2) S(3)].';
v=[S(4) S(5) S(6)].';
omegax=[0 -omega(3) omega(2);
        omega(3) 0 -omega(1);
        -omega(2) omega(1) 0];
dR=cos(theta)*omegax+sin(theta)*omegax^2;
dp=(diag([1,1,1])+sin(theta)*omegax+(1-cos(theta))*omegax^2)*v;
dT=[dR dp;
    0 0 0 1];
end