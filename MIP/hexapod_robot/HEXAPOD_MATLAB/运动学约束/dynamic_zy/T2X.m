function [AdT]=T2X(T)
if size(T)~=[4 4]
    return;
end
R=T(1:3,1:3);
p=T(1:3,4);
px=[0 -p(3) p(2);
    p(3) 0 -p(1);
    -p(2) p(1) 0];
AdT=[R zeros(3);
    px*R R];
end