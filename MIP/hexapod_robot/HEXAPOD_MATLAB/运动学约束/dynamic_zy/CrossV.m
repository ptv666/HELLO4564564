%表示旋量的叉乘矩阵，其中a=1时为正常叉乘，X=[[w],0;[v] [w]];a=2时为*叉乘，X=[[w] [v];0 [w]]
function [X]=CrossV(V,a)
if size(V)~=[6 1]
    return
end
omega=V(1:3);
v=V(4:6);
if nargin==1||a==1
    X=[0 -omega(3) omega(2) 0 0 0;
        omega(3) 0 -omega(1) 0 0 0;
        -omega(2) omega(1) 0 0 0 0;
        0 -v(3) v(2) 0 -omega(3) omega(2);
        v(3) 0 -v(1) omega(3) 0 -omega(1);
        -v(2) v(1) 0 -omega(2) omega(1) 0];
else
    if a==2
        X=[0 -omega(3) omega(2) 0 -v(3) v(2);
            omega(3) 0 -omega(1) v(3) 0 -v(1);
            -omega(2) omega(1) 0 -v(2) v(1) 0;
            0 0 0 0 -omega(3) omega(2);
            0 0 0 omega(3) 0 -omega(1);
            0 0 0 -omega(2) omega(1) 0];
    end
end
end