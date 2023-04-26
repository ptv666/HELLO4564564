function D = matrixA(theta,afa,l,d)
%齐次变换矩阵
    D = [
        cos(theta), -cos(afa)*sin(theta),sin(afa)*sin(theta),l*cos(theta);
        sin(theta), cos(afa)*cos(theta), -sin(afa)*cos(theta),l*sin(theta);
        0, sin(afa), cos(afa) , d;
        0 , 0 , 0, 1;
        ];


end