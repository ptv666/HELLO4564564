%机器人学中的两个坐标系之间的齐次变换矩阵
function D = matrixT_SAVE(theta,afa,l,d)
    D = [
        cos(theta), -sin(theta),0,l;
        cos(afa)*sin(theta), cos(afa)*cos(theta), -sin(afa),-d*sin(afa);
        sin(afa)*sin(theta), sin(afa)*cos(theta), cos(afa) , d*cos(afa);
        0 , 0 , 0, 1;
        ];
end