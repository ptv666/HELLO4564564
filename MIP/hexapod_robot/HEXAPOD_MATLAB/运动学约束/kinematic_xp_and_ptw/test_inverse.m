for i = 1:length(x)
    theta = zeros(1,3);
    point = [x(i),y(i),z(i)]
    theta = inverse_FixJi(point);
    (compute_P(theta(1),theta(2),theta(3)))'

    (theta*180/pi)'
    input('输入一个字符继续')
end
