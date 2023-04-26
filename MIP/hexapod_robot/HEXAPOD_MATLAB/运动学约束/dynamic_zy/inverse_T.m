function iT = inverse_T(T)
%求解齐次变换矩阵的逆矩阵
    assert(all(size(T) == [4 4]))
    R = T(1:3,1:3);
    p = T(1:3,4);
    assert(abs(sum(sum(abs(R * (R.') - eye(3))))) < 1e-15)
    assert(abs(sum(sum(abs((R.') * R - eye(3))))) < 1e-15)
    iT = eye(4);
    iT(1:3,1:3) = R.';
    iT(1:3,4) = -(R.') * p;