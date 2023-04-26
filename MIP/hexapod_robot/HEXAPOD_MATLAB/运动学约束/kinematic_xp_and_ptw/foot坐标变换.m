%% 计算fixJi到foot的坐标变化
syms t1 t2 t3 a2 L1 L2 L3
T_0_1 = DH2T(0,0,0,t1);
T_1_2 = subs(DH2T(a2,L1,0,t2),a2,pi/2);
C_T_2_3 = subs(DH2T(sym(0),L2,sym(0),t3),t3,-pi/2); %先绕着z转动-90°，平移
T_2_3 = C_T_2_3 * DH2T(0,0,0,t3);   %再考虑关节角度t3
T_3_4 = [0, 0,  1, L3;
         0, 1,  0,  0;
        -1, 0,  0,  0;
         0, 0,  0,  1];

T = simplify(T_0_1 * T_1_2 * T_2_3 * T_3_4)
T = subs(T,{L1,L2,L3},{0.18,0.5,0.5})
TT = subs(T,{t1,t2,t3},{0,0,0})

%% 求逆
R = TT(1:3,1:3);
p = TT(1:3,4);
T_inverse = eye(4);
T_inverse(1:3,1:3) = R';
T_inverse(1:3,4) = -R' * p