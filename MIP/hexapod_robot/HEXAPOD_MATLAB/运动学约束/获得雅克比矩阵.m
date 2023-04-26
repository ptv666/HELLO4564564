%% 计算雅克比矩阵（旧版model）
syms t1 t2 t3 a2 Lc Lt Ls
T_0_1 = DH2T(0,0,0,t1);
T_1_2 = subs(DH2T(a2,Lc,0,t2),a2,pi/2);

T_2_3 = DH2T(sym(0),Lt,sym(0),t3);
T_3_4 =  DH2T(sym(0),Ls,0,sym(0));

T = simplify(T_0_1 * T_1_2 * T_2_3 * T_3_4);
P = T(1:3,4)
J11 = diff(P(1),t1);
J12 = diff(P(1),t2);
J13 = diff(P(1),t3);

J21 = diff(P(2),t1);
J22 = diff(P(2),t2);
J23 = diff(P(2),t3);

J31 = diff(P(3),t1);
J32 = diff(P(3),t2);
J33 = diff(P(3),t3);
J = [J11,J12,J13;J21,J22,J23;J31,J32,J33]

%%
T = sub