%单腿动力学
%Sclear;
syms L1 L2 L3 m1 m2 m3 theta1 theta2 theta3 dtheta1 dtheta2 dtheta3 ddtheta1 ddtheta2 ddtheta3 ...
    J1x J1y J1z J2x J2y J2z J3x J3y J3z g J1xy J1zx J1yz J2xy J2zx J2yz J3xy J3zx J3yz ...
    xc1 yc1 zc1 xc2 yc2 zc2 xc3 yc3 zc3;
S1=[0 0 1 0 0 0].';
S2=[0 -1 0 0 0 L1].';
S3=[0 -1 0 0 0 L1+L2].';
%2和3坐标系下各自的转动轴旋量
S22=[0 0 1 0 0 0].';
S33=[0 0 1 0 0 0].';
%静态齐次变换矩阵（没有theta角度）
M01=diag([1 1 1 1]);
M02=[1 0 0 L1;
     0 0 -1 0;
     0 1 0 0;
     0 0 0 1];
M03=[0 1  0   L1+L2;
     0 0 -1   0;
    -1 0  0   0;
     0 0  0   1];
M0e=[0 1 0   L1+L2;
     1 0 0      0;
     0 0 -1     -L3;
     0 0 0      1];%末端坐标系
%齐次变换矩阵（加入theta角度）
T01=S2T(S1,theta1);
T02=S2T(S1,theta1)*S2T(S2,theta2)*M02;
T03=S2T(S1,theta1)*S2T(S2,theta2)*S2T(S3,theta3)*M03;
T0e=S2T(S1,theta1)*S2T(S2,theta2)*S2T(S3,theta3)*M0e;

T12=T01\T02;T23=T02\T03;
X10=T2X(inv(T01)); X21=T2X(inv(T12)); X32=T2X(inv(T23));
X30=T2X(inv(T03));X20=T2X(inv(T02));


V0=[0 0 0 0 0 0].';
A0=[0 0 0 0 0 0].';
V1=X10*V0+S1*dtheta1;
A1=X10*A0+CrossV(V1)*S1*dtheta1+S1*ddtheta1;
V2=X21*V1+S22*dtheta2;
A2=X21*A1+CrossV(V2)*S22*dtheta2+S22*ddtheta2;
V3=X32*V2+S33*dtheta3;
A3=X32*A2+CrossV(V3)*S33*dtheta3+S33*ddtheta3;

A0g=[0 0 0 0 0 -g].';


I1c=[J1x J1xy J1zx 0 0 0;
     J1xy J1y J1yz 0 0 0;
     J1zx J1yz J1z 0 0 0;
     0 0 0 m1 0 0;
     0 0 0 0 m1 0;
     0 0 0 0 0 m1];%1杆质心坐标系下的惯量
I2c=[J2x J2xy J2zx 0 0 0;
     J2xy J2y J2yz 0 0 0;
     J2zx J2yz J2z 0 0 0;
     0 0 0 m2 0 0;
     0 0 0 0 m2 0;
     0 0 0 0 0 m2];%2杆质心坐标系下的惯量
I3c=[J3x J3xy J3zx 0 0 0;
     J3xy J3y J3yz 0 0 0;
     J3zx J3yz J3z 0 0 0;
     0 0 0 m3 0 0;
     0 0 0 0 m3 0;
     0 0 0 0 0 m3];%3杆质心坐标系下的惯量
 
%sw模型坐标系在运动学坐标系下的位姿
T1m=[1 0 0 L1/2;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];
T2m=[1 0 0 L2/2;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];
T3m=[1 0 0 L3/2;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];
%质心坐标系在模型坐标系下的位姿
Tmc1=eye(4);
Tmc2=eye(4);
Tmc3=eye(4);
%质心坐标系在运动学坐标系下的位姿
T1c=T1m*Tmc1;
T2c=T2m*Tmc2;
T3c=T3m*Tmc3;

Xc1=T2X(inv(T1c));%1杆坐标系在1杆质心坐标系下的位姿
Xc2=T2X(inv(T2c));
Xc3=T2X(inv(T3c));
I1=Xc1.'*I1c*Xc1;
I2=Xc2.'*I2c*Xc2;
I3=Xc3.'*I3c*Xc3;

F3=I3*A3+CrossV(V3,2)*I3*V3-I3*X30*A0g;
F2=I2*A2+CrossV(V2,2)*I2*V2-I2*X20*A0g+X32.'*F3;
F1=I1*A1+CrossV(V1,2)*I1*V1-I1*X10*A0g+X21.'*F2;
% %F3=-I3*X30*A0g;
% F2=-I2*X20*A0g+X32.'*F3;
% F1=I1*A1+CrossV(V1,2)*I1*V1-I1*X10*A0g;%

%%
t3=simplify(S33.'*F3)
t2=simplify(S22.'*F2)
t1=simplify(S1.'*F1)
            
tt1 = subs(t1,   {L1,L2,L3,      J1x,J1y,J1z,J2x,J2y,J2z,J3x,J3y,J3z,                                    dtheta1,dtheta2,dtheta3,ddtheta1,ddtheta2,ddtheta3},...
                {0.18,0.5,0.5,  0.011212,0.03169,0.03169,0.07020,0.7228,0.7228,0.0229,0.14949,0.14949,  0,0,0,0,0,0})
tt2 = subs(t2,   {L1,L2,L3,      J1x,J1y,J1z,J2x,J2y,J2z,J3x,J3y,J3z,                                    dtheta1,dtheta2,dtheta3,ddtheta1,ddtheta2,ddtheta3},...
                {0.18,0.5,0.5,  0.011212,0.03169,0.03169,0.07020,0.7228,0.7228,0.0229,0.14949,0.14949,  0,0,0,0,0,0})
tt3 = subs(t3,   {L1,L2,L3,      J1x,J1y,J1z,J2x,J2y,J2z,J3x,J3y,J3z,                                    dtheta1,dtheta2,dtheta3,ddtheta1,ddtheta2,ddtheta3},...
                {0.18,0.5,0.5,  0.011212,0.03169,0.03169,0.07020,0.7228,0.7228,0.0229,0.14949,0.14949,  0,0,0,0,0,0})


  
t1 = subs(t1,   {L1,L2,L3,      m1,m2,m3,               J1x,J1y,J1z,J2x,J2y,J2z,J3x,J3y,J3z,                                    dtheta1,dtheta2,dtheta3,ddtheta1,ddtheta2,ddtheta3},...
                {0.18,0.5,0.5,  3.6031,21.9855,7.4018,	0.011212,0.03169,0.03169,0.07020,0.7228,0.7228,0.0229,0.14949,0.14949,	0,0,0,0,0,0})
t2 = subs(t2,   {L1,L2,L3,      m1,m2,m3,               J1x,J1y,J1z,J2x,J2y,J2z,J3x,J3y,J3z,                                    dtheta1,dtheta2,dtheta3,ddtheta1,ddtheta2,ddtheta3},...
                {0.18,0.5,0.5,  3.6031,21.9855,7.4018, 	0.011212,0.03169,0.03169,0.07020,0.7228,0.7228,0.0229,0.14949,0.14949,  0,0,0,0,0,0})
t3 = subs(t3,   {L1,L2,L3,      m1,m2,m3,               J1x,J1y,J1z,J2x,J2y,J2z,J3x,J3y,J3z,                                    dtheta1,dtheta2,dtheta3,ddtheta1,ddtheta2,ddtheta3},...
                {0.18,0.5,0.5,  3.6031,21.9855,7.4018,	0.011212,0.03169,0.03169,0.07020,0.7228,0.7228,0.0229,0.14949,0.14949,  0,0,0,0,0,0})

vpa(t1,10),vpa(t2,10),vpa(t3,10)