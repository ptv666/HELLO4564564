%test_my_Minkowski_sum
%% 01:简单的wiki
A = [1,0;0,1;0,-1];
B = [0,0;1,1;1,-1];
tic
C = Minkowski_sum(A,B);
toc
%% 02：notion中
A = [0,0;-3,1;2,3];
B = [0,0;-2,3;0,4;1,1];
tic
C = Minkowski_sum(A,B);
toc
%% 03：ubuntu中测试
A = [1,0;0,1;1,1];
B = [1,2;1,3;2,2];
tic
C = Minkowski_sum(A,B);
toc
%% 04：四棱锥的测试
A = [0,0,0;
     1,1,1;
     -1,1,1;
     -1,-1,1;
     1,-1,1];
B = A;
tic
C = Minkowski_sum(A,B);
toc
%% 
hold on
plot(A(:,1),A(:,2),'*')
plot(B(:,1),B(:,2),'o')
plot(C(:,1),C(:,2),'^')