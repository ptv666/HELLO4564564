%�������ߵļ���:�ڶ��ȵ���㡢�м�㡢�յ㡢����ʱ�䡢��Ҫ�����ʱ��
function PlanningPoint=TrajectoryPlanning6(p1,p2,p3,t3,CurrentTime) %CurrentTime���ڵ�ǰ���������е�ʱ�� t3�ǳ���ʱ�� ������������
%����λ��p1(�����)���ٶ�v1=0�����ٶ�a1=0
v1=[0,0,0]; a1=[0,0,0]; t1=0;   %����ʱ��Ϊ0
%�յ�(�����)��λ��p3(�����)���ٶ�v3=0�����ٶ�a3=0
v3=[0,0,0];a3=[0,0,0];
%�м���ʱ��Ϊʱ���е�
t2=(t1+t3)/2; %��Ӧʱ��Ϊt2
%�滮�Ĺ켣����ʽ
%P=Ar0+Ar1*t+Ar2*t^2+Ar3*t^3+Ar4*t^4+Ar5*t^5+Ar6*t^6;��Ҫ���߸�������������߸�����
MatrixCanshu=[1 0 0 0 0 0 0;                    %t1λ��
              1 t2 t2^2 t2^3 t2^4 t2^5 t2^6;    %t2λ��
              1 t3 t3^2 t3^3 t3^4 t3^5 t3^6;    %t3λ��
              0 1 0 0 0 0 0;                    %t1�ٶ�
              0 1 2*t3 3*t3^2 4*t3^3 5*t3^4 6*t3^5; %t3�ٶ�
              0 0 2 0 0 0 0;                    %t1���ٶ�
              0 0 2 6*t3 12*t3^2 20*t3^3 30*t3^4];  %t3���ٶ�
MatrixGeiding=[p1;p2;p3;v1;v3;a1;a3];   %�߸���֪������ע�������зֱ����x��y��z��������
XishuA=MatrixCanshu\MatrixGeiding;%����ĵ�һ����x��ϵ�� �ڶ�����y��ϵ�� ��������z��ϵ��
t=CurrentTime;
PlanningX=XishuA(1,1)+XishuA(2,1)*t+XishuA(3,1)*t^2+XishuA(4,1)*t^3+XishuA(5,1)*t^4+XishuA(6,1)*t^5+XishuA(7,1)*t^6;
PlanningY=XishuA(1,2)+XishuA(2,2)*t+XishuA(3,2)*t^2+XishuA(4,2)*t^3+XishuA(5,2)*t^4+XishuA(6,2)*t^5+XishuA(7,2)*t^6;
PlanningZ=XishuA(1,3)+XishuA(2,3)*t+XishuA(3,3)*t^2+XishuA(4,3)*t^3+XishuA(5,3)*t^4+XishuA(6,3)*t^5+XishuA(7,3)*t^6;
PlanningPoint=[PlanningX PlanningY PlanningZ];  %���Ϊ1��3������
end







