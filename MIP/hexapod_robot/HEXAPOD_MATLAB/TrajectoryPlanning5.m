%������߼���:������ʼλ�á���ֹλ�á���ʱ�䡢��Ҫ����λ�õ�ʱ��
function PlanningPoint = TrajectoryPlanning5(p1,p3,t_cost,CurrentTime) %CurrentTime���ڵ�ǰ���������е�ʱ�� t3�ǳ���ʱ�� �����������
v1=[0,0,0];a1=[0,0,0];  %�����ٶȼ��ٶ�ȫ����
%�������ز���
v3=[0,0,0]; a3=[0,0,0];
%����һά�����Բ��ü����м����Ϣ
MatrixCanshu=[1 0 0 0 0 0 ;1 t_cost t_cost^2 t_cost^3 t_cost^4 t_cost^5;0 1 0 0 0 0 ;0 1 2*t_cost 3*t_cost^2 4*t_cost^3 5*t_cost^4;0 0 2 0 0 0 ;0 0 2 6*t_cost 12*t_cost^2 20*t_cost^3];
MatrixGeiding=[p1;p3;v1;v3;a1;a3];
XishuA=MatrixCanshu\MatrixGeiding;%����ĵ�һ����x��ϵ�� �ڶ�����y��ϵ�� ��������z��ϵ��
t=CurrentTime;
PlanningX=XishuA(1,1)+XishuA(2,1).*t+XishuA(3,1).*t.^2+XishuA(4,1).*t.^3+XishuA(5,1).*t.^4+XishuA(6,1).*t.^5;
PlanningY=XishuA(1,2)+XishuA(2,2).*t+XishuA(3,2).*t.^2+XishuA(4,2).*t.^3+XishuA(5,2).*t.^4+XishuA(6,2).*t.^5;
PlanningZ=XishuA(1,3)+XishuA(2,3).*t+XishuA(3,3).*t.^2+XishuA(4,3).*t.^3+XishuA(5,3).*t.^4+XishuA(6,3).*t.^5;
PlanningPoint=[PlanningX PlanningY PlanningZ];
end