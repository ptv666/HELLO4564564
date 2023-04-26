%五次曲线计算:传入起始位置、终止位置、总时间、需要计算位置的时刻
function PlanningPoint = TrajectoryPlanning5(p1,p3,t_cost,CurrentTime) %CurrentTime是在当前周期下运行的时间 t3是持续时间 五次样条曲线
v1=[0,0,0];a1=[0,0,0];  %起点的速度加速度全是零
%落足点相关参数
v3=[0,0,0]; a3=[0,0,0];
%少了一维，所以不用加入中间点信息
MatrixCanshu=[1 0 0 0 0 0 ;1 t_cost t_cost^2 t_cost^3 t_cost^4 t_cost^5;0 1 0 0 0 0 ;0 1 2*t_cost 3*t_cost^2 4*t_cost^3 5*t_cost^4;0 0 2 0 0 0 ;0 0 2 6*t_cost 12*t_cost^2 20*t_cost^3];
MatrixGeiding=[p1;p3;v1;v3;a1;a3];
XishuA=MatrixCanshu\MatrixGeiding;%结果的第一列是x的系数 第二列是y的系数 第三列是z的系数
t=CurrentTime;
PlanningX=XishuA(1,1)+XishuA(2,1).*t+XishuA(3,1).*t.^2+XishuA(4,1).*t.^3+XishuA(5,1).*t.^4+XishuA(6,1).*t.^5;
PlanningY=XishuA(1,2)+XishuA(2,2).*t+XishuA(3,2).*t.^2+XishuA(4,2).*t.^3+XishuA(5,2).*t.^4+XishuA(6,2).*t.^5;
PlanningZ=XishuA(1,3)+XishuA(2,3).*t+XishuA(3,3).*t.^2+XishuA(4,3).*t.^3+XishuA(5,3).*t.^4+XishuA(6,3).*t.^5;
PlanningPoint=[PlanningX PlanningY PlanningZ];
end