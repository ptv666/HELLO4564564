%六次曲线的计算:摆动腿的起点、中间点、终点、持续时间、需要计算的时刻
function PlanningPoint=TrajectoryPlanning6(p1,p2,p3,t3,CurrentTime) %CurrentTime是在当前周期下运行的时间 t3是持续时间 六次样条曲线
%起点的位置p1(传入的)、速度v1=0、加速度a1=0
v1=[0,0,0]; a1=[0,0,0]; t1=0;   %起点的时间为0
%终点(落足点)的位置p3(传入的)、速度v3=0、加速度a3=0
v3=[0,0,0];a3=[0,0,0];
%中间点的时间为时间中点
t2=(t1+t3)/2; %对应时间为t2
%规划的轨迹的形式
%P=Ar0+Ar1*t+Ar2*t^2+Ar3*t^3+Ar4*t^4+Ar5*t^5+Ar6*t^6;需要有七个条件才能求解七个参数
MatrixCanshu=[1 0 0 0 0 0 0;                    %t1位置
              1 t2 t2^2 t2^3 t2^4 t2^5 t2^6;    %t2位置
              1 t3 t3^2 t3^3 t3^4 t3^5 t3^6;    %t3位置
              0 1 0 0 0 0 0;                    %t1速度
              0 1 2*t3 3*t3^2 4*t3^3 5*t3^4 6*t3^5; %t3速度
              0 0 2 0 0 0 0;                    %t1加速度
              0 0 2 6*t3 12*t3^2 20*t3^3 30*t3^4];  %t3加速度
MatrixGeiding=[p1;p2;p3;v1;v3;a1;a3];   %七个已知条件，注意有三列分别代表x、y、z三个方向
XishuA=MatrixCanshu\MatrixGeiding;%结果的第一列是x的系数 第二列是y的系数 第三列是z的系数
t=CurrentTime;
PlanningX=XishuA(1,1)+XishuA(2,1)*t+XishuA(3,1)*t^2+XishuA(4,1)*t^3+XishuA(5,1)*t^4+XishuA(6,1)*t^5+XishuA(7,1)*t^6;
PlanningY=XishuA(1,2)+XishuA(2,2)*t+XishuA(3,2)*t^2+XishuA(4,2)*t^3+XishuA(5,2)*t^4+XishuA(6,2)*t^5+XishuA(7,2)*t^6;
PlanningZ=XishuA(1,3)+XishuA(2,3)*t+XishuA(3,3)*t^2+XishuA(4,3)*t^3+XishuA(5,3)*t^4+XishuA(6,3)*t^5+XishuA(7,3)*t^6;
PlanningPoint=[PlanningX PlanningY PlanningZ];  %输出为1×3行向量
end







