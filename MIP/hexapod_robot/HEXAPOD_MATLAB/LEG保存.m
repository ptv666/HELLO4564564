classdef LEG_SAVE
    %什么是臂端坐标系？-类成员属性中的-根关节坐标系
    %什么是FlagPlotSphereDeadPoint（标志是否画触觉反馈不行的落足点）？-构造函数中的-
    %T_W_FixLegP2中FixLegP2是什么坐标系？-是扇形坐标系吗
    properties 
        theta = []; %根关节相对base的默认角度，-180°—180°
        %三个关节角
        theta1 = [];
        theta2 = [];
        theta3 = [];
        T_B_0 = [];%机体到臂端（根关节）坐标系的齐次变换矩阵，所以0代表根关节；在构造函数中就设置了
        T_W_B = [];%机体坐标系到世界坐标系的齐次变换矩阵
        isInverseSolution = 0;%返回值为1，代表成功求解
        isSupport = 1;  %是否是支撑腿
        isDead=0;%是否是废腿  当状态为1时，表示由于没有落足点，不能用它作支撑腿 只能用它做摆动腿
        
        SwingLegStartPoint=[];  %该腿在摆动周期的初始点
        SwingLegEndPoint=[];    %该腿在摆动周期的末点
        SwingLegPlanningPoint=[]; %摆动腿下一个仿真帧的规划位置        
%         StrideHight=0.1; %摆动腿摆动的高度 与初末点连线垂直向上

        TrajectoryRecordH=[];%在摆动周期的点存储在这里边 这个暂时用不到
        LegNum = [];    %腿的编号，1-6
        StrideLength = 0; %该摆动腿的步长，初末点的欧氏距离
        UsablePoint=[];%可用的落足点集合 每一个规划运动的循环进程中更新一次
        
        Initp4BPoint=[];%初始状态下足端在机体坐标系下的坐标
        CountTouchTimes=0; %记录在一个摆动周期下触地次数（由于有触觉反射）
        SetReflectthreshold=70;%设置发生足端反射的阈值 在将来的程序中 不同的腿可能发生的阈值不同
        
        TPlanningTouchChangePoint=0;
        TPlanningFootMove=0;    %摆动腿运动时间
        CurrentTimeLeg=0;       %当前规划时间
        T_TimeFoot=0;
        StrideHight=0.1;%在腿摆动的过程中腿的迈动高度
        
        TouchTryTimes=3; %触地反射试的次数 如果试了这么多次还不行，isdead=1，这个值在demo中被换成2
        
        FootSizeD=0.08;%足的直径
        PhysicsInformation=[];%本条腿触到的物理信息
        
        TouchChangeFlag=1;%用于反射层的出错规划 标志位
        LegActionIng=0;%标志该腿是否在动 0为不动 1为动
        FlagPlotSphereDeadPoint=0;%标志是否画触觉反馈不行的落足点。
    end
    
    properties (Dependent)  %自动更新
        T_W_1 = []; %根关节相对world的T
        T_W_2 = []; %二连杆1关节相对world的T
        T_W_3 = []; %二连杆2关节相对world的其T
        T_W_4 = []; %足端相对于world的T
        p1_B = [];  %根关节相对于base的位置
        p2_B = [];  %二连杆1关节相对base的位置
        p3_B = [];  %二连杆2关节相对base的位置
        p4_B = [];  %足端相对于base的位置
        p1_W = [];  %相对于world的位置
        p2_W = [];
        p3_W = [];
        p4_W = [];	%足端相对世界的位置
        T_W_FixLegP2 = [];% 固定2号坐标系位置到世界坐标系的旋转矩阵，是扇形坐标系吗？
        T_B_FixLegP2 = [];% 固定2号坐标系位置到机器人机体坐标系的旋转矩阵
        SectorPlotData = [];    %世界坐标系下扇形的点集合
    end
    

    
    
    methods %构造函数，设置腿的编号、三个关节角初始值、根关节相对机体的T，机体相对于world的T
        function output = LEG(leg_Num)
            output.LegNum = leg_Num;    %腿的编号
            output.theta = mod((60*leg_Num + 60),360);  %根关节相对机体的默认角度
            if(output.theta>180)
                output.theta = output.theta-360;
            elseif(output.theta<-180)
                output.theta = output.theta+360;
            end
            output.theta = output.theta/180*pi;
            output.T_B_0 = matrixBody_Leg(output.theta);    %根关节相对于机体的T
            %默认位置下的关节角
            output.theta1 = 0;
            output.theta2 = 0;
            output.theta3 = -pi/2;
            output.T_W_B = eye(4);  %机体相对world的T，默认时两个坐标系是重合的
            
            output.FlagPlotSphereDeadPoint=0;%标志是否画触觉反馈不行的落足点。
        end
    end
    
    methods
        function obj = setTheta(obj , theta_in_1,theta_in_2,theta_in_3) %设置关节角
            obj.theta1 = theta_in_1;
            obj.theta2 = theta_in_2;
            obj.theta3 = theta_in_3;
        end 
        
        function obj = OnrLegInverseKinematicsWoldAxis(obj,p4_W_input)  %求解逆运动学，输入world坐标系下的足端位置（列向量）
            obj.isInverseSolution = 0;
            vector_p4_B_temp = obj.T_W_B\[p4_W_input;1];    %obj.T_W_B逆 * [p4_W_input;1]；空间点在base坐标系下的描述
            vector_p4_O_temp = obj.T_B_0\vector_p4_B_temp;  %空间点在根关节坐标系下的描述
            
            tmp_theta1 = atan2(vector_p4_O_temp(2),vector_p4_O_temp(1));    %求解根关节的转角
            
            %二连杆坐标系只有xz方向是自由的
            N = vector_p4_O_temp(3);    %在二连杆坐标系下的z坐标
            %二连杆坐标系下的x坐标
            if(vector_p4_O_temp(2) == 0)
                M = vector_p4_O_temp(1) - 0.18; %当根关节处于原位时
            else
                M = vector_p4_O_temp(2)/sin(tmp_theta1) - 0.18; %当 根关节没有处于原位时
            end
            if(sqrt(M^2 + N^2) > 0.5+0.5)   %足端位置超出了二连杆的工作空间范围
                disp('OUT OF RANGE1');
                return;
            end

            temp_acos =  acos( ( 0.5^2 + M^2 + N^2 - 0.5^2 ) / ( 2 * 0.5 * sqrt(M^2 + N^2) ) );
            if(abs(temp_acos) < 0.000001 ) 
                temp_acos = 0;  %防止acos出现0.00000001+ 0.0000000000000i的虚数
            end
            tmp_theta2 = atan2(N,M) + temp_acos;%有双解
            tmp_theta3 = atan2( N - 0.5*sin(tmp_theta2) , M - 0.5*cos(tmp_theta2)) - tmp_theta2;
            
            if (isAngleOutOfRange(tmp_theta1,tmp_theta2,tmp_theta3)  == 1)
                
                obj.theta1 = tmp_theta1;
                obj.theta2 = tmp_theta2;
                obj.theta3 = tmp_theta3;
                obj.isInverseSolution = 1; %求解成功
                return;
            end
            disp('OUT OF RANGE2');
        end
        
        function obj=leg_move(obj,x,y,z) %腿移动到世界坐标系下的x y z 位置
            p1(1,1) = x; 
            p1(2,1) = y; 
            p1(3,1) = z; 
            obj = obj.OnrLegInverseKinematicsWoldAxis(p1);
        end
        
        function obj=LegSwingActionTrajectory(obj,t3,time,StrideHight) %该腿按照六次样条曲线生成的轨迹移动，t3为摆动持续时间，time为当前时刻，六次曲线中间的的高度
            %定义六次曲线的三个点
            p1=obj.SwingLegStartPoint';
            p2=obj.SwingLegStartPoint'+(obj.SwingLegEndPoint'-obj.SwingLegStartPoint')/2+[0 0 StrideHight];%这里假设运行轨迹是垂直于地面的
            p3=obj.SwingLegEndPoint';
            obj.SwingLegPlanningPoint=TrajectoryPlanning6(p1,p2,p3,t3,time);    %求解time时刻下六次曲线的位置
            obj.SwingLegPlanningPoint=obj.SwingLegPlanningPoint';   %行向量转换为列向量
            %腿移动到time时刻在六次曲线的位置
            temp=obj.leg_move(obj.SwingLegPlanningPoint(1),obj.SwingLegPlanningPoint(2),obj.SwingLegPlanningPoint(3));
            obj=temp;
        end
        
        %该函数没有用到
        function obj=NewLegSwingActionTrajectory(obj,StartPoint,MidPoint,EndPoint,t3,time) %提供一模板 六次样条曲线 t3为摆动持续时间，time为当前时刻
            p1=StartPoint;
            p2=MidPoint;
            p3=EndPoint;
            obj.SwingLegPlanningPoint=TrajectoryPlanning6(p1,p2,p3,t3,time);
            obj.SwingLegPlanningPoint=obj.SwingLegPlanningPoint';
            TrajectoryRecord=obj.SwingLegPlanningPoint; %
            obj.TrajectoryRecordH=[obj.TrajectoryRecordH TrajectoryRecord];
            if (obj.TrajectoryRecordH(3,size(obj.TrajectoryRecordH,2))<-0.5)
            end
            temp=obj.leg_move(obj.SwingLegPlanningPoint(1),obj.SwingLegPlanningPoint(2),obj.SwingLegPlanningPoint(3));
            obj=temp;
        end

        function T_W_1=get.T_W_1(obj)%根关节相对于world的T矩阵
            T_0_1 = matrixT(obj.theta1,0,0,0);  %二连杆的1号关节相对于根关节的T
            T_W_1 = obj.T_W_B*obj.T_B_0*T_0_1;  %二连杆的1号关节相对于世界坐标系的T
        end
        function T_W_2=get.T_W_2(obj)%二连杆1关节相对于world的T矩阵
            T_1_2 = matrixT(obj.theta2,pi/2,0.18,0);
            T_W_2 = obj.T_W_1*T_1_2;
        end
        function T_W_3=get.T_W_3(obj)%二连杆2关节相对于world的T矩阵
            T_2_3 = matrixT(obj.theta3,0,0.5,0);
            T_W_3 = obj.T_W_2*T_2_3;
        end
        function T_W_4=get.T_W_4(obj)%足端相对于world的T矩阵
            T_3_4 = matrixT(0,0,0.5,0);
            T_W_4 = obj.T_W_3*T_3_4;
        end
        
        function p1_B=get.p1_B(obj)%根关节相对于base的位置列向量
            p1_B = obj.T_W_B\obj.T_W_1;
            p1_B = p1_B(1:3,4);
        end
        function p2_B=get.p2_B(obj)%二连杆1关节相对于base的位置列向量
            p2_B = obj.T_W_B\obj.T_W_2;
            p2_B = p2_B(1:3,4);
        end
        function p3_B=get.p3_B(obj) %二连杆2关节相对于base的位置列向量
            p3_B = obj.T_W_B\obj.T_W_3;
            p3_B = p3_B(1:3,4);
        end
        function p4_B=get.p4_B(obj)%足端相对于base的位置列向量
            p4_B = obj.T_W_B\obj.T_W_4;
            p4_B = p4_B(1:3,4);
        end
        
        function p1_W=get.p1_W(obj)%根关节相对于world的位置列向量
            p1_W = obj.T_W_1(1:3,4);
        end
        function p2_W=get.p2_W(obj)%二连杆1关节相对于world的位置列向量
            p2_W = obj.T_W_2(1:3,4);
        end
        function p3_W=get.p3_W(obj)%二连杆2关节相对于world的位置列向量
            p3_W = obj.T_W_3(1:3,4);
        end
        function p4_W=get.p4_W(obj)%足端相对于world的位置列向量
            p4_W = obj.T_W_4(1:3,4);
        end
        
        
        function T_W_FixLegP2 = get.T_W_FixLegP2(obj)%扇形坐标系相对于world的坐标系的T矩阵
            T_W_FixLegP2 = obj.T_W_B*obj.T_B_0 * matrixT(0,pi/2,0.18,0)*[0,1,0,0;0,0,1,0;1,0,0,0;0,0,0,1];
        end
        function T_B_FixLegP2 = get.T_B_FixLegP2(obj)%扇形坐标系相对于base的坐标系的T矩阵
            T_B_FixLegP2 = obj.T_B_0 * matrixT(0,pi/2,0.18,0)*[0,1,0,0;0,0,1,0;1,0,0,0;0,0,0,1];
        end
        
        function SectorPlotData = get.SectorPlotData(obj)%根据足端z，获得扇形点集
            % 计算足端点在固定2号坐标系下的位置，计算落足点在扇形坐标系下的位置
            p4_FixLegP2 = (obj.T_W_FixLegP2)\[obj.p4_W;1]; % 相当于inv(obj.T_W_FixLegP2)*obj.p4_W;
            
            tmp = p4_FixLegP2(3)+0.5;
            R = 0.8858*tmp^3 + -0.7787*tmp^2 + 0.5656*tmp + 0.8608; %扇形半径
            
            
            aplha= pi/4:pi/40:3*pi/4;   %扇形张角
            x= R*cos(aplha);            %扇形上的点x坐标
            y= R*sin(aplha);            %扇形上的点y坐标
            
            %在扇形坐标系下，圆心就是坐标原点
            x = [0,x,0];    
            y = [0,y,0];
            z = ones(1,size(x,2)) * p4_FixLegP2(3);

            %将固定2号坐标系下的坐标（扇形）转换到世界坐标系下，方便显示
            SectorPlotData =  obj.T_W_FixLegP2 * [[x;y;z];ones(1,size(x,2))];

        end
        
        %根据机器人移动后的位置，判定落足点是否满足关节角约束；Fake代表还不是这个状态
        function CheckFlag = CheckWorldPointInLegSpaceFakeT_W_B(obj,p4_W_input,T_W_BFake) %检查世界坐标系下的坐标是否在某一腿的某一特定的TWB的工作空间内 在为1 不在为0 而下一个函数是当前状态下真实的 输入参数包括当前状态和虚拟状态
            CheckFlag=1;
            obj.isInverseSolution = 0;
            vector_p4_B_temp = T_W_BFake\[p4_W_input;1]; %obj.T_W_B逆 * [p4_W_input;1]
            vector_p4_O_temp = obj.T_B_0\vector_p4_B_temp;  %获得世界坐标系下的落足点在根关节坐标系下的表达
            
            tmp_theta1 = atan2(vector_p4_O_temp(2),vector_p4_O_temp(1));
            N = vector_p4_O_temp(3);
            if(vector_p4_O_temp(2) == 0)
                M = vector_p4_O_temp(1) - 0.18;
            else
                M = vector_p4_O_temp(2)/sin(tmp_theta1) - 0.18;
            end
            if(sqrt(M^2 + N^2) > 0.5+0.5)
%                 disp('OUT OF RANGE');
                CheckFlag=0;
                return;
            end
            temp_acos =  acos( ( 0.5^2 + M^2 + N^2 - 0.5^2 ) / ( 2 * 0.5 * sqrt(M^2 + N^2) ) );
            if(abs(temp_acos) < 0.000001 ) 
                temp_acos = 0;  %防止acos出现0.00000001+ 0.0000000000000i的虚数
            end
            tmp_theta2 = atan2(N,M) + temp_acos;%有双解
            tmp_theta3 = atan2( N - 0.5*sin(tmp_theta2) , M - 0.5*cos(tmp_theta2)) - tmp_theta2;
            
            if (isAngleOutOfRange(tmp_theta1,tmp_theta2,tmp_theta3)  == 1)
                return;
            end
            
            CheckFlag=0;
        end
        %根据机器人移动后的位置，判定落足点是否在扇形区域内；Fake代表还不是这个状态
        function CheckFlag=CheckWorldPointInShanxinFakeT_W_B(obj,p4_W_input,FakeT_W_B)
            %             Coordinate = obj.T_W_FixLegP2 \ [x;y;z;1]; %将世界坐标系下的坐标转化到固定2号坐标系下
            x1=p4_W_input(1);
            y1=p4_W_input(2);
            z1=p4_W_input(3);
            tmp = z1+0.5;
            R = 0.8858*tmp^3 + -0.7787*tmp^2 + 0.5656*tmp + 0.8608;
            aplha= pi/4:pi/40:3*pi/4;
            x= R*cos(aplha);
            y= R*sin(aplha);
            T_W_FixLegP2Fake=FakeT_W_B*obj.T_B_0 * matrixT(0,pi/2,0.18,0)*[0,1,0,0;0,0,1,0;1,0,0,0;0,0,0,1];
            
            x = [0,x,0];
            y = [0,y,0];
            z = ones(1,size(x,2)) * z1;
            CurrentSectorPlotData= T_W_FixLegP2Fake * [[x;y;z];ones(1,size(x,2))];
            
            theta1tmp=T_W_FixLegP2Fake\(CurrentSectorPlotData(1:4,size(CurrentSectorPlotData,2)-1)-CurrentSectorPlotData(1:4,1));
            theta11=atan2(theta1tmp(2),theta1tmp(1));
            theta2tmp=T_W_FixLegP2Fake\(CurrentSectorPlotData(1:4,2)-CurrentSectorPlotData(1:4,1));
            theta22=atan2(theta2tmp(2),theta2tmp(1));
            
            thetatmp=T_W_FixLegP2Fake\[([x1;y1;z1]-CurrentSectorPlotData(1:3,1));0];
            theta00=atan2(thetatmp(2),thetatmp(1));
            range=0;
            if (theta00<theta11-range&&theta00>theta22+range)||(theta00>theta11+range&&theta00<theta22-range)
                if norm(thetatmp(1:2,:))<R
                    CheckFlag=1;
                else
                    CheckFlag=0;
                end
            else
                CheckFlag=0;
            end
        end
        %根据机器人当前的位置，判定落足点是否在关节角范围内
        function CheckFlag = CheckWorldPointInLegSpace(obj,p4_W_input) %检查世界坐标系下的坐标是否在某一腿的工作空间内 在为1 不在为0
            CheckFlag=1;
            obj.isInverseSolution = 0;
            vector_p4_B_temp = obj.T_W_B\[p4_W_input;1]; %obj.T_W_B逆 * [p4_W_input;1]
            vector_p4_O_temp = obj.T_B_0\vector_p4_B_temp;
            
            tmp_theta1 = atan2(vector_p4_O_temp(2),vector_p4_O_temp(1));
            N = vector_p4_O_temp(3);
            if(vector_p4_O_temp(2) == 0)
                M = vector_p4_O_temp(1) - 0.18;
            else
                M = vector_p4_O_temp(2)/sin(tmp_theta1) - 0.18;
            end
            if(sqrt(M^2 + N^2) > 0.5+0.5)
%                 disp('OUT OF RANGE');
                CheckFlag=0;
                return;
            end
            temp_acos =  acos( ( 0.5^2 + M^2 + N^2 - 0.5^2 ) / ( 2 * 0.5 * sqrt(M^2 + N^2) ) );
            if(abs(temp_acos) < 0.000001 ) 
                temp_acos = 0;  %防止acos出现0.00000001+ 0.0000000000000i的虚数
            end
            tmp_theta2 = atan2(N,M) + temp_acos;%有双解
            tmp_theta3 = atan2( N - 0.5*sin(tmp_theta2) , M - 0.5*cos(tmp_theta2)) - tmp_theta2;
            
            if (isAngleOutOfRange(tmp_theta1,tmp_theta2,tmp_theta3)  == 1)       
                return;
            end
            CheckFlag=0;
        end
        
        %在当前状态下的可落足点集合中选择最好的落足点，也就是能沿着速度方向前进最大距离的落足点
        function BestFootPoint=GetBestFootPointFromUseablePoint(obj,VelocityVector)
            if isempty(obj.UsablePoint)==0
                VelocityVector=VelocityVector/norm(VelocityVector);
                PerVector=obj.UsablePoint(1,:)-obj.SwingLegStartPoint';
                DanweiPerVector=PerVector/norm(PerVector);

                BestFootPoint=obj.UsablePoint(1,:);

                Dot=dot(DanweiPerVector,VelocityVector);
                LengthMax=Dot*norm(PerVector);
                for i=1:size(obj.UsablePoint,1)
                    PerVector=obj.UsablePoint(i,:)-obj.SwingLegStartPoint';
                    DanweiPerVector=PerVector/norm(PerVector);
                    if dot(DanweiPerVector,VelocityVector)*norm(PerVector)>LengthMax
                        LengthMax=dot(PerVector,VelocityVector)*norm(PerVector);
                        BestFootPoint=obj.UsablePoint(i,:);
                    end
                end
                BestFootPoint=BestFootPoint';
%                 obj.isDead=0;
%                 T_W_43(BestFootPoint(1),BestFootPoint(2),BestFootPoint(3),'k*');
            else
                aa=12;
%                 BestFootPoint=obj.p4_W; %没有可落足点腿就不动
                BestFootPoint=[];
%                 obj.isDead=1;%没有可落足点腿就随着机体动，认为腿费了。
%                 disp('有一个腿没有可落足点');
%                 error('没有可落足点');
            end
        end
        
        % n 为在机体坐标系下的运动矢量,获得单腿扇形的运动裕度
        function [lineData,margin] = getKinematicsMargin(obj,n)
            n=-n/norm(n);
            % 计算足端点在固定2号坐标系下的位置
            p4_FixLegP2 = (obj.T_B_FixLegP2)\[obj.p4_B;1]; % 相当于inv(obj.T_W_FixLegP2)*obj.p4_W;
            
            tmp = p4_FixLegP2(3)+0.5;
            R = 0.8858*tmp^3 + -0.7787*tmp^2 + 0.5656*tmp + 0.8608;
            
            %计算固定坐标系下的前进向量  n为机体坐标系下的前进矢量
            tmp = [n';0] + [obj.p4_B;1];
            %计算固定2号坐标系下的前进矢量
            tmp = (obj.T_B_FixLegP2)\tmp; % 相当于inv(obj.T_W_FixLegP2)*obj.p4_W;
            nn = [tmp(1)-p4_FixLegP2(1),tmp(2)-p4_FixLegP2(2)];
%             theta_tmp = atan2(,);
            n = nn/norm(nn);
            %计算与扇形的交点
            resultP_FixLegP2 =  GetIntersectionSector(R,p4_FixLegP2(1),p4_FixLegP2(2),n);
            %计算运动学稳定裕度
            margin = sqrt( (p4_FixLegP2(1)-resultP_FixLegP2(1))^2 + (p4_FixLegP2(2)-resultP_FixLegP2(2))^2 );
%             margin=0.5*margin;
            %计算交点在世界坐标系下的位置
            resultP_W = obj.T_W_FixLegP2* [resultP_FixLegP2' ;p4_FixLegP2(3);1];
            %添加到一个变量输出，便于后期可视化
            lineData = [obj.p4_W,resultP_W(1:3,1)];
            margin=norm([lineData(1,1),lineData(2,1)]-[lineData(1,2),lineData(2,2)]); %WZK
            margin=0.8*margin; %再留余量WZK 如果不乘系数进一步缩小运动空间的话会出错。
            if obj.LegNum == 1
                hold on;
            end
        end
        
        %摆动腿规划,获取落足点；如果是容错退那么就在默认位置上抬起0.2m
        function [obj,point]=SwingLegActionPlanning(obj,startPoint,MoveDirection,TransitMatrix) 
            obj.SwingLegStartPoint=startPoint;
            if obj.isDead==0
               point=obj.GetBestFootPointFromUseablePoint(MoveDirection);
               if norm(point-startPoint)<0.0001
                   obj.StrideHight=0;
               end
            else
               obj.StrideHight=0;
               T_W_BFakeleg1=obj.T_W_B * TransitMatrix;
               point=T_W_BFakeleg1*[obj.Initp4BPoint;1];
               point=point(1:3,:)+[0,0,0.2]';
            end
            obj.SwingLegEndPoint=point; %确定该腿放的末端位置，这个point应该是由地面的落足点决定的 接口
        end
        
        function PhysicsInformation=GetPhysicsInformationFromTouchTerrainMap(obj,TerrainMapConsiderFootMatIncludingTouch,leg_p4W) %得到leg_p4W点对应的物理信息。
%             PhysicsInformation=-1;
            for i=1:size(TerrainMapConsiderFootMatIncludingTouch,1)
                if norm(TerrainMapConsiderFootMatIncludingTouch(i,1:3)-leg_p4W)<0.001
                    PhysicsInformation=TerrainMapConsiderFootMatIncludingTouch(i,4);
                    PhysicsInformation=[PhysicsInformation;i];
                    return;
                end
            end
            PhysicsInformation(1,1)=-10000;
            PhysicsInformation(2,1)=-10000;
            disp('没有得到相应的物理信息 可能存在的原因是阈值设置不正确或者腿未触地或者足不正确的放在落脚点上或者被删除了');
        end
        
        function obj=ReflectLegChangeFootPoint(obj,StartPoint) %发生反射的时候需要找到反射的落脚点
            if (size(obj.UsablePoint,1)==0)
                disp('没有可反射的其他落足点 归到初始位置');
%                 point=obj.T_W_B*[obj.Initp4BPoint;1];
%                 point=point(1:3,:)+[0,0,0.2]';
%                 obj.SwingLegEndPoint=point;
                obj.isDead=1; %由于没有落足点当做废腿处理 不能用它作支撑腿 只能用它做摆动腿
                return;
            end
            StartPoint=StartPoint';
            obj.SwingLegStartPoint=StartPoint;
            obj.SwingLegEndPoint=obj.UsablePoint(1,:);
           
            Norm=10000;
            for i=1:size(obj.UsablePoint,1)
%                 PhysicsInformation=obj.GetPhysicsInformationFromTouchTerrainMap(TerrainMapConsiderFootMatIncludingTouch,obj.UsablePoint(i,:));
                if norm(obj.UsablePoint(i,:)-StartPoint)<Norm && norm(obj.UsablePoint(i,:)-StartPoint)>0.001 %不等于0防止选到落足点本身
                    Norm=norm(obj.UsablePoint(i,:)-StartPoint);
                    obj.SwingLegEndPoint=obj.UsablePoint(i,:);
%                     if PhysicsInformation(1)>SetReflectthreshold%大于这个阈值才认为可落足
%                         obj.SwingLegEndPoint=obj.UsablePoint(i,:);
%                     else %否则通过触觉判断该地不可落足，删除该片区域
%                         plot3(TerrainMapConsiderFootMatIncludingTouch(PhysicsInformation(2),1),TerrainMapConsiderFootMatIncludingTouch(PhysicsInformation(2),2),TerrainMapConsiderFootMatIncludingTouch(PhysicsInformation(2),3),'k*','LineWidth',4);
%                         TerrainMapConsiderFootMatIncludingTouch(PhysicsInformation(2),:)=[];
%                     end
%                 else
%                     if norm(obj.UsablePoint(i,:)-StartPoint)<0.001 %进来这里表明该点走不成，所以需要删掉
%                         plot3(TerrainMapConsiderFootMatIncludingTouch(PhysicsInformation(2),1),TerrainMapConsiderFootMatIncludingTouch(PhysicsInformation(2),2),TerrainMapConsiderFootMatIncludingTouch(PhysicsInformation(2),3),'k*','LineWidth',4);
%                         TerrainMapConsiderFootMatIncludingTouch(PhysicsInformation(2),:)=[];
%                     end
                end
            end
%             TerrainMapConsiderFootMatIncludingTouch1=TerrainMapConsiderFootMatIncludingTouch;
            obj.SwingLegStartPoint=obj.SwingLegStartPoint';
            obj.SwingLegEndPoint=obj.SwingLegEndPoint';
            %             ||norm(obj.SwingLegEndPoint-obj.UsablePoint(1,:)')<0.001
            
        end 
        
        function obj=DeletePointFromUsablePoint(obj,point) %将一点从可选点集合中删除 这是因为触觉判断不可落足
%             num=size(obj.UsablePoint,1);
            output=obj.UsablePoint;
            for i=1:size(obj.UsablePoint,1)
                if norm(point'-obj.UsablePoint(i,1:3))<0.001
                    output(i,:)=[];
                    break;
                end
            end
            if size(output,1)==size(obj.UsablePoint,1)
                disp('没有从可选点集合中找到该点');
            end
            obj.UsablePoint=output;
        end
        
        %一条腿触地后的一系列规划操作
        function [obj,TerrainMapConsiderFootMatIncludingTouch1,PlotFlag]=ReflectLegActionPlanning(obj,MinusSupportState,TerrainMapConsiderFootMatIncludingTouch,time,TimeInterval)
            PlotFlag = 0;
            if MinusSupportState==1%表示当前落地，上一时刻摆动
                obj.PhysicsInformation=obj.GetPhysicsInformationFromTouchTerrainMap(TerrainMapConsiderFootMatIncludingTouch,obj.p4_W');
                if obj.PhysicsInformation(1)<obj.SetReflectthreshold
                    obj.CountTouchTimes=obj.CountTouchTimes+1;
%                     ReflectLegList=[ReflectLegList,1];
%                     plot3(TerrainMapConsiderFootMatIncludingTouch(PhysicsInformation(2),1),TerrainMapConsiderFootMatIncludingTouch(PhysicsInformation(2),2),TerrainMapConsiderFootMatIncludingTouch(PhysicsInformation(2),3),'k*','LineWidth',4);
                    obj.FlagPlotSphereDeadPoint=1;
                    TerrainMapConsiderFootMatIncludingTouch(obj.PhysicsInformation(2),:)=[];
                    TerrainMapConsiderFootMatIncludingTouch1=TerrainMapConsiderFootMatIncludingTouch;
                    obj.TPlanningFootMove=obj.TPlanningTouchChangePoint+1*TimeInterval;%按照触觉规划的来迈腿
                    obj.CurrentTimeLeg = time+TimeInterval;
                    temp=obj.DeletePointFromUsablePoint(obj.p4_W);
                    obj=temp;
                    temp=obj.ReflectLegChangeFootPoint(obj.p4_W);
                    obj=temp;
                    obj.TouchChangeFlag=0; %防止再次进入再规划
                    if obj.CountTouchTimes>obj.TouchTryTimes
                        obj.isDead=1;
                    end
                    
                else
%                     PhysicsInformation=obj.GetPhysicsInformationFromTouchTerrainMap(TerrainMapConsiderFootMatIncludingTouch,leg_p4W); %得到leg_p4W点对应的物理信息。
%                     if obj.LegNum==6
%                         ss=1;
%                     end
                    obj.isDead=0;
                    obj.SwingLegStartPoint=obj.p4_W;
                    obj.SwingLegEndPoint=obj.p4_W;
                    obj.isSupport=1; 
                    obj.LegActionIng=0; %该腿动完了
                    
                    PlotFlag = 1;
                end
            end
            TerrainMapConsiderFootMatIncludingTouch1=TerrainMapConsiderFootMatIncludingTouch;
        end
        %当isdead=1的时候，执行的反射层规划操作
        function obj=ReflectDEADLegActionPlanning(obj,time,TimeInterval) 
            if obj.isDead==1 && obj.TouchChangeFlag==0
                obj.TouchChangeFlag=1; %规划层必须保证只执行一次，否则执行层没法动。
                obj.SwingLegStartPoint=obj.p4_W;
                point=obj.T_W_B*[obj.Initp4BPoint;1];
                point=point(1:3,:)+[0,0,0.2]';
                obj.SwingLegEndPoint=point;
                obj.TPlanningFootMove=obj.TPlanningTouchChangePoint+1*TimeInterval;%按照触觉规划的来迈腿 7*interval
                obj.CurrentTimeLeg = time+TimeInterval;
            end
        end
        %腿部运动 相当于驱动器 执行器
        function obj=LegAction(obj,time,TimeInterval)
            obj.T_TimeFoot=time-obj.CurrentTimeLeg;%当前周期足端的时间戳 六次规划曲线有可能不能求解出结果，需要人为移动
            if obj.T_TimeFoot<obj.TPlanningFootMove
                if obj.isSupport==0
            %         if (time-CurrentTime<=(1-Beita)*Tgait)
                    temp=obj.LegSwingActionTrajectory(obj.TPlanningFootMove,obj.T_TimeFoot,obj.StrideHight); %按照这样的轨迹迈步，可以改动 A* rrt之类的，需要改变内部的函数内容
                    obj=temp;
                    obj.LegActionIng=1; %该腿正在动
                    if obj.LegNum==6 %画6号腿的轨迹
                        plot3(obj.p4_W(1),obj.p4_W(2),obj.p4_W(3),'c.');
                    end
            %         end
                end
            else
                if obj.T_TimeFoot>=obj.TPlanningFootMove && obj.T_TimeFoot<=obj.TPlanningFootMove+3*TimeInterval
%                 HEXAPOD1.leg1.p4_W(3)+0.5>yuzhiz||HEXAPOD1.leg2.p4_W(3)+0.5>yuzhiz||HEXAPOD1.leg3.p4_W(3)+0.5>yuzhiz||HEXAPOD1.leg4.p4_W(3)+0.5>yuzhiz||HEXAPOD1.leg5.p4_W(3)+0.5>yuzhiz||HEXAPOD1.leg6.p4_W(3)+0.5>yuzhiz
                    if obj.isSupport==0
                        temp=obj.leg_move(obj.SwingLegEndPoint(1),obj.SwingLegEndPoint(2),obj.SwingLegEndPoint(3));
                        obj=temp;
                        obj.LegActionIng=0; %该腿动完了
                    end
                end
            end
        end
        
    end
    
    
    
end


%返回值为1，则满足要求。
%检查关节角度是否超出限制
function flag = isAngleOutOfRange(theta1, theta2, theta3)
        flag = -2;
        %根关节：-40°至40°
        if (theta1 >= -40/180*pi) && (theta1 <= 40/180*pi)   %检查赋值要求
            flag = flag + 1;
        end
        %二连杆关节1：-90°至90°
        if (theta2 >= -90/180*pi) && (theta2 <= 50/180*pi)   %检查赋值要求
            flag = flag + 1;
        end
        %二连杆关节2：-150°至0°
        if (theta3 >= -150/180*pi) && (theta3 <= 0/180*pi)   %检查赋值要求
            flag = flag + 1;
        end
end



