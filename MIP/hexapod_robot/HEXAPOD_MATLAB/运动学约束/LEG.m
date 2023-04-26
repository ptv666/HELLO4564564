classdef LEG
    %什么是臂端坐标系？-类成员属性中的-根关节坐标系
    %什么是FlagPlotSphereDeadPoint（标志是否画触觉反馈不行的落足点）？-构造函数中的-
    %T_W_FixLegP2中FixLegP2是什么坐标系？-是扇形坐标系吗
    properties 
        LegNum = [];    %腿的编号，1-6,构造函数中赋值
        
        %只能由外部HEXAPOD对象修改，而不能自己修改
        isSupport = 1;  %是否是支撑腿
        isDead = 0;     %是否是废腿  当状态为1时，表示由于没有落足点，不能用它作支撑腿 只能用它做摆动腿
        
        isInverseSolution = 0;  %返回值为1，代表成功求解
        LegActionIng = 0;       %标志该腿是否在动 0为不动 1为动，用于在规划和动画的显示
        
        Initp4BPoint=[];%初始状态下足端在机体坐标系下的坐标
        theta = [];     %单腿安装在机体的哪个位置上0°,60°,120°,180°,-120°,-60°
        theta1 = [];
        theta2 = [];
        theta3 = [];
        FootSizeD=0.08; %足的直径
        
        T_B_0 = []; %base上单腿如何安装的齐次变换矩阵
        T_W_B = []; %机体坐标系到世界坐标系的齐次变换矩阵
        
        %如何判断是否是容错腿
        UsablePoint = [];   %可用的落足点集合 每一个规划运动的循环进程中更新一次
        
        %有关摆动腿的轨迹规划的落足点
        SwingLegStartPoint = [];    %该腿在摆动周期的初始点
        SwingLegEndPoint = [];      %该腿在摆动周期的末点
        SwingLegPlanningPoint = []; %摆动腿下一个仿真帧的规划位置
        
        %摆动腿曲线插值属性
%         StrideLength = 0;   %该摆动腿的步长，初末点的欧氏距离，需要计算得到的
        StrideHight = -100;    %在腿摆动的过程中腿的迈动高度，需要由外部传入
        
        %有关时间的属性
        Time_Move_Total_cost=0;    %摆动腿总共运动时间1.5s
        Time_Leg_Statr_Act=0;       %摆动腿开始运动的时刻是多少0-80s
        T_TimeFoot=0;           %
        
    end
    
    properties (Dependent)  %自动更新
        T_W_1;  %根关节相对world的T
        T_W_2;  %二连杆1关节相对world的T
        T_W_3;  %二连杆2关节相对world的其T
        T_W_4;  %足端相对于world的T
        
        %三个关节和足端相对于base的位置
        p1_B;   %根关节相对于base的位置
        p2_B;   %二连杆1关节相对base的位置
        p3_B;   %二连杆2关节相对base的位置
        p4_B;   %足端相对于base的位置
        
        %三个关节与足端相对于世界的位置
        p1_W;   
        p2_W;   
        p3_W;   
        p4_W;   %足端相对世界的位置
        
        T_W_FixLegP2;   % 固定2号坐标系位置到世界坐标系的旋转矩阵，是扇形坐标系吗？
        T_B_FixLegP2;   % 固定2号坐标系位置到机器人机体坐标系的旋转矩阵
        SectorPlotData; %世界坐标系下扇形的点集合
    end
    

    
    %构造函数，设置腿的编号、三个关节角初始值、根关节相对机体的T
    %%%%%%%%%机体相对于world的T应该有HEXPOAD传入，在HEXAPOD的构造函数中，调用绘图函数，绘图函数会吧T_W_B传给Leg
    methods
        function output = LEG(leg_Num)
            output.LegNum = leg_Num;                    %腿的编号
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
            
            output.T_W_B = eye(4);  %base坐标系就是世界坐标系
            %%%%%%%%%%%机体相对world的T，默认时两个坐标系是重合的,由于在HEXAPOD中也传参了T_W_B，所以这里稍显冗余，而且确实需要由外部传入才合理
            
        end
    end
    
    methods
        %设置关节角
        function obj = setTheta(obj , theta_in_1,theta_in_2,theta_in_3) 
            obj.theta1 = theta_in_1;
            obj.theta2 = theta_in_2;
            obj.theta3 = theta_in_3;
        end
        
        %腿移动到世界坐标系下的x y z 位置
        function obj=leg_move(obj,x,y,z)
            p1(1,1) = x; 
            p1(2,1) = y; 
            p1(3,1) = z; 
            obj = obj.OnrLegInverseKinematicsWoldAxis(p1);
        end
        
        %求解逆运动学，输入world坐标系下的足端位置（列向量）
        function obj = OnrLegInverseKinematicsWoldAxis(obj,p4_W_input)  
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
                sprintf('LEG%d 的逆运动学OUT OF RANGE1',leg_Num);
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
            else
                sprintf('LEG%d 的逆运动学OUT OF RANGE2',leg_Num);
            end
        end
        
        
         % n为在机体坐标系下的运动矢量,通过计算与扇形的交点，获得单腿扇形的运动裕度
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
            n = nn/norm(nn);
            %计算与扇形的交点
            resultP_FixLegP2 =  GetIntersectionSector(R,p4_FixLegP2(1),p4_FixLegP2(2),n);
            %计算运动学稳定裕度
            margin = sqrt( (p4_FixLegP2(1)-resultP_FixLegP2(1))^2 + (p4_FixLegP2(2)-resultP_FixLegP2(2))^2 );
            
            %计算交点在世界坐标系下的位置
            resultP_W = obj.T_W_FixLegP2* [resultP_FixLegP2' ;p4_FixLegP2(3);1];
            %添加到一个变量输出，便于后期可视化
            lineData = [obj.p4_W,resultP_W(1:3,1)];
            margin=norm([lineData(1,1),lineData(2,1)]-[lineData(1,2),lineData(2,2)]); %WZK
            margin=0.8*margin; %再留余量WZK 如果不乘系数进一步缩小运动空间的话会出错。
%             if obj.LegNum == 1
%                 hold on;
%             end
        end
        
        
        %根据机器人移动后的位置，判定落足点是否满足关节角约束；Fake代表还不是这个状态
        function CheckFlag = CheckWorldPointInLegSpaceFakeT_W_B(obj,p4_W_input,T_W_BFake) %检查世界坐标系下的坐标是否在某一腿的某一特定的TWB的工作空间内 在为1 不在为0 而下一个函数是当前状态下真实的 输入参数包括当前状态和虚拟状态
            CheckFlag=1;
            
            vector_p4_B_temp = T_W_BFake\[p4_W_input;1];    %移动后base坐标系下的足端位置
            vector_p4_O_temp = obj.T_B_0\vector_p4_B_temp;  %获得世界坐标系下的落足点在根关节坐标系下的表达
            
            %接下来就和求逆相同的操作了
            tmp_theta1 = atan2(vector_p4_O_temp(2),vector_p4_O_temp(1));
            N = vector_p4_O_temp(3);
            if(vector_p4_O_temp(2) == 0)
                M = vector_p4_O_temp(1) - 0.18;
            else
                M = vector_p4_O_temp(2)/sin(tmp_theta1) - 0.18;
            end
            if(sqrt(M^2 + N^2) > 0.5+0.5)
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
            else
                CheckFlag=0;
            end
        end
        %根据机器人移动后的位置，判定落足点是否在扇形区域内；Fake代表还不是这个状态
        function CheckFlag=CheckWorldPointInShanxinFakeT_W_B(obj,p4_W_input,FakeT_W_B)
            %base移动后的落足点
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
            CurrentSectorPlotData= T_W_FixLegP2Fake * [[x;y;z];ones(1,size(x,2))];  %世界系下的扇形点集坐标
            
            %通过求夹角判断是否在扇形之内
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
        
        
        
        %在当前状态下的可落足点集合中选择最好的落足点，也就是能沿着速度方向前进最大距离的落足点，如果没有落足点返回空集
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
            else
                BestFootPoint=[];
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
        
        
        %%%%%%%%%------------------------------------------
        %%%%%%%%这里可以用时间间隔先计算出一系列点，这样就不用重复调用该函数了
        %该腿按照六次样条曲线生成的轨迹移动，t3为摆动持续时间，time为当前时刻，六次曲线中间的的高度
        function obj=LegSwingActionTrajectory(obj,t3,time,StrideHight) 
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
        %腿部运动 相当于驱动器 执行器；获取每一个运动时刻足端的位置
        function obj=LegAction(obj,time,TimeInterval)
            obj.T_TimeFoot=time-obj.Time_Leg_Statr_Act;%当前周期足端的时间戳 六次规划曲线有可能不能求解出结果，需要人为移动
            if obj.T_TimeFoot<obj.Time_Move_Total_cost
                if obj.isSupport==0
                    temp=obj.LegSwingActionTrajectory(obj.Time_Move_Total_cost,obj.T_TimeFoot,obj.StrideHight); %获取当前time时刻下的摆动腿六次曲线轨迹上的位置
                    obj=temp;
                    obj.LegActionIng=1; %该腿正在动
                    if obj.LegNum==6 %画6号腿的轨迹上的离散点
                        plot3(obj.p4_W(1),obj.p4_W(2),obj.p4_W(3),'c.');
                    end
                end
            else
                if obj.T_TimeFoot>=obj.Time_Move_Total_cost && obj.T_TimeFoot<=obj.Time_Move_Total_cost+3*TimeInterval
                    if obj.isSupport==0
                        temp=obj.leg_move(obj.SwingLegEndPoint(1),obj.SwingLegEndPoint(2),obj.SwingLegEndPoint(3));
                        obj=temp;
                        obj.LegActionIng=0; %该腿动完了
                    end
                end
            end
        end
        
    end
    
    %Dependent属性的计算方法
    methods
        %根关节相对于world的T矩阵
        function T_W_1=get.T_W_1(obj)
            T_0_1 = matrixT(obj.theta1,0,0,0);  %二连杆的1号关节相对于根关节的T
            T_W_1 = obj.T_W_B*obj.T_B_0*T_0_1;  %二连杆的1号关节相对于世界坐标系的T
        end
        %二连杆1关节相对于world的T矩阵
        function T_W_2=get.T_W_2(obj)
            T_1_2 = matrixT(obj.theta2,pi/2,0.18,0);
            T_W_2 = obj.T_W_1*T_1_2;
        end
        %二连杆2关节相对于world的T矩阵
        function T_W_3=get.T_W_3(obj)
            T_2_3 = matrixT(obj.theta3,0,0.5,0);
            T_W_3 = obj.T_W_2*T_2_3;
        end
        %足端相对于world的T矩阵
        function T_W_4=get.T_W_4(obj)
            T_3_4 = matrixT(0,0,0.5,0);
            T_W_4 = obj.T_W_3*T_3_4;
        end
        
        %取出相对world中的位置列向量
        %根关节相对于world的位置列向量
        function p1_W=get.p1_W(obj)
            p1_W = obj.T_W_1(1:3,4);
        end
        %二连杆1关节相对于world的位置列向量
        function p2_W=get.p2_W(obj)
            p2_W = obj.T_W_2(1:3,4);
        end
        %二连杆2关节相对于world的位置列向量
        function p3_W=get.p3_W(obj)
            p3_W = obj.T_W_3(1:3,4);
        end
        %足端相对于world的位置列向量
        function p4_W=get.p4_W(obj)
            p4_W = obj.T_W_4(1:3,4);
        end
        
        %用相对于world的结果求解相对base的结果
        %根关节相对于base的位置列向量
        function p1_B=get.p1_B(obj)
            p1_B = obj.T_W_B\obj.T_W_1;
            p1_B = p1_B(1:3,4);
        end
        %二连杆1关节相对于base的位置列向量
        function p2_B=get.p2_B(obj)
            p2_B = obj.T_W_B\obj.T_W_2;
            p2_B = p2_B(1:3,4);
        end
        %二连杆2关节相对于base的位置列向量
        function p3_B=get.p3_B(obj) 
            p3_B = obj.T_W_B\obj.T_W_3;
            p3_B = p3_B(1:3,4);
        end
        %足端相对于base的位置列向量
        function p4_B=get.p4_B(obj)
            p4_B = obj.T_W_B\obj.T_W_4;
            p4_B = p4_B(1:3,4);
        end
        
        
        %扇形坐标系相对于base的坐标系的T矩阵
        function T_B_FixLegP2 = get.T_B_FixLegP2(obj)
            T_B_FixLegP2 = obj.T_B_0 * matrixT(0,pi/2,0.18,0)*[0,1,0,0;0,0,1,0;1,0,0,0;0,0,0,1];
        end
        %扇形坐标系相对于world的坐标系的T矩阵
        function T_W_FixLegP2 = get.T_W_FixLegP2(obj)
            T_W_FixLegP2 = obj.T_W_B*obj.T_B_0 * matrixT(0,pi/2,0.18,0)*[0,1,0,0;0,0,1,0;1,0,0,0;0,0,0,1];
        end
        
        %根据足端z，获得扇形点集
        function SectorPlotData = get.SectorPlotData(obj)
            % 计算足端点在固定2号坐标系下的位置，计算落足点在扇形坐标系下的位置
            p4_FixLegP2 = (obj.T_W_FixLegP2)\[obj.p4_W;1]; % 相当于inv(obj.T_W_FixLegP2)*obj.p4_W;
            
            tmp = p4_FixLegP2(3)+0.5;
            R = 0.8858*tmp^3 + -0.7787*tmp^2 + 0.5656*tmp + 0.8608; %扇形半径
%             sprintf('Leg%d的扇形半径为:%.4f',obj.LegNum,R)
            
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
        
    end
    
    
end


%返回值为1，则满足要求。
%检查关节角度是否超出限制
function flag = isAngleOutOfRange(theta1, theta2, theta3)
        flag = -2;
        %根关节：-40°至40°
        if (theta1 >= -40/180*pi) && (theta1 <= 40/180*pi)
            flag = flag + 1;
        end
        %二连杆关节1：-90°至90°
        if (theta2 >= -90/180*pi) && (theta2 <= 50/180*pi)
            flag = flag + 1;
        end
        %二连杆关节2：-150°至0°
        if (theta3 >= -150/180*pi) && (theta3 <= 0/180*pi)
            flag = flag + 1;
        end
end

%机器人学中的两个坐标系之间的齐次变换矩阵
function D = matrixT(theta,afa,a,d)
    D = [
        cos(theta), -sin(theta),0,a;
        cos(afa)*sin(theta), cos(afa)*cos(theta), -sin(afa),-d*sin(afa);
        sin(afa)*sin(theta), sin(afa)*cos(theta), cos(afa) , d*cos(afa);
        0 , 0 , 0, 1;
        ];
end

%获得T_B_0，单腿根关节相对base的T
function D = matrixBody_Leg(theta)
%机器人base正六边形边长为0.4m
    D = [
        cos(theta), -sin(theta),0,cos(theta)*0.4;
        sin(theta), cos(theta),0,sin(theta)*0.4;
        0, 0, 1 , 0;
        0 , 0 , 0, 1;
        ];
end
