
FreeGait2();

%自由步态 第2版本 考虑落脚点 考虑容错 考虑足端反射
function FreeGait2()
    clc;clear;close all

    TimeInterval=0.1;%定义仿真间隔
    
    load('E:\bc\matlab\motion_planning\hexapod_robot\HEXAPOD_MATLAB\FootstepDesign.mat'); %落足点集和，每一行是一个落足点
    TerrainMapConsiderFootMat=footStepData;
    save('TerrainMapConsiderFootMat1.mat','TerrainMapConsiderFootMat');
    
    HEXAPOD1=HEXAPOD(); %创建机器人对象
    HEXAPOD1.SupportStateListNew=HEXAPOD1.GenerateSupportStateListNew(); %新的按照自由组合得到的表 效果不好！

    HEXAPOD1.SupportState=[1 1 1 1 1 1];    %初始化支撑状态
        
    DestinationPoint=[5 0 0];   %目标点
    HEXAPOD1.MoveDestinationPoint=DestinationPoint; %把目标点信息传给机器人，行向量
    
    %传递给机器人base坐标系下的移动方向信息，移动方向信息用于确定移动的步长，base下一个位置
    Move2DestinationDirection=HEXAPOD1.MoveDestinationPoint-HEXAPOD1.GetBodyPosition(); %行向量
    Move2DestinationDirection=Move2DestinationDirection/norm(Move2DestinationDirection);
    Move2DestinationDirection=HEXAPOD1.T_W_B\[Move2DestinationDirection';1];    %移动方向在base坐标系下的表示，列向量
    Move2DestinationDirection=Move2DestinationDirection(1:3,1)';    %提取base坐标系下的方向，又变为行向量
    Move2DestinationDirection=Move2DestinationDirection/norm(Move2DestinationDirection);    %行向量
    HEXAPOD1.MoveDirectionVector=Move2DestinationDirection;     %行向量
    
    %定义摆动腿的摆动高度
    StrideHightsub=0.3; 
    HEXAPOD1.leg1.StrideHight=StrideHightsub;
    HEXAPOD1.leg2.StrideHight=StrideHightsub;
    HEXAPOD1.leg3.StrideHight=StrideHightsub;
    HEXAPOD1.leg4.StrideHight=StrideHightsub;
    HEXAPOD1.leg5.StrideHight=StrideHightsub;
    HEXAPOD1.leg6.StrideHight=StrideHightsub;
    
    CountStatic=0;      %记录是否所有腿都动完，动完了再规划下一个动作
    LegActionDone=1;    %记录腿是否动完
    
    
    %开始仿真前，先把机器人安放在初始位置
    [HEXAPOD1,TerrainMapConsiderFootMat]=HEXAPOD1.PutLegsOnUseableTerrainAtStartTime(TerrainMapConsiderFootMat); 
    
    %开始仿真,0:0.1:80,800次
    for time=0:TimeInterval:80
        
        %规划层(主要是视觉规划)
         if LegActionDone==1 %足端是否运动完
            LegActionDone=0; %腿必然没有动完
            
%             state=HEXAPOD1.GetCurrentSupportState(TerrainMapConsiderFootMat); %从地图落脚点信息得到当前支撑状态表
            state = HEXAPOD1.SupportState;
            
            %从备选状态集中 “专家法” 选择下一个支撑状态
            State=HEXAPOD1.GetNextSupportStateFromSupportStateList(state);
            HEXAPOD1=HEXAPOD1.ChangeSupport(State);
            
            
            %按照默认移动方向移动，先确定步长，再确定机体移动后的T，再获取摆动腿可行落足点集合
            HEXAPOD1.CurrentMaxStrideLength=HEXAPOD1.GetCurrentMaxStrideLength();   %获取最大步长
            CurrentBodyStrideLength=HEXAPOD1.CurrentMaxStrideLength; %机器人机体移动这么大的步长

            HEXAPOD1.BodyStartPoint=HEXAPOD1.GetBodyPosition(); %机器人base当前位置
            HEXAPOD1.BodyEndPoint=HEXAPOD1.BodyStartPoint+CurrentBodyStrideLength*HEXAPOD1.MoveDirectionVector; %机器人走一步之后base终止位置
            
            BodyMoveDirectionVector=HEXAPOD1.MoveDirectionVector;   %[1 0 0]行向量
            BodyMoveStrideLength=HEXAPOD1.CurrentMaxStrideLength;   %body前进距离，也就是移动步长
            BodyMoveDirectionVector=BodyMoveDirectionVector/norm(BodyMoveDirectionVector); %规范化移动方向
            TransitMatrix=[[1 0 0;0 1 0;0 0 1;0 0 0],[BodyMoveStrideLength*BodyMoveDirectionVector';1]]; %虚拟移动，移动后base的T矩阵
            
            %获取可落足点，没有的就是容错腿
            TerrainMapVisionSensed=HEXAPOD1.SenseTerrainMapByVision(TerrainMapConsiderFootMat); %通过视觉检测落足点信息 传感器的探测范围有限！
            HEXAPOD1=HEXAPOD1.GetUseableFootTerrain(TerrainMapVisionSensed,TransitMatrix); %这句话里边设置：没有落足点类型的isdead
                        
            SwingStartPoint=HEXAPOD1.SwingPoint; %得到当前摆动腿的坐标以及哪几条腿在摆动%第四行是摆动腿的标号
            
            %将摆动腿的初始点和末点存储在摆动腿类的属性中,并且把落足点标记为白色
            for i=1:size(SwingStartPoint,2)    
               if SwingStartPoint(size(SwingStartPoint,1),i)==1
                   [temp,point]=HEXAPOD1.leg1.SwingLegActionPlanning(SwingStartPoint(1:3,i),HEXAPOD1.MoveDirectionVector,TransitMatrix); %摆动腿规划
                   if(temp.isDead == 0)
                        HEXAPOD1.terrain=HEXAPOD1.terrain.changeCellColor(point(1),point(2),'white');
                   end
                   HEXAPOD1.leg1=temp;
               end
               if SwingStartPoint(size(SwingStartPoint,1),i)==2
                   [temp,point]=HEXAPOD1.leg2.SwingLegActionPlanning(SwingStartPoint(1:3,i),HEXAPOD1.MoveDirectionVector,TransitMatrix); %摆动腿规划
                   if(temp.isDead == 0)
                        HEXAPOD1.terrain=HEXAPOD1.terrain.changeCellColor(point(1),point(2),'white');
                   end
                   HEXAPOD1.leg2=temp;                 
               end
               if SwingStartPoint(size(SwingStartPoint,1),i)==3
                   [temp,point]=HEXAPOD1.leg3.SwingLegActionPlanning(SwingStartPoint(1:3,i),HEXAPOD1.MoveDirectionVector,TransitMatrix); %摆动腿规划
                   if(temp.isDead == 0)
                        HEXAPOD1.terrain=HEXAPOD1.terrain.changeCellColor(point(1),point(2),'white');
                   end
                   HEXAPOD1.leg3=temp;
               end
               if SwingStartPoint(size(SwingStartPoint,1),i)==4
                   [temp,point]=HEXAPOD1.leg4.SwingLegActionPlanning(SwingStartPoint(1:3,i),HEXAPOD1.MoveDirectionVector,TransitMatrix); %摆动腿规划
                   if(temp.isDead == 0)
                        HEXAPOD1.terrain=HEXAPOD1.terrain.changeCellColor(point(1),point(2),'white');
                   end
                   HEXAPOD1.leg4=temp;
               end
               if SwingStartPoint(size(SwingStartPoint,1),i)==5
                   [temp,point]=HEXAPOD1.leg5.SwingLegActionPlanning(SwingStartPoint(1:3,i),HEXAPOD1.MoveDirectionVector,TransitMatrix); %摆动腿规划
                   if(temp.isDead == 0)
                        HEXAPOD1.terrain=HEXAPOD1.terrain.changeCellColor(point(1),point(2),'white');
                   end
                   HEXAPOD1.leg5=temp;                  
               end
               if SwingStartPoint(size(SwingStartPoint,1),i)==6
                   [temp,point]=HEXAPOD1.leg6.SwingLegActionPlanning(SwingStartPoint(1:3,i),HEXAPOD1.MoveDirectionVector,TransitMatrix); %摆动腿规划
                   if(temp.isDead == 0)
                        HEXAPOD1.terrain=HEXAPOD1.terrain.changeCellColor(point(1),point(2),'white');
                   end
                   HEXAPOD1.leg6=temp;
               end
            end
            
            Time_Move_Body_Total=1; %机体在一个周期运动的时间
            Time_Planning_Foot_Move_Total=1.5;%足端在一个周期内迈动的时间
            
            %摆动腿运动1.5s
            HEXAPOD1.leg1.Time_Move_Total_cost=Time_Planning_Foot_Move_Total;
            HEXAPOD1.leg2.Time_Move_Total_cost=Time_Planning_Foot_Move_Total;
            HEXAPOD1.leg3.Time_Move_Total_cost=Time_Planning_Foot_Move_Total;
            HEXAPOD1.leg4.Time_Move_Total_cost=Time_Planning_Foot_Move_Total;
            HEXAPOD1.leg5.Time_Move_Total_cost=Time_Planning_Foot_Move_Total;
            HEXAPOD1.leg6.Time_Move_Total_cost=Time_Planning_Foot_Move_Total;
            
            Time_Start_Body = time; %开始运动时刻
            
            %腿部开始运动时间
            HEXAPOD1.leg1.Time_Leg_Statr_Act=time;
            HEXAPOD1.leg2.Time_Leg_Statr_Act=time;
            HEXAPOD1.leg3.Time_Leg_Statr_Act=time;
            HEXAPOD1.leg4.Time_Leg_Statr_Act=time;
            HEXAPOD1.leg5.Time_Leg_Statr_Act=time;
            HEXAPOD1.leg6.Time_Leg_Statr_Act=time;
            
            %绘制地图，颜色就与默认的灰色边线不同了，边线换为了淡蓝色
            HEXAPOD1.terrain = HEXAPOD1.terrain.plotTerrain();
        end
        %规划层结束，规划出了下一个支撑状态、容错状态、摆动腿落足点、摆动腿运动时间、容错腿落足点、质心开始运动和结束运动位置
        %执行层
        temp=HEXAPOD1.leg1.LegAction(time,TimeInterval);%腿部运动，返回运动后的腿
        HEXAPOD1.leg1=temp;
        temp=HEXAPOD1.leg2.LegAction(time,TimeInterval);%腿部运动，返回运动后的腿
        HEXAPOD1.leg2=temp;
        temp=HEXAPOD1.leg3.LegAction(time,TimeInterval);%腿部运动，返回运动后的腿
        HEXAPOD1.leg3=temp;
        temp=HEXAPOD1.leg4.LegAction(time,TimeInterval);%腿部运动，返回运动后的腿
        HEXAPOD1.leg4=temp;
        temp=HEXAPOD1.leg5.LegAction(time,TimeInterval);%腿部运动，返回运动后的腿
        HEXAPOD1.leg5=temp;
        temp=HEXAPOD1.leg6.LegAction(time,TimeInterval);%腿部运动，返回运动后的腿
        HEXAPOD1.leg6=temp;
        
        %当前时刻距离开始运动经过了多长时间
        delta_Time_Body=time-Time_Start_Body;
        if delta_Time_Body<Time_Move_Body_Total     %运动时间是否小于1s，那么就使用五次多项式计算质心位置
            HEXAPOD1=HEXAPOD1.BodyActionTrajectory5(HEXAPOD1.BodyStartPoint,HEXAPOD1.BodyEndPoint,Time_Move_Body_Total,delta_Time_Body);
            HEXAPOD1.BodyActionIng=1;
        else
            if delta_Time_Body>=Time_Move_Body_Total && delta_Time_Body<Time_Move_Body_Total+2*TimeInterval
                [temp,flag]=HEXAPOD1.changePos_fixLegPosition_SupportState(HEXAPOD1.BodyEndPoint(1),HEXAPOD1.BodyEndPoint(2),HEXAPOD1.BodyEndPoint(3),0,0,0);
                if flag==1
                    HEXAPOD1=temp;
                    TrajectoryRecord=HEXAPOD1.BodyEndPoint';
                    HEXAPOD1.BodyTrajectoryRecordH=[HEXAPOD1.BodyTrajectoryRecordH TrajectoryRecord];
                else
                    disp('逆运动学无法求解,机体移动失败');
                end
                HEXAPOD1.BodyActionIng=0;
            end
        end
        %%

        %当没有动作时重置规划层
        if HEXAPOD1.BodyActionIng==0 && HEXAPOD1.leg1.LegActionIng==0 && HEXAPOD1.leg2.LegActionIng==0 && HEXAPOD1.leg3.LegActionIng==0 && HEXAPOD1.leg4.LegActionIng==0 && HEXAPOD1.leg5.LegActionIng==0 && HEXAPOD1.leg6.LegActionIng==0
            CountStatic=CountStatic+1;
            if CountStatic==1
                CountStatic=0;
                LegActionDone=1;
            end
        end
        
        HEXAPOD1=HEXAPOD1.plotHexapod();  
        
        
        pause(TimeInterval);
        

    end 
end

