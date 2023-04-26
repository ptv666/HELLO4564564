%% 对角步态函数
function DuiJiao33BuTai()
    clc
    clear 
    close all
    % close all
    HEXAPOD1=HEXAPOD();
    VectorVelocity=0.1*[1 1 0];
    X=[0 0 0];
    % [lineData,margin] = HEXAPOD1.leg1.getKinematicsMargin(VectorVelocity)
    % HEXAPOD1 = HEXAPOD1.plotHexapod();

    TimeInterval=0.1;
    Tgait=3; %步态周期
    Beita=0.5; %摆动相周期
    StateCount=0;
    StrideHight=0.4; %迈腿高度
    % TrajectoryRecordH=[0;0;0];    
    for time=0:TimeInterval:50
%         HEXAPOD1.CurrentMaxStrideLength=HEXAPOD1.GetCurrentMaxStrideLength();%这是一个动态变化的量 时变的 由这句话更新
        CurrentMaxStrideLength=HEXAPOD1.GetCurrentMaxStrideLength();

        
        HEXAPOD1.MoveDirectionVector = VectorVelocity;
        if mod(time,(1-Beita)*Tgait)==0
            StateCount=StateCount+1;
            if mod(StateCount,2)==0
                temp=HEXAPOD1.ChangeSupport([0 1 0 1 0 1]);
            else
                temp=HEXAPOD1.ChangeSupport([1 0 1 0 1 0]);
            end
                HEXAPOD1=temp;
                %             CurrentContactPoint=HEXAPOD1.ContactPoint;
                %            NextContactPoint=CurrentContactPoint+(HEXAPOD1.CurrentMaxStrideLength*(VectorVelocity/norm(VectorVelocity)))';          
                SwingStartPoint=HEXAPOD1.SwingPoint; %得到当前摆动腿的坐标以及哪几条腿在摆动%第四行是摆动腿的标号
                StrideLength=0.4; %迈步长
                HEXAPOD1.leg1.StrideLength=StrideLength;
                HEXAPOD1.leg2.StrideLength=StrideLength;
                HEXAPOD1.leg3.StrideLength=StrideLength;
                HEXAPOD1.leg4.StrideLength=StrideLength;
                HEXAPOD1.leg5.StrideLength=StrideLength;
                HEXAPOD1.leg6.StrideLength=StrideLength;
                for i=1:size(SwingStartPoint,2)    %将摆动腿的初始点存储在摆动腿类的一个属性中
                   if SwingStartPoint(size(SwingStartPoint,1),i)==1
                       HEXAPOD1.leg1.SwingLegStartPoint=SwingStartPoint(1:3,i);
    %                    HEXAPOD1.leg1.SwingLegEndPoint=HEXAPOD1.leg1.SwingLegStartPoint+(HEXAPOD1.CurrentMaxStrideLength*(VectorVelocity/norm(VectorVelocity)))'; %摆动腿的末位置 
                       HEXAPOD1.leg1.SwingLegEndPoint=HEXAPOD1.leg1.SwingLegStartPoint+HEXAPOD1.leg1.StrideLength*(VectorVelocity/norm(VectorVelocity))';
                   end
                   if SwingStartPoint(size(SwingStartPoint,1),i)==2
                       HEXAPOD1.leg2.SwingLegStartPoint=SwingStartPoint(1:3,i);
    %                    HEXAPOD1.leg2.SwingLegEndPoint=HEXAPOD1.leg2.SwingLegStartPoint+(HEXAPOD1.CurrentMaxStrideLength*(VectorVelocity/norm(VectorVelocity)))'; %摆动腿的末位置 
                       HEXAPOD1.leg2.SwingLegEndPoint=HEXAPOD1.leg2.SwingLegStartPoint+HEXAPOD1.leg2.StrideLength*(VectorVelocity/norm(VectorVelocity))';
                   end
                   if SwingStartPoint(size(SwingStartPoint,1),i)==3
                       HEXAPOD1.leg3.SwingLegStartPoint=SwingStartPoint(1:3,i);
    %                    HEXAPOD1.leg3.SwingLegEndPoint=HEXAPOD1.leg3.SwingLegStartPoint+(HEXAPOD1.CurrentMaxStrideLength*(VectorVelocity/norm(VectorVelocity)))'; %摆动腿的末位置 
                       HEXAPOD1.leg3.SwingLegEndPoint=HEXAPOD1.leg3.SwingLegStartPoint+HEXAPOD1.leg3.StrideLength*(VectorVelocity/norm(VectorVelocity))';
                   end
                   if SwingStartPoint(size(SwingStartPoint,1),i)==4
                       HEXAPOD1.leg4.SwingLegStartPoint=SwingStartPoint(1:3,i);
    %                    HEXAPOD1.leg4.SwingLegEndPoint=HEXAPOD1.leg4.SwingLegStartPoint+(HEXAPOD1.CurrentMaxStrideLength*(VectorVelocity/norm(VectorVelocity)))'; %摆动腿的末位置 
                       HEXAPOD1.leg4.SwingLegEndPoint=HEXAPOD1.leg4.SwingLegStartPoint+HEXAPOD1.leg4.StrideLength*(VectorVelocity/norm(VectorVelocity))';
                   end
                   if SwingStartPoint(size(SwingStartPoint,1),i)==5
                       HEXAPOD1.leg5.SwingLegStartPoint=SwingStartPoint(1:3,i);
    %                    HEXAPOD1.leg5.SwingLegEndPoint=HEXAPOD1.leg5.SwingLegStartPoint+(HEXAPOD1.CurrentMaxStrideLength*(VectorVelocity/norm(VectorVelocity)))'; %摆动腿的末位置 
                       HEXAPOD1.leg5.SwingLegEndPoint=HEXAPOD1.leg5.SwingLegStartPoint+HEXAPOD1.leg5.StrideLength*(VectorVelocity/norm(VectorVelocity))';
                   end
                   if SwingStartPoint(size(SwingStartPoint,1),i)==6
                       HEXAPOD1.leg6.SwingLegStartPoint=SwingStartPoint(1:3,i);
    %                    HEXAPOD1.leg6.SwingLegEndPoint=HEXAPOD1.leg6.SwingLegStartPoint+(HEXAPOD1.CurrentMaxStrideLength*(VectorVelocity/norm(VectorVelocity)))'; %摆动腿的末位置 
                       HEXAPOD1.leg6.SwingLegEndPoint=HEXAPOD1.leg6.SwingLegStartPoint+HEXAPOD1.leg6.StrideLength*(VectorVelocity/norm(VectorVelocity))';
                   end
                end
                CurrentTime=time;
        end

        if HEXAPOD1.leg1.isSupport==0
    %         if (time-CurrentTime<=(1-Beita)*Tgait)
            temp=HEXAPOD1.leg1.LegSwingActionTrajectory((1-Beita)*Tgait,time-CurrentTime,StrideHight);
            HEXAPOD1.leg1=temp;
    %         end
        end
        if HEXAPOD1.leg2.isSupport==0
    %         if (time-CurrentTime<=(1-Beita)*Tgait)
            temp=HEXAPOD1.leg2.LegSwingActionTrajectory((1-Beita)*Tgait,time-CurrentTime,StrideHight);
            HEXAPOD1.leg2=temp;
    %         end
        end
        if HEXAPOD1.leg3.isSupport==0
    %         if (time-CurrentTime<=(1-Beita)*Tgait)
            temp=HEXAPOD1.leg3.LegSwingActionTrajectory((1-Beita)*Tgait,time-CurrentTime,StrideHight);
            HEXAPOD1.leg3=temp;
    %         end
        end
        if HEXAPOD1.leg4.isSupport==0
    %         if (time-CurrentTime<=(1-Beita)*Tgait)
            temp=HEXAPOD1.leg4.LegSwingActionTrajectory((1-Beita)*Tgait,time-CurrentTime,StrideHight);
            HEXAPOD1.leg4=temp;
    %         end
        end
        if HEXAPOD1.leg5.isSupport==0
    %         if (time-CurrentTime<=(1-Beita)*Tgait)
            temp=HEXAPOD1.leg5.LegSwingActionTrajectory((1-Beita)*Tgait,time-CurrentTime,StrideHight);
            HEXAPOD1.leg5=temp;
    %         end
        end
        if HEXAPOD1.leg6.isSupport==0
    %         if (time-CurrentTime<=(1-Beita)*Tgait)
            temp=HEXAPOD1.leg6.LegSwingActionTrajectory((1-Beita)*Tgait,time-CurrentTime,StrideHight);
            HEXAPOD1.leg6=temp;
    %         end
        end


        X=X+TimeInterval*VectorVelocity;
    %     [temp,flag]=HEXAPOD1.changePos_135fixLegPosition(X(1),X(2),X(3),0,0,0);
        [temp,flag]=HEXAPOD1.changePos_fixLegPosition_SupportState(X(1),X(2),X(3),0,0,0);
        if flag==1
            HEXAPOD1=temp;

        end
        HEXAPOD1=HEXAPOD1.plotHexapod();
%         COG2PolygonDistance=HEXAPOD1.GetCOG2PolygonDistance()
        pause(TimeInterval);
    end
end

%% 自由步态 第一版本 不考虑落脚点
function FreeGait1()
    clc
    clear 
    close all
    % close all
    TimeInterval=0.1;%定义仿真间隔
    HEXAPOD1=HEXAPOD();
    VectorVelocity=0.1*[1 1 0]; %定义机体前进速度，但实际上用到五次轨迹规划后这个就没用了 但是仍能指导方向
% plot_gzkjdyt(HEXAPOD1);
    % [lineData,margin] = HEXAPOD1.leg1.getKinematicsMargin(VectorVelocity)

    TPlanning=1;      
    HEXAPOD1.SupportState=[1 1 1 1 1 1];  
    State=HEXAPOD1.SupportState;
    CountState=1;
    VectorVelocity=0.1*[1 1 0]; %定义机体前进速度，但实际上用到五次轨迹规划后这个就没用了 但是仍能指导方向
    %开始仿真
    for time=0:TimeInterval:50
%         HEXAPOD1.CurrentMaxStrideLength=HEXAPOD1.GetCurrentMaxStrideLength();%这是一个动态变化的量 时变的 由这句话更新
        HEXAPOD1.MoveDirectionVector = VectorVelocity/norm(VectorVelocity); %指明移动方向
%         T_TimeFoot=time-TPlanning*floor(time/TPlanning); %当前周期足端的时间戳
%         T_TimeBody=time-TPlanning*floor(time/TPlanning);%当前周期身体运动的时间戳
        
        %规划层
        if mod(time,TPlanning)==0 
            CountState=CountState+1;
            if CountState==4
                aaa=1;
%                 VectorVelocity=0.1*[1 0 0];
%                 error("s");
            end
            TPlanningFoot=1.5; %足端在一个周期内迈动的时间
            TPlanningBody=1; %机体在一个周期运动的时间
            TPlanning=max(TPlanningFoot,TPlanningBody)+10*TimeInterval; %一个规划周期
            
            
            
            State=HEXAPOD1.GetNextSupportState(State);
            SupportState=State; %每隔一个TPlanning，支撑状态改变一次 这个state根据环境改变 接口
            temp=HEXAPOD1.ChangeSupport(SupportState);
            HEXAPOD1=temp;
            
            HEXAPOD1.CurrentMaxStrideLength=HEXAPOD1.GetCurrentMaxStrideLength();
             
            CurrentBodyStrideLength=HEXAPOD1.CurrentMaxStrideLength; %这个应该是自适应的 跟环境有关 接口
            HEXAPOD1.BodyStartPoint=HEXAPOD1.GetBodyPosition();
            HEXAPOD1.BodyEndPoint=HEXAPOD1.BodyStartPoint+CurrentBodyStrideLength*HEXAPOD1.MoveDirectionVector;
            
            StrideHight=0.1; %迈腿高度 这个也与环境有关 接口
            SwingStartPoint=HEXAPOD1.SwingPoint; %得到当前摆动腿的坐标以及哪几条腿在摆动%第四行是摆动腿的标号
            for i=1:size(SwingStartPoint,2)    %将摆动腿的初始点和末点存储在摆动腿类的属性中
                   if SwingStartPoint(size(SwingStartPoint,1),i)==1
                       HEXAPOD1.leg1.SwingLegStartPoint=SwingStartPoint(1:3,i);
                       MoveDirectionMID=HEXAPOD1.MoveDirectionVector;
                       HEXAPOD1.MoveDirectionVector=-HEXAPOD1.MoveDirectionVector;
                       point=HEXAPOD1.leg1.SwingLegStartPoint+MoveDirectionMID'*(HEXAPOD1.CurrentMaxStrideLength+HEXAPOD1.leg1_kinematicsMargin);
                       HEXAPOD1.MoveDirectionVector=MoveDirectionMID;
    %                    HEXAPOD1.leg1.SwingLegEndPoint=HEXAPOD1.leg1.SwingLegStartPoint+(HEXAPOD1.CurrentMaxStrideLength*(VectorVelocity/norm(VectorVelocity)))'; %摆动腿的末位置 
                       HEXAPOD1.leg1.SwingLegEndPoint=point; %确定该腿放的末端位置，这个point应该是由地面的落足点决定的 接口
                   end
                   if SwingStartPoint(size(SwingStartPoint,1),i)==2
                       HEXAPOD1.leg2.SwingLegStartPoint=SwingStartPoint(1:3,i);
                       MoveDirectionMID=HEXAPOD1.MoveDirectionVector;
                       HEXAPOD1.MoveDirectionVector=-HEXAPOD1.MoveDirectionVector;
                       point=HEXAPOD1.leg2.SwingLegStartPoint+MoveDirectionMID'*(HEXAPOD1.CurrentMaxStrideLength+HEXAPOD1.leg2_kinematicsMargin);
                       HEXAPOD1.MoveDirectionVector=MoveDirectionMID;
    %                    HEXAPOD1.leg2.SwingLegEndPoint=HEXAPOD1.leg2.SwingLegStartPoint+(HEXAPOD1.CurrentMaxStrideLength*(VectorVelocity/norm(VectorVelocity)))'; %摆动腿的末位置 
                       HEXAPOD1.leg2.SwingLegEndPoint=point; %确定该腿放的末端位置
                   end
                   if SwingStartPoint(size(SwingStartPoint,1),i)==3
                       HEXAPOD1.leg3.SwingLegStartPoint=SwingStartPoint(1:3,i);
                       
                       MoveDirectionMID=HEXAPOD1.MoveDirectionVector;
                       HEXAPOD1.MoveDirectionVector=-HEXAPOD1.MoveDirectionVector;
                       point=HEXAPOD1.leg3.SwingLegStartPoint+MoveDirectionMID'*(HEXAPOD1.CurrentMaxStrideLength+HEXAPOD1.leg3_kinematicsMargin);
%                        if point(3)~=-0.5
%                            sss=1;
%                        end
                       HEXAPOD1.MoveDirectionVector=MoveDirectionMID;
    %                    HEXAPOD1.leg3.SwingLegEndPoint=HEXAPOD1.leg3.SwingLegStartPoint+(HEXAPOD1.CurrentMaxStrideLength*(VectorVelocity/norm(VectorVelocity)))'; %摆动腿的末位置 
                       HEXAPOD1.leg3.SwingLegEndPoint=point; %确定该腿放的末端位置
                   end
                   if SwingStartPoint(size(SwingStartPoint,1),i)==4
                       HEXAPOD1.leg4.SwingLegStartPoint=SwingStartPoint(1:3,i);
                       MoveDirectionMID=HEXAPOD1.MoveDirectionVector;
                       HEXAPOD1.MoveDirectionVector=-HEXAPOD1.MoveDirectionVector;
                       point=HEXAPOD1.leg4.SwingLegStartPoint+MoveDirectionMID'*(HEXAPOD1.CurrentMaxStrideLength+HEXAPOD1.leg4_kinematicsMargin);
                       HEXAPOD1.MoveDirectionVector=MoveDirectionMID;
    %                    HEXAPOD1.leg4.SwingLegEndPoint=HEXAPOD1.leg4.SwingLegStartPoint+(HEXAPOD1.CurrentMaxStrideLength*(VectorVelocity/norm(VectorVelocity)))'; %摆动腿的末位置 
                       HEXAPOD1.leg4.SwingLegEndPoint=point; %确定该腿放的末端位置
                   end
                   if SwingStartPoint(size(SwingStartPoint,1),i)==5
                       HEXAPOD1.leg5.SwingLegStartPoint=SwingStartPoint(1:3,i);
                       MoveDirectionMID=HEXAPOD1.MoveDirectionVector;
                       HEXAPOD1.MoveDirectionVector=-HEXAPOD1.MoveDirectionVector;
                       point=HEXAPOD1.leg5.SwingLegStartPoint+MoveDirectionMID'*(HEXAPOD1.CurrentMaxStrideLength+HEXAPOD1.leg5_kinematicsMargin);
                       HEXAPOD1.MoveDirectionVector=MoveDirectionMID;
    %                    HEXAPOD1.leg5.SwingLegEndPoint=HEXAPOD1.leg5.SwingLegStartPoint+(HEXAPOD1.CurrentMaxStrideLength*(VectorVelocity/norm(VectorVelocity)))'; %摆动腿的末位置 
                       HEXAPOD1.leg5.SwingLegEndPoint=point; %确定该腿放的末端位置
                   end
                   if SwingStartPoint(size(SwingStartPoint,1),i)==6
                       HEXAPOD1.leg6.SwingLegStartPoint=SwingStartPoint(1:3,i);
                       MoveDirectionMID=HEXAPOD1.MoveDirectionVector;
                       HEXAPOD1.MoveDirectionVector=-HEXAPOD1.MoveDirectionVector;
                       point=HEXAPOD1.leg6.SwingLegStartPoint+MoveDirectionMID'*(HEXAPOD1.CurrentMaxStrideLength+HEXAPOD1.leg6_kinematicsMargin);
                       HEXAPOD1.MoveDirectionVector=MoveDirectionMID;
    %                    HEXAPOD1.leg6.SwingLegEndPoint=HEXAPOD1.leg6.SwingLegStartPoint+(HEXAPOD1.CurrentMaxStrideLength*(VectorVelocity/norm(VectorVelocity)))'; %摆动腿的末位置 
                       HEXAPOD1.leg6.SwingLegEndPoint=point; %确定该腿放的末端位置
                   end
            end
            CurrentTime = time;
        end
        
        %执行层
        T_TimeFoot=time-CurrentTime;%当前周期足端的时间戳
        if T_TimeFoot<=TPlanningFoot
            if HEXAPOD1.leg1.isSupport==0
        %         if (time-CurrentTime<=(1-Beita)*Tgait)
                temp=HEXAPOD1.leg1.LegSwingActionTrajectory(TPlanningFoot,T_TimeFoot,StrideHight); %按照这样的轨迹迈步，可以改动 A* rrt之类的，需要改变内部的函数内容
                HEXAPOD1.leg1=temp;
        %         end
            end
            if HEXAPOD1.leg2.isSupport==0
        %         if (time-CurrentTime<=(1-Beita)*Tgait)
                temp=HEXAPOD1.leg2.LegSwingActionTrajectory(TPlanningFoot,T_TimeFoot,StrideHight);
                HEXAPOD1.leg2=temp;
        %         end
            end
            if HEXAPOD1.leg3.isSupport==0
        %         if (time-CurrentTime<=(1-Beita)*Tgait)
                temp=HEXAPOD1.leg3.LegSwingActionTrajectory(TPlanningFoot,T_TimeFoot,StrideHight);
                HEXAPOD1.leg3=temp;
        %         end
            end
            if HEXAPOD1.leg4.isSupport==0
        %         if (time-CurrentTime<=(1-Beita)*Tgait)
                temp=HEXAPOD1.leg4.LegSwingActionTrajectory(TPlanningFoot,T_TimeFoot,StrideHight);
                HEXAPOD1.leg4=temp;
        %         end
            end
            if HEXAPOD1.leg5.isSupport==0
        %         if (time-CurrentTime<=(1-Beita)*Tgait)
                temp=HEXAPOD1.leg5.LegSwingActionTrajectory(TPlanningFoot,T_TimeFoot,StrideHight);
                HEXAPOD1.leg5=temp;
        %         end
            end
            if HEXAPOD1.leg6.isSupport==0
        %         if (time-CurrentTime<=(1-Beita)*Tgait)
                temp=HEXAPOD1.leg6.LegSwingActionTrajectory(TPlanningFoot,T_TimeFoot,StrideHight);
                HEXAPOD1.leg6=temp;
        %         end
            end
        end
        
        T_TimeBody=time-CurrentTime;%当前周期身体运动的时间戳
        if T_TimeBody<=TPlanningBody
            temp=HEXAPOD1.BodyActionTrajectory5(HEXAPOD1.BodyStartPoint,HEXAPOD1.BodyEndPoint,TPlanningBody,T_TimeBody);
            HEXAPOD1=temp;
        end
        
        HEXAPOD1=HEXAPOD1.plotHexapod();
        pause(TimeInterval);
    end
end

%% 
function plot_gzkjdyt(HEXAPOD1) %绘制工作空间的点云图
    figure
    for i=1:10000
        c = -1.5 + (1.5+1.5).*rand(1,3);
               if HEXAPOD1.leg1.CheckWorldPointInLegSpace([c(1);c(2);c(3)])
                   plot3(c(1),c(2),c(3),'r.');
                   hold on
                   drawnow
               end
               if HEXAPOD1.leg2.CheckWorldPointInLegSpace([c(1);c(2);c(3)])
                   plot3(c(1),c(2),c(3),'b.');
                   hold on
                   drawnow
               end
               if HEXAPOD1.leg3.CheckWorldPointInLegSpace([c(1);c(2);c(3)])
                   plot3(c(1),c(2),c(3),'y.');
                   hold on
                   drawnow
               end
               if HEXAPOD1.leg4.CheckWorldPointInLegSpace([c(1);c(2);c(3)])
                   plot3(c(1),c(2),c(3),'g.');
                   hold on
                   drawnow
               end
               if HEXAPOD1.leg5.CheckWorldPointInLegSpace([c(1);c(2);c(3)])
                   plot3(c(1),c(2),c(3),'m.');
                   hold on
                   drawnow
               end
               if HEXAPOD1.leg6.CheckWorldPointInLegSpace([c(1);c(2);c(3)])
                   plot3(c(1),c(2),c(3),'k.');
                   hold on
                   drawnow
               end
    end
end
%% 
function jpg2avi()
    clc
    % clear
    % close all
        route='D:\Study\硕士\我的工作\机器人学 (1)\机器人学 (1)\image\';%基本路径
    name='';%
    d=dir([route name '*.bmp']);%.jpg格式

    WriterObj=VideoWriter('D:\Study\硕士\我的工作\机器人学 (1)\机器人学 (1)\image\Bolt.avi');%待合成的视频(不仅限于avi格式)的文件路径
    open(WriterObj);

    n_frames=numel(d);% n_frames表示图像帧的总数
    for i=1:n_frames 
    frame=imread([route name '' d(i).name]);%读取图像，放在变量frame中
    writeVideo(WriterObj,frame);%将frame放到变量WriterObj中
    %%为每一帧图像编号
    %imshow(frame);
    %text(5,18,num2str(i),'color','y','Fontweight','bold','FontSize',18);
    %writeVideo(WriterObj,frame2im(getframe(gcf)));
    end
    close(WriterObj);

end

%% 录制视频
%把视频录制下来的操作
frame=getframe(gcf);
imind=frame2im(frame);
[imind,cm] = rgb2ind(imind,256);

if gifFlag~=99
    imwrite(imind,cm,'test4.gif','gif', 'Loopcount',inf,'DelayTime',1e-4);
    gifFlag = 99;
else
    imwrite(imind,cm,'test4.gif','gif','WriteMode','append','DelayTime',1e-4);
end

