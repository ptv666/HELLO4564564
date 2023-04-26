
FreeGait2();

%���ɲ�̬ ��2�汾 ������ŵ� �����ݴ� ������˷���
function FreeGait2()
    clc;clear;close all

    TimeInterval=0.1;%���������
    
    load('E:\bc\matlab\motion_planning\hexapod_robot\HEXAPOD_MATLAB\FootstepDesign.mat'); %����㼯�ͣ�ÿһ����һ�������
    TerrainMapConsiderFootMat=footStepData;
    save('TerrainMapConsiderFootMat1.mat','TerrainMapConsiderFootMat');
    
    HEXAPOD1=HEXAPOD(); %���������˶���
    HEXAPOD1.SupportStateListNew=HEXAPOD1.GenerateSupportStateListNew(); %�µİ���������ϵõ��ı� Ч�����ã�

    HEXAPOD1.SupportState=[1 1 1 1 1 1];    %��ʼ��֧��״̬
        
    DestinationPoint=[5 0 0];   %Ŀ���
    HEXAPOD1.MoveDestinationPoint=DestinationPoint; %��Ŀ�����Ϣ���������ˣ�������
    
    %���ݸ�������base����ϵ�µ��ƶ�������Ϣ���ƶ�������Ϣ����ȷ���ƶ��Ĳ�����base��һ��λ��
    Move2DestinationDirection=HEXAPOD1.MoveDestinationPoint-HEXAPOD1.GetBodyPosition(); %������
    Move2DestinationDirection=Move2DestinationDirection/norm(Move2DestinationDirection);
    Move2DestinationDirection=HEXAPOD1.T_W_B\[Move2DestinationDirection';1];    %�ƶ�������base����ϵ�µı�ʾ��������
    Move2DestinationDirection=Move2DestinationDirection(1:3,1)';    %��ȡbase����ϵ�µķ����ֱ�Ϊ������
    Move2DestinationDirection=Move2DestinationDirection/norm(Move2DestinationDirection);    %������
    HEXAPOD1.MoveDirectionVector=Move2DestinationDirection;     %������
    
    %����ڶ��ȵİڶ��߶�
    StrideHightsub=0.3; 
    HEXAPOD1.leg1.StrideHight=StrideHightsub;
    HEXAPOD1.leg2.StrideHight=StrideHightsub;
    HEXAPOD1.leg3.StrideHight=StrideHightsub;
    HEXAPOD1.leg4.StrideHight=StrideHightsub;
    HEXAPOD1.leg5.StrideHight=StrideHightsub;
    HEXAPOD1.leg6.StrideHight=StrideHightsub;
    
    CountStatic=0;      %��¼�Ƿ������ȶ����꣬�������ٹ滮��һ������
    LegActionDone=1;    %��¼���Ƿ���
    
    
    %��ʼ����ǰ���Ȱѻ����˰����ڳ�ʼλ��
    [HEXAPOD1,TerrainMapConsiderFootMat]=HEXAPOD1.PutLegsOnUseableTerrainAtStartTime(TerrainMapConsiderFootMat); 
    
    %��ʼ����,0:0.1:80,800��
    for time=0:TimeInterval:80
        
        %�滮��(��Ҫ���Ӿ��滮)
         if LegActionDone==1 %����Ƿ��˶���
            LegActionDone=0; %�ȱ�Ȼû�ж���
            
%             state=HEXAPOD1.GetCurrentSupportState(TerrainMapConsiderFootMat); %�ӵ�ͼ��ŵ���Ϣ�õ���ǰ֧��״̬��
            state = HEXAPOD1.SupportState;
            
            %�ӱ�ѡ״̬���� ��ר�ҷ��� ѡ����һ��֧��״̬
            State=HEXAPOD1.GetNextSupportStateFromSupportStateList(state);
            HEXAPOD1=HEXAPOD1.ChangeSupport(State);
            
            
            %����Ĭ���ƶ������ƶ�����ȷ����������ȷ�������ƶ����T���ٻ�ȡ�ڶ��ȿ�������㼯��
            HEXAPOD1.CurrentMaxStrideLength=HEXAPOD1.GetCurrentMaxStrideLength();   %��ȡ��󲽳�
            CurrentBodyStrideLength=HEXAPOD1.CurrentMaxStrideLength; %�����˻����ƶ���ô��Ĳ���

            HEXAPOD1.BodyStartPoint=HEXAPOD1.GetBodyPosition(); %������base��ǰλ��
            HEXAPOD1.BodyEndPoint=HEXAPOD1.BodyStartPoint+CurrentBodyStrideLength*HEXAPOD1.MoveDirectionVector; %��������һ��֮��base��ֹλ��
            
            BodyMoveDirectionVector=HEXAPOD1.MoveDirectionVector;   %[1 0 0]������
            BodyMoveStrideLength=HEXAPOD1.CurrentMaxStrideLength;   %bodyǰ�����룬Ҳ�����ƶ�����
            BodyMoveDirectionVector=BodyMoveDirectionVector/norm(BodyMoveDirectionVector); %�淶���ƶ�����
            TransitMatrix=[[1 0 0;0 1 0;0 0 1;0 0 0],[BodyMoveStrideLength*BodyMoveDirectionVector';1]]; %�����ƶ����ƶ���base��T����
            
            %��ȡ������㣬û�еľ����ݴ���
            TerrainMapVisionSensed=HEXAPOD1.SenseTerrainMapByVision(TerrainMapConsiderFootMat); %ͨ���Ӿ�����������Ϣ ��������̽�ⷶΧ���ޣ�
            HEXAPOD1=HEXAPOD1.GetUseableFootTerrain(TerrainMapVisionSensed,TransitMatrix); %��仰������ã�û����������͵�isdead
                        
            SwingStartPoint=HEXAPOD1.SwingPoint; %�õ���ǰ�ڶ��ȵ������Լ��ļ������ڰڶ�%�������ǰڶ��ȵı��
            
            %���ڶ��ȵĳ�ʼ���ĩ��洢�ڰڶ������������,���Ұ��������Ϊ��ɫ
            for i=1:size(SwingStartPoint,2)    
               if SwingStartPoint(size(SwingStartPoint,1),i)==1
                   [temp,point]=HEXAPOD1.leg1.SwingLegActionPlanning(SwingStartPoint(1:3,i),HEXAPOD1.MoveDirectionVector,TransitMatrix); %�ڶ��ȹ滮
                   if(temp.isDead == 0)
                        HEXAPOD1.terrain=HEXAPOD1.terrain.changeCellColor(point(1),point(2),'white');
                   end
                   HEXAPOD1.leg1=temp;
               end
               if SwingStartPoint(size(SwingStartPoint,1),i)==2
                   [temp,point]=HEXAPOD1.leg2.SwingLegActionPlanning(SwingStartPoint(1:3,i),HEXAPOD1.MoveDirectionVector,TransitMatrix); %�ڶ��ȹ滮
                   if(temp.isDead == 0)
                        HEXAPOD1.terrain=HEXAPOD1.terrain.changeCellColor(point(1),point(2),'white');
                   end
                   HEXAPOD1.leg2=temp;                 
               end
               if SwingStartPoint(size(SwingStartPoint,1),i)==3
                   [temp,point]=HEXAPOD1.leg3.SwingLegActionPlanning(SwingStartPoint(1:3,i),HEXAPOD1.MoveDirectionVector,TransitMatrix); %�ڶ��ȹ滮
                   if(temp.isDead == 0)
                        HEXAPOD1.terrain=HEXAPOD1.terrain.changeCellColor(point(1),point(2),'white');
                   end
                   HEXAPOD1.leg3=temp;
               end
               if SwingStartPoint(size(SwingStartPoint,1),i)==4
                   [temp,point]=HEXAPOD1.leg4.SwingLegActionPlanning(SwingStartPoint(1:3,i),HEXAPOD1.MoveDirectionVector,TransitMatrix); %�ڶ��ȹ滮
                   if(temp.isDead == 0)
                        HEXAPOD1.terrain=HEXAPOD1.terrain.changeCellColor(point(1),point(2),'white');
                   end
                   HEXAPOD1.leg4=temp;
               end
               if SwingStartPoint(size(SwingStartPoint,1),i)==5
                   [temp,point]=HEXAPOD1.leg5.SwingLegActionPlanning(SwingStartPoint(1:3,i),HEXAPOD1.MoveDirectionVector,TransitMatrix); %�ڶ��ȹ滮
                   if(temp.isDead == 0)
                        HEXAPOD1.terrain=HEXAPOD1.terrain.changeCellColor(point(1),point(2),'white');
                   end
                   HEXAPOD1.leg5=temp;                  
               end
               if SwingStartPoint(size(SwingStartPoint,1),i)==6
                   [temp,point]=HEXAPOD1.leg6.SwingLegActionPlanning(SwingStartPoint(1:3,i),HEXAPOD1.MoveDirectionVector,TransitMatrix); %�ڶ��ȹ滮
                   if(temp.isDead == 0)
                        HEXAPOD1.terrain=HEXAPOD1.terrain.changeCellColor(point(1),point(2),'white');
                   end
                   HEXAPOD1.leg6=temp;
               end
            end
            
            Time_Move_Body_Total=1; %������һ�������˶���ʱ��
            Time_Planning_Foot_Move_Total=1.5;%�����һ��������������ʱ��
            
            %�ڶ����˶�1.5s
            HEXAPOD1.leg1.Time_Move_Total_cost=Time_Planning_Foot_Move_Total;
            HEXAPOD1.leg2.Time_Move_Total_cost=Time_Planning_Foot_Move_Total;
            HEXAPOD1.leg3.Time_Move_Total_cost=Time_Planning_Foot_Move_Total;
            HEXAPOD1.leg4.Time_Move_Total_cost=Time_Planning_Foot_Move_Total;
            HEXAPOD1.leg5.Time_Move_Total_cost=Time_Planning_Foot_Move_Total;
            HEXAPOD1.leg6.Time_Move_Total_cost=Time_Planning_Foot_Move_Total;
            
            Time_Start_Body = time; %��ʼ�˶�ʱ��
            
            %�Ȳ���ʼ�˶�ʱ��
            HEXAPOD1.leg1.Time_Leg_Statr_Act=time;
            HEXAPOD1.leg2.Time_Leg_Statr_Act=time;
            HEXAPOD1.leg3.Time_Leg_Statr_Act=time;
            HEXAPOD1.leg4.Time_Leg_Statr_Act=time;
            HEXAPOD1.leg5.Time_Leg_Statr_Act=time;
            HEXAPOD1.leg6.Time_Leg_Statr_Act=time;
            
            %���Ƶ�ͼ����ɫ����Ĭ�ϵĻ�ɫ���߲�ͬ�ˣ����߻�Ϊ�˵���ɫ
            HEXAPOD1.terrain = HEXAPOD1.terrain.plotTerrain();
        end
        %�滮��������滮������һ��֧��״̬���ݴ�״̬���ڶ�������㡢�ڶ����˶�ʱ�䡢�ݴ�������㡢���Ŀ�ʼ�˶��ͽ����˶�λ��
        %ִ�в�
        temp=HEXAPOD1.leg1.LegAction(time,TimeInterval);%�Ȳ��˶��������˶������
        HEXAPOD1.leg1=temp;
        temp=HEXAPOD1.leg2.LegAction(time,TimeInterval);%�Ȳ��˶��������˶������
        HEXAPOD1.leg2=temp;
        temp=HEXAPOD1.leg3.LegAction(time,TimeInterval);%�Ȳ��˶��������˶������
        HEXAPOD1.leg3=temp;
        temp=HEXAPOD1.leg4.LegAction(time,TimeInterval);%�Ȳ��˶��������˶������
        HEXAPOD1.leg4=temp;
        temp=HEXAPOD1.leg5.LegAction(time,TimeInterval);%�Ȳ��˶��������˶������
        HEXAPOD1.leg5=temp;
        temp=HEXAPOD1.leg6.LegAction(time,TimeInterval);%�Ȳ��˶��������˶������
        HEXAPOD1.leg6=temp;
        
        %��ǰʱ�̾��뿪ʼ�˶������˶೤ʱ��
        delta_Time_Body=time-Time_Start_Body;
        if delta_Time_Body<Time_Move_Body_Total     %�˶�ʱ���Ƿ�С��1s����ô��ʹ����ζ���ʽ��������λ��
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
                    disp('���˶�ѧ�޷����,�����ƶ�ʧ��');
                end
                HEXAPOD1.BodyActionIng=0;
            end
        end
        %%

        %��û�ж���ʱ���ù滮��
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

