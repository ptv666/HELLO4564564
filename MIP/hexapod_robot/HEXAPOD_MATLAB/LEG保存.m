classdef LEG_SAVE
    %ʲô�Ǳ۶�����ϵ��-���Ա�����е�-���ؽ�����ϵ
    %ʲô��FlagPlotSphereDeadPoint����־�Ƿ񻭴����������е�����㣩��-���캯���е�-
    %T_W_FixLegP2��FixLegP2��ʲô����ϵ��-����������ϵ��
    properties 
        theta = []; %���ؽ����base��Ĭ�ϽǶȣ�-180�㡪180��
        %�����ؽڽ�
        theta1 = [];
        theta2 = [];
        theta3 = [];
        T_B_0 = [];%���嵽�۶ˣ����ؽڣ�����ϵ����α任��������0������ؽڣ��ڹ��캯���о�������
        T_W_B = [];%��������ϵ����������ϵ����α任����
        isInverseSolution = 0;%����ֵΪ1������ɹ����
        isSupport = 1;  %�Ƿ���֧����
        isDead=0;%�Ƿ��Ƿ���  ��״̬Ϊ1ʱ����ʾ����û������㣬����������֧���� ֻ���������ڶ���
        
        SwingLegStartPoint=[];  %�����ڰڶ����ڵĳ�ʼ��
        SwingLegEndPoint=[];    %�����ڰڶ����ڵ�ĩ��
        SwingLegPlanningPoint=[]; %�ڶ�����һ������֡�Ĺ滮λ��        
%         StrideHight=0.1; %�ڶ��Ȱڶ��ĸ߶� ���ĩ�����ߴ�ֱ����

        TrajectoryRecordH=[];%�ڰڶ����ڵĵ�洢������� �����ʱ�ò���
        LegNum = [];    %�ȵı�ţ�1-6
        StrideLength = 0; %�ðڶ��ȵĲ�������ĩ���ŷ�Ͼ���
        UsablePoint=[];%���õ�����㼯�� ÿһ���滮�˶���ѭ�������и���һ��
        
        Initp4BPoint=[];%��ʼ״̬������ڻ�������ϵ�µ�����
        CountTouchTimes=0; %��¼��һ���ڶ������´��ش����������д������䣩
        SetReflectthreshold=70;%���÷�����˷������ֵ �ڽ����ĳ����� ��ͬ���ȿ��ܷ�������ֵ��ͬ
        
        TPlanningTouchChangePoint=0;
        TPlanningFootMove=0;    %�ڶ����˶�ʱ��
        CurrentTimeLeg=0;       %��ǰ�滮ʱ��
        T_TimeFoot=0;
        StrideHight=0.1;%���Ȱڶ��Ĺ������ȵ������߶�
        
        TouchTryTimes=3; %���ط����ԵĴ��� ���������ô��λ����У�isdead=1�����ֵ��demo�б�����2
        
        FootSizeD=0.08;%���ֱ��
        PhysicsInformation=[];%�����ȴ�����������Ϣ
        
        TouchChangeFlag=1;%���ڷ����ĳ���滮 ��־λ
        LegActionIng=0;%��־�����Ƿ��ڶ� 0Ϊ���� 1Ϊ��
        FlagPlotSphereDeadPoint=0;%��־�Ƿ񻭴����������е�����㡣
    end
    
    properties (Dependent)  %�Զ�����
        T_W_1 = []; %���ؽ����world��T
        T_W_2 = []; %������1�ؽ����world��T
        T_W_3 = []; %������2�ؽ����world����T
        T_W_4 = []; %��������world��T
        p1_B = [];  %���ؽ������base��λ��
        p2_B = [];  %������1�ؽ����base��λ��
        p3_B = [];  %������2�ؽ����base��λ��
        p4_B = [];  %��������base��λ��
        p1_W = [];  %�����world��λ��
        p2_W = [];
        p3_W = [];
        p4_W = [];	%�����������λ��
        T_W_FixLegP2 = [];% �̶�2������ϵλ�õ���������ϵ����ת��������������ϵ��
        T_B_FixLegP2 = [];% �̶�2������ϵλ�õ������˻�������ϵ����ת����
        SectorPlotData = [];    %��������ϵ�����εĵ㼯��
    end
    

    
    
    methods %���캯���������ȵı�š������ؽڽǳ�ʼֵ�����ؽ���Ի����T�����������world��T
        function output = LEG(leg_Num)
            output.LegNum = leg_Num;    %�ȵı��
            output.theta = mod((60*leg_Num + 60),360);  %���ؽ���Ի����Ĭ�ϽǶ�
            if(output.theta>180)
                output.theta = output.theta-360;
            elseif(output.theta<-180)
                output.theta = output.theta+360;
            end
            output.theta = output.theta/180*pi;
            output.T_B_0 = matrixBody_Leg(output.theta);    %���ؽ�����ڻ����T
            %Ĭ��λ���µĹؽڽ�
            output.theta1 = 0;
            output.theta2 = 0;
            output.theta3 = -pi/2;
            output.T_W_B = eye(4);  %�������world��T��Ĭ��ʱ��������ϵ���غϵ�
            
            output.FlagPlotSphereDeadPoint=0;%��־�Ƿ񻭴����������е�����㡣
        end
    end
    
    methods
        function obj = setTheta(obj , theta_in_1,theta_in_2,theta_in_3) %���ùؽڽ�
            obj.theta1 = theta_in_1;
            obj.theta2 = theta_in_2;
            obj.theta3 = theta_in_3;
        end 
        
        function obj = OnrLegInverseKinematicsWoldAxis(obj,p4_W_input)  %������˶�ѧ������world����ϵ�µ����λ�ã���������
            obj.isInverseSolution = 0;
            vector_p4_B_temp = obj.T_W_B\[p4_W_input;1];    %obj.T_W_B�� * [p4_W_input;1]���ռ����base����ϵ�µ�����
            vector_p4_O_temp = obj.T_B_0\vector_p4_B_temp;  %�ռ���ڸ��ؽ�����ϵ�µ�����
            
            tmp_theta1 = atan2(vector_p4_O_temp(2),vector_p4_O_temp(1));    %�����ؽڵ�ת��
            
            %����������ϵֻ��xz���������ɵ�
            N = vector_p4_O_temp(3);    %�ڶ���������ϵ�µ�z����
            %����������ϵ�µ�x����
            if(vector_p4_O_temp(2) == 0)
                M = vector_p4_O_temp(1) - 0.18; %�����ؽڴ���ԭλʱ
            else
                M = vector_p4_O_temp(2)/sin(tmp_theta1) - 0.18; %�� ���ؽ�û�д���ԭλʱ
            end
            if(sqrt(M^2 + N^2) > 0.5+0.5)   %���λ�ó����˶����˵Ĺ����ռ䷶Χ
                disp('OUT OF RANGE1');
                return;
            end

            temp_acos =  acos( ( 0.5^2 + M^2 + N^2 - 0.5^2 ) / ( 2 * 0.5 * sqrt(M^2 + N^2) ) );
            if(abs(temp_acos) < 0.000001 ) 
                temp_acos = 0;  %��ֹacos����0.00000001+ 0.0000000000000i������
            end
            tmp_theta2 = atan2(N,M) + temp_acos;%��˫��
            tmp_theta3 = atan2( N - 0.5*sin(tmp_theta2) , M - 0.5*cos(tmp_theta2)) - tmp_theta2;
            
            if (isAngleOutOfRange(tmp_theta1,tmp_theta2,tmp_theta3)  == 1)
                
                obj.theta1 = tmp_theta1;
                obj.theta2 = tmp_theta2;
                obj.theta3 = tmp_theta3;
                obj.isInverseSolution = 1; %���ɹ�
                return;
            end
            disp('OUT OF RANGE2');
        end
        
        function obj=leg_move(obj,x,y,z) %���ƶ�����������ϵ�µ�x y z λ��
            p1(1,1) = x; 
            p1(2,1) = y; 
            p1(3,1) = z; 
            obj = obj.OnrLegInverseKinematicsWoldAxis(p1);
        end
        
        function obj=LegSwingActionTrajectory(obj,t3,time,StrideHight) %���Ȱ������������������ɵĹ켣�ƶ���t3Ϊ�ڶ�����ʱ�䣬timeΪ��ǰʱ�̣����������м�ĵĸ߶�
            %�����������ߵ�������
            p1=obj.SwingLegStartPoint';
            p2=obj.SwingLegStartPoint'+(obj.SwingLegEndPoint'-obj.SwingLegStartPoint')/2+[0 0 StrideHight];%����������й켣�Ǵ�ֱ�ڵ����
            p3=obj.SwingLegEndPoint';
            obj.SwingLegPlanningPoint=TrajectoryPlanning6(p1,p2,p3,t3,time);    %���timeʱ�����������ߵ�λ��
            obj.SwingLegPlanningPoint=obj.SwingLegPlanningPoint';   %������ת��Ϊ������
            %���ƶ���timeʱ�����������ߵ�λ��
            temp=obj.leg_move(obj.SwingLegPlanningPoint(1),obj.SwingLegPlanningPoint(2),obj.SwingLegPlanningPoint(3));
            obj=temp;
        end
        
        %�ú���û���õ�
        function obj=NewLegSwingActionTrajectory(obj,StartPoint,MidPoint,EndPoint,t3,time) %�ṩһģ�� ������������ t3Ϊ�ڶ�����ʱ�䣬timeΪ��ǰʱ��
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

        function T_W_1=get.T_W_1(obj)%���ؽ������world��T����
            T_0_1 = matrixT(obj.theta1,0,0,0);  %�����˵�1�Źؽ�����ڸ��ؽڵ�T
            T_W_1 = obj.T_W_B*obj.T_B_0*T_0_1;  %�����˵�1�Źؽ��������������ϵ��T
        end
        function T_W_2=get.T_W_2(obj)%������1�ؽ������world��T����
            T_1_2 = matrixT(obj.theta2,pi/2,0.18,0);
            T_W_2 = obj.T_W_1*T_1_2;
        end
        function T_W_3=get.T_W_3(obj)%������2�ؽ������world��T����
            T_2_3 = matrixT(obj.theta3,0,0.5,0);
            T_W_3 = obj.T_W_2*T_2_3;
        end
        function T_W_4=get.T_W_4(obj)%��������world��T����
            T_3_4 = matrixT(0,0,0.5,0);
            T_W_4 = obj.T_W_3*T_3_4;
        end
        
        function p1_B=get.p1_B(obj)%���ؽ������base��λ��������
            p1_B = obj.T_W_B\obj.T_W_1;
            p1_B = p1_B(1:3,4);
        end
        function p2_B=get.p2_B(obj)%������1�ؽ������base��λ��������
            p2_B = obj.T_W_B\obj.T_W_2;
            p2_B = p2_B(1:3,4);
        end
        function p3_B=get.p3_B(obj) %������2�ؽ������base��λ��������
            p3_B = obj.T_W_B\obj.T_W_3;
            p3_B = p3_B(1:3,4);
        end
        function p4_B=get.p4_B(obj)%��������base��λ��������
            p4_B = obj.T_W_B\obj.T_W_4;
            p4_B = p4_B(1:3,4);
        end
        
        function p1_W=get.p1_W(obj)%���ؽ������world��λ��������
            p1_W = obj.T_W_1(1:3,4);
        end
        function p2_W=get.p2_W(obj)%������1�ؽ������world��λ��������
            p2_W = obj.T_W_2(1:3,4);
        end
        function p3_W=get.p3_W(obj)%������2�ؽ������world��λ��������
            p3_W = obj.T_W_3(1:3,4);
        end
        function p4_W=get.p4_W(obj)%��������world��λ��������
            p4_W = obj.T_W_4(1:3,4);
        end
        
        
        function T_W_FixLegP2 = get.T_W_FixLegP2(obj)%��������ϵ�����world������ϵ��T����
            T_W_FixLegP2 = obj.T_W_B*obj.T_B_0 * matrixT(0,pi/2,0.18,0)*[0,1,0,0;0,0,1,0;1,0,0,0;0,0,0,1];
        end
        function T_B_FixLegP2 = get.T_B_FixLegP2(obj)%��������ϵ�����base������ϵ��T����
            T_B_FixLegP2 = obj.T_B_0 * matrixT(0,pi/2,0.18,0)*[0,1,0,0;0,0,1,0;1,0,0,0;0,0,0,1];
        end
        
        function SectorPlotData = get.SectorPlotData(obj)%�������z��������ε㼯
            % ������˵��ڹ̶�2������ϵ�µ�λ�ã��������������������ϵ�µ�λ��
            p4_FixLegP2 = (obj.T_W_FixLegP2)\[obj.p4_W;1]; % �൱��inv(obj.T_W_FixLegP2)*obj.p4_W;
            
            tmp = p4_FixLegP2(3)+0.5;
            R = 0.8858*tmp^3 + -0.7787*tmp^2 + 0.5656*tmp + 0.8608; %���ΰ뾶
            
            
            aplha= pi/4:pi/40:3*pi/4;   %�����Ž�
            x= R*cos(aplha);            %�����ϵĵ�x����
            y= R*sin(aplha);            %�����ϵĵ�y����
            
            %����������ϵ�£�Բ�ľ�������ԭ��
            x = [0,x,0];    
            y = [0,y,0];
            z = ones(1,size(x,2)) * p4_FixLegP2(3);

            %���̶�2������ϵ�µ����꣨���Σ�ת������������ϵ�£�������ʾ
            SectorPlotData =  obj.T_W_FixLegP2 * [[x;y;z];ones(1,size(x,2))];

        end
        
        %���ݻ������ƶ����λ�ã��ж�������Ƿ�����ؽڽ�Լ����Fake�����������״̬
        function CheckFlag = CheckWorldPointInLegSpaceFakeT_W_B(obj,p4_W_input,T_W_BFake) %�����������ϵ�µ������Ƿ���ĳһ�ȵ�ĳһ�ض���TWB�Ĺ����ռ��� ��Ϊ1 ����Ϊ0 ����һ�������ǵ�ǰ״̬����ʵ�� �������������ǰ״̬������״̬
            CheckFlag=1;
            obj.isInverseSolution = 0;
            vector_p4_B_temp = T_W_BFake\[p4_W_input;1]; %obj.T_W_B�� * [p4_W_input;1]
            vector_p4_O_temp = obj.T_B_0\vector_p4_B_temp;  %�����������ϵ�µ�������ڸ��ؽ�����ϵ�µı��
            
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
                temp_acos = 0;  %��ֹacos����0.00000001+ 0.0000000000000i������
            end
            tmp_theta2 = atan2(N,M) + temp_acos;%��˫��
            tmp_theta3 = atan2( N - 0.5*sin(tmp_theta2) , M - 0.5*cos(tmp_theta2)) - tmp_theta2;
            
            if (isAngleOutOfRange(tmp_theta1,tmp_theta2,tmp_theta3)  == 1)
                return;
            end
            
            CheckFlag=0;
        end
        %���ݻ������ƶ����λ�ã��ж�������Ƿ������������ڣ�Fake�����������״̬
        function CheckFlag=CheckWorldPointInShanxinFakeT_W_B(obj,p4_W_input,FakeT_W_B)
            %             Coordinate = obj.T_W_FixLegP2 \ [x;y;z;1]; %����������ϵ�µ�����ת�����̶�2������ϵ��
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
        %���ݻ����˵�ǰ��λ�ã��ж�������Ƿ��ڹؽڽǷ�Χ��
        function CheckFlag = CheckWorldPointInLegSpace(obj,p4_W_input) %�����������ϵ�µ������Ƿ���ĳһ�ȵĹ����ռ��� ��Ϊ1 ����Ϊ0
            CheckFlag=1;
            obj.isInverseSolution = 0;
            vector_p4_B_temp = obj.T_W_B\[p4_W_input;1]; %obj.T_W_B�� * [p4_W_input;1]
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
                temp_acos = 0;  %��ֹacos����0.00000001+ 0.0000000000000i������
            end
            tmp_theta2 = atan2(N,M) + temp_acos;%��˫��
            tmp_theta3 = atan2( N - 0.5*sin(tmp_theta2) , M - 0.5*cos(tmp_theta2)) - tmp_theta2;
            
            if (isAngleOutOfRange(tmp_theta1,tmp_theta2,tmp_theta3)  == 1)       
                return;
            end
            CheckFlag=0;
        end
        
        %�ڵ�ǰ״̬�µĿ�����㼯����ѡ����õ�����㣬Ҳ�����������ٶȷ���ǰ��������������
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
%                 BestFootPoint=obj.p4_W; %û�п�������ȾͲ���
                BestFootPoint=[];
%                 obj.isDead=1;%û�п�������Ⱦ����Ż��嶯����Ϊ�ȷ��ˡ�
%                 disp('��һ����û�п������');
%                 error('û�п������');
            end
        end
        
        % n Ϊ�ڻ�������ϵ�µ��˶�ʸ��,��õ������ε��˶�ԣ��
        function [lineData,margin] = getKinematicsMargin(obj,n)
            n=-n/norm(n);
            % ������˵��ڹ̶�2������ϵ�µ�λ��
            p4_FixLegP2 = (obj.T_B_FixLegP2)\[obj.p4_B;1]; % �൱��inv(obj.T_W_FixLegP2)*obj.p4_W;
            
            tmp = p4_FixLegP2(3)+0.5;
            R = 0.8858*tmp^3 + -0.7787*tmp^2 + 0.5656*tmp + 0.8608;
            
            %����̶�����ϵ�µ�ǰ������  nΪ��������ϵ�µ�ǰ��ʸ��
            tmp = [n';0] + [obj.p4_B;1];
            %����̶�2������ϵ�µ�ǰ��ʸ��
            tmp = (obj.T_B_FixLegP2)\tmp; % �൱��inv(obj.T_W_FixLegP2)*obj.p4_W;
            nn = [tmp(1)-p4_FixLegP2(1),tmp(2)-p4_FixLegP2(2)];
%             theta_tmp = atan2(,);
            n = nn/norm(nn);
            %���������εĽ���
            resultP_FixLegP2 =  GetIntersectionSector(R,p4_FixLegP2(1),p4_FixLegP2(2),n);
            %�����˶�ѧ�ȶ�ԣ��
            margin = sqrt( (p4_FixLegP2(1)-resultP_FixLegP2(1))^2 + (p4_FixLegP2(2)-resultP_FixLegP2(2))^2 );
%             margin=0.5*margin;
            %���㽻������������ϵ�µ�λ��
            resultP_W = obj.T_W_FixLegP2* [resultP_FixLegP2' ;p4_FixLegP2(3);1];
            %��ӵ�һ��������������ں��ڿ��ӻ�
            lineData = [obj.p4_W,resultP_W(1:3,1)];
            margin=norm([lineData(1,1),lineData(2,1)]-[lineData(1,2),lineData(2,2)]); %WZK
            margin=0.8*margin; %��������WZK �������ϵ����һ����С�˶��ռ�Ļ������
            if obj.LegNum == 1
                hold on;
            end
        end
        
        %�ڶ��ȹ滮,��ȡ����㣻������ݴ�����ô����Ĭ��λ����̧��0.2m
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
            obj.SwingLegEndPoint=point; %ȷ�����ȷŵ�ĩ��λ�ã����pointӦ�����ɵ�������������� �ӿ�
        end
        
        function PhysicsInformation=GetPhysicsInformationFromTouchTerrainMap(obj,TerrainMapConsiderFootMatIncludingTouch,leg_p4W) %�õ�leg_p4W���Ӧ��������Ϣ��
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
            disp('û�еõ���Ӧ��������Ϣ ���ܴ��ڵ�ԭ������ֵ���ò���ȷ������δ���ػ����㲻��ȷ�ķ�����ŵ��ϻ��߱�ɾ����');
        end
        
        function obj=ReflectLegChangeFootPoint(obj,StartPoint) %���������ʱ����Ҫ�ҵ��������ŵ�
            if (size(obj.UsablePoint,1)==0)
                disp('û�пɷ������������� �鵽��ʼλ��');
%                 point=obj.T_W_B*[obj.Initp4BPoint;1];
%                 point=point(1:3,:)+[0,0,0.2]';
%                 obj.SwingLegEndPoint=point;
                obj.isDead=1; %����û������㵱�����ȴ��� ����������֧���� ֻ���������ڶ���
                return;
            end
            StartPoint=StartPoint';
            obj.SwingLegStartPoint=StartPoint;
            obj.SwingLegEndPoint=obj.UsablePoint(1,:);
           
            Norm=10000;
            for i=1:size(obj.UsablePoint,1)
%                 PhysicsInformation=obj.GetPhysicsInformationFromTouchTerrainMap(TerrainMapConsiderFootMatIncludingTouch,obj.UsablePoint(i,:));
                if norm(obj.UsablePoint(i,:)-StartPoint)<Norm && norm(obj.UsablePoint(i,:)-StartPoint)>0.001 %������0��ֹѡ������㱾��
                    Norm=norm(obj.UsablePoint(i,:)-StartPoint);
                    obj.SwingLegEndPoint=obj.UsablePoint(i,:);
%                     if PhysicsInformation(1)>SetReflectthreshold%���������ֵ����Ϊ������
%                         obj.SwingLegEndPoint=obj.UsablePoint(i,:);
%                     else %����ͨ�������жϸõز������㣬ɾ����Ƭ����
%                         plot3(TerrainMapConsiderFootMatIncludingTouch(PhysicsInformation(2),1),TerrainMapConsiderFootMatIncludingTouch(PhysicsInformation(2),2),TerrainMapConsiderFootMatIncludingTouch(PhysicsInformation(2),3),'k*','LineWidth',4);
%                         TerrainMapConsiderFootMatIncludingTouch(PhysicsInformation(2),:)=[];
%                     end
%                 else
%                     if norm(obj.UsablePoint(i,:)-StartPoint)<0.001 %������������õ��߲��ɣ�������Ҫɾ��
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
        
        function obj=DeletePointFromUsablePoint(obj,point) %��һ��ӿ�ѡ�㼯����ɾ�� ������Ϊ�����жϲ�������
%             num=size(obj.UsablePoint,1);
            output=obj.UsablePoint;
            for i=1:size(obj.UsablePoint,1)
                if norm(point'-obj.UsablePoint(i,1:3))<0.001
                    output(i,:)=[];
                    break;
                end
            end
            if size(output,1)==size(obj.UsablePoint,1)
                disp('û�дӿ�ѡ�㼯�����ҵ��õ�');
            end
            obj.UsablePoint=output;
        end
        
        %һ���ȴ��غ��һϵ�й滮����
        function [obj,TerrainMapConsiderFootMatIncludingTouch1,PlotFlag]=ReflectLegActionPlanning(obj,MinusSupportState,TerrainMapConsiderFootMatIncludingTouch,time,TimeInterval)
            PlotFlag = 0;
            if MinusSupportState==1%��ʾ��ǰ��أ���һʱ�̰ڶ�
                obj.PhysicsInformation=obj.GetPhysicsInformationFromTouchTerrainMap(TerrainMapConsiderFootMatIncludingTouch,obj.p4_W');
                if obj.PhysicsInformation(1)<obj.SetReflectthreshold
                    obj.CountTouchTimes=obj.CountTouchTimes+1;
%                     ReflectLegList=[ReflectLegList,1];
%                     plot3(TerrainMapConsiderFootMatIncludingTouch(PhysicsInformation(2),1),TerrainMapConsiderFootMatIncludingTouch(PhysicsInformation(2),2),TerrainMapConsiderFootMatIncludingTouch(PhysicsInformation(2),3),'k*','LineWidth',4);
                    obj.FlagPlotSphereDeadPoint=1;
                    TerrainMapConsiderFootMatIncludingTouch(obj.PhysicsInformation(2),:)=[];
                    TerrainMapConsiderFootMatIncludingTouch1=TerrainMapConsiderFootMatIncludingTouch;
                    obj.TPlanningFootMove=obj.TPlanningTouchChangePoint+1*TimeInterval;%���մ����滮��������
                    obj.CurrentTimeLeg = time+TimeInterval;
                    temp=obj.DeletePointFromUsablePoint(obj.p4_W);
                    obj=temp;
                    temp=obj.ReflectLegChangeFootPoint(obj.p4_W);
                    obj=temp;
                    obj.TouchChangeFlag=0; %��ֹ�ٴν����ٹ滮
                    if obj.CountTouchTimes>obj.TouchTryTimes
                        obj.isDead=1;
                    end
                    
                else
%                     PhysicsInformation=obj.GetPhysicsInformationFromTouchTerrainMap(TerrainMapConsiderFootMatIncludingTouch,leg_p4W); %�õ�leg_p4W���Ӧ��������Ϣ��
%                     if obj.LegNum==6
%                         ss=1;
%                     end
                    obj.isDead=0;
                    obj.SwingLegStartPoint=obj.p4_W;
                    obj.SwingLegEndPoint=obj.p4_W;
                    obj.isSupport=1; 
                    obj.LegActionIng=0; %���ȶ�����
                    
                    PlotFlag = 1;
                end
            end
            TerrainMapConsiderFootMatIncludingTouch1=TerrainMapConsiderFootMatIncludingTouch;
        end
        %��isdead=1��ʱ��ִ�еķ����滮����
        function obj=ReflectDEADLegActionPlanning(obj,time,TimeInterval) 
            if obj.isDead==1 && obj.TouchChangeFlag==0
                obj.TouchChangeFlag=1; %�滮����뱣ִֻ֤��һ�Σ�����ִ�в�û������
                obj.SwingLegStartPoint=obj.p4_W;
                point=obj.T_W_B*[obj.Initp4BPoint;1];
                point=point(1:3,:)+[0,0,0.2]';
                obj.SwingLegEndPoint=point;
                obj.TPlanningFootMove=obj.TPlanningTouchChangePoint+1*TimeInterval;%���մ����滮�������� 7*interval
                obj.CurrentTimeLeg = time+TimeInterval;
            end
        end
        %�Ȳ��˶� �൱�������� ִ����
        function obj=LegAction(obj,time,TimeInterval)
            obj.T_TimeFoot=time-obj.CurrentTimeLeg;%��ǰ������˵�ʱ��� ���ι滮�����п��ܲ��������������Ҫ��Ϊ�ƶ�
            if obj.T_TimeFoot<obj.TPlanningFootMove
                if obj.isSupport==0
            %         if (time-CurrentTime<=(1-Beita)*Tgait)
                    temp=obj.LegSwingActionTrajectory(obj.TPlanningFootMove,obj.T_TimeFoot,obj.StrideHight); %���������Ĺ켣���������ԸĶ� A* rrt֮��ģ���Ҫ�ı��ڲ��ĺ�������
                    obj=temp;
                    obj.LegActionIng=1; %�������ڶ�
                    if obj.LegNum==6 %��6���ȵĹ켣
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
                        obj.LegActionIng=0; %���ȶ�����
                    end
                end
            end
        end
        
    end
    
    
    
end


%����ֵΪ1��������Ҫ��
%���ؽڽǶ��Ƿ񳬳�����
function flag = isAngleOutOfRange(theta1, theta2, theta3)
        flag = -2;
        %���ؽڣ�-40����40��
        if (theta1 >= -40/180*pi) && (theta1 <= 40/180*pi)   %��鸳ֵҪ��
            flag = flag + 1;
        end
        %�����˹ؽ�1��-90����90��
        if (theta2 >= -90/180*pi) && (theta2 <= 50/180*pi)   %��鸳ֵҪ��
            flag = flag + 1;
        end
        %�����˹ؽ�2��-150����0��
        if (theta3 >= -150/180*pi) && (theta3 <= 0/180*pi)   %��鸳ֵҪ��
            flag = flag + 1;
        end
end



