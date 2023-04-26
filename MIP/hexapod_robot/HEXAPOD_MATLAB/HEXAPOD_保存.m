classdef HEXAPOD
    properties
        %��������
        leg1 = LEG(1);leg2 = LEG(2); leg3 = LEG(3);
        leg4 = LEG(4);leg5 = LEG(5);leg6 = LEG(6);
        
        %��������
        terrain= TERRAIN();
        
        %������֧��״̬��Ϣ
        SupportState=[1 1 1 1 1 1]; %��ʾ��ǰ֧��״̬
        SupportStateList = [];      %֧��״̬�б�֧���ȵ���Ŀ����3
        SupportStateListNew = [];   %�µ�֧��״̬�б�
        
        %body�˶���Ϣ
        CurrentMaxStrideLength = 0; %��ǰ֧��״̬�¿��ߵ���󲽳�
        BodyStartPoint = [];        %������һ���˶����ڵĳ�λ��
        BodyEndPoint = [];          %������һ���˶����ڵ�ĩλ��
        BodyActionIng = 0;          %��־�����Ƿ��ڶ� 0Ϊ���� 1Ϊ��
        BodyTrajectoryRecordH = []; %�����˶���洢�������
        
        %һЩȫ����Ϣ
        MoveDirectionVector = [1,0,0];  %base����ϵ�µ��ƶ�������demo�ж������¸�ֵ��
        MoveDestinationPoint = [];      %�����ƶ���Ŀ���
        VisionSensorRange = 5;          %�Ӿ��������ĸ�֪��Χ
    end
    
    properties (Access = 'private')
        roll = [];  %������̬
        pitch = [];
        yaw = [];
        x = []; %����λ��
        y = [];
        z = [];
        
    end
  
    properties (Access = 'private') %������ʾ�ġ���������
        %�ȵ�����ֱ��
        leg1_line = line();leg2_line = line();leg3_line = line();
        leg4_line = line();leg5_line = line();leg6_line = line();
        %�ȵ�����
        leg1_sector_line = line();leg2_sector_line = line();leg3_sector_line = line();
        leg4_sector_line = line();leg5_sector_line = line();leg6_sector_line = line();
        %�����ε��˶�ԣ��
        leg1_kinematics_line = line();leg2_kinematics_line = line();leg3_kinematics_line = line();
        leg4_kinematics_line = line();leg5_kinematics_line = line();leg6_kinematics_line = line();
        
        %�����˻���������
        lineHexagon = line();
        %��������ϵ
        frame_base_x_line = line();frame_base_y_line = line();frame_base_z_line = line();
        
        
        
        %֧�Ŷ���ξ�̬�ȶ�ԣ��
        shortestMarginLine = line();
        %֧�Ŷ����
        marginArea = line();        
        %��ʾǰ����������ǰ������
        COG2PolygonLine=line();
        
        %�ݴ�����˵�������ʾ
        DeadPointleg1=line();DeadPointleg2=line();DeadPointleg3=line();
        DeadPointleg4=line();DeadPointleg5=line();DeadPointleg6=line();
        
        %�����ϵĹؽں����
        Pointgj1=line();Pointgj2=line();Pointgj3=line();Pointgj4=line();Pointgj5=line();Pointgj6=line();Pointgj7=line();Pointgj8=line();
        Pointgj9=line();Pointgj10=line();Pointgj11=line();Pointgj12=line();Pointgj13=line();Pointgj14=line();Pointgj15=line();Pointgj16=line();
        Pointgj17=line();Pointgj18=line();Pointgj19=line();Pointgj20=line();Pointgj21=line();Pointgj22=line();Pointgj23=line();Pointgj24=line();
    end
    
    %������ؽ��������ڻ�������ϵ�µ�λ�ã��̶����䣩
    properties (Constant,Access = 'private')   
        PA1_B = [-0.2;0.2*sqrt(3);0];
        PA2_B = [-0.4;0;0];
        PA3_B = [-0.2;-0.2*sqrt(3);0];
        PA4_B = [0.2;-0.2*sqrt(3);0];
        PA5_B = [0.4;0;0];
        PA6_B = [0.2;0.2*sqrt(3);0];
        
        Ob_B = [0;0;0];
        Oxb_B = [0.15;0;0];
        oyb_B = [0;0.15;0];
        ozb_B = [0;0;0.15];
    end
    
    properties (Dependent)
        EulerAngle;
        Bxyz_W;     %����λ������
        T_W_B;      %base�����world��T����
        BodyMargin; %�����ȶ�ԣ�ȣ�������ε���̱�
        
        %���ȵ������ȶ�ԣ��
        leg1_kinematicsMargin;leg2_kinematicsMargin;leg3_kinematicsMargin;
        leg4_kinematicsMargin;leg5_kinematicsMargin;leg6_kinematicsMargin;
        
        ContactPoint;   %��ǰ֧�ֽŵ�ȫ��λ�ã�3��n��ÿһ�д�����֧��λ��
        SwingPoint;     %�ڶ���λ��
    end

    properties (Dependent,Access = 'private')
        %С��������ϵ��λ������ĩ��
        Oxb_W;Oyb_W;Ozb_W;
        
        %���ӻ���������������
        PA1_W;PA2_W;PA3_W;PA4_W;PA5_W;PA6_W;
        
        %֧�Ŷ���ε��ȶ�ԣ��
        MinMarginIntersection;
    end
    
    
    methods 
        %���캯��
        function output = HEXAPOD(~)
            output.roll = 0;output.pitch = 0;output.yaw = 0;
            output.x = 0;output.y = 0;output.z = 0;
            output.leg1 = LEG(1);output.leg2 = LEG(2);output.leg3 = LEG(3);
            output.leg4 = LEG(4);output.leg5 = LEG(5);output.leg6 = LEG(6);
            output.terrain = TERRAIN();
            
            SupportListMid=output.GenerateSupportStateListNew(); %��̬�ȶ���֧��״̬��
            mid=SupportListMid(:,4);
            SupportListMid(:,4)=SupportListMid(:,6);
            SupportListMid(:,6)=mid;
            output.SupportStateList=SupportListMid;

            output = output.plotHexapod();
        end

        % ���ӻ����������ظ����ã�ɾ���ɵģ������µ�
        %���캯��ͬʱ��ͼʹ�õģ����ƣ������˻��������Ρ������������ȵ����ˡ������ȵ�18���ؽڡ�6����ˡ��ݴ��ȵ�������ʾ��
        %6�����Ρ�6��������ǰ�����򽻵㡢֧�Ŷ���Ρ����ĵ�֧�Ŷ������̾��롢����ǰ�������֧�Ŷ���ν���
        function obj=plotHexapod(obj)
            delete(obj.leg1_line);  %�����ȣ�ʵ���Ͼ��Ƕ�����
            delete(obj.leg2_line);
            delete(obj.leg3_line);
            delete(obj.leg4_line);
            delete(obj.leg5_line);
            delete(obj.leg6_line);
            delete(obj.lineHexagon);    %���壬��������
            delete(obj.frame_base_x_line);
            delete(obj.frame_base_y_line);
            delete(obj.frame_base_z_line);
            delete(obj.leg1_sector_line);   %ÿ���ȵ�����
            delete(obj.leg2_sector_line);
            delete(obj.leg3_sector_line);
            delete(obj.leg4_sector_line);
            delete(obj.leg5_sector_line);
            delete(obj.leg6_sector_line);
            delete(obj.leg1_kinematics_line);   %ÿ�������ε��ȶ�ԣ��
            delete(obj.leg2_kinematics_line);
            delete(obj.leg3_kinematics_line);
            delete(obj.leg4_kinematics_line);
            delete(obj.leg5_kinematics_line);
            delete(obj.leg6_kinematics_line);
            
            delete(obj.marginArea);         
            delete(obj.shortestMarginLine);
            delete(obj.COG2PolygonLine);
            
            delete(obj.DeadPointleg1);  %û����������
            delete(obj.DeadPointleg2);
            delete(obj.DeadPointleg3);
            delete(obj.DeadPointleg4);
            delete(obj.DeadPointleg5);
            delete(obj.DeadPointleg6);
            
            delete(obj.Pointgj1);   %��������18���ؽڣ�����6�����
            delete(obj.Pointgj2);
            delete(obj.Pointgj3);
            delete(obj.Pointgj4);
            delete(obj.Pointgj5);
            delete(obj.Pointgj6);
            delete(obj.Pointgj7);
            delete(obj.Pointgj8);
            delete(obj.Pointgj9);
            delete(obj.Pointgj10);
            delete(obj.Pointgj11);
            delete(obj.Pointgj12);
            delete(obj.Pointgj13);
            delete(obj.Pointgj14);
            delete(obj.Pointgj15);
            delete(obj.Pointgj16);
            delete(obj.Pointgj17);
            delete(obj.Pointgj18);
            delete(obj.Pointgj19);
            delete(obj.Pointgj20);
            delete(obj.Pointgj21);
            delete(obj.Pointgj22);
            delete(obj.Pointgj23);
            delete(obj.Pointgj24);
            
            %���ݻ�������ϵ����������ϵ����ת����
            obj.leg1.T_W_B = obj.T_W_B;
            obj.leg2.T_W_B = obj.T_W_B;
            obj.leg3.T_W_B = obj.T_W_B;
            obj.leg4.T_W_B = obj.T_W_B;
            obj.leg5.T_W_B = obj.T_W_B;
            obj.leg6.T_W_B = obj.T_W_B;
            
            %����������������ȫ�Ǻ�ɫ
            obj.leg1_sector_line = plot3(obj.leg1.SectorPlotData(1,:),obj.leg1.SectorPlotData(2,:),obj.leg1.SectorPlotData(3,:),':','Color',[0.1 0.1 0.1],'LineWidth',1.5);
            obj.leg2_sector_line = plot3(obj.leg2.SectorPlotData(1,:),obj.leg2.SectorPlotData(2,:),obj.leg2.SectorPlotData(3,:),':','Color',[0.1 0.1 0.1],'LineWidth',1.5);
            obj.leg3_sector_line = plot3(obj.leg3.SectorPlotData(1,:),obj.leg3.SectorPlotData(2,:),obj.leg3.SectorPlotData(3,:),':','Color',[0.1 0.1 0.1],'LineWidth',1.5);
            obj.leg4_sector_line = plot3(obj.leg4.SectorPlotData(1,:),obj.leg4.SectorPlotData(2,:),obj.leg4.SectorPlotData(3,:),':','Color',[0.1 0.1 0.1],'LineWidth',1.5);
            obj.leg5_sector_line = plot3(obj.leg5.SectorPlotData(1,:),obj.leg5.SectorPlotData(2,:),obj.leg5.SectorPlotData(3,:),':','Color',[0.1 0.1 0.1],'LineWidth',1.5);
            obj.leg6_sector_line = plot3(obj.leg6.SectorPlotData(1,:),obj.leg6.SectorPlotData(2,:),obj.leg6.SectorPlotData(3,:),':','Color',[0.1 0.1 0.1],'LineWidth',1.5);
            
            %���Ƶ��Ⱥ����εĽ��㣬�ӵ�ǰ����λ�õ������ν���
            [lineData,~] = obj.leg1.getKinematicsMargin(obj.MoveDirectionVector);
            obj.leg1_kinematics_line = plot3(lineData(1,:),lineData(2,:),lineData(3,:),'g-');   %��ɫ������֮ǰ
            [lineData,~] = obj.leg2.getKinematicsMargin(obj.MoveDirectionVector);
            obj.leg2_kinematics_line = plot3(lineData(1,:),lineData(2,:),lineData(3,:),'g-');   %��ɫ
            [lineData,~] = obj.leg3.getKinematicsMargin(obj.MoveDirectionVector);
            obj.leg3_kinematics_line = plot3(lineData(1,:),lineData(2,:),lineData(3,:),'g-');   %��ɫ
            [lineData,~] = obj.leg4.getKinematicsMargin(obj.MoveDirectionVector);
            obj.leg4_kinematics_line = plot3(lineData(1,:),lineData(2,:),lineData(3,:),'g-');   %��ɫ
            [lineData,~] = obj.leg5.getKinematicsMargin(obj.MoveDirectionVector);
            obj.leg5_kinematics_line = plot3(lineData(1,:),lineData(2,:),lineData(3,:),'g-','LineWidth',3);   %��ɫ
            [lineData,~] = obj.leg6.getKinematicsMargin(obj.MoveDirectionVector);
            obj.leg6_kinematics_line = plot3(lineData(1,:),lineData(2,:),lineData(3,:),'g-');   %��ɫ

            %�����Ȳ��Ĺؽڣ��ĸ���һ���ȣ����ؽ�λ�á������˵������ؽڡ����
            lineL1 = [obj.leg1.p1_W,obj.leg1.p2_W,obj.leg1.p3_W,obj.leg1.p4_W];
            lineL2 = [obj.leg2.p1_W,obj.leg2.p2_W,obj.leg2.p3_W,obj.leg2.p4_W];
            lineL3 = [obj.leg3.p1_W,obj.leg3.p2_W,obj.leg3.p3_W,obj.leg3.p4_W];
            lineL4 = [obj.leg4.p1_W,obj.leg4.p2_W,obj.leg4.p3_W,obj.leg4.p4_W];
            lineL5 = [obj.leg5.p1_W,obj.leg5.p2_W,obj.leg5.p3_W,obj.leg5.p4_W];
            lineL6 = [obj.leg6.p1_W,obj.leg6.p2_W,obj.leg6.p3_W,obj.leg6.p4_W];
            lineH = [obj.PA1_W,obj.PA2_W,obj.PA3_W,obj.PA4_W,obj.PA5_W,obj.PA6_W,obj.PA1_W];    %�����������
            
            size1=5;
            size2=2;
            obj.Pointgj1=plot3(obj.leg1.p1_W(1),obj.leg1.p1_W(2),obj.leg1.p1_W(3),'kx','LineWidth',size1);
            obj.Pointgj2=plot3(obj.leg1.p2_W(1),obj.leg1.p2_W(2),obj.leg1.p2_W(3),'kx','LineWidth',size1);
            obj.Pointgj3=plot3(obj.leg1.p3_W(1),obj.leg1.p3_W(2),obj.leg1.p3_W(3),'kx','LineWidth',size1);
            obj.Pointgj4=plot3(obj.leg1.p4_W(1),obj.leg1.p4_W(2),obj.leg1.p4_W(3),'ko','LineWidth',size2);
            
            obj.Pointgj5=plot3(obj.leg2.p1_W(1),obj.leg2.p1_W(2),obj.leg2.p1_W(3),'kx','LineWidth',size1);
            obj.Pointgj6=plot3(obj.leg2.p2_W(1),obj.leg2.p2_W(2),obj.leg2.p2_W(3),'kx','LineWidth',size1);
            obj.Pointgj7=plot3(obj.leg2.p3_W(1),obj.leg2.p3_W(2),obj.leg2.p3_W(3),'kx','LineWidth',size1);
            obj.Pointgj8=plot3(obj.leg2.p4_W(1),obj.leg2.p4_W(2),obj.leg2.p4_W(3),'k.','LineWidth',size2);
            
            obj.Pointgj9=plot3(obj.leg3.p1_W(1),obj.leg3.p1_W(2),obj.leg3.p1_W(3),'kx','LineWidth',size1);
            obj.Pointgj10=plot3(obj.leg3.p2_W(1),obj.leg3.p2_W(2),obj.leg3.p2_W(3),'kx','LineWidth',size1);
            obj.Pointgj11=plot3(obj.leg3.p3_W(1),obj.leg3.p3_W(2),obj.leg3.p3_W(3),'kx','LineWidth',size1);
            obj.Pointgj12=plot3(obj.leg3.p4_W(1),obj.leg3.p4_W(2),obj.leg3.p4_W(3),'k.','LineWidth',size2);
            
            obj.Pointgj13=plot3(obj.leg4.p1_W(1),obj.leg4.p1_W(2),obj.leg4.p1_W(3),'kx','LineWidth',size1);
            obj.Pointgj14=plot3(obj.leg4.p2_W(1),obj.leg4.p2_W(2),obj.leg4.p2_W(3),'kx','LineWidth',size1);
            obj.Pointgj15=plot3(obj.leg4.p3_W(1),obj.leg4.p3_W(2),obj.leg4.p3_W(3),'kx','LineWidth',size1);
            obj.Pointgj16=plot3(obj.leg4.p4_W(1),obj.leg4.p4_W(2),obj.leg4.p4_W(3),'k.','LineWidth',size2);
           
            obj.Pointgj17=plot3(obj.leg5.p1_W(1),obj.leg5.p1_W(2),obj.leg5.p1_W(3),'kx','LineWidth',size1);
            obj.Pointgj18=plot3(obj.leg5.p2_W(1),obj.leg5.p2_W(2),obj.leg5.p2_W(3),'kx','LineWidth',size1);
            obj.Pointgj19=plot3(obj.leg5.p3_W(1),obj.leg5.p3_W(2),obj.leg5.p3_W(3),'kx','LineWidth',size1);
            obj.Pointgj20=plot3(obj.leg5.p4_W(1),obj.leg5.p4_W(2),obj.leg5.p4_W(3),'k.','LineWidth',size2);
            
            obj.Pointgj21=plot3(obj.leg6.p1_W(1),obj.leg6.p1_W(2),obj.leg6.p1_W(3),'kx','LineWidth',size1);
            obj.Pointgj22=plot3(obj.leg6.p2_W(1),obj.leg6.p2_W(2),obj.leg6.p2_W(3),'kx','LineWidth',size1);
            obj.Pointgj23=plot3(obj.leg6.p3_W(1),obj.leg6.p3_W(2),obj.leg6.p3_W(3),'kx','LineWidth',size1);
            obj.Pointgj24=plot3(obj.leg6.p4_W(1),obj.leg6.p4_W(2),obj.leg6.p4_W(3),'k.','LineWidth',size2);
            
            %����֧�Ŷ����
            temp = obj.ContactPoint;
            temp(:,size(obj.ContactPoint,2)+1) = obj.ContactPoint(:,1); %Ϊ��ʹ����αպ�
            obj.marginArea=plot3(temp(1,:),temp(2,:),(obj.z-0.5)*ones(1,size(temp,2)),':','Color',[0.1 0.1 0.1],'LineWidth',1.5);
            
            %���������ȶ�ԣ�ȣ����ĵ�֧�Ŷ������̾��루������ǰ������
            temp = [obj.x,obj.MinMarginIntersection(1,1);obj.y,obj.MinMarginIntersection(1,2);obj.z-0.5,obj.z-0.5];
            obj.shortestMarginLine =  plot3(temp(1,:),temp(2,:),temp(3,:),'b-','LineWidth',1.5);            
            
            %�����Ȳ����ˡ��������������ϵ
            obj.leg1_line = plot3(lineL1(1,:),lineL1(2,:),lineL1(3,:),'r-','LineWidth',3);
            obj.leg2_line = plot3(lineL2(1,:),lineL2(2,:),lineL2(3,:),'r-','LineWidth',3);
            obj.leg3_line = plot3(lineL3(1,:),lineL3(2,:),lineL3(3,:),'r-','LineWidth',3);
            obj.leg4_line = plot3(lineL4(1,:),lineL4(2,:),lineL4(3,:),'r-','LineWidth',3);
            obj.leg5_line = plot3(lineL5(1,:),lineL5(2,:),lineL5(3,:),'y-','LineWidth',3);  %����ȵ�����������ƣ�ʹ�û�ɫ
            obj.leg6_line = plot3(lineL6(1,:),lineL6(2,:),lineL6(3,:),'r-','LineWidth',3);
            obj.lineHexagon = plot3(lineH(1,:),lineH(2,:),lineH(3,:),'r-','LineWidth',3);
            obj.frame_base_x_line = plot3([obj.Bxyz_W(1),obj.Oxb_W(1)],[obj.Bxyz_W(2),obj.Oxb_W(2)],[obj.Bxyz_W(3),obj.Oxb_W(3)],'r'); %���ƻ�������ϵ
            obj.frame_base_y_line = plot3([obj.Bxyz_W(1),obj.Oyb_W(1)],[obj.Bxyz_W(2),obj.Oyb_W(2)],[obj.Bxyz_W(3),obj.Oyb_W(3)],'g');
            obj.frame_base_z_line = plot3([obj.Bxyz_W(1),obj.Ozb_W(1)],[obj.Bxyz_W(2),obj.Ozb_W(2)],[obj.Bxyz_W(3),obj.Ozb_W(3)],'b');
            
            %��ǰ�������ϣ����ĵ�֧�Ŷ���ε��ȶ�ԣ��
            COG2PolygonDistance=obj.GetCOG2PolygonDistance();   
            Check=[obj.x obj.y 0]+COG2PolygonDistance*(obj.MoveDirectionVector/norm(obj.MoveDirectionVector));  %�����ȶ�ԣ���������ڻ�ͼ
            obj.COG2PolygonLine=line([obj.x Check(1) ], [obj.y Check(2)],[-0.5 -0.5]);  %��ͼ
            
            %������ݴ��ˣ���ô���������
            if obj.leg1.isDead==1
                obj.DeadPointleg1=plot3(obj.leg1.p4_W(1),obj.leg1.p4_W(2),obj.leg1.p4_W(3),'r+','LineWidth',8);
            end
            if obj.leg2.isDead==1
                obj.DeadPointleg2=plot3(obj.leg2.p4_W(1),obj.leg2.p4_W(2),obj.leg2.p4_W(3),'r+','LineWidth',8);
            end
            if obj.leg3.isDead==1
                obj.DeadPointleg3=plot3(obj.leg3.p4_W(1),obj.leg3.p4_W(2),obj.leg3.p4_W(3),'r+','LineWidth',8);
            end
            if obj.leg4.isDead==1
                obj.DeadPointleg4=plot3(obj.leg4.p4_W(1),obj.leg4.p4_W(2),obj.leg4.p4_W(3),'r+','LineWidth',8);
            end
            if obj.leg5.isDead==1
                obj.DeadPointleg5=plot3(obj.leg5.p4_W(1),obj.leg5.p4_W(2),obj.leg5.p4_W(3),'r+','LineWidth',8);
            end
            if obj.leg6.isDead==1
                obj.DeadPointleg6=plot3(obj.leg6.p4_W(1),obj.leg6.p4_W(2),obj.leg6.p4_W(3),'r+','LineWidth',8);
            end
        end
        
        %����û�б�ʹ��
        function [obj,flag] = changePos_fixLegPosition(obj,in_X,in_Y,in_Z,in_yaw,in_pitch,in_roll)%�������̶������ �ı�������̬
            %�����µĻ��嵽��������ϵ����ת����
            obj.x = in_X;
            obj.y = in_Y;
            obj.z = in_Z;
            obj.yaw = in_yaw;
            obj.pitch = in_pitch;
            obj.roll = in_roll;
            %����ת���󴫵ݵ�leg��Ա
            leg1p4 = obj.leg1.p4_W;
            leg2p4 = obj.leg2.p4_W;
            leg3p4 = obj.leg3.p4_W;
            leg4p4 = obj.leg4.p4_W;
            leg5p4 = obj.leg5.p4_W;
            leg6p4 = obj.leg6.p4_W;
            obj.leg1.T_W_B = obj.T_W_B;
            obj.leg2.T_W_B = obj.T_W_B;
            obj.leg3.T_W_B = obj.T_W_B;
            obj.leg4.T_W_B = obj.T_W_B;
            obj.leg5.T_W_B = obj.T_W_B;
            obj.leg6.T_W_B = obj.T_W_B;
            %ͨ�����˶�ѧ���¼���ؽڽǶ�
            obj.leg1 = obj.leg1.OnrLegInverseKinematicsWoldAxis(leg1p4);
            obj.leg2 = obj.leg2.OnrLegInverseKinematicsWoldAxis(leg2p4);
            obj.leg3 = obj.leg3.OnrLegInverseKinematicsWoldAxis(leg3p4);
            obj.leg4 = obj.leg4.OnrLegInverseKinematicsWoldAxis(leg4p4);
            obj.leg5 = obj.leg5.OnrLegInverseKinematicsWoldAxis(leg5p4);
            obj.leg6 = obj.leg6.OnrLegInverseKinematicsWoldAxis(leg6p4);
            
            %flagΪ1�����ɹ��� ������ⲻ�ɹ�
            flag =  (obj.leg1.isInverseSolution & obj.leg2.isInverseSolution & obj.leg3.isInverseSolution & ...
                    obj.leg4.isInverseSolution & obj.leg5.isInverseSolution & obj.leg6.isInverseSolution);
        end
        
        function Out=GetBodyPosition(obj) %��ȡ��������,������
            x1=obj.x;
            y1=obj.y;
            z1=obj.z;
            Out=[x1,y1,z1];
        end
        
        %��ǰ���޸ĺ�T_W_B����ʹ��
        function [obj,flag] = changePos_fixLegPosition_SupportState(obj,in_X,in_Y,in_Z,in_yaw,in_pitch,in_roll) %����supportstate�޸Ļ������
            %�����µĻ��嵽��������ϵ����ת����
            obj.x = in_X;
            obj.y = in_Y;
            obj.z = in_Z;
            obj.yaw = in_yaw;
            obj.pitch = in_pitch;
            obj.roll = in_roll;
            
            flag=0;
            SupportState1=obj.SupportState; %֧��״̬��6��0,1
            %����ת���󴫵ݵ�leg��Ա
            if SupportState1(1)==1
                leg1p4 = obj.leg1.p4_W;
                obj.leg1.T_W_B = obj.T_W_B;
                obj.leg1 = obj.leg1.OnrLegInverseKinematicsWoldAxis(leg1p4);            %ͨ�����˶�ѧ���¼���ؽڽǶ�
                if obj.leg1.isInverseSolution==0
%                     flag=flag;
                else
                    flag=flag+1;
                end
            end
            if SupportState1(2)==1
                leg2p4 = obj.leg2.p4_W;
                obj.leg2.T_W_B = obj.T_W_B;
                obj.leg2 = obj.leg2.OnrLegInverseKinematicsWoldAxis(leg2p4);
                if obj.leg2.isInverseSolution==0
%                     flag=flag;
                else
                    flag=flag+1;
                end
            end
            if SupportState1(3)==1
                leg3p4 = obj.leg3.p4_W;
                obj.leg3.T_W_B = obj.T_W_B;
                obj.leg3 = obj.leg3.OnrLegInverseKinematicsWoldAxis(leg3p4);                
                if obj.leg3.isInverseSolution==0
%                     flag=flag;
                else
                    flag=flag+1;
                end
            end
            if SupportState1(4)==1
                leg4p4 = obj.leg4.p4_W;
                obj.leg4.T_W_B = obj.T_W_B;
                obj.leg4 = obj.leg4.OnrLegInverseKinematicsWoldAxis(leg4p4);
                if obj.leg4.isInverseSolution==0
%                     flag=flag;
                else
                    flag=flag+1;
                end
            end
            if SupportState1(5)==1
                leg5p4 = obj.leg5.p4_W;
                obj.leg5.T_W_B = obj.T_W_B;
                obj.leg5 = obj.leg5.OnrLegInverseKinematicsWoldAxis(leg5p4);
                if obj.leg5.isInverseSolution==0
%                     flag=flag;
                else
                    flag=flag+1;
                end
            end
            if SupportState1(6)==1
                leg6p4 = obj.leg6.p4_W;
                obj.leg6.T_W_B = obj.T_W_B;
                obj.leg6 = obj.leg6.OnrLegInverseKinematicsWoldAxis(leg6p4);
                if obj.leg6.isInverseSolution==0
%                     flag=flag;
                else
                    flag=flag+1;
                end
            end
            %flagΪ1�����ɹ��� ������ⲻ�ɹ�
            if flag==sum(SupportState1)
                flag=1;
            else
                flag=0;
            end

            
%             flag =  (obj.leg1.isInverseSolution & obj.leg2.isInverseSolution & obj.leg3.isInverseSolution & ...
%                     obj.leg4.isInverseSolution & obj.leg5.isInverseSolution & obj.leg6.isInverseSolution);
         end
        
        function [obj,flag] = changePos_135fixLegPosition(obj,in_X,in_Y,in_Z,in_yaw,in_pitch,in_roll) %1 3 5 ����֧�ţ���������ƶ������ǵ��¹ؽڽ�
            %�����µĻ��嵽��������ϵ����ת����
            obj.x = in_X;
            obj.y = in_Y;
            obj.z = in_Z;
            obj.yaw = in_yaw;
            obj.pitch = in_pitch;
            obj.roll = in_roll;

            %����ת���󴫵ݵ�leg��Ա
            leg1p4 = obj.leg1.p4_W;
            leg3p4 = obj.leg3.p4_W;
            leg5p4 = obj.leg5.p4_W;

            obj.leg1.T_W_B = obj.T_W_B;
             obj.leg2.T_W_B = obj.T_W_B;
            obj.leg3.T_W_B = obj.T_W_B;
             obj.leg4.T_W_B = obj.T_W_B;
            obj.leg5.T_W_B = obj.T_W_B;
             obj.leg6.T_W_B = obj.T_W_B;
             
            %ͨ�����˶�ѧ���¼���ؽڽǶ�
            obj.leg1 = obj.leg1.OnrLegInverseKinematicsWoldAxis(leg1p4);
            obj.leg3 = obj.leg3.OnrLegInverseKinematicsWoldAxis(leg3p4);
            obj.leg5 = obj.leg5.OnrLegInverseKinematicsWoldAxis(leg5p4);
            
            %flagΪ1�����ɹ��� ������ⲻ�ɹ�
            flag =  (obj.leg1.isInverseSolution  & obj.leg3.isInverseSolution  ...
                    & obj.leg5.isInverseSolution );
        end

        function [obj,flag] = changePos_246fixLegPosition(obj,in_X,in_Y,in_Z,in_yaw,in_pitch,in_roll) %2 4 6 ����֧�ţ���������ƶ������ǵ��¹ؽڽ�
            %�����µĻ��嵽��������ϵ����ת����
            obj.x = in_X;
            obj.y = in_Y;
            obj.z = in_Z;
            obj.yaw = in_yaw;
            obj.pitch = in_pitch;
            obj.roll = in_roll;
%             obj.T_W_B = [angle2dcm(in_yaw,in_pitch,in_roll,'ZYX'),[in_X;in_Y;in_Z];[0,0,0,1];];
            %����ת���󴫵ݵ�leg��Ա

            leg2p4 = obj.leg2.p4_W;
            leg4p4 = obj.leg4.p4_W;
            leg6p4 = obj.leg6.p4_W;
            
            obj.leg1.T_W_B = obj.T_W_B;
            obj.leg2.T_W_B = obj.T_W_B;
            obj.leg3.T_W_B = obj.T_W_B;
            obj.leg4.T_W_B = obj.T_W_B;
            obj.leg5.T_W_B = obj.T_W_B;
            obj.leg6.T_W_B = obj.T_W_B;
            
            %ͨ�����˶�ѧ���¼���ؽڽǶ�
            obj.leg2 = obj.leg2.OnrLegInverseKinematicsWoldAxis(leg2p4);
            obj.leg4 = obj.leg4.OnrLegInverseKinematicsWoldAxis(leg4p4);
            obj.leg6 = obj.leg6.OnrLegInverseKinematicsWoldAxis(leg6p4);
            
            %flagΪ1�����ɹ��� ������ⲻ�ɹ�
            flag =  (  obj.leg2.isInverseSolution  & ...
                    obj.leg4.isInverseSolution & obj.leg6.isInverseSolution);
        end
        
        function [obj,TerrainMapConsiderFootMat1]=PutLegsOnUseableTerrainAtStartTime(obj,TerrainMapConsiderFootMat) %��һ��ʼ�Ͱѻ����˷ŵ���ŵ��ϡ�
            obj.leg1.Initp4BPoint=obj.leg1.p4_B;
            obj.leg2.Initp4BPoint=obj.leg2.p4_B;
            obj.leg3.Initp4BPoint=obj.leg3.p4_B;
            obj.leg4.Initp4BPoint=obj.leg4.p4_B;
            obj.leg5.Initp4BPoint=obj.leg5.p4_B;
            obj.leg6.Initp4BPoint=obj.leg6.p4_B;
            
            temp=obj.ChangeSupport([0 0 0 0 0 0]);  %�������ȶ��ǰڶ��ȣ�Ѱ����һ��֧��װ��
            temp=temp.GetUseableFootTerrain(TerrainMapConsiderFootMat,[1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1]);
            Numleg1=size(temp.leg1.UsablePoint,1);  %�õ�ÿ���ȿ��õ��������Ŀ��������������������洢��ÿһ����һ�������
            Numleg2=size(temp.leg2.UsablePoint,1);
            Numleg3=size(temp.leg3.UsablePoint,1);
            Numleg4=size(temp.leg4.UsablePoint,1);
            Numleg5=size(temp.leg5.UsablePoint,1);
            Numleg6=size(temp.leg6.UsablePoint,1);
            if Numleg1~=0||Numleg2~=0||Numleg3~=0||Numleg4~=0||Numleg5~=0||Numleg6~=0
                disp('��ʼ��������ʧ��,û���㹻�Ŀ�������㣬��Ϊ�������');
                TerrainMapConsiderFootMat1=[TerrainMapConsiderFootMat;obj.leg1.p4_W';obj.leg2.p4_W';obj.leg3.p4_W';obj.leg4.p4_W';obj.leg5.p4_W';obj.leg6.p4_W'];
                return;
            end
            
            %����ӱ�ѡ�������ѡ��һ������㣬Ȼ������ƶ���ȥ
            Numleg1i=randi(Numleg1,1,1); %Ϊ�˱���һ���ԷŲ�����ȷ��λ�á�
            Numleg2i=randi(Numleg2,1,1);
            Numleg3i=randi(Numleg3,1,1);
            Numleg4i=randi(Numleg4,1,1);
            Numleg5i=randi(Numleg5,1,1);
            Numleg6i=randi(Numleg6,1,1);
            

            obj.leg1=obj.leg1.leg_move(temp.leg1.UsablePoint(Numleg1i,1),temp.leg1.UsablePoint(Numleg1i,2),temp.leg1.UsablePoint(Numleg1i,3));
            obj.leg2=obj.leg2.leg_move(temp.leg2.UsablePoint(Numleg2i,1),temp.leg2.UsablePoint(Numleg2i,2),temp.leg2.UsablePoint(Numleg2i,3));
            obj.leg3=obj.leg3.leg_move(temp.leg3.UsablePoint(Numleg3i,1),temp.leg3.UsablePoint(Numleg3i,2),temp.leg3.UsablePoint(Numleg3i,3));
            obj.leg4=obj.leg4.leg_move(temp.leg4.UsablePoint(Numleg4i,1),temp.leg4.UsablePoint(Numleg4i,2),temp.leg4.UsablePoint(Numleg4i,3));
            obj.leg5=obj.leg5.leg_move(temp.leg5.UsablePoint(Numleg5i,1),temp.leg5.UsablePoint(Numleg5i,2),temp.leg5.UsablePoint(Numleg5i,3));
            obj.leg6=obj.leg6.leg_move(temp.leg6.UsablePoint(Numleg6i,1),temp.leg6.UsablePoint(Numleg6i,2),temp.leg6.UsablePoint(Numleg6i,3));
        end

        
%%
        %���ɾ�̬�ȶ���֧��״̬��������һ�ξͲ����ٱ�
        function SupportStateListNew=GenerateSupportStateListNew(obj)
            state=[0 1];
            SupportStateListNew=[];
            for a=1:2
                for b=1:2
                    for c=1:2
                        for d=1:2
                            for e=1:2
                                for f=1:2
                                    if sum([state(a),state(b),state(c),state(d),state(e),state(f)])>=3
                                        SupportStateListNew=[SupportStateListNew;state(a),state(b),state(c),state(d),state(e),state(f)];
                                    end
                                end
                            end
                        end
                    end
                end
            end
            SupportStateListNew=SupportStateListNew(2:size(SupportStateListNew,1),:);
        end
        
        %��֧�����ȶ�ѧԣ�ȼ������ȶ�ԣ�Ȼ�ȡ�����������
        function CurrentMaxStrideLength = GetCurrentMaxStrideLength(obj) 
            CurrentSupportState=obj.SupportState;
            Xishu=1;
            if CurrentSupportState(1)==1
                margin1 = Xishu*obj.leg1_kinematicsMargin;
            else
                margin1=10000;
            end
            if CurrentSupportState(2)==1
                margin2 = Xishu*obj.leg2_kinematicsMargin;
            else
                margin2=10000;
            end
            if CurrentSupportState(3)==1
                margin3 = Xishu*obj.leg3_kinematicsMargin;
            else
                margin3=10000;
            end
            if CurrentSupportState(4)==1
                margin4 = Xishu*obj.leg4_kinematicsMargin;
            else
                margin4=10000;
            end
            if CurrentSupportState(5)==1
                margin5 = Xishu*obj.leg5_kinematicsMargin;
            else
                margin5=10000;
            end
            if CurrentSupportState(6)==1
                margin6 = Xishu*obj.leg6_kinematicsMargin;
            else
                margin6=10000;
            end
            %�ȶ�ԣ����СֵΪ0.05
            COG2PolygonDistance=obj.GetCOG2PolygonDistance() - 0.1;
            CurrentMaxStrideLength = min([margin1 margin2 margin3 margin4 margin5 margin6 COG2PolygonDistance])/2;
            if CurrentMaxStrideLength==10000
                CurrentMaxStrideLength=0;
                error("��������,֧��״̬��Ϊ0");
            end
            if CurrentMaxStrideLength==0
                CurrentMaxStrideLength=0;
                error("��������");
            end
        end
        
        %��ȡǰ���������ĵ�֧�Ŷ���εľ���
        function COG2PolygonDistance=GetCOG2PolygonDistance(obj) 
            X1 = obj.ContactPoint(1,:); %��ȡ֧��λ�õ�x,y����
            Y1 = obj.ContactPoint(2,:);
            flag=0;

            if( inpolygon(obj.x,obj.y,X1,Y1) == 1 ) %�ж������Ƿ���֧�Ŷ�����ڲ�
                for i=1:size(obj.ContactPoint,2)
                    XA=obj.ContactPoint(1,i);
                    YA=obj.ContactPoint(2,i);
                    ii=i+1;
                    if ii>size(obj.ContactPoint,2)
                        ii=1;
                    end
                    XB=obj.ContactPoint(1,ii);
                    YB=obj.ContactPoint(2,ii);
                    MoveDirection=obj.MoveDirectionVector/norm(obj.MoveDirectionVector); %���ߵķ�������,��base����ϵ�µ�
                    MoveDirection=obj.T_W_B(1:3,1:3)*MoveDirection'; %ת������������ϵ
                    D=abs((YB-YA)*obj.x-(XB-XA)*obj.y+(XB-XA)*YA-XA*(YB-YA))/sqrt((YB-YA)^2+(XB-XA)^2); %�㵽ֱ�ߵĴ�ֱ����
                    VectorD=[0 -1 0;1 0 0;0 0 1]*[XB-XA,YB-YA,0]'; %��ֱ���������������ת90��
                    VectorD=VectorD/norm(VectorD);
                    Costheta=dot(MoveDirection,VectorD);
                    Dist=abs(D/Costheta); %���н��������ֱ�ߵĽ���ľ���
                    Check=[obj.x obj.y 0]+Dist*(obj.MoveDirectionVector/norm(obj.MoveDirectionVector));
                    
                    Check(1)=roundn(Check(1),-4);  %������λС��
                    Check(2)=roundn(Check(2),-4);  %������λС��
                    XA=roundn(XA,-4);
                    XB=roundn(XB,-4);
                    YA=roundn(YA,-4);
                    YB=roundn(YB,-4);
                    shuzhi11=abs(dot (([Check(1),Check(2)]-[XA,YA])/norm([Check(1),Check(2)]-[XA,YA]),([Check(1),Check(2)]-[XB,YB])/norm([Check(1),Check(2)]-[XB,YB])));
                    shuzhi11=roundn(shuzhi11,-4);
                    if norm([Check(1),Check(2)]-[XA,YA])==0||norm([Check(1),Check(2)]-[XB,YB])==0
                        COG2PolygonDistance=Dist;
                        flag=1;
                    else
                        if shuzhi11==1
                            COG2PolygonDistance=Dist;
                            flag=1;
                        end
                    end
                end
            else
                COG2PolygonDistance=0;  %����û�б�������Ϊ�ڻ�ȡ���������ǰ������ĺ����ж��ƶ�����Ϊ0����������ˡ�
            end
            if flag==0
                COG2PolygonDistance=0;
            end
        end
        
        %�õ���ǰ֧��״̬�µ���������֮���ƽ���ȶ�ԣ��
        function CurrentSupportStateMargin=GetSupportStateMargin(obj,CurrentSupportState) 
            CurrentSupportStateMargin=0;
            Count=0;
            if CurrentSupportState(1)==1
               CurrentSupportStateMargin=CurrentSupportStateMargin+obj.leg1_kinematicsMargin;
               Count=Count+1;
            end
            if CurrentSupportState(2)==1
               CurrentSupportStateMargin=CurrentSupportStateMargin+obj.leg2_kinematicsMargin;
               Count=Count+1;
            end
            if CurrentSupportState(3)==1
               CurrentSupportStateMargin=CurrentSupportStateMargin+obj.leg3_kinematicsMargin;
               Count=Count+1;
            end
            if CurrentSupportState(4)==1
               CurrentSupportStateMargin=CurrentSupportStateMargin+obj.leg4_kinematicsMargin;
               Count=Count+1;
            end
            if CurrentSupportState(5)==1
               CurrentSupportStateMargin=CurrentSupportStateMargin+obj.leg5_kinematicsMargin;
               Count=Count+1;
            end
            if CurrentSupportState(6)==1
               CurrentSupportStateMargin=CurrentSupportStateMargin+obj.leg6_kinematicsMargin;
               Count=Count+1;
            end
            CurrentSupportStateMargin=CurrentSupportStateMargin/Count;
        end
        
        %��������������С�������ȶ�ԣ��
        function minus=GetSupportStateKinematicMinus(obj,CurrentSupportState) 
            minus = obj.leg1_kinematicsMargin;
            if(minus > obj.leg2_kinematicsMargin)
                minus = obj.leg2_kinematicsMargin;
            end
            if(minus > obj.leg3_kinematicsMargin)
                minus = obj.leg3_kinematicsMargin;
            end
            if(minus > obj.leg4_kinematicsMargin)
                minus = obj.leg4_kinematicsMargin;
            end
            if(minus > obj.leg5_kinematicsMargin)
                minus = obj.leg5_kinematicsMargin;
            end
            if(minus > obj.leg6_kinematicsMargin)
                minus = obj.leg6_kinematicsMargin;
            end
        end
%%
        %�ı䵱ǰ��֧��״̬
        function obj=ChangeSupport(obj,state)
            obj.leg1.isSupport=state(1);
            obj.leg2.isSupport=state(2);
            obj.leg3.isSupport=state(3);
            obj.leg4.isSupport=state(4);
            obj.leg5.isSupport=state(5);
            obj.leg6.isSupport=state(6);
            obj.SupportState=state;
        end
        
        %�ܹ��ж������ڶ���
        function swingNum=GetSwingLegNum(obj,SupportState)
            swingNum=0; 
            if SupportState(1)==0
                    swingNum = swingNum + 1; 
            end
            if SupportState(2)==0
                    swingNum = swingNum + 1;  
            end
            if SupportState(3)==0
                    swingNum = swingNum + 1; 
            end
            if SupportState(4)==0
                    swingNum = swingNum + 1; 
            end
            if SupportState(5)==0
                    swingNum = swingNum + 1; 
            end
            if SupportState(6)==0
                    swingNum = swingNum + 1; 
            end
        end
        
        %�ж�֧��״̬�ͷ����Ƿ���Ǣ��%�ж�����֧�ŵ����Ƿ��Ƿ��ȣ�û��������������ԭ��
        function CheckFlagMatchDeadLeg=CheckSupportStateMatchDeadLeg(obj,SupportState) 
            CheckFlagMatchDeadLeg=1; %����
            if SupportState(1)==1
                if obj.leg1.isDead==1
                    CheckFlagMatchDeadLeg=0;%������
                end
            end
            if SupportState(2)==1
                if obj.leg2.isDead==1
                    CheckFlagMatchDeadLeg=0;
                end
            end
            if SupportState(3)==1
                if obj.leg3.isDead==1
                    CheckFlagMatchDeadLeg=0;
                end
            end
            if SupportState(4)==1
                if obj.leg4.isDead==1
                    CheckFlagMatchDeadLeg=0;
                end
            end
            if SupportState(5)==1
                if obj.leg5.isDead==1
                    CheckFlagMatchDeadLeg=0;
                end
            end
            if SupportState(6)==1
                if obj.leg6.isDead==1
                    CheckFlagMatchDeadLeg=0;
                end
            end
        end
        
        %Ч������ ���µĵ�ǰ֧��״̬������һ֧��״̬ ���ݻ�õ��ȶ�ԣ�����������
        function NextSupportState=GetNextSupportStateFromSupportStateList(obj,CurrentSupportState) 
            flag=0;
            NextSupportStateMargin = [];
            NextSupportStateKinematicMinus = [];
            
            for i=1:size(obj.SupportStateList,1)    
                NextSupportStateMargin(i) =obj.GetSupportStateMargin(obj.SupportStateList(i,:));
                NextSupportStateKinematicMinus(i) = obj.GetSupportStateKinematicMinus(obj.SupportStateList(i,:));
                %ȥ����ǰ״̬����ȫ֧��״̬����дһ��ѽ
                if( obj.SupportStateList(i,:) == [1,1,1,1,1,1])%CurrentMaxStrideLength
                    NextSupportStateMargin(i) = -1 ;
                end
            end
            
            for jjj = 1:size(obj.SupportStateList,1)
                    [~,i] = max(NextSupportStateMargin);
                    
                    count = 1;
                    %���
                    SubContactPoint = [];
                    if obj.SupportStateList(i,1) == 1
                        SubContactPoint(count,:) = obj.leg1.p4_W';
                        count = count + 1;
                    end
                    if obj.SupportStateList(i,2) == 1
                        SubContactPoint(count,:) = obj.leg2.p4_W';
                        count = count + 1;
                    end
                    if obj.SupportStateList(i,3) == 1
                        SubContactPoint(count,:) = obj.leg3.p4_W';
                        count = count + 1;
                    end
                    if obj.SupportStateList(i,4) == 1
                        SubContactPoint(count,:) = obj.leg4.p4_W';
                        count = count + 1;
                    end
                    if obj.SupportStateList(i,5) == 1
                        SubContactPoint(count,:) = obj.leg5.p4_W';
                        count = count + 1;
                    end
                    if obj.SupportStateList(i,6) == 1
                        SubContactPoint(count,:) = obj.leg6.p4_W';
                    end
                    SubContactPoint = SubContactPoint';
                    X1 = SubContactPoint(1,:);
                    Y1 = SubContactPoint(2,:);
                    CheckFlagMatchDeadLeg=obj.CheckSupportStateMatchDeadLeg(obj.SupportStateList(i,:));
                    swingNum = obj.GetSwingLegNum(obj.SupportStateList(i,:));
                    deadNum = obj.leg1.isDead + obj.leg2.isDead + obj.leg3.isDead + obj.leg4.isDead + obj.leg5.isDead + obj.leg6.isDead;
                    
                    if( inpolygon(obj.x,obj.y,X1,Y1) == 1 && CheckFlagMatchDeadLeg==1 && NextSupportStateMargin(i) > 0) %ѡ��xy��֧�Ŷ�����ڵ���� ���Ȳ�������֧��
                        NextSupportState=obj.SupportStateList(i,:);
                        
                        %����ڶ���ȫ�Ƿ��ȣ�������ȶ�ԣ�ȵ���0.01 --->�����岻�ܼ���ǰ��������ô������״̬
                        if deadNum == swingNum  && NextSupportStateKinematicMinus(i) < 0.05 
                            NextSupportStateMargin(i) = -1;
                            continue;
                        end
                        
                        
                        %��֤��ʱ�����ȶ�ԣ�ȴ���0.1
                        [MinMarginIntersection1,~] = GetMinInpolygonIntersection([X1;Y1],[obj.x,obj.y]);
                        BodyMargin1 = norm(MinMarginIntersection1 - [obj.x,obj.y]);
                        if BodyMargin1 < 0.1 
                            NextSupportStateMargin(i) = -1;
                            continue;
                        end
                        
                        
                        
                        flag=1; %֤���ڱ����ҵ���һ��state Ҳ�п�����ȫ�Ҳ���state
                        
                        break;
                    else
                        NextSupportStateMargin(i) = -1;
                    end
            end
                    if flag==0
                           error('can not find next support state');
                    end
 
        end
        
        %����ֻ����ŵ����ṩ֧��
        function CurrentSupportState=GetCurrentSupportState(obj,TerrainMapConsiderFootMat) 
            Yuzhi=0.0001;
            CurrentSupportState=[0 0 0 0 0 0];
            
            %�������п������������ж�
            for i=1:size(TerrainMapConsiderFootMat,1)
                if TerrainMapConsiderFootMat(i,1)>obj.x-2&&TerrainMapConsiderFootMat(i,1)<obj.x+2&&TerrainMapConsiderFootMat(i,2)>obj.y-2&&TerrainMapConsiderFootMat(i,2)<obj.y+2
                    if norm(obj.leg1.p4_W-TerrainMapConsiderFootMat(i,:)')<=Yuzhi
                        CurrentSupportState(1,1)=1;
                    end
                    if norm(obj.leg2.p4_W-TerrainMapConsiderFootMat(i,:)')<=Yuzhi
                        CurrentSupportState(1,2)=1;   
                    end
                    if norm(obj.leg3.p4_W-TerrainMapConsiderFootMat(i,:)')<=Yuzhi
                        CurrentSupportState(1,3)=1;
                    end
                    if norm(obj.leg4.p4_W-TerrainMapConsiderFootMat(i,:)')<=Yuzhi
                        CurrentSupportState(1,4)=1;
                    end
                    if norm(obj.leg5.p4_W-TerrainMapConsiderFootMat(i,:)')<=Yuzhi
                        CurrentSupportState(1,5)=1;
                    end
                    if norm(obj.leg6.p4_W-TerrainMapConsiderFootMat(i,:)')<=Yuzhi
                        CurrentSupportState(1,6)=1;
                    end
                end
            end
            if sum(CurrentSupportState)==0
                disp('���нŶ�����');
            end
            obj.SupportState=CurrentSupportState;   %���ڲ������ã�����obj����û�з����ı�
        end
        
        %�ӵ�ǰ֧��״̬������һ֧��״̬ ���ݻ�õ��ȶ�ԣ�����������
        function NextSupportState=GetNextSupportState(obj,CurrentSupportState) 
            flag=0;

            NextSupportState=obj.SupportStateList(1,:);
            NextSupportStateMargin=obj.GetSupportStateMargin(obj.SupportStateList(1,:));

            for i=1:size(obj.SupportStateList,1)-1
                NextSupportStateMargin=obj.GetSupportStateMargin(obj.SupportStateList(i,:));
                if NextSupportStateMargin-CurrentSupportStateMargin>=Minus
                    count = 1;
                    %���
                    SubContactPoint = [];
                    if obj.SupportStateList(i,1) == 1
                        SubContactPoint(count,:) = obj.leg1.p4_W';
                        count = count + 1;
                    end
                    if obj.SupportStateList(i,2) == 1
                        SubContactPoint(count,:) = obj.leg2.p4_W';
                        count = count + 1;
                    end
                    if obj.SupportStateList(i,3) == 1
                        SubContactPoint(count,:) = obj.leg3.p4_W';
                        count = count + 1;
                    end
                    if obj.SupportStateList(i,4) == 1
                        SubContactPoint(count,:) = obj.leg4.p4_W';
                        count = count + 1;
                    end
                    if obj.SupportStateList(i,5) == 1
                        SubContactPoint(count,:) = obj.leg5.p4_W';
                        count = count + 1;
                    end
                    if obj.SupportStateList(i,6) == 1
                        SubContactPoint(count,:) = obj.leg6.p4_W';
                    end
                    SubContactPoint = SubContactPoint';
                    X1 = SubContactPoint(1,:);
                    Y1 = SubContactPoint(2,:);
                    CheckFlagMatchDeadLeg=CheckSupportStateMatchDeadLeg(obj,obj.SupportStateList(i,:));
                    if( inpolygon(obj.x,obj.y,X1,Y1) == 1 && CheckFlagMatchDeadLeg==1 ) %ѡ��xy��֧�Ŷ�����ڵ���� ���Ȳ�������֧��
                        NextSupportState=obj.SupportStateList(i,:);
                        Minus=NextSupportStateMargin-CurrentSupportStateMargin;
                        flag=1; %֤���ڱ����ҵ���һ��state Ҳ�п�����ȫ�Ҳ���state
                    end
                end
            end
            if flag==0
                NextSupportState=CurrentSupportState;
                aa=1;%��ȫ�Ҳ���state ��ζ�����е���Ϊ���õ�state  �����ڶ������ ���ǵ�ǰ�Ŀ϶���
            end
        end
        
        %������������߹滮�����Ĺ켣 t3 Ϊ����ʱ�� CurrentTimeΪ��ǰʱ��
        function obj=BodyActionTrajectory5(obj,StartPoint,EndPoint,t3,CurrentTime)
            X=TrajectoryPlanning5(StartPoint,EndPoint,t3,CurrentTime);
            [temp,flag]=obj.changePos_fixLegPosition_SupportState(X(1),X(2),X(3),0,0,0);
            if flag==1
%                 BodyPlanningPoint
                obj=temp;
                X=X';
%                 TrajectoryRecord=X;
%                 obj.BodyTrajectoryRecordH=[obj.BodyTrajectoryRecordH TrajectoryRecord];
%                 if (obj.BodyTrajectoryRecordH(3,size(obj.BodyTrajectoryRecordH,2))<-0.5)
%                 end
            else
                disp('�����ƶ�ʧ��');
            end
        end
        
        %���»����˵�ǰ֧��״̬
        function obj=UpdateLogicSupportState(obj)
            obj.SupportState = [obj.leg1.isSupport obj.leg2.isSupport obj.leg3.isSupport obj.leg4.isSupport obj.leg5.isSupport obj.leg6.isSupport];
        end
        
        %ģ������Ե�����Ӿ�̽�⣬�����õ���̽�ⷶΧ�ڵ�������Ѱ��base����֮�ڵĽڵ�
        function TerrainMapVisionSensed=SenseTerrainMapByVision(obj,TerrainMapAll)
            SensorRange=obj.VisionSensorRange;
            TerrainMapVisionSensed=[];
            for i=1:size(TerrainMapAll,1)
                if TerrainMapAll(i,1)>obj.x-SensorRange&&TerrainMapAll(i,1)<obj.x+SensorRange&&TerrainMapAll(i,2)>obj.y-SensorRange&&TerrainMapAll(i,2)<obj.y+SensorRange && TerrainMapAll(i,3)>obj.z-SensorRange&&TerrainMapAll(i,3)<obj.z+SensorRange  %ֻ����������Χ�ڵĵ��� һ����Ϊ�˼��ټ����� ��һ����Ϊ���кϴ�����̽�ⷶΧ���޵�ʵ�� 
                    TerrainMapVisionSensed=[TerrainMapVisionSensed;TerrainMapAll(i,:)];
                end
            end
            if isempty(TerrainMapVisionSensed)==1
                disp('��ǰ������Χû�п��õ������!');
            end
        end
        
        %�����õ���ŵ� TransitMatrixΪ�����滮����������ڵ���ת����
        function obj=GetUseableFootTerrain(obj,TerrainMap,TransitMatrix) 
            if obj.leg1.isSupport==0
                T_W_BFakeleg1=obj.leg1.T_W_B * TransitMatrix; %��ʾ��T W B�����������ƶ������ƶ�֮���TWB
            end
            if obj.leg2.isSupport==0
                T_W_BFakeleg2=obj.leg2.T_W_B * TransitMatrix; %��ʾ��T W B�����������ƶ������ƶ�֮���TWB
            end
            if obj.leg3.isSupport==0
                T_W_BFakeleg3=obj.leg3.T_W_B * TransitMatrix; %��ʾ��T W B�����������ƶ������ƶ�֮���TWB
            end
            if obj.leg4.isSupport==0
                T_W_BFakeleg4=obj.leg4.T_W_B * TransitMatrix; %��ʾ��T W B�����������ƶ������ƶ�֮���TWB
            end
            if obj.leg5.isSupport==0
                T_W_BFakeleg5=obj.leg5.T_W_B * TransitMatrix; %��ʾ��T W B�����������ƶ������ƶ�֮���TWB
            end
            if obj.leg6.isSupport==0
                T_W_BFakeleg6=obj.leg6.T_W_B * TransitMatrix; %��ʾ��T W B�����������ƶ������ƶ�֮���TWB
            end
            Leg1UseablePoint=[];
            Leg2UseablePoint=[];
            Leg3UseablePoint=[];
            Leg4UseablePoint=[];
            Leg5UseablePoint=[];
            Leg6UseablePoint=[];
            SensorRange=obj.VisionSensorRange;
            
            %��յ�ͼ��ɫΪ������ɫ
            obj.terrain=obj.terrain.setColorAsBackground();
            
            %�жϻ����ƶ���������Ƿ��������ڣ���ÿ������ӿ�������ĵ�
            for i=1:size(TerrainMap,1)
                %ֻ����������Χ�ڵĵ��� һ����Ϊ�˼��ټ����� ��һ����Ϊ���кϴ�����̽�ⷶΧ���޵�ʵ��
                if TerrainMap(i,1)>obj.x-SensorRange&&TerrainMap(i,1)<obj.x+SensorRange&&TerrainMap(i,2)>obj.y-SensorRange&&TerrainMap(i,2)<obj.y+SensorRange && TerrainMap(i,3)>obj.z-SensorRange&&TerrainMap(i,3)<obj.z+SensorRange
                    plot3(TerrainMap(i,1)',TerrainMap(i,2)',TerrainMap(i,3)','r.','MarkerSize',5);  %�����еĿ�������ɸô������
                    
                    if obj.leg1.isSupport==0
                       
                        CheckFlagleg1 = obj.leg1.CheckWorldPointInLegSpaceFakeT_W_B(TerrainMap(i,1:3)',T_W_BFakeleg1);
                        CheckFlag1leg1=obj.leg1.CheckWorldPointInShanxinFakeT_W_B(TerrainMap(i,1:3)',T_W_BFakeleg1);
                        if CheckFlagleg1==1&&CheckFlag1leg1==1                           
                            obj.terrain=obj.terrain.changeCellColor(TerrainMap(i,1)',TerrainMap(i,2)','red2');
                            Leg1UseablePoint=[Leg1UseablePoint;TerrainMap(i,1:3)];
                        end
                    end
                    if obj.leg2.isSupport==0
                        CheckFlagleg2 = obj.leg2.CheckWorldPointInLegSpaceFakeT_W_B(TerrainMap(i,1:3)',T_W_BFakeleg2);
                        CheckFlag1leg2=obj.leg2.CheckWorldPointInShanxinFakeT_W_B(TerrainMap(i,1:3)',T_W_BFakeleg2);
                        if CheckFlagleg2==1&&CheckFlag1leg2==1
                            obj.terrain=obj.terrain.changeCellColor(TerrainMap(i,1)',TerrainMap(i,2)','green');
                            Leg2UseablePoint=[Leg2UseablePoint;TerrainMap(i,1:3)];
                        end
                    end
                    if obj.leg3.isSupport==0
                        CheckFlagleg3 = obj.leg3.CheckWorldPointInLegSpaceFakeT_W_B(TerrainMap(i,1:3)',T_W_BFakeleg3);
                        CheckFlag1leg3=obj.leg3.CheckWorldPointInShanxinFakeT_W_B(TerrainMap(i,1:3)',T_W_BFakeleg3);
                        if CheckFlagleg3==1&&CheckFlag1leg3==1
                            obj.terrain=obj.terrain.changeCellColor(TerrainMap(i,1)',TerrainMap(i,2)','blue');
                            Leg3UseablePoint=[Leg3UseablePoint;TerrainMap(i,1:3)];
                        end
                    end
                    if obj.leg4.isSupport==0
                        CheckFlagleg4 = obj.leg4.CheckWorldPointInLegSpaceFakeT_W_B(TerrainMap(i,1:3)',T_W_BFakeleg4);
                        CheckFlag1leg4=obj.leg4.CheckWorldPointInShanxinFakeT_W_B(TerrainMap(i,1:3)',T_W_BFakeleg4);
                        if CheckFlagleg4==1&&CheckFlag1leg4==1
                            obj.terrain=obj.terrain.changeCellColor(TerrainMap(i,1)',TerrainMap(i,2)','YellowDirt');
                            Leg4UseablePoint=[Leg4UseablePoint;TerrainMap(i,1:3)];
                        end
                    end
                    if obj.leg5.isSupport==0
                        CheckFlagleg5 = obj.leg5.CheckWorldPointInLegSpaceFakeT_W_B(TerrainMap(i,1:3)',T_W_BFakeleg5);
                        CheckFlag1leg5=obj.leg5.CheckWorldPointInShanxinFakeT_W_B(TerrainMap(i,1:3)',T_W_BFakeleg5);
                        if CheckFlagleg5==1&&CheckFlag1leg5==1
                            obj.terrain=obj.terrain.changeCellColor(TerrainMap(i,1)',TerrainMap(i,2)','Yellow');
                            Leg5UseablePoint=[Leg5UseablePoint;TerrainMap(i,1:3)];
                        end
                    end
                    if obj.leg6.isSupport==0
                        CheckFlagleg6 = obj.leg6.CheckWorldPointInLegSpaceFakeT_W_B(TerrainMap(i,1:3)',T_W_BFakeleg6);
                        CheckFlag1leg6=obj.leg6.CheckWorldPointInShanxinFakeT_W_B(TerrainMap(i,1:3)',T_W_BFakeleg6);
                        if CheckFlagleg6==1&&CheckFlag1leg6==1
                            obj.terrain=obj.terrain.changeCellColor(TerrainMap(i,1)',TerrainMap(i,2)','Purple');
                            Leg6UseablePoint=[Leg6UseablePoint;TerrainMap(i,1:3)];
                        end
                    end
                end                
            end
            
            %Ϊÿ���ȿ�������ĵ��ͼ
            if obj.leg1.isSupport==0
                obj.leg1.UsablePoint=Leg1UseablePoint;
                if isempty(Leg1UseablePoint) %����ǿյ� ˵���ȷ��� û�������
                    obj.leg1.isDead=1;
                     disp('leg1û�п������');
                else
                    obj.leg1.isDead=0;
                end
            end
            if obj.leg2.isSupport==0
                obj.leg2.UsablePoint=Leg2UseablePoint;
                if isempty(Leg2UseablePoint) %����ǿյ� ˵���ȷ��� û�������
                    obj.leg2.isDead=1;
                     disp('leg2û�п������');
                else
                    obj.leg2.isDead=0;
                end
            end
            if obj.leg3.isSupport==0
                obj.leg3.UsablePoint=Leg3UseablePoint;
                if isempty(Leg3UseablePoint) %����ǿյ� ˵���ȷ��� û�������
                    obj.leg3.isDead=1;
                     disp('leg3û�п������');
                else
                    obj.leg3.isDead=0;
                end
            end
            if obj.leg4.isSupport==0
                obj.leg4.UsablePoint=Leg4UseablePoint;
                if isempty(Leg4UseablePoint) %����ǿյ� ˵���ȷ��� û�������
                    obj.leg4.isDead=1;
                     disp('leg4û�п������');
                else
                    obj.leg4.isDead=0;
                end
            end
            if obj.leg5.isSupport==0
                obj.leg5.UsablePoint=Leg5UseablePoint;
                if isempty(Leg5UseablePoint) %����ǿյ� ˵���ȷ��� û�������
                    obj.leg5.isDead=1;
                     disp('leg5û�п������');
                else
                    obj.leg5.isDead=0;
                end
            end
            if obj.leg6.isSupport==0
                obj.leg6.UsablePoint=Leg6UseablePoint;
                if isempty(Leg6UseablePoint) %����ǿյ� ˵���ȷ��� û�������
                    obj.leg6.isDead=1;
                     disp('leg6û�п������');
                else
                    obj.leg6.isDead=0;
                end
            end
        end
        

    end

    methods  %dependent get���� 
        function T_W_B = get.T_W_B(obj)
            T_W_B = [angle2dcm(obj.yaw,obj.pitch,obj.roll,'ZYX'),[obj.x;obj.y;obj.z];[0,0,0,1];];
        end
        
        function PA1_W = get.PA1_W(obj)
            PA1_W = obj.T_W_B * [obj.PA1_B;1];
            PA1_W = PA1_W(1:3,1);
        end
        function PA2_W = get.PA2_W(obj)
            PA2_W = obj.T_W_B * [obj.PA2_B;1];
            PA2_W = PA2_W(1:3,1);
        end
        function PA3_W = get.PA3_W(obj)
            PA3_W = obj.T_W_B * [obj.PA3_B;1];
            PA3_W = PA3_W(1:3,1);
        end
        function PA4_W = get.PA4_W(obj)
            PA4_W = obj.T_W_B * [obj.PA4_B;1];
            PA4_W = PA4_W(1:3,1);
        end
        function PA5_W = get.PA5_W(obj)
            PA5_W = obj.T_W_B * [obj.PA5_B;1];
            PA5_W = PA5_W(1:3,1);
        end
        function PA6_W = get.PA6_W(obj)
            PA6_W = obj.T_W_B * [obj.PA6_B;1];
            PA6_W = PA6_W(1:3,1);
        end
        function Bxyz_W = get.Bxyz_W(obj)
            Bxyz_W = obj.T_W_B * [obj.Ob_B;1];
            Bxyz_W = Bxyz_W(1:3,1);
        end
        function Oxb_W = get.Oxb_W(obj)
            Oxb_W = obj.T_W_B * [obj.Oxb_B;1];
            Oxb_W = Oxb_W(1:3,1);
        end
        function oyb_W = get.Oyb_W(obj)
            oyb_W = obj.T_W_B * [obj.oyb_B;1];
            oyb_W = oyb_W(1:3,1);
        end
        function ozb_W = get.Ozb_W(obj)
            ozb_W = obj.T_W_B * [obj.ozb_B;1];
            ozb_W = ozb_W(1:3,1);
        end
        
        function EulerAngle = get.EulerAngle(obj)
            EulerAngle.roll = obj.roll;
            EulerAngle.yaw = obj.yaw;
            EulerAngle.pitch = obj.pitch;
        end
        
        function ContactPoint = get.ContactPoint(obj)
            count = 1;
            %���
            ContactPoint = [];
            if obj.leg1.isSupport == 1
                ContactPoint(count,:) = obj.leg1.p4_W';
%                 if ContactPoint(count,3)>-0.5 %����˵֧���ȵ���������ƽ����ʱ��Ӧ����-0.5֮��
%                     ss=1;
%                 end
                count = count + 1;
            end
            if obj.leg2.isSupport == 1
                ContactPoint(count,:) = obj.leg2.p4_W';
%                 if ContactPoint(count,3)>-0.5
%                     ss=1;
%                 end
                count = count + 1;
            end
            if obj.leg3.isSupport == 1
                ContactPoint(count,:) = obj.leg3.p4_W';
%                 if ContactPoint(count,3)>-0.5
%                     ss=1;
%                 end
                count = count + 1;
            end
            if obj.leg4.isSupport == 1
                ContactPoint(count,:) = obj.leg4.p4_W';
%                 if ContactPoint(count,3)>-0.5
%                     ss=1;
%                 end
                count = count + 1;
            end
            if obj.leg5.isSupport == 1
                ContactPoint(count,:) = obj.leg5.p4_W';
%                 if ContactPoint(count,3)>-0.5
%                     ss=1;
%                 end
                count = count + 1;
            end
            if obj.leg6.isSupport == 1
                ContactPoint(count,:) = obj.leg6.p4_W';
%                 if ContactPoint(count,3)>-0.5
%                     ss=1;
%                 end
            end
            ContactPoint = ContactPoint';
        end
        
        function SwingPoint = get.SwingPoint(obj)
            count = 1;
            %���
            SwingPoint = [];
            if obj.leg1.isSupport == 0
                SwingPoint(count,:) = obj.leg1.p4_W';   %ת��Ϊ������
                ID(count)=1;
                count = count + 1;
            end
            if obj.leg2.isSupport == 0
                SwingPoint(count,:) = obj.leg2.p4_W';
                ID(count)=2;
                count = count + 1;
            end
            if obj.leg3.isSupport == 0
                SwingPoint(count,:) = obj.leg3.p4_W';
                ID(count)=3;
                count = count + 1;
            end
            if obj.leg4.isSupport == 0
                SwingPoint(count,:) = obj.leg4.p4_W';
                ID(count)=4;
                count = count + 1;
            end
            if obj.leg5.isSupport == 0
                SwingPoint(count,:) = obj.leg5.p4_W';
                ID(count)=5;
                count = count + 1;
            end
            if obj.leg6.isSupport == 0
                SwingPoint(count,:) = obj.leg6.p4_W';
                ID(count)=6;
            end
            SwingPoint = SwingPoint';
            SwingPoint=[SwingPoint;ID]; %ǰ����ÿһ�д����Űڶ������λ�ã����һ�д���ÿһ�е��ȵĴ���
        end
        
        function BodyMargin = get.BodyMargin(obj)
%             BodyMargin = sqrt( (obj.x - obj.MinMarginIntersection(1,1))^2 + (obj.y - obj.MinMarginIntersection(1,2))^2  );
            BodyMargin = norm(obj.MinMarginIntersection - [obj.x,obj.y]);
        end
        
        
        function MinMarginIntersection = get.MinMarginIntersection(obj)
            [MinMarginIntersection,~] = GetMinInpolygonIntersection(obj.ContactPoint,[obj.x,obj.y]);
        end
        
        function leg1_kinematicsMargin = get.leg1_kinematicsMargin(obj)
            [~,leg1_kinematicsMargin] = obj.leg1.getKinematicsMargin(obj.MoveDirectionVector);
        end
        
        function leg2_kinematicsMargin = get.leg2_kinematicsMargin(obj)
            [~,leg2_kinematicsMargin] = obj.leg2.getKinematicsMargin(obj.MoveDirectionVector);
        end
        
        function leg3_kinematicsMargin = get.leg3_kinematicsMargin(obj)
            [~,leg3_kinematicsMargin] = obj.leg3.getKinematicsMargin(obj.MoveDirectionVector);
        end
        
        function leg4_kinematicsMargin = get.leg4_kinematicsMargin(obj)
            [~,leg4_kinematicsMargin] = obj.leg4.getKinematicsMargin(obj.MoveDirectionVector);
        end
        
        function leg5_kinematicsMargin = get.leg5_kinematicsMargin(obj)
            [~,leg5_kinematicsMargin] = obj.leg5.getKinematicsMargin(obj.MoveDirectionVector);
        end
        
        function leg6_kinematicsMargin = get.leg6_kinematicsMargin(obj)
            [~,leg6_kinematicsMargin] = obj.leg6.getKinematicsMargin(obj.MoveDirectionVector);
        end
    end
    

    %���û�����λ�ˣ�ͬʱ��Ҫ�����µ���ת���󴫵ݸ��Ȳ���Ա
    methods
        function obj = set_roll(obj,value)
            obj.roll = value;
            obj.leg1.T_W_B = obj.T_W_B;
            obj.leg2.T_W_B = obj.T_W_B;
            obj.leg3.T_W_B = obj.T_W_B;
            obj.leg4.T_W_B = obj.T_W_B;
            obj.leg5.T_W_B = obj.T_W_B;
            obj.leg6.T_W_B = obj.T_W_B;
        end
        function obj = set_pitch(obj,value)
            obj.pitch = value;
            obj.leg1.T_W_B = obj.T_W_B;
            obj.leg2.T_W_B = obj.T_W_B;
            obj.leg3.T_W_B = obj.T_W_B;
            obj.leg4.T_W_B = obj.T_W_B;
            obj.leg5.T_W_B = obj.T_W_B;
            obj.leg6.T_W_B = obj.T_W_B;
        end
        function obj = set_yaw(obj,value)
            obj.yaw = value;
            obj.leg1.T_W_B = obj.T_W_B;
            obj.leg2.T_W_B = obj.T_W_B;
            obj.leg3.T_W_B = obj.T_W_B;
            obj.leg4.T_W_B = obj.T_W_B;
            obj.leg5.T_W_B = obj.T_W_B;
            obj.leg6.T_W_B = obj.T_W_B;
        end
        function obj = set_x(obj,value)
            obj.x = value;
            obj.leg1.T_W_B = obj.T_W_B;
            obj.leg2.T_W_B = obj.T_W_B;
            obj.leg3.T_W_B = obj.T_W_B;
            obj.leg4.T_W_B = obj.T_W_B;
            obj.leg5.T_W_B = obj.T_W_B;
            obj.leg6.T_W_B = obj.T_W_B;
        end
        function obj = set_y(obj,value)
            obj.y = value;
            obj.leg1.T_W_B = obj.T_W_B;
            obj.leg2.T_W_B = obj.T_W_B;
            obj.leg3.T_W_B = obj.T_W_B;
            obj.leg4.T_W_B = obj.T_W_B;
            obj.leg5.T_W_B = obj.T_W_B;
            obj.leg6.T_W_B = obj.T_W_B;
        end
        function obj = set_z(obj,value)
            obj.z = value;
            obj.leg1.T_W_B = obj.T_W_B;
            obj.leg2.T_W_B = obj.T_W_B;
            obj.leg3.T_W_B = obj.T_W_B;
            obj.leg4.T_W_B = obj.T_W_B;
            obj.leg5.T_W_B = obj.T_W_B;
            obj.leg6.T_W_B = obj.T_W_B;
        end
    end

end
