classdef HEXAPOD
    properties
        %有六条腿
        leg1 = LEG(1);leg2 = LEG(2); leg3 = LEG(3);
        leg4 = LEG(4);leg5 = LEG(5);leg6 = LEG(6);
        
        %创建地形
        terrain= TERRAIN();
        
        %机器人支撑状态信息
        SupportState=[1 1 1 1 1 1]; %表示当前支撑状态
        SupportStateList = [];      %支撑状态列表，支撑腿的数目大于3
        SupportStateListNew = [];   %新的支撑状态列表
        
        %body运动信息
        CurrentMaxStrideLength = 0; %当前支撑状态下可走的最大步长
        BodyStartPoint = [];        %机体在一个运动周期的初位置
        BodyEndPoint = [];          %机体在一个运动周期的末位置
        BodyActionIng = 0;          %标志身体是否在动 0为不动 1为动
        BodyTrajectoryRecordH = []; %所有运动点存储在这里边
        
        %一些全局信息
        MoveDirectionVector = [1,0,0];  %base坐标系下的移动方向，在demo中对它重新赋值了
        MoveDestinationPoint = [];      %机体移动的目标点
        VisionSensorRange = 5;          %视觉传感器的感知范围
    end
    
    properties (Access = 'private')
        roll = [];  %机体姿态
        pitch = [];
        yaw = [];
        x = []; %质心位置
        y = [];
        z = [];
        
    end
  
    properties (Access = 'private') %用于显示的“变量对象”
        %腿的连杆直线
        leg1_line = line();leg2_line = line();leg3_line = line();
        leg4_line = line();leg5_line = line();leg6_line = line();
        %腿的扇形
        leg1_sector_line = line();leg2_sector_line = line();leg3_sector_line = line();
        leg4_sector_line = line();leg5_sector_line = line();leg6_sector_line = line();
        %腿扇形的运动裕度
        leg1_kinematics_line = line();leg2_kinematics_line = line();leg3_kinematics_line = line();
        leg4_kinematics_line = line();leg5_kinematics_line = line();leg6_kinematics_line = line();
        
        %机器人机体六边形
        lineHexagon = line();
        %机体坐标系
        frame_base_x_line = line();frame_base_y_line = line();frame_base_z_line = line();
        
        
        
        %支撑多边形静态稳定裕度
        shortestMarginLine = line();
        %支撑多边形
        marginArea = line();        
        %显示前进方向的最大前进距离
        COG2PolygonLine=line();
        
        %容错腿足端的特殊显示
        DeadPointleg1=line();DeadPointleg2=line();DeadPointleg3=line();
        DeadPointleg4=line();DeadPointleg5=line();DeadPointleg6=line();
        
        %单腿上的关节和足端
        Pointgj1=line();Pointgj2=line();Pointgj3=line();Pointgj4=line();Pointgj5=line();Pointgj6=line();Pointgj7=line();Pointgj8=line();
        Pointgj9=line();Pointgj10=line();Pointgj11=line();Pointgj12=line();Pointgj13=line();Pointgj14=line();Pointgj15=line();Pointgj16=line();
        Pointgj17=line();Pointgj18=line();Pointgj19=line();Pointgj20=line();Pointgj21=line();Pointgj22=line();Pointgj23=line();Pointgj24=line();
    end
    
    %机体根关节六变形在机体坐标系下的位置（固定不变）
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
        Bxyz_W;     %质心位置坐标
        T_W_B;      %base相对于world的T矩阵
        BodyMargin; %机体稳定裕度；到多边形的最短边
        
        %单腿的扇形稳定裕度
        leg1_kinematicsMargin;leg2_kinematicsMargin;leg3_kinematicsMargin;
        leg4_kinematicsMargin;leg5_kinematicsMargin;leg6_kinematicsMargin;
        
        ContactPoint;   %当前支持脚的全局位置，3×n，每一列代表着支撑位置
        SwingPoint;     %摆动腿位置
    end

    properties (Dependent,Access = 'private')
        %小机体坐标系单位向量的末端
        Oxb_W;Oyb_W;Ozb_W;
        
        %可视化六边形坐标所用
        PA1_W;PA2_W;PA3_W;PA4_W;PA5_W;PA6_W;
        
        %支撑多边形的稳定裕度
        MinMarginIntersection;
    end
    
    
    methods 
        %构造函数
        function output = HEXAPOD(~)
            output.roll = 0;output.pitch = 0;output.yaw = 0;
            output.x = 0;output.y = 0;output.z = 0;
            output.leg1 = LEG(1);output.leg2 = LEG(2);output.leg3 = LEG(3);
            output.leg4 = LEG(4);output.leg5 = LEG(5);output.leg6 = LEG(6);
            output.terrain = TERRAIN();
            
            SupportListMid=output.GenerateSupportStateListNew(); %静态稳定的支撑状态集
            mid=SupportListMid(:,4);
            SupportListMid(:,4)=SupportListMid(:,6);
            SupportListMid(:,6)=mid;
            output.SupportStateList=SupportListMid;

            output = output.plotHexapod();
        end

        % 可视化函数，会重复调用，删除旧的，绘制新的
        %构造函数同时绘图使用的，绘制：机器人机体六边形、机器人六条腿的连杆、六条腿的18个关节、6个足端、容错腿的特殊显示；
        %6个扇形、6个扇形与前进方向交点、支撑多边形、质心到支撑多边形最短距离、质心前进方向和支撑多边形交点
        function obj=plotHexapod(obj)
            delete(obj.leg1_line);  %六条腿，实际上就是二连杆
            delete(obj.leg2_line);
            delete(obj.leg3_line);
            delete(obj.leg4_line);
            delete(obj.leg5_line);
            delete(obj.leg6_line);
            delete(obj.lineHexagon);    %机体，正六边形
            delete(obj.frame_base_x_line);
            delete(obj.frame_base_y_line);
            delete(obj.frame_base_z_line);
            delete(obj.leg1_sector_line);   %每条腿的扇形
            delete(obj.leg2_sector_line);
            delete(obj.leg3_sector_line);
            delete(obj.leg4_sector_line);
            delete(obj.leg5_sector_line);
            delete(obj.leg6_sector_line);
            delete(obj.leg1_kinematics_line);   %每条腿扇形的稳定裕度
            delete(obj.leg2_kinematics_line);
            delete(obj.leg3_kinematics_line);
            delete(obj.leg4_kinematics_line);
            delete(obj.leg5_kinematics_line);
            delete(obj.leg6_kinematics_line);
            
            delete(obj.marginArea);         
            delete(obj.shortestMarginLine);
            delete(obj.COG2PolygonLine);
            
            delete(obj.DeadPointleg1);  %没有落足点的腿
            delete(obj.DeadPointleg2);
            delete(obj.DeadPointleg3);
            delete(obj.DeadPointleg4);
            delete(obj.DeadPointleg5);
            delete(obj.DeadPointleg6);
            
            delete(obj.Pointgj1);   %六条腿上18个关节，加上6个足端
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
            
            %传递机体坐标系到世界坐标系的旋转矩阵
            obj.leg1.T_W_B = obj.T_W_B;
            obj.leg2.T_W_B = obj.T_W_B;
            obj.leg3.T_W_B = obj.T_W_B;
            obj.leg4.T_W_B = obj.T_W_B;
            obj.leg5.T_W_B = obj.T_W_B;
            obj.leg6.T_W_B = obj.T_W_B;
            
            %绘制扇形区域，扇形全是黑色
            obj.leg1_sector_line = plot3(obj.leg1.SectorPlotData(1,:),obj.leg1.SectorPlotData(2,:),obj.leg1.SectorPlotData(3,:),':','Color',[0.1 0.1 0.1],'LineWidth',1.5);
            obj.leg2_sector_line = plot3(obj.leg2.SectorPlotData(1,:),obj.leg2.SectorPlotData(2,:),obj.leg2.SectorPlotData(3,:),':','Color',[0.1 0.1 0.1],'LineWidth',1.5);
            obj.leg3_sector_line = plot3(obj.leg3.SectorPlotData(1,:),obj.leg3.SectorPlotData(2,:),obj.leg3.SectorPlotData(3,:),':','Color',[0.1 0.1 0.1],'LineWidth',1.5);
            obj.leg4_sector_line = plot3(obj.leg4.SectorPlotData(1,:),obj.leg4.SectorPlotData(2,:),obj.leg4.SectorPlotData(3,:),':','Color',[0.1 0.1 0.1],'LineWidth',1.5);
            obj.leg5_sector_line = plot3(obj.leg5.SectorPlotData(1,:),obj.leg5.SectorPlotData(2,:),obj.leg5.SectorPlotData(3,:),':','Color',[0.1 0.1 0.1],'LineWidth',1.5);
            obj.leg6_sector_line = plot3(obj.leg6.SectorPlotData(1,:),obj.leg6.SectorPlotData(2,:),obj.leg6.SectorPlotData(3,:),':','Color',[0.1 0.1 0.1],'LineWidth',1.5);
            
            %绘制单腿和扇形的交点，从当前落足位置到与扇形交点
            [lineData,~] = obj.leg1.getKinematicsMargin(obj.MoveDirectionVector);
            obj.leg1_kinematics_line = plot3(lineData(1,:),lineData(2,:),lineData(3,:),'g-');   %红色，更改之前
            [lineData,~] = obj.leg2.getKinematicsMargin(obj.MoveDirectionVector);
            obj.leg2_kinematics_line = plot3(lineData(1,:),lineData(2,:),lineData(3,:),'g-');   %蓝色
            [lineData,~] = obj.leg3.getKinematicsMargin(obj.MoveDirectionVector);
            obj.leg3_kinematics_line = plot3(lineData(1,:),lineData(2,:),lineData(3,:),'g-');   %绿色
            [lineData,~] = obj.leg4.getKinematicsMargin(obj.MoveDirectionVector);
            obj.leg4_kinematics_line = plot3(lineData(1,:),lineData(2,:),lineData(3,:),'g-');   %红色
            [lineData,~] = obj.leg5.getKinematicsMargin(obj.MoveDirectionVector);
            obj.leg5_kinematics_line = plot3(lineData(1,:),lineData(2,:),lineData(3,:),'g-','LineWidth',3);   %红色
            [lineData,~] = obj.leg6.getKinematicsMargin(obj.MoveDirectionVector);
            obj.leg6_kinematics_line = plot3(lineData(1,:),lineData(2,:),lineData(3,:),'g-');   %红色

            %绘制腿部的关节，四个点一条腿：根关节位置、二连杆的两个关节、足端
            lineL1 = [obj.leg1.p1_W,obj.leg1.p2_W,obj.leg1.p3_W,obj.leg1.p4_W];
            lineL2 = [obj.leg2.p1_W,obj.leg2.p2_W,obj.leg2.p3_W,obj.leg2.p4_W];
            lineL3 = [obj.leg3.p1_W,obj.leg3.p2_W,obj.leg3.p3_W,obj.leg3.p4_W];
            lineL4 = [obj.leg4.p1_W,obj.leg4.p2_W,obj.leg4.p3_W,obj.leg4.p4_W];
            lineL5 = [obj.leg5.p1_W,obj.leg5.p2_W,obj.leg5.p3_W,obj.leg5.p4_W];
            lineL6 = [obj.leg6.p1_W,obj.leg6.p2_W,obj.leg6.p3_W,obj.leg6.p4_W];
            lineH = [obj.PA1_W,obj.PA2_W,obj.PA3_W,obj.PA4_W,obj.PA5_W,obj.PA6_W,obj.PA1_W];    %机体的六边形
            
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
            
            %绘制支撑多边形
            temp = obj.ContactPoint;
            temp(:,size(obj.ContactPoint,2)+1) = obj.ContactPoint(:,1); %为了使多边形闭合
            obj.marginArea=plot3(temp(1,:),temp(2,:),(obj.z-0.5)*ones(1,size(temp,2)),':','Color',[0.1 0.1 0.1],'LineWidth',1.5);
            
            %绘制质心稳定裕度，质心到支撑多边形最短距离（不考虑前进方向）
            temp = [obj.x,obj.MinMarginIntersection(1,1);obj.y,obj.MinMarginIntersection(1,2);obj.z-0.5,obj.z-0.5];
            obj.shortestMarginLine =  plot3(temp(1,:),temp(2,:),temp(3,:),'b-','LineWidth',1.5);            
            
            %绘制腿部连杆、机体与机体坐标系
            obj.leg1_line = plot3(lineL1(1,:),lineL1(2,:),lineL1(3,:),'r-','LineWidth',3);
            obj.leg2_line = plot3(lineL2(1,:),lineL2(2,:),lineL2(3,:),'r-','LineWidth',3);
            obj.leg3_line = plot3(lineL3(1,:),lineL3(2,:),lineL3(3,:),'r-','LineWidth',3);
            obj.leg4_line = plot3(lineL4(1,:),lineL4(2,:),lineL4(3,:),'r-','LineWidth',3);
            obj.leg5_line = plot3(lineL5(1,:),lineL5(2,:),lineL5(3,:),'y-','LineWidth',3);  %五号腿的连杆特殊绘制，使用黄色
            obj.leg6_line = plot3(lineL6(1,:),lineL6(2,:),lineL6(3,:),'r-','LineWidth',3);
            obj.lineHexagon = plot3(lineH(1,:),lineH(2,:),lineH(3,:),'r-','LineWidth',3);
            obj.frame_base_x_line = plot3([obj.Bxyz_W(1),obj.Oxb_W(1)],[obj.Bxyz_W(2),obj.Oxb_W(2)],[obj.Bxyz_W(3),obj.Oxb_W(3)],'r'); %绘制机体坐标系
            obj.frame_base_y_line = plot3([obj.Bxyz_W(1),obj.Oyb_W(1)],[obj.Bxyz_W(2),obj.Oyb_W(2)],[obj.Bxyz_W(3),obj.Oyb_W(3)],'g');
            obj.frame_base_z_line = plot3([obj.Bxyz_W(1),obj.Ozb_W(1)],[obj.Bxyz_W(2),obj.Ozb_W(2)],[obj.Bxyz_W(3),obj.Ozb_W(3)],'b');
            
            %在前进方向上，质心到支撑多边形的稳定裕度
            COG2PolygonDistance=obj.GetCOG2PolygonDistance();   
            Check=[obj.x obj.y 0]+COG2PolygonDistance*(obj.MoveDirectionVector/norm(obj.MoveDirectionVector));  %计算稳定裕度向量用于绘图
            obj.COG2PolygonLine=line([obj.x Check(1) ], [obj.y Check(2)],[-0.5 -0.5]);  %绘图
            
            %如果是容错退，那么就特殊绘制
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
        
        %函数没有被使用
        function [obj,flag] = changePos_fixLegPosition(obj,in_X,in_Y,in_Z,in_yaw,in_pitch,in_roll)%足端坐标固定情况下 改变六足姿态
            %计算新的机体到世界坐标系的旋转矩阵
            obj.x = in_X;
            obj.y = in_Y;
            obj.z = in_Z;
            obj.yaw = in_yaw;
            obj.pitch = in_pitch;
            obj.roll = in_roll;
            %将旋转矩阵传递到leg成员
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
            %通过逆运动学重新计算关节角度
            obj.leg1 = obj.leg1.OnrLegInverseKinematicsWoldAxis(leg1p4);
            obj.leg2 = obj.leg2.OnrLegInverseKinematicsWoldAxis(leg2p4);
            obj.leg3 = obj.leg3.OnrLegInverseKinematicsWoldAxis(leg3p4);
            obj.leg4 = obj.leg4.OnrLegInverseKinematicsWoldAxis(leg4p4);
            obj.leg5 = obj.leg5.OnrLegInverseKinematicsWoldAxis(leg5p4);
            obj.leg6 = obj.leg6.OnrLegInverseKinematicsWoldAxis(leg6p4);
            
            %flag为1，求解成功。 否者求解不成功
            flag =  (obj.leg1.isInverseSolution & obj.leg2.isInverseSolution & obj.leg3.isInverseSolution & ...
                    obj.leg4.isInverseSolution & obj.leg5.isInverseSolution & obj.leg6.isInverseSolution);
        end
        
        function Out=GetBodyPosition(obj) %获取六足坐标,行向量
            x1=obj.x;
            y1=obj.y;
            z1=obj.z;
            Out=[x1,y1,z1];
        end
        
        %提前先修改好T_W_B才能使用
        function [obj,flag] = changePos_fixLegPosition_SupportState(obj,in_X,in_Y,in_Z,in_yaw,in_pitch,in_roll) %按照supportstate修改机体参数
            %计算新的机体到世界坐标系的旋转矩阵
            obj.x = in_X;
            obj.y = in_Y;
            obj.z = in_Z;
            obj.yaw = in_yaw;
            obj.pitch = in_pitch;
            obj.roll = in_roll;
            
            flag=0;
            SupportState1=obj.SupportState; %支撑状态，6个0,1
            %将旋转矩阵传递到leg成员
            if SupportState1(1)==1
                leg1p4 = obj.leg1.p4_W;
                obj.leg1.T_W_B = obj.T_W_B;
                obj.leg1 = obj.leg1.OnrLegInverseKinematicsWoldAxis(leg1p4);            %通过逆运动学重新计算关节角度
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
            %flag为1，求解成功。 否者求解不成功
            if flag==sum(SupportState1)
                flag=1;
            else
                flag=0;
            end

            
%             flag =  (obj.leg1.isInverseSolution & obj.leg2.isInverseSolution & obj.leg3.isInverseSolution & ...
%                     obj.leg4.isInverseSolution & obj.leg5.isInverseSolution & obj.leg6.isInverseSolution);
         end
        
        function [obj,flag] = changePos_135fixLegPosition(obj,in_X,in_Y,in_Z,in_yaw,in_pitch,in_roll) %1 3 5 腿做支撑，计算机体移动后它们的新关节角
            %计算新的机体到世界坐标系的旋转矩阵
            obj.x = in_X;
            obj.y = in_Y;
            obj.z = in_Z;
            obj.yaw = in_yaw;
            obj.pitch = in_pitch;
            obj.roll = in_roll;

            %将旋转矩阵传递到leg成员
            leg1p4 = obj.leg1.p4_W;
            leg3p4 = obj.leg3.p4_W;
            leg5p4 = obj.leg5.p4_W;

            obj.leg1.T_W_B = obj.T_W_B;
             obj.leg2.T_W_B = obj.T_W_B;
            obj.leg3.T_W_B = obj.T_W_B;
             obj.leg4.T_W_B = obj.T_W_B;
            obj.leg5.T_W_B = obj.T_W_B;
             obj.leg6.T_W_B = obj.T_W_B;
             
            %通过逆运动学重新计算关节角度
            obj.leg1 = obj.leg1.OnrLegInverseKinematicsWoldAxis(leg1p4);
            obj.leg3 = obj.leg3.OnrLegInverseKinematicsWoldAxis(leg3p4);
            obj.leg5 = obj.leg5.OnrLegInverseKinematicsWoldAxis(leg5p4);
            
            %flag为1，求解成功。 否者求解不成功
            flag =  (obj.leg1.isInverseSolution  & obj.leg3.isInverseSolution  ...
                    & obj.leg5.isInverseSolution );
        end

        function [obj,flag] = changePos_246fixLegPosition(obj,in_X,in_Y,in_Z,in_yaw,in_pitch,in_roll) %2 4 6 腿做支撑，计算机体移动后它们的新关节角
            %计算新的机体到世界坐标系的旋转矩阵
            obj.x = in_X;
            obj.y = in_Y;
            obj.z = in_Z;
            obj.yaw = in_yaw;
            obj.pitch = in_pitch;
            obj.roll = in_roll;
%             obj.T_W_B = [angle2dcm(in_yaw,in_pitch,in_roll,'ZYX'),[in_X;in_Y;in_Z];[0,0,0,1];];
            %将旋转矩阵传递到leg成员

            leg2p4 = obj.leg2.p4_W;
            leg4p4 = obj.leg4.p4_W;
            leg6p4 = obj.leg6.p4_W;
            
            obj.leg1.T_W_B = obj.T_W_B;
            obj.leg2.T_W_B = obj.T_W_B;
            obj.leg3.T_W_B = obj.T_W_B;
            obj.leg4.T_W_B = obj.T_W_B;
            obj.leg5.T_W_B = obj.T_W_B;
            obj.leg6.T_W_B = obj.T_W_B;
            
            %通过逆运动学重新计算关节角度
            obj.leg2 = obj.leg2.OnrLegInverseKinematicsWoldAxis(leg2p4);
            obj.leg4 = obj.leg4.OnrLegInverseKinematicsWoldAxis(leg4p4);
            obj.leg6 = obj.leg6.OnrLegInverseKinematicsWoldAxis(leg6p4);
            
            %flag为1，求解成功。 否者求解不成功
            flag =  (  obj.leg2.isInverseSolution  & ...
                    obj.leg4.isInverseSolution & obj.leg6.isInverseSolution);
        end
        
        function [obj,TerrainMapConsiderFootMat1]=PutLegsOnUseableTerrainAtStartTime(obj,TerrainMapConsiderFootMat) %在一开始就把机器人放到落脚点上。
            obj.leg1.Initp4BPoint=obj.leg1.p4_B;
            obj.leg2.Initp4BPoint=obj.leg2.p4_B;
            obj.leg3.Initp4BPoint=obj.leg3.p4_B;
            obj.leg4.Initp4BPoint=obj.leg4.p4_B;
            obj.leg5.Initp4BPoint=obj.leg5.p4_B;
            obj.leg6.Initp4BPoint=obj.leg6.p4_B;
            
            temp=obj.ChangeSupport([0 0 0 0 0 0]);  %让所有腿都是摆动腿，寻找下一个支撑装填
            temp=temp.GetUseableFootTerrain(TerrainMapConsiderFootMat,[1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1]);
            Numleg1=size(temp.leg1.UsablePoint,1);  %得到每条腿可用的落足点数目，可行落足点是行向量存储，每一行是一个落足点
            Numleg2=size(temp.leg2.UsablePoint,1);
            Numleg3=size(temp.leg3.UsablePoint,1);
            Numleg4=size(temp.leg4.UsablePoint,1);
            Numleg5=size(temp.leg5.UsablePoint,1);
            Numleg6=size(temp.leg6.UsablePoint,1);
            if Numleg1~=0||Numleg2~=0||Numleg3~=0||Numleg4~=0||Numleg5~=0||Numleg6~=0
                disp('初始放置六足失败,没有足够的可用落足点，人为加落足点');
                TerrainMapConsiderFootMat1=[TerrainMapConsiderFootMat;obj.leg1.p4_W';obj.leg2.p4_W';obj.leg3.p4_W';obj.leg4.p4_W';obj.leg5.p4_W';obj.leg6.p4_W'];
                return;
            end
            
            %随机从备选落足点中选出一个落足点，然后把腿移动过去
            Numleg1i=randi(Numleg1,1,1); %为了避免一次性放不到正确的位置。
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
        %生成静态稳定的支撑状态集，生成一次就不会再变
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
        
        %由支撑腿稳定学裕度及机体稳定裕度获取最大迈步步长
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
            %稳定裕度最小值为0.05
            COG2PolygonDistance=obj.GetCOG2PolygonDistance() - 0.1;
            CurrentMaxStrideLength = min([margin1 margin2 margin3 margin4 margin5 margin6 COG2PolygonDistance])/2;
            if CurrentMaxStrideLength==10000
                CurrentMaxStrideLength=0;
                error("步长出错,支撑状态均为0");
            end
            if CurrentMaxStrideLength==0
                CurrentMaxStrideLength=0;
                error("不能走了");
            end
        end
        
        %获取前进方向重心到支撑多边形的距离
        function COG2PolygonDistance=GetCOG2PolygonDistance(obj) 
            X1 = obj.ContactPoint(1,:); %获取支撑位置的x,y坐标
            Y1 = obj.ContactPoint(2,:);
            flag=0;

            if( inpolygon(obj.x,obj.y,X1,Y1) == 1 ) %判断质心是否在支撑多边形内部
                for i=1:size(obj.ContactPoint,2)
                    XA=obj.ContactPoint(1,i);
                    YA=obj.ContactPoint(2,i);
                    ii=i+1;
                    if ii>size(obj.ContactPoint,2)
                        ii=1;
                    end
                    XB=obj.ContactPoint(1,ii);
                    YB=obj.ContactPoint(2,ii);
                    MoveDirection=obj.MoveDirectionVector/norm(obj.MoveDirectionVector); %行走的方向向量,在base坐标系下的
                    MoveDirection=obj.T_W_B(1:3,1:3)*MoveDirection'; %转化到世界坐标系
                    D=abs((YB-YA)*obj.x-(XB-XA)*obj.y+(XB-XA)*YA-XA*(YB-YA))/sqrt((YB-YA)^2+(XB-XA)^2); %点到直线的垂直距离
                    VectorD=[0 -1 0;1 0 0;0 0 1]*[XB-XA,YB-YA,0]'; %将直线所代表的向量旋转90度
                    VectorD=VectorD/norm(VectorD);
                    Costheta=dot(MoveDirection,VectorD);
                    Dist=abs(D/Costheta); %沿行进方向的与直线的交点的距离
                    Check=[obj.x obj.y 0]+Dist*(obj.MoveDirectionVector/norm(obj.MoveDirectionVector));
                    
                    Check(1)=roundn(Check(1),-4);  %保留四位小数
                    Check(2)=roundn(Check(2),-4);  %保留四位小数
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
                COG2PolygonDistance=0;  %这里没有报错是因为在获取机器人最大前进距离的函数中对移动距离为0的情况报错了。
            end
            if flag==0
                COG2PolygonDistance=0;
            end
        end
        
        %得到当前支撑状态下的腿与扇形之间的平均稳定裕度
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
        
        %返回六条腿中最小的扇形稳定裕度
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
        %改变当前的支撑状态
        function obj=ChangeSupport(obj,state)
            obj.leg1.isSupport=state(1);
            obj.leg2.isSupport=state(2);
            obj.leg3.isSupport=state(3);
            obj.leg4.isSupport=state(4);
            obj.leg5.isSupport=state(5);
            obj.leg6.isSupport=state(6);
            obj.SupportState=state;
        end
        
        %总共有多少条摆动腿
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
        
        %判断支撑状态和废腿是否自洽，%判断用来支撑的腿是否是废腿（没有落足点或者其他原因）
        function CheckFlagMatchDeadLeg=CheckSupportStateMatchDeadLeg(obj,SupportState) 
            CheckFlagMatchDeadLeg=1; %正常
            if SupportState(1)==1
                if obj.leg1.isDead==1
                    CheckFlagMatchDeadLeg=0;%不正常
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
        
        %效果不好 从新的当前支撑状态查表得下一支撑状态 根据获得的稳定裕度最大来决定
        function NextSupportState=GetNextSupportStateFromSupportStateList(obj,CurrentSupportState) 
            flag=0;
            NextSupportStateMargin = [];
            NextSupportStateKinematicMinus = [];
            
            for i=1:size(obj.SupportStateList,1)    
                NextSupportStateMargin(i) =obj.GetSupportStateMargin(obj.SupportStateList(i,:));
                NextSupportStateKinematicMinus(i) = obj.GetSupportStateKinematicMinus(obj.SupportStateList(i,:));
                %去掉当前状态，和全支撑状态，少写一句呀
                if( obj.SupportStateList(i,:) == [1,1,1,1,1,1])%CurrentMaxStrideLength
                    NextSupportStateMargin(i) = -1 ;
                end
            end
            
            for jjj = 1:size(obj.SupportStateList,1)
                    [~,i] = max(NextSupportStateMargin);
                    
                    count = 1;
                    %清空
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
                    
                    if( inpolygon(obj.x,obj.y,X1,Y1) == 1 && CheckFlagMatchDeadLeg==1 && NextSupportStateMargin(i) > 0) %选择xy在支撑多边形内的组合 废腿不能用来支撑
                        NextSupportState=obj.SupportStateList(i,:);
                        
                        %如果摆动腿全是废腿，且最低稳定裕度低于0.01 --->（机体不能继续前进），那么放弃该状态
                        if deadNum == swingNum  && NextSupportStateKinematicMinus(i) < 0.05 
                            NextSupportStateMargin(i) = -1;
                            continue;
                        end
                        
                        
                        %保证起步时机体稳定裕度大于0.1
                        [MinMarginIntersection1,~] = GetMinInpolygonIntersection([X1;Y1],[obj.x,obj.y]);
                        BodyMargin1 = norm(MinMarginIntersection1 - [obj.x,obj.y]);
                        if BodyMargin1 < 0.1 
                            NextSupportStateMargin(i) = -1;
                            continue;
                        end
                        
                        
                        
                        flag=1; %证明在表中找到了一组state 也有可能完全找不到state
                        
                        break;
                    else
                        NextSupportStateMargin(i) = -1;
                    end
            end
                    if flag==0
                           error('can not find next support state');
                    end
 
        end
        
        %假设只有落脚点能提供支撑
        function CurrentSupportState=GetCurrentSupportState(obj,TerrainMapConsiderFootMat) 
            Yuzhi=0.0001;
            CurrentSupportState=[0 0 0 0 0 0];
            
            %根据所有可行落足点进行判断
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
                disp('所有脚都悬空');
            end
            obj.SupportState=CurrentSupportState;   %由于不是引用，所以obj本身并没有发生改变
        end
        
        %从当前支撑状态查表得下一支撑状态 根据获得的稳定裕度最大来决定
        function NextSupportState=GetNextSupportState(obj,CurrentSupportState) 
            flag=0;

            NextSupportState=obj.SupportStateList(1,:);
            NextSupportStateMargin=obj.GetSupportStateMargin(obj.SupportStateList(1,:));

            for i=1:size(obj.SupportStateList,1)-1
                NextSupportStateMargin=obj.GetSupportStateMargin(obj.SupportStateList(i,:));
                if NextSupportStateMargin-CurrentSupportStateMargin>=Minus
                    count = 1;
                    %清空
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
                    if( inpolygon(obj.x,obj.y,X1,Y1) == 1 && CheckFlagMatchDeadLeg==1 ) %选择xy在支撑多边形内的组合 废腿不能用来支撑
                        NextSupportState=obj.SupportStateList(i,:);
                        Minus=NextSupportStateMargin-CurrentSupportStateMargin;
                        flag=1; %证明在表中找到了一组state 也有可能完全找不到state
                    end
                end
            end
            if flag==0
                NextSupportState=CurrentSupportState;
                aa=1;%完全找不到state 意味着所有的认为更好的state  都不在多边形内 但是当前的肯定在
            end
        end
        
        %由五次样条曲线规划出来的轨迹 t3 为持续时间 CurrentTime为当前时刻
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
                disp('机体移动失败');
            end
        end
        
        %更新机器人当前支撑状态
        function obj=UpdateLogicSupportState(obj)
            obj.SupportState = [obj.leg1.isSupport obj.leg2.isSupport obj.leg3.isSupport obj.leg4.isSupport obj.leg5.isSupport obj.leg6.isSupport];
        end
        
        %模拟六足对地面的视觉探测，进而得到在探测范围内的落足点表，寻找base五米之内的节点
        function TerrainMapVisionSensed=SenseTerrainMapByVision(obj,TerrainMapAll)
            SensorRange=obj.VisionSensorRange;
            TerrainMapVisionSensed=[];
            for i=1:size(TerrainMapAll,1)
                if TerrainMapAll(i,1)>obj.x-SensorRange&&TerrainMapAll(i,1)<obj.x+SensorRange&&TerrainMapAll(i,2)>obj.y-SensorRange&&TerrainMapAll(i,2)<obj.y+SensorRange && TerrainMapAll(i,3)>obj.z-SensorRange&&TerrainMapAll(i,3)<obj.z+SensorRange  %只检测在这个范围内的地面 一方面为了减少计算量 另一方面为了切合传感器探测范围有限的实际 
                    TerrainMapVisionSensed=[TerrainMapVisionSensed;TerrainMapAll(i,:)];
                end
            end
            if isempty(TerrainMapVisionSensed)==1
                disp('当前六足周围没有可用的落足点!');
            end
        end
        
        %检测可用的落脚点 TransitMatrix为将来规划的相对于现在的旋转矩阵
        function obj=GetUseableFootTerrain(obj,TerrainMap,TransitMatrix) 
            if obj.leg1.isSupport==0
                T_W_BFakeleg1=obj.leg1.T_W_B * TransitMatrix; %表示将T W B矩阵向身体移动方向移动之后的TWB
            end
            if obj.leg2.isSupport==0
                T_W_BFakeleg2=obj.leg2.T_W_B * TransitMatrix; %表示将T W B矩阵向身体移动方向移动之后的TWB
            end
            if obj.leg3.isSupport==0
                T_W_BFakeleg3=obj.leg3.T_W_B * TransitMatrix; %表示将T W B矩阵向身体移动方向移动之后的TWB
            end
            if obj.leg4.isSupport==0
                T_W_BFakeleg4=obj.leg4.T_W_B * TransitMatrix; %表示将T W B矩阵向身体移动方向移动之后的TWB
            end
            if obj.leg5.isSupport==0
                T_W_BFakeleg5=obj.leg5.T_W_B * TransitMatrix; %表示将T W B矩阵向身体移动方向移动之后的TWB
            end
            if obj.leg6.isSupport==0
                T_W_BFakeleg6=obj.leg6.T_W_B * TransitMatrix; %表示将T W B矩阵向身体移动方向移动之后的TWB
            end
            Leg1UseablePoint=[];
            Leg2UseablePoint=[];
            Leg3UseablePoint=[];
            Leg4UseablePoint=[];
            Leg5UseablePoint=[];
            Leg6UseablePoint=[];
            SensorRange=obj.VisionSensorRange;
            
            %清空地图颜色为背景颜色
            obj.terrain=obj.terrain.setColorAsBackground();
            
            %判断机体移动后，落足点是否在扇形内，给每条腿添加可以落足的点
            for i=1:size(TerrainMap,1)
                %只检测在这个范围内的地面 一方面为了减少计算量 另一方面为了切合传感器探测范围有限的实际
                if TerrainMap(i,1)>obj.x-SensorRange&&TerrainMap(i,1)<obj.x+SensorRange&&TerrainMap(i,2)>obj.y-SensorRange&&TerrainMap(i,2)<obj.y+SensorRange && TerrainMap(i,3)>obj.z-SensorRange&&TerrainMap(i,3)<obj.z+SensorRange
                    plot3(TerrainMap(i,1)',TerrainMap(i,2)',TerrainMap(i,3)','r.','MarkerSize',5);  %环境中的可落足点由该代码绘制
                    
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
            
            %为每条腿可以落足的点绘图
            if obj.leg1.isSupport==0
                obj.leg1.UsablePoint=Leg1UseablePoint;
                if isempty(Leg1UseablePoint) %如果是空的 说明腿废了 没有落足点
                    obj.leg1.isDead=1;
                     disp('leg1没有可落足点');
                else
                    obj.leg1.isDead=0;
                end
            end
            if obj.leg2.isSupport==0
                obj.leg2.UsablePoint=Leg2UseablePoint;
                if isempty(Leg2UseablePoint) %如果是空的 说明腿废了 没有落足点
                    obj.leg2.isDead=1;
                     disp('leg2没有可落足点');
                else
                    obj.leg2.isDead=0;
                end
            end
            if obj.leg3.isSupport==0
                obj.leg3.UsablePoint=Leg3UseablePoint;
                if isempty(Leg3UseablePoint) %如果是空的 说明腿废了 没有落足点
                    obj.leg3.isDead=1;
                     disp('leg3没有可落足点');
                else
                    obj.leg3.isDead=0;
                end
            end
            if obj.leg4.isSupport==0
                obj.leg4.UsablePoint=Leg4UseablePoint;
                if isempty(Leg4UseablePoint) %如果是空的 说明腿废了 没有落足点
                    obj.leg4.isDead=1;
                     disp('leg4没有可落足点');
                else
                    obj.leg4.isDead=0;
                end
            end
            if obj.leg5.isSupport==0
                obj.leg5.UsablePoint=Leg5UseablePoint;
                if isempty(Leg5UseablePoint) %如果是空的 说明腿废了 没有落足点
                    obj.leg5.isDead=1;
                     disp('leg5没有可落足点');
                else
                    obj.leg5.isDead=0;
                end
            end
            if obj.leg6.isSupport==0
                obj.leg6.UsablePoint=Leg6UseablePoint;
                if isempty(Leg6UseablePoint) %如果是空的 说明腿废了 没有落足点
                    obj.leg6.isDead=1;
                     disp('leg6没有可落足点');
                else
                    obj.leg6.isDead=0;
                end
            end
        end
        

    end

    methods  %dependent get函数 
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
            %清空
            ContactPoint = [];
            if obj.leg1.isSupport == 1
                ContactPoint(count,:) = obj.leg1.p4_W';
%                 if ContactPoint(count,3)>-0.5 %按理说支撑腿的纵坐标在平地上时不应该在-0.5之上
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
            %清空
            SwingPoint = [];
            if obj.leg1.isSupport == 0
                SwingPoint(count,:) = obj.leg1.p4_W';   %转换为行向量
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
            SwingPoint=[SwingPoint;ID]; %前三行每一列代表着摆动腿足端位置，最后一行代表每一列的腿的代号
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
    

    %设置机器人位姿，同时需要将跟新的旋转矩阵传递给腿部成员
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
