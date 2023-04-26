%%
classdef TERRAIN_SAVE

    properties (Access = 'private')
        %地图的大小，栅格的尺寸
        xMin = -2;
        xMax = 5;
        yMin = -3;
        yMax = 3;
        xStep = 0.1; 
        yStep = 0.1; 
        
        %在构造函数中进行计算
        M = []; %X方向的栅格数量
        N = []; %Y方向的栅格数量
        xData = []; %绘制地图的x数据
        yData = []; %绘制地图的y数据
        zData = []; %绘制地图的z数据，z=-0.5
        
        colorMap = [];  %地图的颜色
        %用于记录永久改变的地图颜色
        colorMapBackgound = [];
        
        terrainSurfPlot;    %地图绘制的surface对象
        PointSphere=[];     %用于存放球
    end
    
    %定义相关颜色
    properties (Constant,Access = 'private')
        LightGray = [0.8;0.8;0.8];
        Gray = [0.75;0.75;0.75]; %视觉判断的地面不可落足点
        Blue = [0.117;0.56;1]; %三号腿视觉可落足点
        White = [1,1,1];
        Red = [1;0;0];
        Red2 = [1;0;1]; %一号腿视觉可落足点
        Purple = [0.63;0.13;0.94];%六号腿视觉可落足点
        Green  = [0;1;0];%二号腿视觉可落足点
        YellowDirt = [0.54;0.21;0.06]; %四号腿视觉可落足点
        Yellow = [1;1;0];%五号腿视觉可落足点
        Black = [0;0;0];%经触觉判断不可落足的点
    end
    
    %构造函数
    methods 
        function output = TERRAIN()
            output.M = ceil((output.xMax - output.xMin)/output.xStep);
            output.N = ceil((output.yMax - output.yMin)/output.yStep);

            [output.xData,output.yData]=meshgrid(linspace(output.xMin,output.xMax,output.M+1),linspace(output.yMin,output.yMax,output.N+1));
            output.zData = ones(output.N+1,output.M+1)*(-0.5);
            
            output.colorMap = zeros(size(output.xData,1),size(output.xData,2),3);
            for i=1:1:size(output.colorMap,1)
                for j=1:1:size(output.colorMap,2)
                    output.colorMap(i,j,:)= output.Gray;
                end
            end
            output.colorMapBackgound = output.colorMap; %都是Gray颜色
            
            figure;
            hold on;
            axis equal;
            axis([output.xMin,output.xMax,output.yMin,output.yMax,-1,0.5]);
            grid off;
            view(40,30);
            
            %坐标轴名称，字号，字体
            xlabel('x/m','fontsize',16,'fontname','Times'); 
            ylabel('y/m','fontsize',16,'fontname','Times'); 
            zlabel('z/m','fontsize',16,'fontname','Times');
            set(gca,'FontSize',16,'fontname','Times');
            %绘制地图surface面
            output.terrainSurfPlot = surface(output.xData,output.yData,output.zData,output.colorMap);
            %surface栅格线的颜色
            set(output.terrainSurfPlot,'edgecolor',[0.6 0.6 0.6]);
        end
    end
    
    methods
        %没有用到
        function obj = changeCellColorBackground(obj,x,y,Color)
            % 检测数据是否超过地图范围，扩展地图
            obj = obj.updateMapSize(x,y);
            
            i = ceil((x-obj.xMin)/obj.xStep+0.000000001);
            j = ceil((y-obj.yMin)/obj.yStep+0.000000001);
            
            switch Color
                case 'red'
                    obj.colorMapBackgound(j,i,1)= 0.89;
                    obj.colorMapBackgound(j,i,2)= 0.09;
                    obj.colorMapBackgound(j,i,3)= 0.05;
                case 'blue'
                    obj.colorMapBackgound(j,i,:)= obj.Blue;
                case 'green'
                    obj.colorMapBackgound(j,i,:)= obj.Green;
                case 'Purple'
                    obj.colorMapBackgound(j,i,:)= obj.Purple;
                case 'gray'
                    obj.colorMapBackgound(j,i,:)= obj.Gray;
                case 'red2'
                    obj.colorMapBackgound(j,i,:)= obj.Red2;
                case 'YellowDirt'
                    obj.colorMapBackgound(j,i,:)= obj.YellowDirt;
                case 'Yellow'
                    obj.colorMapBackgound(j,i,:)= obj.Yellow;
                case 'Black'
                    obj.colorMapBackgound(j,i,:)= obj.Black;
                case 'white'
                    obj.colorMapBackgound(j,i,:)= obj.White;
            end
        end
        
        %设置Map颜色和背景相同
        function obj = setColorAsBackground(obj)
            obj.colorMap = obj.colorMapBackgound;
        end
        
        %给x,y坐标位置的栅格换颜色
        function obj = changeCellColor(obj,x,y,Color)
            % 检测数据是否超过地图范围，扩展地图
            obj = obj.updateMapSize(x,y);
            
            i = ceil((x-obj.xMin)/obj.xStep+0.000000001);
            j = ceil((y-obj.yMin)/obj.yStep+0.000000001);
            
            switch Color
                case 'red'
                    obj.colorMap(j,i,1)= 0.89;
                    obj.colorMap(j,i,2)= 0.09;
                    obj.colorMap(j,i,3)= 0.05;
                case 'blue'
                    obj.colorMap(j,i,:)= obj.Blue;
                case 'green'
                    obj.colorMap(j,i,:)= obj.Green;
                case 'Purple'
                    obj.colorMap(j,i,:)= obj.Purple;
                case 'gray'
                    obj.colorMap(j,i,:)= obj.Gray;
                case 'red2'
                    obj.colorMap(j,i,:)= obj.Red2;
                case 'YellowDirt'
                    obj.colorMap(j,i,:)= obj.YellowDirt;
                case 'Yellow'
                    obj.colorMap(j,i,:)= obj.Yellow;
                case 'Black'
                    obj.colorMap(j,i,:)= obj.Black;
                case 'white'
                    obj.colorMap(j,i,:)= obj.White;
            end
            
            
        end
        
        %根据colorMap更新地图颜色
        function obj = plotTerrain(obj)
            delete(obj.terrainSurfPlot);
            obj.terrainSurfPlot = surface(obj.xData,obj.yData,obj.zData,obj.colorMap);
            set(obj.terrainSurfPlot,'edgecolor',[0.69 0.8 0.9]);    %换了栅格边线的颜色
        end
        
        %没有用到
        function obj=DeletePointSphere(obj)
            for i=1:size(obj.PointSphere,1)
                delete(obj.PointSphere(i));
            end
        end
        %没有用到
        function obj = deleteSphereList(obj)
            delete(obj.PointSphere);
        end
        
        
        function obj = GenerateSphere(obj, r, centerx, centery, centerz,Color) %在地面生成球面 N网格密度，不写此参数的话默认为50
%             hold on;
            delete(obj.terrainSurfPlot);
            delete(obj.PointSphere);
        %             
%             
            %为了增加立体感，考虑到球是在地面上的，所以centerz应该加一点或者减一点。
            centerz=centerz+0.01;
            % 检测数据是否超过地图范围，扩展地图
            obj = obj.updateMapSize(centerx,centery);
            obj.terrainSurfPlot = surface(obj.xData,obj.yData,obj.zData,obj.colorMap);
            set(obj.terrainSurfPlot,'edgecolor',[0.69 0.8 0.9]);
%             
%             i = ceil((centerx-obj.xMin)/obj.xStep+0.000000001);
%             j = ceil((centery-obj.yMin)/obj.yStep+0.000000001);
%             
            
            [x,y,z] = sphere(20);
            obj.PointSphere=surf(r*x+centerx, r*y+centery, r*z+centerz);
%             obj.PointSphere = [obj.PointSphere;mid];
            
%             ii=size(obj.PointSphere,1);
%             delete(obj.PointSphere(ii));
            switch Color
                case 'red'
                    obj.PointSphere.EdgeColor =obj.Red';
                    obj.PointSphere.FaceColor =obj.Red' ;
                case 'blue'
                    obj.PointSphere.EdgeColor =obj.Blue';
                    obj.PointSphere.FaceColor =obj.Blue' ;
                case 'green'
                    obj.PointSphere.EdgeColor =obj.Green';
                    obj.PointSphere.FaceColor =obj.Green' ;
                case 'Purple'
                    obj.PointSphere.EdgeColor =obj.Purple';
                    obj.PointSphere.FaceColor =obj.Purple' ;
                case 'gray'
                    obj.PointSphere.EdgeColor =obj.Gray';
                    obj.PointSphere.FaceColor =obj.Gray' ;
                case 'red2'
                    obj.PointSphere.EdgeColor =obj.Red2';
                    obj.PointSphere.FaceColor =obj.Red2' ;
                case 'YellowDirt'
                    obj.PointSphere.EdgeColor =obj.YellowDirt';
                    obj.PointSphere.FaceColor =obj.YellowDirt' ;
                case 'Yellow'
                    obj.PointSphere.EdgeColor =obj.Yellow';
                    obj.PointSphere.FaceColor =obj.Yellow' ;
                case 'Black'
                    obj.PointSphere.EdgeColor =obj.Black';
                    obj.PointSphere.FaceColor =obj.Black' ;
                case 'white'
                    obj.PointSphere.EdgeColor =obj.White';
                    obj.PointSphere.FaceColor =obj.White' ;
            end
            
        end
        
        %更新地图大小
        function obj = updateMapSize(obj,x,y)
            mode1 = 0;
            mode2 = 0;
            mode3 = 0;
            mode4 = 0; 
            if x < obj.xMin, xMinTmp = x;mode1 = 1; end
            if y < obj.yMin, yMinTmp = y;mode2 = 1; end
            if x > obj.xMax, xMaxTmp = x;mode3 = 1; end
            if y > obj.yMax, yMaxTmp = y;mode4 = 1; end
            
            if mode4==1
                n = ceil((yMaxTmp - obj.yMax)/obj.yStep);
                obj.N = n + obj.N;
                obj.yMax = obj.yMax + n*obj.yStep;

                [obj.xData,obj.yData]=meshgrid(linspace(obj.xMin,obj.xMax,obj.M+1),linspace(obj.yMin,obj.yMax,obj.N+1));
                obj.zData = ones(obj.N+1,obj.M+1)*(-0.5);

                colorMapReserve = obj.colorMap;
                colorMapReserveBackground = obj.colorMapBackgound;

                obj.colorMap = zeros(size(obj.xData,1),size(obj.xData,2),3);
                obj.colorMapBackgound = zeros(size(obj.xData,1),size(obj.xData,2),3);

                for i=1:1:size(obj.colorMap,2)
                    for j=n+1:1:size(obj.colorMap,1)
                        obj.colorMap(j,i,:)= obj.Gray;
                        obj.colorMapBackgound(j,i,:)= obj.Gray;
                    end
                end
                obj.colorMapBackgound(1:obj.N+1 - n,1:obj.M+1,:) = colorMapReserveBackground;
                obj.colorMap(1:obj.N+1 - n,1:obj.M+1,:) = colorMapReserve;
                axis([obj.xMin,obj.xMax,obj.yMin,obj.yMax,-1,0.5]);
            end
            
            
            if mode3==1
                m = ceil((xMaxTmp - obj.xMax)/obj.xStep);
                obj.M = m + obj.M;
                obj.xMax = obj.xMax + m*obj.xStep;

                [obj.xData,obj.yData]=meshgrid(linspace(obj.xMin,obj.xMax,obj.M+1),linspace(obj.yMin,obj.yMax,obj.N+1));
                obj.zData = ones(obj.N+1,obj.M+1)*(-0.5);

                colorMapReserve = obj.colorMap;
                colorMapReserveBackground = obj.colorMapBackgound;

                obj.colorMap = zeros(size(obj.xData,1),size(obj.xData,2),3);
                obj.colorMapBackgound = zeros(size(obj.xData,1),size(obj.xData,2),3);

                obj.colorMap = zeros(size(obj.xData,1),size(obj.xData,2),3);

                for j=1:1:size(obj.colorMap,1)
                    for i=m+1:1:size(obj.colorMap,2)
                        obj.colorMap(j,i,:)= obj.Gray;
                        obj.colorMapBackgound(j,i,:)= obj.Gray;
                    end
                end
                obj.colorMap(1:obj.N+1,1:obj.M+1 - m,:) = colorMapReserve;
                obj.colorMapBackgound(1:obj.N+1,1:obj.M+1 - m,:) = colorMapReserveBackground;
                axis([obj.xMin,obj.xMax,obj.yMin,obj.yMax,-1,0.5]);
            end
            
            if mode2==1
                    n = ceil((obj.yMin - yMinTmp)/obj.yStep);
                    obj.N = n + obj.N;
                    obj.yMin = obj.yMin - n*obj.yStep;

                    [obj.xData,obj.yData]=meshgrid(linspace(obj.xMin,obj.xMax,obj.M+1),linspace(obj.yMin,obj.yMax,obj.N+1));
                    obj.zData = ones(obj.N+1,obj.M+1)*(-0.5);
                    
                    colorMapReserve = obj.colorMap;
                    colorMapReserveBackground = obj.colorMapBackgound;
                    
                    obj.colorMap = zeros(size(obj.xData,1),size(obj.xData,2),3);
                    obj.colorMapBackgound = zeros(size(obj.xData,1),size(obj.xData,2),3);

                    for i=1:1:size(obj.colorMap,2)
                        for j=1:1:n
                            obj.colorMap(j,i,:)= obj.Gray;
                            obj.colorMapBackgound(j,i,:)= obj.Gray;
                        end
                    end
                    obj.colorMap(n+1:obj.N+1,1:obj.M+1,:) = colorMapReserve;
                    obj.colorMapBackgound(n+1:obj.N+1,1:obj.M+1,:) = colorMapReserveBackground;
                    axis([obj.xMin,obj.xMax,obj.yMin,obj.yMax,-1,0.5]);
            end
            
            if mode1==1
                m = ceil((obj.xMin - xMinTmp)/obj.xStep);
                obj.M = m + obj.M;
                obj.xMin = obj.xMin - m*obj.xStep;

                [obj.xData,obj.yData]=meshgrid(linspace(obj.xMin,obj.xMax,obj.M+1),linspace(obj.yMin,obj.yMax,obj.N+1));
                obj.zData = ones(obj.N+1,obj.M+1)*(-0.5);

                colorMapReserve = obj.colorMap;
                colorMapReserveBackground = obj.colorMapBackgound;

                obj.colorMap = zeros(size(obj.xData,1),size(obj.xData,2),3);
                obj.colorMapBackgound = zeros(size(obj.xData,1),size(obj.xData,2),3);

                for j=1:1:size(obj.colorMap,1)
                    for i=1:1:m
                        obj.colorMap(j,i,:)= obj.Gray;
                        obj.colorMapBackgound(j,i,:)= obj.Gray;
                    end
                end
                obj.colorMap(1:obj.N+1,m+1:obj.M+1,:) = colorMapReserve;
                obj.colorMapBackgound(1:obj.N+1,m+1:obj.M+1,:) = colorMapReserveBackground;
                axis([obj.xMin,obj.xMax,obj.yMin,obj.yMax,-1,0.5]);
            end
        end
        
    end
    
end