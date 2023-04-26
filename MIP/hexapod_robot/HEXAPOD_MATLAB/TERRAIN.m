%%
classdef TERRAIN

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
        
        colorMap = [];  %地图的颜色，会随着扇形可行落足点出现而变换
        
        %用于记录永久改变的地图颜色,它一直存储着灰色，在构造函数中赋初值，当地图扩大时，也只保存灰色
        colorMapBackgound = [];
        
        terrainSurfPlot;    %地图绘制的surface对象

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
        %设置Map颜色和背景相同,实际上并没有感觉到有什么用，因为colorMap和colorMapBackgound似乎是完全相同的
        function obj = setColorAsBackground(obj)
            obj.colorMap = obj.colorMapBackgound;
        end
        
        %给x,y坐标位置的栅格换颜色->colorMap
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