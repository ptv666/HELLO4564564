 function TerrainMapConsiderFoot=GenerateTerrainMap2D(xMin,xMax,yMin,yMax,footstepN) %xMin,xMax,yMin,yMax,footstepN 为了以后的可重复调用性，不设置入口参数
%     xMin = -8;
%     xMax = 8;
%     yMin = -8;
%     yMax = 8;

%     rectangle('Position',[xMin,yMin,xMax,yMax],...
%     'LineWidth',2,'LineStyle','-');
%     x=[xMin xMax xMax xMin];
%     y=[yMin yMin yMax yMax];
%     fill(x,y,'p')
%     hold on;

%     footstepN = 1500;
    x = xMin + (xMax-xMin).*rand(footstepN,1); 
    y = yMin + (yMax-yMin).*rand(footstepN,1);
    z = -0.5*ones(footstepN,1);
    x1=sort(x);
    for i=1:size(x1,1)
        for j=1:size(x,1)
            if x(j)==x1(i)
                y1(i,1)=y(j);
                break;
            end
        end
    end
    TerrainMap=[[x1,y1,z],ones(footstepN,1)]; %生成500行 三列 x1从大到小排列
    
    
    FootSize=0.1;
%     Record=[];
    for i=1:size(TerrainMap,1)-1 %去掉靠的过于接近的落脚点，以便能够装得下脚
        if TerrainMap(i,4)==1
            for j=i+1:size(TerrainMap,1)
                if norm(TerrainMap(i,1:3)-TerrainMap(j,1:3))>0&&norm(TerrainMap(i,1:3)-TerrainMap(j,1:3))<=FootSize&&TerrainMap(j,4)==1
                    TerrainMap(j,4)=0;
                    TerrainMap(j,5:7)=TerrainMap(i,1:3);
                end
            end
        end
    end
%     plot3(TerrainMap(:,1),TerrainMap(:,2),TerrainMap(:,3),'r.');
    TerrainMapConsiderFoot=[];
    for i=1:size(TerrainMap,1)
        if TerrainMap(i,4)==1
            TerrainMapConsiderFoot=[TerrainMapConsiderFoot;TerrainMap(i,1:3)];
        end
    end
% 	hold on 
    
%     plot3(TerrainMapConsiderFoot(:,1),TerrainMapConsiderFoot(:,2),TerrainMapConsiderFoot(:,3),'bo');

%     figure
%     for i=1:size(TerrainMapConsiderFoot,1)
%         x=TerrainMapConsiderFoot(i,1);
%         y=TerrainMapConsiderFoot(i,2);
%         r=0.1/2;
%         rectangle('Position',[x-r,y-r,2*r,2*r],'Curvature',[1,1],'linewidth',1),axis equal
%         hold on
%         drawnow
%     end

 end