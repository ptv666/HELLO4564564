 function TerrainMapConsiderFoot = processFootstepList() %xMin,xMax,yMin,yMax,footstepN Ϊ���Ժ�Ŀ��ظ������ԣ���������ڲ���

    load('C:\Users\10025\Documents\WorkFile\HexapodMatlab\FootstepDesign.mat');
    TerrainMap=footStepData; %����500�� ���� x1�Ӵ�С����
    
    
    FootSize=0.1;
%     Record=[];
    for i=1:size(TerrainMap,1)-1 %ȥ�����Ĺ��ڽӽ�����ŵ㣬�Ա��ܹ�װ���½�
        if TerrainMap(i,4)== 1
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