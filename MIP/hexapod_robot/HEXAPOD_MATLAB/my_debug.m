clear;clc;close all
load('FootstepDesign.mat'); %落足点集和，每一行是一个落足点
TerrainMapConsiderFootMat=footStepData;

TouchInformation = 0 + (100-0).*rand(size(TerrainMapConsiderFootMat,1),1); %生成地形的触觉信息 是否能够落足用0-100的数量化 
TerrainMapConsiderFootMatIncludingTouch=[TerrainMapConsiderFootMat,TouchInformation]; %通过设置阈值，可以改变触发反射的概率

HEXAPOD1=HEXAPOD();
[temp,TerrainMapConsiderFootMat]=HEXAPOD1.PutLegsOnUseableTerrainAtStartTime(TerrainMapConsiderFootMat); %需要一开始就把六足放到落足点上，也就是在落足点地图中再添加6个新的落足点
HEXAPOD1=temp;



%新加入的六个点
TerrainMapConsiderFootMatIncludingTouchmid = TerrainMapConsiderFootMat(size(TerrainMapConsiderFootMatIncludingTouch,1)+1:size(TerrainMapConsiderFootMat,1),:);
TerrainMapConsiderFootMatIncludingTouchmid=[TerrainMapConsiderFootMatIncludingTouchmid,0 + (100-0).*rand(6,1)]; %生成六个点的物理特性
TerrainMapConsiderFootMatIncludingTouch=[TerrainMapConsiderFootMatIncludingTouch;TerrainMapConsiderFootMatIncludingTouchmid];   %把六个点加入到带有物理特性的落足点中