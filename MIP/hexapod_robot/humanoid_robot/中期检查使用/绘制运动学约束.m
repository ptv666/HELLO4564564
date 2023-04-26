%% 中期答辩绘制单腿工作空间，先运行test_human_planning_has_z.m文件形成一个机器人对象
figure
hold on
axis equal
ylim([-0.5,0.5])
feets_pos = [0,0.25,0,pi/7;0,-0.25,0,0].';  %第一列是左脚，第二列是右脚
 %左脚
    quiver3(feets_pos(1,1),feets_pos(2,1),feets_pos(3,1),...
       cos(feets_pos(4,1)),sin(feets_pos(4,1)),0,0.2,...
       'o','MarkerSize',5,'MarkerEdgeColor',[0.9 0.7 0.1],'MarkerFaceColor',[0.9 0.7 0.1],'Color',[0.9 0.7 0.1],'ShowArrowHead','On',...
       'LineWidth',2.5,'ShowArrowHead',true,'AutoScaleFactor',2)   %左脚
    %右脚
    quiver3(feets_pos(1,2),feets_pos(2,2),feets_pos(3,2),...
       cos(feets_pos(4,2)),sin(feets_pos(4,2)),0,0.2,...
       'o','MarkerSize',5,'MarkerEdgeColor',[0.3 0.9 0.2],'MarkerFaceColor',[0.3 0.9 0.2],'Color',[0.3 0.9 0.2],'ShowArrowHead','On',...
       'LineWidth',2.5,'ShowArrowHead',true,'AutoScaleFactor',2)   %右脚
    %质心 
    scatter3((feets_pos(1,1) + feets_pos(1,2))/2,...
        (feets_pos(2,1) + feets_pos(2,2))/2,...
        (feets_pos(3,1) + feets_pos(3,2))/2,...
        30,'MarkerEdgeColor',[1 0 0],'MarkerFaceColor',[1 0 0])   %质心
  
    a = 0.25;b = 0.15;
    rectangle = [-a,b;
                 a,b;
                 a,-b;
                 -a,-b;
                 -a,b];
    %右脚约束左脚
    T = [cos(feets_pos(4,2)),-sin(feets_pos(4,2));sin(feets_pos(4,2)),cos(feets_pos(4,2))]
    rectangle = (T*rectangle.').'
    new_center = feets_pos(1:2,2) + [-0.5*sin(feets_pos(4,2)); 0.5*cos(feets_pos(4,2))]
    plot([new_center(1),feets_pos(1,2)],[new_center(2),feets_pos(2,2)],'--','Color',[0.3 0.9 0.2])
    plot(rectangle(:,1) + feets_pos(1,1),rectangle(:,2) + feets_pos(2,1),'--','Color',[0.9 0.7 0.1])
    
    %左脚约束右脚
    T = [cos(feets_pos(4,1)),-sin(feets_pos(4,1));sin(feets_pos(4,1)),cos(feets_pos(4,1))]
    rectangle = (T*rectangle.').'
    new_center = feets_pos(1:2,1) + [0.5*sin(feets_pos(4,1)); -0.5*cos(feets_pos(4,1))]
    plot([new_center(1),feets_pos(1,1)],[new_center(2),feets_pos(2,1)],'--','Color',[0.9 0.7 0.1])
    plot(rectangle(:,1) + new_center(1),rectangle(:,2) + new_center(2),'--','Color',[0.3 0.9 0.2])    %右脚约束左脚
    legend('左脚','右脚','质心')
    