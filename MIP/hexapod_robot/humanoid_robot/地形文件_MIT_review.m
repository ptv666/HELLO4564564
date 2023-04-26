%MIT论文复现，这个文件是地形文件
%% introduction图片复现
%使用说明:使用test_human_planning_has_z.m文件，把以下代码粘贴到“加入region”的模块内
%被注释的region_center是去掉中间一块得到的结果
% region_center = [0,0;...
%     0.4,0.25; 0.8,-0.25; 1.2,0.25; 1.6,-0.25; 2.4,-0.25; 2.8,0.25; 3.2,-0.25; 3.6,0.25;...
%     4,0;...
%     2,1.25];
region_center = [0,0;...
    0.4,0.25; 0.8,-0.25; 1.2,0.25; 1.6,-0.25; 2.0,0.25; 2.4,-0.25; 2.8,0.25; 3.2,-0.25; 3.6,0.25;...
    4,0;...
    2,1.25];
region_center(:,3) = 0;
length_x = ones(size(region_center,1),1) * 0.3;
length_y = ones(size(region_center,1),1) * 0.2;
length_x(1) = 0.4;  length_x(end-1) = 0.4;  length_x(end) = 4.5;
length_y(1) = 0.75; length_y(end-1) = 0.75; length_y(end) = 0.19;
rectangle_region_con=creat_rectangle_region_con_has_z(region_center,length_x,length_y);

human_robot.set_foot_region(rectangle_region_con,[1 1])
%% 高度自动选择
%实验1:直接走上去
region_center = [0,0,0; 0,2.5,0.02; 4,2,0.05; 4,-2,0.08; 4,0,0.12];
length_x = [4.98; 5; 2.7; 2.7; 2.7];
length_y = [3.98; 1; 2; 2; 1.5];
rectangle_region_con=creat_rectangle_region_con_has_z(region_center,length_x,length_y);
xlim([-5,8]),ylim([-5,5]),zlim([0,0.3])
%实验2:右侧走上去
region_center = [0,0,0; 0,2.5,0.02; 4,2,0.05; 4,-2,0.15; 4,0,0.3];
length_x = [4.98; 5; 2.5; 2.5; 1];
length_y = [3.98; 1; 2; 2; 1.5];
rectangle_region_con=creat_rectangle_region_con_has_z(region_center,length_x,length_y);
%实验3：左侧垫一步走上去
region_center = [0,0,0; 0,2.5,0.08; 4,2,0.2; 4,-2,0.10; 4,0,0.3];
length_x = [4.98; 5; 2.2; 2.2; 1.2];
length_y = [3.98; 1; 2; 2; 1.5];
rectangle_region_con=creat_rectangle_region_con_has_z(region_center,length_x,length_y);