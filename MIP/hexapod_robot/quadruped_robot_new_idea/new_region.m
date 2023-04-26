%new_region
%% 两个 左右 区域
region_center = [1,0.265,0; 1,-0.265,0];
length_x = [5; 5];
length_y = [0.1; 0.1];
rectangle_region_con_has_z=creat_rectangle_region_con_has_z(region_center,length_x,length_y);
%% 步态切换1的场景——专门设计的步态切换的实验区域
region_center = [0,0,0; 0.9,0,0.1; 1.575,0,0.2; 2.1,0,0.1; 3,0,0];
length_x = [0.95; 0.8; 0.5; 0.5; 1.2];
length_y = [0.8; 0.8; 0.8; 0.8; 0.8];
rectangle_region_con_has_z=creat_rectangle_region_con_has_z(region_center,length_x,length_y);

%%  步态切换2的场景——开始时刻的对角步态运动加一个周期
region_center = [0,0,0; 1.05,0,0.1; 1.715,0,0.2; 2.23,0,0.1; 3.1,0,0];
length_x = [1.3; 0.8; 0.5; 0.5; 1.2];
length_y = [0.8; 0.8; 0.8; 0.8; 0.8];
rectangle_region_con_has_z=creat_rectangle_region_con_has_z(region_center,length_x,length_y);
%% 步态切换2场景——增加结束时刻的区域长度
region_center = [0,0,0; 1.05,0,0.1; 1.715,0,0.2; 2.23,0,0.1; 3.5,0,0];
length_x = [1.3; 0.8; 0.5; 0.5; 2.0];
length_y = [0.8; 0.8; 0.8; 0.8; 0.8];
rectangle_region_con_has_z=creat_rectangle_region_con_has_z(region_center,length_x,length_y);
%% 四足Tonneau学者的场景——未实现
