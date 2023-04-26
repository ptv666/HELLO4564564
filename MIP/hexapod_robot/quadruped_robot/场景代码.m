%% 不太好的想法，一个倾斜的中间地形
region_center = [1,-0.265,0; 0,0.265,0; 1.1,0.265,0.02; 2.1,0.265,0];
length_x = [3;  1.75;  0.4;  1.55]*0.99;
length_y = [0.1;0.1;0.1;0.1]*0.99;
theta	 = [0;  0;  -6;  0]/180*pi;
QingXie_rectangle_region_con=creat__QingXie_rectangle_region_con(region_center,length_x,length_y,theta);

%%
region_center = [0,0,0; 0.8,0,0.04; 1.45,0,0.08; 2.05,0,0.04; 2.75,0,0];
length_x = [0.95; 0.65; 0.6; 0.55; 0.8]*0.99;
length_y = ones(size(region_center,1),1)*0.99;
theta	 = [0; 0; 0; 0; 0]/180*pi;
QingXie_rectangle_region_con=creat__QingXie_rectangle_region_con(region_center,length_x,length_y,theta);

%% 一个大块区域
region_center = [0,0,0];
length_x = [10];
length_y = [10];
theta	 = [0];
QingXie_rectangle_region_con=creat__QingXie_rectangle_region_con(region_center,length_x,length_y,theta);

%% 两个 左右 区域
region_center = [0.5,0.265,0; 0.5,-0.265,0];
length_x = [3; 3];
length_y = [0.1; 0.1];
theta	 = [0; 0];
QingXie_rectangle_region_con=creat__QingXie_rectangle_region_con(region_center,length_x,length_y,theta);

%% 阶梯形式
region_center = [0,0,0; 1.05,0,0.1; 1.715,0,0.2; 2.23,0,0.1; 3.5,0,0];
length_x = [1.3; 0.8; 0.5; 0.5; 2.0];
length_y = [0.8; 0.8; 0.8; 0.8; 0.8];
rectangle_region_con_has_z=creat_rectangle_region_con_has_z(region_center,length_x,length_y);