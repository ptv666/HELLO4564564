function plot_delta_XY_has_Z(Feet_State_List)
%绘制两步之间走了多远
%输入解出来的：Feet_State_List
    delta_XY_left = [];
    delta_XY_right = [];
    delta_Z_left = [];
    delta_Z_right = [];
    
    %计算左脚
    for i=3:2:size(Feet_State_List,2)-1
        temp_distance_XY = norm(Feet_State_List(1:2,i)-Feet_State_List(1:2,i-2));
        temp_distance_Z = Feet_State_List(3,i)-Feet_State_List(3,i-2);
        delta_XY_left = [delta_XY_left,temp_distance_XY];
        delta_Z_left = [delta_Z_left;temp_distance_Z];
    end
    %计算右脚
    for i=4:2:size(Feet_State_List,2)
        temp_distance_XY = norm(Feet_State_List(1:2,i)-Feet_State_List(1:2,i-2));
        temp_distance_Z = Feet_State_List(3,i)-Feet_State_List(3,i-2);
        delta_XY_right = [delta_XY_right,temp_distance_XY];
        delta_Z_right = [delta_Z_right;temp_distance_Z];
    end
    
    %绘制xy位移
    subplot(2,1,1)
%     figure
    hold on
    plot(delta_XY_left,'-o','LineWidth',3.0,'Markersize',5.0)
    text(size(delta_XY_left,2)/2,delta_XY_left(2)/2,['左脚走了',num2str(size(delta_XY_left,2))])    %这里的位置选择全是为了绘图而设计的
    plot(delta_XY_right,'--x','LineWidth',3.0,'Markersize',10.0)
    text(size(delta_XY_right,2)/2,delta_XY_right(2)/4,['右脚走了',num2str(size(delta_XY_right,2))]) %这里的位置选择全是为了绘图而设计的
    legend('左脚XY位移','右脚XY位移')
    axis padded
    hold off
    %绘制z位移
    subplot(2,1,2)
%     figure
    hold on
    plot(delta_Z_left,'-o','LineWidth',3.0,'Markersize',5.0)
    plot(delta_Z_right,'--x','LineWidth',3.0,'Markersize',10.0)
    legend('左脚Z位移','右脚Z位移')
    axis padded
    hold off
end