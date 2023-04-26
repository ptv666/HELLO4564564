function plot_delta_XY(Feet_State_List)
%绘制两步之间走了多远
%输入解出来的：Feet_State_List
    %计算左脚
    delta_XY_left = [];
    delta_XY_right = [];
    for i=3:2:size(Feet_State_List,2)-1
        temp_distance = norm(Feet_State_List(1:2,i)-Feet_State_List(1:2,i-2));
        delta_XY_left = [delta_XY_left,temp_distance];
    end
    for i=4:2:size(Feet_State_List,2)
        temp_distance = norm(Feet_State_List(1:2,i)-Feet_State_List(1:2,i-2));
        delta_XY_right = [delta_XY_right,temp_distance];
    end
    hold on
    plot(delta_XY_left,'-o','LineWidth',3.0,'Markersize',5.0)
    text(size(delta_XY_left,2)/2,delta_XY_left(2)/2,['左脚走了',num2str(size(delta_XY_left,2))])    %这里的位置选择全是为了绘图而设计的，没有任何技术可言
    plot(delta_XY_right,'--x','LineWidth',3.0,'Markersize',10.0)
    text(size(delta_XY_right,2)/2,delta_XY_right(2)/4,['右脚走了',num2str(size(delta_XY_right,2))]) %这里的位置选择全是为了绘图而设计的，没有任何技术可言
    legend('左脚位移','右脚位移')
    hold off
    axis([0 Inf 0 Inf])
end