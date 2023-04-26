function new_Steps_Length_plot(new_quad)
%为new_quad绘制步长
% @new_quad为求解后的机器人对象
    
    length_1 = []; length_2 = [];
    length_3 = []; length_4 = [];
    
    %XY方向位移
    for j=2:new_quad.N
        for i=1:4
            temp_index = (4*i-3):(4*i-2);
            distance = norm(new_quad.vars.Leg_state.value(temp_index,j) - new_quad.vars.Leg_state.value(temp_index,j-1));
            if i==1
                length_1 = [length_1;distance];	%1-左前
            elseif i==2
                length_2 = [length_2;distance];	%2-左后
            elseif i==3
                length_3 = [length_3;distance];	%2-右后
            else
                length_4 = [length_4;distance];	%4-右前
            end
        end
    end

    figure
    subplot(4,1,1)
    plot(length_1);
    title('XY方向的位移')
    legend('1-左前','Location','best')
    
    subplot(4,1,2)
    plot(length_2);
    title('XY方向的位移')
    legend('2-左后','Location','best')
    
    subplot(4,1,3)
    plot(length_3);
    title('XY方向的位移')
    legend('3-右后','Location','best')
    
    subplot(4,1,4)
    plot(length_4);
    title('XY方向的位移')
    legend('4-右前','Location','best')
    
    hold off
    
    height_1 = []; height_2 = [];
    height_3 = []; height_4 = [];
    %Z方向高度变化
    for j=2:new_quad.N
        for i=1:4
            temp_index = 4*i-1;
            distance = new_quad.vars.Leg_state.value(temp_index,j) - new_quad.vars.Leg_state.value(temp_index,j-1);
            if i==1
                height_1 = [height_1;distance];	%1-左前
            elseif i==2
                height_2 = [height_2;distance];	%2-左后
            elseif i==3
                height_3 = [height_3;distance];	%2-右后
            else
                height_4 = [height_4;distance];	%4-右前
            end
        end
    end
    
    figure
    subplot(4,1,1)
    plot(height_1);
    legend('1-左前','Location','best')
    title('Z方向的位移')
    
    subplot(4,1,2)
    plot(height_2);
    legend('2-左后','Location','best')
    title('Z方向的位移')
    
    subplot(4,1,3)
    plot(height_3);
    legend('3-右后','Location','best')
    title('Z方向的位移')
    
    subplot(4,1,4)
    plot(height_4);
    legend('4-右前','Location','best')
    title('Z方向的位移')
    
    hold off
end