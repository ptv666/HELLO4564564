function Plot_Steps_Length(quad_robot)
%为quad_robot绘制步长
% @quad_robot为求解后的机器人对象
    
    length_1 = []; length_2 = [];
    length_3 = []; length_4 = [];
    %XY方向位移
    for i=5:quad_robot.N
        foot = mod(i,4);
        distance = norm(quad_robot.vars.Feet_State_List.value(1:2,i) - quad_robot.vars.Feet_State_List.value(1:2,i-4));
        if foot==1
            length_1 = [length_1;distance];	%1-左前
        elseif foot==2
            length_2 = [length_2;distance];	%2-左后
        elseif foot==3
            length_3 = [length_3;distance];	%2-右后
        else
            length_4 = [length_4;distance];	%4-右前
        end
    end
    figure
    subplot(2,1,1)
    hold on
    plot(length_1, '-o', 'Color', [0.8 0.4 0.4],       'LineWidth', 3.0, 'MarkerEdgeColor', [0.8 0.4 0.4], 'MarkerFaceColor', [0.8 0.4 0.4])
    plot(length_2, '-o', 'Color', [0.9 0.7 0.1], 'LineWidth', 3.0, 'MarkerEdgeColor', [0.9 0.7 0.1], 'MarkerFaceColor', [0.9 0.7 0.1])
    plot(length_3, '--x','Color', [0 1 0.5],     'LineWidth', 1.5, 'MarkerEdgeColor', [0 1 0.5], 'MarkerFaceColor', [0 1 0.5], 'MarkerSize',15.0)
    plot(length_4, '--x','Color', [0 1 1.0],     'LineWidth', 1.5, 'MarkerEdgeColor', [0 1 1.0], 'MarkerFaceColor', [0 1 1.0], 'MarkerSize',15.0)
    legend('1-左前','2-左后','3-右后','4-右前','Location','best')
    title('XY方向的位移')
    hold off
    
    height_1 = []; height_2 = [];
    height_3 = []; height_4 = [];
    %Z方向高度变化
    for i=5:quad_robot.N
        foot = mod(i,4);
        delta_height = (quad_robot.vars.Feet_State_List.value(3,i) - quad_robot.vars.Feet_State_List.value(3,i-4));
        if foot==1
            height_1 = [height_1;delta_height];	%1-左前
        elseif foot==2
            height_2 = [height_2;delta_height];	%2-左后
        elseif foot==3
            height_3 = [height_3;delta_height];	%2-右后
        else
            height_4 = [height_4;delta_height];	%4-右前
        end
    end
%     figure
    subplot(2,1,2)
    hold on
    plot(height_1, '-o', 'Color', [0.8 0.4 0.4],       'LineWidth', 3.0, 'MarkerEdgeColor', [0.8 0.4 0.4], 'MarkerFaceColor', [0.8 0.4 0.4])
    plot(height_2, '-o', 'Color', [0.9 0.7 0.1], 'LineWidth', 3.0, 'MarkerEdgeColor', [0.9 0.7 0.1], 'MarkerFaceColor',[0.9 0.7 0.1])
    plot(height_3, '--x','Color', [0 1 0.5],     'LineWidth', 1.5, 'MarkerEdgeColor', [0 1 0.5], 'MarkerFaceColor', [0 1 0.5], 'MarkerSize',15.0)
    plot(height_4, '--x','Color', [0 1 1.0], 	 'LineWidth', 1.5, 'MarkerEdgeColor', [0 1 1.0], 'MarkerFaceColor', [0 1 1.0], 'MarkerSize',15.0)
    legend('1-左前','2-左后','3-右后','4-右前','Location','best')
    title('Z方向的位移')
    hold off
end