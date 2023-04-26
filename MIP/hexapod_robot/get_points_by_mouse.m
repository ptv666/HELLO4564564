%% 鼠标采点
%前提是需要有一个图框，在图框内采点，按enter结束
x_get = 0;y_get = 0;
x = [];y = [];
while ~isempty(x_get) || ~isempty(y_get)
    [x_get,y_get] = ginput(1)
    hold on
    plot(x_get,y_get,'ro')
    drawnow
    x = [x;x_get];
    y = [y;y_get];
end
display('采点结束');
XY = [x,y];
XY = unique(XY,'rows');