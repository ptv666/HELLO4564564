points = [  0,  0,  0;
            1,  0,  0;
            0,  1,  0;
            1,  1,  0;
            1,  1,  1;
            0,  1,  1;
            1,  0,  1;
            0,  0,  1];
rng(10);
for i=1:5
    temp = rand(1,3);
    points = [points;temp];
end
[k,av] = convhull(points);
plot3(points(:,1),points(:,2),points(:,3),'*')
hold on
plot3(points(k,1),points(k,2),points(k,3))