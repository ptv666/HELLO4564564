P = [0 0; 1 1; 1.5 0.5; 1.5 -0.5; 1.25 0.3; 1 0; 1.25 -0.3; 1 -1];
[k,av] = convhull(P);
plot(P(:,1),P(:,2),'*')
hold on
plot(P(k,1),P(k,2))