clear;
close all;
hexapod = HEXAPOD();
return;
%% 逆运动学绘制工作空间

initialPP = hexapod.leg1.p4_W;
initialP = hexapod.leg1.p2_W;
initialTheta = hexapod.leg1.theta;
for theta = initialTheta - pi*45/180 : 0.05 : initialTheta + pi*45/180
    for len = 0 : 0.05 : 1
        p(1) = initialP(1) + len * cos(theta);
        p(2) = initialP(2) + len * sin(theta);
        p(3) = -0.4;
        hexapod.leg1 = hexapod.leg1.OnrLegInverseKinematicsWoldAxis(p');
        hexapod = hexapod.plotHexapod();
        plot3(hexapod.leg1.p4_W(1),hexapod.leg1.p4_W(2),hexapod.leg1.p4_W(3),'.');
        pause(0.05);
    end
end
hexapod.leg1 = hexapod.leg1.OnrLegInverseKinematicsWoldAxis(initialPP);
hexapod = hexapod.plotHexapod();



initialPP = hexapod.leg2.p4_W;
initialP = hexapod.leg2.p2_W;
initialTheta = hexapod.leg2.theta;

for theta = initialTheta - pi*45/180 : 0.05 : initialTheta + pi*45/180
    for len = 0 : 0.05 : 1
        p(1) = initialP(1) + len * cos(theta);
        p(2) = initialP(2) + len * sin(theta);
        p(3) = -0.5;
        hexapod.leg2 = hexapod.leg2.OnrLegInverseKinematicsWoldAxis(p');
        hexapod = hexapod.plotHexapod();
        plot3(hexapod.leg2.p4_W(1),hexapod.leg2.p4_W(2),hexapod.leg2.p4_W(3),'.');
        pause(0.05);
    end
end
        hexapod.leg2 = hexapod.leg2.OnrLegInverseKinematicsWoldAxis(initialPP);
        hexapod = hexapod.plotHexapod();

        
initialPP = hexapod.leg3.p4_W;
initialP = hexapod.leg3.p2_W;
initialTheta = hexapod.leg3.theta;

for theta = initialTheta - pi*45/180 : 0.05 : initialTheta + pi*45/180
    for len = 0 : 0.05 : 1
        p(1) = initialP(1) + len * cos(theta);
        p(2) = initialP(2) + len * sin(theta);
        p(3) = -0.5;
        hexapod.leg3 = hexapod.leg3.OnrLegInverseKinematicsWoldAxis(p');
        hexapod = hexapod.plotHexapod();
        plot3(hexapod.leg3.p4_W(1),hexapod.leg3.p4_W(2),hexapod.leg3.p4_W(3),'.');
        pause(0.05);
    end
end
        hexapod.leg3 = hexapod.leg3.OnrLegInverseKinematicsWoldAxis(initialPP);
        hexapod = hexapod.plotHexapod();



initialPP = hexapod.leg4.p4_W;
initialP = hexapod.leg4.p2_W;
initialTheta = hexapod.leg4.theta;

for theta = initialTheta - pi*45/180 : 0.05 : initialTheta + pi*45/180
    for len = 0 : 0.05 : 1
        p(1) = initialP(1) + len * cos(theta);
        p(2) = initialP(2) + len * sin(theta);
        p(3) = -0.5;
        hexapod.leg4 = hexapod.leg4.OnrLegInverseKinematicsWoldAxis(p');
        hexapod = hexapod.plotHexapod();
        plot3(hexapod.leg4.p4_W(1),hexapod.leg4.p4_W(2),hexapod.leg4.p4_W(3),'.');
        pause(0.05);
    end
end
        hexapod.leg4 = hexapod.leg4.OnrLegInverseKinematicsWoldAxis(initialPP);
        hexapod = hexapod.plotHexapod();


initialPP = hexapod.leg5.p4_W;
initialP = hexapod.leg5.p2_W;
initialTheta = hexapod.leg5.theta;

for theta = initialTheta - pi*45/180 : 0.05 : initialTheta + pi*45/180
    for len = 0 : 0.05 : 1
        p(1) = initialP(1) + len * cos(theta);
        p(2) = initialP(2) + len * sin(theta);
        p(3) = -0.5;
        hexapod.leg5 = hexapod.leg5.OnrLegInverseKinematicsWoldAxis(p');
        hexapod = hexapod.plotHexapod();
        plot3(hexapod.leg5.p4_W(1),hexapod.leg5.p4_W(2),hexapod.leg5.p4_W(3),'.');
        pause(0.05);
    end
end
        hexapod.leg5 = hexapod.leg5.OnrLegInverseKinematicsWoldAxis(initialPP);
        hexapod = hexapod.plotHexapod();


initialPP = hexapod.leg6.p4_W;
initialP = hexapod.leg6.p2_W;
initialTheta = hexapod.leg6.theta;

for theta = initialTheta - pi*45/180 : 0.05 : initialTheta + pi*45/180
    for len = 0 : 0.05 : 1
        p(1) = initialP(1) + len * cos(theta);
        p(2) = initialP(2) + len * sin(theta);
        p(3) = -0.5;
        hexapod.leg6 = hexapod.leg6.OnrLegInverseKinematicsWoldAxis(p');
        hexapod = hexapod.plotHexapod();
        plot3(hexapod.leg6.p4_W(1),hexapod.leg6.p4_W(2),hexapod.leg6.p4_W(3),'.');
        pause(0.05);
    end
end
        hexapod.leg6 = hexapod.leg6.OnrLegInverseKinematicsWoldAxis(initialPP);
        hexapod = hexapod.plotHexapod();