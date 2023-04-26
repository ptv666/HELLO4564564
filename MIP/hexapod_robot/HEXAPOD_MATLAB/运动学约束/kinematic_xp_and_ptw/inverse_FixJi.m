%% 求解逆运动学
% 输入fixJi坐标系的位置
% 输出三个关节角度
function theta = inverse_FixJi(foot_FixJi)
    tmp_theta1 = atan2(foot_FixJi(2), foot_FixJi(1));
    N = foot_FixJi(3);
    M = 0.0;
    if abs(foot_FixJi(2)) < 0.00000000001
        M = foot_FixJi(1) - 0.18;
    else
        M = foot_FixJi(2) / sin(tmp_theta1) - 0.18;
    end
    if (sqrt(M * M + N * N) > 0.5 + 0.5)
       disp('出错了')
    end
    theta = zeros(3,1);
    theta(1) = tmp_theta1;
    tmp_acos = acos((M * M + N * N) / sqrt(M * M + N * N));
    theta(2) = atan2(N, M) + tmp_acos;
    theta(3) = atan2(N - 0.5 * sin(theta(2)), M - 0.5 * cos(theta(2))) - theta(2);
    if theta(1)>2*pi
        theta(1) = theta(1) - 2*pi;
    end
    if theta(2)>2*pi
        theta(2) = theta(2) - 2*pi;
    end
    if theta(3)>2*pi
        theta(3) = theta(3) - 2*pi;
    end
    if theta(1)<-2*pi
        theta(1) = theta(1) + 2*pi;
    end
    if theta(2)<-2*pi
        theta(2) = theta(2) + 2*pi;
    end
    if theta(3)<-2*pi
        theta(3) = theta(3) + 2*pi;
    end
    if theta(3)>0
         theta(3) = theta(3) - 2*pi;
    end