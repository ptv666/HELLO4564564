function V = hexahedron_V(t1,t2,t3)
%计算驱动器力空间六面体的体积
    J = [-sin(t1)*(0.18 + 0.5*cos(t2 + t3) + 0.5*cos(t2)), -cos(t1)*(0.5*sin(t2 + t3) + 0.5*sin(t2)), -0.5*sin(t2 + t3)*cos(t1);
          cos(t1)*(0.18 + 0.5*cos(t2 + t3) + 0.5*cos(t2)), -sin(t1)*(0.5*sin(t2 + t3) + 0.5*sin(t2)), -0.5*sin(t2 + t3)*sin(t1);
                                                    0,            0.5*cos(t2 + t3) + 0.5*cos(t2),          0.5*cos(t2 + t3)];
    J_T_inverse = pinv(J.');    %结果是3*3矩阵，每一列代表着力空间的坐标轴向量
    J_force = zeros(3,3);
    J_force(:,1) = 2*J_T_inverse(:,1)*1673.28 * 1e-3;  %根关节驱动器力矩为1673.28N*M
    J_force(:,2) = 2*J_T_inverse(:,2)*705.960 * 1e-3;  %髋关节驱动器力矩为705.960N*M
    J_force(:,3) = 2*J_T_inverse(:,3)*763.20 * 1e-3;   %膝关节驱动器力矩为763.20N*M
    
    V = abs(det(J_force));
end