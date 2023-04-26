%双足人形机器人的思路：
%先构建接触序列:               obj.addVariable('Feet_State_List','C',[3,2+obj.N],-10,10); 
%定义末状态到goal的代价值      obj.addCost(Q,c,alpha);   %添加序列末状态和目标状态之间的差值
%定义两步之间的代价值          obj.addCost(Q,[],[]);
%约束单腿运动空间以寻找下一落足点
%: obj.addVariable('sin_t','C',[1 obj.N],-1,1);obj.addVariable('cos_t','C',[1 obj.N],-1,1);
%加入sin，cos
%加入trim自适应调整步长
%加入terrain区域选择

%% 已有变量
%足端接触序列
obj.addVariable('Feet_State_List','C',[3,2+obj.N],-10,10); 
%sin与cos
obj.addVariable('sin_t','C',[1 obj.N],-1,1);
obj.addVariable('cos_t','C',[1 obj.N],-1,1);
%可行矩形的中心
obj.addVariable('foot_feasible_center','C',[2 obj.N],-10,10);


 obj.addSinConstraint(theta_index(i),sin_t_index(i));
                obj.addSinConstraint(theta_index(i),cos_t_index(i));