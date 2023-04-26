function T = DH2T(varargin)
%计算DH参数下的齐次变换矩阵
%输入aplhai_1——αi-1连杆扭转角
%    a_i-1——连杆长度
%    di——连杆偏距
%    theta——关节角
%输出T——连杆坐标系i-1->i的齐次变换矩阵
%注意：输入的角度应为弧度
%      如果输入参数中有一个是sym，则其他的也应为sym

%alphai_1,ai_1,di,thetai
if nargin==4
    alphai_1 = varargin{1};ai_1 = varargin{2};di = varargin{3};thetai = varargin{4};
elseif nargin==1
    DH = varargin{1};
    alphai_1 = DH(1);ai_1 = DH(2);di = DH(3);thetai = DH(4);
end
T1 = angvec2tr(alphai_1,[1,0,0]);
T1([1:3],4) = [ai_1;0;0];
T2 = angvec2tr(thetai,[0,0,1]);
T2([1:3],4) = [0,0,di];
T = T1*T2;
end