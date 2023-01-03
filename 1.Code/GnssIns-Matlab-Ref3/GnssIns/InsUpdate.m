% -------------------------------------------------------------------------
% Description : 惯导姿态、速度、位置更新
% Input       : 输入加速度、角速度
% Date        : 2023-01-03
% Author      : logzhan
% -------------------------------------------------------------------------
function [Qnb,Vel,Pos,AccN] = InsUpdate(gyro,acc,Qnb,Vel,Pos,InsDt,bias_acc,bias_gyro,InsInitFlag)

% 根据位置更新地球模型。为了减小计算量，以后可以降低这一段的更新频率
[Rmeri,Rprim,ge] = CalEarthModel(Pos);

% 根据参数的数量确定是否传感器处理零偏
% 注意方向：算惯导的时候，是加零偏。卡尔曼滤波补偿的时候，新零偏=旧零偏-X。
if(nargin == 10)
    acc1  = acc  + bias_acc;
    gyro1 = gyro + bias_gyro;
else
    acc1  = acc;
    gyro1 = gyro;
end

Kp = 0.5;
acc1 = acc1 / norm(acc1);
v = [2*(Qnb(2)*Qnb(4) - Qnb(1)*Qnb(3))
     2*(Qnb(1)*Qnb(2) + Qnb(3)*Qnb(4))
     Qnb(1)^2 - Qnb(2)^2 - Qnb(3)^2 + Qnb(4)^2];
e = cross(acc1, v);
% 利用加速度计修正陀螺仪的Pitch和Roll
e(3) = 0;
gyro1 = gyro1 + Kp * e;  


%一、计算姿态
wien = twe(Pos);
wenn = twv(Pos,Vel,Rmeri,Rprim);

Cbn  = Quat2DCM(Qnb);
% 扣除地球自转、扣除速度引起的角速度之后，在b系的转动角速度
wnbb = gyro1 - Cbn'*(wien + wenn);
% 姿态更新
Qnb = QuatUpdate(Qnb,wnbb*InsDt);

%速度更新
AccN = Cbn*acc1;%更新这个数，以便于卡尔曼滤波的部分使用'

if(InsInitFlag == false)
    return;
end

% 更新n系重力加速度
gn = [0;0;-ge];
% an = Cnb*fb - venn x (2*wien + wenn)
an = AccN - cross(wien + wien + wenn, Vel) + gn;
% 更新速度
%Vel = Vel + InsDt*an;

% Euler = Quat2EulerDeg(Qnb);
% EnuHeading = Euler(1) / (180 / pi);
% normVel = norm(Vel);
% ve = normVel * cos(EnuHeading);
% vn = normVel * sin(EnuHeading);
% vu = 0;
% Vel = [ve, vn, vu]';
            

% 位置更新
dPos=zeros(3,1);
% ENU组合导航位置更新方程
H = Pos(3);
% 计算位置的微分
% 北向速度得到纬度
dPos(1) = Vel(2) / (Rmeri+H); 
% 东向速度得到经度
dPos(2) = Vel(1) / ((Rprim+H)*cos(Pos(1)));
% 高度更新方程
dPos(3) = Vel(3);

% 更新位置
Pos = Pos + dPos * InsDt; 




