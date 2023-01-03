% -------------------------------------------------------------------------
% Description : �ߵ���̬���ٶȡ�λ�ø���
% Input       : ������ٶȡ����ٶ�
% Date        : 2023-01-03
% Author      : logzhan
% -------------------------------------------------------------------------
function [Qnb,Vel,Pos,AccN] = InsUpdate(gyro,acc,Qnb,Vel,Pos,InsDt,bias_acc,bias_gyro,InsInitFlag)

% ����λ�ø��µ���ģ�͡�Ϊ�˼�С���������Ժ���Խ�����һ�εĸ���Ƶ��
[Rmeri,Rprim,ge] = CalEarthModel(Pos);

% ���ݲ���������ȷ���Ƿ񴫸���������ƫ
% ע�ⷽ����ߵ���ʱ���Ǽ���ƫ���������˲�������ʱ������ƫ=����ƫ-X��
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
% ���ü��ٶȼ����������ǵ�Pitch��Roll
e(3) = 0;
gyro1 = gyro1 + Kp * e;  


%һ��������̬
wien = twe(Pos);
wenn = twv(Pos,Vel,Rmeri,Rprim);

Cbn  = Quat2DCM(Qnb);
% �۳�������ת���۳��ٶ�����Ľ��ٶ�֮����bϵ��ת�����ٶ�
wnbb = gyro1 - Cbn'*(wien + wenn);
% ��̬����
Qnb = QuatUpdate(Qnb,wnbb*InsDt);

%�ٶȸ���
AccN = Cbn*acc1;%������������Ա��ڿ������˲��Ĳ���ʹ��'

if(InsInitFlag == false)
    return;
end

% ����nϵ�������ٶ�
gn = [0;0;-ge];
% an = Cnb*fb - venn x (2*wien + wenn)
an = AccN - cross(wien + wien + wenn, Vel) + gn;
% �����ٶ�
%Vel = Vel + InsDt*an;

% Euler = Quat2EulerDeg(Qnb);
% EnuHeading = Euler(1) / (180 / pi);
% normVel = norm(Vel);
% ve = normVel * cos(EnuHeading);
% vn = normVel * sin(EnuHeading);
% vu = 0;
% Vel = [ve, vn, vu]';
            

% λ�ø���
dPos=zeros(3,1);
% ENU��ϵ���λ�ø��·���
H = Pos(3);
% ����λ�õ�΢��
% �����ٶȵõ�γ��
dPos(1) = Vel(2) / (Rmeri+H); 
% �����ٶȵõ�����
dPos(2) = Vel(1) / ((Rprim+H)*cos(Pos(1)));
% �߶ȸ��·���
dPos(3) = Vel(3);

% ����λ��
Pos = Pos + dPos * InsDt; 




