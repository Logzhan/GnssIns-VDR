%惯导解算
%输入加速度、角速度
function [atti,speed,pos,accn] = InsUpdate(gyro,acc,atti,speed,pos,dTins,bias_acc,bias_gyro)

% 根据位置更新地球模型。为了减小计算量，以后可以降低这一段的更新频率
[Rmeri,Rprim,ge] = CalEarthModel(pos);

% 根据参数的数量确定是否传感器处理零偏
%注意方向：算惯导的时候，是加零偏。卡尔曼滤波补偿的时候，新零偏=旧零偏-X。
if(nargin == 8)
    acc1  = acc  + bias_acc;
    gyro1 = gyro + bias_gyro;
else
    acc1  = acc;
    gyro1 = gyro;
end

%一、计算姿态
wien=twe(pos);
wenn=twv(pos,speed,Rmeri,Rprim);
Cbn=cbn(atti);
wnbb=gyro1-Cbn'*(wien+wenn);%扣除地球自转、扣除速度引起的角速度之后，在b系的转动角速度。
% 姿态更新
%atti = QuatUpdate(atti,wnbb*dTins);


%二、计算速度
accn=Cbn*acc1;%更新这个数，以便于卡尔曼滤波的部分使用

gn=[0;0;-ge];
an=accn-cross(wien+wien+wenn,speed)+gn;
speed=speed+dTins*an;%更新速度

%三、计算位置
dpos=zeros(3,1);

H=pos(3);
dpos(1)=speed(2)/(Rmeri+H);%北向速度得到纬度
dpos(2)=speed(1)/((Rprim+H)*cos(pos(1)));%东向速度得到经度
dpos(3)=speed(3);
pos=pos+dTins*dpos;%更新位置




