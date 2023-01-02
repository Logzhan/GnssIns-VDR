%�ߵ�����
%������ٶȡ����ٶ�
function [atti,speed,pos,accn] = InsUpdate(gyro,acc,atti,speed,pos,dTins,bias_acc,bias_gyro)

% ����λ�ø��µ���ģ�͡�Ϊ�˼�С���������Ժ���Խ�����һ�εĸ���Ƶ��
[Rmeri,Rprim,ge] = CalEarthModel(pos);

% ���ݲ���������ȷ���Ƿ񴫸���������ƫ
%ע�ⷽ����ߵ���ʱ���Ǽ���ƫ���������˲�������ʱ������ƫ=����ƫ-X��
if(nargin == 8)
    acc1  = acc  + bias_acc;
    gyro1 = gyro + bias_gyro;
else
    acc1  = acc;
    gyro1 = gyro;
end

%һ��������̬
wien=twe(pos);
wenn=twv(pos,speed,Rmeri,Rprim);
Cbn=cbn(atti);
wnbb=gyro1-Cbn'*(wien+wenn);%�۳�������ת���۳��ٶ�����Ľ��ٶ�֮����bϵ��ת�����ٶȡ�
% ��̬����
%atti = QuatUpdate(atti,wnbb*dTins);


%���������ٶ�
accn=Cbn*acc1;%������������Ա��ڿ������˲��Ĳ���ʹ��

gn=[0;0;-ge];
an=accn-cross(wien+wien+wenn,speed)+gn;
speed=speed+dTins*an;%�����ٶ�

%��������λ��
dpos=zeros(3,1);

H=pos(3);
dpos(1)=speed(2)/(Rmeri+H);%�����ٶȵõ�γ��
dpos(2)=speed(1)/((Rprim+H)*cos(pos(1)));%�����ٶȵõ�����
dpos(3)=speed(3);
pos=pos+dTins*dpos;%����λ��




