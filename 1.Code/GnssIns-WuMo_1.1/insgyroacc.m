function [atti,speed,pos,accn]=insgyroacc(gyro,acc,atti,speed,pos,dTins,biasacc,biasgyro)
%�ߵ�����
%������ٶȡ����ٶ�


[Rmeri,Rprim,ge]=earthmodelupdate(pos);%����λ�ø��µ���ģ�͡�Ϊ�˼�С���������Ժ���Խ�����һ�εĸ���Ƶ�ʡ�

%׼����������������
%ע�ⷽ����ߵ���ʱ���Ǽ���ƫ���������˲�������ʱ������ƫ=����ƫ-X��
if(nargin==8)
    acc1=acc+biasacc;
    gyro1=gyro+biasgyro;
else
    acc1=acc;
    gyro1=gyro;
end




%һ��������̬
wien=twe(pos);
wenn=twv(pos,speed,Rmeri,Rprim);
Cbn=Qnb2Cbn(atti);
wnbb=gyro1-Cbn'*(wien+wenn);%�۳�������ת���۳��ٶ�����Ľ��ٶ�֮����bϵ��ת�����ٶȡ�
atti=QuatUpdate(atti,wnbb*dTins);%������̬


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




