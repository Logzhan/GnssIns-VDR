function wen=twv(pos,speed,Rmeri,Rprim)
%�����ٶȻ�������ת���������ٶȻ������������ת��lat=tpos.num[0][0];

H=pos(3);
lat=pos(1);
vE=speed(1);
vN=speed(2);
wen=zeros(3,1);
if(nargin==2)
    [Rmeri,Rprim,~]=earthmodelupdate(pos);
end
wen(1)=(-vN/(Rmeri+H));%��
wen(2)=vE/(Rprim+H);%��
wen(3)=wen(2)*tan(lat);%��!!