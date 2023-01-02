function wen=twv(pos,speed,Rmeri,Rprim)
%北向速度会引起东向转动；东向速度会引起北向和天向转动lat=tpos.num[0][0];

H=pos(3);
lat=pos(1);
vE=speed(1);
vN=speed(2);
wen=zeros(3,1);
if(nargin==2)
    [Rmeri,Rprim,~]=earthmodelupdate(pos);
end
wen(1)=(-vN/(Rmeri+H));%东
wen(2)=vE/(Rprim+H);%北
wen(3)=wen(2)*tan(lat);%天!!