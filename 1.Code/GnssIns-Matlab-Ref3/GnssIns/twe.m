function wiet=twe(pos)

we=7.292115e-5;
latitude=pos(1);
wiet=zeros(3,1);
wiet(2)=cos(latitude)*we;%±±
wiet(3)=sin(latitude)*we;%Ìì

