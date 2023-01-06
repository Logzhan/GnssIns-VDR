function F=getfk(atti,speed,pos,accn)
%���X��˳����Ϊ��γ���ߣ��������ٶȣ���������̬��������������ƫ��������ٶȼ���ƫ��
%γ����Ҫ�Ի���Ϊ��λ��
%20220707������һ��


we=7.292115e-5;

[Rmeri,Rprim,~]=earthmodelupdate(pos);

F=zeros(15,15);
RpH1=1.0/(Rprim+pos(3));%1/(î��Ȧ+�ߣ�
RmH1=1.0/(Rmeri+pos(3));%1/(����Ȧ+�ߣ�
cosphi=cos(pos(1));
secphi=1.0/cosphi;
sinphi=sin(pos(1));
tanphi=tan(pos(1));

vE=speed(1);
vN=speed(2);
vU=speed(3);



%λ�ö�λ��Ӱ����Ӿ���
Fpp=[0,                     0,  RmH1*RmH1*(-vN);
    RpH1*vE*secphi*tanphi,  0,  RpH1*RpH1*(-vE)*secphi;
    0,                      0,  0];
F(1:3,1:3)=Fpp;


%�ٶȶ�λ��Ӱ����Ӿ���
Fvp=[0,             RmH1,   0;
    RpH1*secphi,    0,      0;
    0,              0,      1];
F(1:3,4:6)=Fvp;
	

%λ�ö��ٶ�Ӱ����Ӿ���
Fpv=[2*we*cosphi*vN+2*we*sinphi*vU+vN*vE*RpH1*secphi*secphi,    0,  RpH1*RpH1*(vE*vU-vN*vE*tanphi);
    -(2*vE*we*cosphi+vE*vE*RpH1*secphi*secphi),                    0,  RmH1*RmH1*vN*vU+RpH1*RpH1*vE*vE*tanphi;
    (-2.0)*vE*we*sinphi,                                        0,  (-RmH1*RmH1*vN*vN-RpH1*RpH1*vE*vE)];
F(4:6,1:3)=Fpv;

%�ٶȶ��ٶ�Ӱ����Ӿ���
Fvv=[(vN*tanphi-vU)*RpH1,                   2.0*we*sinphi+vE*RpH1*tanphi,   (-2.0)*we*cosphi-vE*RpH1;
    (-2.0)*(we*sinphi+vE*RpH1*tanphi),      (-RmH1)*vU,                     (-RmH1)*vN;
    2.0*(we*cosphi+vE*RpH1),                2*vN*RmH1,                      0];
F(4:6,4:6)=Fvv;

%��̬���ٶ�Ӱ����Ӿ���
fE=accn(1);
fN=accn(2);
fU=accn(3);

Fav=[0,     -fU,    fN;
    fU,     0,      -fE;
    -fN,    fE,     0];
F(4:6,7:9)=Fav;


%λ�ö���̬Ӱ����Ӿ���
Fpa=[0,                                 0,  vN*RmH1*RmH1;
    (-we)*sinphi,                       0,  (-vE)*RpH1*RpH1;
    we*cosphi+vE*RpH1*secphi*secphi,    0,  (-vE)*tanphi*RpH1*RpH1];
F(7:9,1:3)=Fpa;


%�ٶȶ���̬Ӱ����Ӿ���
Fva=[0,             (-RmH1),    0;
    RpH1,           0,          0;
    RpH1*tanphi,    0,          0];
F(7:9,4:6)=Fva;

%��̬����̬Ӱ����Ӿ���
Faa=[0,                             we*sinphi+vE*RpH1*tanphi,   -(we*cosphi+vE*RpH1);
    -(we*sinphi+vE*RpH1*tanphi),    0,                          -(vN*RmH1);
    we*cosphi+vE*RpH1,              vN*RmH1,                    0];
F(7:9,7:9)=Faa;


cbnm=Qnb2Cbn(atti);
F(7:9,10:12)=(-cbnm);%��������ƫ����̬��Ӱ�졣
F(4:6,13:15)=cbnm;%���ٶȼ���ƫ���ٶȵ�Ӱ�졣




