% Description : 误差X的顺序定义为 ：纬经高，东北天速度，东北天姿态，三轴陀螺仪零偏，三轴加速度计零偏
%               纬经需要以弧度为单位。
% Input       : Pos(1): 纬度，Pos(2): 经度 
% Date        : 2023-01-05
% Author      : logzhan,刘天一(20220707)
function F = GetFk(Qnb,Vel,Pos,AccN)

we=7.292115e-5;

[Rmeri,Rprim,~] = CalEarthModel(Pos);

F=zeros(15,15);
% 1 / (卯酉圈+高）
RpH1=1.0/(Rprim + Pos(3));
% 1 / (子午圈+高）
RmH1=1.0/(Rmeri + Pos(3));

cosphi=cos(Pos(1));
secphi=1.0/cosphi;
sinphi=sin(Pos(1));
tanphi=tan(Pos(1));

vE = Vel(1);
vN = Vel(2);
vU = Vel(3);

% 位置对位置影响的子矩阵
Fpp=[0,                      0,  RmH1*RmH1*(-vN);
     RpH1*vE*secphi*tanphi,  0,  RpH1*RpH1*(-vE)*secphi;
     0,                      0,  0];
F(1:3,1:3)=Fpp;


% 速度对位置影响的子矩阵
Fvp=[0,             RmH1,   0;
    RpH1*secphi,    0,      0;
    0,              0,      1];
F(1:3,4:6)=Fvp;
	
% 位置对速度影响的子矩阵
Fpv=[2*we*cosphi*vN+2*we*sinphi*vU+vN*vE*RpH1*secphi*secphi,    0,  RpH1*RpH1*(vE*vU-vN*vE*tanphi);
    -(2*vE*we*cosphi+vE*vE*RpH1*secphi*secphi),                 0,  RmH1*RmH1*vN*vU+RpH1*RpH1*vE*vE*tanphi;
    (-2.0)*vE*we*sinphi,                                        0,  (-RmH1*RmH1*vN*vN-RpH1*RpH1*vE*vE)];
F(4:6,1:3)=Fpv;

% 速度对速度影响的子矩阵
Fvv=[(vN*tanphi-vU)*RpH1,                    2.0*we*sinphi+vE*RpH1*tanphi,   (-2.0)*we*cosphi-vE*RpH1;
    (-2.0)*(we*sinphi+vE*RpH1*tanphi),      (-RmH1)*vU,                      (-RmH1)*vN;
     2.0*(we*cosphi+vE*RpH1),                2*vN*RmH1,                      0];
F(4:6,4:6)=Fvv;

% 姿态对速度影响的子矩阵
fE = AccN(1);
fN = AccN(2);
fU = AccN(3);

Fav=[ 0,   -fU,    fN;
     fU,     0,   -fE;
    -fN,    fE,     0];

F(4:6,7:9)=Fav;


% 位置对姿态影响的子矩阵
Fpa=[0,                                 0,  vN*RmH1*RmH1;
    (-we)*sinphi,                       0,  (-vE)*RpH1*RpH1;
    we*cosphi+vE*RpH1*secphi*secphi,    0,  (-vE)*tanphi*RpH1*RpH1];
F(7:9,1:3)=Fpa;


% 速度对姿态影响的子矩阵
Fva=[0,             (-RmH1),    0;
    RpH1,           0,          0;
    RpH1*tanphi,    0,          0];
F(7:9,4:6)=Fva;

% 姿态对姿态影响的子矩阵
Faa=[0,                             we*sinphi+vE*RpH1*tanphi,   -(we*cosphi+vE*RpH1);
    -(we*sinphi+vE*RpH1*tanphi),    0,                          -(vN*RmH1);
    we*cosphi+vE*RpH1,              vN*RmH1,                    0];
F(7:9,7:9)=Faa;


cbnm = Quat2DCM(Qnb);

% 陀螺仪零偏对姿态的影响
F(7:9,10:12) = (-cbnm);

% 加速度计零偏对速度的影响
F(4:6,13:15) = cbnm;




