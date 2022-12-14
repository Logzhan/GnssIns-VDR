% 组合导航

clear;
close all;

% 姿态和旋转数学库
addpath(genpath('./GeekMath'));
% 地球地理函数库
addpath(genpath('./GeekGeo'));
% 组合导航函数库
addpath(genpath('./GnssIns'));

%% 卡尔曼滤波参数
Pk1=diag([1e-12,1e-12,9,0.1,0.1,0.1,3e-4,3e-4,3e-4,0,0,0,0,0,0]');
Q0=diag([0,0,0,1e-4,1e-4,1e-4,1e-10,1e-10,1e-10,0,0,0,0,0,0]');
Phi1=eye(15);

% GNSS的位置精度和速度精度噪声矩阵
R = diag([1e-12,1e-12,9,0.1,0.1,0.1]');

H=eye(15,15);
H=H(1:6,:);

Q1=zeros(15);

Z1=zeros(6,1);
X1=zeros(15,1);

%% 初始值
% 初始欧拉角，ENU坐标系下定义,采用右(X)、前(Y)、上(Z)坐标系。定义绕Z为Yaw,绕
% 为Pitch,绕Y轴为Roll.
InitYaw   = -143.26;         % 绕Z轴旋转
InitPitch =  75.69;          % 绕X轴旋转
InitRoll  =  12.2;           % 绕Y轴旋转

% 按照Yaw、Pitch、Roll(ZXY)欧拉角顺序创建四元数
atti1 = Euler2Quat(InitPitch/(180/pi),InitRoll/(180/pi),InitYaw/(180/pi),'ZXY'); 

speed1=[-37.8472;-50.5556;0.0694];
pos0=[0.657102175971747;1.895330468487405;3765];
pos1=pos0;

dTins=0.004;

load('db.mat');
L=length(db);
dataA=zeros(L,30);

biasgyro=zeros(3,1);
biasacc=zeros(3,1);

%% 导航
for k=1:1:L
        
    gyro=db(k,1:3)';
    acc=db(k,4:6)';
    
    [atti1,speed1,pos1,accn1]=insgyroacc(gyro,acc,atti1,speed1,pos1,dTins,biasacc,biasgyro);    
    Phi1=stateupdate(dTins,atti1,speed1,pos1,accn1,Phi1);
    Q1=Q1+Q0*dTins;
    
    if(mod(k,20)==0)
        GnssPos = db(k,7:9)';
        GnssVel = db(k,10:12)';  
        Z1=[pos1-GnssPos;speed1-GnssVel];       
        % 卡尔曼状态更新
        [X1,Pk1,Phi1] = EkfStateUpdate(Z1,H,Pk1,Q1,R,Phi1);
        Q1=zeros(15);
        pos1=pos1-X1(1:3,1);
        speed1=speed1-X1(4:6,1);
        atti1 = QuatUpdate(atti1,(Qnb2Cbn(atti1))'*X1(7:9,1));
        biasgyro=biasgyro-X1(10:12,1);
        biasacc=biasacc-X1(13:15,1);
    end
    
    %数据保存 
    dataA(k,1:9)=[getoula(atti1)',speed1',pos1'];
    dataA(k,10:15)=Z1';
    dataA(k,16:30)=X1';
end

t=dTins*((1:L)'-1);

% 保存仿真结果到csv中
result_table = table(dataA);
writetable(result_table,'GnssInsResult.csv');

figure
subplot(3,1,1)
plot(t,dataA(:,7)*180/pi,t,db(:,7)*180/pi);
ylabel('纬度');
subplot(3,1,2)
plot(t,dataA(:,8)*180/pi,t,db(:,8)*180/pi);
ylabel('经度');
subplot(3,1,3)
plot(t,dataA(:,9),t,db(:,9));
ylabel('高度');
legend('组合导航','卫星');
xlabel('时间（s）');

figure

pya=(dataA(:,7)-pos0(1))/2/pi*40000000;
pyb=(db(:,7)-pos0(1))/2/pi*40000000;
pxa=(dataA(:,8)-pos0(2))/2/pi*40000000*cos(pos0(1));
pxb=(db(:,8)-pos0(2))/2/pi*40000000*cos(pos0(1));
pza=dataA(:,9);
pzb=db(:,9);

plot3(pxa,pya,pza);
hold on
plot3(pxb,pyb,pzb);
axis equal
title('轨迹');


figure
subplot(3,1,1)
plot(t,dataA(:,4),t,db(:,10));
ylabel('东速');
subplot(3,1,2)
plot(t,dataA(:,5),t,db(:,11));
ylabel('北速');
subplot(3,1,3)
plot(t,dataA(:,6),t,db(:,12));
ylabel('天速');
legend('组合导航','卫星');
xlabel('时间（s）');

















