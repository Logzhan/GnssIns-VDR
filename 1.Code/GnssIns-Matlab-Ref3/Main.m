%%------------------------------------------------------------------------- 
% Desprition : 组合导航算法
% Date       : 2022-12-31
% Author     : logzhan
%%-------------------------------------------------------------------------
clc;
clear;
close all;

%% 函数功能库加载
addpath(genpath('./Math'));
addpath(genpath('./Data'));
addpath(genpath('./GnssIns'));
addpath(genpath('./Geo'));

%% 数据集加载
load('db.mat');
L=length(db);

%% 组合导航核心参数配置
dTins=0.004;

%% 卡尔曼滤波参数
Pk1=diag([1e-12,1e-12,9,0.1,0.1,0.1,3e-4,3e-4,3e-4,0,0,0,0,0,0]');
Q0=diag([0,0,0,1e-4,1e-4,1e-4,1e-10,1e-10,1e-10,0,0,0,0,0,0]');
Phi=eye(15);

R=diag([1e-12,1e-12,9,0.1,0.1,0.1]');
% 生成6*15的H矩阵单阵
H=eye(15,15);
H=H(1:6,:);
% 矩阵初始化
Q=zeros(15);
Z=zeros(6,1);
X=zeros(15,1);

%% 初始值
% 顺序为航向、俯仰、横滚   
Qnb = EulerDeg2Quat(-143.26,75.69,12.2);
% 东速、北速、天速
Vel=[-37.8472;-50.5556;0.0694];
% 纬度、经度、高度
Pos0=[0.657102175971747;1.895330468487405;3765];
Pos=Pos0;

Result=zeros(L,30);

GyroBias=zeros(3,1);
AccBias=zeros(3,1);

%% 导航
for k=1:1:L
    gyro = db(k,1:3)';
    acc  = db(k,4:6)';
    % 惯性传感器更新
    [Qnb,Vel,Pos,accn1] = InsUpdate(gyro,acc,Qnb,Vel,Pos,dTins,AccBias,GyroBias); 
    % EKF状态更新
    Phi = EkfStateUpdate(dTins,Qnb,Vel,Pos,accn1,Phi);
    % 更新Q矩阵
    Q = Q + Q0*dTins;
    
    if(mod(k,20)==0)
        gnsspos=db(k,7:9)';
        gnssspeed=db(k,10:12)';  
        
        Z = [Pos-gnsspos;Vel-gnssspeed];       
        % EKF状态修正
        [X,Pk1,Phi] = Kalman(Z,H,Pk1,Q,R,Phi);
        Q=zeros(15);
        % 位置更新
        Pos = Pos-X(1:3,1);
        % 速度更新
        Vel = Vel-X(4:6,1);
        % 姿态更新
        Qnb = QuatUpdate(Qnb,(cbn(Qnb))'*X(7:9,1));
        % 更新陀螺仪零偏
        GyroBias = GyroBias - X(10:12,1);
        % 更新加速度计零偏
        AccBias = AccBias - X(13:15,1);
    end
    
    %数据保存 
    Result(k,1:9)=[Quat2EulerDeg(Qnb)',Vel',Pos'];
    Result(k,10:15)=Z';
    Result(k,16:30)=X';
end

t=dTins*((1:L)'-1);

figure
subplot(3,1,1)
plot(t,Result(:,7)*180/pi,t,db(:,7)*180/pi);
ylabel('纬度');
subplot(3,1,2)
plot(t,Result(:,8)*180/pi,t,db(:,8)*180/pi);
ylabel('经度');
subplot(3,1,3)
plot(t,Result(:,9),t,db(:,9));
ylabel('高度');
legend('组合导航','卫星');
xlabel('时间（s）');

figure

pya=(Result(:,7)-Pos0(1))/2/pi*40000000;
pyb=(db(:,7)-Pos0(1))/2/pi*40000000;
pxa=(Result(:,8)-Pos0(2))/2/pi*40000000*cos(Pos0(1));
pxb=(db(:,8)-Pos0(2))/2/pi*40000000*cos(Pos0(1));
pza=Result(:,9);
pzb=db(:,9);

plot3(pxa,pya,pza);
hold on
plot3(pxb,pyb,pzb);
axis equal
title('轨迹');


figure
subplot(3,1,1)
plot(t,Result(:,4),t,db(:,10));
ylabel('东速');
subplot(3,1,2)
plot(t,Result(:,5),t,db(:,11));
ylabel('北速');
subplot(3,1,3)
plot(t,Result(:,6),t,db(:,12));
ylabel('天速');
legend('组合导航','卫星');
xlabel('时间（s）');

% 保存仿真结果到csv中
result_table = table(Result);
writetable(result_table,'GnssInsResult.csv');











