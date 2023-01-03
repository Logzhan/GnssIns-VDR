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
addpath(genpath('./quaternion_library'));
%% 数据集加载


load NmeaData.txt
load IMUData.txt
L = length(IMUData);

%% 组合导航核心参数配置
dTins=0.01;

%% 卡尔曼滤波参数
Pk1=diag([1e-12,1e-12,9,0.1,0.1,0.1,3e-4,3e-4,3e-4,0,0,0,0,0,0]');
Q0=diag([0,0,0,1e-2,1e-2,1e-2,1e-5,1e-5,1e-5,0,0,0,0,0,0]');
Phi=eye(15);

R=diag([1e-14,1e-14,1e-12,1e-12,1e-12,1e-2]');
% 生成6*15的H矩阵单阵
H=eye(15,15);
H=H(1:6,:);
% 矩阵初始化
Q=zeros(15);
Z=zeros(6,1);
X=zeros(15,1);

%% 初始值
% 顺序为航向、俯仰、横滚   
Qnb = EulerDeg2Quat(0,0,0);
% 东速、北速、天速
Vel=[0;0;0];
% 纬度、经度、高度
Pos0=[NmeaData(1,2);NmeaData(1,3);0];
Pos=Pos0;

Result=zeros(L,30);
GyroBias=zeros(3,1);
AccBias=zeros(3,1);

gnss_idx = 1;
InsInitAlign = false;
LastEnuHeading = 1000;
%% 导航
for k=1:1:L
    %Q=zeros(15);
    gyro = IMUData(k,2:4);
    acc  = IMUData(k,5:7);
    
    [Qnb,Vel,Pos,AccN] = InsUpdate(gyro',acc',Qnb,Vel,Pos,dTins,AccBias,GyroBias,InsInitAlign); 
    
    if(InsInitAlign == true)
        % EKF状态更新
        Phi = EkfStateUpdate(dTins,Qnb,Vel,Pos,AccN,Phi);
        % 更新Q矩阵
        Q = Q + Q0*dTins;
    end
    if(mod(k,100)==0 && gnss_idx < length(NmeaData))
        % 获取Gnss的位置、速度、航向信息
        GnssPos     = NmeaData(gnss_idx,2:4)';
        GnssVelMod  = NmeaData(gnss_idx,5);
        GnssHeading = NmeaData(gnss_idx,6);
        % 数据索引更新
        gnss_idx = gnss_idx + 1;
        
        if(GnssHeading ~= 1000 && GnssVelMod > 5 && InsInitAlign == false && gnss_idx > 55)
            InsInitAlign = true;
            % GPS航向角转换到ENU航向角
            EnuHeading = -GnssHeading + pi / 2;
            if(EnuHeading < 0)
                EnuHeading = EnuHeading + 2 * pi;
            end
            % 用GPS的航向角替换IMU的航向角
            Euler = Quat2EulerDeg(Qnb);
            Euler(1) = EnuHeading * (180 / pi);
            Qnb = EulerDeg2Quat(Euler(1),Euler(2),Euler(3));
            
            ve = GnssVelMod * cos(EnuHeading);
            vn = GnssVelMod * sin(EnuHeading);
            vu = 0;
            GnssVel = [ve, vn, vu]';
            Vel = GnssVel;
            Pos = GnssPos;
            LastEnuHeading = EnuHeading;
        elseif(InsInitAlign == true)
            % 从Qnb获取IMU的航向
            Euler = Quat2EulerDeg(Qnb) / (180 / pi);
            %EnuHeading = Euler(1);
            EnuHeading = LastEnuHeading;
            % 如果GPS航向角有效
            if(GnssHeading ~= 1000 && GnssVelMod > 2)
                EnuHeading = -GnssHeading + pi / 2;
                LastEnuHeading = EnuHeading;
            end
            
            if(EnuHeading < 0)
                EnuHeading = EnuHeading + 2 * pi;
            end
            
            ve = GnssVelMod * cos(EnuHeading);
            vn = GnssVelMod * sin(EnuHeading);
            vu = 0;
            GnssVel = [ve, vn, vu]';
            % 更新位置误差、速度误差
            Z = [Pos-GnssPos;Vel-GnssVel]; 
            % EKF状态修正
            [X,Pk1,Phi] = Kalman(Z,H,Pk1,Q,R,Phi);
            Q=zeros(15);
            % 位置更新
            Pos = Pos-X(1:3,1);
%             % 速度更新
            Vel = Vel-X(4:6,1);
            % 姿态更新
            Qnb = QuatUpdate(Qnb,(Quat2DCM(Qnb))'*X(7:9,1));
            % 更新陀螺仪零偏
%             GyroBias = GyroBias - X(10:12,1);
%             % 更新加速度计零偏
%             AccBias = AccBias - X(13:15,1);
%             Vel = GnssVel;
%             Pos = GnssPos;
        else
            Pos = GnssPos;
        end
    end
    %数据保存 
    Result(k,1:9)=[Quat2EulerDeg(Qnb)',Vel',Pos'];
    Result(k,10:15)=Z';
    Result(k,16:30)=X';
end

t=dTins*((1:L)'-1);

figure;
plot(NmeaData(:,3)*(180/pi), NmeaData(:,2)*(180/pi),'r');
hold on;
plot(Result(:,8)*(180/pi),Result(:,7)*(180/pi),'b.');
title('GNSS轨迹对比GPS轨迹');

figure;
subplot(3,1,1)
plot(t,Result(:,7)*180/pi,t,Result(:,7)*180/pi);
ylabel('纬度');
subplot(3,1,2)
plot(t,Result(:,8)*180/pi,t,Result(:,8)*180/pi);
ylabel('经度');
subplot(3,1,3)
plot(t,Result(:,9),t,Result(:,9));
ylabel('高度');
legend('组合导航','卫星');
xlabel('时间（s）');

figure;
plot(t,Result(:,1:3));
legend('Yaw','Pitch','Roll');

figure;
plot(t,Result(:,22:24).*(180/pi));
legend('东向姿态误差','东北姿态误差','天向姿态误差');


% 保存仿真结果到csv中
result_table = table(Result);
writetable(result_table,'GnssInsResult.csv');











