close all;
clc;
clear;
% 添加Math库目录
addpath(genpath('./Math'));
addpath(genpath('./Data'));
addpath(genpath('./GnssIns'));

disp('-------- 基于EKF的位置速度观测组合导航程序 ------------');
% 加载数据集
load IMU_Raw.mat       % 惯导原始数据
load Reference_Data.mat    % GPS测量数据

disp('Step2:初始化参数;');
%% 一些导航参数常数项
WIE   =  7.292115e-5;           % 地球自转角速度
r0    =  6378137.0;             % 地球半径
EE    =  0.0818191908426;      % 偏心率
d2r   =  pi/180;             % degree to radian
r2d   =  180/pi;             % radian to degree
dh2rs =  d2r/3600;           % deg/h to rad/s
%% 导航坐标系下初始化姿态，速度，位置
yaw   =  0*pi/180; % 航向角
pitch =  0*pi/180; % 俯仰角
roll  =  0*pi/180; % 滚动角
cbn = Euler2DCM(roll,pitch,yaw);
cnb = cbn';
q   = DCM2Quat(cbn)';

Vn=0;%北向速度
Ve=0;%东向速度
Vd=0;%地向速度
V_last=[Vn Ve Vd]';

% 初始位置
Lati = 31.4913627505302*pi/180;%纬度
Longi= 120.849577188492*pi/180;%经度
Alti = 6.6356;%高度

sampt0=1/200;%惯导系统更新时间200Hz

Rn = r0*(1-EE^2)/(1-EE^2*(sin(Lati))^2)^1.5;         %子午圈曲率半径
Re = r0/(1-EE^2*(sin(Lati))^2)^0.5;                  %卯酉圈曲率半径
% 初始化重力向量
g_u = -9.7803267711905*(1+0.00193185138639*sin(Lati)^2)...
    /((1-0.00669437999013*sin(Lati)^2)^0.5 *(1.0 + Alti/r0)^2);
g = [0 0 -g_u]';%重力
g0=9.80665;
%% 卡尔曼滤波P、Q、R设置
% P的设置
std_roll    =   (5)*d2r;
std_pitch   =   (5)*d2r;
std_yaw     =   (60)*d2r;

std_vel     =   0.1;
std_pos     =   5;

std_gyro    =   3*0.5*dh2rs;             % 陀螺随机漂移0.5度/小时
std_acc     =   3*0.15e-3*g0;            % 加表零偏0.15mg

Pfilter     =   diag([std_roll^2 std_pitch^2 std_yaw^2 std_vel^2 std_vel^2 std_vel^2 (std_pos/3600/30/57.3)^2 (std_pos/3600/30/57.3)^2 std_pos^2  std_gyro^2 std_gyro^2 std_gyro^2 std_acc^2 std_acc^2 std_acc^2]);

% Q的设置
std_Wg      =   0.15*(2.909*1e-4);       % 陀螺漂移噪声,度/根号小时转化成rad/根号秒
std_Wa      =   0.21/60/3;               % 加表漂移噪声
Qkf         =   diag([std_Wg^2 std_Wg^2 std_Wg^2 std_Wa^2 std_Wa^2 std_Wa^2]);

G			=   zeros(15, 6);
F = zeros(15);
F_i=zeros(9,9);
F_s=zeros(9,6);
H           =   zeros(6,15);
H(1:3,4:6)  =   eye(3);
H(4:6,7:9)  =   eye(3);
% R的设置
R           =   diag([std_vel^2 std_vel^2 std_vel^2 (std_pos/3600/30/57.3)^2 (std_pos/3600/30/57.3)^2 (std_pos)^2]);
%% 误差状态初始置零
Xfilter = zeros(15, 1);
data_length=1750;
Navi_result=zeros(data_length,19);
gps_count =1;
gyr = IMU_Raw(1:end,1:3); % 角增量
acc = IMU_Raw(1:end,4:6); % 比力积分增量
disp('Step3:开始计算........');

% GPS为10Hz, IMU降采样到100Hz处理
for i=1:data_length * 200 / 2
    %% 角增量和比力积分增量
    ang_1 = gyr(2*i-1,:)' * sampt0;
    ang_2 = gyr(2*i,:)'* sampt0;
    
    vel_1 = acc(2*i-1,:)'* sampt0;
    vel_2 = acc(2*i,:)'* sampt0;
    
    % Ins惯性更新
    [Vn,Ve,Vd,angle,velocity,Wien,Wenn,Winn] = InsVelUpdate(ang_1,ang_2,vel_1,vel_2, Vn, Ve, Vd, ...
                        Lati, Alti, Re, Rn, cnb, cbn, sampt0, V_last, g);
    
    V_last=[Vn Ve Vd]';
    
    %经纬高度更新
    Lati  = Lati  +  2.0*sampt0*Vn/(Rn+Alti);
    Longi = Longi +  2.0*sampt0*Ve/((Re+Alti)*cos(Lati));
    Alti  = Alti  -  2.0*sampt0*Vd;
    
    Rn = r0*(1-EE^2)/(1-EE^2*(sin(Lati))^2)^1.5;         %更新子午圈半径
    Re = r0/(1-EE^2*(sin(Lati))^2)^0.5;                  %更新横向曲率半径
    
    % 根据维度计算重力加速度
    g_u = -9.7803267711905*(1+0.00193185138639*sin(Lati)^2)...
        /((1-0.00669437999013*sin(Lati)^2)^0.5 *(1.0 + Alti/r0)^2);
    g = [0 0 -g_u]';
    
    %计算等效旋转矢量
    TV = angle + (2.0/3.0)*cross(ang_1,ang_2);
    %计算四元数
    NS = TV' * TV ;
    if   NS < 1.0e-8
        dM = [0 -TV(1) -TV(2) -TV(3)
            TV(1) 0 TV(3) -TV(2)
            TV(2) -TV(3) 0 TV(1)
            TV(3) TV(2) -TV(1) 0];
        QM = (1-NS/8.0+NS^2/384.0)*eye(4)+(0.5-NS/48.0)*dM;
        q = QM* q;
        q = q/norm(q);
    else
        Delta_Re = cos(norm(TV)/2);
        Delta_Im = sin(norm(TV)/2)/norm(TV) * TV;
        q_new= [Delta_Re Delta_Im']';
        Q_matrix = [q(1)     -q(2)     -q(3)      -q(4)
                    q(2)      q(1)     -q(4)       q(3)
                    q(3)      q(4)      q(1)      -q(2)
                    q(4)     -q(3)      q(2)       q(1)];
        q = Q_matrix* q_new;
        q = q/norm(q);
    end
    
    %四元数→DCM
    cbn = Quat2DCM(q);
    cnb = cbn';
    
    %% GNSS更新
    if(mod(i,10) == 0)
        Discret_T=0.1;
        %Discret_T=1;
        %构建Kalman滤波器系统阵15*15

        % vel_1 = acc(2*i-1,:)'* sampt0;
        % vel_2 = acc(2*i,:)'* sampt0;
        % velocity = vel_1 + vel_2
        Fn = cbn*( velocity / 2.0) / sampt0;

        Fer=[-WIE*sin(Lati)                                      0   -Ve/((Re+Alti)*(Re+Alti));
              0                                                  0    Vn/((Rn+Alti)*(Rn+Alti));
             -WIE*cos(Lati)-Ve/((Re+Alti)*cos(Lati)*cos(Lati))   0    Ve*tan(Lati)/((Re+Alti)*(Re+Alti))];

        Fev=[0             1.0/(Re+Alti)            0;
            -1.0/(Rn+Alti) 0                        0;
            0              -tan(Lati)/(Re+Alti)     0];

        Frr=[0                                  0      -Vn/((Rn+Alti)*(Rn+Alti));
             Ve*tan(Lati)/((Re+Alti)*cos(Lati)) 0      -Ve/((Re+Alti)*(Re+Alti)*cos(Lati));
             0                                  0       0];

        Frv=[1/(Rn+Alti)   0                         0;
             0             1/((Re+Alti)*cos(Lati))   0;
             0             0                        -1];
         
        TwoWien_Winn = 2*Wien + Wenn;

        DetaWr=[-WIE*sin(Lati)   0   0;
                 0               0   0;
                -WIE*cos(Lati)   0   0];
        
        % 组建状态转移矩阵
        F(1:3,1:3)=-SkewSymmetric(Winn);
        F(1:3,4:6)=Fev;
        F(1:3,7:9)=Fer;
        F(1:3,10:12)=-cbn;
        
        F(4:6,1:3) = SkewSymmetric(Fn);
        F(4:6,4:6) =-SkewSymmetric(TwoWien_Winn)  + SkewSymmetric(V_last)*Fev;
        F(4:6,7:9) = SkewSymmetric(V_last)*DetaWr + SkewSymmetric(V_last)*Fer;
        F(4:6,13:15)=cbn;
        
        F(7:9,4:6)=Frv;
        F(7:9,7:9)=Frr;
        
        G=[-cbn        zeros(3,3)
            zeros(3,3) cbn
            zeros(9,6)          ];     
        %% 矩阵离散化
        discreteF = eye(15);
        temp      = 1;
        
        for ii = 1 : 10
            temp = temp * ii;
            discreteF = discreteF + (Discret_T*F) ^ ii / temp;
        end
        
        M1  = G * Qkf * G'*Discret_T;
        discreteQ=M1;
        % P = FPF + Q
        Pfilter = discreteF * Pfilter * discreteF' + discreteQ;
        % Xkp = F * Xk
        Xfilter = discreteF * Xfilter;
    end
    
    if (mod(i,100)==0)
        %% 与GPS结果松组合位置速度观测
        Z(1,1) = Vn- Reference_data(gps_count+1,5);
        Z(2,1) = Ve- Reference_data(gps_count+1,6);
        Z(3,1) = Vd- Reference_data(gps_count+1,7);
        Z(4,1) = Lati - Reference_data(gps_count+1,2)*pi/180;
        Z(5,1) = Longi- Reference_data(gps_count+1,3)*pi/180;
        Z(6,1) = Alti - Reference_data(gps_count+1,4);
                
        K       = Pfilter * H'/(H * Pfilter * H' + R);
        Xfilter = Xfilter + K * (Z - H * Xfilter);
        Pfilter = (eye(15, 15) - K * H) * Pfilter * (eye(15, 15) - K * H)' + K * R * K';
        
        % 北东地速度更新
        Vn = Vn - Xfilter(4);
        Ve = Ve - Xfilter(5);
        Vd = Vd - Xfilter(6);
        
        V_last=[Vn Ve Vd]';
        % 经纬高更新
        Lati  = Lati - Xfilter(7);
        Longi = Longi -Xfilter(8);
        Alti  = Alti - Xfilter(9);
        %% 四元数校正与更新
        deta_q = [cos(norm(Xfilter(1:3))/2) (sin(norm(Xfilter(1:3))/2)/norm(Xfilter(1:3)) * Xfilter(1:3))']';
        % 四元数更新
        q   = QuatMul(deta_q,q);
        
        % 更新旋转矩阵
        cbn = Quat2DCM(q);
        cnb = cbn';
        %%  把除了常值漂移之外的误差归零
        Xfilter(1:9,1)=zeros(9,1);
        
        %%  保存结果
        Navi_result(gps_count,1) = gps_count;
        Navi_result(gps_count,2)=Vn- Reference_data(gps_count+1,5) ;
        Navi_result(gps_count,3)=Ve- Reference_data(gps_count+1,6) ;
        Navi_result(gps_count,4)=Vd- Reference_data(gps_count+1,7) ;
        Navi_result(gps_count,5)=(Lati*r2d-Reference_data(gps_count+1,2))*3600*30;
        Navi_result(gps_count,6)=(Longi*r2d-Reference_data(gps_count+1,3))*3600*30*cos(Reference_data(gps_count+1,2)*d2r);
        Navi_result(gps_count,7)=Alti-Reference_data(gps_count+1,4) ;
        Navi_result(gps_count,8) =atan2(cbn(3,2), cbn(3,3))*180/pi; % Roll
        Navi_result(gps_count,9) =asin(-cbn(3,1))*180/pi;           % Pitch
        Navi_result(gps_count,10)=atan2(cbn(2,1),cbn(1,1))*180/pi;  % Yaw
        Navi_result(gps_count,11:13)  =   Xfilter(10:12)'*180/pi;
        Navi_result(gps_count,14:16)  =   Xfilter(13:15)';
        gps_count = gps_count + 1;
        
    end
end
figure(1);
set(gcf,'unit','centimeters','position',[1 1 12.9 9]);
subplot(3,1,1)
plot(Navi_result(1:data_length,1)*1,Navi_result(1:data_length,8),'r','LineWidth',1);
hold on;
grid on
xlabel('Time (s)');ylabel('Roll (deg)');
title('EKF Attitudes')

subplot(3,1,2)
plot(Navi_result(1:data_length,1)*1,Navi_result(1:data_length,9),'g','LineWidth',1);
hold on;
grid on
xlabel('Time (s)');ylabel('Pitch (deg)');
subplot(3,1,3)
plot(Navi_result(1:data_length,1)*1,Navi_result(1:data_length,10),'b','LineWidth',1);
hold on;
grid on
xlabel('Time (s)');ylabel('Yaw (deg)');
% 位置误差
figure(2);
set(gcf,'unit','centimeters','position',[1 1 12.9 9]);
plot(Navi_result(1:data_length,1)*1,Navi_result(1:data_length,5),'r','LineWidth',1);
hold on;
plot(Navi_result(1:data_length,1)*1,Navi_result(1:data_length,6),'g','LineWidth',1);
hold on;
plot(Navi_result(1:data_length,1)*1,Navi_result(1:data_length,7),'b','LineWidth',1);
hold on;
xlabel('Time (s)');ylabel('Positions errors (m)')
legend('North','East','Down')
grid on;
title('EKF Position Errors');
% 速度误差
figure(3);
set(gcf,'unit','centimeters','position',[1 1 12.9 9]);
plot(Navi_result(1:data_length,1)*1,Navi_result(1:data_length,2),'r','LineWidth',1);
hold on;
plot(Navi_result(1:data_length,1)*1,Navi_result(1:data_length,3),'g','LineWidth',1);
hold on;
plot(Navi_result(1:data_length,1)*1,Navi_result(1:data_length,4),'b','LineWidth',1);
hold on;
xlabel('Time (s)');ylabel('Velocity errors (m/s)')
legend('Vn','Ve','Vd')
grid on;
title('EKF Velocity Errors');
% 陀螺漂移
figure(4);
set(gcf,'unit','centimeters','position',[1 1 12.9 9]);
subplot(3,1,1)
plot(Navi_result(1:data_length,1)*1,Navi_result(1:data_length,11),'r','LineWidth',1);
hold on;
grid on
xlabel('Time (s)');ylabel('X (deg/s)');
title('EKF Gyro Constant Biases');

subplot(3,1,2)
plot(Navi_result(1:data_length,1)*1,Navi_result(1:data_length,12),'g','LineWidth',1);
hold on;
grid on
xlabel('Time (s)');ylabel('Y (deg/s)');
subplot(3,1,3)
plot(Navi_result(1:data_length,1)*1,Navi_result(1:data_length,13),'b','LineWidth',1);
hold on;
grid on
xlabel('Time (s)');ylabel('Z (deg/s)');
% 加表漂移
figure(5);
set(gcf,'unit','centimeters','position',[1 1 12.9 9]);
subplot(3,1,1)
plot(Navi_result(1:data_length,1)*1,Navi_result(1:data_length,14),'r','LineWidth',1);
hold on;
grid on
xlabel('Time (s)');ylabel('X (m/s^2)');
title('EKF Accelerometer Constant Biases');

subplot(3,1,2)
plot(Navi_result(1:data_length,1)*1,Navi_result(1:data_length,15),'g','LineWidth',1);
hold on;
grid on
xlabel('Time (s)');ylabel('Y (m/s^2)');
subplot(3,1,3)
plot(Navi_result(1:data_length,1)*1,Navi_result(1:data_length,16),'b','LineWidth',1);
hold on;
grid on
xlabel('Time (s)');ylabel('Z (m/s^2)');