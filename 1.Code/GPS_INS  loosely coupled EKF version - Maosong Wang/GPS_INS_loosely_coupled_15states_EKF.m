%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                          ��ï��-2022-09-05
%EKF INS/GPS����ϵ�����15״̬������ϵ����NED(������)
%Copyright owned by Maosong Wang
%����wangmaosong12@nudt.edu.cn
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clear
clc
disp('***** ����EKF��λ���ٶȹ۲���ϵ������� *****');
disp('Step1:��������;');

load IMU_data200.mat       %�ߵ�ԭʼ����
load Reference_data.mat    %GPS��������

disp('Step2:��ʼ������;');
%% һЩ��������������
WIE   = 7.292115e-5;           % ������ת���ٶ�
r0    = 6378137.0;             % ����뾶
EE    =  0.0818191908426;      % ƫ����
d2r   =  pi/180;             % degree to radian
r2d   =  180/pi;             % radian to degree
dh2rs =  d2r/3600;           % deg/h to rad/s
%% ��������ϵ�³�ʼ����̬���ٶȣ�λ��
yaw   = (0)*pi/180;%�����
pitch = 0*pi/180;%������
roll  = 0*pi/180;%������
cbn=eul2dcm(roll,pitch,yaw);
cnb=cbn';
q=dcm2quat(cbn)';

Vn=0;%�����ٶ�
Ve=0;%�����ٶ�
Vd=0;%�����ٶ�
V_last=[Vn Ve Vd]';

Lati = 31.4913627505302*pi/180;%γ��
Longi= 120.849577188492*pi/180;%����
Alti = 6.6356;%�߶�

sampt0=1/200;%�ߵ�ϵͳ����ʱ��200Hz

Rn = r0*(1-EE^2)/(1-EE^2*(sin(Lati))^2)^1.5;         %����Ȧ���ʰ뾶
Re = r0/(1-EE^2*(sin(Lati))^2)^0.5;                  %î��Ȧ���ʰ뾶
g_u = -9.7803267711905*(1+0.00193185138639*sin(Lati)^2)...
    /((1-0.00669437999013*sin(Lati)^2)^0.5 *(1.0 + Alti/r0)^2);
g = [0 0 -g_u]';%����
g0=9.80665;
%% �������˲�P��Q��R����
% P������
std_roll    =   (5)*d2r;
std_pitch   =   (5)*d2r;
std_yaw     =   (60)*d2r;

std_vel     =   0.1;
std_pos     =   5;

std_gyro    =   3*0.5*dh2rs;             % �������Ư��0.5��/Сʱ
std_acc     =   3*0.15e-3*g0;            % �ӱ���ƫ0.15mg

Pfilter     =   diag([std_roll^2 std_pitch^2 std_yaw^2 std_vel^2 std_vel^2 std_vel^2 (std_pos/3600/30/57.3)^2 (std_pos/3600/30/57.3)^2 std_pos^2  std_gyro^2 std_gyro^2 std_gyro^2 std_acc^2 std_acc^2 std_acc^2]);

% Q������
std_Wg      =   0.15*(2.909*1e-4);       % ����Ư������,��/����Сʱת����rad/������
std_Wa      =   0.21/60/3;               % �ӱ�Ư������
Qkf         =   diag([std_Wg^2 std_Wg^2 std_Wg^2 std_Wa^2 std_Wa^2 std_Wa^2]);

G			=   zeros(15, 6);
F = zeros(15);
F_i=zeros(9,9);
F_s=zeros(9,6);
H           =   zeros(6,15);
H(1:3,4:6)  =   eye(3);
H(4:6,7:9)  =   eye(3);
% R������
R           =   diag([std_vel^2 std_vel^2 std_vel^2 (std_pos/3600/30/57.3)^2 (std_pos/3600/30/57.3)^2 (std_pos)^2]);
%% ���״̬��ʼ����
Xfilter = zeros(15, 1);
data_length=1750;
Navi_result=zeros(data_length,19);
gps_count =1;
d_angle= IMU_data200(1:end,1:3);% ������
d_vel  = IMU_data200(1:end,4:6);% ������������
disp('Step3:��ʼ����........');
for i=1:data_length*200/2
    %%     �������ͱ�����������
    ang_1 = d_angle(2*i-1,:)';
    ang_2 = d_angle(2*i,:)';
    
    vel_1 = d_vel(2*i-1,:)';
    vel_2 = d_vel(2*i,:)';
    
    Wien = [ WIE*cos(Lati) 0 -WIE*sin(Lati)]';
    Wenn = [Ve/(Re+Alti) -Vn/(Rn+Alti) -Ve*tan(Lati)/(Re+Alti) ]';
    
    Winn = Wien + Wenn;
    Winb = cnb * Winn;
    %˫������Чת��ʸ������
    ang_1 = ang_1 - Winb * sampt0;
    ang_2 = ang_2 - Winb * sampt0;
    angle = ang_1+ang_2;
    velocity = vel_1+vel_2;
    
    %��������
    Vel_scull_b = velocity + 0.5*cross(angle,velocity)...
        + (2.0/3.0)*(cross(vel_1,ang_2)-cross(vel_2,ang_1));
    vel_scull_n = cbn * Vel_scull_b;
    Wien2_Wenn_V = cross((2.0*Wien + Wenn),V_last);
    %�ٶȸ���
    Vel_update = V_last + vel_scull_n + 2.0*sampt0 *( g - Wien2_Wenn_V );
    Vn = Vel_update(1);
    Ve = Vel_update(2);
    Vd = Vel_update(3);
    
    V_last=[Vn Ve Vd]';
    
    %��γ�߶ȸ���
    Lati  = Lati +  2.0*sampt0*Vn/(Rn+Alti);
    Longi = Longi + 2.0*sampt0*Ve/((Re+Alti)*cos(Lati));
    Alti  = Alti - 2.0*sampt0*Vd;
    
    Rn = r0*(1-EE^2)/(1-EE^2*(sin(Lati))^2)^1.5;         %��������Ȧ�뾶
    Re = r0/(1-EE^2*(sin(Lati))^2)^0.5;                  %���º������ʰ뾶
    
    g_u = -9.7803267711905*(1+0.00193185138639*sin(Lati)^2)...
        /((1-0.00669437999013*sin(Lati)^2)^0.5 *(1.0 + Alti/r0)^2);
    g = [0 0 -g_u]';
    %�����Ч��תʸ��
    TV = angle + (2.0/3.0)*cross(ang_1,ang_2);
    %������Ԫ��
    NS = TV' * TV ;
    if   NS<1.0e-8
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
        Q_matrix = [  q(1)     -q(2)     -q(3)      -q(4)
            q(2)      q(1)     -q(4)      q(3)
            q(3)     q(4)      q(1)      -q(2)
            q(4)    -q(3)      q(2)       q(1)];
        q = Q_matrix* q_new;
        q = q/norm(q);
    end
    
    %��Ԫ����DCM
    cbn=quat2dcm(q);
    cnb = cbn';
    if(mod(i,10)==0)
        Discret_T=0.1;
        %����Kalman�˲���ϵͳ��15*15
        Fn = cbn*(velocity/2.0)/sampt0;
        Fer=[-WIE*sin(Lati) 0 -Ve/((Re+Alti)*(Re+Alti));
            0         0  Vn/((Rn+Alti)*(Rn+Alti));
            -WIE*cos(Lati)-Ve/((Re+Alti)*cos(Lati)*cos(Lati)) 0  Ve*tan(Lati)/((Re+Alti)*(Re+Alti))];
        Fev=[0 1.0/(Re+Alti) 0;
            -1.0/(Rn+Alti) 0 0;
            0 -tan(Lati)/(Re+Alti) 0];
        Frr=[0             0           -Vn/((Rn+Alti)*(Rn+Alti));
            Ve*tan(Lati)/((Re+Alti)*cos(Lati)) 0 -Ve/((Re+Alti)*(Re+Alti)*cos(Lati));
            0               0                      0];
        Frv=[1/(Rn+Alti)     0    0;
            0          1/((Re+Alti)*cos(Lati)) 0;
            0                0     -1];
        
        TwoWien_Winn= 2*Wien + Wenn;
        DetaWr=[-WIE*sin(Lati) 0 0;
            0         0 0;
            -WIE*cos(Lati) 0 0];
        
        F(1:3,1:3)=-Skew_symmetric(Winn);
        F(1:3,4:6)=Fev;
        F(1:3,7:9)=Fer;
        F(1:3,10:12)=-cbn;
        
        F(4:6,1:3)=Skew_symmetric(Fn);
        F(4:6,4:6)=-Skew_symmetric(TwoWien_Winn)+Skew_symmetric(V_last)*Fev;
        F(4:6,7:9)=Skew_symmetric(V_last)*DetaWr+Skew_symmetric(V_last)*Fer;
        F(4:6,13:15)=cbn;
        
        F(7:9,4:6)=Frv;
        F(7:9,7:9)=Frr;
        
        G=[-cbn    zeros(3,3)
            zeros(3,3) cbn
            zeros(9,6)];     
        %% ������ɢ��
        discreteF = eye(15);
        temp      = 1;
        
        for ii = 1 : 10
            temp = temp * ii;
            discreteF = discreteF + (Discret_T*F) ^ ii / temp;
        end
        M1  = G * Qkf * G'*Discret_T;
        discreteQ=M1;
        
        Pfilter = discreteF * Pfilter * discreteF' + discreteQ;
        Xfilter = discreteF * Xfilter;
    end
    
    if (mod(i,100)==0)
        %% ��GPS��������λ���ٶȹ۲�
        Z(1,1) = Vn- Reference_data(gps_count+1,5);
        Z(2,1) = Ve- Reference_data(gps_count+1,6);
        Z(3,1) = Vd- Reference_data(gps_count+1,7);
        Z(4,1) = Lati - Reference_data(gps_count+1,2)*pi/180;
        Z(5,1) = Longi- Reference_data(gps_count+1,3)*pi/180;
        Z(6,1) = Alti - Reference_data(gps_count+1,4);
                
        K       = Pfilter * H'/(H * Pfilter * H' + R);
        Xfilter = Xfilter + K * (Z - H * Xfilter);
        Pfilter = (eye(15, 15) - K * H) * Pfilter * (eye(15, 15) - K * H)' + K * R * K';
        
        Vn = Vn - Xfilter(4);
        Ve = Ve - Xfilter(5);
        Vd = Vd - Xfilter(6);
        V_last=[Vn Ve Vd]';
        Lati = Lati - Xfilter(7);
        Longi=Longi -Xfilter(8);
        Alti = Alti - Xfilter(9);
        %% ��Ԫ��У��
        deta_q=[cos(norm(Xfilter(1:3))/2) (sin(norm(Xfilter(1:3))/2)/norm(Xfilter(1:3)) * Xfilter(1:3))']';
        q=f_quat_muti(deta_q,q);
        cbn=quat2dcm(q);
        cnb = cbn';
        %%  �ѳ��˳�ֵƯ��֮���������
        Xfilter(1:9,1)=zeros(9,1);
        %%  ������
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

% �����������csv��
result_table = table(Navi_result);
writetable(result_table,'GnssInsResult.csv');

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
% λ�����
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
% �ٶ����
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
% ����Ư��
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
% �ӱ�Ư��
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