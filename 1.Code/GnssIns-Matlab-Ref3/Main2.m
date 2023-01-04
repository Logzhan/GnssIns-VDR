%% ------------------------------------------------------------------------- 
% Desprition : ��ϵ����㷨
% Date       : 2022-01-05
% Author     : logzhan
%% -------------------------------------------------------------------------
clc;
clear;
close all;

%% �������ܿ����
addpath(genpath('./Math'));
addpath(genpath('./Data'));
addpath(genpath('./GnssIns'));
addpath(genpath('./Geo'));
addpath(genpath('./quaternion_library'));
%% ���ݼ�����

load NmeaData.txt
load IMUData.txt
L = length(IMUData);

%% ��ϵ������Ĳ�������
% IMU�Ĳ���Ƶ��Ϊ100Hz
Fs    = 100;        
InsDt = 1/Fs;
% �Ƿ񱣴�Csv
SaveDataCsv = false;

%% �������˲�����
P0 = diag([1e-12,1e-12,9,0.1,0.1,0.1,3e-4,3e-4,3e-4,0,0,0,0,0,0]');
Q0 = diag([0,0,0,1e-2,1e-2,1e-2,1e-5,1e-5,1e-5,0,0,0,0,0,0]');
Phi= eye(15);

R=diag([1e-14,1e-14,1e-12,1e-12,1e-12,1e-2]');
% ����6*15��H������
H=eye(15,15);
H=H(1:6,:);
% �����ʼ��
Q=zeros(15);
Z=zeros(6,1);
X=zeros(15,1);
P=P0;
%% ��ʼֵ
% ˳��Ϊ���򡢸��������   
Qnb = EulerDeg2Quat(0,0,0);
% ���١����١�����
Vel=[0;0;0];
% γ�ȡ����ȡ��߶�
Pos0=[NmeaData(1,2);NmeaData(1,3);0];
Pos=Pos0;

Result=zeros(L,30);
GyroBias=zeros(3,1);
AccBias=zeros(3,1);

gnss_idx = 1;
InsInitAlign = false;
LastEnuHeading = 1000;
%% ����
for k=1:1:L
    Gyro = IMUData(k,2:4);
    Acc  = IMUData(k,5:7);
    
    [Qnb,Vel,Pos,AccN] = InsUpdate(Gyro',Acc',Qnb,Vel,Pos,InsDt,AccBias,GyroBias,InsInitAlign); 
    % ��������ɳ�ʼ��׼��������EkfStateUpdate,��ʱ��û��GPSԼ������������λ��
    % ��Ư�ƺ�Զ
    if(InsInitAlign == true)
        % EKF״̬����
        Phi = EkfStateUpdate(InsDt,Qnb,Vel,Pos,AccN,Phi);
        % ����Q����
        Q = Q + Q0*InsDt;
    end
    if(mod(k,100)==0 && gnss_idx < length(NmeaData))
        % ��ȡGnss��λ�á��ٶȡ�������Ϣ
        GnssPos     = NmeaData(gnss_idx,2:4)';
        GnssVelMod  = NmeaData(gnss_idx,5);
        GnssHeading = NmeaData(gnss_idx,6);
        % ������������
        gnss_idx = gnss_idx + 1;
        
        if(GnssHeading ~= 1000 && GnssVelMod > 5 && InsInitAlign == false && gnss_idx > 55)
            InsInitAlign = true;
            % GPS�����ת����ENU�����
            EnuHeading = -GnssHeading + pi / 2;
            if(EnuHeading < 0)
                EnuHeading = EnuHeading + 2 * pi;
            end
            % ��GPS�ĺ�����滻IMU�ĺ����
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
            % ��Qnb��ȡIMU�ĺ���
            Euler = Quat2EulerDeg(Qnb) / (180 / pi);
            %EnuHeading = Euler(1);
            EnuHeading = LastEnuHeading;
            % ���GPS�������Ч
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
            % ����λ�����ٶ����
            Z = [Pos-GnssPos;Vel-GnssVel]; 
            % EKF״̬����
            [X,P,Phi] = Kalman(Z,H,P,Q,R,Phi);
            % ���Q����
            Q=zeros(15);
            % λ�ø���
            Pos = Pos-X(1:3,1);
            % �ٶȸ���
            Vel = Vel-X(4:6,1);
            % ��̬����
            Qnb = QuatUpdate(Qnb,(Quat2DCM(Qnb))'*X(7:9,1));
            % ������������ƫ
%             GyroBias = GyroBias - X(10:12,1);
%             % ���¼��ٶȼ���ƫ
%             AccBias = AccBias - X(13:15,1);
        else
            Pos = GnssPos;
        end
    end
    %���ݱ��� 
    Result(k,1:9)=[Quat2EulerDeg(Qnb)',Vel',Pos'];
    Result(k,10:15)=Z';
    Result(k,16:30)=X';
end

t=InsDt*((1:L)'-1);

figure;
plot(NmeaData(:,3)*(180/pi), NmeaData(:,2)*(180/pi),'r');
hold on;
plot(Result(:,8)*(180/pi),Result(:,7)*(180/pi),'b.');
title('GNSS�켣�Ա�GPS�켣');

figure;
subplot(3,1,1)
plot(t,Result(:,7)*180/pi,t,Result(:,7)*180/pi);
ylabel('γ��');
subplot(3,1,2)
plot(t,Result(:,8)*180/pi,t,Result(:,8)*180/pi);
ylabel('����');
subplot(3,1,3)
plot(t,Result(:,9),t,Result(:,9));
ylabel('�߶�');
legend('��ϵ���','����');
xlabel('ʱ�䣨s��');
title('γ����');

figure;
plot(t,Result(:,1:3));
legend('Yaw','Pitch','Roll');
title('�ں���̬');

figure;
plot(t,Result(:,22:24).*(180/pi));
legend('������̬���','������̬���','������̬���');

if(SaveDataCsv == true)
    % �����������csv��
    result_table = table(Result);
    writetable(result_table,'GnssInsResult.csv');
end











