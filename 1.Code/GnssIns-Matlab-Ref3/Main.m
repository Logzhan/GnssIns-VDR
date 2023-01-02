%%------------------------------------------------------------------------- 
% Desprition : ��ϵ����㷨
% Date       : 2022-12-31
% Author     : logzhan
%%-------------------------------------------------------------------------
clc;
clear;
close all;

%% �������ܿ����
addpath(genpath('./Math'));
addpath(genpath('./Data'));
addpath(genpath('./GnssIns'));
addpath(genpath('./Geo'));

%% ���ݼ�����
load('db.mat');
L=length(db);

%% ��ϵ������Ĳ�������
dTins=0.004;

%% �������˲�����
Pk1=diag([1e-12,1e-12,9,0.1,0.1,0.1,3e-4,3e-4,3e-4,0,0,0,0,0,0]');
Q0=diag([0,0,0,1e-4,1e-4,1e-4,1e-10,1e-10,1e-10,0,0,0,0,0,0]');
Phi=eye(15);

R=diag([1e-12,1e-12,9,0.1,0.1,0.1]');
% ����6*15��H������
H=eye(15,15);
H=H(1:6,:);
% �����ʼ��
Q=zeros(15);
Z=zeros(6,1);
X=zeros(15,1);

%% ��ʼֵ
% ˳��Ϊ���򡢸��������   
Qnb = EulerDeg2Quat(-143.26,75.69,12.2);
% ���١����١�����
Vel=[-37.8472;-50.5556;0.0694];
% γ�ȡ����ȡ��߶�
Pos0=[0.657102175971747;1.895330468487405;3765];
Pos=Pos0;

Result=zeros(L,30);

GyroBias=zeros(3,1);
AccBias=zeros(3,1);

%% ����
for k=1:1:L
    gyro = db(k,1:3)';
    acc  = db(k,4:6)';
    % ���Դ���������
    [Qnb,Vel,Pos,accn1] = InsUpdate(gyro,acc,Qnb,Vel,Pos,dTins,AccBias,GyroBias); 
    % EKF״̬����
    Phi = EkfStateUpdate(dTins,Qnb,Vel,Pos,accn1,Phi);
    % ����Q����
    Q = Q + Q0*dTins;
    
    if(mod(k,20)==0)
        gnsspos=db(k,7:9)';
        gnssspeed=db(k,10:12)';  
        
        Z = [Pos-gnsspos;Vel-gnssspeed];       
        % EKF״̬����
        [X,Pk1,Phi] = Kalman(Z,H,Pk1,Q,R,Phi);
        Q=zeros(15);
        % λ�ø���
        Pos = Pos-X(1:3,1);
        % �ٶȸ���
        Vel = Vel-X(4:6,1);
        % ��̬����
        Qnb = QuatUpdate(Qnb,(cbn(Qnb))'*X(7:9,1));
        % ������������ƫ
        GyroBias = GyroBias - X(10:12,1);
        % ���¼��ٶȼ���ƫ
        AccBias = AccBias - X(13:15,1);
    end
    
    %���ݱ��� 
    Result(k,1:9)=[Quat2EulerDeg(Qnb)',Vel',Pos'];
    Result(k,10:15)=Z';
    Result(k,16:30)=X';
end

t=dTins*((1:L)'-1);

figure
subplot(3,1,1)
plot(t,Result(:,7)*180/pi,t,db(:,7)*180/pi);
ylabel('γ��');
subplot(3,1,2)
plot(t,Result(:,8)*180/pi,t,db(:,8)*180/pi);
ylabel('����');
subplot(3,1,3)
plot(t,Result(:,9),t,db(:,9));
ylabel('�߶�');
legend('��ϵ���','����');
xlabel('ʱ�䣨s��');

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
title('�켣');


figure
subplot(3,1,1)
plot(t,Result(:,4),t,db(:,10));
ylabel('����');
subplot(3,1,2)
plot(t,Result(:,5),t,db(:,11));
ylabel('����');
subplot(3,1,3)
plot(t,Result(:,6),t,db(:,12));
ylabel('����');
legend('��ϵ���','����');
xlabel('ʱ�䣨s��');

% �����������csv��
result_table = table(Result);
writetable(result_table,'GnssInsResult.csv');











