clc;
clear;
addpath(genpath('./WangMaoSong'));
addpath(genpath('./WuMo'));
addpath(genpath('./quaternion_library'));
addpath(genpath('./GeekMathLib'));

YawDeg   = 10; % ��Z��
PitchDeg = 20; % ��Y��
RollDeg  = 30; % ��X��

% ��Ĥ��ת��
q = setoula(YawDeg, PitchDeg, RollDeg);
num = ' % 1.7f';
a = sprintf('\rZYX Euler angles to Quaternion:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num, '\t', num), q);
disp(strcat(a,b));

q = dcm2quat(eul2dcm(RollDeg/(180/pi),PitchDeg/(180/pi),YawDeg/(180/pi)));
num = ' % 1.7f';
a = sprintf('\rZYX Euler angles to Quaternion:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num, '\t', num), q);
disp(strcat(a,b));

q = rotMat2quatern(euler2rotMat(RollDeg/(180/pi), PitchDeg/(180/pi), YawDeg/(180/pi)));
num = ' % 1.7f';
a = sprintf('\rZYX Euler angles to Quaternion:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num, '\t', num), q);
disp(strcat(a,b));

%% ������Ԫ������ת����
% ����һ����Ԫ��
q = setoula(YawDeg, PitchDeg, RollDeg);

R1 = quatern2rotMat(q');
num = ' % 1.7f';
a = sprintf('\rX-IMU Library Quaternion to Rotation Matrix:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num), R1(1,:));
c = sprintf(strcat('\r', num, '\t', num, '\t', num), R1(2,:));
d = sprintf(strcat('\r', num, '\t', num, '\t', num), R1(3,:));
disp(strcat(a,b,c,d));

R2 = quat2dcm(q);
num = ' % 1.7f';
a = sprintf('\rWangMaoSong Library Quaternion to Rotation Matrix:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num), R2(1,:));
c = sprintf(strcat('\r', num, '\t', num, '\t', num), R2(2,:));
d = sprintf(strcat('\r', num, '\t', num, '\t', num), R2(3,:));
disp(strcat(a,b,c,d));


R4 = Qnb2Cbn(q);
num = ' % 1.7f';
a = sprintf('\rGeek Library Quaternion to Rotation Matrix:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num), R4(1,:));
c = sprintf(strcat('\r', num, '\t', num, '\t', num), R4(2,:));
d = sprintf(strcat('\r', num, '\t', num, '\t', num), R4(3,:));
disp(strcat(a,b,c,d));

%% ��Ԫ���˷�����
q1 = f_quat_muti(q,q);
num = ' % 1.12f';
a = sprintf('\rWangMaoSong Library Quaternion Multpuly:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num, '\t', num), q1);
disp(strcat(a,b));

q2 = quaternProd(q', q')';
num = ' % 1.12f';
a = sprintf('\rX-IMU Library Quaternion Multpuly:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num, '\t', num), q2);
disp(strcat(a,b));


