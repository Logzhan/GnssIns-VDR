clc;
clear;
addpath(genpath('./WangMaoSong'));
addpath(genpath('./WuMo'));
addpath(genpath('./quaternion_library'));
addpath(genpath('./GeekMathLib'));

%% ENU����ϵ��
YawDeg   = 10; % ��Z��
PitchDeg = 20; % ��X��
RollDeg  = 30; % ��Y��

% �ȴ���һ����Ԫ��
q = Euler2Quat2(PitchDeg/(180/pi),RollDeg/(180/pi),YawDeg/(180/pi),'ZXY');
num = ' % 1.7f';
a = sprintf('\rZYX Euler angles to Quaternion:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num, '\t', num), q);
disp(strcat(a,b));

% ��㴴��һ����ת����
TV = [YawDeg,RollDeg,PitchDeg] ./ (180/pi);

% ��Ԫ��������ת��������1
q1 = QuatUpdate2(q,TV');
num = ' % 1.7f';
a = sprintf('\rZYX Euler angles to Quaternion:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num, '\t', num), q1);
disp(strcat(a,b));

% ��Ԫ��������ת��������2
q2 = QuatUpdate(q',TV);
num = ' % 1.7f';
a = sprintf('\rZYX Euler angles to Quaternion:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num, '\t', num), q2);
disp(strcat(a,b));



