clc;
clear;
addpath(genpath('./WangMaoSong'));
addpath(genpath('./WuMo'));
addpath(genpath('./quaternion_library'));
addpath(genpath('./GeekMathLib'));

%% ENU坐标系下
YawDeg   = 10; % 绕Z轴
PitchDeg = 20; % 绕X轴
RollDeg  = 30; % 绕Y轴

% 先创建一个四元数
q = Euler2Quat2(PitchDeg/(180/pi),RollDeg/(180/pi),YawDeg/(180/pi),'ZXY');
num = ' % 1.7f';
a = sprintf('\rZYX Euler angles to Quaternion:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num, '\t', num), q);
disp(strcat(a,b));

% 随便创建一个旋转向量
TV = [YawDeg,RollDeg,PitchDeg] ./ (180/pi);

% 四元数更新旋转向量方法1
q1 = QuatUpdate2(q,TV');
num = ' % 1.7f';
a = sprintf('\rZYX Euler angles to Quaternion:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num, '\t', num), q1);
disp(strcat(a,b));

% 四元数更新旋转向量方法2
q2 = QuatUpdate(q',TV);
num = ' % 1.7f';
a = sprintf('\rZYX Euler angles to Quaternion:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num, '\t', num), q2);
disp(strcat(a,b));



