clc;
clear;
addpath(genpath('./WangMaoSong'));
addpath(genpath('./WuMo'));
addpath(genpath('./quaternion_library'));
addpath(genpath('./GeekMathLib'));

%% ENU×ø±êÏµÏÂ
YawDeg   = 10; % ÈÆZÖá
PitchDeg = 20; % ÈÆXÖá
RollDeg  = 30; % ÈÆYÖá

q = Euler2Quat2(PitchDeg/(180/pi),RollDeg/(180/pi),YawDeg/(180/pi),'ZXY');
num = ' % 1.7f';
a = sprintf('\rZYX Euler angles to Quaternion:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num, '\t', num), q);
disp(strcat(a,b));

q = Euler2Quat(PitchDeg/(180/pi),RollDeg/(180/pi),YawDeg/(180/pi),'ZXY');
num = ' % 1.7f';
a = sprintf('\rZYX Euler angles to Quaternion:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num, '\t', num), q);
disp(strcat(a,b));

q = setoula(YawDeg,PitchDeg,RollDeg);
num = ' % 1.7f';
a = sprintf('\rZYX Euler angles to Quaternion:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num, '\t', num), q);
disp(strcat(a,b));

q = quaternConj(rotMat2quatern(euler2rotMat(PitchDeg/(180/pi),RollDeg/(180/pi),YawDeg/(180/pi))));
num = ' % 1.7f';
a = sprintf('\rZYX Euler angles to Quaternion:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num, '\t', num), q);
disp(strcat(a,b));

q = Euler2Quat2(PitchDeg/(180/pi),RollDeg/(180/pi),YawDeg/(180/pi),'ZYX');
num = ' % 1.7f';
a = sprintf('\rZYX Euler angles to Quaternion:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num, '\t', num), q);
disp(strcat(a,b));


