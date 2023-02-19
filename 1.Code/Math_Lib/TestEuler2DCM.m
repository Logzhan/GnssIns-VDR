clc;
clear;
addpath(genpath('./WangMaoSong'));
addpath(genpath('./WuMo'));
addpath(genpath('./quaternion_library'));
addpath(genpath('./GeekMathLib'));
addpath(genpath('./YGM'));

% Rx = 10 / (180/pi);
% Ry = 20 / (180/pi);
% Rz = 90 / (180/pi);
% 
% vec = [1,0,0];Rx=0;Ry=0;
% cbn = eul2dcm(Rx,Ry,Rz);
% num = ' % 1.7f';
% a = sprintf('\rX-IMU Library Quaternion to Rotation Matrix:');
% b = sprintf(strcat('\r', num, '\t', num, '\t', num), cbn(1,:));
% c = sprintf(strcat('\r', num, '\t', num, '\t', num), cbn(2,:));
% d = sprintf(strcat('\r', num, '\t', num, '\t', num), cbn(3,:));
% disp(strcat(a,b,c,d));
% tvec = cbn*vec';
% a = sprintf('\rZYX Euler angles to Quaternion:');
% b = sprintf(strcat('\r', num, '\t', num, '\t', num), tvec(1),tvec(2),tvec(3));
% disp(strcat(a,b));
% 
% cbn1 = euler2rotMat(Rx,Ry,Rz);
% num = ' % 1.7f';
% a = sprintf('\rX-IMU Library Quaternion to Rotation Matrix:');
% b = sprintf(strcat('\r', num, '\t', num, '\t', num), cbn1(1,:));
% c = sprintf(strcat('\r', num, '\t', num, '\t', num), cbn1(2,:));
% d = sprintf(strcat('\r', num, '\t', num, '\t', num), cbn1(3,:));
% disp(strcat(a,b,c,d));
% 
% tvec = cbn1*vec';
% a = sprintf('\rZYX Euler angles to Quaternion:');
% b = sprintf(strcat('\r', num, '\t', num, '\t', num), tvec(1),tvec(2),tvec(3));
% disp(strcat(a,b));
% 
% cbn = quat2dcm(Euler2Quat(Rx,Ry,Rz,'ZYX'));
% num = ' % 1.7f';
% a = sprintf('\rX-IMU Library Quaternion to Rotation Matrix:');
% b = sprintf(strcat('\r', num, '\t', num, '\t', num), cbn(1,:));
% c = sprintf(strcat('\r', num, '\t', num, '\t', num), cbn(2,:));
% d = sprintf(strcat('\r', num, '\t', num, '\t', num), cbn(3,:));
% disp(strcat(a,b,c,d));

%% 测试ZXY欧拉角(ENU)转DCM
Rx = 10 / (180/pi);
Ry = 20 / (180/pi);
Rz = 30 / (180/pi);

cbn = a2mat([Rx,Ry,Rz]);
num = ' % 1.11f';
a = sprintf('\rX-IMU Library Quaternion to Rotation Matrix:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num), cbn(1,:));
c = sprintf(strcat('\r', num, '\t', num, '\t', num), cbn(2,:));
d = sprintf(strcat('\r', num, '\t', num, '\t', num), cbn(3,:));
disp(strcat(a,b,c,d));

cbn = Euler2DCM(Rx,Ry,Rz,'ENU');
num = ' % 1.11f';
a = sprintf('\rX-IMU Library Quaternion to Rotation Matrix:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num), cbn(1,:));
c = sprintf(strcat('\r', num, '\t', num, '\t', num), cbn(2,:));
d = sprintf(strcat('\r', num, '\t', num, '\t', num), cbn(3,:));
disp(strcat(a,b,c,d));

%% 测试DCM(Cnb)转欧拉角
% 创建初始cbn
cnb = Euler2DCM(Rx,Ry,Rz,'ENU');
cnb = a2mat([Rx,Ry,Rz]);

att = m2att(cnb) * (180/pi);
num = ' % 1.7f';
a = sprintf('\rYanGongMin Cnb to att:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num), att(1),att(2),att(3));
disp(strcat(a,b));

att = DCM2Euler(cnb,'ENU')* (180/pi);
num = ' % 1.7f';
a = sprintf('\rWuMo Cnb to att:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num), att(1),att(2),att(3));
disp(strcat(a,b));



