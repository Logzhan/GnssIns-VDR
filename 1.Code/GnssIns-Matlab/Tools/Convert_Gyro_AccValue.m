%% ��ֵ���ݼӱ�����
close all;
clear;
clc;

load mimu_data.mat
IMU = mimu_data;

% ����ǰ100���������ڼ�����������ƫ
averGyro=[mean(IMU(1:200*1,4)) mean(IMU(1:200*1,5)) mean(IMU(1:200*1,6))];

IMU_data(:,1) = IMU(:,4)-averGyro(1);
IMU_data(:,2) = IMU(:,5)-averGyro(2);
IMU_data(:,3) = IMU(:,6)-averGyro(3);

IMU_data(:,4) = IMU(:,1);
IMU_data(:,5) = IMU(:,2);
IMU_data(:,6) = IMU(:,3);

IMU_data200=IMU_data(2:end,:);

