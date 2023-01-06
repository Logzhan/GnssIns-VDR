close all;clear;clc;
load gps_data.mat
GPS=gps_data;
GPSt1=fix(GPS(:,7)./10000).*3600;
GPSt2=mod(GPS(:,7),10000);
GPSt3=fix(GPSt2./100).*60;
GPSt4=mod(GPSt2,100);
GPSTIME=GPSt1+GPSt3+GPSt4;
GPS_data(:,1)=GPSTIME;
GPS_data(:,2)=GPS(:,1)*180/pi;
GPS_data(:,3)=GPS(:,2)*180/pi;
GPS_data(:,4)=GPS(:,3);
GPS_data(:,5)=GPS(:,4);
GPS_data(:,6)=GPS(:,5);
GPS_data(:,7)=GPS(:,6);

Reference_data=GPS_data(1:10:end,:);

figure(1);
set(gcf,'unit','centimeters','position',[1 1 12.9 9]);
plot(Reference_data(1:end,2),Reference_data(1:end,3),'b','LineWidth',1);
hold on;
grid on

xlabel('Î³¶È (deg)');ylabel('¾­¶È (deg)');
title('GPS ¹ì¼£');