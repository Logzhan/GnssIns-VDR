%查看数据

clear
close all

db=load('datain.txt');
dataA=load('dataout.txt');

dTins=0.004;
L=length(db);
t=dTins*((1:L)'-1);
pos0=[0.657102175971747;1.895330468487405;3765];


figure
subplot(3,1,1)
plot(t,dataA(:,7)*180/pi,t,db(:,7)*180/pi);
ylabel('纬度');
subplot(3,1,2)
plot(t,dataA(:,8)*180/pi,t,db(:,8)*180/pi);
ylabel('经度');
subplot(3,1,3)
plot(t,dataA(:,9),t,db(:,9));
ylabel('高度');
legend('组合导航','卫星');
xlabel('时间（s）');

figure

pya=(dataA(:,7)-pos0(1))/2/pi*40000000;
pyb=(db(:,7)-pos0(1))/2/pi*40000000;
pxa=(dataA(:,8)-pos0(2))/2/pi*40000000*cos(pos0(1));
pxb=(db(:,8)-pos0(2))/2/pi*40000000*cos(pos0(1));
pza=dataA(:,9);
pzb=db(:,9);

plot3(pxa,pya,pza);
hold on
plot3(pxb,pyb,pzb);
axis equal
title('轨迹');


figure
subplot(3,1,1)
plot(t,dataA(:,4),t,db(:,10));
ylabel('东速');
subplot(3,1,2)
plot(t,dataA(:,5),t,db(:,11));
ylabel('北速');
subplot(3,1,3)
plot(t,dataA(:,6),t,db(:,12));
ylabel('天速');
legend('组合导航','卫星');
xlabel('时间（s）');

















