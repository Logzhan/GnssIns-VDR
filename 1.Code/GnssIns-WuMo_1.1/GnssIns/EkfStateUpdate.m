% -----------------------------------------------------------------------------
% Function    : KfStateUpdate 卡尔曼状态更新
% Description : 卡尔曼滤波，处理卫星数据，开环、不反馈
% Author      : logzhan
% Date        : 2023-01-08
% -----------------------------------------------------------------------------
function [X,Pk,Phi] = EkfStateUpdate(Z,H,Pk,Q,R,Phi)

Pkk=Phi*Pk*(Phi')+Q;

K=Pkk*(H')/(H*Pkk*(H')+R);
X=K*Z;

IKH=eye(15)-K*H;
Pk=IKH*Pkk*(IKH')+K*R*(K');

Phi=eye(15);


%组合导航后续处理，参考下面

% tpos=tpos-X.submat(0,0,3,1);
% tspeed=tspeed-X.submat(3,0,3,1);
% qattitude.update((~qattitude.Cbn())*X.submat(6,0,3,1));
% biasgyro=biasgyro-X.submat(9,0,3,1);
% biasacc=biasacc-X.submat(12,0,3,1);





