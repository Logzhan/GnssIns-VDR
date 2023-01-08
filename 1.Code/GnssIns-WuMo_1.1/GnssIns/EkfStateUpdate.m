% -----------------------------------------------------------------------------
% Function    : KfStateUpdate ������״̬����
% Description : �������˲��������������ݣ�������������
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


%��ϵ��������������ο�����

% tpos=tpos-X.submat(0,0,3,1);
% tspeed=tspeed-X.submat(3,0,3,1);
% qattitude.update((~qattitude.Cbn())*X.submat(6,0,3,1));
% biasgyro=biasgyro-X.submat(9,0,3,1);
% biasacc=biasacc-X.submat(12,0,3,1);




