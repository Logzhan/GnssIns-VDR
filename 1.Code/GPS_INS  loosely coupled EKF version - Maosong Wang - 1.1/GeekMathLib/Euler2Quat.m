% -----------------------------------------------------------------------------
% Function    : Euler2Quat ŷ����ת����Ԫ��
% Description : phi : ��X����ת��   theta : ��Y����ת��  psi : ��Z����ת��
%               coor : ֧��ZYX��ZXY����ת˳��
% Author      : logzhan
% Date        : 2023-01-07
% -----------------------------------------------------------------------------
function q = Euler2Quat(phi, theta, psi, coor)
q=[1,0,0,0]';

%% ʵ�ַ�ʽ1
if(strcmp(coor,'ZYX'))
    q=[1,0,0,0]';
    q = QuatUpdate(q,[0,0,psi]);
    q = QuatUpdate(q,[0,theta,0]);
    q = QuatUpdate(q,[phi,0,0]);
elseif(strcmp(coor,'ENU') || strcmp(coor,'ZXY'))
    q=[1,0,0,0]';
    q = QuatUpdate(q,[0,0,psi]);
    q = QuatUpdate(q,[phi,0,0]);
    q = QuatUpdate(q,[0,theta,0]);  
end

%% ���ն��巨ʵ��
    
end
