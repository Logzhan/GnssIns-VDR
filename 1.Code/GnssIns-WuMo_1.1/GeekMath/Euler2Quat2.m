% -----------------------------------------------------------------------------
% Function    : Euler2Quat ŷ����ת����Ԫ��
% Description : phi : ��X����ת��   theta : ��Y����ת��  psi : ��Z����ת��
%               coor : ֧��ZYX��ZXY����ת˳��
% Author      : logzhan
% Date        : 2023-01-07
% -----------------------------------------------------------------------------
function q = Euler2Quat2(phi, theta, psi, coor)
q = [1,0,0,0];
qx = VecAngle2Quat([1,0,0],phi);
qy = VecAngle2Quat([0,1,0],theta);
qz = VecAngle2Quat([0,0,1],psi);

if(strcmp(coor,'ZYX'))
    q = QuatMult(QuatMult(qz,qy),qx);
elseif(strcmp(coor,'ZXY'))
    q = QuatMult(QuatMult(qz,qx),qy);
end
