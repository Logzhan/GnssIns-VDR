% -----------------------------------------------------------------------------
% Function    : Euler2Quat 欧拉角转换四元数
% Description : phi : 绕X轴旋转量   theta : 绕Y轴旋转量  psi : 绕Z轴旋转量
%               coor : 支持ZYX、ZXY等旋转顺序
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
