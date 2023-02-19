% -----------------------------------------------------------------------------
% Function    : QuatMult 四元数转乘法运算
% Description : 按照四元数乘法定义计算
% Author      : logzhan
% Date        : 2023-01-05
% Reference   : X-IMU
% -----------------------------------------------------------------------------
function q = QuatMult2(a, b)
    p  = b(1);
    qv = b(2:4);
    quaternion_matrix_right = p*eye(4)+ [0 -qv';qv -Vec2Skew(qv)];
    q = quaternion_matrix_right*a;
    q = q/norm(q);
end

