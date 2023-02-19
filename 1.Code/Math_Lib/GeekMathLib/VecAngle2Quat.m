% -----------------------------------------------------------------------------
% Function    : RotVecQuat 旋转向量转四元数
% Description : 按照四元数乘法定义计算
% Author      : logzhan
% Date        : 2023-01-05
% Reference   : 按照四元数的定义, q = [cos(theta/2),Vec*sin(theta/2)]
% -----------------------------------------------------------------------------
function [ q ] = VecAngle2Quat(Vec, angle)
    q0 = cos(angle./2);
    q1 = Vec(1)*sin(angle./2);
    q2 = Vec(2)*sin(angle./2);
    q3 = Vec(3)*sin(angle./2);
    q = [q0 q1 q2 q3];
end

