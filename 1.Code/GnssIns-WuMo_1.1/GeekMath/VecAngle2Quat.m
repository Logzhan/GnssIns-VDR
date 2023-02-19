% -----------------------------------------------------------------------------
% Function    : RotVecQuat ��ת����ת��Ԫ��
% Description : ������Ԫ���˷��������
% Author      : logzhan
% Date        : 2023-01-05
% Reference   : ������Ԫ���Ķ���, q = [cos(theta/2),Vec*sin(theta/2)]
% -----------------------------------------------------------------------------
function [ q ] = VecAngle2Quat(Vec, angle)
    q0 = cos(angle./2);
    q1 = Vec(1)*sin(angle./2);
    q2 = Vec(2)*sin(angle./2);
    q3 = Vec(3)*sin(angle./2);
    q = [q0 q1 q2 q3];
end

