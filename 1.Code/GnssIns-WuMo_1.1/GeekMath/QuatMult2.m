% -----------------------------------------------------------------------------
% Function    : QuatMult ��Ԫ��ת�˷�����
% Description : ������Ԫ���˷��������
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

