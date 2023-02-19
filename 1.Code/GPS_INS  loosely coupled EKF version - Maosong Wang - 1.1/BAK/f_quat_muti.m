function [ Q3 ] = f_quat_muti( Q1,Q2 )

% function  : mutiply two quaternion, Q3 = Q1 * Q2.
% auther    : Xian Z.W.
% data      : 2013-04-17

    p = Q2(1);
    q = Q2(2:4);
    quaternion_matrix_right = p*eye(4)+ [0 -q';q -Vec2Skew(q)];
    Q3 = quaternion_matrix_right*Q1;
    Q3 = Q3/norm(Q3);
% % 
% %     % p = Q1(1);
% %     % q = Q1(2:4);
% %     % quaternion_matrix_left = p*eye(4)+ [0 -q';q vector2skrewmatrix(q)];
% %     % Q3_1 = quaternion_matrix_left*Q2;
% %     % Q3_1 = Q3_1/norm(Q3_1);
% % % % q1 = Q1/norm( Q1 );
% % % % q2 = Q2/norm( Q2 );
% % % % 
% % % % a1 = q1(1); b1 = q1(2); c1 = q1(3); d1 = q1(4);
% % % % a2 = q2(1); b2 = q2(2); c2 = q2(3); d2 = q2(4);
% % % % Q3 = [ a1*a2-b1*b2-c1*c2-d1*d2;...
% % % %       a1*b2+b1*a2+c1*d2-d1*c2;...
% % % %       a1*c2-b1*d2+c1*a2+d1*b2;...
% % % %       a1*d2+b1*c2-c1*b2+d1*a2; ];

end

