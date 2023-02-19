function [ q ] = f_quat_muti( a,b )

% function  : mutiply two quaternion, q = a * b.
% auther    : Xian Z.W.
% data      : 2013-04-17

    p  = b(1);
    qv = b(2:4);
    quaternion_matrix_right = p*eye(4)+ [0 -qv';qv -f_skew(qv)];
    q = quaternion_matrix_right*a;
    q = q/norm(q);
% % 
% %     % p = a(1);
% %     % q = a(2:4);
% %     % quaternion_matrix_left = p*eye(4)+ [0 -q';q vector2skrewmatrix(q)];
% %     % q_1 = quaternion_matrix_left*b;
% %     % q_1 = q_1/norm(q_1);

% % % % q1 = a/norm( a );
% % % % q2 = b/norm( b );
% % % % 
% % % % a1 = q1(1); b1 = q1(2); c1 = q1(3); d1 = q1(4);
% % % % a2 = q2(1); b2 = q2(2); c2 = q2(3); d2 = q2(4);
% % % % q = [ a1*a2-b1*b2-c1*c2-d1*d2;...
% % % %       a1*b2+b1*a2+c1*d2-d1*c2;...
% % % %       a1*c2-b1*d2+c1*a2+d1*b2;...
% % % %       a1*d2+b1*c2-c1*b2+d1*a2; ];

end

