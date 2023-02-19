function [q] = QuatUpdate(q,TV)

THM = [0,    -TV(1), -TV(2), -TV(3);
       TV(1), 0,      TV(3), -TV(2);
       TV(2),-TV(3),  0,      TV(1);
       TV(3), TV(2), -TV(1),  0    ];
   
A = eye(4)*cos(norm(TV)/2);

if(norm(TV) < (1e-6))
    A = A + 0.5 * THM;
else
    A=A+THM*(sin(norm(TV)/2)/norm(TV));
end
q = A*q;

%     % ��תʸ��������Ԫ��
%     Delta_Re = cos(norm(TV)/2);
%     Delta_Im = sin(norm(TV)/2)/norm(TV) * TV;
%     % ��תʸ��ת��Ϊ��Ԫ��
%     q_new= [Delta_Re Delta_Im']';
%     % ��Ԫ�����
%     q = QuatMult(q,q_new');
