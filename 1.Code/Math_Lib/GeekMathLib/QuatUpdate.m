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

%     % 旋转矢量构建四元数
%     Delta_Re = cos(norm(TV)/2);
%     Delta_Im = sin(norm(TV)/2)/norm(TV) * TV;
%     % 旋转矢量转换为四元数
%     q_new= [Delta_Re Delta_Im']';
%     % 四元数相乘
%     q = QuatMult(q,q_new');
