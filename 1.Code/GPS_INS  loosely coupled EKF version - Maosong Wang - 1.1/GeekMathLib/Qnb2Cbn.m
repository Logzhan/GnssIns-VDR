% -----------------------------------------------------------------------------
% Function    : Qnb2dcm 四元数转方向余弦矩阵(旋转矩阵)
% Description : Qnb转Cbn,Qnb是从N系到B系的四元数。Cbn表示从B系到N系转换的旋转矩阵
%               四元数到旋转矩阵的转换和欧拉角的旋转顺序无关。是通用的转换函数
% Author      : logzhan
% Date        : 2023-01-05
% -----------------------------------------------------------------------------
function Cbn = Qnb2Cbn(Qnb) 
Cbn=[Qnb(1)^2+Qnb(2)^2-Qnb(3)^2-Qnb(4)^2    2*(Qnb(2)*Qnb(3)-Qnb(1)*Qnb(4))       2*(Qnb(2)*Qnb(4)+Qnb(1)*Qnb(3))
     2*(Qnb(2)*Qnb(3)+Qnb(1)*Qnb(4))        Qnb(1)^2-Qnb(2)^2+Qnb(3)^2-Qnb(4)^2   2*(Qnb(3)*Qnb(4)-Qnb(1)*Qnb(2))
     2*(Qnb(2)*Qnb(4)-Qnb(1)*Qnb(3))        2*(Qnb(3)*Qnb(4)+Qnb(1)*Qnb(2))       Qnb(1)^2-Qnb(2)^2-Qnb(3)^2+Qnb(4)^2];
end