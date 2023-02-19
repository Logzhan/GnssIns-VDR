% -----------------------------------------------------------------------------
% Function    : QuatUpdate2 四元数更新旋转矢量
% Description : q : 当前四元数   TV : 旋转矢量
% Author      : logzhan
% Date        : 2023-01-10 0:53
% Reference   : 捷联惯导算法与组合导航原理讲义.严恭敏, P250, 旋转矢量转换为
%               变换四元数
% -----------------------------------------------------------------------------
function [ q ] = QuatUpdate2(q, TV)
NS = TV' * TV ;
if   NS<1.0e-8
    dM = [0 -TV(1) -TV(2) -TV(3)
        TV(1) 0 TV(3) -TV(2)
        TV(2) -TV(3) 0 TV(1)
        TV(3) TV(2) -TV(1) 0];
    % 如果模的平方根很小，可以用泰勒展开对前几项求三角函数，降低计算量
    % cos(n/2)=1-n2/8+n4/384; sin(n/2)/n=1/2-n2/48+n4/3840
    QM = (1-NS/8.0+NS^2/384.0)*eye(4)+(0.5-NS/48.0)*dM;
    q = QM * q;
    q = q/norm(q);
else
    % 旋转矢量构建四元数
    q0 = cos(norm(TV)/2);
    Delta_Im = sin(norm(TV)/2)/norm(TV) * TV;
    q_new= [q0 Delta_Im']';
    %旧四元数先写为矩阵形式,方便下一步的四元数乘法
    Q_matrix = [  q(1)     -q(2)     -q(3)      -q(4)
        q(2)      q(1)     -q(4)      q(3)
        q(3)     q(4)      q(1)      -q(2)
        q(4)    -q(3)      q(2)       q(1)];
    % 把四元数乘法转换为矩阵相乘
    q = Q_matrix * q_new;
    q = q/norm(q);
end
end

