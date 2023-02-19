function [ q ] = QuatUpdate2(q, TV)
NS = TV' * TV ;
if   NS<1.0e-8
    dM = [0 -TV(1) -TV(2) -TV(3)
        TV(1) 0 TV(3) -TV(2)
        TV(2) -TV(3) 0 TV(1)
        TV(3) TV(2) -TV(1) 0];
    % cos(n/2)=1-n2/8+n4/384; sin(n/2)/n=1/2-n2/48+n4/3840
    % 猜测QM是一种降低计算量的方法，因为对于sin(theta)和cos(theta)
    % 在数量级很小的时候都可以简单方式线性化，对于嵌入式而言，计算
    % sin和cos较为耗费资源
    QM = (1-NS/8.0+NS^2/384.0)*eye(4)+(0.5-NS/48.0)*dM;
    q = QM* q;
    q = q/norm(q);
else
    % 旋转矢量构建四元数
    Delta_Re = cos(norm(TV)/2);
    Delta_Im = sin(norm(TV)/2)/norm(TV) * TV;
    % 旋转矢量转换为四元数
    q_new= [Delta_Re Delta_Im']';
    %旧四元数先写为矩阵形式,方便下一步的四元数乘法
    Q_matrix = [  q(1)     -q(2)     -q(3)      -q(4)
        q(2)      q(1)     -q(4)      q(3)
        q(3)     q(4)      q(1)      -q(2)
        q(4)    -q(3)      q(2)       q(1)];
    q = Q_matrix* q_new;
    q = q/norm(q);
end
end

