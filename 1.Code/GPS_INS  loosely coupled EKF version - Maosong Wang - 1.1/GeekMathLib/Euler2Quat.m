% -----------------------------------------------------------------------------
% Function    : Euler2Quat 欧拉角转换四元数
% Description : phi : 绕X轴旋转量   theta : 绕Y轴旋转量  psi : 绕Z轴旋转量
%               coor : 支持ZYX、ZXY等旋转顺序
% Author      : logzhan
% Date        : 2023-01-07
% -----------------------------------------------------------------------------
function q = Euler2Quat(phi, theta, psi, coor)
q=[1,0,0,0]';

%% 实现方式1
if(strcmp(coor,'ZYX'))
    q=[1,0,0,0]';
    q = QuatUpdate(q,[0,0,psi]);
    q = QuatUpdate(q,[0,theta,0]);
    q = QuatUpdate(q,[phi,0,0]);
elseif(strcmp(coor,'ENU') || strcmp(coor,'ZXY'))
    q=[1,0,0,0]';
    q = QuatUpdate(q,[0,0,psi]);
    q = QuatUpdate(q,[phi,0,0]);
    q = QuatUpdate(q,[0,theta,0]);  
end

%% 按照定义法实现
    
end
