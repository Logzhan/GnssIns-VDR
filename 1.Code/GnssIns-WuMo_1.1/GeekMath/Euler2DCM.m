% -----------------------------------------------------------------------------
% Function    : Euler2DCM 
% Description : 欧拉角转方向余弦矩阵
% Author      : logzhan
% Date        : 2023-01-10 01:14
% Reference   : 捷联惯导算法与组合导航原理讲义.严恭敏, P249, 姿态角转换姿态阵               
% -----------------------------------------------------------------------------
function [ R ] = Euler2DCM(phi, theta, psi, coor)
if(strcmp(coor,'NED') ||strcmp(coor,'ZYX'))
    R(1,1) = cos(psi).*cos(theta);
    R(1,2) = -sin(psi).*cos(phi) + cos(psi).*sin(theta).*sin(phi);
    R(1,3) = sin(psi).*sin(phi) + cos(psi).*sin(theta).*cos(phi);

    R(2,1) = sin(psi).*cos(theta);
    R(2,2) = cos(psi).*cos(phi) + sin(psi).*sin(theta).*sin(phi);
    R(2,3) = -cos(psi).*sin(phi) + sin(psi).*sin(theta).*cos(phi);

    R(3,1) = -sin(theta);
    R(3,2) = cos(theta).*sin(phi);
    R(3,3) = cos(theta).*cos(phi);
elseif(strcmp(coor,'ENU') || strcmp(coor,'ZXY'))
    % 参考捷联惯导算法与组合导航原理讲义，P249，姿态角转换姿态阵。2023-01-10
    % 01:16 校对和书籍提供代码结果一致，书中代码写法优化了重复计算的sin和cos
    R(1,1) = cos(psi).*cos(theta) - sin(psi).*sin(phi).*sin(theta);
    R(1,2) = -sin(psi).*cos(phi);
    R(1,3) = cos(phi).*sin(theta) + sin(psi).*sin(phi).*cos(theta);
    
    R(2,1) = sin(psi).*cos(theta) + cos(psi).*sin(phi).*sin(theta);
    R(2,2) = cos(psi).*cos(phi);
    R(2,3) = sin(psi).*sin(theta) - cos(psi).*sin(phi).*cos(theta);
    
    R(3,1) = -cos(phi).*sin(theta);
    R(3,2) = sin(phi);
    R(3,3) = cos(phi).*cos(theta);
    % 返回的R是Cnb
end
end

