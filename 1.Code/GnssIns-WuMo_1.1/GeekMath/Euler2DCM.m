% -----------------------------------------------------------------------------
% Function    : Euler2DCM 
% Description : ŷ����ת�������Ҿ���
% Author      : logzhan
% Date        : 2023-01-10 01:14
% Reference   : �����ߵ��㷨����ϵ���ԭ����.�Ϲ���, P249, ��̬��ת����̬��               
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
    % �ο������ߵ��㷨����ϵ���ԭ���壬P249����̬��ת����̬��2023-01-10
    % 01:16 У�Ժ��鼮�ṩ������һ�£����д���д���Ż����ظ������sin��cos
    R(1,1) = cos(psi).*cos(theta) - sin(psi).*sin(phi).*sin(theta);
    R(1,2) = -sin(psi).*cos(phi);
    R(1,3) = cos(phi).*sin(theta) + sin(psi).*sin(phi).*cos(theta);
    
    R(2,1) = sin(psi).*cos(theta) + cos(psi).*sin(phi).*sin(theta);
    R(2,2) = cos(psi).*cos(phi);
    R(2,3) = sin(psi).*sin(theta) - cos(psi).*sin(phi).*cos(theta);
    
    R(3,1) = -cos(phi).*sin(theta);
    R(3,2) = sin(phi);
    R(3,3) = cos(phi).*cos(theta);
    % ���ص�R��Cnb
end
end

