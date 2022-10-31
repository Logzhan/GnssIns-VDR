%----------------------------------------------------------------------
% Description : 四元数转旋转矩阵
% Author      : loghzan
% Date        : 2022-10-31
%----------------------------------------------------------------------
function DCM = Quat2DCM(quat) 
DCM = [
       quat(1)^2+quat(2)^2-quat(3)^2-quat(4)^2,  2*(quat(2)*quat(3)-quat(1)*quat(4)),     2*(quat(2)*quat(4)+quat(1)*quat(3));
       2*(quat(2)*quat(3)+quat(1)*quat(4)),      quat(1)^2-quat(2)^2+quat(3)^2-quat(4)^2, 2*(quat(3)*quat(4)-quat(1)*quat(2));
       2*(quat(2)*quat(4)-quat(1)*quat(3)),      2*(quat(3)*quat(4)+quat(1)*quat(2)),     quat(1)^2-quat(2)^2-quat(3)^2+quat(4)^2;
      ];
end