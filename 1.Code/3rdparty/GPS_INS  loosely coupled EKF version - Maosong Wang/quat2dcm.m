function DCM = quat2dcm(quat) 
DCM=[quat(1)^2+quat(2)^2-quat(3)^2-quat(4)^2 2*(quat(2)*quat(3)-quat(1)*quat(4)) 2*(quat(2)*quat(4)+quat(1)*quat(3))
2*(quat(2)*quat(3)+quat(1)*quat(4)) quat(1)^2-quat(2)^2+quat(3)^2-quat(4)^2 2*(quat(3)*quat(4)-quat(1)*quat(2))
2*(quat(2)*quat(4)-quat(1)*quat(3)) 2*(quat(3)*quat(4)+quat(1)*quat(2)) quat(1)^2-quat(2)^2-quat(3)^2+quat(4)^2];
end