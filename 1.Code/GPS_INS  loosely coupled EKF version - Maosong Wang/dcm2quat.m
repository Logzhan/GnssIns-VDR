%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                          韩云霄-201203027020
%                              cbn转四元数
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function q0=dcm2quat(cbn)
q0(1) = sqrt(1+cbn(1,1)+cbn(2,2)+cbn(3,3))/2;
q0(2) = (cbn(3,2)-cbn(2,3))/(4*q0(1));
q0(3) = (cbn(1,3)-cbn(3,1))/(4*q0(1));
q0(4) = (cbn(2,1)-cbn(1,2))/(4*q0(1));
temp = sqrt(q0(1)*q0(1)+q0(2)*q0(2)+q0(3)*q0(3)+q0(4)*q0(4));
%标准化四元数
q0(1)=q0(1)/temp;
q0(2)=q0(2)/temp;
q0(3)=q0(3)/temp;
q0(4)=q0(4)/temp;
end