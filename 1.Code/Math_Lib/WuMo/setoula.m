% 这个欧拉角转四元数本质是：右前上坐标系下，ZXY顺序的欧拉角
% yaw   : 绕z轴
% pitch : 绕x轴
% roll  : 绕y轴
function q = setoula(rx,ry,rz,coor)
q=[1,0,0,0]';

q=updatedeg(q,0,0,rz);%北偏西航向为正
q=updatedeg(q,rx,0,0);
q=updatedeg(q,0,ry,0);


if(nargin==3)
    coor='ENU';
end

if(strcmp(coor,'ENU'))
    
elseif(strcmp(coor,'NUE'))
    q=updatedeg(q,90,0,0);
    q=updatedeg(q,0,90,0);    
else
end

end
