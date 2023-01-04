function q = EulerDeg2Quat(yawdeg,pitchdeg,rolldeg,coor)
% 初始四元数，对于EulerAngles = [0,0,0]
q=[1,0,0,0]';

% 北偏西航向为正

% 先旋转Yaw(Z)
q = QuatUpdateDeg(q,0,0,yawdeg);
% 再旋转Pitch(X)
q = QuatUpdateDeg(q,pitchdeg,0,0);
% 最后旋转Roll(Y)
q = QuatUpdateDeg(q,0,rolldeg,0);

% 默认采用是ENU坐标系
if(nargin==3)
    coor='ENU';
end

if(strcmp(coor,'ENU'))
    
elseif(strcmp(coor,'NUE'))
    q = QuatUpdateDeg(q,90,0,0);
    q = QuatUpdateDeg(q,0,90,0);    
else
end

end
