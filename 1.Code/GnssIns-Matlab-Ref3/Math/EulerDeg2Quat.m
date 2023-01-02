function q = EulerDeg2Quat(yawdeg,pitchdeg,rolldeg,coor)
q=[1,0,0,0]';
q = QuatUpdateDeg(q,0,0,yawdeg);%北偏西航向为正
q = QuatUpdateDeg(q,pitchdeg,0,0);
q = QuatUpdateDeg(q,0,rolldeg,0);

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
