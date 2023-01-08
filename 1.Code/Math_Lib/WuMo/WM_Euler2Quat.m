function q = WM_Euler2Quat(yawdeg,pitchdeg,rolldeg,coor)
q=[1,0,0,0]';

% 先绕Z轴(Yaw)旋转
q = qupdate(q,[0,0,yawdeg/(180/pi)]);
% 再绕X轴旋转(Pitch)
q = qupdate(q,[pitchdeg/(180/pi),0,0]);
% 最后绕Y轴(Roll)旋转
q = qupdate(q,[0,rolldeg/(180/pi),0]);

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
