function q=setoula(yawdeg,pitchdeg,rolldeg,coor)
q=[1,0,0,0]';
q=updatedeg(q,0,0,yawdeg);%北偏西航向为正
q=updatedeg(q,pitchdeg,0,0);
q=updatedeg(q,0,rolldeg,0);

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
