function ou=getoula(q,coor)
%四元数转欧拉角

if(nargin==1)
    coor='ENU';
end

if(strcmp(coor,'ENU'))
    
elseif(strcmp(coor,'NUE'))
    q=updatedeg(q,0,-90,0);
    q=updatedeg(q,-90,0,0);    
else
end

cnb=(Qnb2Cbn(q))';
ou=zeros(3,1);
ou(1)=atan2(-cnb(2,1),cnb(2,2));
ou(2)=asin(cnb(2,3));%不要在俯仰90度的时候用欧拉角。
ou(3)=atan2(-cnb(1,3),cnb(3,3));
ou=ou*180/pi;
