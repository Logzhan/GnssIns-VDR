function ou=getoula(q,coor)
%��Ԫ��תŷ����

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
ou(2)=asin(cnb(2,3));%��Ҫ�ڸ���90�ȵ�ʱ����ŷ���ǡ�
ou(3)=atan2(-cnb(1,3),cnb(3,3));
ou=ou*180/pi;
