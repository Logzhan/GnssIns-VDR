% ���ŷ����ת��Ԫ�������ǣ���ǰ������ϵ�£�ZXY˳���ŷ����
% yaw   : ��z��
% pitch : ��x��
% roll  : ��y��
function q = setoula(rx,ry,rz,coor)
q=[1,0,0,0]';

q=updatedeg(q,0,0,rz);%��ƫ������Ϊ��
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
