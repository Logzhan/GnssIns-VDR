function q = EulerDeg2Quat(yawdeg,pitchdeg,rolldeg,coor)
% ��ʼ��Ԫ��������EulerAngles = [0,0,0]
q=[1,0,0,0]';

% ��ƫ������Ϊ��

% ����תYaw(Z)
q = QuatUpdateDeg(q,0,0,yawdeg);
% ����תPitch(X)
q = QuatUpdateDeg(q,pitchdeg,0,0);
% �����תRoll(Y)
q = QuatUpdateDeg(q,0,rolldeg,0);

% Ĭ�ϲ�����ENU����ϵ
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
