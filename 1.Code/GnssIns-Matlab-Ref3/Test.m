
yawdeg = 10;
pitchdeg = 20;
rolldeg = 30;
q=[1,0,0,0]';
% ����תYaw(Z)
q = QuatUpdateDeg(q,0,0,yawdeg);
% ����תPitch(X)
q = QuatUpdateDeg(q,pitchdeg,0,0);
% �����תRoll(Y)
q = QuatUpdateDeg(q,0,rolldeg,0);

num = ' % 1.7f';
a = sprintf('\rRotation matrix to quaternion:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num, '\t', num), q);
disp(strcat(a,b));

q=[1,0,0,0]';
% ����תYaw(Z)
q = QuatUpdateDeg(q,0,0,yawdeg);
% �����תRoll(Y)
q = QuatUpdateDeg(q,0,rolldeg,0);
% ����תPitch(X)
q = QuatUpdateDeg(q,pitchdeg,0,0);

num = ' % 1.7f';
a = sprintf('\rRotation matrix to quaternion:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num, '\t', num), q);
disp(strcat(a,b));