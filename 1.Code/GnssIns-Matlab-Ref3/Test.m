
yawdeg = 10;
pitchdeg = 20;
rolldeg = 30;
q=[1,0,0,0]';
% 先旋转Yaw(Z)
q = QuatUpdateDeg(q,0,0,yawdeg);
% 再旋转Pitch(X)
q = QuatUpdateDeg(q,pitchdeg,0,0);
% 最后旋转Roll(Y)
q = QuatUpdateDeg(q,0,rolldeg,0);

num = ' % 1.7f';
a = sprintf('\rRotation matrix to quaternion:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num, '\t', num), q);
disp(strcat(a,b));

q=[1,0,0,0]';
% 先旋转Yaw(Z)
q = QuatUpdateDeg(q,0,0,yawdeg);
% 最后旋转Roll(Y)
q = QuatUpdateDeg(q,0,rolldeg,0);
% 再旋转Pitch(X)
q = QuatUpdateDeg(q,pitchdeg,0,0);

num = ' % 1.7f';
a = sprintf('\rRotation matrix to quaternion:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num, '\t', num), q);
disp(strcat(a,b));