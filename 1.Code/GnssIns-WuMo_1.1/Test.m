InitYaw   = -143.26;         % ��Z����ת
InitPitch =  75.69;          % ��X����ת
InitRoll  =  12.2;           % ��Y����ת

% ����Yaw��Pitch��Roll(ZXY)ŷ����˳�򴴽���Ԫ��
atti1 = Euler2Quat(InitPitch/(180/pi),InitRoll/(180/pi),InitYaw/(180/pi),'ZXY'); 
ouler = getoula(atti1);
ouler2 = getoula(atti1);
