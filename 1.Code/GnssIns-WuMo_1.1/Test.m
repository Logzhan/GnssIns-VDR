InitYaw   = -143.26;         % 绕Z轴旋转
InitPitch =  75.69;          % 绕X轴旋转
InitRoll  =  12.2;           % 绕Y轴旋转

% 按照Yaw、Pitch、Roll(ZXY)欧拉角顺序创建四元数
atti1 = Euler2Quat(InitPitch/(180/pi),InitRoll/(180/pi),InitYaw/(180/pi),'ZXY'); 
ouler = getoula(atti1);
ouler2 = getoula(atti1);
