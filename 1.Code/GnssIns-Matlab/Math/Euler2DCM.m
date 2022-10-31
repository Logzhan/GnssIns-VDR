%----------------------------------------------------------------------
% Description : Å·À­½Ç×ªÐý×ª¾ØÕó
% Author      : loghzan
% Date        : 2022-10-31
%----------------------------------------------------------------------
function cbn = Euler2DCM(roll,pitch,yaw)
cbn=[cos(pitch)*cos(yaw),-cos(roll)*sin(yaw)+sin(roll)*sin(pitch)*cos(yaw),sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw);
    cos(pitch)*sin(yaw),cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw),-sin(roll)*cos(yaw)+cos(roll)*sin(pitch)*sin(yaw);
    -sin(pitch),sin(roll)*cos(pitch),cos(roll)*cos(pitch)];
end