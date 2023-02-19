% -----------------------------------------------------------------------------
% Function    : DCM2Euler 
% Description : 欧拉角转方向余弦矩阵，目前仅支持ZXY顺规欧拉角
% Input       : Cnb:方向余弦矩阵  coor:坐标轴类型或者欧拉角顺规，目前仅支持
%               ZXY
% Output      : 按照XYZ顺序返回姿态角
% Author      : logzhan
% Date        : 2023-01-10 01:54
% Reference   : 捷联惯导算法与组合导航原理讲义.严恭敏, P262, 公式B-4.    
%               捷联惯导算法与组合导航原理讲义.严恭敏, P249, 姿态阵转换为姿态角
% -----------------------------------------------------------------------------
function [att] = DCM2Euler(Cnb,coor)
if(strcmp(coor,'ZXY') || strcmp(coor,'ENU'))
    % 捷联惯导算法与组合导航原理讲义.严恭敏, P262, 公式B-4.
    % 捷联惯导算法与组合导航原理讲义.严恭敏, P249, 姿态阵转换为姿态角.
    if abs(Cnb(3,2))<=0.999999
        att = [ asin(Cnb(3,2)); -atan2(Cnb(3,1),Cnb(3,3)); -atan2(Cnb(1,2),Cnb(2,2)) ];
    else
        att = [ asin(Cnb(3,2)); atan2(Cnb(1,3),Cnb(1,1)); 0 ];
    end
end

