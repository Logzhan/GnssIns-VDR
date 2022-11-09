/*
导航计算类-ENU
1.20
lty
244917988@qq.com

*/
#ifndef _H_NAVI
#define _H_NAVI

#include "Mat.h"
#include "GnssInsDef.h"

/*
状态量顺序为：纬经高，东北天速度，东北天角度，三个陀螺仪，三个加速度计。
*/
void InsStateUpdate(IMU_t& imu, GnssIns_t& gins);

#endif