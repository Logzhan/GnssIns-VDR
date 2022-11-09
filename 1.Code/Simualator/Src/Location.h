/*
导航计算类-ENU
1.20
lty
244917988@qq.com

*/
#ifndef _H_NAVI
#define _H_NAVI

#include "Mat.h"

extern const double deg2rad; //角度转换弧度的系数
extern const double rad2deg; //弧度转换角度的系数
extern const double we;      //地球自转角速率


extern double dTins;

extern Mat qa;
extern Mat tspeed;
extern Mat tpos;

extern Mat AccBias;
extern Mat GyroBias;

void Kalman_Init();


Mat EulerDeg2Quat(double yawdeg, double pitchdeg, double rolldeg);

void InsStateUpdate(double gx, double gy, double gz, double ax, double ay, double az);


/*
状态量顺序为：纬经高，东北天速度，东北天角度，三个陀螺仪，三个加速度计。
*/

void StatePredict();
void StateCorrectUpdate(double lati, double longi, double height, double ve, double vn, double vu);//卫星处理。集成了卡尔曼滤波和误差补偿



#endif