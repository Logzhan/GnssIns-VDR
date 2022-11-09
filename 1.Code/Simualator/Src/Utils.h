#ifndef  _VDR_UTILS_H_
#define  _VDR_UTILS_H_
#include "GnssInsDef.h"
#include "Mat.h"
/**---------------------------------------------------------------------
* Function    : Quat2DCM
* Description : 四元素转方向余弦矩阵
* Date        : 2022/09/14 logzhan
*---------------------------------------------------------------------**/
Mat Quat2DCM(Mat qu);

/**---------------------------------------------------------------------
* Function    : Quat2Euler
* Description : 四元数转欧拉角(ENU)
* Date        : 2022/09/14 logzhan
*---------------------------------------------------------------------**/
Mat Quat2Euler(Mat qb);

/**---------------------------------------------------------------------
* Function    : QuatAttUpdate
* Description : 用较小的欧拉角更新四元数，这里的欧拉角数值不能太大
* Date        : 2022/11/148 logzhan
*---------------------------------------------------------------------**/
Mat QuatAttUpdate(Mat q1, Mat th);

/**---------------------------------------------------------------------
* Function    : EulerDeg2Quat
* Description : 欧拉角(角度)转四元数
* Date        : 2022/11/18 logzhan
*---------------------------------------------------------------------**/
Mat EulerDeg2Quat(double yawdeg, double pitchdeg, double rolldeg);

/**----------------------------------------------------------------------
* Function    : CalEarthModel
* Description : 计算地球参数模型
* Date        : 2022/11/8 logzhan
*---------------------------------------------------------------------**/
EarthPara_t CalEarthModel(double Lat, double Height);

#endif