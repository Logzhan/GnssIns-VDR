#ifndef  _VDR_UTILS_H_
#define  _VDR_UTILS_H_
#include "GnssInsDef.h"
#include "Mat.h"
/**---------------------------------------------------------------------
* Function    : Quat2DCM
* Description : ��Ԫ��ת�������Ҿ���
* Date        : 2022/09/14 logzhan
*---------------------------------------------------------------------**/
Mat Quat2DCM(Mat qu);

/**---------------------------------------------------------------------
* Function    : Quat2Euler
* Description : ��Ԫ��תŷ����(ENU)
* Date        : 2022/09/14 logzhan
*---------------------------------------------------------------------**/
Mat Quat2Euler(Mat qb);

/**---------------------------------------------------------------------
* Function    : QuatAttUpdate
* Description : �ý�С��ŷ���Ǹ�����Ԫ���������ŷ������ֵ����̫��
* Date        : 2022/11/148 logzhan
*---------------------------------------------------------------------**/
Mat QuatAttUpdate(Mat q1, Mat th);

/**---------------------------------------------------------------------
* Function    : EulerDeg2Quat
* Description : ŷ����(�Ƕ�)ת��Ԫ��
* Date        : 2022/11/18 logzhan
*---------------------------------------------------------------------**/
Mat EulerDeg2Quat(double yawdeg, double pitchdeg, double rolldeg);

/**----------------------------------------------------------------------
* Function    : CalEarthModel
* Description : ����������ģ��
* Date        : 2022/11/8 logzhan
*---------------------------------------------------------------------**/
EarthPara_t CalEarthModel(double Lat, double Height);

#endif