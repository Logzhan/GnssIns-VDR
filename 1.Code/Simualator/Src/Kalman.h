#ifndef _VDR_KALMAN_H_
#define _VDR_KALMAN_H_

#include "GnssInsDef.h"

/**----------------------------------------------------------------------
* Function    : Kalman_Init
* Description : 初始化卡尔曼滤波器相关
* Date        : 2022/11/09 logzhan
*---------------------------------------------------------------------**/
void Kalman_Init(GnssIns_t& gins);

void StatePredict(GnssIns_t& gins);

/**---------------------------------------------------------------------
* Function    : StateCorrectUpdate
* Description : 卡尔曼状态更新以及状态修正
* Date        : 2022/11/18 logzhan
*---------------------------------------------------------------------**/
void StateUpdate(Gnss_t& gnss, GnssIns_t& gins);

#endif