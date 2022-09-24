#ifndef _VDR_LOCATION_H_
#define _VDR_LOCATION_H_

#include "VDRBase.h"

/**---------------------------------------------------------------------
* Function    : VDRNav_Init
* Description : VDR导航系统初始化
* Date        : 2022/09/21 logzhan
*---------------------------------------------------------------------**/
void VDRNav_Init(void);

/**----------------------------------------------------------------------
* Function    : InsLocation
* Description : VDR惯导位置更新
* Date        : 2022-09-21 logzhan
*---------------------------------------------------------------------**/
void InsLocation(void);

/**----------------------------------------------------------------------
* Function    : GnssInsLocFusion
* Description : VDR的GNSS和INS融合定位
* Date        : 2022/09/21 logzhan
*---------------------------------------------------------------------**/
void GnssInsLocationUpdate(void);

#endif