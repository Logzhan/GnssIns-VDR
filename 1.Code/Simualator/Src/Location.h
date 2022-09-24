#ifndef _VDR_LOCATION_H_
#define _VDR_LOCATION_H_

#include "VDRBase.h"

/**---------------------------------------------------------------------
* Function    : VDRNav_Init
* Description : VDR����ϵͳ��ʼ��
* Date        : 2022/09/21 logzhan
*---------------------------------------------------------------------**/
void VDRNav_Init(void);

/**----------------------------------------------------------------------
* Function    : InsLocation
* Description : VDR�ߵ�λ�ø���
* Date        : 2022-09-21 logzhan
*---------------------------------------------------------------------**/
void InsLocation(void);

/**----------------------------------------------------------------------
* Function    : GnssInsLocFusion
* Description : VDR��GNSS��INS�ں϶�λ
* Date        : 2022/09/21 logzhan
*---------------------------------------------------------------------**/
void GnssInsLocationUpdate(void);

#endif