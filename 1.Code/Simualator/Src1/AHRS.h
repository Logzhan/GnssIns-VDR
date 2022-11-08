#ifndef  _VDR_AHRS_H_
#define  _VDR_AHRS_H_

#include "VDRBase.h"

/**----------------------------------------------------------------------
* Function    : AHRS_Init
* Description : AHRS��ʼ��
* Date        : 2022/09/21 logzhan
*---------------------------------------------------------------------**/
void AHRS_Init();

/**----------------------------------------------------------------------
* Function    : MahonyUpdateAHRS
* Description : Mahony��̬�����㷨
* Date        : 2022/09/21 logzhan
*---------------------------------------------------------------------**/
void MahonyUpdateAHRS(void);

/**----------------------------------------------------------------------
* Function    : UpdateAHRS
* Description : AHRS�ںϽ���
* Date        : 2022/09/23 logzhan
*---------------------------------------------------------------------**/
int UpdateAHRS(IMU_t* IMU);

#endif