/******************** (C) COPYRIGHT 2022 Geek************************************
* File Name          : Location.cpp
* Current Version    : V1.0
* Author             : logzhan
* Date of Issued     : 2022.09.21
* Comments           : 
�����㷨������
********************************************************************************/
/* Header File Including -----------------------------------------------------*/
#include "DirectionEstimator.h"
#include "Kalman.h"
#include "AHRS.h"
#include "Pedometer.h"
#include "Detector.h"
#include "Location.h"

/* Global Variable Definition ------------------------------------------------*/
EKFPara_t EkFPara;
VDR_t     VDR;

/**---------------------------------------------------------------------
* Function    : VDRNav_Init
* Description : VDR����ϵͳ��ʼ��
* Date        : 2022/09/21 logzhan
*---------------------------------------------------------------------**/
void VDRNav_Init(void) 
{
	Detector_Init();
	AHRS_Init();
	EKF_Init();
	Pedometer_Init();
	DirectionEstimator_Init();
}

/**----------------------------------------------------------------------
* Function    : InsLocation
* Description : VDR�ߵ�λ�ø���
* Date        : 2022-09-21
*---------------------------------------------------------------------**/
void InsLocationUpdate(IMU_t* ImuData, EKFPara_t* Ekf)
{
	if (UpdateAHRS(ImuData)) {
		
	}
	/* Updating imu info in order to detect user moving type. */
	DetectorUpdateIMU(ImuData);
	/* calculate user step info update. */
	PedometerUpdate(ImuData, &VDR.Steps);
	/* Using multiply sensor info to predict user real moving direction. */
	DirectionPredict(&VDR.Heading);
	/* Extend kalman filter update system state when ins data update. */
	EKFUpdateInsState();
}


/**----------------------------------------------------------------------
* Function    : GnssInsLocFusion
* Description : VDR��GNSS��INS�ں϶�λ
* Date        : 2022/09/21 logzhan
*---------------------------------------------------------------------**/
void GnssInsLocationUpdate(void) {

}