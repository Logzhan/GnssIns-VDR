/******************** (C) COPYRIGHT 2022 Geek************************************
* File Name          : Detector.cpp
* Current Version    : V1.0
* Author             : logzhan
* Date of Issued     : 2022.09.24
* Comments           : 
�˶������
********************************************************************************/
/* Header File Including -----------------------------------------------------*/
#include <iostream>
#include "Detector.h"

/**---------------------------------------------------------------------
* Function    : pdr_initDetector
* Description : ��ʼ���˶���������ʶ���û�������һ���˶�ģʽ
* Date        : 2020/02/16 logzhan & logzhan
*---------------------------------------------------------------------**/
void Detector_Init() {
	std::cout << "Detector_Init" << std::endl;
	DetectorReset();
}

/**---------------------------------------------------------------------
* Function    : DetectorReset
* Description : ����VDR�˶������
* Date        : 2022/09/23 logzhan
*---------------------------------------------------------------------**/
void DetectorReset(void) {
	std::cout << "DetectorReset" << std::endl;
}

/**---------------------------------------------------------------------
* Function    : DetectorUpdateIMU
* Description : �����˶����ͼ������imu��Ϣ���������һ����ʱ�����������
*               �û����˶�����
* Date        : 2022/09/23
*---------------------------------------------------------------------**/
void DetectorUpdateIMU(IMU_t* imu) {

}

/**---------------------------------------------------------------------
* Function    : DetectMotionType
* Description : pdr�˶����ͼ��
* Date        : 2020/7/20
*---------------------------------------------------------------------**/
int DetectMotionType() {
	return DETECTOR_TYPE_HANDHELD;
}