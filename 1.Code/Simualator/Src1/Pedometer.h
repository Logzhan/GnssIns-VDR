#ifndef _VDR_PEDOMETER_H_
#define _VDR_PEDOMETER_H_

#include "VDRBase.h"

void Pedometer_Init(void);

void PedometerUpdate(IMU_t* imu, uint64_t* step);

#endif