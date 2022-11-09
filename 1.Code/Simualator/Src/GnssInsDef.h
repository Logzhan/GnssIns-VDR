#ifndef  _VDR_BASE_H_
#define  _VDR_BASE_H_

#include <stdint.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include "Mat.h"

#define ACCURACY_ERR_MAX                    1000                 // GPS的accuracy最大值,一般用于初始化用
#define N                                   4                    // 矩阵维数
#define MAX_NO_GPS_PREDICT                  10		             // 无GPS信息状态，最大位置推算数量	
#define IMU_SENSOR_AXIS                     3                    // IMU传感器数据轴数量，默认3

// 用户运动识别
#define DETECTOR_TYPE_STATIC                0                    // 用户静止 
#define DETECTOR_TYPE_IRREGULAR             1                    // 无规律运动
#define DETECTOR_TYPE_HANDHELD              2                    // 手持运动
#define DETECTOR_TYPE_SWINGING              3                    // 摆手运动
#define DETECTOR_NO_ERROR                   0

#define VDR_TRUE                            1
#define VDR_FALSE                           0

typedef struct {
	double Xk[N];             // 系统状态变量  xk[0]: 北向x  xk[1]：东向y  xk[2]：步长  xk[3] ：航向角
	double pXk[N];            // 最佳预测变量  xk[0]: 北向x  xk[1]：东向y  xk[2]：步长  xk[3] ：航向角
	double Zk[N];
	double pPk[N][N];
	double Pk[N][N];
	double Phi[N][N];
	double hk[N][N];
	double Q[N][N];           // 卡尔曼滤波的Q矩阵(过程噪声)
	double R[N][N];           // 卡尔曼滤波R矩阵(观测噪声)
	double Kk[N][N];
	double Lambda;
	double pLat;
	double pLon;
	double initHeading;
}EKFPara_t;

typedef struct Sensor {
	uint8_t update;
	int     type;
	double  time;
	double  s[IMU_SENSOR_AXIS];
}Sensor_t;

typedef struct {
	Sensor_t acc;
	Sensor_t gyr;
	Sensor_t mag;
}IMU_t;

// 用户运动类型分类器
typedef struct DETECTOR {
	uint32_t type;                              // 用户运动类别 ： 0:静止运动 1：无规律运动 2：手持运动 3：摆手运动
	uint32_t lastType;
	uint64_t tick;                              // 次数统计，用于调整检测器工作频率
}Detector_t;


typedef struct VDR {
	uint32_t        Status;                     // VDR当前状态
	uint32_t        MotionType;                 // 用户运动类型
	// 速度相关
	double          GnssSpeed;                  // GNSS速度
	double          Heading;                    // 航向
	// 步数相关
	uint64_t        Steps;                      // 当前步数信息
	uint64_t        LastSteps;                  // 上一次的步数
} VDR_t;



typedef struct {
	double lat;
	double lon;
	double height;
	double ve;
	double vn;
	double vu;
	double heading;
}Gnss_t;


typedef struct {
	double Rp;
	double Rm;
	double ge;
	double wien;
	double wenn;
}EarthPara_t;


typedef struct {
	double lat;
	double lon;
	double height;
	double ve;
	double vn;
	double vu;
	double heading;
	double att[3];
	double dt;
	Mat    GyrBias;
	Mat    AccBias;
	Mat    qbn;
	Mat    vel;
	Mat    pos;
	Mat    AccN;
}GnssIns_t;


typedef struct {
	Mat Phi;
	Mat Pk;
	Mat Eye;
	Mat Q;
	Mat Q0;
	Mat R;
	Mat Fpp;
	Mat Fvp;
	Mat Fpv;
	Mat Fvv;
	Mat Fav;
	Mat Fpa;
	Mat Fva;
	Mat Faa;
}Kalman_t;


const double deg2rad = M_PI / 180.0;
const double rad2deg = 180.0 * M_1_PI;
// 地球自转角速度
const double we      = 7.2921158e-5;

#endif // ! _VDR_BASE_H
