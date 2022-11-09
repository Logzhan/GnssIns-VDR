#ifndef  _VDR_BASE_H_
#define  _VDR_BASE_H_

#include <stdint.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include "Mat.h"

#define ACCURACY_ERR_MAX                    1000                 // GPS��accuracy���ֵ,һ�����ڳ�ʼ����
#define N                                   4                    // ����ά��
#define MAX_NO_GPS_PREDICT                  10		             // ��GPS��Ϣ״̬�����λ����������	
#define IMU_SENSOR_AXIS                     3                    // IMU������������������Ĭ��3

// �û��˶�ʶ��
#define DETECTOR_TYPE_STATIC                0                    // �û���ֹ 
#define DETECTOR_TYPE_IRREGULAR             1                    // �޹����˶�
#define DETECTOR_TYPE_HANDHELD              2                    // �ֳ��˶�
#define DETECTOR_TYPE_SWINGING              3                    // �����˶�
#define DETECTOR_NO_ERROR                   0

#define VDR_TRUE                            1
#define VDR_FALSE                           0

typedef struct {
	double Xk[N];             // ϵͳ״̬����  xk[0]: ����x  xk[1]������y  xk[2]������  xk[3] �������
	double pXk[N];            // ���Ԥ�����  xk[0]: ����x  xk[1]������y  xk[2]������  xk[3] �������
	double Zk[N];
	double pPk[N][N];
	double Pk[N][N];
	double Phi[N][N];
	double hk[N][N];
	double Q[N][N];           // �������˲���Q����(��������)
	double R[N][N];           // �������˲�R����(�۲�����)
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

// �û��˶����ͷ�����
typedef struct DETECTOR {
	uint32_t type;                              // �û��˶���� �� 0:��ֹ�˶� 1���޹����˶� 2���ֳ��˶� 3�������˶�
	uint32_t lastType;
	uint64_t tick;                              // ����ͳ�ƣ����ڵ������������Ƶ��
}Detector_t;


typedef struct VDR {
	uint32_t        Status;                     // VDR��ǰ״̬
	uint32_t        MotionType;                 // �û��˶�����
	// �ٶ����
	double          GnssSpeed;                  // GNSS�ٶ�
	double          Heading;                    // ����
	// �������
	uint64_t        Steps;                      // ��ǰ������Ϣ
	uint64_t        LastSteps;                  // ��һ�εĲ���
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
// ������ת���ٶ�
const double we      = 7.2921158e-5;

#endif // ! _VDR_BASE_H
