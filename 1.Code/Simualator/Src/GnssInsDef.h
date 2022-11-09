#ifndef  _VDR_BASE_H_
#define  _VDR_BASE_H_

#include <stdint.h>

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

#endif // ! _VDR_BASE_H
