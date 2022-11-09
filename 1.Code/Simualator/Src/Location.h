/*
����������-ENU
1.20
lty
244917988@qq.com

*/
#ifndef _H_NAVI
#define _H_NAVI

#include "Mat.h"

extern const double deg2rad; //�Ƕ�ת�����ȵ�ϵ��
extern const double rad2deg; //����ת���Ƕȵ�ϵ��
extern const double we;      //������ת������


extern double dTins;

extern Mat qa;
extern Mat tspeed;
extern Mat tpos;

extern Mat AccBias;
extern Mat GyroBias;

void Kalman_Init();


Mat EulerDeg2Quat(double yawdeg, double pitchdeg, double rolldeg);

void InsStateUpdate(double gx, double gy, double gz, double ax, double ay, double az);


/*
״̬��˳��Ϊ��γ���ߣ��������ٶȣ�������Ƕȣ����������ǣ��������ٶȼơ�
*/

void StatePredict();
void StateCorrectUpdate(double lati, double longi, double height, double ve, double vn, double vu);//���Ǵ��������˿������˲�������



#endif