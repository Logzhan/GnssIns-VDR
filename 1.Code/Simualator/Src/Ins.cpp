/******************** (C) COPYRIGHT 2022 Geek************************************
* File Name          : Nav.cpp
* Current Version    : V1.0
* Author             : logzhan
* Date of Issued     : 2022.11.08
* Comments           : ��ϵ���λ���ں�(ENU ����������ϵ)
********************************************************************************/
/* Header File Including -----------------------------------------------------*/
#include "Ins.h"
#include "Utils.h"

//Ϊ�˷�ֹ��ջ��������Բ�Ҫ�ŵ������С�
Mat wien;
Mat wenn;
Mat wnbb;
Mat gn;
Mat dpos;

Mat Wien(double lat)
{
	const double we = 7.2921158e-5;
	Mat wien(3,1,0);
	wien.mat[1][0] = cos(lat)*we;
	wien.mat[2][0] = sin(lat)*we;//��
	return wien;
}

Mat Wenn(double lat, double H, double vE, double vN)
{
	EarthPara_t EarthPara = CalEarthModel(lat, H);
	// �����ٶȻ�������ת���������ٶȻ������������ת��
	Mat wenn(3,1,0);
	// ����������ϵ
	wenn.mat[0][0] = (-vN/(EarthPara.Rm + H));
	wenn.mat[1][0] = vE/(EarthPara.Rp + H);
	wenn.mat[2][0] = wenn.mat[1][0]*tan(lat);
	
	return wenn;
}

void InsStateUpdate(IMU_t& imu, GnssIns_t& gins)
{
	Mat gyro(3,1,0);
	Mat acc(3,1,0);

	gyro.mat[0][0] = imu.gyr.s[0];
	gyro.mat[1][0] = imu.gyr.s[1];
	gyro.mat[2][0] = imu.gyr.s[2];
	acc.mat[0][0]  = imu.acc.s[0];
	acc.mat[1][0]  = imu.acc.s[1];
	acc.mat[2][0]  = imu.acc.s[2];

	//����λ�ø��µ���ģ�͡�Ϊ�˼�С���������Ժ���Խ�����һ�εĸ���Ƶ�ʡ�
	EarthPara_t EarthPara = CalEarthModel(gins.pos.mat[0][0], gins.pos.mat[2][0]);

	//׼����������������ע�ⷽ��
	Mat acc1  = acc  + gins.AccBias;
	Mat gyro1 = gyro + gins.GyrBias;

	// ������̬
	Mat wien = Wien(gins.pos.mat[0][0]);
	Mat wenn = Wenn(gins.pos.mat[0][0], gins.pos.mat[2][0], gins.vel.mat[0][0], gins.vel.mat[1][0]);

	Mat cbn = Quat2DCM(gins.qbn);
	// �۳�������ת���۳��ٶ�����Ľ��ٶ�֮����bϵ��ת�����ٶ�
	wnbb = gyro1-(~cbn)*(wien + wenn);
	// ������̬
	gins.qbn = QuatAttUpdate(gins.qbn, wnbb * gins.dt);

	// �����ٶ�
	gins.AccN = cbn*acc1;//������������Ա��ڿ������˲��Ĳ���ʹ��
	gn.Init(3,1,0);
	gn.mat[2][0] = -EarthPara.ge;

	// ���µ���ϵ�ٶȵ�΢�� 
	Mat DVn = gins.AccN - ((wien + wien + wenn)^ gins.vel) + gn;
	gins.vel = gins.vel + gins.dt * DVn;//�����ٶ�

	// ����λ��
	dpos.Init(3,1,0);
	double H = gins.pos.mat[2][0];
	dpos.mat[0][0] = gins.vel.mat[1][0] / (EarthPara.Rm + H);//�����ٶȵõ�γ��
	dpos.mat[1][0] = gins.vel.mat[0][0] / ((EarthPara.Rp + H)*cos(gins.pos.mat[0][0]));//�����ٶȵõ�����
	dpos.mat[2][0] = gins.vel.mat[2][0];

	gins.pos = gins.pos +  dpos * gins.dt;
}

