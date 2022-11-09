/******************** (C) COPYRIGHT 2022 Geek************************************
* File Name          : Nav.cpp
* Current Version    : V1.0
* Author             : logzhan
* Date of Issued     : 2022.11.08
* Comments           : 组合导航位置融合(ENU 东北天坐标系)
********************************************************************************/
/* Header File Including -----------------------------------------------------*/
#include "Ins.h"
#include "Utils.h"

//为了防止堆栈溢出，所以不要放到函数中。
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
	wien.mat[2][0] = sin(lat)*we;//天
	return wien;
}

Mat Wenn(double lat, double H, double vE, double vN)
{
	EarthPara_t EarthPara = CalEarthModel(lat, H);
	// 北向速度会引起东向转动；东向速度会引起北向和天向转动
	Mat wenn(3,1,0);
	// 东北天坐标系
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

	//根据位置更新地球模型。为了减小计算量，以后可以降低这一段的更新频率。
	EarthPara_t EarthPara = CalEarthModel(gins.pos.mat[0][0], gins.pos.mat[2][0]);

	//准备，补偿传感器误差。注意方向
	Mat acc1  = acc  + gins.AccBias;
	Mat gyro1 = gyro + gins.GyrBias;

	// 计算姿态
	Mat wien = Wien(gins.pos.mat[0][0]);
	Mat wenn = Wenn(gins.pos.mat[0][0], gins.pos.mat[2][0], gins.vel.mat[0][0], gins.vel.mat[1][0]);

	Mat cbn = Quat2DCM(gins.qbn);
	// 扣除地球自转、扣除速度引起的角速度之后，在b系的转动角速度
	wnbb = gyro1-(~cbn)*(wien + wenn);
	// 更新姿态
	gins.qbn = QuatAttUpdate(gins.qbn, wnbb * gins.dt);

	// 计算速度
	gins.AccN = cbn*acc1;//更新这个数，以便于卡尔曼滤波的部分使用
	gn.Init(3,1,0);
	gn.mat[2][0] = -EarthPara.ge;

	// 更新导航系速度的微分 
	Mat DVn = gins.AccN - ((wien + wien + wenn)^ gins.vel) + gn;
	gins.vel = gins.vel + gins.dt * DVn;//更新速度

	// 更新位置
	dpos.Init(3,1,0);
	double H = gins.pos.mat[2][0];
	dpos.mat[0][0] = gins.vel.mat[1][0] / (EarthPara.Rm + H);//北向速度得到纬度
	dpos.mat[1][0] = gins.vel.mat[0][0] / ((EarthPara.Rp + H)*cos(gins.pos.mat[0][0]));//东向速度得到经度
	dpos.mat[2][0] = gins.vel.mat[2][0];

	gins.pos = gins.pos +  dpos * gins.dt;
}

