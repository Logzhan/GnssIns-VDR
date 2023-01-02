/******************** (C) COPYRIGHT 2022 Geek************************************
* File Name          : Utils.c
* Current Version    : V1.0
* Author             : logzhan
* Date of Issued     : 2022.11.18
* Comments           : 组合导航工具类函数
********************************************************************************/
/* Header File Including -----------------------------------------------------*/
#include "GnssInsDef.h"
#include "Utils.h"

extern const double deg2rad; // 角度转换弧度的系数
extern const double rad2deg; // 弧度转换角度的系数
extern const double we;      // 地球自转角速率

/**---------------------------------------------------------------------
* Function    : Quat2DCM
* Description : 四元素转方向余弦矩阵
* Date        : 2022/09/14 logzhan
*---------------------------------------------------------------------**/
Mat Quat2DCM(Mat qu)
{
	double q0, q1, q2, q3;
	q0 = qu.mat[0][0];
	q1 = qu.mat[1][0];
	q2 = qu.mat[2][0];
	q3 = qu.mat[3][0];
	double q00, q11, q22, q33;
	q00 = q0 * q0;
	q11 = q1 * q1;
	q22 = q2 * q2;
	q33 = q3 * q3;

	Mat c(3, 3, 0);

	c.mat[0][0] = q00 + q11 - q22 - q33;
	c.mat[0][1] = 2 * (q1 * q2 - q0 * q3);
	c.mat[0][2] = 2 * (q1 * q3 + q0 * q2);
	c.mat[1][0] = 2 * (q1 * q2 + q0 * q3);
	c.mat[1][1] = q00 - q11 + q22 - q33;
	c.mat[1][2] = 2 * (q2 * q3 - q0 * q1);
	c.mat[2][0] = 2 * (q1 * q3 - q0 * q2);
	c.mat[2][1] = 2 * (q2 * q3 + q0 * q1);
	c.mat[2][2] = q00 - q11 - q22 + q33;

	return c;
}

/**---------------------------------------------------------------------
* Function    : Quat2Euler
* Description : 四元数转欧拉角(ENU)
* Date        : 2022/09/14 logzhan
*---------------------------------------------------------------------**/
Mat Quat2Euler(Mat qb)
{
	Mat cbn = Quat2DCM(qb);
	Mat attideg(3, 1, 0);
	Mat cnb = (~cbn);
	Mat ou(3, 1, 0);
	ou.mat[0][0] = atan2(-cnb.mat[1][0], cnb.mat[1][1]);
	ou.mat[1][0] = asin(cnb.mat[1][2]);
	ou.mat[2][0] = atan2(-cnb.mat[0][2], cnb.mat[2][2]);
	return ou * rad2deg;
}
/**---------------------------------------------------------------------
* Function    : QuatAttUpdate
* Description : 用较小的欧拉角更新四元数，这里的欧拉角数值不能太大
* Date        : 2022/09/14 logzhan
*---------------------------------------------------------------------**/
Mat QuatAttUpdate(Mat q1, Mat th)
{
	double thabs = th.absvec();

	Mat TH(4, 4, 0);
	TH.mat[0][1] = (-th.mat[0][0]);
	TH.mat[0][2] = (-th.mat[1][0]);
	TH.mat[0][3] = (-th.mat[2][0]);
	TH.mat[1][2] = (th.mat[2][0]);
	TH.mat[1][3] = (-th.mat[1][0]);
	TH.mat[2][3] = (th.mat[0][0]);
	TH = TH - (~TH);

	Mat A = Mat(4, 4, 1) * cos(thabs * 0.5);

	if (thabs < 1e-6) {
		A = A + TH * 0.5;
	}
	else {
		A = A + TH * sin(thabs * 0.5) / thabs;
	}
	return A * q1;
}

/**---------------------------------------------------------------------
* Function    : EulerDeg2Quat
* Description : 欧拉角(角度)转四元数
* Date        : 2022/11/18 logzhan
*---------------------------------------------------------------------**/
Mat EulerDeg2Quat(double yawdeg, double pitchdeg, double rolldeg)
{
	Mat qu(4, 1, 1);

	Mat th;
	th.Init(3, 1, 0);
	th.mat[2][0] = yawdeg * deg2rad;
	qu = QuatAttUpdate(qu, th);

	th.Init(3, 1, 0);
	th.mat[0][0] = pitchdeg * deg2rad;
	qu = QuatAttUpdate(qu, th);

	th.Init(3, 1, 0);
	th.mat[1][0] = rolldeg * deg2rad;
	qu = QuatAttUpdate(qu, th);

	return qu;
}

/**----------------------------------------------------------------------
* Function    : CalEarthModel
* Description : 计算地球参数模型
* Date        : 2022/11/8 logzhan
*---------------------------------------------------------------------**/
EarthPara_t CalEarthModel(double Lat, double Height)
{
	EarthPara_t EarthPara;
	const double Re = 6378137;
	const double ee = 1 / 298.25722;

	double lat = Lat;//纬度
	double sin_phi = sin(lat);
	double sin2_phi = sin_phi * sin_phi;
	EarthPara.Rp = Re * (1 + ee * sin2_phi);
	EarthPara.Rm = Re * (1 - 2 * ee + 3 * ee * sin2_phi);

	double g0 = 9.7803267714;
	double sl = sin(lat);
	double s2l = sin(2 * lat);
	double sl2 = sl * sl;
	double s2l2 = s2l * s2l;
	double hr = 1 + Height / Re;

	EarthPara.ge = g0 * (1 + 0.0053024 * sl2 - 0.0000059 * s2l2) / (hr * hr);
	return EarthPara;
}