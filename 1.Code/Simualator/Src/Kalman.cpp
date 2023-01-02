#include "Kalman.h"
#include "Mat.h"
#include "Utils.h"

static Mat Phi(15, 15, 1);
static Mat Pk(15, 15, 0);
static Mat Eye(15, 15, 1);
static Mat Q(15, 15, 0);
static Mat Q0(15, 15, 0);
static Mat R(6, 6, 1);
//为了防止堆栈溢出，所以不要放到函数中。
static Mat Fpp(3, 3, 0);
static Mat Fvp(3, 3, 0);
static Mat Fpv(3, 3, 0);
static Mat Fvv(3, 3, 0);
static Mat Fav(3, 3, 0);
static Mat Fpa(3, 3, 0);
static Mat Fva(3, 3, 0);
static Mat Faa(3, 3, 0);

/**----------------------------------------------------------------------
* Function    : Kalman_Init
* Description : 初始化卡尔曼滤波器相关
* Date        : 2022/11/09 logzhan
*---------------------------------------------------------------------**/
void Kalman_Init(GnssIns_t& gins)
{
	Pk.mat[0][0] = 1e-12;
	Pk.mat[1][1] = 1e-12;
	Pk.mat[2][2] = 9;
	Pk.mat[3][3] = 0.1;
	Pk.mat[4][4] = 0.1;
	Pk.mat[5][5] = 0.1;
	Pk.mat[6][6] = 3e-4;
	Pk.mat[7][7] = 3e-4;
	Pk.mat[8][8] = 3e-4;

	Q0.mat[3][3] = 1e-4;
	Q0.mat[4][4] = 1e-4;
	Q0.mat[5][5] = 1e-4;
	Q0.mat[6][6] = 1e-10;
	Q0.mat[7][7] = 1e-10;
	Q0.mat[8][8] = 1e-10;

	R.mat[0][0] = 1e-12;
	R.mat[1][1] = 1e-12;
	R.mat[2][2] = 9;
	R.mat[3][3] = 0.1;
	R.mat[4][4] = 0.1;
	R.mat[5][5] = 0.1;

	gins.qbn = EulerDeg2Quat(0.5, -0.4, 0.3);
	gins.pos.mat[0][0] = 40 * deg2rad;
	gins.pos.mat[1][0] = 120 * deg2rad;
	gins.pos.mat[2][0] = 500;
}

/**---------------------------------------------------------------------
* Function    : EkfStateUpdate
* Description : 卡尔曼状态更新以及状态修正
* Date        : 2022/11/18 logzhan
*---------------------------------------------------------------------**/
void EkfStateUpdate(Gnss_t& gnss, GnssIns_t& gins)
{
	Mat Z(6, 1, 0);
	Z.mat[0][0] = gins.pos.mat[0][0] - gnss.lat;
	Z.mat[1][0] = gins.pos.mat[1][0] - gnss.lon;
	Z.mat[2][0] = gins.pos.mat[2][0] - gnss.height;
	Z.mat[3][0] = gins.vel.mat[0][0] - gnss.ve;
	Z.mat[4][0] = gins.vel.mat[1][0] - gnss.vn;
	Z.mat[5][0] = gins.vel.mat[2][0] - gnss.vu;

	Mat H(6, 15, 1);

	Mat Pkk = Phi * Pk * (~Phi) + Q;
	Mat K = Pkk * (~H) / (H * Pkk * (~H) + R);
	// 因为每次滤波之后补偿了误差，所以状态预测总是0
	Mat X = K * Z;

	// 更新协方差
	Mat IKH = Eye - K * H;
	Pk = IKH * Pkk * (~IKH) + K * R * (~K);

	Phi.Init(15, 15, 1);
	Q.Init(15, 15, 0);
	// 位置修正
	gins.pos = gins.pos - X.SubMat(0, 0, 3, 1);
	// 速度修正
	gins.vel = gins.vel - X.SubMat(3, 0, 3, 1);
	// 姿态修正
	gins.qbn = QuatAttUpdate(gins.qbn, (~Quat2DCM(gins.qbn)) * X.SubMat(6, 0, 3, 1));
	// 传感器零偏修正
	//biasgyro=biasgyro-X.submat(9,0,3,1);
	//biasacc=biasacc-X.submat(12,0,3,1);
}
/**----------------------------------------------------------------------
* Function    : GetF
* Description : 获取F矩阵
* Date        : 2022/11/09 logzhan
*---------------------------------------------------------------------**/
Mat GetF(GnssIns_t& gins)
{
	/*
	误差的顺序定义为：纬经高，东北天速度，东北天姿态，三轴陀螺仪零偏，三轴加速度计零偏。
	纬经需要以弧度为单位。
	*/
	Mat F(15, 15, 0);

	EarthPara_t EarthPara = CalEarthModel(gins.pos.mat[0][0], gins.pos.mat[2][0]);
	// 1/(卯酉圈+高）
	double RpH1 = 1.0 / (EarthPara.Rp + gins.pos.mat[2][0]);
	// 1/(子午圈+高）
	double RmH1 = 1.0 / (EarthPara.Rm + gins.pos.mat[2][0]);

	double cosphi = cos(gins.pos.mat[0][0]);
	double secphi = 1.0 / cosphi;
	double sinphi = sin(gins.pos.mat[0][0]);
	double tanphi = tan(gins.pos.mat[0][0]);

	double vE = gins.vel.mat[0][0];
	double vN = gins.vel.mat[1][0];
	double vU = gins.vel.mat[2][0];

	// 位置对位置影响的子矩阵	
	// 高度对纬度的影响，与北向速度有关
	Fpp.mat[0][2] = RmH1 * RmH1 * (-vN);
	// 纬度对经度的影响，与纬度有关，与东向速度也有关。
	Fpp.mat[1][0] = RpH1 * vE * secphi * tanphi;
	// 高度对经度的影响，与东向速度有关，与纬度有关。实际就是影响局部的横切面半径。
	Fpp.mat[1][2] = RpH1 * RpH1 * (-vE) * secphi;
	F.FillSubMat(0, 0, Fpp);

	// 速度对位置影响的子矩阵	
	// 北向速度对纬度影响
	Fvp.mat[0][1] = RmH1;
	// 东向速度对经度的影响
	Fvp.mat[1][0] = RpH1 * secphi;
	// 天向速度对高度的影响
	Fvp.mat[2][2] = 1;
	F.FillSubMat(0, 3, Fvp);

	// 位置对速度影响的子矩阵	
	// 纬度对东向速度的影响
	Fpv.mat[0][0] = 2 * we * cosphi * vN + 2 * we * sinphi * vU + vN * vE * RpH1 * secphi * secphi;
	// 高度对东向速度的影响
	Fpv.mat[0][2] = RpH1 * RpH1 * (vE * vU - vN * vE * tanphi);
	// 纬度对北向速度的影响
	Fpv.mat[1][0] = (-(2 * vE * we * cosphi + vE * vE * RpH1 * secphi * secphi));
	// 高度对北向速度的影响
	Fpv.mat[1][2] = RmH1 * RmH1 * vN * vU + RpH1 * RpH1 * vE * vE * tanphi;
	// 纬度对天向速度的影响
	Fpv.mat[2][0] = (-2.0) * vE * we * sinphi;
	// 高度对天向速度的影响
	Fpv.mat[2][2] = (-RmH1 * RmH1 * vN * vN - RpH1 * RpH1 * vE * vE);
	F.FillSubMat(3, 0, Fpv);

	// 速度对速度影响的子矩阵	
	Fvv.mat[0][0] = (vN * tanphi - vU) * RpH1;                   // 东向速度对东向速度的影响
	Fvv.mat[0][1] = 2.0 * we * sinphi + vE * RpH1 * tanphi;      // 北向速度对东向速度的影响
	Fvv.mat[0][2] = (-2.0) * we * cosphi - vE * RpH1;            // 天向速度对东向速度的影响
	Fvv.mat[1][0] = (-2.0) * (we * sinphi + vE * RpH1 * tanphi); // 东向速度对北向速度的影响
	Fvv.mat[1][1] = (-RmH1) * vU;                                // 北向速度对北向速度的影响
	Fvv.mat[1][2] = (-RmH1) * vN;                                // 天向速度对北向速度的影响
	Fvv.mat[2][0] = 2.0 * (we * cosphi + vE * RpH1);             // 东向速度对天向速度的影响, 有的书公式写错了
	Fvv.mat[2][1] = 2 * vN * RmH1;                               // 北向速度对天向速度的影响
	F.FillSubMat(3, 3, Fvv);

	// 姿态对速度影响的子矩阵
	double fE = gins.fn.mat[0][0];
	double fN = gins.fn.mat[1][0];
	double fU = gins.fn.mat[2][0];

	// 北向角度对东向速度的影响
	Fav.mat[0][1] = (-fU);  
	// 天向角度对东向速度的影响
	Fav.mat[0][2] = fN;  
	// 东向角度对北向速度的影响
	Fav.mat[1][0] = fU;   
	// 天向角度对北向速度的影响
	Fav.mat[1][2] = (-fE);  
	// 东向角度对天向速度的影响
	Fav.mat[2][0] = (-fN);  
	// 北向角度对天向速度的影响
	Fav.mat[2][1] = fE;   
	F.FillSubMat(3, 6, Fav);


	// 位置对姿态影响的子矩阵	
	// 高度对东向姿态的影响
	Fpa.mat[0][2] = vN * RmH1 * RmH1;
	// 纬度对北向姿态的影响
	Fpa.mat[1][0] = (-we) * sinphi;
	// 高度对北向姿态的影响
	Fpa.mat[1][2] = (-vE) * RpH1 * RpH1;
	// 纬度对天向姿态的影响
	Fpa.mat[2][0] = we * cosphi + vE * RpH1 * secphi * secphi;
	// 高度对天向姿态的影响
	Fpa.mat[2][2] = (-vE) * tanphi * RpH1 * RpH1;
	F.FillSubMat(6, 0, Fpa);


	// 速度对姿态影响的子矩阵	
	Fva.mat[0][1] = (-RmH1);                          // 北向速度对东向姿态的影响
	Fva.mat[1][0] = RpH1;                             // 东向速度对北向姿态的影响
	Fva.mat[2][0] = RpH1 * tanphi;                    // 东向速度对天向姿态的影响
	F.FillSubMat(6, 3, Fva);

	// 姿态对姿态影响的子矩阵	
	Faa.mat[0][1] = we * sinphi + vE * RpH1 * tanphi; // 北向姿态对东向姿态的影响
	Faa.mat[2][0] = we * cosphi + vE * RpH1;          // 东向姿态对天向姿态的影响
	Faa.mat[2][1] = vN * RmH1;                        // 北向姿态对天向姿态的影响
	Faa.mat[0][2] = (-Faa.mat[2][0]);                 // 天向姿态对东向姿态的影响
	Faa.mat[1][0] = (-Faa.mat[0][1]);                 // 东向姿态对北向姿态的影响
	Faa.mat[1][2] = (-Faa.mat[2][1]);                 // 天向姿态对北向姿态的影响
	F.FillSubMat(6, 6, Faa);

	Mat cbn = Quat2DCM(gins.qbn);
	// 陀螺仪零偏对姿态的影响。有的书把这个公式写错了SB
	F.FillSubMat(6, 9, (-1) * cbn);
	// 加速度计零偏对速度的影响
	F.FillSubMat(3, 12, cbn);

	return F;
}
/**----------------------------------------------------------------------
* Function    : EkfStatePredict
* Description : 卡尔曼状态预测
* Date        : 2022/11/09 logzhan
*---------------------------------------------------------------------**/
void EkfStatePredict(GnssIns_t& gins)
{
	Phi = (Eye + GetF(gins) * gins.dt) * Phi;
	Q = Q + Q0 * gins.dt;
}