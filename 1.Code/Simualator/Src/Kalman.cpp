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
* Function    : EKF_Init
* Description : 初始化卡尔曼滤波器相关
* Date        : 2022/09/21 logzhan
*---------------------------------------------------------------------**/
void EKF_Init(void) {

}


/**----------------------------------------------------------------------
* Function    : EKFCalQRMatrix
* Description : 根据GPS信号特征调整卡尔曼滤波噪声矩阵，q矩阵是过程噪声矩阵，
*               需要跟惯导预测位置精度相关。r矩阵为GNSS观测噪声，跟GPS输出的
*               信息精度有关。
* Date        : 2022/09/21 logzhan
*---------------------------------------------------------------------**/
void EKFCalQRMatrix() {

}


static Mat GetF()
{
	/*
	误差的顺序定义为：纬经高，东北天速度，东北天姿态，三轴陀螺仪零偏，三轴加速度计零偏。
	纬经需要以弧度为单位。
	*/
	Mat F(15, 15, 0);

	//double RpH1 = 1.0 / (Rprim + tpos.num[2][0]);//1/(卯酉圈+高）
	//double RmH1 = 1.0 / (Rmeri + tpos.num[2][0]);//1/(子午圈+高）

	//double cosphi = cos(tpos.num[0][0]);
	//double secphi = 1.0 / cosphi;
	//double sinphi = sin(tpos.num[0][0]);
	//double tanphi = tan(tpos.num[0][0]);

	//double vE = tspeed.num[0][0];
	//double vN = tspeed.num[1][0];
	//double vU = tspeed.num[2][0];

	////位置对位置影响的子矩阵	
	//Fpp.num[0][2] = RmH1 * RmH1 * (-vN);//高度对纬度的影响，与北向速度有关
	//Fpp.num[1][0] = RpH1 * vE * secphi * tanphi;//纬度对经度的影响，与纬度有关，与东向速度也有关。
	//Fpp.num[1][2] = RpH1 * RpH1 * (-vE) * secphi;//高度对经度的影响，与东向速度有关，与纬度有关。实际就是影响局部的横切面半径。
	//F.FillSubMat(0, 0, Fpp);

	////速度对位置影响的子矩阵	
	//Fvp.num[0][1] = RmH1;//北向速度对纬度影响
	//Fvp.num[1][0] = RpH1 * secphi;//东向速度对经度的影响
	//Fvp.num[2][2] = 1;//天向速度对高度的影响
	//F.FillSubMat(0, 3, Fvp);


	////位置对速度影响的子矩阵	
	//Fpv.num[0][0] = 2 * we * cosphi * vN + 2 * we * sinphi * vU + vN * vE * RpH1 * secphi * secphi;//纬度对东向速度的影响
	//Fpv.num[0][2] = RpH1 * RpH1 * (vE * vU - vN * vE * tanphi);//高度对东向速度的影响
	//Fpv.num[1][0] = (-(2 * vE * we * cosphi + vE * vE * RpH1 * secphi * secphi));//纬度对北向速度的影响
	//Fpv.num[1][2] = RmH1 * RmH1 * vN * vU + RpH1 * RpH1 * vE * vE * tanphi;//高度对北向速度的影响
	//Fpv.num[2][0] = (-2.0) * vE * we * sinphi;//纬度对天向速度的影响
	//Fpv.num[2][2] = (-RmH1 * RmH1 * vN * vN - RpH1 * RpH1 * vE * vE);//高度对天向速度的影响
	//F.FillSubMat(3, 0, Fpv);

	////速度对速度影响的子矩阵	
	//Fvv.num[0][0] = (vN * tanphi - vU) * RpH1;//东向速度对东向速度的影响
	//Fvv.num[0][1] = 2.0 * we * sinphi + vE * RpH1 * tanphi;//北向速度对东向速度的影响
	//Fvv.num[0][2] = (-2.0) * we * cosphi - vE * RpH1;//天向速度对东向速度的影响
	//Fvv.num[1][0] = (-2.0) * (we * sinphi + vE * RpH1 * tanphi);//东向速度对北向速度的影响
	//Fvv.num[1][1] = (-RmH1) * vU;//北向速度对北向速度的影响
	//Fvv.num[1][2] = (-RmH1) * vN;//天向速度对北向速度的影响
	//Fvv.num[2][0] = 2.0 * (we * cosphi + vE * RpH1);//东向速度对天向速度的影响//有的书公式写错了
	//Fvv.num[2][1] = 2 * vN * RmH1;//北向速度对天向速度的影响
	//F.FillSubMat(3, 3, Fvv);

	////姿态对速度影响的子矩阵
	//double fE = accn.num[0][0];
	//double fN = accn.num[1][0];
	//double fU = accn.num[2][0];


	//Fav.num[0][1] = (-fU);//北向角度对东向速度的影响
	//Fav.num[0][2] = fN;//天向角度对东向速度的影响
	//Fav.num[1][0] = fU;//东向角度对北向速度的影响
	//Fav.num[1][2] = (-fE);//天向角度对北向速度的影响
	//Fav.num[2][0] = (-fN);//东向角度对天向速度的影响
	//Fav.num[2][1] = fE;//北向角度对天向速度的影响
	//F.FillSubMat(3, 6, Fav);


	////位置对姿态影响的子矩阵	
	//Fpa.num[0][2] = vN * RmH1 * RmH1;//高度对东向姿态的影响
	//Fpa.num[1][0] = (-we) * sinphi;//纬度对北向姿态的影响
	//Fpa.num[1][2] = (-vE) * RpH1 * RpH1;//高度对北向姿态的影响
	//Fpa.num[2][0] = we * cosphi + vE * RpH1 * secphi * secphi;//纬度对天向姿态的影响
	//Fpa.num[2][2] = (-vE) * tanphi * RpH1 * RpH1;//高度对天向姿态的影响
	//F.FillSubMat(6, 0, Fpa);


	////速度对姿态影响的子矩阵	
	//Fva.num[0][1] = (-RmH1);//北向速度对东向姿态的影响
	//Fva.num[1][0] = RpH1;//东向速度对北向姿态的影响
	//Fva.num[2][0] = RpH1 * tanphi;//东向速度对天向姿态的影响
	//F.FillSubMat(6, 3, Fva);

	////姿态对姿态影响的子矩阵	
	//Faa.num[0][1] = we * sinphi + vE * RpH1 * tanphi;//北向姿态对东向姿态的影响
	//Faa.num[2][0] = we * cosphi + vE * RpH1;//东向姿态对天向姿态的影响
	//Faa.num[2][1] = vN * RmH1;//北向姿态对天向姿态的影响
	//Faa.num[0][2] = (-Faa.num[2][0]);//天向姿态对东向姿态的影响
	//Faa.num[1][0] = (-Faa.num[0][1]);//东向姿态对北向姿态的影响
	//Faa.num[1][2] = (-Faa.num[2][1]);//天向姿态对北向姿态的影响
	//F.FillSubMat(6, 6, Faa);

	//Mat cbn = Quat2DCM(qa);
	//// 陀螺仪零偏对姿态的影响。有的书把这个公式写错了SB
	//F.FillSubMat(6, 9, (-1) * cbn);
	//// 加速度计零偏对速度的影响
	//F.FillSubMat(3, 12, cbn);

	return F;
}

/**----------------------------------------------------------------------
* Function    : StateUpdate
* Description : 扩展卡尔曼滤波器的状态预测
* Date        : 2022/11/18 logzhan
*---------------------------------------------------------------------**/
static void StateUpdate() {

}

/**----------------------------------------------------------------------
* Function    : StatePredict
* Description : 扩展卡尔曼滤波器状态预测
* Date        : 2022/11/8 logzhan
*---------------------------------------------------------------------**/
static void StatePredict() {
	//Phi = (Eye + getF() * dTins) * Phi;
	//Q = Q + Q0 * dTins;
}