/******************** (C) COPYRIGHT 2022 Geek************************************
* File Name          : Nav.cpp
* Current Version    : V1.0
* Author             : logzhan
* Date of Issued     : 2022.11.08
* Comments           : 组合导航位置融合(ENU 东北天坐标系)
********************************************************************************/
/* Header File Including -----------------------------------------------------*/
#include "Location.h"
#include "Utils.h"

const double deg2rad = M_PI/180.0;
const double rad2deg = 180.0*M_1_PI;
const double we = 7.2921158e-5;

double dTins;

Mat qa(4,1,1);
Mat tspeed(3,1,0);
Mat tpos(3,1,0);

Mat AccBias(3,1,0);
Mat	GyroBias(3,1,0);

double Rprim,Rmeri,ge;

Mat AccN;

Mat Wien()
{
	double latitude = tpos.mat[0][0];
	Mat wien(3,1,0);
	wien.mat[1][0] = cos(latitude)*we;
	wien.mat[2][0] = sin(latitude)*we;//天
	return wien;
}

Mat Wenn()
{
	//北向速度会引起东向转动；东向速度会引起北向和天向转动
	double lat = tpos.mat[0][0];
	double H   = tpos.mat[2][0];
	double vE  = tspeed.mat[0][0];
	double vN  = tspeed.mat[1][0];
	
	Mat wen(3,1,0);
	// 东
	wen.mat[0][0] = (-vN/(Rmeri + H));
	// 北
	wen.mat[1][0] = vE/(Rprim + H);
	// 天
	wen.mat[2][0] = wen.mat[1][0]*tan(lat);
	
	return wen;
}

void CalEarthModel()
{
	const double Re=6378137;
	const double ee=1/298.25722;

	double lat=tpos.mat[0][0];//纬度
	double sin_phi=sin(lat);
	double sin2_phi=sin_phi*sin_phi;
	Rprim=Re*(1+ee*sin2_phi);
	Rmeri=Re*(1-2*ee+3*ee*sin2_phi);

	double g0=9.7803267714;
	double sl=sin(lat);
	double s2l=sin(2*lat);
	double sl2=sl*sl;
	double s2l2=s2l*s2l;
	double hr=1+tpos.mat[2][0]/Re;

	ge=g0*(1+0.0053024*sl2-0.0000059*s2l2)/(hr*hr);
}


//为了防止堆栈溢出，所以不要放到函数中。
Mat acc1;
Mat gyro1;
Mat wien;
Mat wenn;
Mat wnbb;
Mat gn;
Mat dpos;

void InsStateUpdate(double gx,double gy,double gz,double ax,double ay,double az)
{
	Mat gyro(3,1,0);
	Mat acc(3,1,0);

	gyro.mat[0][0]=gx;
	gyro.mat[1][0]=gy;
	gyro.mat[2][0]=gz;
	acc.mat[0][0]=ax;
	acc.mat[1][0]=ay;
	acc.mat[2][0]=az;

	//根据位置更新地球模型。为了减小计算量，以后可以降低这一段的更新频率。
	CalEarthModel();

	//准备，补偿传感器误差。注意方向
	acc1  = acc + AccBias;
	gyro1 = gyro + GyroBias;

	// 计算姿态
	wien = Wien();
	wenn = Wenn();
	Mat cbn = Quat2DCM(qa);
	//扣除地球自转、扣除速度引起的角速度之后，在b系的转动角速度
	wnbb = gyro1-(~cbn)*(wien+wenn);
	qa = QuatAttUpdate(qa,wnbb*dTins);//更新姿态


	// 计算速度
	AccN = cbn*acc1;//更新这个数，以便于卡尔曼滤波的部分使用
	gn.Init(3,1,0);
	gn.mat[2][0]=(-ge);

	// 更新导航系速度的微分 
	Mat DVn = AccN - ((wien + wien + wenn)^tspeed) + gn;
	tspeed = tspeed + dTins * DVn;//更新速度

	// 更新位置
	dpos.Init(3,1,0);
	double H = tpos.mat[2][0];
	dpos.mat[0][0] = tspeed.mat[1][0] / (Rmeri + H);//北向速度得到纬度
	dpos.mat[1][0] = tspeed.mat[0][0] / ((Rprim + H)*cos(tpos.mat[0][0]));//东向速度得到经度
	dpos.mat[2][0] = tspeed.mat[2][0];
	tpos=tpos+dTins*dpos;
}


Mat Phi(15,15,1);
Mat Pk(15,15,0);
Mat Eye(15,15,1);
Mat Q(15,15,0);
Mat Q0(15,15,0);
Mat R(6,6,1);


void Kalman_Init()//调参数
{
	dTins=0.005;

	Pk.mat[0][0]=1e-12;
	Pk.mat[1][1]=1e-12;
	Pk.mat[2][2]=9;
	Pk.mat[3][3]=0.1;
	Pk.mat[4][4]=0.1;
	Pk.mat[5][5]=0.1;
	Pk.mat[6][6]=3e-4;
	Pk.mat[7][7]=3e-4;
	Pk.mat[8][8]=3e-4;

	Q0.mat[3][3]=1e-4;
	Q0.mat[4][4]=1e-4;
	Q0.mat[5][5]=1e-4;
	Q0.mat[6][6]=1e-10;
	Q0.mat[7][7]=1e-10;
	Q0.mat[8][8]=1e-10;

	R.mat[0][0]=1e-12;
	R.mat[1][1]=1e-12;
	R.mat[2][2]=9;
	R.mat[3][3]=0.1;
	R.mat[4][4]=0.1;
	R.mat[5][5]=0.1;


	qa=EulerDeg2Quat(0.5,-0.4,0.3);
	tpos.mat[0][0] = 40*deg2rad;
	tpos.mat[1][0] = 120*deg2rad;
	tpos.mat[2][0] = 500;
}

Mat StateUpdate(Mat Z,Mat H)
{
	Mat Pkk = Phi*Pk*(~Phi)+Q;
	Mat K = Pkk*(~H)/(H*Pkk*(~H)+R);
	// 因为每次滤波之后补偿了误差，所以状态预测总是0
	Mat X = K*Z;
	
	// 更新协方差
	Mat IKH = Eye-K*H;
	Pk = IKH*Pkk*(~IKH)+K*R*(~K);
	
	Phi.Init(15,15,1);
	return X;
}


void StateCorrectUpdate(double lat,double lon,double height,double Ve,double Vn ,double Vu)
{
	Mat Z(6,1,0);
	Z.mat[0][0] = tpos.mat[0][0] - lat;
	Z.mat[1][0] = tpos.mat[1][0] - lon;
	Z.mat[2][0] = tpos.mat[2][0] - height;
	Z.mat[3][0] = tspeed.mat[0][0] - Ve;
	Z.mat[4][0] = tspeed.mat[1][0] - Vn;
	Z.mat[5][0] = tspeed.mat[2][0] - Vu;

	Mat H(6,15,1);

	Mat Pkk = Phi * Pk * (~Phi) + Q;
	Mat K = Pkk * (~H) / (H * Pkk * (~H) + R);
	// 因为每次滤波之后补偿了误差，所以状态预测总是0
	Mat X = K * Z;

	// 更新协方差
	Mat IKH = Eye - K * H;
	Pk = IKH * Pkk * (~IKH) + K * R * (~K);

	Phi.Init(15, 15, 1);
	Q.Init(15,15,0);

	tpos = tpos-X.SubMat(0,0,3,1);
	tspeed = tspeed-X.SubMat(3,0,3,1);
	qa = QuatAttUpdate(qa,(~Quat2DCM(qa))*X.SubMat(6,0,3,1));
	//biasgyro=biasgyro-X.submat(9,0,3,1);
	//biasacc=biasacc-X.submat(12,0,3,1);
}


//为了防止堆栈溢出，所以不要放到函数中。
Mat Fpp(3, 3, 0);
Mat Fvp(3, 3, 0);
Mat Fpv(3, 3, 0);
Mat Fvv(3, 3, 0);
Mat Fav(3, 3, 0);
Mat Fpa(3, 3, 0);
Mat Fva(3, 3, 0);
Mat Faa(3, 3, 0);

Mat GetF()
{
	/*
	误差的顺序定义为：纬经高，东北天速度，东北天姿态，三轴陀螺仪零偏，三轴加速度计零偏。
	纬经需要以弧度为单位。
	*/

	Mat F(15,15,0);
	
	double RpH1=1.0/(Rprim+tpos.mat[2][0]);//1/(卯酉圈+高）
	double RmH1=1.0/(Rmeri+tpos.mat[2][0]);//1/(子午圈+高）

	double cosphi=cos(tpos.mat[0][0]);
	double secphi=1.0/cosphi;
	double sinphi=sin(tpos.mat[0][0]);
	double tanphi=tan(tpos.mat[0][0]);

	double vE=tspeed.mat[0][0];
	double vN=tspeed.mat[1][0];
	double vU=tspeed.mat[2][0];

	//位置对位置影响的子矩阵	
	Fpp.mat[0][2]=RmH1*RmH1*(-vN);//高度对纬度的影响，与北向速度有关
	Fpp.mat[1][0]=RpH1*vE*secphi*tanphi;//纬度对经度的影响，与纬度有关，与东向速度也有关。
	Fpp.mat[1][2]=RpH1*RpH1*(-vE)*secphi;//高度对经度的影响，与东向速度有关，与纬度有关。实际就是影响局部的横切面半径。
	F.FillSubMat(0,0,Fpp);
	
	//速度对位置影响的子矩阵	
	Fvp.mat[0][1]=RmH1;//北向速度对纬度影响
	Fvp.mat[1][0]=RpH1*secphi;//东向速度对经度的影响
	Fvp.mat[2][2]=1;//天向速度对高度的影响
	F.FillSubMat(0,3,Fvp);
	

	//位置对速度影响的子矩阵	
	Fpv.mat[0][0]=2*we*cosphi*vN+2*we*sinphi*vU+vN*vE*RpH1*secphi*secphi;//纬度对东向速度的影响
	Fpv.mat[0][2]=RpH1*RpH1*(vE*vU-vN*vE*tanphi);//高度对东向速度的影响
	Fpv.mat[1][0]=(-(2*vE*we*cosphi+vE*vE*RpH1*secphi*secphi));//纬度对北向速度的影响
	Fpv.mat[1][2]=RmH1*RmH1*vN*vU+RpH1*RpH1*vE*vE*tanphi;//高度对北向速度的影响
	Fpv.mat[2][0]=(-2.0)*vE*we*sinphi;//纬度对天向速度的影响
	Fpv.mat[2][2]=(-RmH1*RmH1*vN*vN-RpH1*RpH1*vE*vE);//高度对天向速度的影响
	F.FillSubMat(3,0,Fpv);

	//速度对速度影响的子矩阵	
	Fvv.mat[0][0]=(vN*tanphi-vU)*RpH1;//东向速度对东向速度的影响
	Fvv.mat[0][1]=2.0*we*sinphi+vE*RpH1*tanphi;//北向速度对东向速度的影响
	Fvv.mat[0][2]=(-2.0)*we*cosphi-vE*RpH1;//天向速度对东向速度的影响
	Fvv.mat[1][0]=(-2.0)*(we*sinphi+vE*RpH1*tanphi);//东向速度对北向速度的影响
	Fvv.mat[1][1]=(-RmH1)*vU;//北向速度对北向速度的影响
	Fvv.mat[1][2]=(-RmH1)*vN;//天向速度对北向速度的影响
	Fvv.mat[2][0]=2.0*(we*cosphi+vE*RpH1);//东向速度对天向速度的影响//有的书公式写错了
	Fvv.mat[2][1]=2*vN*RmH1;//北向速度对天向速度的影响
	F.FillSubMat(3,3,Fvv);

	//姿态对速度影响的子矩阵
	double fE=AccN.mat[0][0];
	double fN=AccN.mat[1][0];
	double fU=AccN.mat[2][0];
	
	
	Fav.mat[0][1]=(-fU);//北向角度对东向速度的影响
	Fav.mat[0][2]=fN;//天向角度对东向速度的影响
	Fav.mat[1][0]=fU;//东向角度对北向速度的影响
	Fav.mat[1][2]=(-fE);//天向角度对北向速度的影响
	Fav.mat[2][0]=(-fN);//东向角度对天向速度的影响
	Fav.mat[2][1]=fE;//北向角度对天向速度的影响
	F.FillSubMat(3,6,Fav);


	//位置对姿态影响的子矩阵	
	Fpa.mat[0][2]=vN*RmH1*RmH1;//高度对东向姿态的影响
	Fpa.mat[1][0]=(-we)*sinphi;//纬度对北向姿态的影响
	Fpa.mat[1][2]=(-vE)*RpH1*RpH1;//高度对北向姿态的影响
	Fpa.mat[2][0]=we*cosphi+vE*RpH1*secphi*secphi;//纬度对天向姿态的影响
	Fpa.mat[2][2]=(-vE)*tanphi*RpH1*RpH1;//高度对天向姿态的影响
	F.FillSubMat(6,0,Fpa);
	

	//速度对姿态影响的子矩阵	
	Fva.mat[0][1]=(-RmH1);//北向速度对东向姿态的影响
	Fva.mat[1][0]=RpH1;//东向速度对北向姿态的影响
	Fva.mat[2][0]=RpH1*tanphi;//东向速度对天向姿态的影响
	F.FillSubMat(6,3,Fva);

	//姿态对姿态影响的子矩阵	
	Faa.mat[0][1]=we*sinphi+vE*RpH1*tanphi;//北向姿态对东向姿态的影响
	Faa.mat[2][0]=we*cosphi+vE*RpH1;//东向姿态对天向姿态的影响
	Faa.mat[2][1]=vN*RmH1;//北向姿态对天向姿态的影响
	Faa.mat[0][2]=(-Faa.mat[2][0]);//天向姿态对东向姿态的影响
	Faa.mat[1][0]=(-Faa.mat[0][1]);//东向姿态对北向姿态的影响
	Faa.mat[1][2]=(-Faa.mat[2][1]);//天向姿态对北向姿态的影响
	F.FillSubMat(6,6,Faa);
	
	Mat cbn = Quat2DCM(qa);
	// 陀螺仪零偏对姿态的影响。有的书把这个公式写错了SB
	F.FillSubMat(6,9,(-1)*cbn);
	// 加速度计零偏对速度的影响
	F.FillSubMat(3,12,cbn);
	
	return F;
}


void StatePredict()
{
	Phi = (Eye + GetF() * dTins) * Phi;
	Q = Q + Q0*dTins;
}
