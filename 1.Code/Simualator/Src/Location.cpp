/******************** (C) COPYRIGHT 2022 Geek************************************
* File Name          : Nav.cpp
* Current Version    : V1.0
* Author             : logzhan
* Date of Issued     : 2022.11.08
* Comments           : ��ϵ���λ���ں�(ENU ����������ϵ)
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
	wien.mat[2][0] = sin(latitude)*we;//��
	return wien;
}

Mat Wenn()
{
	//�����ٶȻ�������ת���������ٶȻ������������ת��
	double lat = tpos.mat[0][0];
	double H   = tpos.mat[2][0];
	double vE  = tspeed.mat[0][0];
	double vN  = tspeed.mat[1][0];
	
	Mat wen(3,1,0);
	// ��
	wen.mat[0][0] = (-vN/(Rmeri + H));
	// ��
	wen.mat[1][0] = vE/(Rprim + H);
	// ��
	wen.mat[2][0] = wen.mat[1][0]*tan(lat);
	
	return wen;
}

void CalEarthModel()
{
	const double Re=6378137;
	const double ee=1/298.25722;

	double lat=tpos.mat[0][0];//γ��
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


//Ϊ�˷�ֹ��ջ��������Բ�Ҫ�ŵ������С�
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

	//����λ�ø��µ���ģ�͡�Ϊ�˼�С���������Ժ���Խ�����һ�εĸ���Ƶ�ʡ�
	CalEarthModel();

	//׼����������������ע�ⷽ��
	acc1  = acc + AccBias;
	gyro1 = gyro + GyroBias;

	// ������̬
	wien = Wien();
	wenn = Wenn();
	Mat cbn = Quat2DCM(qa);
	//�۳�������ת���۳��ٶ�����Ľ��ٶ�֮����bϵ��ת�����ٶ�
	wnbb = gyro1-(~cbn)*(wien+wenn);
	qa = QuatAttUpdate(qa,wnbb*dTins);//������̬


	// �����ٶ�
	AccN = cbn*acc1;//������������Ա��ڿ������˲��Ĳ���ʹ��
	gn.Init(3,1,0);
	gn.mat[2][0]=(-ge);

	// ���µ���ϵ�ٶȵ�΢�� 
	Mat DVn = AccN - ((wien + wien + wenn)^tspeed) + gn;
	tspeed = tspeed + dTins * DVn;//�����ٶ�

	// ����λ��
	dpos.Init(3,1,0);
	double H = tpos.mat[2][0];
	dpos.mat[0][0] = tspeed.mat[1][0] / (Rmeri + H);//�����ٶȵõ�γ��
	dpos.mat[1][0] = tspeed.mat[0][0] / ((Rprim + H)*cos(tpos.mat[0][0]));//�����ٶȵõ�����
	dpos.mat[2][0] = tspeed.mat[2][0];
	tpos=tpos+dTins*dpos;
}


Mat Phi(15,15,1);
Mat Pk(15,15,0);
Mat Eye(15,15,1);
Mat Q(15,15,0);
Mat Q0(15,15,0);
Mat R(6,6,1);


void Kalman_Init()//������
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
	// ��Ϊÿ���˲�֮�󲹳���������״̬Ԥ������0
	Mat X = K*Z;
	
	// ����Э����
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
	// ��Ϊÿ���˲�֮�󲹳���������״̬Ԥ������0
	Mat X = K * Z;

	// ����Э����
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


//Ϊ�˷�ֹ��ջ��������Բ�Ҫ�ŵ������С�
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
	����˳����Ϊ��γ���ߣ��������ٶȣ���������̬��������������ƫ��������ٶȼ���ƫ��
	γ����Ҫ�Ի���Ϊ��λ��
	*/

	Mat F(15,15,0);
	
	double RpH1=1.0/(Rprim+tpos.mat[2][0]);//1/(î��Ȧ+�ߣ�
	double RmH1=1.0/(Rmeri+tpos.mat[2][0]);//1/(����Ȧ+�ߣ�

	double cosphi=cos(tpos.mat[0][0]);
	double secphi=1.0/cosphi;
	double sinphi=sin(tpos.mat[0][0]);
	double tanphi=tan(tpos.mat[0][0]);

	double vE=tspeed.mat[0][0];
	double vN=tspeed.mat[1][0];
	double vU=tspeed.mat[2][0];

	//λ�ö�λ��Ӱ����Ӿ���	
	Fpp.mat[0][2]=RmH1*RmH1*(-vN);//�߶ȶ�γ�ȵ�Ӱ�죬�뱱���ٶ��й�
	Fpp.mat[1][0]=RpH1*vE*secphi*tanphi;//γ�ȶԾ��ȵ�Ӱ�죬��γ���йأ��붫���ٶ�Ҳ�йء�
	Fpp.mat[1][2]=RpH1*RpH1*(-vE)*secphi;//�߶ȶԾ��ȵ�Ӱ�죬�붫���ٶ��йأ���γ���йء�ʵ�ʾ���Ӱ��ֲ��ĺ�����뾶��
	F.FillSubMat(0,0,Fpp);
	
	//�ٶȶ�λ��Ӱ����Ӿ���	
	Fvp.mat[0][1]=RmH1;//�����ٶȶ�γ��Ӱ��
	Fvp.mat[1][0]=RpH1*secphi;//�����ٶȶԾ��ȵ�Ӱ��
	Fvp.mat[2][2]=1;//�����ٶȶԸ߶ȵ�Ӱ��
	F.FillSubMat(0,3,Fvp);
	

	//λ�ö��ٶ�Ӱ����Ӿ���	
	Fpv.mat[0][0]=2*we*cosphi*vN+2*we*sinphi*vU+vN*vE*RpH1*secphi*secphi;//γ�ȶԶ����ٶȵ�Ӱ��
	Fpv.mat[0][2]=RpH1*RpH1*(vE*vU-vN*vE*tanphi);//�߶ȶԶ����ٶȵ�Ӱ��
	Fpv.mat[1][0]=(-(2*vE*we*cosphi+vE*vE*RpH1*secphi*secphi));//γ�ȶԱ����ٶȵ�Ӱ��
	Fpv.mat[1][2]=RmH1*RmH1*vN*vU+RpH1*RpH1*vE*vE*tanphi;//�߶ȶԱ����ٶȵ�Ӱ��
	Fpv.mat[2][0]=(-2.0)*vE*we*sinphi;//γ�ȶ������ٶȵ�Ӱ��
	Fpv.mat[2][2]=(-RmH1*RmH1*vN*vN-RpH1*RpH1*vE*vE);//�߶ȶ������ٶȵ�Ӱ��
	F.FillSubMat(3,0,Fpv);

	//�ٶȶ��ٶ�Ӱ����Ӿ���	
	Fvv.mat[0][0]=(vN*tanphi-vU)*RpH1;//�����ٶȶԶ����ٶȵ�Ӱ��
	Fvv.mat[0][1]=2.0*we*sinphi+vE*RpH1*tanphi;//�����ٶȶԶ����ٶȵ�Ӱ��
	Fvv.mat[0][2]=(-2.0)*we*cosphi-vE*RpH1;//�����ٶȶԶ����ٶȵ�Ӱ��
	Fvv.mat[1][0]=(-2.0)*(we*sinphi+vE*RpH1*tanphi);//�����ٶȶԱ����ٶȵ�Ӱ��
	Fvv.mat[1][1]=(-RmH1)*vU;//�����ٶȶԱ����ٶȵ�Ӱ��
	Fvv.mat[1][2]=(-RmH1)*vN;//�����ٶȶԱ����ٶȵ�Ӱ��
	Fvv.mat[2][0]=2.0*(we*cosphi+vE*RpH1);//�����ٶȶ������ٶȵ�Ӱ��//�е��鹫ʽд����
	Fvv.mat[2][1]=2*vN*RmH1;//�����ٶȶ������ٶȵ�Ӱ��
	F.FillSubMat(3,3,Fvv);

	//��̬���ٶ�Ӱ����Ӿ���
	double fE=AccN.mat[0][0];
	double fN=AccN.mat[1][0];
	double fU=AccN.mat[2][0];
	
	
	Fav.mat[0][1]=(-fU);//����ǶȶԶ����ٶȵ�Ӱ��
	Fav.mat[0][2]=fN;//����ǶȶԶ����ٶȵ�Ӱ��
	Fav.mat[1][0]=fU;//����ǶȶԱ����ٶȵ�Ӱ��
	Fav.mat[1][2]=(-fE);//����ǶȶԱ����ٶȵ�Ӱ��
	Fav.mat[2][0]=(-fN);//����Ƕȶ������ٶȵ�Ӱ��
	Fav.mat[2][1]=fE;//����Ƕȶ������ٶȵ�Ӱ��
	F.FillSubMat(3,6,Fav);


	//λ�ö���̬Ӱ����Ӿ���	
	Fpa.mat[0][2]=vN*RmH1*RmH1;//�߶ȶԶ�����̬��Ӱ��
	Fpa.mat[1][0]=(-we)*sinphi;//γ�ȶԱ�����̬��Ӱ��
	Fpa.mat[1][2]=(-vE)*RpH1*RpH1;//�߶ȶԱ�����̬��Ӱ��
	Fpa.mat[2][0]=we*cosphi+vE*RpH1*secphi*secphi;//γ�ȶ�������̬��Ӱ��
	Fpa.mat[2][2]=(-vE)*tanphi*RpH1*RpH1;//�߶ȶ�������̬��Ӱ��
	F.FillSubMat(6,0,Fpa);
	

	//�ٶȶ���̬Ӱ����Ӿ���	
	Fva.mat[0][1]=(-RmH1);//�����ٶȶԶ�����̬��Ӱ��
	Fva.mat[1][0]=RpH1;//�����ٶȶԱ�����̬��Ӱ��
	Fva.mat[2][0]=RpH1*tanphi;//�����ٶȶ�������̬��Ӱ��
	F.FillSubMat(6,3,Fva);

	//��̬����̬Ӱ����Ӿ���	
	Faa.mat[0][1]=we*sinphi+vE*RpH1*tanphi;//������̬�Զ�����̬��Ӱ��
	Faa.mat[2][0]=we*cosphi+vE*RpH1;//������̬��������̬��Ӱ��
	Faa.mat[2][1]=vN*RmH1;//������̬��������̬��Ӱ��
	Faa.mat[0][2]=(-Faa.mat[2][0]);//������̬�Զ�����̬��Ӱ��
	Faa.mat[1][0]=(-Faa.mat[0][1]);//������̬�Ա�����̬��Ӱ��
	Faa.mat[1][2]=(-Faa.mat[2][1]);//������̬�Ա�����̬��Ӱ��
	F.FillSubMat(6,6,Faa);
	
	Mat cbn = Quat2DCM(qa);
	// ��������ƫ����̬��Ӱ�졣�е���������ʽд����SB
	F.FillSubMat(6,9,(-1)*cbn);
	// ���ٶȼ���ƫ���ٶȵ�Ӱ��
	F.FillSubMat(3,12,cbn);
	
	return F;
}


void StatePredict()
{
	Phi = (Eye + GetF() * dTins) * Phi;
	Q = Q + Q0*dTins;
}
