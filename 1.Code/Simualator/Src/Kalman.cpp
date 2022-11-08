#include "Kalman.h"
#include "Mat.h"
#include "Utils.h"
static Mat Phi(15, 15, 1);
static Mat Pk(15, 15, 0);
static Mat Eye(15, 15, 1);
static Mat Q(15, 15, 0);
static Mat Q0(15, 15, 0);
static Mat R(6, 6, 1);
//Ϊ�˷�ֹ��ջ��������Բ�Ҫ�ŵ������С�
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
* Description : ��ʼ���������˲������
* Date        : 2022/09/21 logzhan
*---------------------------------------------------------------------**/
void EKF_Init(void) {

}


/**----------------------------------------------------------------------
* Function    : EKFCalQRMatrix
* Description : ����GPS�ź����������������˲���������q�����ǹ�����������
*               ��Ҫ���ߵ�Ԥ��λ�þ�����ء�r����ΪGNSS�۲���������GPS�����
*               ��Ϣ�����йء�
* Date        : 2022/09/21 logzhan
*---------------------------------------------------------------------**/
void EKFCalQRMatrix() {

}


static Mat GetF()
{
	/*
	����˳����Ϊ��γ���ߣ��������ٶȣ���������̬��������������ƫ��������ٶȼ���ƫ��
	γ����Ҫ�Ի���Ϊ��λ��
	*/
	Mat F(15, 15, 0);

	//double RpH1 = 1.0 / (Rprim + tpos.num[2][0]);//1/(î��Ȧ+�ߣ�
	//double RmH1 = 1.0 / (Rmeri + tpos.num[2][0]);//1/(����Ȧ+�ߣ�

	//double cosphi = cos(tpos.num[0][0]);
	//double secphi = 1.0 / cosphi;
	//double sinphi = sin(tpos.num[0][0]);
	//double tanphi = tan(tpos.num[0][0]);

	//double vE = tspeed.num[0][0];
	//double vN = tspeed.num[1][0];
	//double vU = tspeed.num[2][0];

	////λ�ö�λ��Ӱ����Ӿ���	
	//Fpp.num[0][2] = RmH1 * RmH1 * (-vN);//�߶ȶ�γ�ȵ�Ӱ�죬�뱱���ٶ��й�
	//Fpp.num[1][0] = RpH1 * vE * secphi * tanphi;//γ�ȶԾ��ȵ�Ӱ�죬��γ���йأ��붫���ٶ�Ҳ�йء�
	//Fpp.num[1][2] = RpH1 * RpH1 * (-vE) * secphi;//�߶ȶԾ��ȵ�Ӱ�죬�붫���ٶ��йأ���γ���йء�ʵ�ʾ���Ӱ��ֲ��ĺ�����뾶��
	//F.FillSubMat(0, 0, Fpp);

	////�ٶȶ�λ��Ӱ����Ӿ���	
	//Fvp.num[0][1] = RmH1;//�����ٶȶ�γ��Ӱ��
	//Fvp.num[1][0] = RpH1 * secphi;//�����ٶȶԾ��ȵ�Ӱ��
	//Fvp.num[2][2] = 1;//�����ٶȶԸ߶ȵ�Ӱ��
	//F.FillSubMat(0, 3, Fvp);


	////λ�ö��ٶ�Ӱ����Ӿ���	
	//Fpv.num[0][0] = 2 * we * cosphi * vN + 2 * we * sinphi * vU + vN * vE * RpH1 * secphi * secphi;//γ�ȶԶ����ٶȵ�Ӱ��
	//Fpv.num[0][2] = RpH1 * RpH1 * (vE * vU - vN * vE * tanphi);//�߶ȶԶ����ٶȵ�Ӱ��
	//Fpv.num[1][0] = (-(2 * vE * we * cosphi + vE * vE * RpH1 * secphi * secphi));//γ�ȶԱ����ٶȵ�Ӱ��
	//Fpv.num[1][2] = RmH1 * RmH1 * vN * vU + RpH1 * RpH1 * vE * vE * tanphi;//�߶ȶԱ����ٶȵ�Ӱ��
	//Fpv.num[2][0] = (-2.0) * vE * we * sinphi;//γ�ȶ������ٶȵ�Ӱ��
	//Fpv.num[2][2] = (-RmH1 * RmH1 * vN * vN - RpH1 * RpH1 * vE * vE);//�߶ȶ������ٶȵ�Ӱ��
	//F.FillSubMat(3, 0, Fpv);

	////�ٶȶ��ٶ�Ӱ����Ӿ���	
	//Fvv.num[0][0] = (vN * tanphi - vU) * RpH1;//�����ٶȶԶ����ٶȵ�Ӱ��
	//Fvv.num[0][1] = 2.0 * we * sinphi + vE * RpH1 * tanphi;//�����ٶȶԶ����ٶȵ�Ӱ��
	//Fvv.num[0][2] = (-2.0) * we * cosphi - vE * RpH1;//�����ٶȶԶ����ٶȵ�Ӱ��
	//Fvv.num[1][0] = (-2.0) * (we * sinphi + vE * RpH1 * tanphi);//�����ٶȶԱ����ٶȵ�Ӱ��
	//Fvv.num[1][1] = (-RmH1) * vU;//�����ٶȶԱ����ٶȵ�Ӱ��
	//Fvv.num[1][2] = (-RmH1) * vN;//�����ٶȶԱ����ٶȵ�Ӱ��
	//Fvv.num[2][0] = 2.0 * (we * cosphi + vE * RpH1);//�����ٶȶ������ٶȵ�Ӱ��//�е��鹫ʽд����
	//Fvv.num[2][1] = 2 * vN * RmH1;//�����ٶȶ������ٶȵ�Ӱ��
	//F.FillSubMat(3, 3, Fvv);

	////��̬���ٶ�Ӱ����Ӿ���
	//double fE = accn.num[0][0];
	//double fN = accn.num[1][0];
	//double fU = accn.num[2][0];


	//Fav.num[0][1] = (-fU);//����ǶȶԶ����ٶȵ�Ӱ��
	//Fav.num[0][2] = fN;//����ǶȶԶ����ٶȵ�Ӱ��
	//Fav.num[1][0] = fU;//����ǶȶԱ����ٶȵ�Ӱ��
	//Fav.num[1][2] = (-fE);//����ǶȶԱ����ٶȵ�Ӱ��
	//Fav.num[2][0] = (-fN);//����Ƕȶ������ٶȵ�Ӱ��
	//Fav.num[2][1] = fE;//����Ƕȶ������ٶȵ�Ӱ��
	//F.FillSubMat(3, 6, Fav);


	////λ�ö���̬Ӱ����Ӿ���	
	//Fpa.num[0][2] = vN * RmH1 * RmH1;//�߶ȶԶ�����̬��Ӱ��
	//Fpa.num[1][0] = (-we) * sinphi;//γ�ȶԱ�����̬��Ӱ��
	//Fpa.num[1][2] = (-vE) * RpH1 * RpH1;//�߶ȶԱ�����̬��Ӱ��
	//Fpa.num[2][0] = we * cosphi + vE * RpH1 * secphi * secphi;//γ�ȶ�������̬��Ӱ��
	//Fpa.num[2][2] = (-vE) * tanphi * RpH1 * RpH1;//�߶ȶ�������̬��Ӱ��
	//F.FillSubMat(6, 0, Fpa);


	////�ٶȶ���̬Ӱ����Ӿ���	
	//Fva.num[0][1] = (-RmH1);//�����ٶȶԶ�����̬��Ӱ��
	//Fva.num[1][0] = RpH1;//�����ٶȶԱ�����̬��Ӱ��
	//Fva.num[2][0] = RpH1 * tanphi;//�����ٶȶ�������̬��Ӱ��
	//F.FillSubMat(6, 3, Fva);

	////��̬����̬Ӱ����Ӿ���	
	//Faa.num[0][1] = we * sinphi + vE * RpH1 * tanphi;//������̬�Զ�����̬��Ӱ��
	//Faa.num[2][0] = we * cosphi + vE * RpH1;//������̬��������̬��Ӱ��
	//Faa.num[2][1] = vN * RmH1;//������̬��������̬��Ӱ��
	//Faa.num[0][2] = (-Faa.num[2][0]);//������̬�Զ�����̬��Ӱ��
	//Faa.num[1][0] = (-Faa.num[0][1]);//������̬�Ա�����̬��Ӱ��
	//Faa.num[1][2] = (-Faa.num[2][1]);//������̬�Ա�����̬��Ӱ��
	//F.FillSubMat(6, 6, Faa);

	//Mat cbn = Quat2DCM(qa);
	//// ��������ƫ����̬��Ӱ�졣�е���������ʽд����SB
	//F.FillSubMat(6, 9, (-1) * cbn);
	//// ���ٶȼ���ƫ���ٶȵ�Ӱ��
	//F.FillSubMat(3, 12, cbn);

	return F;
}

/**----------------------------------------------------------------------
* Function    : StateUpdate
* Description : ��չ�������˲�����״̬Ԥ��
* Date        : 2022/11/18 logzhan
*---------------------------------------------------------------------**/
static void StateUpdate() {

}

/**----------------------------------------------------------------------
* Function    : StatePredict
* Description : ��չ�������˲���״̬Ԥ��
* Date        : 2022/11/8 logzhan
*---------------------------------------------------------------------**/
static void StatePredict() {
	//Phi = (Eye + getF() * dTins) * Phi;
	//Q = Q + Q0 * dTins;
}