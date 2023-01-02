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
* Function    : Kalman_Init
* Description : ��ʼ���������˲������
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
* Description : ������״̬�����Լ�״̬����
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
	// ��Ϊÿ���˲�֮�󲹳���������״̬Ԥ������0
	Mat X = K * Z;

	// ����Э����
	Mat IKH = Eye - K * H;
	Pk = IKH * Pkk * (~IKH) + K * R * (~K);

	Phi.Init(15, 15, 1);
	Q.Init(15, 15, 0);
	// λ������
	gins.pos = gins.pos - X.SubMat(0, 0, 3, 1);
	// �ٶ�����
	gins.vel = gins.vel - X.SubMat(3, 0, 3, 1);
	// ��̬����
	gins.qbn = QuatAttUpdate(gins.qbn, (~Quat2DCM(gins.qbn)) * X.SubMat(6, 0, 3, 1));
	// ��������ƫ����
	//biasgyro=biasgyro-X.submat(9,0,3,1);
	//biasacc=biasacc-X.submat(12,0,3,1);
}
/**----------------------------------------------------------------------
* Function    : GetF
* Description : ��ȡF����
* Date        : 2022/11/09 logzhan
*---------------------------------------------------------------------**/
Mat GetF(GnssIns_t& gins)
{
	/*
	����˳����Ϊ��γ���ߣ��������ٶȣ���������̬��������������ƫ��������ٶȼ���ƫ��
	γ����Ҫ�Ի���Ϊ��λ��
	*/
	Mat F(15, 15, 0);

	EarthPara_t EarthPara = CalEarthModel(gins.pos.mat[0][0], gins.pos.mat[2][0]);
	// 1/(î��Ȧ+�ߣ�
	double RpH1 = 1.0 / (EarthPara.Rp + gins.pos.mat[2][0]);
	// 1/(����Ȧ+�ߣ�
	double RmH1 = 1.0 / (EarthPara.Rm + gins.pos.mat[2][0]);

	double cosphi = cos(gins.pos.mat[0][0]);
	double secphi = 1.0 / cosphi;
	double sinphi = sin(gins.pos.mat[0][0]);
	double tanphi = tan(gins.pos.mat[0][0]);

	double vE = gins.vel.mat[0][0];
	double vN = gins.vel.mat[1][0];
	double vU = gins.vel.mat[2][0];

	// λ�ö�λ��Ӱ����Ӿ���	
	// �߶ȶ�γ�ȵ�Ӱ�죬�뱱���ٶ��й�
	Fpp.mat[0][2] = RmH1 * RmH1 * (-vN);
	// γ�ȶԾ��ȵ�Ӱ�죬��γ���йأ��붫���ٶ�Ҳ�йء�
	Fpp.mat[1][0] = RpH1 * vE * secphi * tanphi;
	// �߶ȶԾ��ȵ�Ӱ�죬�붫���ٶ��йأ���γ���йء�ʵ�ʾ���Ӱ��ֲ��ĺ�����뾶��
	Fpp.mat[1][2] = RpH1 * RpH1 * (-vE) * secphi;
	F.FillSubMat(0, 0, Fpp);

	// �ٶȶ�λ��Ӱ����Ӿ���	
	// �����ٶȶ�γ��Ӱ��
	Fvp.mat[0][1] = RmH1;
	// �����ٶȶԾ��ȵ�Ӱ��
	Fvp.mat[1][0] = RpH1 * secphi;
	// �����ٶȶԸ߶ȵ�Ӱ��
	Fvp.mat[2][2] = 1;
	F.FillSubMat(0, 3, Fvp);

	// λ�ö��ٶ�Ӱ����Ӿ���	
	// γ�ȶԶ����ٶȵ�Ӱ��
	Fpv.mat[0][0] = 2 * we * cosphi * vN + 2 * we * sinphi * vU + vN * vE * RpH1 * secphi * secphi;
	// �߶ȶԶ����ٶȵ�Ӱ��
	Fpv.mat[0][2] = RpH1 * RpH1 * (vE * vU - vN * vE * tanphi);
	// γ�ȶԱ����ٶȵ�Ӱ��
	Fpv.mat[1][0] = (-(2 * vE * we * cosphi + vE * vE * RpH1 * secphi * secphi));
	// �߶ȶԱ����ٶȵ�Ӱ��
	Fpv.mat[1][2] = RmH1 * RmH1 * vN * vU + RpH1 * RpH1 * vE * vE * tanphi;
	// γ�ȶ������ٶȵ�Ӱ��
	Fpv.mat[2][0] = (-2.0) * vE * we * sinphi;
	// �߶ȶ������ٶȵ�Ӱ��
	Fpv.mat[2][2] = (-RmH1 * RmH1 * vN * vN - RpH1 * RpH1 * vE * vE);
	F.FillSubMat(3, 0, Fpv);

	// �ٶȶ��ٶ�Ӱ����Ӿ���	
	Fvv.mat[0][0] = (vN * tanphi - vU) * RpH1;                   // �����ٶȶԶ����ٶȵ�Ӱ��
	Fvv.mat[0][1] = 2.0 * we * sinphi + vE * RpH1 * tanphi;      // �����ٶȶԶ����ٶȵ�Ӱ��
	Fvv.mat[0][2] = (-2.0) * we * cosphi - vE * RpH1;            // �����ٶȶԶ����ٶȵ�Ӱ��
	Fvv.mat[1][0] = (-2.0) * (we * sinphi + vE * RpH1 * tanphi); // �����ٶȶԱ����ٶȵ�Ӱ��
	Fvv.mat[1][1] = (-RmH1) * vU;                                // �����ٶȶԱ����ٶȵ�Ӱ��
	Fvv.mat[1][2] = (-RmH1) * vN;                                // �����ٶȶԱ����ٶȵ�Ӱ��
	Fvv.mat[2][0] = 2.0 * (we * cosphi + vE * RpH1);             // �����ٶȶ������ٶȵ�Ӱ��, �е��鹫ʽд����
	Fvv.mat[2][1] = 2 * vN * RmH1;                               // �����ٶȶ������ٶȵ�Ӱ��
	F.FillSubMat(3, 3, Fvv);

	// ��̬���ٶ�Ӱ����Ӿ���
	double fE = gins.fn.mat[0][0];
	double fN = gins.fn.mat[1][0];
	double fU = gins.fn.mat[2][0];

	// ����ǶȶԶ����ٶȵ�Ӱ��
	Fav.mat[0][1] = (-fU);  
	// ����ǶȶԶ����ٶȵ�Ӱ��
	Fav.mat[0][2] = fN;  
	// ����ǶȶԱ����ٶȵ�Ӱ��
	Fav.mat[1][0] = fU;   
	// ����ǶȶԱ����ٶȵ�Ӱ��
	Fav.mat[1][2] = (-fE);  
	// ����Ƕȶ������ٶȵ�Ӱ��
	Fav.mat[2][0] = (-fN);  
	// ����Ƕȶ������ٶȵ�Ӱ��
	Fav.mat[2][1] = fE;   
	F.FillSubMat(3, 6, Fav);


	// λ�ö���̬Ӱ����Ӿ���	
	// �߶ȶԶ�����̬��Ӱ��
	Fpa.mat[0][2] = vN * RmH1 * RmH1;
	// γ�ȶԱ�����̬��Ӱ��
	Fpa.mat[1][0] = (-we) * sinphi;
	// �߶ȶԱ�����̬��Ӱ��
	Fpa.mat[1][2] = (-vE) * RpH1 * RpH1;
	// γ�ȶ�������̬��Ӱ��
	Fpa.mat[2][0] = we * cosphi + vE * RpH1 * secphi * secphi;
	// �߶ȶ�������̬��Ӱ��
	Fpa.mat[2][2] = (-vE) * tanphi * RpH1 * RpH1;
	F.FillSubMat(6, 0, Fpa);


	// �ٶȶ���̬Ӱ����Ӿ���	
	Fva.mat[0][1] = (-RmH1);                          // �����ٶȶԶ�����̬��Ӱ��
	Fva.mat[1][0] = RpH1;                             // �����ٶȶԱ�����̬��Ӱ��
	Fva.mat[2][0] = RpH1 * tanphi;                    // �����ٶȶ�������̬��Ӱ��
	F.FillSubMat(6, 3, Fva);

	// ��̬����̬Ӱ����Ӿ���	
	Faa.mat[0][1] = we * sinphi + vE * RpH1 * tanphi; // ������̬�Զ�����̬��Ӱ��
	Faa.mat[2][0] = we * cosphi + vE * RpH1;          // ������̬��������̬��Ӱ��
	Faa.mat[2][1] = vN * RmH1;                        // ������̬��������̬��Ӱ��
	Faa.mat[0][2] = (-Faa.mat[2][0]);                 // ������̬�Զ�����̬��Ӱ��
	Faa.mat[1][0] = (-Faa.mat[0][1]);                 // ������̬�Ա�����̬��Ӱ��
	Faa.mat[1][2] = (-Faa.mat[2][1]);                 // ������̬�Ա�����̬��Ӱ��
	F.FillSubMat(6, 6, Faa);

	Mat cbn = Quat2DCM(gins.qbn);
	// ��������ƫ����̬��Ӱ�졣�е���������ʽд����SB
	F.FillSubMat(6, 9, (-1) * cbn);
	// ���ٶȼ���ƫ���ٶȵ�Ӱ��
	F.FillSubMat(3, 12, cbn);

	return F;
}
/**----------------------------------------------------------------------
* Function    : EkfStatePredict
* Description : ������״̬Ԥ��
* Date        : 2022/11/09 logzhan
*---------------------------------------------------------------------**/
void EkfStatePredict(GnssIns_t& gins)
{
	Phi = (Eye + GetF(gins) * gins.dt) * Phi;
	Q = Q + Q0 * gins.dt;
}