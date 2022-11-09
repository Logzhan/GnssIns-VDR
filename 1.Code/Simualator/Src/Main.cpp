/******************** (C) COPYRIGHT 2022 Geek************************************
* File Name          : Main.cpp
* Current Version    : V1.0
* Author             : logzhan
* Date of Issued     : 2022.11.08
* Comments           : ��ϵ����㷨(ENU ����������ϵ)
********************************************************************************/
/* Header File Including -----------------------------------------------------*/
#define _CRT_SECURE_NO_WARNINGS
#include "GnssInsDef.h"
#include "Location.h"
#include "Utils.h"
#include <stdio.h>
#include <stdlib.h>


int main()
{
	int L = 100000;
	FILE* fp;
	FILE* fpin;
	Mat ou;
	IMU_t  imu;
	Gnss_t gnss;

	char fin[1000];

	Kalman_Init();//���ò���

	dTins = 0.004;
	//���ó�ֵ����̬���ٶȡ�λ��
	qa = EulerDeg2Quat(-143.26, 75.69, 12.2);
	tspeed.mat[0][0] = (-37.8472);
	tspeed.mat[1][0] = (-50.5556);
	tspeed.mat[2][0] = 0.0694;
	tpos.mat[0][0] = 0.657102175971747;
	tpos.mat[1][0] = 1.895330468487405;
	tpos.mat[2][0] = 3765;

	fpin = fopen("datain.txt", "r");//��������
	fp = fopen("dataout.txt", "w");//�������

	for (int k = 1; k <= L; k++)
	{
		if (fgets(fin, 1000, fpin) <= 0){
			break;
		}

		int flag = sscanf(fin, "%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf", 
			&imu.gyr.s[0], &imu.gyr.s[1], &imu.gyr.s[2],
			&imu.acc.s[0], &imu.acc.s[1], &imu.acc.s[2],
			&gnss.lat, &gnss.lon, &gnss.height, 
			&gnss.ve, &gnss.vn, &gnss.vu);

		if (!flag) {
			continue;
		}
		// ����״̬����
		InsStateUpdate(imu.gyr.s[0], imu.gyr.s[1], imu.gyr.s[2], 
					   imu.acc.s[0], imu.acc.s[1], imu.acc.s[2]);
		// ״̬����
		StatePredict();

		if ((k % 20) == 0)//��ϵ���
		{
			StateCorrectUpdate(gnss.lat, gnss.lon, gnss.height, gnss.ve, gnss.vn, gnss.vu);
		}

		//��¼����
		ou = Quat2Euler(qa);
		fprintf(fp, "%12.6lf,%12.6lf,%12.6lf", ou.mat[0][0], ou.mat[1][0], ou.mat[2][0]);
		fprintf(fp, "%12.6lf,%12.6lf,%12.6lf", tspeed.mat[0][0], tspeed.mat[1][0], tspeed.mat[2][0]);
		fprintf(fp, "%18.9lf,%18.9lf,%12.6lf\n", tpos.mat[0][0], tpos.mat[1][0], tpos.mat[2][0]);

	}
	fclose(fpin);
	fclose(fp);
	return 0;
}
