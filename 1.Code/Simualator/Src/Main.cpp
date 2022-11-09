/******************** (C) COPYRIGHT 2022 Geek************************************
* File Name          : Main.cpp
* Current Version    : V1.0
* Author             : logzhan
* Date of Issued     : 2022.11.08
* Comments           : 组合导航算法(ENU 东北天坐标系)
********************************************************************************/
/* Header File Including -----------------------------------------------------*/
#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "GnssInsDef.h"
#include "Kalman.h"
#include "Ins.h"
#include "Utils.h"
#include "KmlSupport.h"

//vector<KmlTracks_t> GnssInsTracks;

void GnssIns_Init(GnssIns_t& gins) {
	memset(&gins, 0, sizeof(gins));
	// Ins更新250Hz
	gins.dt = 0.004;

	gins.qbn.Init(4, 1, 1);
	gins.vel.Init(4, 1, 1);
	gins.pos.Init(4, 1, 1);

	// 卡尔曼滤波器初始化
	Kalman_Init(gins);
}

int main()
{
	int L = 100000;
	FILE* fp;
	FILE* fpin;
	Mat att;
	IMU_t  imu;
	Gnss_t gnss;
	GnssIns_t gins;

	char fin[1000];

	GnssIns_Init(gins);

	//设置初值，姿态、速度、位置
	gins.qbn = EulerDeg2Quat(-143.26, 75.69, 12.2);

	gins.vel.mat[0][0] = (-37.8472);
	gins.vel.mat[1][0] = (-50.5556);
	gins.vel.mat[2][0] = 0.0694;

	gins.pos.mat[0][0] = 0.657102175971747;
	gins.pos.mat[1][0] = 1.895330468487405;
	gins.pos.mat[2][0] = 3765;

	fpin = fopen("datain.txt", "r");//输入数据
	fp = fopen("dataout.txt", "w");//输出数据

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
		// 惯性状态更新
		InsStateUpdate(imu, gins);
		// 状态更新
		StatePredict(gins);
		// 组合导航位置优化
		if ((k % 20) == 0){
			// gnss参考信息融合
			StateUpdate(gnss, gins);
			// 结果保存
			//KmlTracks_t points;
			//memset(&points, 0, sizeof(points));
			//points.GpsLat = gnss.lat * rad2deg;
			//points.GpsLon = gnss.lon * rad2deg;
			//points.lat    = tpos.mat[0][0] * rad2deg;
			//points.lon    = tpos.mat[1][0] * rad2deg;
			//GnssInsTracks.push_back(points);
		}
		
		//记录数据
		att = Quat2Euler(gins.qbn);
		fprintf(fp, "%12.6lf,%12.6lf,%12.6lf", att.mat[0][0], att.mat[1][0], att.mat[2][0]);
		fprintf(fp, "%12.6lf,%12.6lf,%12.6lf", gins.vel.mat[0][0], gins.vel.mat[1][0], gins.vel.mat[2][0]);
		fprintf(fp, "%18.9lf,%18.9lf,%12.6lf\n", gins.pos.mat[0][0], gins.pos.mat[1][0], gins.pos.mat[2][0]);
	}
	//KmlWrite("./", "Result", "_ver1.0", GnssInsTracks);
	fclose(fpin);
	fclose(fp);
	return 0;
}
