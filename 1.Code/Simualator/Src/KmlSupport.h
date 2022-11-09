#ifndef _VDR_KML_SUPPORT_H
#define _VDR_KML_SUPPORT_H

#include <string>
#include <vector>
using namespace std;


typedef struct ResultTracks {
	double lat;
	double lon;
	double GpsLat;
	double GpsLon;
	double GpsHeading;
	double heading;
	double utcTime;
	double hdop;
	double accuracy;
	double vel;
	double time;
	string motionType;
}KmlTracks_t;

/**----------------------------------------------------------------------
* Function    : KmlWrite
* Description : ��pdr�㷨�����gps��pdr�켣дΪkml��ʽ
*               path    : kml�ļ�������ļ�·��
*               name    : kml�ļ���������
*               postfix �����������ƺ�����ӵĺ�׺�������������ͻ��߰汾
* Date        : 2020/11/1 logzhan
*---------------------------------------------------------------------**/
void KmlWrite(string path, string name, string postfix, vector<KmlTracks_t>& res);

#endif // _VDR_KML_SUPPORT_H
