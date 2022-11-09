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
* Description : 将pdr算法输出的gps和pdr轨迹写为kml形式
*               path    : kml文件的输出文件路径
*               name    : kml文件主体名称
*               postfix ：在主体名称后面添加的后缀，用于区分类型或者版本
* Date        : 2020/11/1 logzhan
*---------------------------------------------------------------------**/
void KmlWrite(string path, string name, string postfix, vector<KmlTracks_t>& res);

#endif // _VDR_KML_SUPPORT_H
