#ifndef _VDR_KML_SUPPORT_H
#define _VDR_KML_SUPPORT_H

#include <string>
using namespace std;

namespace VDR {

typedef struct ResultTracks {
	double lat;
	double lon;
	double heading;
	double utcTime;
	double hdop;
	double accuracy;
	double vel;
	double time;
	string motionType;
}KmlTracks_t;


}
#endif // _VDR_KML_SUPPORT_H
