#ifndef _VDR_COORD_SUPPORT_H
#define _VDR_COORD_SUPPORT_H

#include <string>
using namespace std;

namespace VDR {

void WGS842ECEF(double* plla, double* ecef);

void ECEF2WGS84(double* ecef, double* plla);

void ECEF2NED(double* ecef, double* plla, double* ned);

void NED2ECEF(double* plla, double* ned, double* ecef0, double* ecef);

void WGS842NED(double* plla, double* ref_lla, double* ned);

void NED2WGS84(double* ref_plla, double* ned, double* plla);

}
#endif // _VDR_KML_SUPPORT_H