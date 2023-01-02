#ifndef _VDR_KALMAN_H_
#define _VDR_KALMAN_H_

#include "GnssInsDef.h"

/**----------------------------------------------------------------------
* Function    : Kalman_Init
* Description : ³õÊ¼»¯¿¨¶ûÂüÂË²¨Æ÷Ïà¹Ø
* Date        : 2022/11/09 logzhan
*---------------------------------------------------------------------**/
void Kalman_Init(GnssIns_t& gins);

/**----------------------------------------------------------------------
* Function    : EkfStatePredict
* Description : ¿¨¶ûÂü×´Ì¬Ô¤²â
* Date        : 2022/11/09 logzhan
*---------------------------------------------------------------------**/
void EkfStatePredict(GnssIns_t& gins);

/**---------------------------------------------------------------------
* Function    : EkfStateUpdate
* Description : ¿¨¶ûÂü×´Ì¬¸üÐÂÒÔ¼°×´Ì¬ÐÞÕý
* Date        : 2022/11/18 logzhan
*---------------------------------------------------------------------**/
void EkfStateUpdate(Gnss_t& gnss, GnssIns_t& gins);

#endif