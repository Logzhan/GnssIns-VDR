#ifndef  _VDR_QUATERNION_H_
#define  _VDR_QUATERNION_H_

#ifdef __cplusplus
extern "C" {
#endif

/**---------------------------------------------------------------------
* Function    : QuaternionNorm
* Description : ��Ԫ����һ��
* Date        : 2022/09/21 logzhan
*---------------------------------------------------------------------**/
void QuaternionNorm(float* q0, float* q1, float* q2, float* q3);

/**---------------------------------------------------------------------
* Function    : QuaternConj
* Description : ��Ԫ��������Ԫ��
* Date        : 2022/09/21 logzhan
*---------------------------------------------------------------------**/
void QuaternConj(float qc[], float q[]);

/**---------------------------------------------------------------------
* Function    : QuaternProd
* Description : ��Ԫ���˷�
* Date        : 2022/09/21 logzhan
*---------------------------------------------------------------------**/
void QuaternProd(float qab[], float qa[], float qb[]);

#ifdef __cplusplus
}
#endif

#endif