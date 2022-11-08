#ifdef __cplusplus
extern "C" {
#endif

#ifndef _VDR_MATRIX_H_
#define _VDR_MATRIX_H_

#define  N           4

/**---------------------------------------------------------------------
* Function    : MatrixTrans
* Description : ����ת��
* Date        : 2022/09/14 logzhan
*---------------------------------------------------------------------**/
void MatrixTrans(double a[N][N], double r[N][N]);

/**---------------------------------------------------------------------
* Function    : VecMatMultiply
* Description : �����;������ r = b * a
* Date        : 2022/09/14 logzhan
*---------------------------------------------------------------------**/
void VecMatMultiply(double a[N], double b[N][N], double r[N]);

/**---------------------------------------------------------------------
* Function    : MatrixMultiply
* Description : ����;������ r = a * b
* Date        : 2022/09/14 logzhan
*---------------------------------------------------------------------**/
void MatrixMultiply(double a[N][N], double b[N][N], double r[N][N]);

/**---------------------------------------------------------------------
* Function    : MatrixAdd
* Description : ����;������ r = a + b, ע���������֧��a = a + b
* Date        : 2022/09/14 logzhan
*---------------------------------------------------------------------**/
void MatrixAdd(double a[N][N], double b[N][N], double r[N][N]);

/**---------------------------------------------------------------------
* Function    : VectorAdd
* Description : ������������� r = a + b, ע���������֧��a = a + b
* Date        : 2022/09/14 logzhan
*---------------------------------------------------------------------**/
void VectorAdd(double a[N], double b[N], double r[N]);

/**---------------------------------------------------------------------
* Function    : MatrixSub
* Description : ����;������ r = a - b, ע���������֧��a = a - b
* Date        : 2022/09/14 logzhan
*---------------------------------------------------------------------**/
void MatrixSub(double a[N][N], double b[N][N], double r[N][N]);

/**---------------------------------------------------------------------
* Function    : VectorSub
* Description : ������������� r = a - b, ע���������֧��a = a - b
* Date        : 2022/09/14 logzhan
*---------------------------------------------------------------------**/
void VectorSub(double a[N], double b[N], double r[N]);

/**---------------------------------------------------------------------
* Function    : MatrixInverse
* Description : �����������
* Date        : 2022/09/14 logzhan
*---------------------------------------------------------------------**/
void MatrixInverse(double(*M)[N], double(*MInv)[N]);

#endif

#ifdef __cplusplus
}
#endif