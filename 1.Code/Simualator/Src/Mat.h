/******************** (C) COPYRIGHT 2022 Geek************************************
* File Name          : Mat.h
* Current Version    : V1.0
* Author             : logzhan
* Date of Issued     : 2022.09.14
* Comments           : 导航的矩阵计算
********************************************************************************/
/* Header File Including -----------------------------------------------------*/
#ifndef _H_MAT_
#define _H_MAT_

#define MAT_MAX 15 //决定了能处理的最大矩阵


#include <string.h>
#define _USE_MATH_DEFINES
#include <math.h>

class Mat
{
public:
	Mat();
	Mat(int setm,int setn,int kind);//kind=1单位阵，kind=0零矩阵,其它不初始化内容。
	void Init(int setm,int setn,int kind);//kind=1单位阵，kind=0零矩阵,其它不初始化内容。

	void Zero(void);
	//这些关键数本应该作为private的。但是为了方便，我也做成了public
	int m;//行数
	int n;//列数
	double mat[MAT_MAX][MAT_MAX];//矩阵数据内容

	//特殊的矩阵
	Mat SubMat(int a,int b,int lm,int ln);//获取矩阵一部分
	void FillSubMat(int a,int b,Mat s);//填充子矩阵

	//向量专用
	double absvec();//这个是向量的长度。不是个别元素的绝对值。
	double Sqrt();//向量长度的平方
	friend Mat operator ^(Mat a,Mat b);//叉乘

	//运算
	friend Mat operator *(double k,Mat a);
	friend Mat operator *(Mat a,double k);
	friend Mat operator /(Mat a,double k);
	friend Mat operator *(Mat a,Mat b);
	friend Mat operator +(Mat a,Mat b);
	friend Mat operator -(Mat a,Mat b);
	friend Mat operator ~(Mat a);//转置	
	friend Mat operator /(Mat a,Mat b);//a*inv(b)
	friend Mat operator %(Mat a,Mat b);//inv(a)*b

	//MAT inv();//逆矩阵

private:
	// 为了用高斯消元法，做的一些函数
	// 交换两行
	void RowExchange(int a, int b);
	// 某一行乘以系数
	void RowMul(int a,double k);
	// 对某一行加减另一行的倍数
	void RowAdd(int a,int b, double k);
	// 交换两列
	void ColExchange(int a, int b);
	// 某一列乘以系数
	void ColMul(int a,double k);
	// 对某一列加减另一列的倍数
	void ColAdd(int a,int b,double k);
	

};






#endif






