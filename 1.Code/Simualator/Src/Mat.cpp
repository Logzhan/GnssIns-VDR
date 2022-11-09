/******************** (C) COPYRIGHT 2022 Geek************************************
* File Name          : Mat.cpp
* Current Version    : V1.0
* Author             : logzhan
* Date of Issued     : 2022.09.14
* Comments           : 矩阵运算库
********************************************************************************/
/* Header File Including -----------------------------------------------------*/
#include "Mat.h"

double mind(double a,double b)
{
	double c=a;
	if(b<c)
	{
		c=b;
	}
	return c;
}

int mini(int a,int b)
{
	int c=a;
	if(b<c)
	{
		c=b;
	}
	return c;
	
}

//不要在成员函数内调用构造函数
Mat::Mat()
{
	Init(1,1,0);
}

Mat::Mat(int setm,int setn,int kind)
{
	Init(setm,setn,kind);
}

void Mat::Init(int setm,int setn,int kind)
{
	this->m = setm;
	this->n = setn;
	if((kind==0)||(kind==1))
	{
		memset(mat,0,MAT_MAX*MAT_MAX*sizeof(double));
	}

	if(kind==1)
	{
		int x;
		//C原有的max min会导致两次运行自变量。有附带操作的东西不要直接放到max里面。
		int xend = mini(this->m, this->n);
		for(x=0;x < xend;x++){
			mat[x][x] = 1;
		}
	}
}
void Mat::Zero() {

}

Mat Mat::SubMat(int a,int b,int lm,int ln)
{

	int aend=a+lm-1;
	int bend=b+ln-1;


	Mat s(lm,ln,-1);
	int x,y;
	for(x=0;x<lm;x++)
	{
		for(y=0;y<ln;y++)
		{
			s.mat[x][y]=mat[a+x][b+y];
		}
	}
	return s;
}


void Mat::FillSubMat(int a,int b,Mat s)
{
	int x,y;
	for(x=0;x<s.m;x++)
	{
		for(y=0;y<s.n;y++)
		{
			mat[a+x][b+y]=s.mat[x][y];
		}
	}
}


Mat operator *(double k, Mat a)
{
	Mat b(a.m,a.n,-1);
	int x,y;
	for(x=0;x<a.m;x++)
	{
		for(y=0;y<a.n;y++)
		{
			b.mat[x][y]=k*a.mat[x][y];
		}
	}
	return b;
}
Mat operator *(Mat a,double k)
{
	return k*a;
}
Mat operator /(Mat a,double k)
{
	return (1/k)*a;
}
Mat operator *(Mat a,Mat b)
{
	Mat c(a.m,b.n,-1);
	int x,y,z;
	double s;
	for(x=0;x<a.m;x++)
	{
		for(y=0;y<b.n;y++)
		{
			s=0;
			for(z=0;z<a.n;z++)
			{
				s=s+a.mat[x][z]*b.mat[z][y];
			}
			c.mat[x][y]=s;
		}
	}
	return c;

}

Mat operator +(Mat a,Mat b)
{

	Mat c=a;
	int x,y;
	for(x=0;x<c.m;x++)
	{
		for(y=0;y<c.n;y++)
		{
			c.mat[x][y]+=b.mat[x][y];
		}
	}
	return c;
}
Mat operator -(Mat a,Mat b)
{

	Mat c=a;
	int x,y;
	for(x=0;x<c.m;x++)
	{
		for(y=0;y<c.n;y++)
		{
			c.mat[x][y]-=b.mat[x][y];
		}
	}
	return c;
}
Mat operator ~(Mat a)
{
	Mat b(a.n,a.m,-1);
	int x,y;
	for(x=0;x<a.m;x++)
	{
		for(y=0;y<a.n;y++)
		{
			b.mat[y][x]=a.mat[x][y];

		}
	}
	return b;
}



Mat operator /(Mat a,Mat b)
{
	//高斯消元法

	int x,xb;
	for(x=0;x<b.n;x++)
	{
		//首先找到最佳的列。让起始元素最大
		double s=0;
		int p=x;
		double sxb;
		for(xb=x;xb<b.n;xb++)
		{
			sxb=fabs(b.mat[x][xb]);
			if(sxb>s)
			{
				p=xb;
				s=sxb;
			}
		}
		//同时变换两侧矩阵
		if(x!=p)
		{
			a.ColExchange(x,p);
			b.ColExchange(x,p);
		}

		//这一列归一
		double k=1/b.mat[x][x];//这一句不要嵌套到下面两行中，否则会因为更新不同步导致计算错误。
		a.ColMul(x,k);
		b.ColMul(x,k);

		//把其它列归零
		for(xb=0;xb<b.n;xb++)
		{
			if(xb!=x)
			{
				k=(-b.mat[x][xb]);
				a.ColAdd(xb,x,k);
				b.ColAdd(xb,x,k);
			}
		}
	}
	
	return a;
}


Mat operator %(Mat a,Mat b)
{
	//高斯消元法
	int x,xb;
	for(x=0;x<a.m;x++)
	{
		//首先找到最佳的行。让起始元素最大
		double s=0;
		int p=x;
		double sxb;
		for(xb=x;xb<a.m;xb++)
		{
			sxb=fabs(a.mat[xb][x]);
			if(sxb>s)
			{
				p=xb;
				s=sxb;
			}
		}
		//同时变换两侧矩阵
		if(x!=p)
		{
			a.RowExchange(x,p);
			b.RowExchange(x,p);
		}

		//这一行归一
		double k=1/a.mat[x][x];//这一句不要嵌套到下面两行中，否则会因为更新不同步导致计算错误。
		a.RowMul(x,k);
		b.RowMul(x,k);

		//把其它行归零
		for(xb=0;xb<a.m;xb++)
		{
			if(xb!=x)
			{
				k=(-a.mat[xb][x]);
				a.RowAdd(xb,x,k);
				b.RowAdd(xb,x,k);
			}
		}
	}
	
	return b;
}


void Mat::RowExchange(int a, int b)
{
	double s[MAT_MAX];
	int ncpy=n*sizeof(double);
	memcpy(s,mat[a],ncpy);
	memcpy(mat[a],mat[b],ncpy);
	memcpy(mat[b],s,ncpy);
}
void Mat::RowMul(int a,double k)
{
	int y;
	for(y=0;y<n;y++)
	{
		mat[a][y]= mat[a][y]*k;
	}
}
void Mat::RowAdd(int a,int b,double k)
{
	int y;
	for(y=0;y<n;y++)
	{
		mat[a][y]= mat[a][y]+ mat[b][y]*k;
	}
}

void Mat::ColExchange(int a, int b)
{
	double s;
	int x;
	for(x=0;x<m;x++)
	{
		s=mat[x][a];
		mat[x][a]=mat[x][b];
		mat[x][b]=s;
	}
}
void Mat::ColMul(int a,double k)
{
	int x;
	for(x=0;x<m;x++)
	{
		mat[x][a]=mat[x][a]*k;
	}
}
void Mat::ColAdd(int a,int b,double k)
{
	int x;
	for(x=0;x<m;x++)
	{
		mat[x][a]=mat[x][a]+mat[x][b]*k;
	}
}



double Mat::Sqrt()
{
	int x;
	double numx;
	double s=0;
	for(x=0;x<m;x++)
	{
		numx=mat[x][0];
		s+=(numx*numx);
	}
	return s;
}
double Mat::absvec()
{
	return sqrt(Sqrt());
}

Mat operator ^(Mat a, Mat b)
{
	double ax=a.mat[0][0];
	double ay=a.mat[1][0];
	double az=a.mat[2][0];
	double bx=b.mat[0][0];
	double by=b.mat[1][0];
	double bz=b.mat[2][0];

	Mat c(3,1,-1);
	c.mat[0][0]=ay*bz-az*by;
	c.mat[1][0]=(-(ax*bz-az*bx));
	c.mat[2][0]=ax*by-ay*bx;

	return c;
}

