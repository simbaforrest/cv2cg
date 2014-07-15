#pragma once
/*
 *  Copyright (c) 2010  Chen Feng (cforrest (at) umich.edu)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

/* CvMatHelper.h
   CvMat related helper functions */

#include <string.h> //memset
#include <math.h>   //sqrt

#include "OpenCVHeaders.h"

namespace CvMatHelper
{
#define CreateCvMatHead(h,r,c,p)  CvMat h = cvMat(r,c,CV_64FC1, const_cast<double*>(p))

inline void identity(int n, double *A)
{
	memset(A, 0, sizeof(double)*n*n);
	for(int i=0; i<n; ++i) {
		A[i *n+i]=1.0;
	}
}

inline void zeros(int m, int n, double *A)
{
	memset(A, 0, sizeof(double)*m*n);
}

// A<mxn> -> AT<nxm>
inline void transpose(int m,  int n,
                      double const *A, double *AT)
{
	cv::Mat mA(m,n,CV_64FC1,const_cast<double*>(A));
	cv::Mat mAT(n,m,CV_64FC1,AT);
	mAT = mA.t();
}

// R<amxbn> = A<amxan> * B<bmxbn>, an==bm
inline void mul(int am, int an, int bm, int bn,
                double const *A, double const *B, double *R)
{
	cv::Mat mA(am,an,CV_64FC1,const_cast<double*>(A));
	cv::Mat mB(bm,bn,CV_64FC1,const_cast<double*>(B));
	cv::Mat mR(am,bn,CV_64FC1,R);
	mR = mA*mB;
}

// R<amxbn> = A<amxan>' * B<bmxbn>, am==bm
inline void mulAtB(int am, int an, int bm, int bn,
                   double const *A, double const *B, double *R)
{
	cv::Mat mA(am,an,CV_64FC1,const_cast<double*>(A));
	cv::Mat mB(bm,bn,CV_64FC1,const_cast<double*>(B));
	cv::Mat mR(an,bn,CV_64FC1,R);
	mR = mA.t()*mB;
}

// R<amxbn> = A<amxan> * B<bmxbn>', an==bn
inline void mulABt(int am, int an, int bm, int bn,
                   double const *A, double const *B, double *R)
{
	cv::Mat mA(am,an,CV_64FC1,const_cast<double*>(A));
	cv::Mat mB(bm,bn,CV_64FC1,const_cast<double*>(B));
	cv::Mat mR(am,bm,CV_64FC1,R);
	mR = mA*mB.t();
}

// R<1x1> = x'<1xn> * A<nxn> * y<nx1>
inline double mulxtAy(int n,
                      double const *x, double const *A, double const *y)
{
	cv::Mat mA(n,n,CV_64FC1,const_cast<double*>(A));
	cv::Mat mx(1,n,CV_64FC1,const_cast<double*>(x));
	cv::Mat my(n,1,CV_64FC1,const_cast<double*>(y));
	cv::Mat ret = mx * mA * my;
	return ret.at<double>(0,0);
}

// R<mxn> = A<mxn> * s
inline void scale(int m,  int n,
                  double const *A, double s, double *R)
{
	int mn = m*n;
	for(int i=0; i<mn; ++i) {
		R[i]=A[i]*s;
	}
}

// R<nxn> = A<nxn> ^ idx
inline void pow(int n, double const *A, int idx, double *R)
{
	cv::Mat mA(n,n,CV_64FC1,const_cast<double*>(A));
	cv::Mat mR(n,n,CV_64FC1,R);
	cv::pow(mA, idx, mR);
}

// R<mxn> = A<mxn> + B<mxn>
inline void add(int m, int n,
                double const *A, double const *B, double *R)
{
	cv::Mat mA(m,n,CV_64FC1,const_cast<double*>(A));
	cv::Mat mB(m,n,CV_64FC1,const_cast<double*>(B));
	cv::Mat mR(m,n,CV_64FC1,R);
	mR = mA+mB;
}

// R<mxn> = A<mxn> - B<mxn>
inline void sub(int m, int n,
                double const *A, double const *B, double *R)
{
	cv::Mat mA(m,n,CV_64FC1,const_cast<double*>(A));
	cv::Mat mB(m,n,CV_64FC1,const_cast<double*>(B));
	cv::Mat mR(m,n,CV_64FC1,R);
	mR = mA-mB;
}

inline double det(int n, double const *A)
{
	cv::Mat mA(n,n,CV_64FC1,const_cast<double*>(A));
	return cv::determinant(mA);
}

inline void inv(int n, double const *A, double *Ainv, int method=cv::DECOMP_SVD)
{
	cv::Mat mA(n,n,CV_64FC1,const_cast<double*>(A));
	cv::Mat mAi(n,n,CV_64FC1,Ainv);
	mAi = mA.inv(method);
}

// U<mxmn> * S<mnxmn> * VT<mnxn> = A<mxn>, mn=min(m,n)
inline void svd(int m, int n,
                double const *A, double *U, double *S, double *VT)
{
	int mn = (std::min)(m,n);
	cv::Mat mA(m,n,CV_64FC1,const_cast<double*>(A));
	cv::Mat mU(m,mn,CV_64FC1,U);
	cv::Mat mS(mn,mn,CV_64FC1,S);
	cv::Mat mVT(mn,n,CV_64FC1,VT);
	cv::SVD::compute(mA,mS,mU,mVT);
}

// x<nx1> = arg(x) min ||A<mxn> * x - b<mx1>||
// i.e. solve Ax=b
inline bool solve(int m, int n,
                 double const *A, double const *b, double *x, int method=cv::DECOMP_SVD)
{
	cv::Mat mA(m,n,CV_64FC1,const_cast<double*>(A));
	cv::Mat mb(m,1,CV_64FC1,const_cast<double*>(b));
	cv::Mat mx(n,1,CV_64FC1,x);
	return cv::solve(mA,mb,mx);
}

// x<nx1> = arg(x) min ||A<mxn> * x||, s.j. ||x||=1
inline void nullvector(int m, int n, double const *A, double *x)
{
	cv::Mat mA(m,n,CV_64FC1,const_cast<double*>(A));
	cv::Mat mx(n,1,CV_64FC1,x);
	cv::SVD::solveZ(mA,mx);
}

inline void cross(double const *A, double const *B, double *R)
{
	R[0]=A[1]*B[2]-A[2]*B[1];
	R[1]=A[2]*B[0]-A[0]*B[2];
	R[2]=A[0]*B[1]-A[1]*B[0];
}

inline double cross2D(double const *A, double const *B)
{
	return A[0]*B[1]-A[1]*B[0];
}

inline double dot(int am, int bm, double const *A, double const *B)
{
	double ret;
	mulAtB(am,1,bm,1,A,B,&ret);
	return ret;
}

//return the L2 norm of a vector (assume an==1 or am==1)
inline double normL2(int am, int an, double const *A)
{
	int n = am*an;
	return sqrt( dot(n,n,A,A) );
}

////////////////////////////////////////////
///  Utils
// convert a pointer to a continuous memory block
// to a 2D array with "cols" columns in Precision type
template<int cols, typename Precision=double *>
struct FixMat {
	typedef  Precision (*Type)[cols];

	template<typename SrcType>
	static Type ConvertType(SrcType src) {
		return reinterpret_cast<Type>(src);
	}
};

}//CvMatHelper
