#pragma once
/*
 *  Copyright (c) 2010  Chen Feng (cforrest (at) umich.edu)
 *    and the University of Michigan
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

//standard include
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdio.h>
#include <time.h>
//opencv include
#include "OpenCVHelper.h"

namespace CvMatHelper
{
#define CreateCvMatHead(h,r,c,p) CvMat h = cvMat(r,c,CV_64FC1, const_cast<double*>(p))
#define IDX(m,n,N) ((m)*(N)+(n)) //N cols

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
	CreateCvMatHead(_A,m,n,A);
	CreateCvMatHead(_AT,n,m,AT);
	cvTranspose(&_A,&_AT);
}

// R<amxbn> = A<amxan> * B<bmxbn>, an==bm
inline void mul(int am, int an, int bm, int bn,
                double const *A, double const *B, double *R)
{
	CreateCvMatHead(_A,am,an,A);
	CreateCvMatHead(_B,bm,bn,B);
	CreateCvMatHead(_R,am,bn,R);
	cvMatMul(&_A,&_B,&_R);
}

// R<amxbn> = A<amxan>' * B<bmxbn>, am==bm
inline void mulAtB(int am, int an, int bm, int bn,
                   double const *A, double const *B, double *R)
{
	CreateCvMatHead(_A,am,an,A);
	CreateCvMatHead(_B,bm,bn,B);
	CreateCvMatHead(_R,an,bn,R);
	cvGEMM(&_A,&_B,1,0,0,&_R,CV_GEMM_A_T);
}

// R<amxbn> = A<amxan> * B<bmxbn>', an==bn
inline void mulABt(int am, int an, int bm, int bn,
                   double const *A, double const *B, double *R)
{
	CreateCvMatHead(_A,am,an,A);
	CreateCvMatHead(_B,bm,bn,B);
	CreateCvMatHead(_R,am,bm,R);
	cvGEMM(&_A,&_B,1,0,0,&_R,CV_GEMM_B_T);
}

// R<1x1> = x'<1xn> * A<nxn> * y<nx1>
inline double mulxtAy(int n,
                      double const *x, double const *A, double const *y)
{
	CreateCvMatHead(_A,n,n,A);
	CreateCvMatHead(_y,n,1,y);
	CreateCvMatHead(_xt,1,n,x);
	double result;
	CreateCvMatHead(_R,1,1,&result);
	CvMat *tmp = cvCreateMat(n,1,CV_64FC1);
	cvMatMul(&_A,&_y,tmp);
	cvMatMul(&_xt,tmp,&_R);
	cvReleaseMat(&tmp);
	return result;
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
	CreateCvMatHead(_A,n,n,A);
	CreateCvMatHead(_R,n,n,R);
	cvPow(&_A,&_R,idx);
}

// R<mxn> = A<mxn> + B<mxn>
inline void add(int m, int n,
                double const *A, double const *B, double *R)
{
	CreateCvMatHead(_A,m,n,A);
	CreateCvMatHead(_B,m,n,B);
	CreateCvMatHead(_R,m,n,R);
	cvAdd(&_A,&_B,&_R);
}

// R<mxn> = A<mxn> - B<mxn>
inline void sub(int m, int n,
                double const *A, double const *B, double *R)
{
	CreateCvMatHead(_A,m,n,A);
	CreateCvMatHead(_B,m,n,B);
	CreateCvMatHead(_R,m,n,R);
	cvSub(&_A,&_B,&_R);
}

inline double det(int n, double const *A)
{
	CreateCvMatHead(_A,n,n,A);
	return cvDet(&_A);
}

inline double inv(int n, double const *A, double *Ainv, int method=CV_SVD)
{
	CreateCvMatHead(_A,n,n,A);
	CreateCvMatHead(_Ainv,n,n,Ainv);
	return cvInvert(&_A,&_Ainv,method);
}

// U<mxmn> * S<mnxmn> * VT<mnxn> = A<mxn>, mn=min(m,n)
inline void svd(int m, int n,
                double const *A, double *U, double *S, double *VT)
{
	int mn = std::min(m,n);
	CreateCvMatHead(_A,m,n,A);
	CreateCvMatHead(_U,m,mn,U);
	CreateCvMatHead(_S,mn,mn,S);
	CreateCvMatHead(_VT,mn,n,VT);
	cvSVD(&_A,&_S,&_U,&_VT,CV_SVD_V_T);
}

// x<nx1> = arg(x) min ||A<mxn> * x - b<mx1>||
// i.e. solve Ax=b
inline int solve(int m, int n,
                 double const *A, double const *b, double *x, int method=CV_SVD)
{
	CreateCvMatHead(_A,m,n,A);
	CreateCvMatHead(_b,m,1,b);
	CreateCvMatHead(_x,n,1,x);
	return cvSolve(&_A,&_b,&_x,method);
}

// x<nx1> = arg(x) min ||A<mxn> * x||, s.j. ||x||=1
inline void nullvector(int m, int n, double const *A, double *x)
{
	int mn = std::min(m,n);
	CreateCvMatHead(_A,m,n,A);
	CvMat *U = cvCreateMat(m,mn,CV_64FC1);
	CvMat *S = cvCreateMat(mn,mn,CV_64FC1);
	CvMat *V = cvCreateMat(n,mn,CV_64FC1);
	cvSVD(&_A,S,U,V);
	int end = mn-1;
	for(int i=0; i<n; ++i) {
		x[i] = CV_MAT_ELEM(*V, double, i, end);
	}
	cvReleaseMat(&U);
	cvReleaseMat(&S);
	cvReleaseMat(&V);
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
