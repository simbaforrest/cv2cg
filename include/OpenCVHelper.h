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

/* OpenCVHelper.h
   a set of helper functions for easier access to opencv */

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
#include "cv.h"
#include "highgui.h"

namespace CvMatHelper {
#define CreateCvMatHead(h,r,c,p) CvMat h = cvMat(r,c,CV_64FC1, const_cast<double*>(p))
#define IDX(m,n,N) ((m)*(N)+(n)) //N cols

	inline void identity(int n, double* A) {
		memset(A, 0, sizeof(double)*n*n);
		for(int i=0; i<n; ++i) A[i*n+i]=1.0;
	}

	inline void zeros(int m, int n, double* A) {
		memset(A, 0, sizeof(double)*m*n);
	}

	// A<mxn> -> AT<nxm>
	inline void transpose(int m,  int n,
		double const* A, double* AT) {
		CreateCvMatHead(_A,m,n,A);
		CreateCvMatHead(_AT,n,m,AT);
		cvTranspose(&_A,&_AT);
	}

	// R<amxbn> = A<amxan> * B<bmxbn>, an==bm
	inline void mul(int am, int an, int bm, int bn,
		double const* A, double const* B, double* R) {
		CreateCvMatHead(_A,am,an,A);
		CreateCvMatHead(_B,bm,bn,B);
		CreateCvMatHead(_R,am,bn,R);
		cvMatMul(&_A,&_B,&_R);
	}

	// R<amxbn> = A<amxan>' * B<bmxbn>, am==bm
	inline void mulAtB(int am, int an, int bm, int bn,
		double const* A, double const* B, double* R) {
		CreateCvMatHead(_A,am,an,A);
		CreateCvMatHead(_B,bm,bn,B);
		CreateCvMatHead(_R,am,bn,R);
		cvGEMM(&_A,&_B,1,0,0,&_R,CV_GEMM_A_T);
	}

	// R<amxbn> = A<amxan> * B<bmxbn>', an==bn
	inline void mulABt(int am, int an, int bm, int bn,
		double const* A, double const* B, double* R) {
		CreateCvMatHead(_A,am,an,A);
		CreateCvMatHead(_B,bm,bn,B);
		CreateCvMatHead(_R,am,bn,R);
		cvGEMM(&_A,&_B,1,0,0,&_R,CV_GEMM_B_T);
	}

	// R<1x1> = x'<1xn> * A<nxn> * x<nx1>
	inline double mulxtAx(int n, double const* A, double const* x) {
		CreateCvMatHead(_A,n,n,A);
		CreateCvMatHead(_x,n,1,x);
		CreateCvMatHead(_xt,1,n,x);
		double result;
		CreateCvMatHead(_R,1,1,&result);
		CvMat* tmp = cvCreateMat(n,1,CV_64FC1);
		cvMatMul(&_A,&_x,tmp);
		cvMatMul(&_xt,tmp,&_R);
		cvReleaseMat(&tmp);
		return result;
	}

	// R<mxn> = A<mxn> * s
	inline void scale(int m,  int n,
		double const* A, double s, double* R) {
		int mn = m*n;
		for(int i=0; i<mn; ++i) R[i]=A[i]*s;
	}

	// R<nxn> = A<nxn> ^ idx
	inline void pow(int n, double const* A, int idx, double* R) {
		CreateCvMatHead(_A,n,n,A);
		CreateCvMatHead(_R,n,n,R);
		cvPow(&_A,&_R,idx);
	}

	// R<mxn> = A<mxn> + B<mxn>
	inline void add(int m, int n,
		double const* A, double const* B, double* R) {
		CreateCvMatHead(_A,m,n,A);
		CreateCvMatHead(_B,m,n,B);
		CreateCvMatHead(_R,m,n,R);
		cvAdd(&_A,&_B,&_R);
	}

	// R<mxn> = A<mxn> - B<mxn>
	inline void sub(int m, int n,
		double const* A, double const* B, double* R) {
		CreateCvMatHead(_A,m,n,A);
		CreateCvMatHead(_B,m,n,B);
		CreateCvMatHead(_R,m,n,R);
		cvSub(&_A,&_B,&_R);
	}

	inline double det(int n, double const* A) {
		CreateCvMatHead(_A,n,n,A);
		return cvDet(&_A);
	}

	inline double inv(int n, double const* A, double* Ainv, int method=CV_SVD) {
		CreateCvMatHead(_A,n,n,A);
		CreateCvMatHead(_Ainv,n,n,Ainv);
		return cvInvert(&_A,&_Ainv,method);
	}

	// U<mxmn> * S<mnxmn> * VT<mnxn> = A<mxn>, mn=min(m,n)
	inline void svd(int m, int n,
		double const* A, double *U, double *S, double *VT) {
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
		double const *A, double const *b, double *x, int method=CV_SVD) {
		CreateCvMatHead(_A,m,n,A);
		CreateCvMatHead(_b,m,1,b);
		CreateCvMatHead(_x,n,1,x);
		return cvSolve(&_A,&_b,&_x,method);
	}

	// x<nx1> = arg(x) min ||A<mxn> * x||, s.j. ||x||=1
	inline void nullvector(int m, int n, double const *A, double *x) {
		int mn = std::min(m,n);
		CreateCvMatHead(_A,m,n,A);
		CvMat* U = cvCreateMat(m,mn,CV_64FC1);
		CvMat* S = cvCreateMat(mn,mn,CV_64FC1);
		CvMat* V = cvCreateMat(n,mn,CV_64FC1);
		cvSVD(&_A,S,U,V);
		int end = mn-1;
		for(int i=0; i<n; ++i) x[i] = CV_MAT_ELEM(*V, double, i, end);
		cvReleaseMat(&U);
		cvReleaseMat(&S);
		cvReleaseMat(&V);
	}

	/*********************I/O***********************/
	/* print the given n x m matrix */
	template<typename Precision>
	void print(int m, int n, Precision *A, char* mat_name=0) {
		int i, j;

		if(mat_name) {
			printf("%s = \n\n", mat_name);
		}

		for (i = 0; i < m; i++) {
			printf("  ");
			for (j = 0; j < n; j++) {
				printf(" % 10f", (float)A[i * n + j]);
			}
			printf("\n");
		}
		printf("\n");
	}

	/* Read a matrix from a file */
	template<typename Precision>
	bool ReadFile(int m, int n, Precision *matrix, char *fname) {
		FILE *f = NULL;
		fopen_s(&f, fname, "r");
		int i;

		if (f == NULL) {
			char msg[200];
			sprintf_s<200>(msg,"In reading matrix %s\n", fname);
			Log::e(msg, "MatrixManip::ReadFile");
			return false;
		}

		for (i = 0; i < m * n; i++) {
			fscanf_s(f, "%lf", matrix + i);
		}

		fclose(f);
		return true;
	}

	/* Write a matrix to a file */
	template<typename Precision>
	bool WriteFile(int m, int n, Precision *matrix, char *fname) {
		FILE *f = NULL;
		fopen_s(&f, fname, "w");
		int i, j, idx;

		if (f == NULL) {
			char msg[200];
			sprintf_s<200>(msg,"In writing matrix to %s\n", fname);
			Log::e(msg, "MatrixManip::WriteFile");
			return false;
		}

		idx = 0;
		for (i = 0; i < m; i++) {
			for (j = 0; j < n; j++, idx++) {
				fprintf(f, "%0.16e ", matrix[idx]);
			}
			fprintf(f, "\n");
		}

		fclose(f);
		return true;
	}

	template<typename Precision>
	std::ostream& print(std::ostream& o, Precision* p, int m, int n) {
		o.setf(std::ios::scientific);
		for(int i=0; i<m; ++i) {
			for(int j=0; j<n; ++j) {
				o << p[IDX(i,j,n)] << " ";
			}
			o << std::endl;
		}
		return o;
	}

	////////////////////////////////////////////
	///  Utils
	// convert a pointer to a continuous memory block
	// to a 2D array with "cols" columns in Precision type
	template<int cols, typename Precision=double*>
	struct FixMat {
		typedef  Precision (*Type)[cols];

		template<typename SrcType>
		static Type ConvertType(SrcType src)
		{
			return reinterpret_cast<Type>(src);
		}
	};


}//CvMatHelper

namespace CameraHelper {
	// s[u,v,1]' = P * [x,y,z,1]'
	// P<3x4>, projection matrix
	inline void project(double const* P,
		double x, double y, double z,
		double& u, double& v)
	{
		double w;
		u = P[IDX(0,0,4)] * x + P[IDX(0,1,4)] * y
		  + P[IDX(0,2,4)] * z + P[IDX(0,3,4)] * 1;
		v = P[IDX(1,0,4)] * x + P[IDX(1,1,4)] * y
		  + P[IDX(1,2,4)] * z + P[IDX(1,3,4)] * 1;
		w = P[IDX(2,0,4)] * x + P[IDX(2,1,4)] * y
		  + P[IDX(2,2,4)] * z + P[IDX(2,3,4)] * 1;
		if(fabs(w)<1e-10)
			u = v = 0; // ideal point
		else {
			u/=w;
			v/=w;
		}
	}

	inline void project(double const* P,
		double const *X, double *p)
	{
		project(P, X[0], X[1], X[2], p[0], p[1]);
	}

	// K<3x3> * [R<3x3> , T<3x1>] = P<3x4>
	// K<3x3> * R<3x3> [I<3x3> , -C<3x1>] = P<3x4>
	// T<3x1> = -R<3x3> * C<3x1>
	inline void decompose(double const* P,
		double* K, double* R, double* T, double* C) {
		CreateCvMatHead(_P,3,4,P);
		CreateCvMatHead(_K,3,3,K);
		CreateCvMatHead(_R,3,3,R);
		double tmp[4];
		CreateCvMatHead(_tmp,4,1,tmp);
		//translation vector is camera Center in world coordinate system
		//so it is C rather than T in opencv manual
		cvDecomposeProjectionMatrix(&_P,&_K,&_R,&_tmp);
		C[0]=tmp[0]/tmp[3];
		C[1]=tmp[1]/tmp[3];
		C[2]=tmp[2]/tmp[3];
		CvMatHelper::mul(3,3,3,1,R,C,T); //T = -RC, C = -R'T
		T[0]*=-1; T[1]*=-1; T[2]*=-1;
	}

	inline void compose(double const* K,
		double const* R, double const* T,
		double* P, bool TisC=false) {
		double PP[12]={0};
		if(!TisC) { //K[R,T]
			PP[0]=R[0]; PP[1]=R[1]; PP[2] =R[2]; PP[3] =T[0];
			PP[4]=R[3]; PP[5]=R[4]; PP[6] =R[5]; PP[7] =T[1];
			PP[8]=R[6]; PP[9]=R[7]; PP[10]=R[8]; PP[11]=T[2];
			CvMatHelper::mul(3,3,3,4,K,PP,P);
		} else { //KR[I,-C], remember now T is C
			P[0]=1; P[1]=0; P[2] =0; P[3] =-T[0];
			P[4]=0; P[5]=1; P[6] =0; P[7] =-T[1];
			P[8]=0; P[9]=0; P[10]=1; P[11]=-T[2];
			CvMatHelper::mul(3,3,3,4,R,P,PP);
			CvMatHelper::mul(3,3,3,4,K,PP,P);
		}
	}
}//CameraHelper

namespace helper {
	using namespace CvMatHelper;
	using namespace CameraHelper;
}