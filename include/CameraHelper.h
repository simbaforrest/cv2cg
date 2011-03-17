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

/* CameraHelper.h
   Multiple View Geometry related helper functions */

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

	inline void RotationMatrix_PH_CV(double *R)
	{
		for(int i=1; i<3; ++i)
			for(int j=0; j<3; ++j)
				R[i*3+j] *= -1;
	}

	inline void triangulate(const double x1, const double y1,
		const double x2, const double y2,
		const double P1[12], const double P2[12], double X[3])
	{
		double A[12] = {
			P1[0]-x1*P1[8], P1[1]-x1*P1[9], P1[2]-x1*P1[10],
			P1[4]-y1*P1[8], P1[5]-y1*P1[9], P1[6]-y1*P1[10],
			P2[0]-x2*P2[8], P2[1]-x2*P2[9], P2[2]-x2*P2[10],
			P2[4]-y2*P2[8], P2[5]-y2*P2[9], P2[6]-y2*P2[10]
		};
		double b[4] = {
			x1*P1[11]-P1[3],
			y1*P1[11]-P1[7],
			x2*P2[11]-P2[3],
			y2*P2[11]-P2[7]
		};

		CreateCvMatHead(_A,4,3,A);
		CreateCvMatHead(_b,4,1,b);
		CreateCvMatHead(_X,3,1,X);
		cvSolve(&_A,&_b,&_X, CV_SVD);

		//double A[16] = {
		//	x1*P1[8]-P1[0], x1*P1[9]-P1[1], x1*P1[10]-P1[2], x1*P1[11]-P1[3],
		//	y1*P1[8]-P1[4], y1*P1[9]-P1[5], y1*P1[10]-P1[6], y1*P1[11]-P1[7],
		//	x2*P2[8]-P2[0], x2*P2[9]-P2[1], x2*P2[10]-P2[2], x2*P2[11]-P2[3],
		//	y2*P2[8]-P2[4], y2*P2[9]-P2[5], y2*P2[10]-P2[6], y2*P2[11]-P2[7]
		//};
		//double x[4] = {0};
		//double B[4] = {0};
		////double u[16],w[16],v[16];

		//CreateCVHead(_A,4,4,A);
		//CreateCVHead(_x,4,1,x);
		//CreateCVHead(_B,4,1,B);
		////CreateCVHead(_u,4,4,u);
		////CreateCVHead(_w,4,4,w);
		////CreateCVHead(_v,4,4,v);

		////cvSVD(&_A, &_w, &_u, &_v);
		//cvSolve(&_A, &_B, &_x, CV_SVD);
		//X[0] = x[0]/x[3];
		//X[1] = x[1]/x[3];
		//X[2] = x[2]/x[3];
	}
}//CameraHelper
