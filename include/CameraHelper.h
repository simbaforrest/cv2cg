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
#include "cv.h"
#include "cxcore.h"
#include "highgui.h"
#include "cv.hpp"
#include "cxcore.hpp"
#include "highgui.hpp"
#include "cvwimage.h"

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
