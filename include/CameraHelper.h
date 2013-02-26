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

namespace CameraHelper
{

// s[u,v,1]' = P * [x,y,z,1]'
// P<3x4>, projection matrix
inline void project(double const *P,
                    double x, double y, double z,
                    double &u, double &v)
{
	double w;
	u = P[IDX(0,0,4)] * x + P[IDX(0,1,4)] * y
	    + P[IDX(0,2,4)] * z + P[IDX(0,3,4)] * 1;
	v = P[IDX(1,0,4)] * x + P[IDX(1,1,4)] * y
	    + P[IDX(1,2,4)] * z + P[IDX(1,3,4)] * 1;
	w = P[IDX(2,0,4)] * x + P[IDX(2,1,4)] * y
	    + P[IDX(2,2,4)] * z + P[IDX(2,3,4)] * 1;
	if(fabs(w)<1e-10) {
		u = v = 0;    // ideal point
	} else {
		u/=w;
		v/=w;
	}
}

inline void project(double const *P,
                    double const *X, double *p)
{
	project(P, X[0], X[1], X[2], p[0], p[1]);
}

// K<3x3> * [R<3x3> , T<3x1>] = P<3x4>
// K<3x3> * R<3x3> [I<3x3> , -C<3x1>] = P<3x4>
// T<3x1> = -R<3x3> * C<3x1>
inline void decompose(double const *P,
                      double *K, double *R, double *T, double *C)
{
	cv::Mat mP(3,4,CV_64FC1,const_cast<double*>(P));
	cv::Mat mK(3,3,CV_64FC1,K);
	cv::Mat mR(3,3,CV_64FC1,R);
	double tmp[4];
	cv::Mat mtmp(4,1,CV_64FC1,tmp);
	//translation vector is camera Center in world coordinate system
	//so it is C rather than T in opencv manual
	decomposeProjectionMatrix(mP,mK,mR,mtmp);
	C[0]=tmp[0]/tmp[3];
	C[1]=tmp[1]/tmp[3];
	C[2]=tmp[2]/tmp[3];
	CvMatHelper::mul(3,3,3,1,R,C,T); //T = -RC, C = -R'T
	T[0]*=-1;
	T[1]*=-1;
	T[2]*=-1;
}

inline void compose(double const *K,
                    double const *R, double const *T,
                    double *P, bool TisC=false)
{
	double PP[12]= {0};
	if(!TisC) { //K[R,T]
		PP[0]=R[0], PP[1]=R[1], PP[2]= R[2], PP[3]= T[0];
		PP[4]=R[3], PP[5]=R[4], PP[6]= R[5], PP[7]= T[1];
		PP[8]=R[6], PP[9]=R[7], PP[10]=R[8], PP[11]=T[2];
		CvMatHelper::mul(3,3,3,4,K,PP,P);
	} else { //KR[I,-C], remember now T is C
		P[0]=1, P[1]=0, P[2] =0, P[3] =-T[0];
		P[4]=0, P[5]=1, P[6] =0, P[7] =-T[1];
		P[8]=0, P[9]=0, P[10]=1, P[11]=-T[2];
		CvMatHelper::mul(3,3,3,4,R,P,PP);
		CvMatHelper::mul(3,3,3,4,K,PP,P);
	}
}

inline void RotationMatrix_PH_CV(double *R)
{
	for(int i=1; i<3; ++i)
		for(int j=0; j<3; ++j) {
			R[i*3+j] *= -1;
		}
}

inline bool triangulate(const double x1, const double y1,
                        const double x2, const double y2,
                        const double P1[12], const double P2[12], double X[3])
{
	double A[12] = {
		P1[0]-x1 *P1[8], P1[1]-x1 *P1[9], P1[2]-x1 *P1[10],
		P1[4]-y1 *P1[8], P1[5]-y1 *P1[9], P1[6]-y1 *P1[10],
		P2[0]-x2 *P2[8], P2[1]-x2 *P2[9], P2[2]-x2 *P2[10],
		P2[4]-y2 *P2[8], P2[5]-y2 *P2[9], P2[6]-y2 *P2[10]
	};
	double b[4] = {
		x1 *P1[11]-P1[3],
		y1 *P1[11]-P1[7],
		x2 *P2[11]-P2[3],
		y2 *P2[11]-P2[7]
	};

	return CvMatHelper::solve(4,3,A,b,X);
}

//compute plane pose (R and T) from K and Homography
//H = s^(-1) * K * [r1,r2,T]
//this method assumes the world origin [0,0,0] lies
//in front of the camera, i.e. T[3]>0 is ensured
inline void RTfromKH(const double K[9], const double H[9],
                     double R[9], double T[3])
{
	double invK[9];
	double A[9];
	CvMatHelper::inv(3,K,invK);
	CvMatHelper::mul(3,3,3,3,invK,H,A);
	//as suggested by AprilTag, use geometric average to scale
	double s1 = sqrt(A[0]*A[0]+A[3]*A[3]+A[6]*A[6]);
	double s2 = sqrt(A[1]*A[1]+A[4]*A[4]+A[7]*A[7]);
	double s = (A[8]>=0?1.0:-1.0)/sqrt(s1*s2); //ensure T[3]>0
	if(fabs(A[8])<1e-8) {
		std::cout<<"[RTfromKH warn] T[3]~0, please check!"<<std::endl;
	}
	CvMatHelper::scale(3,3,A,s,A);
	//TODO, should we normalize r1 and r2 respectively?
	//  what's the difference between this and polar decomposition then?
	double r1[3]= {A[0],A[3],A[6]};
	double r2[3]= {A[1],A[4],A[7]};
	double r3[3]= {0};
	CvMatHelper::cross(r1,r2,r3);
	R[0]=r1[0], R[1]=r2[0], R[2]=r3[0];
	R[3]=r1[1], R[4]=r2[1], R[5]=r3[1];
	R[6]=r1[2], R[7]=r2[2], R[8]=r3[2];
	T[0]=A[2],  T[1]=A[5],  T[2]=A[8]; //translation

//simbaforrest: add this will cause affect rendering effect, since lost info
	//as suggested by AprilTag, do polar decomposition so R is orthogonal
	//R = (UV')(VSV')
//	double U[9],S[9],VT[9];
//	CvMatHelper::svd(3,3,R,U,S,VT);
//	CvMatHelper::mul(3,3,3,3,U,VT,R);
}

}//CameraHelper
