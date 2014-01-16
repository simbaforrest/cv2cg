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

/* RotationHelper.h
   rotation related helper functions */

#include <math.h>  //sqrt

#include "OpenCVHeaders.h"
#include "CvMatHelper.h"

namespace RotationHelper
{
//Q<4x1> quaternion in the form [w xi yj zk]
//R<3x3> rotation matrix
#define Qw (Q[0])
#define Qx (Q[1])
#define Qy (Q[2])
#define Qz (Q[3])

inline void mat2quat(double const *R, double *Q)
{
	//algorithm from (2011.05.01)
	//	http://www.flipcode.com/documents/matrfaq.html#Q55
	double T = R[0]+R[4]+R[8]+1; //1 + trace of R
	if(T>0) {
		double S = 0.5/sqrt(T);
		Qw = 0.25/S;			// w
		Qx = (R[7]-R[5])*S;	// xi
		Qy = (R[2]-R[6])*S;	// yj
		Qz = (R[3]-R[1])*S;	// zk
	} else if (R[0]>=R[4] && R[0]>=R[8]) { // col 0
		double S = sqrt( 1.0 + R[0] - R[4] - R[8] ) * 2;
		Qx = 0.5 / S;
		Qy = (R[1] + R[3] ) / S;
		Qz = (R[2] + R[6] ) / S;
		Qw = (R[5] + R[7] ) / S;
	} else if (R[4]>=R[0] && R[4]>=R[8]) { // col 1
		double S = sqrt( 1.0 + R[4] - R[0] - R[8] ) * 2;
		Qx = (R[1] + R[3] ) / S;
		Qy = 0.5 / S;
		Qz = (R[5] + R[7] ) / S;
		Qw = (R[2] + R[6] ) / S;
	} else { // col 3
		double S = sqrt( 1.0 + R[8] - R[0] - R[4] ) * 2;
		Qx = (R[2] + R[6] ) / S;
		Qy = (R[5] + R[7] ) / S;
		Qz = 0.5 / S;
		Qw = (R[1] + R[3] ) / S;
	}
}

//may scale Q in case norm(Q)!=1
inline void quat2mat(double *Q, double *R)
{
	//Ensure Q has unit norm
	double s = 1.0/CvMatHelper::normL2(4,1,Q);
	CvMatHelper::scale(4,1,Q,s,Q);

	//Set up convenience variables
	double w2 = Qw*Qw;
	double x2 = Qx*Qx;
	double y2 = Qy*Qy;
	double z2 = Qz*Qz;
	double xy = Qx*Qy;
	double xz = Qx*Qz;
	double yz = Qy*Qz;
	double wx = Qw*Qx;
	double wy = Qw*Qy;
	double wz = Qw*Qz;

	R[0]=w2+x2-y2-z2;
	R[1]=2*(xy - wz);
	R[2]=2*(wy + xz);
	R[3]=2*(wz + xy);
	R[4]=w2-x2+y2-z2;
	R[5]=2*(yz - wx);
	R[6]=2*(xz - wy);
	R[7]=2*(wx + yz);
	R[8]=w2-x2-y2+z2;
}

}// end of RotationHelper
