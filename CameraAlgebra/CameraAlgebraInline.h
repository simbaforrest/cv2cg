#ifndef __CAMERA_ALGEBRA_INLINE_HEADER__
#define __CAMERA_ALGEBRA_INLINE_HEADER__
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

/* CameraAlgebraInline.h */

#include "MatrixManip.h"
#include "math.h"

namespace CameraAlgebra {
	// s[u,v,1]' = P * [x,y,z,1]'
	// P 3*4, projection matrix
	inline void Project(double const* P,
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

	inline void Project(double const* P,
		double const *X, double *p)
	{
		Project(P, X[0], X[1], X[2], p[0], p[1]);
	}

	// convert eular angle in photogrammetry manner
	// to rotation matrix in computer vision
	inline void PhAngle2CVRotMat(const double angles[3],
		double *R)
	{
		double phi = angles[0], omega = angles[1], kappa = angles[2];

		double a1,a2,a3,b1,b2,b3,c1,c2,c3;

		a1 = cos(phi) * cos(kappa) - sin(phi) * sin(omega) * sin(kappa);	//a1
		a2 = - cos(phi) * sin(kappa) - sin(phi) * sin(omega) * cos(kappa);	//a2
		a3 = - sin(phi) * cos(omega);											//a3

		b1 = cos(omega) * sin(kappa);											//b1
		b2 = cos(omega) * cos(kappa);											//b2
		b3 = - sin(omega);														//b3

		c1 = sin(phi) * cos(kappa) + cos(phi) * sin(omega) * sin(kappa);	//c1
		c2 = - sin(phi) * sin(kappa) + cos(phi) * sin(omega) * cos(kappa);	//c2
		c3 = cos(phi) * cos(omega);											//c3

		R[0*3+0] =  a1; R[0*3+1] =  b1; R[0*3+2] =  c1;
		R[1*3+0] = -a2; R[1*3+1] = -b2; R[1*3+2] = -c2;
		R[2*3+0] = -a3; R[2*3+1] = -b3; R[2*3+2] = -c3;
	}

	//convert rotation matrix from ph manner to cv manner, or inversely
	inline void RotationMatrix_PH_CV(double *R)
	{
		for(int i=1; i<3; ++i)
			for(int j=0; j<3; ++j)
				R[i*3+j] *= -1;
	}
}//namespace

#endif//__CAMERA_ALGEBRA_INLINE_HEADER__