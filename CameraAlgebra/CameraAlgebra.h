#ifndef __CAMERA_ALGEBRA_HEADER__
#define __CAMERA_ALGEBRA_HEADER__
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

/* CameraAlgebra.h */

#include "MatrixManip.h"
#include "CameraAlgebraInline.h"

namespace CameraAlgebra {
	/*******************Calibration**********************/
	// Camear Calibration using Direct Linear Transform(DLT)
	// p npoints*2, image points
	// X npoints*3, world points
	// P 3*4, projection matrix, sp = PX
	bool DLT(int npoints, double const *p, double const *X,
		double *P);
	// Evaluate estimated projection matrix P by
	// ReprojectErr=ImagePoint-WorldPoint*P
	// meanErr, mean error of ReprojectErr
	// maxErr, max error of ReprojectErr
	// sumsqErr, sum of squared error of ReprojectErr
	// sd, standard deviation
	// detail error for each points will be reported
	// by MatrixManip::Log::i
	void ReprojectionError(int npoints, double const *p,
		double const *X, double const *P, double& meanErr,
		double& maxErr, double& sumsqErr, double& sd);

	/*******************Utils**********************/
	// compose a projection matrix
	bool Compose(double const* K, double const *C, double const *R,
		double *P);
	// camera matrix (projection matrix) decomposition
	bool Decompose(const double P[3][4],
		double K[3][3], double R[3][3], double C[3], double T[3]);
}//namespace

#endif//__CAMERA_ALGEBRA_HEADER__