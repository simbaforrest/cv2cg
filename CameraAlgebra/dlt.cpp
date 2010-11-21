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

/* dlt.cpp */

#include "CameraAlgebra.h"

namespace CameraAlgebra {
	// Camear Calibration using Direct Linear Transform(DLT)
	// p npoints*2, image points
	// X npoints*3, world points
	// P 3*4, projection matrix, sp = PX
	bool DLT(int npoints, double const *p, double const *X,
		double *P)
	{
		if(npoints<6) {
			MatrixManip::Log::e(
				"require at least 6 correspondences!",
				__FUNCTION__);
			return false;
		}

		//solution matrix A, dim = (2*npoints)*12
		double *A = new double[2*npoints*12];
		MatrixManip::Zeros(2*npoints,12,A);

		using MatrixManip::FixMat;
		FixMat<2, double const>::Type _p
			= FixMat<2, double const>::ConvertType(p);
		FixMat<3, double const>::Type _X
			= FixMat<3, double const>::ConvertType(X);
		FixMat<12, double>::Type _A
			= FixMat<12, double>::ConvertType(A);

		// Form the solution matrix A
		for(int i=0; i<npoints; ++i) {
			_A[2*i][0] = -1 * _X[i][0];
			_A[2*i][1] = -1 * _X[i][1];
			_A[2*i][2] = -1 * _X[i][2];
			_A[2*i][3] = -1 * 1;
			_A[2*i][8] = _p[i][0] * _X[i][0];
			_A[2*i][9] = _p[i][0] * _X[i][1];
			_A[2*i][10]= _p[i][0] * _X[i][2];
			_A[2*i][11]= _p[i][0] * 1;

			_A[2*i+1][4] = -1 * _X[i][0];
			_A[2*i+1][5] = -1 * _X[i][1];
			_A[2*i+1][6] = -1 * _X[i][2];
			_A[2*i+1][7] = -1 * 1;
			_A[2*i+1][8] = _p[i][1] * _X[i][0];
			_A[2*i+1][9] = _p[i][1] * _X[i][1];
			_A[2*i+1][10]= _p[i][1] * _X[i][2];
			_A[2*i+1][11]= _p[i][1] * 1;
		}
		//MatrixManip::WriteFile(2*npoints, 12, A, 
		//"solutionmatrix.matrix");

		return MatrixManip::NullVector(2*npoints,12,A,P);
	}
}//namespace
