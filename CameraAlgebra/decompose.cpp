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

/* decompose.cpp */

#include "CameraAlgebra.h"

namespace CameraAlgebra {
	// compose a projection matrix
	bool Compose(double const* K, double const *C, double const *R,
		double *P)
	{
		double T[3];
		MatrixManip::Product331(R, C, T);
		double RT[3][4] = {
			R[0*3+0],R[0*3+1],R[0*3+2],-T[0],
			R[1*3+0],R[1*3+1],R[1*3+2],-T[1],
			R[2*3+0],R[2*3+1],R[2*3+2],-T[2]
		};
		return MatrixManip::Product(3,3,3,4,K,RT[0],P);
	}

	bool Decompose(const double P[3][4],
		double K[3][3], double R[3][3], double C[3], double T[3])
	{
		//Inspired by vxl vpgl_perspective_decomposition
		// Extract the left sub matrix H from [ H T ]
		// and check that it has rank 3.
		double H[3][3] = {0};
		for(int i=0; i<3; ++i) for(int j=0; j<3; ++j)
			H[i][j] = P[i][j];
		for(int i=0; i<3; ++i)
			T[i] = P[i][3];

		double det = MatrixManip::Det3x3(H[0]);
		if(det == 0) return false;

		// H=K*R, T=-K*R*C, so H*C=-T => C=-inv(H)*T
		double iH[9]={0};
		MatrixManip::Inv(3,H[0],iH);
		MatrixManip::Product331(iH,T,C);
		MatrixManip::Scale(3,1,C,-1,C);

		// To insure a true rotation (determinant = 1) 
		// we must start with a positive determinant H.
		// This is decomposed into K and R, each with 
		// positive determinant.
		if ( det < 0 ) {
			MatrixManip::Scale(3,3,H[0],-1,H[0]);
			MatrixManip::Scale(3,1,T,-1,T);
		}

		// Now find the RQ decomposition of the sub matrix and 
		// use these to find the params.
		if( !MatrixManip::RQ(3,3,H[0],K[0],R[0]) ) return false;

		// We almost have the K and R parameter blocks, but we 
		// must be sure that the diagonal entries of K are positive.
		int r0pos = K[0][0] > 0 ? 1 : -1;
		int r1pos = K[1][1] > 0 ? 1 : -1;
		int r2pos = K[2][2] > 0 ? 1 : -1;
		int diag[3] = { r0pos, r1pos, r2pos };
		for ( int i = 0; i < 3; i++ ){
			for ( int j = 0; j < 3; j++ ){
				K[i][j] *= diag[j];
				R[i][j] *= diag[i];
			}
		}
		MatrixManip::Scale(3,3,K[0],1.0/K[2][2],K[0]);

		MatrixManip::Product331(R[0],C,T);
		MatrixManip::Scale(3,1,T,-1,T);

		return true;
	}

}//namespace
