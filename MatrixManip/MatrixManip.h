#ifndef __MATRIX_MANIP_HEADER__
#define __MATRIX_MANIP_HEADER__
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

/* MatrixManip.h */
/* Matrix Calculations, wrapped from CLapack + CMinpack */
/* Mainly adopted from matrix lib at Bundler by Noah Snavely
   (snavely (at) cs.washington.edu)*/

#include "MatrixInline.h"

namespace MatrixManip {
	/*****************Basic Operation*******************/

	/* Fill a given matrix with an n x n identity matrix */
	void Identity(int n, double *A);
	/* Fill a given matrix with an m x n matrix of zeroes */
	void Zeros(int m, int n, double *A);
	/* Transpose the m x n matrix A and put the result in the n x m matrix AT */
	void Transpose(int m, int n, double const *A, double *AT);
	/* Compute the matrix product R = AB */
	bool Product(int Am, int An, int Bm, int Bn, 
		double const *A, double const *B, double *R);
	/* Compute the matrix product R = A B^T */
	bool ProductABt(int Am, int An, int Bm, int Bn,
		double const *A, double const *B, double *R);
	/* Compute the matrix product R = A^T B */
	bool ProductAtB(int Am, int An, int Bm, int Bn,
		double const *A, double const *B, double *R);
	/* Return the product x**T A x */
	double ProductxtAx(int n, double *A, double *x);
	/* Compute the power of a matrix */
	// Note: in this case, address(A) can be the same as address(R)
	void Power(int n, double const *A, int pow, double *R);
	/* Compute the matrix sum R = A + B */
	// Note: in this case, address(A or B) can be the same as address(R)
	bool Sum(int Am, int An, int Bm, int Bn, 
		double const *A, double const *B, double *R);
	/* Compute the matrix difference R = A - B */
	// Note: in this case, address(A or B) can be the same as address(R)
	bool Diff(int Am, int An, int Bm, int Bn,
		double const *A, double const *B, double *R);
	/* Compute the determinant of a 3x3 matrix */
	double Det3(double const *A);
	/* Scale a matrix by a scalar */
	// Note: in this case, address(A) can be the same as address(R)
	void Scale(int m, int n, double const *A, double s, double *R);
	/* Get the norm of the matrix
	   Note: in fact it is sqrt(sum(sum(A(:,:).*A(:,:)))) */
	double Norm(int m, int n, double const *A);
	/* Get the [squared] norm of the matrix
	   Note: in fact it is sum(sum(A(:,:).*A(:,:))) */
	double NormSq(int m, int n, double const *A);
	void Cross4(double const *u, double const *v, double const *w, 
		double *x);

	/*****************Advanced Operation*******************/
	
	/* LU decomposition of A */
	// P * A = L * U
	// L = lower triangle matrix with unit diagonal elements
	// U = upper triangle matrix
	// P = permutation matrix
	bool LU(int n, double const *A, double* L, double *U, double *P);
	/* Invert the n-by-n matrix A, storing the result in Ainv */
	bool Inv(int n, double const *A, double *Ainv);
	bool InvInplace(int n, double *A);
	/* Compute singular value decomposition of an m x n matrix A */
	// A = U * S * VT
	// A m*n, U m*m, S m*n, VT n*n, U and VT are orthogonal
	bool SVD(int m, int n, double const *A, double *U, double *S, double *VT);
	/* Find the unit vector that minimizes ||Ax|| */
	bool NullVector(int m, int n, double const *A, double *x);
	/* Solve an n x n system */
	bool Solve(int n, double const *A, double const *b, double *x);
	/* Compute Cholesky decomposition of an nxn matrix */
	// A = UT*U, U is a upper triangle matrix
	bool Cholesky(int n, double const *A, double *U);
	/* Compute a QR factorization of an m by n matrix A */
	bool QR(int m, int n, double const *A, double *Q, double *R);
	/* Compute an RQ factorization of an m by n matrix A */
	bool RQ(int m, int n, double const *A, double *R, double *Q);
	/* n: the order of matrix A
	* A: matrix for which the eigenvectors/values are to be computed
	* evec n*n: output array containing the eigenvectors in each cols
	* eval n*1: output array containing the eigenvalues
	* Note: Assumes the results are real! */
	// return real eval count
	int Eigen(int n, double const *A, double *evec, double *eval);
	/* Driver for the lapack function dgelss, which finds x that */
	// Minimize 2-norm(| b - A*x |)
	// A m*n, b n*1, x n*1
	bool Lss(double const *A, double const *b, double *x,
		int m, int n, int nrhs);

	/* Driver for the minpack function lmdif, which uses
	* Levenberg-Marquardt for non-linear least squares minimization */
	void Lmdif(void *fcn, int m, int n, double *xvec, double tol);
	void Lmdif2(void *fcn, int m, int n, double *xvec, double tol);
	void Lmdif3(void *fcn, int m, int n, double *xvec, double tol,
		int maxfev, double *H);

	/*****************Convert*******************/

	/* Convert a rotation matrix to axis and angle representation */
	void Mat2AxisAngle(double *R, double *axis, double *angle);
	void AxisAngle2Mat(double *axis, double angle, double *R);
	void AxisAngle2Mat4(double *axis, double angle, double *R);
	/* Convert a matrix to a normalize quaternion */
	void Mat2Quat(double *R, double *q);
	/* Convert a normalized quaternion to a matrix */
	void Quat2Mat(double *q, double *R);
	/* Decompose a square matrix into an orthogonal matrix and a symmetric
	* positive semidefinite matrix */
	void MatPolarDecomposition(int n, double *A, double *Q, double *S);
	void Slerp(double *v1, double *v2, double t, double *v3);
}//namespace

#pragma comment(linker, "/NODEFAULTLIB:Libcmt.lib")

#endif//__MATRIX_MANIP_HEADER__