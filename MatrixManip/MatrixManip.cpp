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

/* MatrixManip.cpp */
/* Matrix Calculations, wrapped from CLapack + CMinpack */
/* Mainly adopted and modified from matrix lib at Bundler by Noah Snavely
   (snavely (at) cs.washington.edu)*/

#include <assert.h>

#include <stdlib.h>
#include <stdio.h>

#include <float.h>
#include <math.h>
#include <string.h>
#include <time.h>

extern"C"
{
#include <f2c.h>
#include <clapack.h>
}

#include "minpack.h"


#include "MatrixManip.h"

namespace MatrixManip {
#define CLAMP(x,mn,mx) (((x) < mn) ? mn : (((x) > mx) ? mx : (x)))

#define LogE(msg) Log::e(msg, __FUNCTION__)
#define LogW(msg) Log::w(msg, __FUNCTION__)
#define LogI(msg) Log::i(msg, __FUNCTION__)
#define LogD(msg) Log::d(msg, __FUNCTION__)
	/*****************Basic Operation*******************/

	/* Fill a given matrix with an n x n identity matrix */
	void Identity(int n, double *A) {
		int i, j;

		for (i = 0; i < n; i++) {
			for (j = 0; j < n; j++) {
				if (i == j)
					A[i * n + j] = 1.0;
				else
					A[i * n + j] = 0.0;
			}
		}
	}

	/* Fill a given matrix with an m x n matrix of zeroes */
	void Zeros(int m, int n, double *A) {
#if 1
		int i, j;
		for (i = 0; i < m; i++) {
			for (j = 0; j < n; j++) {
				A[i * n + j] = 0.0;
			}
		}
#else
		memset(A, 0, sizeof(double)*m*n);
#endif
	}

	/* Transpose the m x n matrix A and put the result in the n x m matrix AT */
	void Transpose(int m, int n, double const *A, double *AT) {
		if(A==AT) {
			LogW("address(A)==address(AT)!!!");
		}

		int i, j;
		for (i = 0; i < m; i++)
			for (j = 0; j < n; j++)
				AT[j * m + i] = A[i * n + j];
	}

	/* Compute the matrix product R = AB */
	bool Product(int Am, int An, int Bm, int Bn, 
		double const *A, double const *B, double *R) {
			bool ret = true;
			int r = Am;
			int c = Bn;
			int m = An;

			int i, j, k;

			if(A==R || B==R) {
				LogW("address(A)==address(R) or address(B)==address(R)!!!");
				ret = false;
			}

			if (An != Bm) {
				LogE("the number of columns of A and the number of rows of B must be equal");
				return false;
			}

			for (i = 0; i < r; i++) {
				for (j = 0; j < c; j++) {
					R[i * c + j] = 0.0;
					for (k = 0; k < m; k++) {
						R[i * c + j] += A[i * An + k] * B[k * Bn + j];
					}
				}
			}
			return ret;
	}

	/* Compute the power of a matrix */
	void Power(int n, double const *A, int pow, double *R)
	{
		int i;

		/* Slow way */
		double *curr = (double *) malloc(sizeof(double) * n * n);
		double *tmp = (double *) malloc(sizeof(double) * n * n);

		Identity(n, curr);
		for (i = 0; i < pow; i++) {
			Product(n, n, n, n, curr, A, tmp);
			memcpy(curr, tmp, sizeof(double) * n * n);
		}

		memcpy(R, curr, sizeof(double) * n * n);

		free(curr);
		free(tmp);
	}

	/* Compute the matrix sum R = A + B */
	bool Sum(int Am, int An, int Bm, int Bn, 
		double const *A, double const *B, double *R) {
			bool ret = true;
			int r = Am;
			int c = An;
			int n = r * c, i;

			if (Am != Bm || An != Bn) {
				LogE("mismatched dimensions");
				return false;
			}

			for (i = 0; i < n; i++) {
				R[i] = A[i] + B[i];
			}
			return ret;
	}

	/* Compute the matrix difference R = A - B */
	bool Diff(int Am, int An, int Bm, int Bn,
		double const *A, double const *B, double *R) {
		bool ret = true;
		int r = Am;
		int c = An;
		int n = r * c, i;

		if (Am != Bm || An != Bn) {
			LogE("mismatched dimensions");
			return false;
		}

		for (i = 0; i < n; i++) {
			R[i] = A[i] - B[i];
		}
		return ret;
	}

	/* Compute the matrix product R = A^T B */
	bool ProductAtB(int Am, int An, int Bm, int Bn,
		double const *A, double const *B, double *R) {
		bool ret = true;
		int r = An;
		int c = Bn;
		int m = Am;

		int i, j, k;

		if(A==R || B==R) {
			LogW("address(A)==address(R) or address(B)==address(R)!!!");
			ret = false;
		}

		if (Am != Bm) {
			LogE("the number of rows of A and the number of rows of B must be equal!");
			return false;
		}

		for (i = 0; i < r; i++) {
			for (j = 0; j < c; j++) {
				R[i * c + j] = 0.0;
				for (k = 0; k < m; k++) {
					R[i * c + j] += A[k * An + i] * B[k * Bn + j];
				}
			}
		}
		return ret;
	}

	/* Compute the matrix product R = A B^T */
	bool ProductABt(int Am, int An, int Bm, int Bn,
		double const *A, double const *B, double *R) {
		bool ret = true;
		int r = Am;
		int c = Bm;
		int m = An;

		int i, j, k;

		if(A==R || B==R) {
			LogW("address(A)==address(R) or address(B)==address(R)!!!");
			ret = false;
		}

		if (An != Bn) {
			LogE("the number of columns of A and the number of columns of B must be equal");
			return false;
		}

		for (i = 0; i < r; i++) {
			for (j = 0; j < c; j++) {
				R[i * c + j] = 0.0;
				for (k = 0; k < m; k++) {
					R[i * c + j] += A[i * An + k] * B[j * Bn + k];
				}
			}
		}
		return ret;
	}

	/* Return the product x**T A x */
	double ProductxtAx(int n, double *A, double *x) {
		double *tmp = (double *)malloc(sizeof(double) * n);
		double result;

		if(A==x) {
			LogE("address(A)==address(x)!");
			return DBL_MAX;
		}

		Product(n, n, n, 1, A, x, tmp);
		Product(1, n, n, 1, x, tmp, &result);

		free(tmp);

		return result;
	}

	/* Scale a matrix by a scalar */
	void Scale(int m, int n, double const *A, double s, double *R) {
		int i;
		int entries = m * n;

		for (i = 0; i < entries; i++) {
			R[i] = A[i] * s;
		}
	}

		/* Compute the determinant of a 3x4 matrix */
	double Det3(double const *A)
	{
		/* 0 1 2
		* 3 4 5
		* 6 7 8 */

		return 
			A[0] * (A[4] * A[8] - A[5] * A[7]) -
			A[1] * (A[3] * A[8] - A[5] * A[6]) +
			A[2] * (A[3] * A[7] - A[4] * A[6]);
	}

	/* Get the norm of the matrix */
	double Norm(int m, int n, double const *A) {
		double sum = 0.0;
		int i;
		int entries = m * n;

		for (i = 0; i < entries; i++) {
			sum += A[i] * A[i];
		}

		return sqrt(sum);
	}

	/* Get the [squared] norm of the matrix */
	double NormSq(int m, int n, double const *A) 
	{
		double sum = 0.0;
		int i;
		int entries = m * n;

		for (i = 0; i < entries; i++) {
			sum += A[i] * A[i];
		}

		return sum;
	}

	/* Cross three 4x1 vectors */
	void Cross4(double const *u, double const *v, double const *w, 
		double *x)
	{
		double sub1[9] = 
		{ u[1], u[2], u[3],
		v[1], v[2], v[3],
		w[1], w[2], w[3] };

		double sub2[9] = 
		{ u[0], u[2], u[3],
		v[0], v[2], v[3],
		w[0], w[2], w[3] };

		double sub3[9] = 
		{ u[0], u[1], u[3],
		v[0], v[1], v[3],
		w[0], w[1], w[3] };

		double sub4[9] = 
		{ u[0], u[1], u[2],
		v[0], v[1], v[2],
		w[0], w[1], w[2] };

		double det1 = Det3(sub1);
		double det2 = Det3(sub2);
		double det3 = Det3(sub3);
		double det4 = Det3(sub4);

		x[0] = det1;
		x[1] = det2;
		x[2] = det3;
		x[3] = det4;
	}

	/*****************Advanced Operation*******************/

	/* LU decomposition of A */
	// P * A = L * U
	// L = lower triangle matrix with unit diagonal elements
	// U = upper triangle matrix
	// P = permutation matrix
	bool LU(int n, double const *A, double* L, double *U, double *P)
	{
		double *At = (double*)malloc(sizeof(double) * n * n);
		int *ipiv = (int*)malloc(sizeof(int) * n);
		int m = n;
		int lda = n;
		int info;
		int i, j;

		/* Transpose A info At like FORTRAN likes */
		Transpose(n,n, A, At);

		/* Make calls to FORTRAN routines */
		dgetrf_((integer*)&m, (integer*)&n, At, (integer*)&lda,
			(integer*)ipiv, (integer*)&info);

		if (info != 0) {
			char msg[100];
			sprintf_s<100>(msg, "dgetrf_ exited with error code %d\n", info);
			LogE(msg);
			free(At);
			free(ipiv);
			return false;
		}

		Identity(n, L);
		Zeros(n,n, U);
		Identity(n, P);
		for(i = 0; i < n; ++i) {
			//lower
			for(j = 0; j < i; ++j)
				//At(j,i) since it is still in fortran manner
				L[IDX(i,j,n)] = At[IDX(j,i,n)];
			//upper
			for(j = i; j < n; ++j)
				//At(j,i) since it is still in fortran manner
				U[IDX(i,j,n)] = At[IDX(j,i,n)];
			//ipiv
			if(ipiv[i] != (i+1))
				SwapRow(n,n, P, i, ipiv[i]-1);
		}

		free(At);
		free(ipiv);
		return true;
	}

	bool Inv(int n, double const *A, double *Ainv) {
		double *At = (double*)malloc(sizeof(double) * n * n);
		int m = n;
		int lda = n;
		int info;
		int *ipiv = (int*)malloc(sizeof(int) * n);
		int lwork = n * 512;
		int i, j;
		double *work = (double*)malloc(sizeof(double) * lwork);

		assert(At != NULL);
		assert(ipiv != NULL);
		assert(work != NULL);

		/* Transpose A info At like FORTRAN likes */
		for (i = 0; i < n; i++)
			for (j = 0; j < n; j++)
				At[i * n + j] = A[j * n + i];

		/* Make calls to FORTRAN routines */
		dgetrf_((integer*)&m, (integer*)&n, At, (integer*)&lda,
			(integer*)ipiv, (integer*)&info);
		if (info != 0) {
			char msg[100];
			sprintf_s<100>(msg, "dgetrf_ exited with error code %d\n", info);
			LogE(msg);
			free(At);
			free(ipiv);
			free(work);
			return false;
		}

		dgetri_((integer*)&n, At, (integer*)&lda, (integer*)ipiv,
			work, (integer*)&lwork, (integer*)&info);
		if (info != 0) {
			char msg[100];
			sprintf_s<100>(msg, "dgetri_ exited with error code %d\n", info);
			LogE(msg);
			free(At);
			free(ipiv);
			free(work);
			return false;
		}

		/* Transpose back into Ainv */
		for (i = 0; i < n; i++)
			for (j = 0; j < n; j++)
				Ainv[i * n + j] = At[j * n + i];

		free(At);
		free(ipiv);
		free(work);
		return true;
	}

	bool InvInplace(int n, double *A) {
		int m = n;
		int lda = n;
		int info;
		int *ipiv = (int*)malloc(sizeof(int) * n);
		int lwork = n * 512;
		double *work = (double*)malloc(sizeof(double) * lwork);

		/* Make calls to FORTRAN routines */
		dgetrf_((integer*)&m, (integer*)&n, A, (integer*)&lda,
			(integer*)ipiv, (integer*)&info);
		if (info != 0) {
			char msg[100];
			sprintf_s<100>(msg, "dgetrf_ exited with error code %d\n", info);
			LogE(msg);
			return false;
		}

		dgetri_((integer*)&n, A, (integer*)&lda, (integer*)ipiv,
			work, (integer*)&lwork, (integer*)&info);
		if (info != 0) {
			char msg[100];
			sprintf_s<100>(msg, "dgetri_ exited with error code %d\n", info);
			LogE(msg);
			return false;
		}

		free(ipiv);
		free(work);
		return true;
	}

	/* Compute singular value decomposition of an m x n matrix A */
	bool SVD(int m, int n, double const *A, double *U, double *S, double *VT) {
		double *AT, *UT, *V;

		char jobu = 'a';
		char jobvt = 'a';

		int lda = m;
		int ldu = m;
		int ldvt = n;

		int lwork = 10 * max(3 * min(m, n) + max(m, n), 5 * min(m, n));
		double *work;

		int info;

		/* Transpose A */
		AT = (double *)malloc(sizeof(double) * m * n);    
		Transpose(m, n, A, AT);

		/* Create temporary matrices for output of dgesvd */
		UT = (double *)malloc(sizeof(double) * m * m);
		V = (double *)malloc(sizeof(double) * n * n);

		work = (double*)malloc(sizeof(double) * lwork);

		dgesvd_(&jobu, &jobvt, (integer*)&m, (integer*)&n, AT,
			(integer*)&lda, S, UT, (integer*)&ldu, V,
			(integer*)&ldvt, work, (integer*)&lwork, (integer*)&info);

		if (info != 0) {
			char msg[100];
			sprintf_s<100>(msg, "dgesvd_ exited with error code %d\n", info);
			LogE(msg);
			free(AT);
			free(UT); 
			free(V);
			free(work);
			return false;
		}

		Transpose(m, m, UT, U);
		Transpose(n, n, V, VT);

		double* tmpS = (double*)malloc(sizeof(double)*min(m,n));
		for(int i=0; i<min(m,n); ++i)
			tmpS[i] = S[i];

		Zeros(m,n,S);
		for(int i=0; i<min(m,n); ++i)
			S[IDX(i,i,n)] = tmpS[i];

		free(tmpS);
		free(AT);
		free(UT); 
		free(V);
		free(work);
		return true;
	}

	/* Compute singular value decomposition of an m x n matrix A *
	* (only compute S and VT) */
	// not in header now
	bool SVDvt(int m, int n, double const *A, double *S, double *VT) {
		double *AT, *V;

		char jobu = 'n';
		char jobvt = 'a';

		int lda = m;
		int ldu = m;
		int ldvt = n;

		int lwork = 10 * max(3 * min(m, n) + max(m, n), 5 * min(m, n));
		double *work;

		int info;

		bool ret=true;

		/* Transpose A */
		AT = (double *)malloc(sizeof(double) * m * n);    
		Transpose(m, n, A, AT);

		/* Create temporary matrices for output of dgesvd */
		V = (double *)malloc(sizeof(double) * n * n);

		work = (double*)malloc(sizeof(double) * lwork);

		dgesvd_(&jobu, &jobvt, (integer*)&m, (integer*)&n, AT,
			(integer*)&lda, S, NULL, (integer*)&ldu, V,
			(integer*)&ldvt, work, (integer*)&lwork, (integer*)&info);

		if (info != 0) {
			char msg[100];
			sprintf_s<100>(msg, 
				"dgesvd_ exited with error code %d\n", info);
			LogE(msg);
			ret = false;
		}

		// Transpose(m, m, UT, U);
		Transpose(n, n, V, VT);

		free(AT);
		free(V);
		free(work);

		return ret;
	}

	/* Solve an n x n system */
	bool Solve(int n, double const *A, double const *b, double *x) {
		double *Atmp = (double*)malloc(sizeof(double) * n * n);
		double *btmp = (double*)malloc(sizeof(double) * n);

		int nrhs = 1;
		int lda = n;
		int ldb = n;
		int *ipiv = (int*)calloc(sizeof(int), n);

		int info;

		int i, j;

		/* Go from row- to column-major */
		for (i = 0; i < n; i++)
			for (j = 0; j < n; j++)
				Atmp[j * n + i] = A[i * n + j];

		for (i = 0; i < n; i++)
			btmp[i] = b[i];

		/* Make the FORTRAN call */
		dgesv_((integer*)&n, (integer*)&nrhs, Atmp,
			(integer*)&lda, (integer*)ipiv, btmp,
			(integer*)&ldb, (integer*)&info);

		if (info != 0) {
			char msg[100];
			sprintf_s<100>(msg, "Error [%d] in call to dgesv_", info);
			LogE(msg);
			free(Atmp);
			free(btmp);
			free(ipiv);
			return false;
		}

		/* Go from column- to row-major */
		for (i = 0; i < n; i++)
			x[i] = btmp[i];

		free(Atmp);
		free(btmp);
		free(ipiv);
		return true;
	}

	/* Compute Cholesky decomposition of an nxn matrix */
	// A = UT*U, U is a upper triangle matrix
	bool Cholesky(int n, double const *A, double *U) {
		double *AT;
		char uplo = 'U';
		int lda = n;
		int info;

		/* Transpose A */
		AT = (double *)malloc(sizeof(double) * n * n);
		Transpose(n, n, A, AT);

		/* Call lapack routine */
		dpotrf_(&uplo, (integer*)&n, AT,
			(integer*)&lda, (integer*)&info);
		if (info != 0) {
			char msg[100];
			sprintf_s<100>(msg, "dpotrf_ exited with error code %d\n", info);
			LogE(msg);
			free(AT);
			return false;
		}

		/* Transpose AT */
		Transpose(n, n, AT, U);

		//wipe out unref value in U
		for(int i=0; i<n; ++i) for(int j=0; j<i; ++j)
			U[IDX(i,j,n)]=0;

		free(AT);
		return true;
	}

	/* Compute a QR factorization of an m by n matrix A */
	bool QR(int m, int n, double const *A, double *Q, double *R)
	{
		double *AT;
		int lda = m;
		double *tau;
		int tau_dim = min(m, n);
		double *work;
		int block_size = 64; /* Just a guess... */
		int lwork = n * block_size;
		int info;
		double *H;
		double *v, *vvT;
		double *Qtmp;

		int i, j;

		/* Transpose A */
		AT = (double *) malloc(sizeof(double) * m * n);
		Transpose(m, n, A, AT);

		/* Call the LAPACK routine */
		tau = (double *) malloc(sizeof(double) * tau_dim);
		work = (double *) malloc(sizeof(double) * lwork);
		dgeqrf_((integer*)&m, (integer*)&n, AT, (integer*)&lda,
			tau, work, (integer*)&lwork, (integer*)&info);

		if (info < 0) {
			char msg[100];
			sprintf_s<100>(msg, "dgeqrf_ exited with error code %d\n", info);
			LogE(msg);

			free(AT);
			free(work);
			free(tau);

			return false;
		}

		/* Extract the R matrix */
		for (i = 0; i < m; i++) {
			for (j = 0; j < n; j++) {
				if (j < i)
					R[i * n + j] = 0.0;
				else
					R[i * n + j] = AT[j * m + i];
			}
		}


		/* Now extract the Q matrix */
		H = (double *) malloc(sizeof(double) * m * m);
		v = (double *) malloc(sizeof(double) * m);
		vvT = (double *) malloc(sizeof(double) * m * m);
		Qtmp = (double *) malloc(sizeof(double) * m * m);

		for (i = 0; i < tau_dim; i++) {
			Identity(m, H);

			for (j = 0; j < m; j++) {
				if (j < i)
					v[j] = 0.0;
				else if (j == i)
					v[j] = 1.0;
				else
					v[j] = AT[i * m + j];
			}

			ProductABt(m, 1, m, 1, v, v, vvT);
			Scale(m, m, vvT, tau[i], vvT);	
			Diff(m, m, m, m, H, vvT, H);

			if (i == 0) {
				memcpy(Q, H, sizeof(double) * m * m);
			} else {
				Product(m, m, m, m, Q, H, Qtmp);
				memcpy(Q, Qtmp, sizeof(double) * m * m);
			}
		}

		free(H);
		free(v);
		free(vvT);
		free(Qtmp);

		free(tau);
		free(work);
		free(AT);
		return true;
	}

	/* Compute an RQ factorization of an m by n matrix A */
	bool RQ(int m, int n, double const *A, double *R, double *Q)
	{
		double *AT;
		int lda = m;
		double *tau;
		int tau_dim = min(m, n);
		double *work;
		int block_size = 64; /* Just a guess... */
		int lwork = n * block_size;
		int info;
		double *H;
		double *v, *vvT;
		double *Qtmp;

		int i, j;

		/* Transpose A */
		AT = (double *) malloc(sizeof(double) * m * n);
		Transpose(m, n, A, AT);

		/* Call the LAPACK routine */
		tau = (double *) malloc(sizeof(double) * tau_dim);
		work = (double *) malloc(sizeof(double) * lwork);
		dgerqf_((integer*)&m, (integer*)&n, AT, (integer*)&lda,
			tau, work, (integer*)&lwork, (integer*)&info);

		if (info < 0) {
			char msg[100];
			sprintf_s<100>(msg, "dgerqf_ exited with error code %d\n", info);
			LogE(msg);

			free(AT);
			free(work);
			free(tau);

			return false;
		}

		/* Extract the R matrix */
		for (i = 0; i < m; i++) {
			for (j = 0; j < n; j++) {
				if (j < i)
					R[i * n + j] = 0.0;
				else
					R[i * n + j] = AT[(n - m + j) * m + i];
			}
		}


		/* Now extract the Q matrix */
		H = (double *) malloc(sizeof(double) * n * n);
		v = (double *) malloc(sizeof(double) * n);
		vvT = (double *) malloc(sizeof(double) * n * n);
		Qtmp = (double *) malloc(sizeof(double) * n * n);

		for (i = 0; i < tau_dim; i++) {
			Identity(m, H);

			for (j = 0; j < n; j++) {
				if (j > n - tau_dim + i)
					v[j] = 0.0;
				else if (j == n - tau_dim + i)
					v[j] = 1.0;
				else
					v[j] = AT[j * m + (m-tau_dim+i)];
			}

			ProductABt(n, 1, n, 1, v, v, vvT);
			Scale(n, n, vvT, tau[i], vvT);
			Diff(n, n, n, n, H, vvT, H);

			if (i == 0) {
				memcpy(Q, H, sizeof(double) * n * n);
			} else {
				Product(n, n, n, n, Q, H, Qtmp);
				memcpy(Q, Qtmp, sizeof(double) * n * n);
			}
		}

		Product(m, n, n, n, R, Q, H);

		free(H);
		free(v);
		free(vvT);
		free(Qtmp);

		free(tau);
		free(work);
		free(AT);
		return true;
	}

	/* n: the order of matrix A
	* A: matrix for which the eigenvectors/values are to be computed
	* evec n*n: output array containing the eigenvectors in each cols
	* eval n*1: output array containing the eigenvalues
	* Note: Assumes the results are real! */
	int Eigen(int n, double const *A, double *evec, double *eval) {
		char jobvl = 'N';  /* Don't compute left eigenvectors */
		char jobvr = 'V';  /* Do compute right eigenvectors */
		int lda = n;
		double *Atmp = (double *)malloc(sizeof(double) * n * n);
		double *wr = (double *)malloc(sizeof(double) * n);
		double *wi = (double *)malloc(sizeof(double) * n);
		double *vl = NULL;
		int ldvl = 1;
		double *vr = (double *)malloc(sizeof(double) * n * n);
		int ldvr = n;
		int lwork;
		double *work, work_query[1];
		int info;

		int i, j, count = 0;

		/* Transpose the matrix for FORTRAN */
		for (i = 0; i < n; i++) {
			for (j = 0; j < n; j++) {
				if (A[i * n + j] != A[i * n + j]) {
					LogE("NaN encountered");

					free(Atmp);
					free(wr);
					free(wi);
					free(vr);

					return 0;
				}

				Atmp[j * n + i] = A[i * n + j];
			}
		}

		/* Query dgeev for the optimal value of lwork */
		lwork = -1;
		dgeev_(&jobvl, &jobvr, (integer *)(&n), Atmp,
			(integer *)(&lda), wr, wi, vl, (integer *)(&ldvl),
			vr, (integer*)&ldvr, work_query, (integer*)&lwork,
			(integer*)&info);
		lwork = (int) work_query[0];
		work = (double*)malloc(sizeof(double) * lwork);

		/* Make the call to dgeev */
		dgeev_(&jobvl, &jobvr, (integer*)&n, Atmp, (integer*)&lda, wr, wi, vl, (integer*)&ldvl, vr, (integer*)&ldvr, work, (integer*)&lwork, (integer*)&info);

		if (info < 0) {
			char msg[100];
			sprintf_s<100>(msg, "In call to dgeev (argument %d was invalid)", -info);
			LogE(msg);
		} else if (info > 0) {
			LogE("Not all eigenvalues have converged");
		}

		/* Check that all eigenvalues are real */
		for (i = 0; i < n; i++) {
			if (wi[i] != 0.0) {
				// printf("[dgeev] Eigenvalue has non-zero imaginary part\n");
			} else {
				eval[count] = wr[i];

				for (j = 0; j < n; j++)
					evec[j * n + count] = vr[i * n + j]; //cols are eigen vecs
					//evec[count * n + j] = vr[i * n + j]; //original: rows are eigen vecs

				count++;
			}
		}

		/* Clean up */
		free(work);
		free(Atmp);
		free(wr);
		free(wi);
		free(vr);

		return count;
	}

	typedef void (*Lmdif_Func)(const int *m, const int *n, const double *x, double *fvec,
			   int *iflag );

	inline const char* Lmdif_parse_info(int info) {
		switch (info) {
		case 0:	return ("Improper input parameters");
		case 1:	return ("Sum of squares tolerance reached");
		case 2:	return ("x is within tolerance");
		case 3:	return ("Sum of squares and x are within tolerance");
		case 4:	return ("fvec orthogonal");
		case 5:	return ("max function calls made");
		case 6:	return ("tolerance is too small (squares)");
		case 7:	return ("tolerance is too small (x)");
		}
		return ("Lmdif_parse_info???");
	}

	void Lmdif(void *fcn, int m, int n, double *xvec, double tol) {
		int info;
		int lwa = m * n + 5 * n + m;
		int *iwa;
		double *fvec, *wa;

		if (n > m) {
			LogE("lmdif called with n > m");
			return;
		}

		iwa = (int *)malloc(sizeof(int) * n);
		fvec = (double *)malloc(sizeof(double) * m);
		wa = (double *)malloc(sizeof(double) * lwa);

		//#ifdef WIN32
		//    LMDIF1(fcn, &m, &n, xvec, fvec, &tol, &info, iwa, wa, &lwa);
		//#else
		Lmdif_Func func = (Lmdif_Func)fcn;
		lmdif1_(func, &m, &n, xvec, fvec, &tol, &info, iwa, wa, &lwa);
		//#endif

		free(iwa);
		free(fvec);
		free(wa);
	}

	void Lmdif2(void *fcn, int m, int n, double *xvec, double tol) {
		int info;
		double *fvec;
		double gtol = 0, epsfcn = 0;
		int maxfev = 200 * (n + 1);
		double *diag;
		int mode = 1;
		double factor = 100;
		int nprint = 1;
		int nfev;
		double *fjac;
		int ldfjac = m;
		int *ipvt;
		double *qtf;
		double *wa1, *wa2, *wa3, *wa4;

		if (n > m) {
			LogE("lmdif called with n > m");
			return;
		}

		fvec = (double *)malloc(sizeof(double) * m);
		diag = (double *)malloc(sizeof(double) * n);
		fjac = (double *)malloc(sizeof(double) * m * n);
		ipvt = (int *)malloc(sizeof(int) * n);
		qtf = (double *)malloc(sizeof(double) * n);
		wa1 = (double *)malloc(sizeof(double) * n);
		wa2 = (double *)malloc(sizeof(double) * n);
		wa3 = (double *)malloc(sizeof(double) * n);
		wa4 = (double *)malloc(sizeof(double) * m);

		//#ifdef WIN32
		//    LMDIF(fcn, &m, &n, xvec, fvec, &tol, &tol, &gtol, &maxfev, 
		//           &epsfcn, diag, &mode, &factor, &nprint, &info, &nfev, 
		//           fjac, &ldfjac, ipvt, qtf, wa1, wa2, wa3, wa4);
		//#else
		Lmdif_Func func = (Lmdif_Func)fcn;
		lmdif_(func, &m, &n, xvec, fvec, &tol, &tol, &gtol, &maxfev, 
			&epsfcn, diag, &mode, &factor, &nprint, &info, &nfev, 
			fjac, &ldfjac, ipvt, qtf, wa1, wa2, wa3, wa4);
		//#endif

		LogI( Lmdif_parse_info(info) );

		/* Clean up */
		free(fvec);
		free(diag);
		free(fjac);
		free(ipvt);
		free(qtf);
		free(wa1);
		free(wa2);
		free(wa3);
		free(wa4);
	}

	void Lmdif3(void *fcn, int m, int n, double *xvec, double tol,
		int maxfev, double *H) {
			int info;
			double *fvec;
			double gtol = 0, epsfcn = 0;
			// int maxfev = 200 * (n + 1);
			double *diag;
			int mode = 1;
			double factor = 100;
			int nprint = 1;
			int nfev;
			double *fjac;
			int ldfjac = m;
			int *ipvt;
			double *qtf;
			double *wa1, *wa2, *wa3, *wa4;

			if (maxfev == -1)
				maxfev = 200 * (n + 1);

			if (n > m) {
				LogE("lmdif called with n > m");
				return;
			}

			fvec = (double *)malloc(sizeof(double) * m);
			diag = (double *)malloc(sizeof(double) * n);
			fjac = (double *)malloc(sizeof(double) * m * n);
			ipvt = (int *)malloc(sizeof(int) * n);
			qtf = (double *)malloc(sizeof(double) * n);
			wa1 = (double *)malloc(sizeof(double) * n);
			wa2 = (double *)malloc(sizeof(double) * n);
			wa3 = (double *)malloc(sizeof(double) * n);
			wa4 = (double *)malloc(sizeof(double) * m);

			//#ifdef WIN32
			//    LMDIF(fcn, &m, &n, xvec, fvec, &tol, &tol, &gtol, &maxfev, 
			//           &epsfcn, diag, &mode, &factor, &nprint, &info, &nfev, 
			//           fjac, &ldfjac, ipvt, qtf, wa1, wa2, wa3, wa4);
			//#else
			Lmdif_Func func = (Lmdif_Func)fcn;
			lmdif_(func, &m, &n, xvec, fvec, &tol, &tol, &gtol, &maxfev, 
				&epsfcn, diag, &mode, &factor, &nprint, &info, &nfev, 
				fjac, &ldfjac, ipvt, qtf, wa1, wa2, wa3, wa4);
			//#endif
			
			LogI( Lmdif_parse_info(info) );

			/* Copy the Hessian in fjac to H */
			if (H != NULL) {
				int i, j;
				double *P, *tmp;
				double *R;
				clock_t start, end;

				R = (double*)malloc(sizeof(double) * n * n);

				for (i = 0; i < n; i++) {
					for (j = 0; j < n; j++) {
						if (j < i) {
							R[i * n + j] = 0.0;
						} else {
							int idx0 = j * m + i;
							int idx1 = i * n + j;

							R[idx1] = fjac[idx0];
						}
					}
				}

				start = clock();

				ProductAtB(n, n, n, n, R, R, H);
				// cblas_dgemm_driver_transpose(n, n, n, R, R, H);

				/* Unpermute the entries of H */
				P = (double*)malloc(sizeof(double) * n * n);

				for (i = 0; i < n; i++) {
					int p = ipvt[i] - 1;
					if (p < 0)
						LogE("p < 0!");

					for (j = 0; j < n; j++) {
						if (p == j) 
							P[i * n + j] = 1.0;
						else
							P[i * n + j] = 0.0;
					}
				}


				tmp = (double*)malloc(sizeof(double) * n * n);

				ProductAtB(n, n, n, n, P, H, tmp);
				Product(n, n, n, n, tmp, P, H);
				// cblas_dgemm_driver_transpose(n, n, n, P, H, tmp);
				// cblas_dgemm_driver(n, n, n, tmp, P, H);

				end = clock();

				char msg[100];
				sprintf_s<100>(msg, "post-process took %0.3fs",
					(double) (end - start) / CLOCKS_PER_SEC);
				LogI(msg);

				free(P);
				free(tmp);
			}

			/* Clean up */
			free(fvec);
			free(diag);
			free(fjac);
			free(ipvt);
			free(qtf);
			free(wa1);
			free(wa2);
			free(wa3);
			free(wa4);
	}


	bool Lss(double const *A, double const *b, double *x, 
		int m, int n, int nrhs) {
		if (m < n) {
			LogE("driver now only works when m >= n");
			return false;
		}
		double *Atmp = (double*)malloc(sizeof(double) * m * n);
		double *btmp = (double*)malloc(sizeof(double) * m * nrhs);
		int lda = m;
		int ldb = m;
		double *s = (double*)malloc(sizeof(double) * n); /* Output array */
		double rcond = -1.0;
		int rank; /* Output */
		int lwork = 16 * (3 * min(m, n) + 
			max(max(2 * min(m, n), max(m, n)), nrhs));
		double *work = (double*)malloc(sizeof(double) * lwork);
		int info;

		int i, j;

		/* Go from row- to column-major */
		for (i = 0; i < m; i++)
			for (j = 0; j < n; j++)
				Atmp[j * m + i] = A[i * n + j];

		for (i = 0; i < m; i++)
			for (j = 0; j < nrhs; j++)
				btmp[j * m + i] = b[i * nrhs + j];

		/* Make the FORTRAN call */
		dgelss_((integer*)&m, (integer*)&n, (integer*)&nrhs,
			Atmp, (integer*)&lda, btmp, (integer*)&ldb, 
			s, &rcond, (integer*)&rank, work, (integer*)&lwork,
			(integer*)&info);
		if (info != 0) {
			char msg[100];
			sprintf_s<100>(msg, "dgelss_ exited with error code %d\n", info);
			LogE(msg);
			free(Atmp);
			free(btmp);
			free(s);
			free(work);
			return false;
		}

		/* Go from column- to row-major */
		for (i = 0; i < n; i++)
			for (j = 0; j < nrhs; j++)
				x[i * nrhs + j] = btmp[j * m + i];

		free(Atmp);
		free(btmp);
		free(s);
		free(work);
		return true;
	}

	/* Find the unit vector that minimizes ||Ax|| */
	bool NullVector(int m, int n, double const *A, double *x)
	{
		/* Do an SVD */
		double *S, *VT;
		int num_svs = min(m, n);
		int i, min_idx = -1;
		double min_sv = DBL_MAX;

		VT = (double *) malloc(sizeof(double) * n * n);
		S = (double *) malloc(sizeof(double) * num_svs);

		bool ret = true;
		if( !SVDvt(m, n, A, S, VT) )
			ret = false;

		// Print(n, n, VT);
		// Print(1, num_svs, S);

		/* Return the column of V associated with the smallest singular
		* value */

		for (i = 0; i < num_svs; i++) {
			if (S[i] < min_sv) {
				min_sv = S[i];
				min_idx = i;
			}
		}

		for (i = 0; i < n; i++) {
			x[i] = VT[min_idx * n + i];
		}

		// free(U);
		free(VT);
		free(S);
		return ret;
	}

	/*****************Convert*******************/

	/* Convert a rotation matrix to axis and angle representation */
	void Mat2AxisAngle(double *R, double *axis, double *angle) {
		double d1 = R[7] - R[5];
		double d2 = R[2] - R[6];
		double d3 = R[3] - R[1];

		double norm = sqrt(d1 * d1 + d2 * d2 + d3 * d3);
		double x = (R[7] - R[5]) / norm;
		double y = (R[2] - R[6]) / norm;
		double z = (R[3] - R[1]) / norm;

		*angle = acos((R[0] + R[4] + R[8] - 1.0) * 0.5);

		axis[0] = x;
		axis[1] = y;
		axis[2] = z;
	}

	void AxisAngle2Mat(double *axis, double angle, double *R) {
		double ident[9];
		double n[9] = { 0.0, -axis[2], axis[1],
			axis[2], 0.0, -axis[0],
			-axis[1], axis[0], 0.0 };

		double nsq[9], sn[9], cnsq[9], tmp[9];

		double c, s;

		c = cos(angle);
		s = sin(angle);

		Identity(3, ident);
		Product3x3(n, n, nsq);
		Scale(3, 3, n, s, sn);
		Scale(3, 3, nsq, (1 - c), cnsq);

		Sum(3, 3, 3, 3, ident, sn, tmp);
		Sum(3, 3, 3, 3, tmp, cnsq, R);
	}

	void AxisAngle2Mat4(double *axis, double angle, double *R) {
		double ident[9];
		double n[9] = { 0.0, -axis[2], axis[1],
			axis[2], 0.0, -axis[0],
			-axis[1], axis[0], 0.0 };

		double nsq[9], sn[9], cnsq[9], tmp[9];
		double R3[9];

		double c, s;

		c = cos(angle);
		s = sin(angle);

		Identity(3, ident);
		Product3x3(n, n, nsq);
		Scale(3, 3, n, s, sn);
		Scale(3, 3, nsq, (1 - c), cnsq);

		Sum(3, 3, 3, 3, ident, sn, tmp);
		Sum(3, 3, 3, 3, tmp, cnsq, R3);

		Identity(4, R);
		memcpy(R, R3, 3 * sizeof(double));
		memcpy(R + 4, R3 + 3, 3 * sizeof(double));
		memcpy(R + 8, R3 + 6, 3 * sizeof(double));
	}

	/* Decompose a square matrix into an orthogonal matrix and a symmetric
	* positive semidefinite matrix */
	void MatPolarDecomposition(int n, double *A, double *Q, double *S) 
	{
		double *U, *diag, *VT;
		double *diag_full, *tmp;
		int i;

		U = (double *) malloc(sizeof(double) * n * n);
		diag = (double *) malloc(sizeof(double) * n);
		VT = (double *) malloc(sizeof(double) * n * n);

		/* Compute SVD */
		SVD(n, n, A, U, diag, VT);

		/* Compute Q */
		Product(n, n, n, n, U, VT, Q);

		/* Compute S */
		diag_full = (double *) malloc(sizeof(double) * n * n);

		for (i = 0; i < n * n; i++) {
			diag_full[i] = 0.0;
		}

		for (i = 0; i < n; i++) {
			diag_full[i * n + i] = diag[i];
		}

		tmp = (double *) malloc(sizeof(double) * n * n);
		ProductAtB(n, n, n, n, VT, diag_full, tmp);
		Product(n, n, n, n, tmp, VT, S);

		free(U);
		free(diag);
		free(VT);
		free(diag_full);
		free(tmp);
	}

	/* Convert a matrix to a normalize quaternion */
	void Mat2Quat(double *R, double *q) {
		double n4; // the norm of quaternion multiplied by 4 
		double tr = R[0] + R[4] + R[8]; // trace of martix
		double factor;

		if (tr > 0.0) {
			q[1] = R[5] - R[7];
			q[2] = R[6] - R[2];
			q[3] = R[1] - R[3];
			q[0] = tr + 1.0;
			n4 = q[0];
		} else if ((R[0] > R[4]) && (R[0] > R[8])) {
			q[1] = 1.0 + R[0] - R[4] - R[8];
			q[2] = R[3] + R[1];
			q[3] = R[6] + R[2];
			q[0] = R[5] - R[7];
			n4 = q[1];
		} else if (R[4] > R[8]) {
			q[1] = R[3] + R[1];
			q[2] = 1.0 + R[4] - R[0] - R[8];
			q[3] = R[7] + R[5];
			q[0] = R[6] - R[2]; 
			n4 = q[2];
		} else {
			q[1] = R[6] + R[2];
			q[2] = R[7] + R[5];
			q[3] = 1.0 + R[8] - R[0] - R[4];
			q[0] = R[1] - R[3];
			n4 = q[3];
		}

		factor = 0.5 / sqrt(n4);
		q[0] *= factor;
		q[1] *= factor;
		q[2] *= factor;
		q[3] *= factor;
	}

	/* Convert a normalized quaternion to a matrix */
	void Quat2Mat(double *q, double *R) {
		double sqw = q[0] * q[0];
		double sqx = q[1] * q[1]; 
		double sqy = q[2] * q[2];
		double sqz = q[3] * q[3];

		double tmp1, tmp2;

		R[0] =  sqx - sqy - sqz + sqw; // since sqw + sqx + sqy + sqz = 1
		R[4] = -sqx + sqy - sqz + sqw;    
		R[8] = -sqx - sqy + sqz + sqw;

		tmp1 = q[1] * q[2];
		tmp2 = q[3] * q[0];

		R[1] = 2.0 * (tmp1 + tmp2);    
		R[3] = 2.0 * (tmp1 - tmp2);        

		tmp1 = q[1] * q[3];    
		tmp2 = q[2] * q[0];    

		R[2] = 2.0 * (tmp1 - tmp2);    
		R[6] = 2.0 * (tmp1 + tmp2);    

		tmp1 = q[2] * q[3];    
		tmp2 = q[1] * q[0];    

		R[5] = 2.0 * (tmp1 + tmp2);   
		R[7] = 2.0 * (tmp1 - tmp2);
	}


	void Slerp(double *v1, double *v2, double t, double *v3)
	{
		double dot, angle, sin1t, sint, sina;
		Product(1, 3, 3, 1, v1, v2, &dot);

		angle = acos(CLAMP(dot, -1.0 + 1.0e-8, 1.0 - 1.0e-8));

		sin1t = sin((1.0-t) * angle);
		sint = sin(t * angle);
		sina = sin(angle);

		v3[0] = (sin1t / sina) * v1[0] + (sint / sina) * v2[0];
		v3[1] = (sin1t / sina) * v1[1] + (sint / sina) * v2[1];
		v3[2] = (sin1t / sina) * v1[2] + (sint / sina) * v2[2];
	}

}//namespace

	///* Solve a system of equations using a precomputed LU decomposition */
	//void SolveLU(int n, double *LU, int *ipiv, double *b, double *x)
	//{
	//	double *btmp = (double*)malloc(sizeof(double) * n);    
	//	char trans = 'N';
	//	int nrhs = 1;
	//	int lda = n, ldb = n;
	//	int i;
	//	int info;

	//	for (i = 0; i < n; i++)
	//		btmp[i] = b[i];

	//	dgetrs_(&trans, (integer*)&n, (integer*)&nrhs, LU,
	//		(integer*)&lda, (integer*)ipiv, btmp,
	//		(integer*)&ldb, (integer*)&info);

	//	if (info != 0) {
	//		char msg[100];
	//		sprintf_s<100>(msg, "dgetrs_ exited with error code %d\n", info);
	//		LogE(msg);
	//	}

	//	memcpy(x, btmp, sizeof(double) * n);

	//	free(btmp);
	//}

	//void Lsy(double *A, double *b, double *x, int m, int n, int nrhs) {
	//	if (m < n) {
	//		LogE("driver now only works when m >= n");
	//		return;
	//	} else {
	//		double *Atmp = (double*)malloc(sizeof(double) * m * n);
	//		double *btmp = (double*)malloc(sizeof(double) * m * nrhs);
	//		int lda = m;
	//		int ldb = m;
	//		int *jpvt = (int*)calloc(sizeof(int), n);
	//		double rcond = -1.0;
	//		int rank; /* Output */
	//		int lwork = -1;
	//		double *work = (double*)malloc(sizeof(double) * 1);
	//		int info;

	//		int i, j;

	//		/* Go from row- to column-major */
	//		for (i = 0; i < m; i++)
	//			for (j = 0; j < n; j++)
	//				Atmp[j * m + i] = A[i * n + j];

	//		for (i = 0; i < m; i++)
	//			for (j = 0; j < nrhs; j++)
	//				btmp[j * m + i] = b[i * nrhs + j];

	//		/* Query to find a good size for the work array */
	//		dgelsy_((integer*)&m, (integer*)&n, (integer*)&nrhs, Atmp, (integer*)&lda, btmp, (integer*)&ldb, (integer*)jpvt,
	//			&rcond, (integer*)&rank, work, (integer*)&lwork, (integer*)&info);

	//		lwork = (int) work[0];
	//		/* printf("Work size: %d\n", lwork); */
	//		free(work);
	//		work = (double*)malloc(sizeof(double) * lwork);

	//		/* Make the FORTRAN call */
	//		dgelsy_((integer*)&m, (integer*)&n, (integer*)&nrhs, Atmp, (integer*)&lda, btmp, (integer*)&ldb, (integer*)jpvt,
	//			&rcond, (integer*)&rank, work, (integer*)&lwork, (integer*)&info);

	//		if (info != 0) {
	//			char msg[100];
	//			sprintf_s<100>(msg, "Error [%d] in call to dgelsy", info);
	//			LogE(msg);
	//		}

	//		/* Go from column- to row-major */
	//		for (i = 0; i < n; i++)
	//			for (j = 0; j < nrhs; j++)
	//				x[i * nrhs + j] = btmp[j * m + i];

	//		free(Atmp);
	//		free(btmp);
	//		free(work);
	//		free(jpvt);
	//	}
	//}

	//void LsyTranspose(double *A, double *b, double *x, 
	//	int m, int n, int nrhs) 
	//{
	//	if (m < n) {
	//		LogE("driver now only works when m >= n");
	//		return;
	//	} else {
	//		double *btmp = (double*)malloc(sizeof(double) * m * nrhs);
	//		int lda = m;
	//		int ldb = m;
	//		int *jpvt = (int*)calloc(sizeof(int), n);
	//		double rcond = -1.0;
	//		int rank; /* Output */
	//		int lwork = -1;
	//		double *work = (double*)malloc(sizeof(double) * 1);
	//		int info;

	//		int i, j;

	//		memcpy(btmp, b, sizeof(double) * m);

	//		/* Query to find a good size for the work array */
	//		dgelsy_((integer*)&m, (integer*)&n, (integer*)&nrhs, A, (integer*)&lda, btmp, (integer*)&ldb, (integer*)jpvt,
	//			&rcond, (integer*)&rank, work, (integer*)&lwork, (integer*)&info);

	//		lwork = (int) work[0];
	//		/* printf("Work size: %d\n", lwork); */
	//		free(work);
	//		work = (double*)malloc(sizeof(double) * lwork);

	//		/* Make the FORTRAN call */
	//		dgelsy_((integer*)&m, (integer*)&n, (integer*)&nrhs, A, (integer*)&lda, btmp, (integer*)&ldb, (integer*)jpvt,
	//			&rcond, (integer*)&rank, work, (integer*)&lwork, (integer*)&info);

	//		if (info != 0) {
	//			char msg[100];
	//			sprintf_s<100>(msg, "Error [%d] in call to dgelsy", info);
	//			LogE(msg);
	//		}

	//		/* Go from column- to row-major */
	//		for (i = 0; i < n; i++)
	//			for (j = 0; j < nrhs; j++)
	//				x[i * nrhs + j] = btmp[j * m + i];

	//		free(btmp);
	//		free(work);
	//		free(jpvt);
	//	}
	//}
