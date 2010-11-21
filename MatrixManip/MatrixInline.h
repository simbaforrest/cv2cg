#ifndef __MATRIX_INLINE_HEADER__
#define __MATRIX_INLINE_HEADER__
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

/* MatrixInline.h */

#include <iostream>
#include <iomanip>
#include "MatrixManipLog.h"
using namespace std;

namespace MatrixManip {
#define uint unsigned int
#define IDX(m,n,N) ((m)*(N)+(n)) //N cols

	// convert a pointer to a continuous memory block
	// to a 2D array with "cols" columns in Precision type
	template<int cols, typename Precision=double*>
	struct FixMat {
		typedef  Precision (*Type)[cols];

		template<typename SrcType>
		static Type ConvertType(SrcType src)
		{
			return reinterpret_cast<Type>(src);
		}
	};

	//Determinant of small size matrices
	inline double Det2x2(double const *m) {
		return m[0*2+0] * m[1*2+1] - m[0*2+1]*m[1*2+0];
	}

	inline double Det3x3(double const *m) {
		return 
		+ m[0*3+0]*m[1*3+1]*m[2*3+2]
		- m[0*3+0]*m[2*3+1]*m[1*3+2]
		- m[1*3+0]*m[0*3+1]*m[2*3+2]
		+ m[1*3+0]*m[2*3+1]*m[0*3+2]
		+ m[2*3+0]*m[0*3+1]*m[1*3+2]
		- m[2*3+0]*m[1*3+1]*m[0*3+2];
	}

	inline double Det4x4(double const *m) {
		return
		+ m[0*4+0]*m[1*4+1]*m[2*4+2]*m[3*4+3]
		- m[0*4+0]*m[1*4+1]*m[3*4+2]*m[2*4+3]
		- m[0*4+0]*m[2*4+1]*m[1*4+2]*m[3*4+3]
		+ m[0*4+0]*m[2*4+1]*m[3*4+2]*m[1*4+3]
		+ m[0*4+0]*m[3*4+1]*m[1*4+2]*m[2*4+3]
		- m[0*4+0]*m[3*4+1]*m[2*4+2]*m[1*4+3]
		- m[1*4+0]*m[0*4+1]*m[2*4+2]*m[3*4+3]
		+ m[1*4+0]*m[0*4+1]*m[3*4+2]*m[2*4+3]
		+ m[1*4+0]*m[2*4+1]*m[0*4+2]*m[3*4+3]
		- m[1*4+0]*m[2*4+1]*m[3*4+2]*m[0*4+3]
		- m[1*4+0]*m[3*4+1]*m[0*4+2]*m[2*4+3]
		+ m[1*4+0]*m[3*4+1]*m[2*4+2]*m[0*4+3]
		+ m[2*4+0]*m[0*4+1]*m[1*4+2]*m[3*4+3]
		- m[2*4+0]*m[0*4+1]*m[3*4+2]*m[1*4+3]
		- m[2*4+0]*m[1*4+1]*m[0*4+2]*m[3*4+3]
		+ m[2*4+0]*m[1*4+1]*m[3*4+2]*m[0*4+3]
		+ m[2*4+0]*m[3*4+1]*m[0*4+2]*m[1*4+3]
		- m[2*4+0]*m[3*4+1]*m[1*4+2]*m[0*4+3]
		- m[3*4+0]*m[0*4+1]*m[1*4+2]*m[2*4+3]
		+ m[3*4+0]*m[0*4+1]*m[2*4+2]*m[1*4+3]
		+ m[3*4+0]*m[1*4+1]*m[0*4+2]*m[2*4+3]
		- m[3*4+0]*m[1*4+1]*m[2*4+2]*m[0*4+3]
		- m[3*4+0]*m[2*4+1]*m[0*4+2]*m[1*4+3]
		+ m[3*4+0]*m[2*4+1]*m[1*4+2]*m[0*4+3];
	}

	inline void Product2x2(double const *A, double const *B, double *R)
	{
		R[0] = A[0]*B[0] + A[1]*B[2];
		R[1] = A[0]*B[1] + A[1]*B[3];
		R[2] = A[2]*B[0] + A[3]*B[2];
		R[3] = A[2]*B[1] + A[2]*B[3];
	}

	inline void Product3x3(double const *A, double const *B, double *R)
	{
		R[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
		R[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
		R[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];

		R[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
		R[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
		R[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];

		R[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
		R[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
		R[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];
	}

	inline void Product4x4(double const *A, double const *B, double *R)
	{
		R[0] = A[0] * B[0] + A[1] * B[4] + A[2] * B[8] + A[3] * B[12];
		R[1] = A[0] * B[1] + A[1] * B[5] + A[2] * B[9] + A[3] * B[13];
		R[2] = A[0] * B[2] + A[1] * B[6] + A[2] * B[10] + A[3] * B[14];
		R[3] = A[0] * B[3] + A[1] * B[7] + A[2] * B[11] + A[3] * B[15];

		R[4] = A[4] * B[0] + A[5] * B[4] + A[6] * B[8] + A[7] * B[12];
		R[5] = A[4] * B[1] + A[5] * B[5] + A[6] * B[9] + A[7] * B[13];
		R[6] = A[4] * B[2] + A[5] * B[6] + A[6] * B[10] + A[7] * B[14];
		R[7] = A[4] * B[3] + A[5] * B[7] + A[6] * B[11] + A[7] * B[15];

		R[8] = A[8] * B[0] + A[9] * B[4] + A[10] * B[8] + A[11] * B[12];
		R[9] = A[8] * B[1] + A[9] * B[5] + A[10] * B[9] + A[11] * B[13];
		R[10] = A[8] * B[2] + A[9] * B[6] + A[10] * B[10] + A[11] * B[14];
		R[11] = A[8] * B[3] + A[9] * B[7] + A[10] * B[11] + A[11] * B[15];

		R[12] = A[12] * B[0] + A[13] * B[4] + A[14] * B[8] + A[15] * B[12];
		R[13] = A[12] * B[1] + A[13] * B[5] + A[14] * B[9] + A[15] * B[13];
		R[14] = A[12] * B[2] + A[13] * B[6] + A[14] * B[10] + A[15] * B[14];
		R[15] = A[12] * B[3] + A[13] * B[7] + A[14] * B[11] + A[15] * B[15];
	}

	inline void Product121(double const *A, double const *b, double *r)
	{
		r[0] = A[0] * b[0] + A[1] * b[1];
	}

	inline void Product131(double const *A, double const *b, double *r)
	{
		r[0] = A[0] * b[0] + A[1] * b[1] + A[2] * b[2];
	}

	inline void Product331(double const *A, double const *b, double *r)
	{
		r[0] = A[0] * b[0] + A[1] * b[1] + A[2] * b[2];
		r[1] = A[3] * b[0] + A[4] * b[1] + A[5] * b[2];
		r[2] = A[6] * b[0] + A[7] * b[1] + A[8] * b[2];
	}

	inline void Product341(double const *A, double const *b, double *r)
	{
		r[0] = A[0] * b[0] + A[1] * b[1] + A[2] * b[2] + A[3] * b[3];
		r[1] = A[4] * b[0] + A[5] * b[1] + A[6] * b[2] + A[7] * b[3];
		r[2] = A[8] * b[0] + A[9] * b[1] + A[10] * b[2] + A[11] * b[3];
	}

	inline void Product441(double const *A, double const *b, double *r)
	{
		r[0] = A[0] * b[0] + A[1] * b[1] + A[2] * b[2] + A[3] * b[3];
		r[1] = A[4] * b[0] + A[5] * b[1] + A[6] * b[2] + A[7] * b[3];
		r[2] = A[8] * b[0] + A[9] * b[1] + A[10] * b[2] + A[11] * b[3];
		r[3] = A[12] * b[0] + A[13] * b[1] + A[14] * b[2] + A[15] * b[3];
	}

	// Cross two 3x1 vectors
	inline void Cross(double const *u, double const *v, double *w) {
		w[0] = u[1] * v[2] - u[2] * v[1];
		w[1] = u[2] * v[0] - u[0] * v[2];
		w[2] = u[0] * v[1] - u[1] * v[0];
	}

	// Create the 3x3 cross product matrix from a 3-vector
	inline void CrossMatrix(double const *v, double *v_cross) {
		v_cross[0] = 0.0;   v_cross[1] = -v[2]; v_cross[2] = v[1];
		v_cross[3] = v[2];  v_cross[4] = 0.0;   v_cross[5] = -v[0];
		v_cross[6] = -v[1]; v_cross[7] = v[0];  v_cross[8] = 0.0;
	}

	template<typename Precision>
	void Swap(Precision& a, Precision& b)
	{
		Precision tmp = a;
		a = b;
		b = tmp;		
	}

	// swap row i and row j of a matrix A m*n
	template<typename Precision>
	void SwapRow(int m, int n, Precision* A, int i, int j)
	{
		for(int k=0; k<n; ++k) {
			Swap(A[IDX(i,k,n)], A[IDX(j,k,n)]);
		}
	}

	// swap col i and col j of a matrix A m*n
	template<typename Precision>
	void SwapCol(int m, int n, Precision* A, int i, int j)
	{
		for(int k=0; k<m; ++k) {
			Swap(A[IDX(k,i,n)], A[IDX(k,j,n)]);
		}
	}

	/*****************Special Operation*******************/
	// General a pascal matrix, see matlab function pascal
	inline void Pascal(int n, double *A)
	{
		for(int i=0; i<n; ++i) {
			A[IDX(i,0,n)]=A[IDX(0,i,n)]=1;
		}
		for(int i=1; i<n; ++i)
			for(int j=i; j<n; ++j)
				A[IDX(i,j,n)] = A[IDX(j,i,n)] =
					A[IDX(i-1,j,n)]+A[IDX(i,j-1,n)];
	}

	/*********************I/O***********************/
	/* Print the given n x m matrix */
	template<typename Precision>
	void Print(int m, int n, Precision *A, char* mat_name=0) {
		int i, j;

		if(mat_name) {
			printf("%s = \n\n", mat_name);
		}

		for (i = 0; i < m; i++) {
			printf("  ");
			for (j = 0; j < n; j++) {
				printf(" % 10f", (float)A[i * n + j]);
			}
			printf("\n");
		}
		printf("\n");
	}

	/* Read a matrix from a file */
	template<typename Precision>
	bool ReadFile(int m, int n, Precision *matrix, char *fname) {
		FILE *f = NULL;
		fopen_s(&f, fname, "r");
		int i;

		if (f == NULL) {
			char msg[200];
			sprintf_s<200>(msg,"In reading matrix %s\n", fname);
			Log::e(msg, "MatrixManip::ReadFile");
			return false;
		}

		for (i = 0; i < m * n; i++) {
			fscanf_s(f, "%lf", matrix + i);
		}

		fclose(f);
		return true;
	}

	/* Write a matrix to a file */
	template<typename Precision>
	bool WriteFile(int m, int n, Precision *matrix, char *fname) {
		FILE *f = NULL;
		fopen_s(&f, fname, "w");
		int i, j, idx;

		if (f == NULL) {
			char msg[200];
			sprintf_s<200>(msg,"In writing matrix to %s\n", fname);
			Log::e(msg, "MatrixManip::WriteFile");
			return false;
		}

		idx = 0;
		for (i = 0; i < m; i++) {
			for (j = 0; j < n; j++, idx++) {
				fprintf(f, "%0.16e ", matrix[idx]);
			}
			fprintf(f, "\n");
		}

		fclose(f);
		return true;
	}

	template<typename Precision>
	std::ostream& Print(std::ostream& o, Precision* p, int m, int n) {
		o.setf(std::ios::scientific);
		for(int i=0; i<m; ++i) {
			for(int j=0; j<n; ++j) {
				o << p[IDX(i,j,n)] << " ";
			}
			o << std::endl;
		}
		return o;
	}

}//namespace

// Method to convert 1D array into 2D array:
// double M[2*3]={1,2,3, 4,5,6};
// double M2D[][3] = reinterpret_cast<double (*)[3]>(M)
// for(int i=0; i<2; ++i) { for(int j=0; j<3; ++j)
//    std::cout<<" "<<M2D[i][j]; std::cout<<std::endl;}
#endif//__MATRIX_INLINE_HEADER__
