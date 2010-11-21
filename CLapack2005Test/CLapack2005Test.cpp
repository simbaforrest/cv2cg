#include <stdafx.h>
#include <stdio.h>
//#include <process.h>
#include <iostream>
#include <string>
#include <signal.h>
#include "MatrixManip.h"
#include "CameraAlgebra.h"

double ans[1000]={0};
int ians[1000]={-1};

double A[] = {
	1,2,3,
	4,5,6
};
double I[3][3] = {0};
double At[6] = {0};
double B[] = {
	1,2,3,
	4,5,6,
	7,8,9
};
double C[]={
	1,2,
	3,4
};
double x[]={1,2,3};
double D[]={
	7,8,9,
	4,5,6,
	1,2,3
};
double T[]={
    7,    7,    0,    0,
    7,    7,    3,    0,
    0,    4,    8,    2,
    0,    0,    6,   10
};
double L[9], U[9], P[9];
double b[]={14,32};

	/*****************Basic Operation*******************/

	/* Fill a given matrix with an n x n identity matrix */
	void test_Identity() {std::cout<<"@"<<__FUNCTION__<<std::endl;
		MatrixManip::Print(3,3,I[0], "I before");
		MatrixManip::Identity(3,I[0]);
		MatrixManip::Print(3,3,I[0], "I after");
	std::cout<<std::endl;}
	/* Fill a given matrix with an m x n matrix of zeroes */
	void test_Zeros() {std::cout<<"@"<<__FUNCTION__<<std::endl;
		MatrixManip::Zeros(3,3,I[0]);
		MatrixManip::Print(3,3,I[0], "Zeros(I)");
		MatrixManip::Identity(3,I[0]);
	std::cout<<std::endl;}
	/* Transpose the m x n matrix A and put the result in the n x m matrix AT */
	void test_Transpose() {std::cout<<"@"<<__FUNCTION__<<std::endl;
		MatrixManip::Transpose(2,3,A,At);
		MatrixManip::Print(2,3,A, "A");
		MatrixManip::Print(3,2,At, "At");
	std::cout<<std::endl;}
	void test_TransposeInplace() {std::cout<<"@"<<__FUNCTION__<<std::endl;
		MatrixManip::SwapRow(2,3,A,0,1);
		MatrixManip::Print(2,3, A, "SwapRow(A)");
		MatrixManip::SwapRow(2,3,A,0,1);
	std::cout<<std::endl;}
	/* Compute the matrix product R = AB */
	void test_Product() {std::cout<<"@"<<__FUNCTION__<<std::endl;
		MatrixManip::Product(3,2,2,3,At,A,ans);
		MatrixManip::Print(3,3,ans, "At*A");
	std::cout<<std::endl;}
	/* Compute the power of a matrix */
	void test_Power() {std::cout<<"@"<<__FUNCTION__<<std::endl;
		MatrixManip::Power(2,C,2,ans);
		MatrixManip::Print(2,2,C,"C");
		MatrixManip::Print(2,2,ans,"C^2");
	std::cout<<std::endl;}
	/* Compute the matrix product R = A B^T */
	void test_ProductABt() {std::cout<<"@"<<__FUNCTION__<<std::endl;
		MatrixManip::ProductABt(2,3,3,3,A,B,ans);
		MatrixManip::Print(3,3,B, "B");
		MatrixManip::Print(2,3,ans, "A*B'");
	std::cout<<std::endl;}
	/* Compute the matrix sum R = A + B */
	void test_Sum() {std::cout<<"@"<<__FUNCTION__<<std::endl;
		MatrixManip::Print(3,3,I[0], "I");
		MatrixManip::Print(3,3,B, "B");
		MatrixManip::Sum(3,3,3,3,I[0],B,ans);
		MatrixManip::Print(3,3,ans, "I+B");
	std::cout<<std::endl;}
	/* Compute the matrix difference R = A - B */
	void test_Diff() {std::cout<<"@"<<__FUNCTION__<<std::endl;
		MatrixManip::Diff(3,3,3,3,ans,I[0],ans);
		MatrixManip::Print(3,3,ans, "(I+B)-I");
	std::cout<<std::endl;}
	/* Compute the determinant of a 3x3 matrix */
	void test_Det3() {std::cout<<"@"<<__FUNCTION__<<std::endl;
		MatrixManip::Print(3,3,I[0]);
		std::cout<<MatrixManip::Det3(I[0])<<std::endl;
	std::cout<<std::endl;}
	/* Compute the matrix product R = A^T B */
	void test_ProductAtB() {std::cout<<"@"<<__FUNCTION__<<std::endl;
		MatrixManip::ProductAtB(2,3,2,2,A,C,ans);
		MatrixManip::Print(2,2,C, "C");
		MatrixManip::Print(3,2,ans, "A'*C");
	std::cout<<std::endl;}
	/* Return the product x**T A x */
	void test_ProductxtAx() {std::cout<<"@"<<__FUNCTION__<<std::endl;
		MatrixManip::Print(1,3,x, "x");
		std::cout<<"x'*B*x =\n"<<MatrixManip::ProductxtAx(3,B,x)<<std::endl;
	std::cout<<std::endl;}
	/* Scale a matrix by a scalar */
	void test_Scale() {std::cout<<"@"<<__FUNCTION__<<std::endl;
		MatrixManip::Scale(3,3,I[0],24,ans);
		MatrixManip::Print(3,3,ans, "I*24");
	std::cout<<std::endl;}
	/* Get the norm of the matrix */
	void test_Norm() {std::cout<<"@"<<__FUNCTION__<<std::endl;std::cout<<std::endl;}
	/* Get the [squared] norm of the matrix */
	void test_NormSq() {std::cout<<"@"<<__FUNCTION__<<std::endl;std::cout<<std::endl;}
	void test_Cross4() {std::cout<<"@"<<__FUNCTION__<<std::endl;
	std::cout<<std::endl;}

	/*****************Advanced Operation*******************/
	
	/* Compute (transpose of) LU decomposition of A */
	void test_LU() {std::cout<<"@"<<__FUNCTION__<<std::endl;
		MatrixManip::Print(3,3, B, "B");
		MatrixManip::LU(3,B, L, U, P);
		MatrixManip::Print(3,3,L, "L");
		MatrixManip::Print(3,3,U, "U");
		MatrixManip::Print(3,3,P, "P");
		MatrixManip::Product(3,3,3,3,L,U,ans);
		MatrixManip::Product(3,3,3,3,P,B,ans+100);
		MatrixManip::Print(3,3,ans, "L*U");
		MatrixManip::Print(3,3,ans+100, "P*B");
		MatrixManip::Log::i("test");
	std::cout<<std::endl;}
	void test_LUTranspose() {std::cout<<"@"<<__FUNCTION__<<std::endl;std::cout<<std::endl;}
	/* Solve a system of equations using a precomputed LU decomposition */
	void test_SolveLU() {std::cout<<"@"<<__FUNCTION__<<std::endl;std::cout<<std::endl;}
	/* Invert the n-by-n matrix A, storing the result in Ainv */
	void test_Inv() {std::cout<<"@"<<__FUNCTION__<<std::endl;
		MatrixManip::Print(4,4,T,"T");
		MatrixManip::Inv(4,T,ans);
		MatrixManip::Print(4,4,ans,"inv(T)");
	std::cout<<std::endl;}
	void test_InvInplace() {std::cout<<"@"<<__FUNCTION__<<std::endl;
		memcpy(ans, T, sizeof(double)*16);
		MatrixManip::InvInplace(4,ans);
		MatrixManip::Print(4,4,ans,"inv(T) inplace");
	std::cout<<std::endl;}
	/* Driver for the minpack function lmdif, which uses
	* Levenberg-Marquardt for non-linear least squares minimization */
	void test_Lmdif() {std::cout<<"@"<<__FUNCTION__<<std::endl;std::cout<<std::endl;}
	void test_Lmdif2() {std::cout<<"@"<<__FUNCTION__<<std::endl;std::cout<<std::endl;}
	void test_Lmdif3() {std::cout<<"@"<<__FUNCTION__<<std::endl;std::cout<<std::endl;}
	/* Driver for the lapack function dgelss, which finds x to minimize
	* norm(b - A * x) */
	void test_Lss() {std::cout<<"@"<<__FUNCTION__<<std::endl;
	MatrixManip::Print(4,4,T,"T");
	double t[] = {21.01,30.02,39.98,58.03};
	MatrixManip::Lss(T,t,ans,4,4,1);
	MatrixManip::Print(4,1,ans,"x");
	std::cout<<std::endl;}
	void test_Lsy() {std::cout<<"@"<<__FUNCTION__<<std::endl;std::cout<<std::endl;}
	/* Version of above where matrix is already in column-major order */
	void test_LsyTranspose() {std::cout<<"@"<<__FUNCTION__<<std::endl;std::cout<<std::endl;}
	/* Solve an n x n system */
	void test_Solve() {std::cout<<"@"<<__FUNCTION__<<std::endl;
	MatrixManip::Print(2,2,C,"C");
	MatrixManip::Print(2,1,b,"b");
	MatrixManip::Solve(2, C, b, ans);
	MatrixManip::Print(2,1,ans,"y");
	std::cout<<std::endl;}
	/* n: the order of matrix A
	* A: matrix for which the eigenvectors/values are to be computed
	* evec: output array containing the eigenvectors
	* eval: output array containing the eigenvalues
	* Note: Assumes the results are real! */
	void test_Eigen() {std::cout<<"@"<<__FUNCTION__<<std::endl;
	MatrixManip::Eigen(3,B,ans, ans+100);
	MatrixManip::Print(3,3,ans,"evec");
	MatrixManip::Print(3,1,ans+100, "eval");
	std::cout<<std::endl;}
	/* Compute singular value decomposition of an m x n matrix A */
	void test_SVD() {std::cout<<"@"<<__FUNCTION__<<std::endl;
		MatrixManip::Zeros(1,1000,ans);
		MatrixManip::SVD(2,3,A, ans, ans+100, ans+200);
		MatrixManip::Print(2,2,ans,"U");
		MatrixManip::Print(2,3,ans+100,"S");
		MatrixManip::Print(3,3,ans+200,"VT");
		MatrixManip::Product(2,2,2,3,ans,ans+100, ans+300);
		MatrixManip::Product(2,3,3,3,ans+300,ans+200,ans+400);
		MatrixManip::Print(2,3,ans+400,"U*S*VT");
	std::cout<<std::endl;}
	/* Compute singular value decomposition of an m x n matrix A 
	* (only compute S and VT) */
	void test_SVDvt() {std::cout<<"@"<<__FUNCTION__<<std::endl;std::cout<<std::endl;}
	/* Compute Cholesky decomposition of an nxn matrix */
	void test_Cholesky() {std::cout<<"@"<<__FUNCTION__<<std::endl;
		MatrixManip::Pascal(5,ans);
		MatrixManip::Print(5,5,ans,"Pascal(5)");
		MatrixManip::Cholesky(5,ans,ans+100);
		MatrixManip::Print(5,5,ans+100,"Cholesky(Pascal(5))");
	std::cout<<std::endl;}
	/* Compute a QR factorization of an m by n matrix A */
	void test_QR() {std::cout<<"@"<<__FUNCTION__<<std::endl;
		MatrixManip::Print(3,3,B,"B");
		MatrixManip::QR(3,3,B,ans,ans+100);
		MatrixManip::Print(3,3,ans,"Q");
		MatrixManip::Print(3,3,ans+100,"R");
		MatrixManip::Product(3,3,3,3,ans,ans+100, ans+200);
		MatrixManip::Print(3,3,ans+200,"Q*R");
	std::cout<<std::endl;}
	/* Compute an RQ factorization of an m by n matrix A */
	void test_RQ() {std::cout<<"@"<<__FUNCTION__<<std::endl;
		MatrixManip::Print(3,3,B,"B");
		MatrixManip::RQ(3,3,B,ans,ans+100);
		MatrixManip::Print(3,3,ans,"R");
		MatrixManip::Print(3,3,ans+100,"Q");
		MatrixManip::Product(3,3,3,3,ans,ans+100, ans+200);
		MatrixManip::Print(3,3,ans+200,"R*Q");
	std::cout<<std::endl;}
	/* Find the unit vector that minimizes ||Ax|| */
	void test_matrix_minimum_unit_norm_solution() {std::cout<<"@"<<__FUNCTION__<<std::endl;std::cout<<std::endl;}

	/*****************Convert*******************/

	/* Convert a rotation matrix to axis and angle representation */
	void test_Mat2AxisAngle() {std::cout<<"@"<<__FUNCTION__<<std::endl;std::cout<<std::endl;}
	void test_AxisAngle2Mat() {std::cout<<"@"<<__FUNCTION__<<std::endl;std::cout<<std::endl;}
	void test_AxisAngle2Mat4() {std::cout<<"@"<<__FUNCTION__<<std::endl;std::cout<<std::endl;}
	/* Convert a matrix to a normalize quaternion */
	void test_Mat2Quat() {std::cout<<"@"<<__FUNCTION__<<std::endl;std::cout<<std::endl;}
	/* Convert a normalized quaternion to a matrix */
	void test_Quat2Mat() {std::cout<<"@"<<__FUNCTION__<<std::endl;std::cout<<std::endl;}
	/* Decompose a square matrix into an orthogonal matrix and a symmetric
	* positive semidefinite matrix */
	void test_MatPolarDecomposition() {std::cout<<"@"<<__FUNCTION__<<std::endl;std::cout<<std::endl;}
	void test_Slerp() {std::cout<<"@"<<__FUNCTION__<<std::endl;std::cout<<std::endl;}

	/*********************I/O***********************/

	/* Print the given m x n matrix */
	void test_Print() {std::cout<<"@"<<__FUNCTION__<<std::endl;
	double P[3][4]={
		3.53553e2,3.39645e2,2.77744e2,-1.44946e6,
		-1.03528e2,2.33212e1,4.59607e2,-6.32525e5,
		7.07107e-1,-3.53553e-1,6.12372e-1,-9.18559e2
	};
		double K[3][3],R[3][3],C[3],T[3];
		CameraAlgebra::Decompose(P,K,R,C,T);
		MatrixManip::Print(3,3,K[0], "K");
		MatrixManip::Print(3,3,R[0], "R");
		MatrixManip::Print(3,1,C, "C");
		MatrixManip::Print(3,1,T, "T");
	std::cout<<std::endl;}
	/* Read a matrix from a file */
	void test_ReadFile() {std::cout<<"@"<<__FUNCTION__<<std::endl;
		double rA[6] = {0};
		MatrixManip::Print(2,3,rA);
		MatrixManip::ReadFile(2,3,rA, "test.matrix");
		MatrixManip::Print(2,3,rA);
	std::cout<<std::endl;}
	/* Write a matrix to a file */
	void test_WriteFile() {std::cout<<"@"<<__FUNCTION__<<std::endl;
		MatrixManip::WriteFile(2,3,A, "test.matrix");
		double p[][2]={
1907, 80,
1617, 805,
1544, 676,
2293, 797,
2366, 648,
1708, 881,
2224, 899,
2553, 1019,
1371, 1165,
2268, 1399
		};
		double X[][3]={
-28.2032, -8.89819, 29.2764, 
-23.7429, -1.70781, 27.6314,
-26.7666, -1.63711, 34.1186, 
-30.4854, -1.79169, 24.2703, 
-33.6088, -1.69792, 30.911, 
-27.5322, 0.109337, 32.3601, 
-31.8462, 0.265603, 30.288, 
-35.3244, 1.88656, 31.5824, 
-26.1799, 2.97655, 36.0448, 
-33.0257, 5.04701,32.4985
		};
		double K[3][3],R[3][3],C[3],T[3],P[3][4];
		if( CameraAlgebra::DLT(10, p[0], X[0], P[0]) ) {
		MatrixManip::Print(3,4,P[0],"P");
		CameraAlgebra::Decompose(P,K,R,C,T);
		MatrixManip::Print(3,3,K[0], "K");
		MatrixManip::Print(3,3,R[0], "R");
		MatrixManip::Print(3,1,C, "C");
		MatrixManip::Print(3,1,T, "T");
		double meanErr,maxErr,sumsqErr,sd;
		CameraAlgebra::ReprojectionError(10,p[0],X[0],P[0],meanErr,maxErr,sumsqErr,sd);
		}

		MatrixManip::Print(std::cout<<"KKK=\n",K[0],3,3)<<std::endl;

	std::cout<<std::endl;}

	void Main_Test()
	{
		//MatrixManip::Log::logfile = fopen("MatrixManip.log","w");
		/*****************Basic Operation*******************/
		test_Identity();
		/* Fill a given matrix with an m x n matrix of zeroes */
		test_Zeros();
		/* Transpose the m x n matrix A and put the result in the n x m matrix AT */
		test_Transpose();
		test_TransposeInplace();
		/* Compute the matrix product R = AB */
		test_Product();
		/* Compute the power of a matrix */
		test_Power();
		/* Compute the matrix product R = A B^T */
		test_ProductABt();
		/* Compute the matrix sum R = A + B */
		test_Sum();
		/* Compute the matrix difference R = A - B */
		test_Diff();
		/* Compute the determinant of a 3x3 matrix */
		test_Det3();
		/* Compute the matrix product R = A^T B */
		test_ProductAtB();
		/* Return the product x**T A x */
		test_ProductxtAx();
		/* Scale a matrix by a scalar */
		test_Scale();
		/* Get the norm of the matrix */
		test_Norm();
		/* Get the [squared] norm of the matrix */
		test_NormSq();
		test_Cross4();

		/*****************Advanced Operation*******************/

		/* Compute (transpose of) LU decomposition of A */
		test_LU();
		test_LUTranspose();
		/* Solve a system of equations using a precomputed LU decomposition */
		test_SolveLU();
		/* Invert the n-by-n matrix A, storing the result in Ainv */
		test_Inv();
		test_InvInplace();
		/* Driver for the minpack function lmdif, which uses
		* Levenberg-Marquardt for non-linear least squares minimization */
		test_Lmdif();
		test_Lmdif2();
		test_Lmdif3();
		/* Driver for the lapack function dgelss, which finds x to minimize
		* norm(b - A * x) */
		test_Lss();
		test_Lsy();
		/* Version of above where matrix is already in column-major order */
		test_LsyTranspose();
		/* Solve an n x n system */
		test_Solve();
		/* n: the order of matrix A
		* A: matrix for which the eigenvectors/values are to be computed
		* evec: output array containing the eigenvectors
		* eval: output array containing the eigenvalues
		* Note: Assumes the results are real! */
		test_Eigen();
		/* Compute singular value decomposition of an m x n matrix A */
		test_SVD();
		/* Compute singular value decomposition of an m x n matrix A 
		* (only compute S and VT) */
		test_SVDvt();
		/* Compute Cholesky decomposition of an nxn matrix */
		test_Cholesky();
		/* Compute a QR factorization of an m by n matrix A */
		test_QR();
		/* Compute an RQ factorization of an m by n matrix A */
		test_RQ();
		/* Find the unit vector that minimizes ||Ax|| */
		test_matrix_minimum_unit_norm_solution();

		/*****************Convert*******************/

		/* Convert a rotation matrix to axis and angle representation */
		test_Mat2AxisAngle();
		test_AxisAngle2Mat();
		test_AxisAngle2Mat4();
		/* Convert a matrix to a normalize quaternion */
		test_Mat2Quat();
		/* Convert a normalized quaternion to a matrix */
		test_Quat2Mat();
		/* Decompose a square matrix into an orthogonal matrix and a symmetric
		* positive semidefinite matrix */
		test_MatPolarDecomposition();
		test_Slerp();
		/*********************I/O***********************/
		test_Print();
		test_WriteFile();
		test_ReadFile();
		//fclose(MatrixManip::Log::logfile);
		//MatrixManip::Log::logfile = stdout;
	}

int main()
{
	Main_Test();
	system("pause");
	return 0;
}