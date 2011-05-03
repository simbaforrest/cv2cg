#pragma once

#include <iostream>
#include <iomanip>
using namespace std;

typedef double (*VecMat)[1];
typedef const double (*ConstVecMat)[1];

inline VecMat Vector2Matrix(double M[])
{
	VecMat ret = reinterpret_cast<double ( *)[1]>(M);
	return ret;
}

inline ConstVecMat Vector2Matrix(const double M[])
{
	ConstVecMat ret = reinterpret_cast<const double ( *)[1]>(M);
	return ret;
}

template<unsigned int a, unsigned int b>
std::ostream &Matrix_Display(std::ostream &o, const double M[a][b])
{
	o << setiosflags(ios::scientific);

	for(unsigned int i=0; i<a; ++i) {
		for(unsigned int j=0; j<b; ++j) {
			o << M[i][j] << " " ;
		}
		o << std::endl;
	}
	return o;
}

template<unsigned int a, unsigned int b, unsigned int c>
void Matrix_Times(const double A[a][b], const double B[b][c], double C[a][c])
{
	for(unsigned int i=0; i<a; ++i)
		for(unsigned int j=0; j<c; ++j) {
			C[i][j]=0;
			for(unsigned int k=0; k<b; ++k) {
				C[i][j] += A[i][k] * B[k][j];
			}
		}
}

template<unsigned int a, unsigned int b>
void Matrix_Transpose(const double A[a][b], double At[b][a])
{
	for(unsigned int i=0; i<a; ++i)
		for(unsigned int j=0; j<b; ++j) {
			At[j][i] = A[i][j];
		}
}
