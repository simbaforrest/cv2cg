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

/* IOHelper.h
   helpers related to input/output  */

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <iterator>
#include <string>
#include <vector>
#include <set>
#include <stdio.h>

namespace IOHelper
{
//read one valid line, i.e. non-empty line
//return false if no valid line, i.e. end-of-file
inline bool readValidLine(std::istream &in,
                          std::string &line, char commentchar='#')
{
	line = "";//clear;
	while(in) {
		std::string str;
		std::getline(in, str);
		if(str.length()==0 || str[0]==commentchar) {
			continue;
		}

		//take care of CR/LF, see details:
		// http://www.codeguru.com/forum/showthread.php?t=253826
		char lastchar = str[str.length()-1];
		if(lastchar=='\n' || lastchar=='\r') {
			str.resize(str.length()-1);
		}
		if(str.length()==0) {
			continue;
		}

		line = str;
		break;
	}
	return line.length()!=0;
}

//read calibration matrix from stream
//format: (only numbers and white characters, no other stuff)
//K[0] K[1] K[2]
//K[3] K[4] K[5]
//K[6] K[7] K[8]
template<typename Precision>
bool readCalibFile(std::istream &in, Precision K[9])
{
	for(int i=0; i<9; ++i) {
		double val;
		in >> val;
		K[i] = (Precision) val;
	}
	return true;
}

/* print the given n x m matrix */
template<typename Precision>
void print(int m, int n, Precision *A, const char *mat_name=0)
{
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
bool ReadFile(int m, int n, Precision const *matrix, const char *fname)
{
	FILE *f = NULL;
	f = fopen(fname, "r");
	int i;

	if (f == NULL) {
		printf("[ReadFile error] In reading matrix %s\n", fname);
		return false;
	}

	for (i = 0; i < m * n; i++) {
		fscanf(f, "%lf", matrix + i);
	}

	fclose(f);
	return true;
}

/* Write a matrix to a file */
template<typename Precision>
bool WriteFile(int m, int n, Precision const *matrix, const char *fname)
{
	FILE *f = NULL;
	f = fopen(fname, "w");
	int i, j, idx;

	if (f == NULL) {
		printf("[WriteFile error] In writing matrix to %s\n", fname);
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

//Utils for using c++ stream to print matrix
//e.g.
//	cout<<"R=\n"<<PrintMat<>(3,3,R)<<endl;
template<unsigned int iosflag=std::ios::scientific,
         typename Precision=double>
struct PrintMat {
	int cols,rows;
	Precision const *p;
	PrintMat(int r, int c, Precision const *ptr) {
		cols=c;
		rows=r;
		p=ptr;
	}
#define IDX(m,n,N) ((m)*(N)+(n)) //N cols
	friend inline std::ostream &operator<<(
	    std::ostream &o, const PrintMat &m) {
		o.setf((std::ios_base::fmtflags)iosflag);
		const std::streamsize ps=o.precision();
		o.precision(16);
		for(int i=0; i<m.rows; ++i) {
			for(int j=0; j<m.cols; ++j) {
				o << std::setw(30) << m.p[IDX(i,j,m.cols)] << " ";
			}
			o << std::endl;
		}
		o.precision(ps);
		o.unsetf((std::ios_base::fmtflags)iosflag);
		return o;
	}
#undef IDX
};

template<typename T>
void print(std::ostream &o, const std::vector<T> &array, const char *sep)
{
	std::copy(array.begin(), array.end(), std::ostream_iterator<T>(o, sep));
}

template<typename T>
void print(std::ostream &o, const std::set<T> &s, const char *sep)
{
	std::copy(s.begin(), s.end(), std::ostream_iterator<T>(o, sep));
}

}//end of IOHelper
