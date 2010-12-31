#pragma once
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

/* SparseRec2View.h */

//standard include
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "OpenCVHelper.h"

struct Pt3 {
	Pt3(double _x=0, double _y=0, double _z=0) {
		x=_x; y=_y; z=_z;
	}
	double x,y,z;
};

using namespace std;

class SparseRec2View {
private:
	string imgpath1, imgpath2;
	string dir;
	string imgname1, imgname2;
	IplImage *img1, *img2;
	IplImage *igrey1, *igrey2;
	IplImage *combined;
	CvSeq *key1, *key2;
	CvSeq *des1, *des2;
	CvMemStorage* storage;
	vector<int> pairs;
	int correctPairsNum;
	double K[9];
	double R[9],t[3];//for image 2
	double F[9];
	double lamda;//restrict baseline length
	std::vector<Pt3> result;

private:
	bool loadImage();
	bool surf();
	bool match();
	bool estimateFmatrix();
	bool estimateRelativePose();

public:
	SparseRec2View(string ipath1, string ipath2, double k[9], double lamda_=1);
	~SparseRec2View();

	bool run();
	bool save();
};

void Triangulate(const double x1, const double y1,
		const double x2, const double y2,
		const double P1[12], const double P2[12], double X[3]);