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

#include "Log.h"
#include "OpenCVHelper.h"

using namespace std;
using namespace cv;

class SparseRec2View {
private:
	FeatureDetector* detector;
	DescriptorExtractor* descriptor;
	DescriptorMatcher* matcher;

	//img1 as train image, img2 as query image
	string imgpath1, imgpath2;
	string dir;
	string imgname1, imgname2;
	Mat img1, img2;
	Mat igrey1, igrey2;
	Mat combined;
	vector<KeyPoint> key1, key2;
	vector<DMatch> matches;
	vector<Point3f> results;//reconstructed points
	vector<Point2f> p1, p2;//matched image points
	vector<uchar> inliers;//inliers in p1-p2 for fmatrix, 0 means outlier
	int inliersNum;
	double K[9];
	double R[9],t[3];//for image 2
	double F[9];
	double lamda;//restrict baseline length
	bool _onlymatch;
private:
	bool loadImage();
	bool detect();
	bool match();
	bool fmatrix();//estimate fundamental matrix
	bool estimateRelativePose();

public:
	SparseRec2View(string ipath1, string ipath2, double k[9], double lamda_=1, bool onlymatch=false);
	~SparseRec2View();

	bool run();
	bool save();
};
