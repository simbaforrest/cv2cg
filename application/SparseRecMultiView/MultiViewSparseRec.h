#pragma once
/* 
 *  Copyright (c) 2011  Chen Feng (cforrest (at) umich.edu)
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

/* MultiViewSparseRec.h */

//standard include
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "OpenCVHelper.h"

using namespace std;
using namespace cv;

struct MVSRpicture {
	Mat img,grey;
	vector<KeyPoint> key;//keypoints to be reconstructed
	Mat des;//descriptors for keypoints

	//map to object point idx, map[i]=j means
	//key[i]<->the jth object point
	vector<int> map;
};

struct MVSRobjpoint {
	//<i,j> means this obj point is observed in jth keypoint on picture i
	struct Record {
		int imgi, keyj;
		Record(int i=-1, int j=-1) {imgi=i; keyj=j;}
	};
	vector<Record> obs;
	Point3d pos;
	bool flag;//true if reconstructed, otherwise false

	MVSRobjpoint() {flag=false;}
	MVSRobjpoint(double x, double y, double z) : pos(x,y,z) {flag=true;}
	inline void observe(int imgi, int keyj) {obs.push_back(Record(imgi,keyj));}
};

class MVSR //MultiViewSparseRec
{
private:
	FeatureDetector* detector;
	DescriptorExtractor* descriptor;
	DescriptorMatcher* matcher;

	vector<MVSRpicture> pictures;
	vector<MVSRobjpoint> clouds;
public:
	MVSR(void);
	~MVSR(void);

	bool run(vector<string> imgnamelist);
private:
	bool loadimage(vector<string> imgnamelist);
	bool detect();
	bool match();
	bool save();

	//add match pair for
	//(keypoint[keyi] in img[imgi])<->(keypoint[keyj] in img[imgj])
	void addmatchpair(int imgi, int keyi, int imgj, int keyj);
	//add linkage between object points and image's keypoint
	void linkimgobj(int objIdx, int imgi, int keyi);
};
