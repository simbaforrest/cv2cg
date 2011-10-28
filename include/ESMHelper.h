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

/* ESMHelper.h
   wrapper for ESM homography tracking refinement algorithm */

#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include "Log.h"
#include "OpenCVHelper.h"

extern "C" {
#include "ESMlibry.h"
}

using namespace cv;
using namespace std;

//!!! data still stored in m, and m will be changed to CV_32F if it is not
inline void Mat2imageStruct(Mat &m, imageStruct &I)
{
	if(m.type()!=CV_32FC1) {
		if(m.channels()>1) {
			Mat gm;
			cvtColor(m,gm,CV_RGB2GRAY);
			gm.convertTo(m,CV_32F);
		} else {
			m.convertTo(m,CV_32F);
		}
	}

	I.data = (float *)(m.data);
	I.rows = m.rows;
	I.cols = m.cols;
}

//!!! data will be copied to m
inline void imageStruct2Mat(imageStruct &I, Mat &m)
{
	Mat mI(I.rows,I.cols,CV_32FC1,I.data);
	mI.copyTo(m);
	m.convertTo(m,CV_8U);
}

struct ESMKernel {
	trackStruct T;
	Mat RefImg;   //internally hold the reference/template image
	bool inited;
	int px,py,sx,sy;

	ESMKernel() {
		inited=false;
	}
	~ESMKernel() {
		if(inited) {
			FreeTrack(&T);
		}
	}

	inline bool init(const Mat &refimg,
	                 int posx, int posy,
	                 int sizex, int sizey,
	                 int maxIter=5, int maxPrec=2) {
		px=posx;
		py=posy;
		sx=sizex;
		sy=sizey;

		if(inited) {
			FreeTrack(&T);
		}
		refimg.copyTo(RefImg);
		imageStruct I;
		Mat2imageStruct(RefImg, I);
		// Memory allocation for the tracking
		if (MallTrack (&T, &I, posx, posy, sizex, sizey, maxIter, maxPrec)) {
			return inited = false;
		}
		return inited = true;
	}

	inline bool run(imageStruct &I) {
		if(!inited) {
			cout<<"[ESMKernel::run] please init the ESMKernel first!"<<endl;
			return false;
		}

		// Perform the tracking
		if (MakeTrack (&T, &I)) {
			return false;
		}
		return true;
	}

	inline void setH(double const H[9]) {
		for(int i=0; i<9; i++) {
			T.homog[i] = H[i];
		}
	}

	inline void getH(double H[9]) const {
		for(int i=0; i<9; i++) {
			H[i] = T.homog[i];
		}
	}

	template<typename Iterator>
	void setH(Iterator itr) {
		for(int i=0; i<9; ++i, ++itr) {
			T.homog[i] = (*itr);
		}
	}

	template<typename Iterator>
	void getH(Iterator itr) const {
		for(int i=0; i<9; ++i, ++itr) {
			(*itr) = T.homog[i];
		}
	}
};

struct ESMTracker {
	ESMKernel kernel;
	Mat CurImg; //internally hold the image being tracked

	// miter: the number of iterations of the ESM algorithm (>= 1);
	// mprec: the precision of the ESM algorithm (1..10)
	// low precision = 1, high precision = 10;
	// miter and mprec should be chosen depending on the available
	// computation time;
	// For low-speed PCs or high video framerate choose low miter and low mprec.
	// (posx,posy) The window position (upper left corner)
	// The image coordinates start from (0,0)
	// The window size sizex,sizey
	inline bool init(const Mat &refimg,
	                 int posx, int posy,
	                 int sizex, int sizey,
	                 int maxIter=5, int maxPrec=2) {
		if (!kernel.init(refimg,posx,posy,sizex,sizey,maxIter,maxPrec)) {
			cout<<"[ESMTracker] failed to inited."<<endl;
			return false;
		}
		cout<<"[ESMTracker] inited."<<endl;
		return true;
	}

	inline bool run(Mat &curimg, bool keepCurrent=true) {
		imageStruct I;
		if(keepCurrent) {
			curimg.copyTo(CurImg);
			Mat2imageStruct(CurImg, I);
		} else {
			Mat2imageStruct(curimg, I);
		}
		return kernel.run(I);
	}

	inline void setH(double const H[9]) {
		kernel.setH(H);
	}

	inline void getH(double H[9]) const {
		kernel.getH(H);
	}

	template<typename Iterator>
	void setH(Iterator itr) {
		kernel.setH(itr);
	}

	template<typename Iterator>
	void getH(Iterator itr) const {
		kernel.getH(itr);
	}
};

struct ESMMultipleTracker {
	vector<ESMKernel> kArr;
	Mat CurImg; //internally hold the image being tracked

	inline bool init(const Mat &refimg,
	                 int posx, int posy,
	                 int sizex, int sizey,
	                 int maxIter=5, int maxPrec=2) {
		kArr.push_back(ESMKernel());
		ESMKernel &kernel = kArr[kArr.size()-1];
		if (!kernel.init(refimg,posx,posy,sizex,sizey,maxIter,maxPrec)) {
			cout<<"[ESMMultipleTracker] new("<<kArr.size()-1<<") kernel failed to inited."<<endl;
			kArr.resize(kArr.size()-1);
			return false;
		}
		cout<<"[ESMMultipleTracker] new("<<kArr.size()-1<<") kernel inited."<<endl;
		return true;
	}

	inline bool run(Mat &curimg, bool keepCurrent=true) {
		imageStruct I;
		if(keepCurrent) {
			curimg.copyTo(CurImg);
			Mat2imageStruct(CurImg, I);
		} else {
			Mat2imageStruct(curimg, I);
		}

		int cnt=(int)kArr.size();
		int oi=0;
		for(int i=0; i<cnt; ++oi) {
			ESMKernel &kernel = kArr[i];
			if(!kernel.run(I)) {
				swap(kArr[i],kArr[cnt-1]);
				--cnt;
				cout<<"[ESMMultipleTracker] kernel("
				    <<oi<<") lost track, deleted."<<endl;
			} else {
				++i;
			}
		}
		bool ret = cnt==(int)kArr.size();
		if(cnt==0) {
			kArr.clear();
		} else {
			kArr.resize(cnt);
		}
		return ret;
	}
};
