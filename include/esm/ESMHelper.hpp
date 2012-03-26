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

/* ESMHelper.h
   wrapper for ESM homography tracking refinement algorithm */

#include <iostream>
#include <limits>
#include <vector>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include "OpenCVHelper.h"

#ifndef _WIN32 //ESMlibry.lib under windows seems to be compiled as cpp code rather than c code
extern "C" {
#endif
	#include "ESMlibry.h"
#ifndef _WIN32
}
#endif

#include "ESMInterface.h"

namespace esm {

#ifndef ESM_DEBUG
	#define ESM_DEBUG 0
#endif

using namespace cv;
using namespace std;

/**
convert cv::Mat to imageStruct, data still stored in cv::Mat, but m will be
changed to CV_32F if it is not

@param m cv::Mat which is responsible for deallocate the real data block
@see I imageStruct for ESMlibry
*/
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
#ifdef _WIN32 //win version and linux version of ESMLibry.h is slightly different!
	I.clrs = 1;
#endif
}

/**
convert imageStruct to cv::Mat, data will be copied to m

@param I imageStruct
@see m CV_32FC1 type of image, value between [0,255]
*/
inline void imageStruct2Mat(imageStruct &I, Mat &m)
{
	Mat mI(I.rows,I.cols,CV_32FC1,I.data);
	mI.copyTo(m);
}

struct Tracker : public Interface {
	trackStruct T;
	Mat RefImg;   //internally hold the reference/template image
	bool inited;

	Tracker() { inited=false; }
	~Tracker() { if(inited) FreeTrack(&T); }

	inline bool init(const Mat& refimg) {
#if ESM_DEBUG
		namedWindow("error");
		cvMoveWindow("error",660,10);
		namedWindow("warp");
		cvMoveWindow("warp",660,270);
#endif
		return init(refimg, 0,0, refimg.cols,refimg.rows);
	}

	inline bool operator()(Mat& curimg, Mat& H, double& zncc, double& rms) {
		//update internal homography
		for(int i=0; i<3; ++i)
			for(int j=0; j<3; ++j)
				T.homog[i*3+j] = H.at<double>(i,j);
		bool ret = true;
		if( this->maxIter >0 ) {
			ret = this->run(curimg); //true tracking
			if(ret) {
				for(int i=0; i<3; ++i)
					for(int j=0; j<3; ++j)
						H.at<double>(i,j) = T.homog[i*3+j];
			} else {
				zncc = rms = numeric_limits<double>::quiet_NaN();
				return false;
			}
		}

		//evaluation of tracking quality
		Mat warp;
		warpPerspective(curimg,
			warp, H,
			RefImg.size(), INTER_LINEAR|WARP_INVERSE_MAP);
		warp.convertTo(warp,CV_32F);
		Mat error = warp-RefImg;
		Mat mask = warp!=0;
		rms = helper::rms<float>(error, &mask);
		zncc = helper::zncc<float>(warp,RefImg,&mask);

		cout<<"[ESM] RMS="<<rms<<"|ZNCC="<<zncc<<endl;
#if ESM_DEBUG
		cv::imshow("warp", warp/255.0);
		cv::imshow("error", error/255.0);
#endif
		return ret;
	}

	inline void setTermCrit(int maxIter=5, double mprec=2) {
		T.miter = this->maxIter = maxIter>0?maxIter:0;
		T.mprec = this->mprec   = mprec;
	}

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

	inline bool run(const Mat& curimg) {
		if(!inited) {
			cout<<"[ESMTracker::run] please init first!"<<endl;
			return false;
		}

		Mat CurImg;
		curimg.copyTo(CurImg);
		imageStruct I;
		Mat2imageStruct(CurImg, I);

		// Perform the tracking
		if (MakeTrack (&T, &I)) {
			return false;
		}
		return true;
	}
}; //end of struct Tracker

}//end of namespace esm

typedef esm::Tracker ESMTracker;
