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

/* PerformanceHelper.h
   measure performance */

//standard include
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdio.h>
#include <time.h>
#include <limits.h>
//opencv include
#include "OpenCVHelper.h"

namespace PerformanceHelper
{
using namespace cv;
using namespace std;

/**
computer rms error, only using valid region (non-zero region)

@param error[int] error vector stored in a cv::Mat
@param mask[in] mask->at(i)!=0 means valid region, default is 0, means valid for all
@return rms error
*/
template<typename MatType>
double rms(const Mat &error, const Mat *mask=0)
{
	Mat maskRow;
	int validN;
	if(mask) {
		maskRow = mask->reshape(0,1);
		validN = countNonZero(maskRow);
	} else {
		validN = error.total();
	}
	if(!maskRow.empty() && validN==0)
		return numeric_limits<double>::quiet_NaN();

	double ret = 0;
	Mat errorRow = error.reshape(0,1);
	for(int i=0; i<(int)errorRow.total(); ++i) {
		MatType ie = errorRow.at<MatType>(i);
		bool valid = maskRow.empty() || maskRow.at<MatType>(i)!=0;
		ret += valid*(ie*ie);
	}
	ret /= validN;
	return sqrt(ret);
}

/**
compute zero mean normalized cross-correlation (ZNCC) between Mat w and t,
only using valid region (non-zero region) specified by mask

@param w vector w
@param t vector t
@return zncc
*/
template<typename MatType>
double zncc(const Mat& w, const Mat& t, const Mat *mask=0)
{
	Mat maskRow;
	int validN;
	if(mask) {
		maskRow = mask->reshape(0,1);
		validN = countNonZero(maskRow);
	} else {
		validN = w.total();
	}
	if(!maskRow.empty() && validN==0)
		return numeric_limits<double>::quiet_NaN();

	Scalar mw, mt, dw, dt;
	Mat wRow = w.reshape(0,1);
	Mat tRow = t.reshape(0,1);
	if(maskRow.empty()) {
		meanStdDev(wRow, mw, dw);
		meanStdDev(tRow, mt, dt);
	} else {
		meanStdDev(wRow, mw, dw, maskRow);
		meanStdDev(tRow, mt, dt, maskRow);
	}

	double ret = 0;
	for(int i=0; i<(int)w.total(); ++i) {
		MatType iw = wRow.at<MatType>(i);
		MatType it = tRow.at<MatType>(i);
		ret+= (iw!=0)*(iw-mw.val[0])*(it-mt.val[0]);
	}
	ret/=dw.val[0]*dt.val[0]*validN;
	return ret;
}

/**
\class PerformanceHelper::PerformanceMeasurer PerformanceHelper.h "PerformanceHelper.h"
\brief helper class for measuring time elapse
*/
struct PerformanceMeasurer {
	int scale;
	double startTick;

	/**
	constructor
	
	@param scale time scale, 1 means second, 1000 means milli-second,
	             1/60.0 means minutes, etc.
	*/
	PerformanceMeasurer(int scale=1) {
		this->scale = scale;
		startTick=0;
	}

	/**
	start record time, similar to matlab function "tic";
	
	@return the start tick
	*/
	inline double tic() {
		return startTick = (double)getTickCount();
	}

	/**
	return duration from last tic, in (second * scale), similar to matlab function "toc"
	
	@return duration from last tic,  in (second * scale)
	*/
	inline double toc() {
		return ((double)getTickCount()-startTick)/getTickFrequency() * scale;
	}

	/**
	equivalent to { toc(); tic(); }
	
	@return duration from last tic,  in (second * scale)
	*/
	inline double toctic() {
		double ret = ((double)getTickCount()-startTick)/getTickFrequency() * scale;
		tic();
		return ret;
	}
};

}
