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

#include <limits.h> //quiet_NaN
#include "OpenCVHeaders.h"

namespace PerformanceHelper
{
/**
computer rms error, only using valid region (non-zero region)

@param error[int] error vector stored in a cv::Mat
@param mask[in] mask->at(i)!=0 means valid region, default is 0, means valid for all
@return rms error
*/
inline double rms(const cv::Mat &error, const cv::Mat *mask=0)
{
	assert(error.dims<=2);

	cv::Mat maskRow;
	int validN;
	if(mask) {
		maskRow = mask->reshape(0,1);
		validN = cv::countNonZero(maskRow);
	} else {
		validN = error.total();
	}
	if(!maskRow.empty() && validN==0)
		return std::numeric_limits<double>::quiet_NaN();

	double ret = 0;
	cv::Mat errorRow = error.reshape(0,1);
	cv::Scalar errorSum;
	if(mask) {
		maskRow.convertTo(maskRow,errorRow.type(),1.0/255);
		errorSum=cv::sum(errorRow.mul(errorRow).mul(maskRow));
	} else {
		errorSum=cv::sum(errorRow.mul(errorRow));
	}
	return sqrt(errorSum[0]/validN);
}

/**
compute zero mean normalized cross-correlation (ZNCC) between Mat w and t,
only using valid region (non-zero region) specified by mask

@param w vector w
@param t vector t
@return zncc
*/
inline double zncc(const cv::Mat& w, const cv::Mat& t, const cv::Mat *mask=0)
{
	assert(w.dims<=2 || w.dims==t.dims);

	cv::Mat maskRow;
	int validN;
	if(mask) {
		maskRow = mask->reshape(0,1);
		validN = cv::countNonZero(maskRow);
	} else {
		validN = w.total();
	}
	if(!maskRow.empty() && validN==0)
		return std::numeric_limits<double>::quiet_NaN();

	cv::Scalar mw, mt, dw, dt;
	cv::Mat wRow = w.reshape(0,1);
	cv::Mat tRow = t.reshape(0,1);
	if(maskRow.empty()) {
		cv::meanStdDev(wRow, mw, dw);
		cv::meanStdDev(tRow, mt, dt);
	} else {
		cv::meanStdDev(wRow, mw, dw, maskRow);
		cv::meanStdDev(tRow, mt, dt, maskRow);
	}

	cv::Mat wRow_zm = wRow-mw.val[0];
	cv::Mat tRow_zm = tRow-mt.val[0];
	cv::Scalar errorSum;
	if(mask) {
		maskRow.convertTo(maskRow,wRow_zm.type(),1.0/255);
		errorSum=cv::sum(wRow_zm.mul(tRow_zm).mul(maskRow));
	} else {
		errorSum=cv::sum(wRow_zm.mul(tRow_zm));
	}
	return sqrt(errorSum[0]/(dw.val[0]*dt.val[0]*validN));
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
		return startTick = (double)cv::getTickCount();
	}

	/**
	return duration from last tic, in (second * scale), similar to matlab function "toc"
	
	@return duration from last tic,  in (second * scale)
	*/
	inline double toc() {
		return ((double)cv::getTickCount()-startTick)/cv::getTickFrequency() * scale;
	}

	/**
	equivalent to { toc(); tic(); }
	
	@return duration from last tic,  in (second * scale)
	*/
	inline double toctic() {
		double ret = ((double)cv::getTickCount()-startTick)/cv::getTickFrequency() * scale;
		tic();
		return ret;
	}
};

}
