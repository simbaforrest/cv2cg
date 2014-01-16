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

/* VisHelper.h
   Visualization related helper functions*/

#include <stdlib.h> //rand

#include "OpenCVHeaders.h"

namespace VisHelper
{
const cv::Scalar CV_RED(255,0,0);
const cv::Scalar CV_GREEN(0,255,0);
const cv::Scalar CV_BLUE(0,0,255);
const cv::Scalar CV_RG(255,255,0);
const cv::Scalar CV_RB(255,0,255);
const cv::Scalar CV_GB(0,255,255);
const cv::Scalar CV_WHITE(255,255,255);
const cv::Scalar CV_BLACK(0,0,0);
const cv::Scalar CV_GRAY(128,128,128);

/**
draw a homography (a quad) given the four corners to be mapped by Homography

@param[in,out] image target image to be drawed on
@param[in] Homo homography matrix, <3x3>
@param[in] crns 4 corners in world coordinate system
*/
inline void drawHomography(cv::Mat& image, const cv::Mat& Homo, double const crns[4][2]) {
	static cv::Scalar homocolors[] = {
		CV_BLACK,
		CV_GREEN,
		CV_BLUE,
		CV_RED  };
	const cv::Mat_<double>& mH = Homo;
	std::vector<cv::Point2f> corners(4);
	for(int i = 0; i < 4; i++ ) {
		double ptx = crns[i][0], pty=crns[i][1];
		double w = 1./(mH(2,0)*ptx + mH(2,1)*pty + mH(2,2));
		corners[i] =
		    cv::Point2f((float)((mH(0,0)*ptx + mH(0,1)*pty + mH(0,2))*w),
		            (float)((mH(1,0)*ptx + mH(1,1)*pty + mH(1,2))*w));
	}
	for(int i = 0; i < 4; ++i) {
		const cv::Point& r1 = corners[i%4];
		const cv::Point& r2 = corners[(i+1)%4];
		cv::line( image, r1, r2, homocolors[i], 2 );
	}
	cv::line(image, corners[0], corners[2], CV_GB, 2);
	cv::line(image, corners[1], corners[3], CV_GB, 2);
}
//short-hand for apriltag visulization
inline void drawHomography(cv::Mat& image, const cv::Mat& Homo) {
	static double crns[4][2]={
		{-1, -1},
		{ 1, -1},
		{ 1,  1},
		{-1,  1}
	};
	drawHomography(image,Homo,crns);
}

/**
draw a 3D pyramid on image, projection matrix \f$P=K[R,T]\f$

@param[in,out] image target image to draw
@param[in] K calibration matrix, <3x3>
@param[in] R rotation matrix, <3x3>
@param[in] T translation vector, <3x1>
@param[in] crns 8 corners of the pyramid, 0~3 bottom, 4~7 top, in world coordinate system
*/
inline void drawPyramid(cv::Mat& image,
		double const K[9], double const R[9], double const T[3],
		double const crns[8][3] ) {
	static cv::Scalar linecolors[] = {
		CV_BLACK,
		CV_GREEN,
		CV_BLUE,
		CV_RED  };
	static int lineidx[12][2] = {
		{0,1},{1,2},{2,3},{3,0},
		{4,5},{5,6},{6,7},{7,4},
		{0,4},{1,5},{2,6},{3,7} };
	double P[12];
	CameraHelper::compose(K,R,T,P,false);
	double p[8][2];
	for(int i=0; i<8; ++i) {
		CameraHelper::project(P,crns[i],p[i]);
	}
	for(int i=0; i<3; ++i) {
		for(int j=i*4; j<4+i*4; ++j) {
			int s=lineidx[j][0], e=lineidx[j][1];
			cv::Point r1(p[s][0],p[s][1]);
			cv::Point r2(p[e][0],p[e][1]);
			cv::line( image, r1, r2, linecolors[j%4], 2 );
		}
	}
}

/**
generate pseudocolor look up table, using opencv's own random version

@param ncolors required number of colors
@return pseudocolor look up table
*/
inline std::vector<cv::Scalar> pseudocolor(int ncolors) {
	if(ncolors<=0) ncolors=10;
	std::vector<cv::Scalar> ret(ncolors);
	//theRNG() = (uint64)time(0);
	//generateColors( ret, ncolors );
	for(int i=0; i<ncolors; ++i) {
		cv::Scalar& color=ret[i];
		color[0]=rand()%255;
		color[1]=rand()%255;
		color[2]=rand()%255;
	}
	return ret;
}

}//end of VisHelper
