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

/* UtilHelper.h
   Utility helper functions */

#include <string>
#include <bitset>

#include "OpenCVHeaders.h"

namespace UtilHelper
{

//rgb image -> float gray image range in [0,1], use green channel
inline void green2float(const cv::Mat &rgb, cv::Mat &fim)
{
	fim.create(rgb.size(), CV_32FC1);
	float scale = 1.0f/255.0f;

if(rgb.channels()==3) {
	for(int i=0; i<rgb.rows; ++i) {
		for(int j=0; j<rgb.cols; ++j) {
			fim.at<float>(i,j) = (float) rgb.at<cv::Vec3b>(i,j)[1]*scale;
//			const cv::Vec3b& pix = rgb.at<cv::Vec3b>(i,j);
//			fim.at<float>(i,j) = ( ((int)77*pix[2]+151*pix[1]+28*pix[0])>>8 ) * scale;
		}
	}
} else if(rgb.channels()==4) {
	for(int i=0; i<rgb.rows; ++i) {
		for(int j=0; j<rgb.cols; ++j) {
			fim.at<float>(i,j) = (float) rgb.at<cv::Vec4b>(i,j)[1]*scale;
		}
	}
} else if(rgb.channels()==1) {
	for(int i=0; i<rgb.rows; ++i) {
		for(int j=0; j<rgb.cols; ++j) {
			fim.at<float>(i,j) = (float) rgb.at<uchar>(i,j)*scale;
		}
	}
}

}

//make sure INT64 has 64bits, i.e. sizeof(INT64)=8
#ifdef _WIN32
typedef __int64 INT64;
typedef unsigned __int64 UINT64;
#else
typedef int64_t INT64;
typedef uint64_t UINT64;
#endif

/** Compute the hamming distance between two INT64s. **/
template<typename T>
int hammingDistance(T a, T b)
{
	std::bitset<sizeof(T)*8> axb( (UINT64)a^b );
	return axb.count();
}

template<typename T>
int popcount(T a)
{
	std::bitset<sizeof(T)*8> bita( (UINT64)a );
	return bita.count();
}

#undef UINT64
#undef INT64

//bits of val, e.g. "10" for val=2
template<typename T>
std::string num2bits(T val)
{
	std::string rret;
	while(val) {
		rret.push_back(val&1?'1':'0');
		val>>=1;
	}
	return std::string(rret.rbegin(), rret.rend());
}

#define CV_2_PI (CV_PI*2)
//ensure val lies within (-PI,PI]
//inline double mod2pi_pos(double vin)
//{
//	double q = vin * 0.5/CV_PI + 0.5;
//	int qi = (int) q;
//	return vin - qi*CV_2_PI;
//}

//inline double mod2pi(double vin)
//{
//	double v;

//	if (vin < 0) {
//		v = -mod2pi_pos(-vin);
//	} else {
//		v = mod2pi_pos(vin);
//	}

//	return v;
//}

#define INV_2_PI (0.5/CV_PI)
inline double mod2pi(double val)
{
	int n = (int)floor(0.5-val*INV_2_PI);
	return val+n*CV_2_PI;
}

//Returns a value of v wrapped such that ref and v differ by no more +/-PI
inline double mod2pi(double ref, double v)
{
	return ref + mod2pi(v-ref);
}

inline double deg2rad(double deg)
{
	return deg*CV_PI/180;
}

inline double rad2deg(double rad)
{
	return rad*180/CV_PI;
}

}//end of UtilHelper
