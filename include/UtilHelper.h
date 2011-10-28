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

/* UtilHelper.h
   Utility helper functions */

//standard include
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdarg.h>
#include <stdio.h>
#include <time.h>
#include <bitset>

namespace UtilHelper
{
template<typename TYPE>
void num2str(TYPE num, std::string &str)
{
	std::stringstream ss;
	ss << num;
	ss >> str;
}

template<typename TYPE>
std::string num2str(TYPE num)
{
	std::ostringstream oss;
	oss << num;
	return oss.str();
}

/** Compute the hamming distance between two INT64s. **/
template<typename T>
int hammingDistance(T a, T b)
{
	std::bitset<sizeof(T)*8> axb = a^b;
	return axb.count();
}

template<typename T>
int popcount(T a)
{
	std::bitset<sizeof(T)*8> bita = a;
	return bita.count();
}

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

//ensure val lies within (-PI,PI]
template<typename T>
T mod2pi(T val) {
#define CV_2_PI (CV_PI*2)
	int n = std::max((int)ceil(-0.5-val/CV_2_PI),(int)floor(0.5-val/CV_2_PI));
	return val+n*CV_2_PI;
}

inline double deg2rad(double deg) {
	return deg*CV_PI/180;
}

inline double rad2deg(double rad) {
	return rad*180/CV_PI;
}

}
