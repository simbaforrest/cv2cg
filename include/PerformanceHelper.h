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
//opencv include
#include "OpenCVHelper.h"

namespace PerformanceHelper
{
using namespace cv;
using namespace std;

struct PerformanceMeasurer {
	int scale;
	double startTick;

	PerformanceMeasurer() {
		scale = 1;
		startTick=0;
	}

	inline double tic() {
		return startTick = (double)getTickCount();
	}
	//return duration from last tic, in (second * scale)
	inline double toc() {
		return ((double)getTickCount()-startTick)/getTickFrequency() * scale;
	}

	//equivalent to { toc(); tic(); }
	inline double toctic() {
		double ret = ((double)getTickCount()-startTick)/getTickFrequency() * scale;
		tic();
		return ret;
	}
};

}
