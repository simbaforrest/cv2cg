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

namespace PerformanceHelper {
using namespace cv;
using namespace std;

inline double tic(bool start=true, string outtext="", bool verbose=true) {
	static double t;
	if(start) {
		t = (double)getTickCount();
		if(verbose) cout<<"[tic begin] "<<outtext<<endl;
	} else {
		t = ((double)getTickCount()-t)/getTickFrequency();
		if(verbose) cout<<"[tic end] " << t << " seconds elapsed."<<endl;
	}
	return t;
}

}
