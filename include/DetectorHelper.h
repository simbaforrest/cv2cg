#pragma once
/*
 *  Copyright (c) 2011  Chen Feng (cforrest (at) umich.edu)
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

/* DetectorHelper.h
   my own detectors */

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

namespace DetectorHelper
{
using namespace cv;
using namespace std;

class LongTermDAFD
{
public:

	LongTermDAFD(Ptr<AdjusterAdapter>& a,
	             int min_features, int max_features, int max_iters ) :
		escape_iters_(max_iters), min_features_(min_features),
		max_features_(max_features), adjuster_(a)
	{}

	inline bool empty() {
		return adjuster_.empty() || adjuster_->empty();
	}

	inline void detect(const Mat &image,
	                   vector<KeyPoint>& keypoints,
	                   const Mat &mask=Mat() ) {
		//for oscillation testing
		bool down = false;
		bool up = false;

		//flag for whether the correct threshhold has been reached
		bool thresh_good = false;

		//break if the desired number hasn't been reached.
		int iter_count = escape_iters_;

		while( iter_count > 0 && !(down && up) && !thresh_good && adjuster_->good() ) {
			keypoints.clear();

			//the adjuster takes care of calling the detector with updated parameters
			adjuster_->detect(image, keypoints,mask);

			if( int(keypoints.size()) < min_features_ ) {
				down = true;
				adjuster_->tooFew(min_features_, (int)keypoints.size());
			} else if( int(keypoints.size()) > max_features_ ) {
				up = true;
				adjuster_->tooMany(max_features_, (int)keypoints.size());
			} else {
				thresh_good = true;
			}

			iter_count--;
		}

	}

	int escape_iters_;
	int min_features_, max_features_;
	Ptr<AdjusterAdapter> adjuster_;
};

}
