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

/* TagDetection.hpp
	modified from april/tag/TagDetection.java
*/

#include <iostream>
#include <string>
//opencv include
#include "OpenCVHelper.h"

namespace april
{
namespace tag
{

struct TagDetection {
	bool good;		/** Is the detection good enough? **/
	long obsCode;	/** Observed code **/
	long code;		/** Matched code **/
	int id;			/** What was the ID of the detected tag? **/
	int hammingDistance; /** The hamming distance between the detected code and the true code. **/
	int rotation;	/** How many 90 degree rotations were required to align the code. **/

	/////////////////////////////////////////////////
	// Fields below here are filled in by TagDetector
	/////////////////////////////////////////////////
	/** Position (in fractional pixel coordinates) of the detection. The
	 * points travel around counter clockwise around the target,
	 * always starting from the same corner of the tag. Dimensions
	 * [4][2]. **/
	double p[4][2];

	double cxy[2];	/** Center of tag in pixel coordinates. **/

	/** Measured in pixels, how long was the observed perimeter
	 * (i.e., excluding inferred perimeter which is used to
	 * connect incomplete quads) **/
	double observedPerimeter;

	/** A 3x3 homography that computes pixel coordinates from
	 * tag-relative coordinates. Both the input and output coordinates
	 * are 2D homogenous vectors, with y = Hx. y are pixel
	 * coordinates, x are tag-relative coordinates. Tag coordinates
	 * span from (-1,-1) to (1,1). The orientation of the homography
	 * reflects the orientation of the target. **/
	double homography[3][3];

	/** The homography is relative to image center, whose coordinates
	 * are below.
	 **/
	double hxy[2];

	/** interpolate point given (x,y) is in tag coordinate space from (-1,-1) to (1,1) **/
	inline void interpolate(double x, double y, double ret[2]) {
		double z = homography[2][0]*x + homography[2][1]*y + homography[2][2];
		ret[0] = (homography[0][0]*x + homography[0][1]*y + homography[0][2])/z + hxy[0];
		ret[1] = (homography[1][0]*x + homography[1][1]*y + homography[1][2])/z + hxy[1];
	}

	inline std::string toString() {
		return cv::format("[TagDetection code 0x%010x   id=%-5d   "
		                  "errors=%d   position =  (%8.2f,%8.2f) @ %3d deg]",
		                  code, id, hammingDistance, cxy[0], cxy[1], rotation*90);
	}
};

}//end of tag
}//end of april
