#pragma once
/************************************************************************\

  Copyright 2011 The University of Michigan.
  All Rights Reserved.

  Permission to use, copy, modify and distribute this software
  and its documentation for educational, research and non-profit
  purposes, without fee, and without a written agreement is
  hereby granted, provided that the above copyright notice and
  the following paragraph appear in all copies.

  THIS SOFTWARE IS PROVIDED BY THE UNIVERSITY OF MICHIGAN "AS IS" AND 
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE UNIVERSITY OF MICHIGAN
  OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  
  Authors:

			Chen Feng
            Laboratory for Interactive Visualization in Engineering (LIVE)
			Department of Civil and Environmental Engineering
            2350 Hayward Street, Suite 2340 GG Brown Building
            University of Michigan
            Ann Arbor, MI 48109-2125
			Phone:    (734)764-8495
			EMail:    simbaforrest@gmail.com
			WWW site: http://www.umich.edu/~cforrest
            
			Vineet R. Kamat
            Laboratory for Interactive Visualization in Engineering (LIVE)
			Department of Civil and Environmental Engineering
            2350 Hayward Street, Suite 2340 GG Brown Building
            University of Michigan
            Ann Arbor, MI 48109-2125
            Phone:    (734)764-4325
			EMail:    vkamat@umich.edu
			WWW site: http://pathfinder.engin.umich.edu

\************************************************************************/

/* TagDetection.hpp
	modified from april/tag/TagDetection.java
	git://april.eecs.umich.edu/home/git/april.git
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

	TagDetection() {
		helper::zeros(4,2,p[0]);
		cxy[0]=cxy[1]=0;
		helper::zeros(3,3,homography[0]);
		hxy[0]=hxy[1]=0;
	}

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
