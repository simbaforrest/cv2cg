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
#include "AllHelpers.h"
#include "TagUtils.hpp"

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
	std::string familyName; //in case of multiple tagfamily detections

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

	TagDetection() {
		helper::zeros(4, 2, p[0]);
		cxy[0]=cxy[1]=0;
		helper::zeros(3, 3, homography[0]);
	}

	/** interpolate point given (x,y) is in tag coordinate space from (-1,-1) to (1,1) **/
	inline void interpolate(double x, double y, double ret[2]) const {
		double z = homography[2][0]*x + homography[2][1]*y + homography[2][2];
		ret[0] = (homography[0][0]*x + homography[0][1]*y + homography[0][2])/z;
		ret[1] = (homography[1][0]*x + homography[1][1]*y + homography[1][2])/z;
	}

	/**
	 * undistort the detection results using K and distCoeffs
	 * output undistorted results to up, uc, uH:
	 *
	 * @param up: corresponding to this->p
	 * @param uc: corresponding to this->cxy
	 * @param uH: corresponding to this->homography
	 */
	template<typename MatType>
	inline void undistort(const cv::Mat& K, const cv::Mat& distCoeffs,
			std::vector<cv::Point2d>& up, cv::Point2d& uc, cv::Mat& uH) const {
		static const double pts[5][2]={
			{-1,-1},
			{ 1,-1},
			{ 1, 1},
			{-1, 1},
			{ 0, 0}
		};
		Homography33b homoCalc;
		std::vector<cv::Point2d> uv(5);
		for(int i=0; i<4; ++i)
			uv[i]=cv::Point2d(p[i][0],p[i][1]);
		uv[4]=cv::Point2d(cxy[0],cxy[1]);
		std::vector<cv::Point2d> xy(5);
		up.resize(4);
		cv::undistortPoints(uv,xy,K,distCoeffs);
		for(int i=0; i<4; ++i) {
			uv[i].x=up[i].x=(K.at<MatType>(0,0)*xy[i].x+K.at<MatType>(0,2))/K.at<MatType>(2,2);
			uv[i].y=up[i].y=(K.at<MatType>(1,1)*xy[i].y+K.at<MatType>(1,2))/K.at<MatType>(2,2);
			homoCalc.addCorrespondence(pts[i][0],pts[i][1],uv[i].x,uv[i].y);
		}
		uv[4].x=uc.x=(K.at<MatType>(0,0)*xy[4].x+K.at<MatType>(0,2))/K.at<MatType>(2,2);
		uv[4].y=uc.y=(K.at<MatType>(1,1)*xy[4].y+K.at<MatType>(1,2))/K.at<MatType>(2,2);
		homoCalc.addCorrespondence(pts[4][0],pts[4][1],uv[4].x,uv[4].y);
		double newH[3][3];
		homoCalc.getH(newH);
		cv::Mat newH_(3,3,CV_64FC1,(void*)newH);
		newH_.copyTo(uH);
	}

	inline std::string name() const {
		return familyName + cv::format(".%d", id);
	}

	inline std::string toString(bool bshort=true) const {
		return bshort?
				familyName+cv::format("_%d",id)
				:cv::format("[TagDetection code 0x%010x   id=%-5d   "
		                  "errors=%d   position =  (%8.2f,%8.2f) @ %3d deg]",
		                  code, id, hammingDistance, cxy[0], cxy[1], rotation*90);
	}
};

}//end of tag
}//end of april
