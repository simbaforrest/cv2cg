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

#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include "Log.h"
#include "OpenCVHelper.h"

#include "FastTracker.hpp"

namespace klt {

using namespace std;
using namespace cv;

class Tracker {
protected:
	vector<Point2f> oldX; //old pts to track
	vector<Point2f> newX; //new pts being tracked
	Mat oldframe; //previous gray frame

public:
	vector<uchar> status;
	vector<float> err;
	TermCriteria termcrit;
	Size winSize;
	int maxLevel;
	double derivedLambda;
	fast::TrackParam tp;

	double ransacThresh;
	double inlierThresh; //percentage of inlier to be considered as valid

	vector<unsigned char> match_mask;

public:
	Tracker(int winW=8, int winH=8,
	           int termIter=5, int termEps=0.3,
	           int maxlevel=3, double lambda=0.3,
	           double ransacT=2, double inlierT=0.2) :
		termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,termIter,termEps),
		winSize(winW,winH), maxLevel(maxlevel), derivedLambda(lambda),
		ransacThresh(ransacT), inlierThresh(inlierT)
	{
	}

	/**
	init tracker by setting newX to be H*tplX and refine newX on newframe
	
	@param[in] newframe current frame, CV_8UC1
	@param[in] tplX points on template image
	@param[in] H current homography
	*/
	inline void init(const Mat& newframe,
		const vector<Point2f>& tplX, const Mat& H)
	{
		assert(newframe.type()==CV_8UC1);
		newX.resize(tplX.size());
		Mat nptsmat(newX);
		perspectiveTransform(Mat(tplX), nptsmat, H);
		cornerSubPix(newframe, newX, winSize, Size(-1,-1), termcrit);
		oldX.resize(newX.size());
		std::copy(newX.begin(), newX.end(), oldX.begin());
		newframe.copyTo(oldframe);
	}

	/**
	main klt tracking
	
	@param[in] newframe current gray frame, CV_8UC1
	@param[in] tplX points on template image, tplX.size()==oldX.size()
	@param[out] H homography to be updated
	@param[int,out] image visualization target, default 0 means do not draw
	*/
	inline bool operator()(const Mat& newframe,
		const vector<Point2f>& tplX, Mat& H, Mat *image=0)
	{
		if(oldX.empty()) return false;
		if(oldX.size()!=tplX.size()) return false;
		if(oldframe.empty()) newframe.copyTo(oldframe);

		match_mask.clear();
		status.clear();
		err.clear();

//		fast::track(oldframe, newframe, oldX, newX, status, err, tp);
		calcOpticalFlowPyrLK(oldframe, newframe, oldX, newX,
			                 status, err, winSize,
			                 maxLevel, termcrit, derivedLambda);
		if(!validateKLT( (int)tplX.size() )) return false;

		Mat tmpH = findHomography(Mat(tplX), Mat(newX),
				           match_mask, RANSAC, ransacThresh);
		if( !validateRANSAC(newframe.size(), (int)tplX.size()) ) return false;

		H = tmpH;
		if(image) drawTrail(*image);
		return true;
	}

	inline void globalGeometricConstraintEhancement(
		const vector<Point2f>& tplX, const Mat& H)
	{
		assert(newX.size()==tplX.size());
		Mat nptsmat(newX);
		perspectiveTransform(Mat(tplX), nptsmat, H);
	}

	/**
	update interal tracking data, shall be called at the end of each iteration
	
	@param[in,out] newframe current frame to be stored in variable oldframe
	*/
	inline void update(Mat& newframe) {
		std::swap(oldframe, newframe);
		std::swap(newX, oldX);
	}

	/**
	draw track trail, should be used before update()
	
	@param[in,out] image visualization target
	*/
	inline void drawTrail(Mat& image) {
		for(int i=0; i<(int)newX.size(); ++i) {
			if( status[i] ) {
				circle(image, newX[i], 2, CV_GREEN, -1);
				line(image, newX[i], oldX[i], CV_BLUE );
			}
			if(match_mask[i]) {
				circle( image, newX[i], 1, CV_RED, -1);
			}
		}
	}

protected: //internal helper functions
	/**
	validate KLT result, by checking #(non-zero status points)
	
	@return true if KLT valid
	*/
	inline bool validateKLT(int tsize) {
		int cnt=0;
		for(int i=0; i<(int)newX.size(); ++i) {
			//to make these points rejected by RANSAC
			newX[i] = status[i]?newX[i]:Point2f(0,0);
			cnt += status[i];
		}
		return cnt >= inlierThresh*tsize;
	}

	/**
	validate RANSAC result, by checking #(ransac inlier within current frame)
	
	@param s size of current frame
	@return true if RANSAC valid
	*/
	inline bool validateRANSAC(const Size& s, int tsize) {
		int cnt = 0;
		for(int i=0; i<(int)newX.size(); ++i) {
			const Point2f& p = newX[i];
			cnt += match_mask[i] &&
				p.x>0 && p.y>0 && p.x<s.width && p.y<s.height;
		}
		return cnt >= inlierThresh*tsize;
	}
};//end of class klt::Tracker

}; //end of namespace klt

typedef klt::Tracker KLTTracker;
