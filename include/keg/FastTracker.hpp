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

//standard include
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <list>
#include <set>
#include <stdarg.h>
#include <stdio.h>
#include <time.h>

//opencv include
#include "OpenCVHelper.h"
#include "SearchHelper.h"

#ifndef FAST_DEBUG
	#define FAST_DEBUG 0
#endif

namespace fast {

using namespace cv;
using namespace std;
using SearchHelper::Gridder;

struct TrackParam {
	double gridCellSize;
	double searchRange;//8
	double cmpWinSize;
	int fastThresh;
	int fastUp,fastLow;
	int fastMaxIter;

	TrackParam() {
		gridCellSize=32;
		searchRange=16;
		cmpWinSize=8;
		fastThresh=30;
		fastMaxIter=3;
	}
};

inline void key2pt(const vector<KeyPoint>& src, vector<Point2f>& dst) {
	dst.resize(src.size());
	for(int i=0; i<(int)src.size(); ++i) {
		dst[i] = src[i].pt;
	}
}

inline void dynamicThresh(const Mat& frame, vector<KeyPoint>& keys, TrackParam& tp) {
	Ptr<FeatureDetector> detector;
	for(int i=0; i<tp.fastMaxIter; ++i) {
		detector=new FastFeatureDetector(tp.fastThresh);
		detector->detect(frame, keys);

		int curNum = (int)keys.size();
		if(curNum<tp.fastLow) --tp.fastThresh;
		else if(curNum>tp.fastUp) ++tp.fastThresh;
		else break;
	}
}

template<typename T>
T compare(const Mat& oldF, const Mat& newF,
		const Point2f& ox, const Point2f& nx, double winsize) {
	int W = oldF.cols, H = oldF.rows;
	T err=0;
	for(int i=-winsize; i<=winsize; ++i) {
		int oi=ox.x+i, ni=nx.x+i;
		for(int j=-winsize; j<=winsize; ++j) {
			int oj=ox.y+j, nj=nx.y+j;
			bool ov=(oi>=0 && oi<W)&&(oj>=0 && oj<H);
			bool nv=(ni>=0 && ni<W)&&(nj>=0 && nj<H);
			T opix = ov?oldF.at<uchar>(oj,oi):0;
			T npix = nv?newF.at<uchar>(nj,ni):0;
			err+=fabs(opix-npix);
		}
	}
	return err;
}

inline void track(const Mat& oldF, const Mat& newF,
		const vector<Point2f>& oldX, vector<Point2f>& newX,
		vector<uchar>& status, vector<float> err, TrackParam& tp) {
#if FAST_DEBUG
	assert(oldF.rows==newF.rows && oldF.cols==newF.cols
		&& oldF.channels()==1 && newF.channels()==1);
	helper::PerformanceMeasurer PM(1000);
#endif
	int nX=(int)oldX.size();
	newX.resize(nX);
	status.resize(nX);
	err.resize(nX);
	tp.fastUp = nX*1.5; tp.fastLow = nX*1.2;
	vector<KeyPoint> curX;
#if FAST_DEBUG
	PM.tic();
#endif
	dynamicThresh(newF, curX, tp);
#if FAST_DEBUG
	loglni("[fast::track] dynamicThresh time="<<PM.toctic());
#endif

	Gridder<KeyPoint> gridder(0,0, oldF.cols, oldF.rows, tp.gridCellSize);
	for(int i=0; i<(int)curX.size(); ++i) {
		KeyPoint& kp = curX[i];
		gridder.add(kp.pt.x, kp.pt.y, &kp);
	}//fill gridder

	for(int i=0; i<nX; ++i) {
		const Point2f& op = oldX[i];
		vector<KeyPoint*> sl;
		int slsize=gridder.findAll(op.x, op.y, tp.searchRange, sl);
		int minj=-1;
		float minerr = FLT_MAX;
		for(int j=0; j<slsize; ++j) {
			const Point2f& np = sl[j]->pt;
			if( fabs(np.x - op.x) > tp.searchRange ||
				fabs(np.y - op.y) > tp.searchRange ) continue;
			float err = compare<float>(oldF, newF, op, np, tp.cmpWinSize);
			if(err<minerr) minj=j, minerr=err;
		}
		newX[i] = oldX[i];
		status[i] = minj!=-1;
		err[i] = minerr;
		if(status[i]) newX[i] = sl[minj]->pt;
	}//end of track each keypoint
#if FAST_DEBUG
	loglni("[fast::track] search time="<<PM.toc());
#endif
}

}//end of namespace fast
