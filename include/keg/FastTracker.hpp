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

namespace fast {

using namespace cv;
using namespace std;
using namespace SearchHelper::Gridder;

struct TrackParam {
	double gridCellSize;
	double searchRange;//8
	double cmpWinSize;
	int fastThresh;
	int fastUp,fastLow;
	int fastMaxIter;
};

inline void dynamicThresh(Mat& frame, vector<KeyPoint>& keys, TrackParam& tp) {
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

inline double compare(Mat& oldF, Mat& newF,
		Point2f& ox, Point2f& nx, double winsize) {
	assert(oldF.rows==newF.rows &&
		oldF.cols==newF.cols);
	int W = oldF.cols, H = oldF.rows;
	double err=0;
	for(int i=-winsize; i<=winsize; ++i) {
		int oi=ox.x+i, ni=nx.x+i;
		for(int j=-winsize; j<=winsize; ++j) {
			int oj=ox.y+j, nj=nx.y+j;
			bool ov=(oi>=0 && oi<W)&&(oj>=0 && oj<H);
			bool nv=(ni>=0 && ni<W)&&(nj>=0 && nj<H);
			double opix = ov?oldF.at<uchar>(oj,oi):0;
			double npix = nv?newF.at<uchar>(nj,ni):0;
			err+=fabs(opix-npix);
		}
	}
	return err;
}

inline void pyrTrack(Mat& oldF, Mat& newF,
		vector<KeyPoint>& oldX, vector<KeyPoint>& newX,
		vector<bool>& status, vector<double> err, TrackParam& tp) {
	Mat coF = oldF, cnF = newF;
	newX.resize(oldX.size());
	status.resize(oldX.size());
	err.resize(oldX.size());
	tp.fastThresh=30;
	double multiplier = 1;
	for(int curlevel=0, ix=0; ix<(int)oldX.size(); ++curlevel, multiplier *= 0.5) {
		int curcnt=0, lastix=ix;
		for(;oldX[ix].octave==curlevel;++ix,++curcnt);
		tp.fastUp = curcnt*2; tp.fastLow = curcnt; tp.fastMaxIter=3*multiplier;
		vector<KeyPoint> curX;
		dynamicThresh(cnF, curX, tp);

		Gridder<KeyPoint> gridder(0,0, coF.cols, coF.rows, tp.gridCellSize*multiplier);
		for(int i=0; i<(int)curX.size(), ++i) {
			KeyPoint& kp = curX[i];
			gridder.add(kp.pt.x, kp.pt.y, &kp);
		}//fill gridder

		double csearchrange=tp.searchRange*multiplier;
		for(int i=lastix+1; i<ix; ++i) {
			KeyPoint& kp = oldX[i];
			Point2f op = kp.pt; op.x*=multiplier; op.y*=multiplier;
			vector<KeyPoint*> sl;
			int slsize=gridder.findAll(op.x, op.y, csearchrange, sl);
			int minj=-1;
			double minerr = DBL_MAX;
			for(int j=0; j<slsize; ++j) {
				KeyPoint& tkp = *sl[j];
				Point2f np = tkp.pt;
				if( fabs(np.x - op.x) > csearchrange ||
					fabs(np.y - op.y) > csearchrange ) continue;
				double err = compare(coF, cnF, op, np, tp.cmpWinSize*multiplier);
				if(diff<minerr) mingj=j, minerr=err;
			}
			newX[i] = kp;
			status[i] = minj!=-1;
			err[i] = minerr;
			if(status[i]) newX[i].pt = sl[minj]->pt;
		}//end of track each keypoint

		Mat tmp1; pyrDown(coF,tmp1); coF = tmp1;
		Mat tmp2; pyrDown(cnF,tmp2); cnF = tmp2;
	}//end curlevel
}

class Tracker {
	int fastThresh;
	Ptr<FeatureDetector> detector;
	vector<KeyPoint> keys;
public:
};//end of class fast::Tracker

}//end of namespace fast

typedef fast::Tracker FastTracker;
