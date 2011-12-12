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

#include "apriltag/apriltag.hpp"

namespace keg {

using namespace cv;
using namespace std;
using april::tag::TagFamily;
using april::tag::TagDetector;
using april::tag::TagDetection;

/**
interface for keg recognizer
*/
struct Recognizer {
	virtual void operator()(Mat &nframe,
		vector<Mat>& retH, vector<int>& retId,
		int errorThresh=0) = 0;
};

struct AprilTagRecognizer : public Recognizer {
	Ptr<TagFamily> tagFamily;
	Ptr<TagDetector> detector;

	inline void operator()(Mat &nframe,
		vector<Mat>& retH, vector<int>& retId,
		int errorThresh=0)
	{
		retH.clear(); retId.clear();
		vector<TagDetection> detections;
		double opticalCenter[2] = { nframe.cols/2.0, nframe.rows/2.0 };
		detector->process(nframe, opticalCenter, detections);

		for(int id=0; id<(int)detections.size(); ++id) {
			TagDetection &dd = detections[id];
			if(dd.hammingDistance>errorThresh) continue;

			retId.push_back(dd.id);
			cv::Mat tmp(3,3,CV_64FC1, (double*)dd.homography[0]);
			double vm[] = {1,0,dd.hxy[0],0,1,dd.hxy[1],0,0,1};
			retH.push_back( cv::Mat(3,3,CV_64FC1,vm) * tmp );
		}
	}
};

}//end of namespace keg
