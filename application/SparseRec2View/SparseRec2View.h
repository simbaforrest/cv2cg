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


/* SparseRec2View.h */

//standard include
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "Log.h"
#include "OpenCVHelper.h"

using namespace std;
using namespace cv;

class SparseRec2View {
private:
	FeatureDetector* detector;
	DescriptorExtractor* descriptor;
	DescriptorMatcher* matcher;

	//img1 as train image, img2 as query image
	string imgpath1, imgpath2;
	string dir;
	string imgname1, imgname2;
	Mat img1, img2;
	Mat igrey1, igrey2;
	Mat combined;
	vector<KeyPoint> key1, key2;
	vector<DMatch> matches;
	vector<Point3f> results;//reconstructed points
	vector<Point2f> p1, p2;//matched image points
	vector<uchar> inliers;//inliers in p1-p2 for fmatrix, 0 means outlier
	int inliersNum;
	double K[9];
	double R[9],t[3];//for image 2
	double F[9];
	double lamda;//restrict baseline length
	bool _onlymatch;
private:
	bool loadImage();
	bool detect();
	bool match();
	bool fmatrix();//estimate fundamental matrix
	bool estimateRelativePose();

public:
	SparseRec2View(
		string ipath1, //left image path
		string ipath2, //right image path
		double k[9], //calibration matrix
		double lamda_, //controls the reconstructed scene scale
		bool onlymatch //only perform match, no reconstruction
	);
	~SparseRec2View();

	bool run();
	bool save();
};
