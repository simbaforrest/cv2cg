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

#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include "Log.h"
#include "OpenCVHelper.h"

#ifndef USE_DYNAMIC_PYRAMID_FAST // 0 - Surf; 1 - FAST
	#define USE_DYNAMIC_PYRAMID_FAST 0 //not a big deal on PC
#endif

#include "KLTTracker.hpp"
#include "esm/ESMInterface.h"
#include "KeyFrame.hpp"
#include "Recognizer.hpp"

namespace keg {

using namespace cv;
using namespace std;

double defaultK[9] = {
	9.1556072719327040e+02, 0., 3.1659567931197148e+02,
	0.,	9.2300384975219845e+02, 2.8310067999512370e+02,
	0., 0., 1.
};

/**
\class keg::Tracker KEGTracker.h "keg/KEGTracker.h"
\brief KLT+ESM+Global Geometric Constraint Enhancement Tracking framework

\par Recognizer must implement the following function:
void recognize(Mat &nframe, vector<Mat>& retH, vector<int>& retId,int errorThresh)
which takes a gray image nframe as input and output recognized markers' IDs and
corresponding Hs.

\par Coordinate System:

\par Right-hand 3D coordinate system attached to the marker image (used to render AR):
origin: left-top corner of the image
x-axis: top to bottom of marker image
y-axis: left to right of marker image
z-axis: marker image to face

\par Right-hand 2D image coordinate system (used when estimate homography) :
origin: left-top corner of the image
x-axis: left to right of marker image
y-axis: top to bottom of marker image

\attention {2D coordinate system and 3D coordinate system's difference!}
*/
struct Tracker {
public:
	struct TemplateData {
		int id; //correspond to which april tag
		vector<KeyPoint> keys;
		vector<Point2f> X; //template points, get from recognizer/detector
		vector<Point2f> crns; //corners of template image
		Mat img; //template image, CV_8UC1
		Mat iHI; //inverse of init Homography, X_tag = iHI * X_keg
	};
	vector<TemplateData> tdata;
	int curT; //current template being tracked

	klt::Tracker roughTracker;
	esm::Tracker refiner;
	double nccThresh;

	Mat H; //current homography that maps template points -> current points
	double K[9];

	bool doKstep; //whether to perform KLT
	bool doGstep; //whether to perform geometric constraint enhancement

	inline void doEstep(bool val) {
		if(val) refiner.setTermCrit(5,4);
		else refiner.setTermCrit(0,1);//no refine, only quality measure
	}

public:
	Tracker(double nccT=0.5) {
		nccThresh = nccT;
		loadK(defaultK);
		this->H = Mat::eye(3,3,CV_64FC1);
		doGstep = true;
		doKstep = true;
		refiner.setTermCrit(5, 4);
		curT = -1;
	}

	/**
	load a single template image, extract keypoints using SURF/FAST
	
	@param templatename file path to the template image
	@param threshLow lower bound for #template points
	@param threshUp upper bound for #template points
	*/
	inline void loadTemplate(string templatename,
		int threshLow=800, int threshUp=1000)
	{
		cout<<"[KEG] loading: "<<templatename<<endl;
		tdata.push_back(TemplateData());
		TemplateData& td = tdata.back();
		//load template
		td.img = imread(templatename, 0);
		td.crns.push_back(Point2f(0,0));
		td.crns.push_back(Point2f(td.img.cols,0));
		td.crns.push_back(Point2f(td.img.cols,td.img.rows));
		td.crns.push_back(Point2f(0,td.img.rows));
		//GaussianBlur(td.img, td.img, Size(5,5), 4);

		//limit the number of keypoints to track to [800,1000]
		const int mitr = 100000;
		Ptr<FeatureDetector> detector;
#if USE_DYNAMIC_PYRAMID_FAST
		int fastThresh=15;
		for(int i=0; i<mitr; ++i) {
			detector=new PyramidAdaptedFeatureDetector(
				new FastFeatureDetector(fastThresh) );
			detector->detect(td.img, td.keys);

			int curNum = (int)td.keys.size();
			if(curNum<threshLow) --fastThresh;
			else if(curNum>threshUp) ++fastThresh;
			else break;
		}
		cout<<"[KEG] Final fastThresh="<<fastThresh<<endl;
#else //use dynamic surf
		detector=new cv::SurfFeatureDetector();
		//detector=new DynamicAdaptedFeatureDetector(
		//	new SurfAdjuster, threshLow, threshUp, mitr);
		detector->detect(td.img, td.keys);
#endif //end of USE_DYNAMIC_PYRAMID_FAST
		cout<<"[KEG] #template keypoints="<<td.keys.size()<<endl;

		td.X.resize(td.keys.size());//fill tpts by all td.keys
		for(int i=0; i<(int)td.keys.size(); ++i) {
			td.X[i] = td.keys[i].pt;
		}
	}

	/**
	load calibration matrix
	
	@param[in] Kmat, calibration matrix, <3x3>
	*/
	inline void loadK(double const Kmat[9]) {
		for(int i=0; i<9; ++i) K[i]=Kmat[i];
	}

	/**
	init by initH estimated from outside, such as an apriltag recognizer
	
	@param[in,out] nframe input current frame, maybe turned into gray if RGB
	@param[in,out] init Homography estimated from outside
	@param[out] rms root-mean-square error
	@param[out] ncc normalize cross correlation
	@param[in] current template id, 0 by default
	@param[in] maxiter max number of iterations for ESM refinement, 20 by default
	@param[in] delta rms limit for ESM refinement, 0.001 by default
	@return true if init success
	*/
	inline bool init(Mat &nframe, Mat &initH,
			double& rms, double & ncc,
			int tid=0,
			int maxiter=20, double deltaRMS=0.001)
	{
		if(tid<0 || tid >= (int)tdata.size()) return false;
		const TemplateData& td = tdata[tid];
		//init esm only if different template is found
		if(curT!=tid) refiner.init(td.img);

		//use ESM to refine and check ncc
		int miter = refiner.maxIter;
		double mprec = refiner.mprec;
		refiner.setTermCrit(maxiter,0.01/deltaRMS);
		refiner(nframe, initH, ncc, rms);
		refiner.setTermCrit(miter,mprec);
		if(cvIsNaN(ncc) || ncc<nccThresh) return false;
		curT = tid;
		initH.copyTo(this->H);

		if(doKstep) roughTracker.init(nframe, td.X, H);
		return true;
	}

	/**
	main tracking framework
	
	@param[in,out] nframe new frame, i.e. current frame, maybe turned gray if RGB
	@param[out] rms
	@param[out] ncc
	@param[in,out] image visualization target, default 0 means do not draw
	@return false if loss-of-track
	*/
	inline bool operator()(Mat &nframe, double& rms, double& ncc, Mat *image=0) {
		if(curT<0 || curT>=(int)tdata.size()) return false;
		const TemplateData& td = tdata[curT];
		bool ret;

		//1. KLT + RANSAC
		if(doKstep) roughTracker(nframe, td.X, H, image);

		//2. ESM refinement of Homography
		refiner(nframe, H, ncc, rms);
		ret = !cvIsNaN(ncc) && ncc>nccThresh;

		if(ret) {//image similarity check passed
			//3. Global Geometric Constraint Enhancement
			if(doGstep)
				roughTracker.globalGeometricConstraintEhancement(td.X, H);

			ret=validateH(td.crns);
		}

		if(ret && image) { //visualization
			drawHomo(*image);
			draw3D(*image);
		}

		roughTracker.update(nframe);
		return ret;
	}

protected: //internal helper functions
	/**
	validation of estimated homography, check geometric consistency
	
	@param[in] cpts 4 corner points on template image
	@return true if estimated homography is valid
	*/
	inline bool validateH(const vector<Point2f>& cpts) const {
		vector<Point2f> tmp(4);
		Mat tmpmat(tmp);
		perspectiveTransform(Mat(cpts), tmpmat, H);
		double tmpdir[3][2]= {
			{tmp[1].x-tmp[0].x, tmp[1].y-tmp[0].y},
			{tmp[2].x-tmp[0].x, tmp[2].y-tmp[0].y},
			{tmp[3].x-tmp[0].x, tmp[3].y-tmp[0].y}
		};
		double d12 = helper::cross2D(tmpdir[0],tmpdir[1]) * 0.5;
		double d23 = helper::cross2D(tmpdir[1],tmpdir[2]) * 0.5;
		double area = abs(d12+d23);
		bool s123 = d12*d23>0;
		return s123 && area>=64;
	}

public:
	/**
	get current camera pose, i.e. rotation and translation
	
	@param[out] R [3x3] rotation matrix
	@param[out] T [3x1] translation vector, P = K*[R,T]
	@param[in] verbose output to console or not
	@param[in] doAR rotate properly for AR applications
	*/
	inline void GetCameraPose(double R[3][3], double T[3],
		bool verbose=true, bool doAR=false) {
		double Homo[9];
		std::copy(H.begin<double>(), H.end<double>(), Homo);
		if(doAR) { // R = R * [0,1,0;1,0,0;0,0,-1]
			double tmp[9];
			CameraHelper::RTfromKH(K,Homo,tmp,T);
			double R1[9]= {0,1,0,1,0,0,0,0,-1};
			helper::mul(3,3,3,3,tmp,R1,R[0]);
		} else {
			CameraHelper::RTfromKH(K,Homo,R[0],T);
		}
		if(verbose) {
			cout<<"R=\n"<<helper::PrintMat<>(3,3,R[0])<<endl;
			cout<<"T=\n"<<helper::PrintMat<>(1,3,T)<<endl;
		}
	}

	inline void drawHomo(Mat& image) {
		const TemplateData& td = tdata[curT];
		double crns[4][2]={
				{0, td.img.rows},
				{td.img.cols, td.img.rows},
				{td.img.cols, 0},
				{0, 0}
		};
		helper::drawHomography(image, this->H, crns);
	}

	inline void draw3D(Mat &image) {
		const TemplateData& td = tdata[curT];
		double crns[8][3] = {
			{0, 0, 0},
			{0, td.img.cols, 0},
			{td.img.rows, td.img.cols, 0},
			{td.img.rows, 0, 0},
			{td.img.rows*0.4, td.img.cols*0.4, td.img.rows*0.5},
			{td.img.rows*0.4, td.img.cols*0.6, td.img.rows*0.5},
			{td.img.rows*0.6, td.img.cols*0.6, td.img.rows*0.5},
			{td.img.rows*0.6, td.img.cols*0.4, td.img.rows*0.5}
		};
		//homo to P
		double R[3][3],T[3];
		GetCameraPose(R,T,false,true);
		helper::drawPyramid(image,K,R[0],T,crns);
	}
};//end of struct Tracker

}//end of namespace keg

typedef keg::Tracker KEGTracker;
