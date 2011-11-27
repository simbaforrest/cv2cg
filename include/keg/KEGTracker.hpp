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

#define ESM_ORIGINAL //define this if you have original implementation of ESM
#include "esm/ESMInterface.h"

using namespace cv;
using namespace std;
using namespace esm;

namespace keg {

double defaultK[9] = {
	9.1556072719327040e+02, 0., 3.1659567931197148e+02,
	0.,	9.2300384975219845e+02, 2.8310067999512370e+02,
	0., 0., 1.
};

/**
\class keg::KeyFrame
\brief contains keyframe id, image, rotation, translation, and quality measure
*/
struct KeyFrame {
	int id;
	Mat frame;
	double R[3][3];
	double T[3];
	double rms,ncc;
};

/**
\class keg::Tracker KEGTracker.h "keg/KEGTracker.h"
\brief KLT+ESM+Global Geometric Constraint Enhancement Tracking framework

\par Coordinate_System:

\par Right-hand 3D coordinate system attached to the marker image :
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
	// for KLT
	vector<uchar> status;
	vector<float> err;
	TermCriteria termcrit;
	Size winSize;
	int maxLevel;
	double derivedLambda;
	Mat oframe; //previous gray frame

	// thresholds
	double ransacThresh;
	int inlierNumThresh;
	double nccThresh;

	vector<Point2f> opts; //old pts to track
	vector<Point2f> npts; //new pts being tracked
	vector<Point2f> tpts; //template points, get from recognizer
	vector<Point2f> cpts; //corners of template image
	Mat H; //current homography that maps tpts -> npts

	Mat timg; //template image

	//RANSAC
	vector<unsigned char> match_mask;

	// embeded detector, slow
	Ptr<FeatureDetector> detector;
	Ptr<DescriptorExtractor> descriptor;
	Ptr<DescriptorMatcher> matcher;
	vector<KeyPoint> tkeys;
	Mat tdes;

	esm::Tracker refiner;

	vector<KeyFrame> keyframes;

	bool doKstep; //whether to perform KLT
	bool doGstep; //whether to perform geometric constraint enhancement

	double K[9];

	Tracker(int winW=8, int winH=8,
	           int termIter=5, int termEps=0.3,
	           int maxlevel=3, double lambda=0.3,
	           double nccT=0.5, double ransacT=2, int validT=15) :
		termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,termIter,termEps),
		winSize(winW,winH), maxLevel(maxlevel), derivedLambda(lambda),
		ransacThresh(ransacT), inlierNumThresh(validT), nccThresh(nccT)
	{
		loadK(defaultK);
		this->H = Mat::eye(3,3,CV_64FC1);
		doGstep = true;
		doKstep = true;
		refiner.setTermCrit(5, 10);
	}

	/**
	load template image, extract SURF keypoints
	
	@param templatename file path to the template image
	*/
	inline void loadTemplate(string templatename) {
		//limit the number of keypoints to track to [40,80]
		detector=new DynamicAdaptedFeatureDetector(new SurfAdjuster, 40, 80, 10000);
		descriptor=DescriptorExtractor::create("SURF");
		matcher=DescriptorMatcher::create("FlannBased");
		//load template
		timg = imread(templatename, 0);
		cpts.push_back(Point2f(0,0));
		cpts.push_back(Point2f(timg.cols,0));
		cpts.push_back(Point2f(timg.cols,timg.rows));
		cpts.push_back(Point2f(0,timg.rows));
		//GaussianBlur(timg, timg, Size(5,5), 4);

		refiner.init(timg);

		detector->detect(timg, tkeys);
		descriptor->compute(timg, tkeys, tdes);

		//now use just surf, rather than the dynamic one
		detector = new SurfFeatureDetector;
	}

	/**
	load calibration matrix
	
	@param[in] Kmat, calibration matrix, <3x3>
	*/
	inline void loadK(double const Kmat[9]) {
		for(int i=0; i<9; ++i) {
			K[i]=Kmat[i];
		}
	}

	/**
	init by initH estimated from outside, such as an apriltag recognizer
	
	@param[in,out] gray input current frame, maybe turned into gray if RGB
	@param[in,out] init Homography estimated from outside
	@param[out] rms root-mean-square error
	@param[out] ncc normalize cross correlation
	@param[in] maxiter max number of iterations for ESM refinement, 20 by default
	@param[in] delta rms limit for ESM refinement, 0.01 by default
	@return true if init success
	*/
	inline bool init(Mat &gray, Mat &initH,
			double& rms, double & ncc,
			int maxiter=20, double deltaRMS=0.1) {
		gray.copyTo(oframe);
		//use ESM to refine and check ncc
		int miter = refiner.maxIter;
		double mprec = refiner.mprec;
		refiner.setTermCrit(maxiter,1/deltaRMS);
		refiner(gray, initH, ncc, rms);
		refiner.setTermCrit(miter,mprec);
		if(cvIsNaN(ncc) || ncc<nccThresh) return false;
		initH.copyTo(this->H);

		if(doKstep) {
			npts.resize(tkeys.size());
			tpts.resize(tkeys.size());//fill tpts by all tkeys
			for(int i=0; i<(int)tkeys.size(); ++i) {
				tpts[i] = tkeys[i].pt;
			}
			Mat nptsmat(npts);
			perspectiveTransform(Mat(tpts), nptsmat, H);
			cornerSubPix(gray, npts, winSize, Size(-1,-1), termcrit);
			opts.resize(npts.size());
			std::copy(npts.begin(), npts.end(), opts.begin());
		}
		return true;
	}

	/**
	init by SURF, very slow! only use this if you have no other recognizer
	
	@param[in,out] gray input current frame, maybe turned into gray if RGB
	@param[in,out] rms root-mean-square error
	@param[in,out] ncc normalize cross correlation
	@return true if success
	*/
	inline bool init(Mat &gray, double& rms, double& ncc) {
		gray.copyTo(oframe);
		vector<KeyPoint> keys;
		Mat des;
		vector<DMatch> matches;
		detector->detect(gray, keys);
		if((int)keys.size()<inlierNumThresh) {
			return false;
		}
		descriptor->compute(gray, keys, des);
		matcher->clear();
		matcher->match(des, tdes, matches);
		if((int)matches.size()<inlierNumThresh) {
			return false;
		}
		npts.resize(matches.size());
		tpts.resize(matches.size());
		for(int i=0; i<(int)matches.size(); ++i) {
			const DMatch &m = matches[i];
			npts[i] = keys[m.queryIdx].pt;
			tpts[i] = tkeys[m.trainIdx].pt;
		}
		H = findHomography(Mat(tpts), Mat(npts),
		                   match_mask, RANSAC, ransacThresh);
		refiner(gray, H, ncc, rms);
		if(cvIsNaN(ncc) || ncc<nccThresh) return false;

		Mat nptsmat(npts);
		perspectiveTransform(Mat(tpts), nptsmat, H);
		if((int)match_mask.size()>inlierNumThresh) {
			cornerSubPix(gray, npts, winSize, Size(-1,-1), termcrit);
			opts.resize(npts.size());
			std::copy(npts.begin(), npts.end(), opts.begin());
		}
		return (int)match_mask.size()>inlierNumThresh;
	}

	inline bool withinFrame(const Point2f &p, const Mat &frame) const {
		return p.x>=0 && p.x<frame.cols && p.y>=0 && p.y<frame.rows;
	}

	/**
	main tracking procedure
	
	@param[in,out] nframe new frame, i.e. current frame, maybe turned gray if RGB
	@param[in,out] image target image to be drawed on if valid
	@param[out] rms
	@param[out] ncc
	@return false if loss-of-track
	*/
	inline bool operator()(Mat &nframe, Mat *image, double& rms, double& ncc) {
		bool ret;
		status.clear();
		err.clear();

		//1. KLT
		if(doKstep) {
			if(opts.empty()) return false;
			if(oframe.empty()) nframe.copyTo(oframe);

			calcOpticalFlowPyrLK(oframe, nframe, opts, npts,
				                 status, err, winSize,
				                 maxLevel, termcrit, derivedLambda);
			int cnt=0;
			for(int i=0; i<(int)npts.size(); ++i) {
				//to make these points rejected by RANSAC
				npts[i] = status[i]?npts[i]:Point2f(0,0);
				cnt += status[i];
			}

			if(cnt>=inlierNumThresh) {
				//2. RANSAC, rough estimation of Homography
				match_mask.clear();
				H = findHomography(Mat(tpts), Mat(npts),
						           match_mask, RANSAC, ransacThresh);
			}
		}

		//3. ESM refinement of Homography
		refiner(nframe, H, ncc, rms);
		ret = !cvIsNaN(ncc) && ncc>nccThresh;

		if(ret) {//image similarity check passed
			//4. Global Geometric Constraint Enhancement
			if(doGstep) {
				Mat nptsmat(npts);
				perspectiveTransform(Mat(tpts), nptsmat, H);
			}

			ret=validateH(nframe);
			if(ret && image) {
				if(doKstep) drawTrail(*image);
				drawHomo(*image);
				draw3D(*image);
			}
		}

		std::swap(oframe, nframe);
		std::swap(npts, opts);
		return ret && (!doKstep || !opts.empty());
	}

	/**
	validation of estimated homography, check geometric consistency and #inlier
	FIXME : more on determining the tracking quality for re-init
	
	@param nframe current frame
	@return true if estimated homography is valid
	*/
	inline bool validateH(const Mat& nframe) const {
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

/*------ Rendering Functions ------*/
	inline void drawTrail(Mat& image) {
		//draw track trail
		for(int i=0; i<(int)npts.size(); ++i) {
			if( status[i] ) {
				circle(image, npts[i], 2, CV_GREEN, -1);
				line(image, npts[i], opts[i], CV_BLUE );
			}
			if(match_mask[i]) {
				circle( image, npts[i], 1, CV_RED, -1);
			}
		}
	}

	inline void drawHomo(Mat& image) {
		const double crns[4][2]={
				{0, timg.rows},
				{timg.cols, timg.rows},
				{timg.cols, 0},
				{0, 0}
		};
		helper::drawHomography(image, this->H, crns);
	}

	inline void draw3D(Mat &image) {
		static double crns[8][3] = {
			{0, 0, 0},
			{0, timg.cols, 0},
			{timg.rows, timg.cols, 0},
			{timg.rows, 0, 0},
			{timg.rows*0.4, timg.cols*0.4, timg.rows*0.5},
			{timg.rows*0.4, timg.cols*0.6, timg.rows*0.5},
			{timg.rows*0.6, timg.cols*0.6, timg.rows*0.5},
			{timg.rows*0.6, timg.cols*0.4, timg.rows*0.5}
		};
		//homo to P
		double R[3][3],T[3];
		GetCameraPose(R,T,false,true);
		helper::drawPyramid(image,K,R[0],T,crns);
	}

/*------ Serialization Functions ------*/
	inline void CapKeyFrame(int id, Mat const& frame,
			double const R[3][3], double const T[3],
			double const rms=NAN, double const ncc=NAN) {
		KeyFrame newkf;
		newkf.id = id;
		newkf.frame = frame.clone();
		for(int i=0; i<3; ++i) {
			newkf.T[i] = T[i];
			for(int j=0; j<3; ++j) {
				newkf.R[i][j]=R[i][j];
			}
		}
		newkf.rms = rms;
		newkf.ncc = ncc;
		keyframes.push_back(newkf);
	}

	inline bool SaveKeyFrames(string name) {
		cout<<"[SaveKeyFrames] "<<(int)keyframes.size()<<" frame(s) total!"<<endl;
		if(keyframes.size()<=0) return true;
		name = DirHelper::getFileDir(name);
		string mainname = helper::legalDir(name) + string("frames.main");
		string rmsnccname = name + string("frames.rmsncc");
		string framesRname = name + string("frames.R");
		string framesTname = name + string("frames.T");
		std::ofstream mainout(mainname.c_str());
		mainout << "CAMERAFRUSTUM "<<keyframes.size()<<" 1"<< endl;
		mainout << name << endl;
		std::ofstream rmsncc(rmsnccname.c_str());
//		rmsncc <<"#format: frame_id rms ncc"<<endl;
		std::ofstream framesR(framesRname.c_str());
//		framesR <<"#format: frame_id R[0][0] R[0][1] ... R[2][2]"<<endl;
		std::ofstream framesT(framesTname.c_str());
//		framesT <<"#format: frame_id T[0] T[1] T[2]"<<endl;
		for(int i=0; i<(int)keyframes.size(); ++i) {
			const KeyFrame& kf = keyframes[i];
			if(kf.frame.empty()) {
				continue;
			}
			string num;
			helper::num2str(kf.id,num);
			string prefix = name + string("frame") + num;
			string relativePrefix = string("frame") + num;

			string parname = prefix + string(".par");
			std::ofstream par(parname.c_str());
			par << "n=0.1\nf=10000\n" << endl;
			par << "K=\n" << helper::PrintMat<>(3,3,K) << endl;
			par << "R=\n" << helper::PrintMat<>(3,3,kf.R[0]) << endl;
			par << "T=\n" << helper::PrintMat<>(3,1,kf.T) << endl;
			par.close();
			cout<<"[SaveKeyFrames] "<<parname<<" saved."<<endl;

			string imgname = prefix + string(".jpg");
			imwrite(imgname, kf.frame);
			cout<<"[SaveKeyFrames] "<<imgname<<" saved."<<endl;

			string relativeParname = relativePrefix + string(".par");
			string relativeImgname = relativePrefix + string(".jpg");
			mainout << relativeImgname << endl;
			mainout << relativeParname << endl;

			rmsncc <<kf.id<<" "<<kf.rms<<" "<<kf.ncc<<endl;
			framesR <<kf.id<<" "<<helper::PrintMat<>(1,9,kf.R[0]);
			framesT <<kf.id<<" "<<helper::PrintMat<>(1,3,kf.T);
		}
		rmsncc.close();
		framesR.close();
		mainout.close();
		cout<<"[SaveKeyFrames] DONE!"<<endl;
		return true;
	}
};//end of struct Tracker

}//end of namespace keg

typedef keg::Tracker KEGTracker;
