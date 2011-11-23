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

#include "esm/esm.hpp"

using namespace cv;
using namespace std;
using namespace esm;

namespace keg {

double defaultK[9] = {
	9.1556072719327040e+02, 0., 3.1659567931197148e+02,
	0.,	9.2300384975219845e+02, 2.8310067999512370e+02,
	0., 0., 1.
};

struct KeyFrame {
	int id;
	Mat frame;
	double R[3][3];
	double T[3];
	double rms,ncc;
};

/** KLT+ESM+Geometric Enhancement Tracking framework
Coordinate_System:
Right-hand 3D coordinate system attached to the marker image :
	origin: left-top corner of the image
	x-axis: top to bottom of marker image
	y-axis: left to right of marker image
	z-axis: marker image to face
Right-hand 2D image coordinate system (used when estimate homography) :
	origin: left-top corner of the image
	x-axis: left to right of marker image
	y-axis: top to bottom of marker image
ATTENTION:
	2D coordinate system and 3D coordinate system's difference!
*/
struct Tracker {
	bool doDraw;
	//// for KLT
	vector<uchar> status;
	vector<float> err;
	TermCriteria termcrit;
	Size winSize;
	int maxLevel;
	double derivedLambda;
	/// thresholds
	double ransacThresh;
	int validInlierThresh;
	double nccThresh;

	vector<Point2f> opts; //old pts to track
	vector<Point2f> npts; //new pts being tracked
	vector<Point2f> tpts; //template points, get from recognizer
	vector<Point2f> cpts; //corners of template image
	Mat H; //current homography that maps tpts -> npts

	Mat timg;

	vector<unsigned char> match_mask;

	//// embeded detector, slow
	Ptr<FeatureDetector> detector;
	Ptr<DescriptorExtractor> descriptor;
	Ptr<DescriptorMatcher> matcher;
	vector<KeyPoint> tkeys;
	Mat tdes;

	esm::Refiner refiner; //refiner

	vector<KeyFrame> keyframes;

	int miter;

	double K[9];

	Tracker(int winW=8, int winH=8,
	           int termIter=5, int termEps=0.3,
	           int maxlevel=3, double lambda=0.3,
	           double nccT=0.2, double ransacT=2, int validT=15) :
		termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,termIter,termEps),
		winSize(winW,winH), maxLevel(maxlevel), derivedLambda(lambda),
		ransacThresh(ransacT), validInlierThresh(validT), nccThresh(nccT)
	{
		loadK(defaultK);
		this->H = Mat::eye(3,3,CV_64FC1);
	}

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

		// The tracking parameters
		miter = 5;//  mprec = 4;

		refiner.setTemplateImage(timg);
		refiner.setDeltaRMSLimit(0.5);

		detector->detect(timg, tkeys);
		descriptor->compute(timg, tkeys, tdes);

		//now use just surf, rather than the dynamic one
		detector = new SurfFeatureDetector;
		doDraw=true;
	}

	//load calibration matrix
	inline void loadK(double Kmat[9]) {
		for(int i=0; i<9; ++i) {
			K[i]=Kmat[i];
		}
	}

	//init by initH estimated from outside, such as an apriltag recognizer
	inline bool init(Mat &image, Mat &nframe, Mat initH, double& rms, double & ncc) {
		//use ESM to check ncc
		refiner.setDeltaRMSLimit(0.01);
		refiner.track(nframe, 20, initH, rms, ncc);
		refiner.setDeltaRMSLimit(0.5);
		if(cvIsNaN(ncc) || ncc<0.5) return false;
		initH.copyTo(this->H);

		npts.resize(tkeys.size());
		tpts.resize(tkeys.size());//fill tpts by all tkeys
		for(int i=0; i<(int)tkeys.size(); ++i) {
			tpts[i] = tkeys[i].pt;
		}
		Mat nptsmat(npts);
		perspectiveTransform(Mat(tpts), nptsmat, H);
		cornerSubPix(nframe, npts, winSize, Size(-1,-1), termcrit);
		opts.resize(npts.size());
		std::copy(npts.begin(), npts.end(), opts.begin());
		return true;
	}

	//init by SURF, very slow!!! only use this if you have no other recognizer
	inline bool init(Mat &nframe, double& rms, double& ncc) {
		vector<KeyPoint> keys;
		Mat des;
		vector<DMatch> matches;
		detector->detect(nframe, keys);
		if((int)keys.size()<validInlierThresh) {
			return false;
		}
		descriptor->compute(nframe, keys, des);
		matcher->clear();
		matcher->match(des, tdes, matches);
		if((int)matches.size()<validInlierThresh) {
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
		refiner.track(nframe, miter, H, rms, ncc);
		if(cvIsNaN(ncc) || ncc<=nccThresh) return false;

		Mat nptsmat(npts);
		perspectiveTransform(Mat(tpts), nptsmat, H);
		if((int)match_mask.size()>validInlierThresh) {
			cornerSubPix(nframe, npts, winSize, Size(-1,-1), termcrit);
			opts.resize(npts.size());
			std::copy(npts.begin(), npts.end(), opts.begin());
		}
		return (int)match_mask.size()>validInlierThresh;
	}

	inline bool withinFrame(const Point2f &p, const Mat &frame) const {
		return p.x>=0 && p.x<frame.cols && p.y>=0 && p.y<frame.rows;
	}

	//tracking, old frame(oframe), new frame(nframe)
	//return 0 if loss-of-track
	inline int operator()(Mat &oframe, Mat &nframe, Mat &image,
                          double& rms, double& ncc) {
		int ret;
		status.clear();
		err.clear();

		calcOpticalFlowPyrLK(oframe, nframe, opts, npts,
		                     status, err, winSize,
		                     maxLevel, termcrit, derivedLambda);

		for(int i=0; i<(int)npts.size(); ++i) {
			//to make these points rejected by RANSAC
			npts[i] = status[i]?npts[i]:Point2f(0,0);
		}

		if((int)npts.size()<2*validInlierThresh || tpts.size()!=npts.size()) {
			return false;
		}

		//RANSAC
		match_mask.clear();
		H = findHomography(Mat(tpts), Mat(npts),
		                   match_mask, RANSAC, ransacThresh);

		//ESM refinement
		refiner.track(nframe, miter, H, rms, ncc);
		ret = !cvIsNaN(ncc) && ncc>nccThresh;

		if(ret) {//image similarity check passed
			//Geometric Enhancement, VERY IMPORTANT STEP, STABLIZE!!!
			Mat nptsmat(npts);
			perspectiveTransform(Mat(tpts), nptsmat, H);

			//validation
			ret=validateH(nframe); //TODO do we still need this?
			if(ret && doDraw) {
				drawTrail(image);
				drawHomo(image, H);
				draw3D(image);
			}
		}

		std::swap(npts, opts);
		return ret;
	}

	//FIXME : more on determining the tracking quality for re-init
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
		bool ret = s123 && area>64 && countNonZero(Mat(match_mask)) > validInlierThresh;
		if(ret) {
			int cnt=0;
			for(int i=0; i<(int)npts.size(); ++i) {
				cnt+=(int)(status[i] && match_mask[i]
				           && withinFrame(npts[i],nframe));
			}
			ret = (int)(cnt>validInlierThresh);
		}
		return ret;
	}

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

	inline void drawHomo(Mat& image, const Mat& Homo) {
		const double crns[4][2]={
				{0, timg.rows},
				{timg.cols, timg.rows},
				{timg.cols, 0},
				{0, 0}
		};
		helper::drawHomography(image, Homo, crns);
	}

	inline void draw3D(Mat &image) {
		static cv::Scalar linecolors[] = {
			CV_BLACK,
			CV_GREEN,
			CV_BLUE,
			CV_RED  };
		static int lineidx[12][2] = {
			{0,1},{1,2},{2,3},{3,0},
			{4,5},{5,6},{6,7},{7,4},
			{0,4},{1,5},{2,6},{3,7} };
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
		double Homo[9];
		std::copy(H.begin<double>(), H.end<double>(), Homo);
		double R[9],T[3],P[12],Rf[9];
		CameraHelper::RTfromKH(K,Homo,R,T);
		double R0[9]= {0,1,0,1,0,0,0,0,-1};
		helper::mul(3,3,3,3,R,R0,Rf);
		CameraHelper::compose(K,Rf,T,P,false);
		double p[8][2];
		for(int i=0; i<8; ++i) {
			CameraHelper::project(P,crns[i],p[i]);
		}
		for(int i=0; i<3; ++i) {
			for(int j=i*4; j<4+i*4; ++j) {
				int s=lineidx[j][0], e=lineidx[j][1];
				Point r1(p[s][0],p[s][1]);
				Point r2(p[e][0],p[e][1]);
				line( image, r1, r2, linecolors[j%4], 2 );
			}
		}
	}

	inline void GetCameraPose(double R[3][3], double T[3],
		bool verbose=true, bool doAR=false) {
		double Homo[9];
		std::copy(H.begin<double>(), H.end<double>(), Homo);
		CameraHelper::RTfromKH(K,Homo,R[0],T);
		if(doAR) { // R = R * [0,1,0;1,0,0;0,0,-1]
			std::swap(R[0][0],R[0][1]);
			std::swap(R[1][0],R[1][1]);
			std::swap(R[2][0],R[2][1]);
			R[0][2]=-R[0][2];
			R[1][2]=-R[1][2];
			R[2][2]=-R[2][2];
		}
		if(verbose) {
			cout<<"R=\n"<<helper::PrintMat<>(3,3,R[0])<<endl;
			cout<<"T=\n"<<helper::PrintMat<>(1,3,T)<<endl;
		}
	}

	inline void CapKeyFrame(int id, Mat& frame, double R[3][3], double T[3],
                     double rms=-1, double ncc=-1) {
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
		name = DirHelper::getFileDir(name);
		string mainname = helper::legalDir(name) + string("frames.main");
		string rmsnccname = name + string("frames.rmsncc");
		string framesRname = name + string("frames.R");
		string framesTname = name + string("frames.T");
		std::ofstream mainout(mainname.c_str());
		mainout << "CAMERAFRUSTUM "<<keyframes.size()<<" 1"<< endl;
		mainout << name << endl;
		cerr<<"[SaveKeyFrames] "<<(int)keyframes.size()<<" frame(s) total!"<<endl;
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
		cerr<<"[SaveKeyFrames] DONE!"<<endl;
		return true;
	}
};//end of struct Tracker

}//end of namespace keg

typedef keg::Tracker KEGTracker;
