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

double defaultK[9] = {
	9.1556072719327040e+02, 0., 3.1659567931197148e+02,
	0.,	9.2300384975219845e+02, 2.8310067999512370e+02,
	0., 0., 1.
};

Scalar colors[3] = {
	CV_RED,  //x
	CV_GREEN,//y
	CV_BLUE, //z
};

int lines[12][2] = {
	{0,3},{2,1},{4,7},{6,5},
	{0,1},{2,3},{4,5},{6,7},
	{0,4},{3,7},{5,1},{2,6}
};

struct KeyFrame {
	int id;
	Mat frame;
	double R[3][3];
	double T[3];
	double rms,ncc;
};

//KLT+ESM+Geometric Enhancement Tracking framework
struct KEGTracker {
	bool debug;
	vector<uchar> status;
	vector<float> err;
	TermCriteria termcrit;
	Size winSize;
	int maxLevel;
	double derivedLambda;
	double ransacThresh;
	int drawTresh;
	double nccThresh;

	vector<Point2f> opts; //old pts to track
	vector<Point2f> npts; //new pts being tracked
	vector<Point2f> tpts; //template points, get from recognizer
	vector<Point2f> cpts; //corners of template image
	Mat H; //current homography that maps tpts -> npts

	Mat timg;

	vector<unsigned char> match_mask;

	Ptr<FeatureDetector> detector;
	Ptr<DescriptorExtractor> descriptor;
	Ptr<DescriptorMatcher> matcher;
	vector<KeyPoint> tkeys;
	Mat tdes;

	esm::Refiner refiner; //refiner

	vector<KeyFrame> keyframes;

	int miter;

	double K[9];

	KEGTracker(int winW=8, int winH=8,
	           int termIter=5, int termEps=0.3,
	           int maxlevel=3, double lambda=0.3,
	           double nccT=0.1, double ransacT=2, int drawT=15) :
		termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,termIter,termEps),
		winSize(winW,winH), maxLevel(maxlevel), derivedLambda(lambda),
		ransacThresh(ransacT), drawTresh(drawT), nccThresh(nccT)
	{
		loadK(defaultK);
	}

	inline void loadTemplate(string templatename, bool isDecimate=true) {
		detector=new DynamicAdaptedFeatureDetector(new SurfAdjuster, 40, 80, 20);
		descriptor=DescriptorExtractor::create("SURF");
		matcher=DescriptorMatcher::create("FlannBased");

		//load template
		Mat tmp = imread(templatename, 0);
		if(isDecimate)
			pyrDown(tmp, timg);//scale the template down one-level, faster
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

		debug=true;
	}

	//load calibration matrix
	inline void loadK(double Kmat[9]) {
		for(int i=0; i<9; ++i) {
			K[i]=Kmat[i];
		}
	}

	//init by initH estimated from outside, such as a apriltag recognizer
	inline bool init(Mat & nframe, Mat initH) {
	}

	//FIXME, it should be recognizer's job
	inline bool init(Mat &nframe, double& rms, double& ncc) {
		vector<KeyPoint> keys;
		Mat des;
		vector<DMatch> matches;
		detector->detect(nframe, keys);
		if((int)keys.size()<drawTresh) {
			return false;
		}
		descriptor->compute(nframe, keys, des);
		matcher->clear();
		matcher->match(des, tdes, matches);
		if((int)matches.size()<drawTresh) {
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
		if((int)match_mask.size()>drawTresh) {
			cornerSubPix(nframe, npts, winSize, Size(-1,-1), termcrit);
			opts.resize(npts.size());
			std::copy(npts.begin(), npts.end(), opts.begin());
		}
		return (int)match_mask.size()>drawTresh;
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

		if((int)npts.size()<2*drawTresh || tpts.size()!=npts.size()) {
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
			if(ret && debug) {
				drawTrail(image);
				//drawHomo(image);
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
		bool ret = s123 && area>64 && countNonZero(Mat(match_mask)) > drawTresh;
		if(ret) {
			int cnt=0;
			for(int i=0; i<(int)npts.size(); ++i) {
				cnt+=(int)(status[i] && match_mask[i]
				           && withinFrame(npts[i],nframe));
			}
			ret = (int)(cnt>drawTresh);
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

	inline void drawHomo(Mat& image) {
		//draw homo
		const Mat_<double>& mH = H;
		vector<Point2f> corners(4);
		for(int i = 0; i < 4; i++ ) {
			Point2f pt((float)(i == 0 || i == 3 ? 0 : timg.cols),
			           (float)(i <= 1 ? 0 : timg.rows));
			double w = 1./(mH(2,0)*pt.x + mH(2,1)*pt.y + mH(2,2));
			corners[i] =
			    Point2f((float)((mH(0,0)*pt.x + mH(0,1)*pt.y + mH(0,2))*w),
			            (float)((mH(1,0)*pt.x + mH(1,1)*pt.y + mH(1,2))*w));
		}
		for(int i = 0; i < 4; ++i) {
			Point r1 = corners[i%4];
			Point r2 = corners[(i+1)%4];
			line( image, r1, r2, CV_GREEN, 2 );
		}
		line(image, corners[0], corners[2], CV_BLACK, 2);
		line(image, corners[1], corners[3], CV_BLACK, 2);
	}

	inline void draw3D(Mat &image) {
		static double crns[8][3] = {
			{0, 0, 0},
			{timg.cols, 0, 0},
			{timg.cols, timg.rows, 0},
			{0, timg.rows, 0},
			{timg.cols*0.4, timg.rows*0.4, timg.rows*0.5},
			{timg.cols*0.6, timg.rows*0.4, timg.rows*0.5},
			{timg.cols*0.6, timg.rows*0.6, timg.rows*0.5},
			{timg.cols*0.4, timg.rows*0.6, timg.rows*0.5},
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
				Point r1(p[lines[j][0]][0],p[lines[j][0]][1]);
				Point r2(p[lines[j][1]][0],p[lines[j][1]][1]);
				line( image, r1, r2, colors[i], 2 );
			}
		}
	}

	inline void GetCameraPose(double R[3][3], double T[3]) {
		double Homo[9];
		std::copy(H.begin<double>(), H.end<double>(), Homo);
		double Rf[9];
		CameraHelper::RTfromKH(K,Homo,Rf,T);
		double R0[9]= {0,1,0,1,0,0,0,0,-1};
		helper::mul(3,3,3,3,Rf,R0,R[0]);
		if(debug) {
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
		string swtname = name + string("ar.swt");
		string rmsnccname = name + string("rmsncc.txt");
		std::ofstream swt(swtname.c_str());
		swt << "scene.osg" << endl;
		cerr<<"[SaveKeyFrames] "<<(int)keyframes.size()<<" frame(s) total!"<<endl;
		std::ofstream rmsncc(rmsnccname.c_str());
		rmsncc <<"#format: frame_id rms ncc"<<endl;
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
			par << "n=0.1\nf=1000000\n" << endl;
			par << "K(alphaX alphaY u0 v0)=\n"
			    <<K[0]<<"\n"<<K[4]<<"\n"
			    <<K[2]<<"\n"<<K[5]<< endl;
			par << "R=\n" << helper::PrintMat<>(3,3,kf.R[0]) << endl;
			par << "T=\n" << helper::PrintMat<>(3,1,kf.T) << endl;
			par.close();
			cout<<"[SaveKeyFrames] "<<parname<<" saved."<<endl;

			string imgname = prefix + string(".jpg");
			imwrite(imgname, kf.frame);
			cout<<"[SaveKeyFrames] "<<imgname<<" saved."<<endl;

			string relativeParname = relativePrefix + string(".par");
			string relativeImgname = relativePrefix + string(".jpg");
			swt << relativeImgname << endl;
			swt << relativeParname << endl;

			rmsncc <<kf.id<<" "<<kf.rms<<" "<<kf.ncc<<endl;
		}
		cerr<<"[SaveKeyFrames] DONE!"<<endl;
		return true;
	}
};//end of struct KEGTracker
