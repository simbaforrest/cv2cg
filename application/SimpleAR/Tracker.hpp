/*
 *  Copyright (c) 2010  Chen Feng (cforrest (at) umich.edu)
 *    and the University of Michigan
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include "Log.h"
#include "OpenCVHelper.h"

#include "esm.hpp"

using namespace cv;
using namespace std;

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
	Mat frame;
	double R[3][3];
	double T[3];
};

struct LKTracker {
	bool debug;
	vector<uchar> status;
	vector<float> err;
	TermCriteria termcrit;
	Size winSize;
	int maxLevel;
	double derivedLambda;
	double ransacThresh;
	int drawTresh;

	vector<Point2f> opts; //old pts to track
	vector<Point2f> npts; //new pts being tracked
	vector<Point2f> tpts; //template points, get from recognizer
	vector<Point2f> cpts; //corners of template image
	Mat H; //current homography that maps tpts -> npts

	Mat timg;

	vector<unsigned char> match_mask;

	Ptr<FeatureDetector> detector; //FIXME, it should be recognizer's job
	Ptr<DescriptorExtractor> descriptor;
	Ptr<DescriptorMatcher> matcher;
	vector<KeyPoint> tkeys;
	Mat tdes;

	HomoESM esm; //refiner

	vector<KeyFrame> keyframes;
	
	int miter;

	double K[9];

	LKTracker(int winW=8, int winH=8,
	          int termIter=5, int termEps=0.3,
	          int maxlevel=3, double lambda=0.3,
	          double ransacT=2, int drawT=15) :
		termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,termIter,termEps),
		winSize(winW,winH), maxLevel(maxlevel), derivedLambda(lambda),
		ransacThresh(ransacT), drawTresh(drawT) {}

	void loadTemplate(string templatename) {
		detector=new DynamicAdaptedFeatureDetector(new SurfAdjuster, 40, 80, 20);
		descriptor=DescriptorExtractor::create("SURF");
		matcher=DescriptorMatcher::create("FlannBased");

		//load template
		Mat tmp = imread(templatename, 0);
		pyrDown(tmp, timg);//scale the template to 256x256, faster
		cpts.push_back(Point2f(0,0));
		cpts.push_back(Point2f(timg.cols,0));
		cpts.push_back(Point2f(timg.cols,timg.rows));
		cpts.push_back(Point2f(0,timg.rows));
		//GaussianBlur(timg, timg, Size(5,5), 4);

		// The tracking parameters
		miter = 5;//  mprec = 4;
//		int posx = 0, posy = 0;
//		int sizx = timg.cols, sizy = timg.rows;

		esm.setTemplateImage(timg);
		esm.setDeltaRMSLimit(0.5);
//		if(!esm.init(timg,posx,posy,sizx,sizy,miter,mprec)) {
//			exit(0);
//		}

		detector->detect(timg, tkeys);
		descriptor->compute(timg, tkeys, tdes);

		debug=true;
	}

	//load calibration matrix
	inline void loadK(double Kmat[9]) {
		for(int i=0; i<9; ++i) K[i]=Kmat[i];
	}

	inline bool init(Mat &nframe) {
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
		//!!! notice, do not refine at init stage, not enough data to refine
		double rms;
		esm.track(nframe, miter, H, rms);
		cout<<"[HomoESM] rms="<<rms<<endl;

		Mat nptsmat(npts);
		perspectiveTransform(Mat(tpts), nptsmat, H);
		if((int)match_mask.size()>drawTresh) {
			cornerSubPix(nframe, npts, winSize, Size(-1,-1), termcrit);
			opts.resize(npts.size());
			std::copy(npts.begin(), npts.end(), opts.begin());
		}
		return (int)match_mask.size()>drawTresh;
	}

	inline bool withinFrame(const Point2f &p, const Mat &frame) {
		return p.x>=0 && p.x<frame.cols && p.y>=0 && p.y<frame.rows;
	}

	//tracking, old frame(oframe), new frame(nframe)
	inline int operator()(Mat &oframe, Mat &nframe, Mat &image) {
		status.clear();
		err.clear();

		calcOpticalFlowPyrLK(oframe, nframe, opts, npts,
		                     status, err, winSize,
		                     maxLevel, termcrit, derivedLambda);

		for(int i=0; i<(int)npts.size(); ++i) {
			//to make these points rejected by RANSAC
			npts[i] = status[i]?npts[i]:Point2f(0,0);
		}

		int ret; //FIXME : more on determining the tracking quality for re-init
		if( ( ret=(int)update(nframe) ) ) {
			int cnt=0;
			for(int i=0; i<(int)npts.size(); ++i) {
				cnt+=(int)(status[i] && match_mask[i]
				           && withinFrame(npts[i],nframe));
			}
			ret = (int)(cnt>drawTresh);
			
			if(debug) {
				drawTrail(image);
				drawHomo(image);
				draw3D(image);
			}
		}

		std::swap(npts, opts);
		return ret;
	}

	inline bool update(Mat &nframe) {
		if((int)npts.size()<2*drawTresh || tpts.size()!=npts.size()) {
			return false;
		}
		match_mask.clear();
		H = findHomography(Mat(tpts), Mat(npts),
		                   match_mask, RANSAC, ransacThresh);

		//ESM refinement
		double rms;
		esm.track(nframe, miter, H, rms);
		cout<<"[ESM] rms="<<rms<<endl;

		Mat nptsmat(npts);
		///////VERY IMPORTANT STEP, STABLIZE!!!!!!!
		perspectiveTransform(Mat(tpts), nptsmat, H);

		return validateH() && countNonZero(Mat(match_mask)) > drawTresh;
	}

	inline bool validateH() const {
		vector<Point2f> tmp(4);
		Mat tmpmat(tmp);
		perspectiveTransform(Mat(cpts), tmpmat, H);
		double tmpdir[3][2]={
			{tmp[1].x-tmp[0].x, tmp[1].y-tmp[0].y},
			{tmp[2].x-tmp[0].x, tmp[2].y-tmp[0].y},
			{tmp[3].x-tmp[0].x, tmp[3].y-tmp[0].y}
		};
		double d12 = helper::cross2D(tmpdir[0],tmpdir[1]) * 0.5;
		double d23 = helper::cross2D(tmpdir[1],tmpdir[2]) * 0.5;
		double area = abs(d12+d23);
		bool s123 = d12*d23>0;
		return s123 && area>64;
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
		double R0[9]={0,1,0,1,0,0,0,0,-1};
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
		double R0[9]={0,1,0,1,0,0,0,0,-1};
		helper::mul(3,3,3,3,Rf,R0,R[0]);
		if(debug) {
			cout<<"R=\n"<<helper::PrintMat<>(3,3,R[0])<<endl;
			cout<<"T=\n"<<helper::PrintMat<>(1,3,T)<<endl;
			cout<<"norm(T)="<<sqrt(T[0]*T[0]+T[1]*T[1]+T[2]*T[2])<<endl;
		}
	}

	void CapKeyFrame(Mat& frame, double R[3][3], double T[3]) {
		KeyFrame newkf;
		newkf.frame = frame.clone();
		for(int i=0; i<3; ++i) {
			newkf.T[i] = T[i];
			for(int j=0; j<3; ++j)
				newkf.R[i][j]=R[i][j];
		}
		keyframes.push_back(newkf);
	}

	bool SaveKeyFrames(string name) {
		string swtname = name + string("ar.swt");
		std::ofstream swt(swtname.c_str());
		swt << "scene.osg" << endl;
		for(int i=0; i<(int)keyframes.size(); ++i) {
			const KeyFrame& kf = keyframes[i];
			string num;
			helper::num2str(i,num);
			string prefix = name + string("frame") + num;
			
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

			swt << imgname << endl;
			swt << parname << endl;
		}
		return true;
	}
};//end of struct LKTracker
