#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include "Log.h"
#include "OpenCVHelper.h"

#include "ESMHelper.h"

using namespace cv;
using namespace std;

double K[9] = {
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

	ESMTracker esm; //refiner

	LKTracker(string templatename,
	          int winW=8, int winH=8,
	          int termIter=5, int termEps=0.3,
	          int maxlevel=3, double lambda=0.5,
	          double ransacT=2, int drawT=15) :
		termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,termIter,termEps),
		winSize(winW,winH), maxLevel(maxlevel), derivedLambda(lambda),
		ransacThresh(ransacT), drawTresh(drawT) {
		detector=FeatureDetector::create("SURF");
		descriptor=DescriptorExtractor::create("SURF");
		matcher=DescriptorMatcher::create("FlannBased");

		//load template
		timg = imread(templatename);
		cpts.push_back(Point2f(0,0));
		cpts.push_back(Point2f(timg.cols,0));
		cpts.push_back(Point2f(timg.cols,timg.rows));
		cpts.push_back(Point2f(0,timg.rows));
		//GaussianBlur(timg, timg, Size(5,5), 4);

		// The tracking parameters
		int miter = 6,  mprec = 4;
		int posx = 0, posy = 0;
		int sizx = timg.cols, sizy = timg.rows;

		if(!esm.init(timg,posx,posy,sizx,sizy,miter,mprec)) {
			exit(0);
		}

		detector->detect(timg, tkeys);
		descriptor->compute(timg, tkeys, tdes);

		debug = true;
	}

	inline bool init(Mat &nframe) {
		vector<KeyPoint> keys;
		Mat des;
		vector<DMatch> matches;
		detector->detect(nframe, keys);
		if(keys.size()<drawTresh) {
			return false;
		}
		descriptor->compute(nframe, keys, des);
		matcher->clear();
		matcher->match(des, tdes, matches);
		if(matches.size()<drawTresh) {
			return false;
		}
		npts.resize(matches.size());
		tpts.resize(matches.size());
		for(int i=0; i<matches.size(); ++i) {
			const DMatch &m = matches[i];
			npts[i] = keys[m.queryIdx].pt;
			tpts[i] = tkeys[m.trainIdx].pt;
		}
		H = findHomography(Mat(tpts), Mat(npts),
		                   match_mask, RANSAC, ransacThresh);
		//!!! notice, do not refine at init stage, not enough data to refine

		Mat nptsmat(npts);
		perspectiveTransform(Mat(tpts), nptsmat, H);
		if(match_mask.size()>drawTresh) {
			cornerSubPix(nframe, npts, winSize, Size(-1,-1), termcrit);
			opts.resize(npts.size());
			std::copy(npts.begin(), npts.end(), opts.begin());
		}
		return match_mask.size()>drawTresh;
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
		if(ret=(int)update(nframe)) {
			int cnt=0;
			for(int i=0; i<(int)npts.size(); ++i) {
				cnt+=(int)(status[i] && match_mask[i]
				           && withinFrame(npts[i],nframe));
			}
			ret = (int)(cnt>drawTresh);
			
			if(debug) draw(image);
		}

		std::swap(npts, opts);
		return ret;
	}

	inline bool update(Mat &nframe) {
		if(npts.size()<2*drawTresh || tpts.size()!=npts.size()) {
			return false;
		}
		match_mask.clear();
		H = findHomography(Mat(tpts), Mat(npts),
		                   match_mask, RANSAC, ransacThresh);

		//ESM refinement
		for(int i=0; i<3; i++) {
			for(int j=0; j<3; j++) {
				esm.T.homog[i*3+j] = H.at<double>(i,j);
			}
		}
		if(esm.run(nframe)) {
			for(int i=0; i<3; i++) {
				for(int j=0; j<3; j++) {
					H.at<double>(i,j) = esm.T.homog[i*3+j]/esm.T.homog[8];
				}
			}
		}

		Mat nptsmat(npts);
		///////VERY IMPORTANT STEP, STABLIZE!!!!!!!
		perspectiveTransform(Mat(tpts), nptsmat, H);

		//validate the homography
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

		return s123 && area>64 && countNonZero(Mat(match_mask)) > drawTresh;
	}

	inline void draw(Mat &image) {
		for(int i=0; i<(int)npts.size(); ++i) {
			if( status[i] ) {
				circle(image, npts[i], 2, CV_GREEN, -1);
				line(image, npts[i], opts[i], CV_BLUE );
			}
			if(match_mask[i]) {
				circle( image, npts[i], 1, CV_RED, -1);
			}
		}

		double crns[8][3] = {
			{0, 0, 0},
			{timg.cols, 0, 0},
			{timg.cols, timg.rows, 0},
			{0, timg.rows, 0},
			{timg.cols*0.4, timg.rows*0.4, timg.rows*0.5},
			{timg.cols*0.6, timg.rows*0.4, timg.rows*0.5},
			{timg.cols*0.6, timg.rows*0.6, timg.rows*0.5},
			{timg.cols*0.4, timg.rows*0.6, timg.rows*0.5},
		};
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
		double P[12],Rf[9];
		CameraHelper::RTfromKH(K,Homo,Rf,T);
		double R0[9]={0,1,0,1,0,0,0,0,-1};
		helper::mul(3,3,3,3,Rf,R0,R[0]);
		if(debug) {
			cout<<"R=\n"<<helper::PrintMat<>(3,3,R[0])<<endl;
			cout<<"T=\n"<<helper::PrintMat<>(1,3,T)<<endl;
			cout<<"norm(T)="<<sqrt(T[0]*T[0]+T[1]*T[1]+T[2]*T[2])<<endl;
		}
	}
};
