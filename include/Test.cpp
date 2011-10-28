#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>
#include <list>
#include <vector>

#include "OpenCVHelper.h"
#include "Log.h"

#include "ESMHelper.h"

using namespace std;
using namespace cv;

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

struct Detractor {
	bool debug;

	vector<Point2f> train_pts, query_pts;
	vector<KeyPoint> train_kpts, query_kpts;

	FastFeatureDetector detector;

	Mat H;

	Mat timg;

	ESMTracker esm; //refiner

	Detractor() : detector(20, true) {
		debug=true;
		H = Mat::eye(3, 3, CV_32FC1);
	}

	bool runOn(Mat &frame) {
		Mat gray;
		cvtColor(frame, gray, CV_RGB2GRAY);
		detector.detect(gray, query_kpts);

		//ESM refinement
		if(!esm.run(gray)) {
			randomInit(gray);
		}
		esm.getH(H.begin<double>());

		drawHomo(frame);
//					drawPyramid(frame);
		drawKey(frame);

		return true;
	}

	void randomInit(Mat &gray) {
		int id = rand() % (int)query_kpts.size();
		int miter = 3,  mprec = 2;
		int posx = query_kpts[id].pt.x, posy = query_kpts[id].pt.y;
		int sizx = 100, sizy = 100;
		if( posx<105 || posx>gray.cols-105 || posy<105 || posy>gray.rows-105) {
			return;
		}
		cout<<gray.cols<<"x"<<gray.rows<<":"<<posx<<" "<<posy<<" "<<sizx<<" "<<sizy<<endl;
		esm.init(gray,posx,posy,sizx,sizy,miter,mprec);
		double H[]= {1,0,0,0,1,0,0,0,1};
		esm.setH(H);
	}

	void drawPyramid(Mat &image) {
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

	void drawHomo(Mat &image) {
		//draw homo
//		for(int k=0; k<(int)esm.kArr.size(); ++k) {
//			ESMKernel& kernel = esm.kArr[k];
//			kernel.getH(H.begin<double>());

		const Mat_<double>& mH = H;
		vector<Point2f> corners(4);
		for(int i = 0; i < 4; i++ ) {
			Point2f pt((float)(i == 0 || i == 3 ? esm.kernel.px : esm.kernel.px+esm.kernel.sx),
			           (float)(i <= 1 ? esm.kernel.py : esm.kernel.py+esm.kernel.sy));
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
//		}
	}

	void drawKey(Mat &image) {
		for(int i=0; i<(int)query_kpts.size(); ++i) {
			circle(image, query_kpts[i].pt, 2, CV_BLUE, -1);
		}
	}
};

int main(int ac, char **av)
{
	if (ac != 2) {
		return 1;
	}

	VideoCapture capture;
	capture.open(atoi(av[1]));
	if (!capture.isOpened()) {
		cout << "capture device " << atoi(av[1]) << " failed to open!" << endl;
		return 1;
	}
	if(capture.set(CV_CAP_PROP_FRAME_WIDTH, 640)) {
		cout<<"[main] video width=640"<<endl;
	}
	if(capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480)) {
		cout<<"[main] video height=480"<<endl;
	}

	cout << "following keys do stuff:" << endl;
	cout << "q or escape: quit" << endl;

	Mat frame;
	helper::PerformanceMeasurer PM;

	Detractor detractor;
	for (;;) {
		PM.tic();
		capture >> frame;
		if (frame.empty()) {
			clogI("[main] fps="<<(1.0f/PM.toc())<<endl);
			continue;
		}

		detractor.runOn(frame);
		clogI("[main] fps="<<(1.0f/PM.toc())<<endl);

		imshow("frame", frame);

		char key = (char)waitKey(8);
		switch (key) {
		case 'd':
			detractor.randomInit(frame);
			break;
		case 27:
		case 'q':
			return 0;
		}
	}
	return 0;
}

