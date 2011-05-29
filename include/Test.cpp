#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>
#include <list>
#include <vector>

#include "OpenCVHelper.h"

#include "ESMHelper.h"

using namespace std;
using namespace cv;

const int DESIRED_FTRS = 500;

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

	BriefDescriptorExtractor brief;
	vector<DMatch> matches;

	BruteForceMatcher<Hamming> desc_matcher;

	vector<Point2f> train_pts, query_pts;
	vector<KeyPoint> train_kpts, query_kpts;
	vector<unsigned char> match_mask;

	Mat train_desc, query_desc;

	GridAdaptedFeatureDetector detector;

	Mat H;

	Mat timg;

	ESMTracker esm; //refiner

	bool needInit;

	Detractor() : brief(32),
		detector(new FastFeatureDetector(10, true), DESIRED_FTRS, 4, 4) {
		needInit=true;
		debug=true;
		H = Mat::eye(3, 3, CV_32FC1);

		timg = imread("../data/lena.jpg");
		detector.detect(timg, train_kpts);
		brief.compute(timg, train_kpts, train_desc);

		// The tracking parameters
		int miter = 6,  mprec = 4;
		int posx = 0, posy = 0;
		int sizx = timg.cols, sizy = timg.rows;

		if(!esm.init(timg,posx,posy,sizx,sizy,miter,mprec)) {
			exit(0);
		}
	}

	bool init(Mat& frame) {
		needInit=true;
		Mat gray;
		cvtColor(frame, gray, CV_RGB2GRAY);
		detector.detect(gray, query_kpts);
		brief.compute(gray, query_kpts, query_desc);

		desc_matcher.match(query_desc, train_desc, matches);
		matches2points(train_kpts, query_kpts, matches, train_pts, query_pts);
		if (matches.size() > 7) {
			H = findHomography(Mat(train_pts), Mat(query_pts), match_mask, RANSAC, 4);
			if(countNonZero(Mat(match_mask)) > 15) {
				needInit=false;
			}
		}
		return !needInit;
	}

	bool runOn(Mat& frame) {
		if(needInit) {
			return init(frame);
		}

		needInit = true;

		Mat gray;
		cvtColor(frame, gray, CV_RGB2GRAY);
		detector.detect(gray, query_kpts);
		brief.compute(gray, query_kpts, query_desc);

		if (!train_kpts.empty()) {
			vector<KeyPoint> test_kpts;
			warpKeypoints(H, train_kpts, test_kpts);

			Mat mask = windowedMatchingMask(query_kpts, test_kpts, 50, 50);
			desc_matcher.match(query_desc, train_desc, matches, mask);

			matches2points(train_kpts, query_kpts, matches, train_pts, query_pts);

			if (matches.size() > 5) {
				H = findHomography(Mat(train_pts), Mat(query_pts), match_mask, RANSAC, 4);

				if(countNonZero(Mat(match_mask)) > 15) {
					//ESM refinement
					for(int i=0; i<3; i++) {
						for(int j=0; j<3; j++) {
							esm.T.homog[i*3+j] = H.at<double>(i,j);
						}
					}
					if(esm.run(frame)) {
						for(int i=0; i<3; i++) {
							for(int j=0; j<3; j++) {
								H.at<double>(i,j) = esm.T.homog[i*3+j]/esm.T.homog[8];
							}
						}
					}
					//drawHomo(frame);
					drawPyramid(frame);
					if(debug) {
						drawMatchesRelative(train_kpts, query_kpts, matches, frame, match_mask);
					}
					needInit = false;
				}
			}
		} else {
			Mat out;
			drawKeypoints(gray, query_kpts, out);
			frame = out;
		}

//		train_kpts = query_kpts;
//		query_desc.copyTo(train_desc);
		return !needInit;
	}

	void drawPyramid(Mat& image) {
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

	void drawHomo(Mat& image) {
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

	//Converts matching indices to xy points
	void matches2points(const vector<KeyPoint>& train, const vector<KeyPoint>& query,
	                    const std::vector<cv::DMatch>& matches, std::vector<cv::Point2f>& pts_train,
	                    std::vector<Point2f>& pts_query) {
		pts_train.clear();
		pts_query.clear();
		pts_train.reserve(matches.size());
		pts_query.reserve(matches.size());

		size_t i = 0;

		for (; i < matches.size(); i++) {

			const DMatch & dmatch = matches[i];

			pts_query.push_back(query[dmatch.queryIdx].pt);
			pts_train.push_back(train[dmatch.trainIdx].pt);

		}
	}

	void drawMatchesRelative(const vector<KeyPoint>& train, const vector<KeyPoint>& query,
	                         std::vector<cv::DMatch>& matches, Mat& img, const vector<unsigned char>& mask = vector<
	                                 unsigned char> ()) {
		for (int i = 0; i < (int)matches.size(); i++) {
			if (mask.empty() || mask[i]) {
				Point2f pt_new = query[matches[i].queryIdx].pt;
				Point2f pt_old = train[matches[i].trainIdx].pt;
				Point2f dist = pt_new - pt_old;

				cv::line(img, pt_new, pt_old, Scalar(125, 255, 125), 1);
				cv::circle(img, pt_new, 2, Scalar(255, 0, 125), 1);

			}
		}
	}

	//Takes a descriptor and turns it into an xy point
	void keypoints2points(const vector<KeyPoint>& in, vector<Point2f>& out) {
		out.clear();
		out.reserve(in.size());
		for (size_t i = 0; i < in.size(); ++i) {
			out.push_back(in[i].pt);
		}
	}

	//Takes an xy point and appends that to a keypoint structure
	void points2keypoints(const vector<Point2f>& in, vector<KeyPoint>& out) {
		out.clear();
		out.reserve(in.size());
		for (size_t i = 0; i < in.size(); ++i) {
			out.push_back(KeyPoint(in[i], 1));
		}
	}

	//Uses computed homography H to warp original input points to new planar position
	void warpKeypoints(const Mat& H, const vector<KeyPoint>& in, vector<KeyPoint>& out) {
		vector<Point2f> pts;
		keypoints2points(in, pts);
		vector<Point2f> pts_w(pts.size());
		Mat m_pts_w(pts_w);
		perspectiveTransform(Mat(pts), m_pts_w, H);
		points2keypoints(pts_w, out);
	}
};

int main(int ac, char ** av)
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

	Detractor detractor;
	for (;;) {
		capture >> frame;
		if (frame.empty()) {
			continue;
		}

		detractor.runOn(frame);

		imshow("frame", frame);

		char key = (char)waitKey(2);
		switch (key) {
		case 'd':
			detractor.debug=!detractor.debug;
			break;
		case 27:
		case 'q':
			return 0;
		}
	}
	return 0;
}

