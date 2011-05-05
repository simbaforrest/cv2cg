#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include "Log.hxx"
#include "Log.h"
#include "OpenCVHelper.h"

using namespace cv;
using namespace std;

double K[9] = {
	9.1556072719327040e+02, 0., 3.1659567931197148e+02,
	0.,	9.2300384975219845e+02, 2.8310067999512370e+02,
	0., 0., 1.
};
//double stableQ[4] = {1,0,0,0};
//double stableT[3] = {0,0,0};

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

void help()
{
	cout << "\nHot keys: \n"
	     "\tESC - quit the program\n"
	     "\tr - (re)initialize tracking\n" << endl;
}

//LK tracker
struct LKTracker {
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
	Mat H; //current homography that maps tpts -> npts

	Mat timg;

	vector<unsigned char> match_mask;

	Ptr<FeatureDetector> detector; //FIXME, it should be recognizer's job
	Ptr<DescriptorExtractor> descriptor;
	Ptr<DescriptorMatcher> matcher;
	vector<KeyPoint> tkeys;
	Mat tdes;

	LKTracker(int winW=10, int winH=10,
	          int termIter=20, int termEps=0.03,
	          int maxlevel=3, double lambda=0.5,
	          double ransacT=3, int drawT=15) :
		termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,termIter,termEps),
		winSize(winW,winH), maxLevel(maxlevel), derivedLambda(lambda),
		ransacThresh(ransacT), drawTresh(drawT) {
		detector=FeatureDetector::create("SURF");
		descriptor=DescriptorExtractor::create("SURF");
		matcher=DescriptorMatcher::create("FlannBased");

		//load template
		timg = imread("lena.jpg");
		detector->detect(timg, tkeys);
		descriptor->compute(timg, tkeys, tdes);
	}

	inline bool init(Mat& nframe) {
//		goodFeaturesToTrack(nframe, npts, MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
		vector<KeyPoint> keys;
		Mat des;
		vector<DMatch> matches;
		detector->detect(nframe, keys);
		descriptor->compute(nframe, keys, des);
		matcher->clear();
		matcher->match(des, tdes, matches);

		npts.resize(matches.size());
		tpts.resize(matches.size());
		for(int i=0; i<matches.size(); ++i) {
			const DMatch& m = matches[i];
			npts[i] = keys[m.queryIdx].pt;
			tpts[i] = tkeys[m.trainIdx].pt;
		}
		H = findHomography(Mat(tpts), Mat(npts),
		                   match_mask, RANSAC, ransacThresh);
		if(match_mask.size()>drawTresh) {
			cornerSubPix(nframe, npts, winSize, Size(-1,-1), termcrit);
			//H = Mat::eye(3,3,CV_64F);

//			tpts.resize(npts.size());
//			std::copy(npts.begin(), npts.end(), tpts.begin());
			opts.resize(npts.size());
			std::copy(npts.begin(), npts.end(), opts.begin());
		}
		return match_mask.size()>drawTresh;
	}

	//tracking, old frame(oframe), new frame(nframe)
	inline int operator()(Mat& oframe, Mat& nframe, Mat& image) {
		status.clear();
		err.clear();

		calcOpticalFlowPyrLK(oframe, nframe, opts, npts,
		                     status, err, winSize,
		                     maxLevel, termcrit, derivedLambda);

		for(int i=0; i<(int)npts.size(); ++i) {
			if( !status[i] ) {
				npts[i] = Point2f(0,0); //to make these points rejected by RANSAC
			}
		}

		if(update()) {
			draw(image);
		}

		std::swap(npts, opts);
		return 1;
	}

	inline bool update() {
		if(tpts.size()<2*drawTresh) {
			return false;
		}
		match_mask.clear();
		H = findHomography(Mat(tpts), Mat(npts),
		                   match_mask, RANSAC, ransacThresh);
		Mat nptsmat(npts);
		///////VERY IMPORTANT STEP, STABLIZE!!!!!!!
		perspectiveTransform(Mat(tpts), nptsmat, H);
		//this will slow down the fps
		//cornerSubPix(gray, points[1], winSize, Size(-1,-1), termcrit);
		return countNonZero(Mat(match_mask)) > drawTresh;
	}

	inline void draw(Mat& image) {
		for(int i=0; i<(int)npts.size(); ++i) {
			if( status[i] ) {
				circle(image, npts[i], 3, CV_GREEN, -1, 8);
				line(image, npts[i], opts[i], CV_BLUE );
			}
			if(match_mask[i]) {
				circle( image, npts[i], 2, CV_RED, -1, 7);
			}
		}

		double crns[8][3] = {
			{0, 0, 0},
			{image.cols, 0, 0},
			{image.cols, image.rows, 0},
			{0, image.rows, 0},
			{image.cols*0.4, image.rows*0.4, image.rows*0.5},
			{image.cols*0.6, image.rows*0.4, image.rows*0.5},
			{image.cols*0.6, image.rows*0.6, image.rows*0.5},
			{image.cols*0.4, image.rows*0.6, image.rows*0.5},
		};
		//draw homo
		const Mat_<double>& mH = H;
		vector<Point2f> corners(4);
		for(int i = 0; i < 4; i++ ) {
			Point2f pt((float)(i == 0 || i == 3 ? 0 : image.cols),
			           (float)(i <= 1 ? 0 : image.rows));
			double w = 1./(mH(2,0)*pt.x + mH(2,1)*pt.y + mH(2,2));
			corners[i] =
			    Point2f((float)((mH(0,0)*pt.x + mH(0,1)*pt.y + mH(0,2))*w),
			            (float)((mH(1,0)*pt.x + mH(1,1)*pt.y + mH(1,2))*w));
		}
		for(int i = 0; i < 4; ++i) {
			Point r1 = corners[i%4];
			Point r2 = corners[(i+1)%4];
			line( image, r1, r2, CV_GREEN, 3 );
		}
		line(image, corners[0], corners[2], CV_BLACK, 3);
		line(image, corners[1], corners[3], CV_BLACK, 3);
		//homo to P
		double Homo[9];
		std::copy(H.begin<double>(), H.end<double>(), Homo);
		double R[9],T[3],P[12],tmpQ[4];
		CameraHelper::RTfromKH(K,Homo,R,T);
//		RotationHelper::mat2quat(R,tmpQ);
//		helper::stablize(4,tmpQ,stableQ);
//		RotationHelper::quat2mat(stableQ,R);
//		helper::stablize(3,T,stableT);
		R[2]*=-1, R[5]*=-1, R[8]*=-1;
		CameraHelper::compose(K,R,T,P,false);
		double p[8][2];
		for(int i=0; i<8; ++i) {
			CameraHelper::project(P,crns[i],p[i]);
		}
		for(int i=0; i<3; ++i) {
			for(int j=i*4; j<4+i*4; ++j) {
				Point r1(p[lines[j][0]][0],p[lines[j][0]][1]);
				Point r2(p[lines[j][1]][0],p[lines[j][1]][1]);
				line( image, r1, r2, colors[i], 3 );
			}
		}
		cout<<"R=\n"<<helper::PrintMat<>(3,3,R)<<endl;
		cout<<"T=\n"<<helper::PrintMat<>(1,3,T)<<endl;
		cout<<"norm(T)="<<sqrt(T[0]*T[0]+T[1]*T[1]+T[2]*T[2])<<endl;
	}
};

int main( int argc, char **argv )
{
	VideoCapture cap;

	if( argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0]))) {
		cap.open(argc == 2 ? argv[1][0] - '0' : 0);
	} else if( argc == 2 ) {
		cap.open(argv[1]);
	}

	if( !cap.isOpened() ) {
		cout << "Could not initialize capturing...\n";
		return 0;
	}

	help();

	string winname("LKTracker");
	namedWindow( winname, 1 );

	Mat gray, prevGray, image;
	bool needToInit = false;

	LKTracker tracker;

	for(;;) {
		Mat frame;
		cap >> frame;
		if( frame.empty() ) {
			break;
		}

		frame.copyTo(image);
		cvtColor(image, gray, CV_BGR2GRAY);

		if( needToInit ) {
			cout<<"initing..."<<endl;
			needToInit=!tracker.init(gray);
			cout<<"...inited"<<endl;
		} else if( !tracker.opts.empty() ) {
			cout<<"tracking..."<<endl;
			if(prevGray.empty()) {
				gray.copyTo(prevGray);
			}
			tracker(prevGray, gray, image);
			cout<<"...tracked"<<endl;
		}

		imshow(winname, image);

		char c = (char)waitKey(10);
		if( c == 27 ) {
			break;
		}
		switch( c ) {
		case 'r':
			needToInit = true;
			break;
		default:
			;
		}

		swap(prevGray, gray);
	}

	return 0;
}
