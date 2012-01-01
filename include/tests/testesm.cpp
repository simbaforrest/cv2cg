#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include "Log.h"
#include "OpenCVHelper.h"
#include "SearchHelper.h"

#define ESM_ORIGINAL
#include "esm/ESMInterface.h"

using helper::ImageSource;
using namespace std;
using namespace cv;

Log::Level Log::level = Log::LOG_INFO;

void onMouse( int event, int x, int y, int, void* );

struct Myprocessor : public ImageSource::Processor {
	Mat H;
	Mat gray;
	int cx,cy;
	Mat refimg;
	esm::Tracker refiner;

	inline void operator()(cv::Mat& frame) {
		cvtColor(frame,gray,CV_RGB2GRAY);

		//helper::PerformanceMeasurer PM(1000);
		double rms, ncc;
		refiner(gray, H, ncc, rms);
		//loglni("rms="<<rms<<"| ncc="<<ncc);

		//draw
		static double crns[4][2]={
			{0,0},
			{refimg.cols,0},
			{refimg.cols,refimg.rows},
			{0,refimg.rows}
		};
		helper::drawHomography(frame, H, crns);
	}

	inline void handle(char key) {}


	void selectTarget(cv::Mat& frame) {
		namedWindow("select");
		imshow("select", frame);
		setMouseCallback("select", onMouse, 0);
		waitKey(0);
		destroyWindow("select");
		int s=100;
		refimg = frame(Range(cy-s,cy+s+1), Range(cx-s,cx+s+1)).clone();
		namedWindow("refimg");
		cvtColor(refimg, refimg, CV_RGB2GRAY);
		imshow("refimg", refimg);
		if(!refiner.init(refimg)) {
			loglne("esm::Tracker init fail!");
			exit(-1);
		}
		double h[9] = {1,0,cx-s,0,1,cy-s,0,0,1};
		H = Mat(3,3,CV_64FC1, h).clone();
		cout<<H<<endl;
		refiner.setTermCrit(5,4);
	}
};
Myprocessor processor;

void onMouse( int event, int x, int y, int, void* ) {
	if( event != CV_EVENT_LBUTTONDOWN )
	return;
	processor.cx = x;
	processor.cy = y;
	loglni("(cx,cy)="<<x<<","<<y);
}

int main(int argc, char** argv) {
	if(argc<2) {
		cout<<"[usage] "<<argv[0]<<" <url>"<<endl;
		cout<<"Example:\n";
		cout<<argv[0]<<" photo:///home/simbaforrest/Videos/Webcam/seq_UMshort/*\n";
		cout<<argv[0]<<" camera://0\n";
		cout<<argv[0]<<" video:///home/simbaforrest/Videos/Webcam/keg_april.ogv"<<endl;
		return -1;
	}

	cv::Ptr<ImageSource> is = helper::createImageSource(argv[1]);
	if(is.empty()) {
		loglne("[main] createImageSource failed!");
		return -1;
	}
	is->reportInfo();

	Mat first;
	is->get(first);
	processor.selectTarget(first);

	is->run(processor, -1);

	return 0;
}
