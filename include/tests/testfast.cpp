#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include "Log.h"
#include "OpenCVHelper.h"
#include "SearchHelper.h"

//#define FAST_DEBUG 1
#include "keg/FastTracker.hpp"

using helper::ImageSource;
using namespace std;
using namespace cv;

Log::Level Log::level = Log::LOG_INFO;

struct Myprocessor : public ImageSource::Processor {
	Mat oldF;
	vector<Point2f> oldX;
	fast::TrackParam tp;

	inline void operator()(cv::Mat& frame) {
		Mat newF;
		cv::cvtColor(frame,newF,CV_RGB2GRAY);

		helper::PerformanceMeasurer PM(1000);

		vector<Point2f> newX;
		vector<uchar> status;
		vector<float> err;
		PM.tic();
		fast::track(oldF, newF, oldX, newX, status, err, tp);
		cout<<"[fast::track] total time="<<PM.toc()<<endl;

		//draw
		for(int i=0; i<(int)oldX.size(); ++i) {
			Point2f& op=oldX[i];
			Point2f& np=newX[i];
			circle(frame, op, 2, CV_RG, -1);
//			rectangle(frame, op+Point2f(-tp.searchRange,-tp.searchRange), op+Point2f(tp.searchRange,tp.searchRange), CV_GB);
			if(status[i]) {
				circle(frame, np, 2, CV_RED, -1);
				line(frame, op, np, CV_BLUE);
			}
		}

		swap(oldF,newF);
		swap(oldX,newX);
	}

	inline void handle(char key) {}
};
Myprocessor processor;

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
	
	is->get(processor.oldF);
	cv::cvtColor(processor.oldF,processor.oldF,CV_RGB2GRAY);
	processor.tp.fastMaxIter=100000;
	processor.tp.fastLow=800; processor.tp.fastUp=1000;
	vector<KeyPoint> keys;
	fast::dynamicThresh(processor.oldF, keys, processor.tp);
	fast::key2pt(keys, processor.oldX);
	processor.tp.fastMaxIter=3;
	loglni("tp.fastThresh="<<processor.tp.fastThresh);
	loglni("oldX.size()="<<processor.oldX.size());

	is->run(processor, -1);

	return 0;
}
