#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include "Log.h"
#include "OpenCVHelper.h"
#include "SearchHelper.h"

#define FAST_DEBUG 1
#include "keg/FastTracker.hpp"

using helper::ImageSource;
using namespace std;
using namespace cv;

Log::Level Log::level = Log::LOG_INFO;

struct Myprocessor : public ImageSource::Processor {
	Mat oldF;
	vector<KeyPoint> oldX;
	fast::TrackParam tp;

	inline void operator()(cv::Mat& frame) {
		Mat newF;
		cv::cvtColor(frame,newF,CV_RGB2GRAY);

		helper::PerformanceMeasurer PM(1000);

		vector<KeyPoint> newX;
		vector<bool> status;
		vector<double> err;
		PM.tic();
		fast::track(oldF, newF, oldX, newX, status, err, tp);
		cout<<"fast::pryTrack time="<<PM.toc()<<endl;

		//draw
		for(int i=0; i<(int)oldX.size(); ++i) {
			KeyPoint& okp=oldX[i];
			KeyPoint& nkp=newX[i];
			circle(frame, okp.pt, 2, CV_RG, -1);
//			rectangle(frame, okp.pt+Point2f(-tp.searchRange,-tp.searchRange), okp.pt+Point2f(tp.searchRange,tp.searchRange), CV_GB);
			if(status[i]) {
				circle(frame, nkp.pt, 2, CV_RED, -1);
				line(frame, okp.pt, nkp.pt, CV_BLUE);
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
	processor.tp.fastLow=400; processor.tp.fastUp=600;
	fast::dynamicThresh(processor.oldF, processor.oldX, processor.tp);
	processor.tp.fastMaxIter=1;
	loglni("tp.fastThresh="<<processor.tp.fastThresh);
	loglni("oldX.size()="<<processor.oldX.size());

	is->run(processor, -1);

	return 0;
}
