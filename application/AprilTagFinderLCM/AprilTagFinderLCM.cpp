/************************************************************************\
  Chen Feng <simbaforrest at gmail dot com>
  Copyright 2014 The University of Michigan.
  All Rights Reserved.
\************************************************************************/

#include <iostream>
#include <string>

#include "AllHelpers.h"
#include "apriltag/apriltag.hpp"
#include "apriltag/TagFamilyFactory.hpp"

using namespace std;
using namespace cv;
using april::tag::UINT64;
using april::tag::TagFamily;
using april::tag::TagFamilyFactory;
using april::tag::TagDetector;
using april::tag::TagDetection;
using helper::ImageSource;
using helper::GConfig;

#include "lcm/lcm-cpp.hpp"
#include "apriltag_lcm/TagPose_t.hpp"
#include "apriltag_lcm/TagPoseArray_t.hpp"
#include "image_lcm/image_t.hpp"

#include <ctime> //for timestamp

std::vector< cv::Ptr<TagFamily> > gTagFamilies;
cv::Ptr<TagDetector> gDetector;

struct AprilTagprocessor : public ImageHelper::ImageSource::Processor {
	double tagTextScale;
	int tagTextThickness;
	bool isPhoto; //whether image source is photo/list or others

	int hammingThresh;

	bool useSeqIntTimestamp;
	lcm::LCM lcm;
	std::string TAG_CHANNEL_NAME;
	int publishImage;
	double publishImageScale;
	std::string IMG_CHANNEL_NAME;

	double K[9];

	virtual ~AprilTagprocessor() {}
	AprilTagprocessor() : isPhoto(false) {
		if(!lcm.good()) {
			tagle("LCM initialization failed!");
			exit(-1);
		}

		ConfigHelper::Config& cfg = GConfig::Instance();
		tagTextScale = cfg.get<double>("AprilTagprocessor:tagTextScale",1.0f);
		tagTextThickness = cfg.get<int>("AprilTagprocessor:tagTextThickness",1);
		hammingThresh = cfg.get<int>("AprilTagprocessor:hammingThresh",0);
		gDetector->segDecimate = cfg.get<bool>("AprilTagprocessor:segDecimate",false);
		useSeqIntTimestamp = cfg.get<bool>("useSeqIntTimestamp",true);
		TAG_CHANNEL_NAME = cfg.get<std::string>("TAG_CHANNEL_NAME","AprilTagFinderLCM.tag");
		IMG_CHANNEL_NAME = cfg.get<std::string>("IMG_CHANNEL_NAME","AprilTagFinderLCM.img");
		publishImage = cfg.get<int>("publishImage",0); //by default: do not publish image
		publishImageScale = cfg.get<double>("publishImageScale",1.0); //by default: no scaling
		if(publishImageScale<=0) publishImageScale=1.0; //must be positive
		
		std::vector<double> K_;
		if(!cfg->exist("K") || 9!=(cfg.getRoot()["K"]>>K_)) {
			logli("[AprilTagFinderLCM.warn] calibration matrix K"
				" not correctly specified in config!");
			K[0] = 525; K[1]=0;   K[2]=320;
			K[3] = 0;   K[4]=525; K[5]=240;
			K[6] = 0;   K[7]=0;   K[8]=1;
		} else {
			for(int i=0; i<9; ++i) K[i] = K_[i]/K_[8];
		}
		logli("[AprilTagFinderLCM] K=\n"<<helper::PrintMat<>(3,3,K));
	}
	
	static void TagDetection2TagPose_t(const TagDetection& td, apriltag_lcm::TagPose_t& out, double K[9])
	{
		out.id = td.id;
		out.hammingDistance = td.hammingDistance;
		std::copy(&(td.p[0][0]), &(td.p[0][0])+8, &(out.p[0][0]));
		std::copy(td.cxy, td.cxy+2, out.cxy);
		std::copy(&(td.homography[0][0]), &(td.homography[0][0])+9, &(out.homography[0][0]));
		helper::RTfromKH(K, &td.homography[0][0], &out.R[0][0], out.t, true);
		out.familyName = td.familyName;
	}

	void Mat2image_t(const cv::Mat& img, image_lcm::image_t& out) const
	{
		assert(img.isContinuous());
		out.w = img.cols;
		out.h = img.rows;
		out.c = img.channels();
		out.len = img.elemSize()*img.total();
		out.I.resize(out.len);
		std::copy(img.data, img.data+out.len, &(out.I[0]));
	}
	
	int64_t getTimestamp() const {
		int64_t ret;
		if(useSeqIntTimestamp) {
			static int64_t cnt=0;
			ret=cnt++;
		} else {
			time_t timer;
			time(&timer);
			ret=(int64_t)timer;
		}
		return ret;
	}

/////// Override
	void operator()(cv::Mat& frame) {
		int64_t timestamp = getTimestamp();
		static helper::PerformanceMeasurer PM;
		vector<TagDetection> detections;
		cv::Mat frame2process;
		if(publishImage==1) {
			frame2process.create(frame.size(), CV_8UC1);
			cv::cvtColor(frame, frame2process, cv::COLOR_BGR2GRAY);
			//UtilHelper::green2float(frame, frame2process);
		} else {
			frame2process = frame;
		}
		PM.tic();
		gDetector->process(frame2process, detections);
		logld("[TagDetector] process time = "<<PM.toc()<<" sec.");

		//visualization & publishing
		apriltag_lcm::TagPoseArray_t msg;
		msg.detections.resize(detections.size());
		int nValidDetections=0;
		logld(">>> find: ");
		for(int i=0; i<(int)detections.size(); ++i) {
			TagDetection &dd = detections[i];
			if(dd.hammingDistance>this->hammingThresh) continue;
			TagDetection2TagPose_t(dd, msg.detections[nValidDetections], K);
			++nValidDetections;

			logld("id="<<dd.id<<", hdist="<<dd.hammingDistance<<", rotation="<<dd.rotation);
			cv::putText( frame, dd.toString(), cv::Point((int)dd.cxy[0],(int)dd.cxy[1]),
				         CV_FONT_NORMAL, tagTextScale, helper::CV_BLUE, tagTextThickness );
			cv::Mat Homo = cv::Mat(3,3,CV_64FC1,dd.homography[0]);
			helper::drawHomography(frame, Homo);
			cv::circle(frame, cv::Point2d(dd.p[0][0],dd.p[0][1]), 3, helper::CV_GREEN, 2);
		}
		if(nValidDetections>0) {
			msg.detections.resize(nValidDetections);
			msg.num_detections = nValidDetections;
			msg.timestamp = timestamp;
			lcm.publish(TAG_CHANNEL_NAME, &msg);
		}
		if(publishImage>0) {
			image_lcm::image_t img_msg;
			cv::Mat frame2publish;
			if (publishImageScale!=1.0) {
				cv::resize(frame2process, frame2publish, cv::Size(),
					publishImageScale, publishImageScale);
			} else {
				frame2publish = frame2process;
			}
			Mat2image_t(frame2publish, img_msg);
			img_msg.timestamp=timestamp;
			lcm.publish(IMG_CHANNEL_NAME, &img_msg);
		}
	}

	void handle(char key) {
		switch (key) {
		case 'd':
			gDetector->segDecimate = !(gDetector->segDecimate);
			logli("[ProcessVideo] gDetector.segDecimate="<<gDetector->segDecimate); break;
		case '1':
			LogHelper::GetOrSetLogLevel(LogHelper::LOG_DEBUG); break;
		case '2':
			LogHelper::GetOrSetLogLevel(LogHelper::LOG_INFO); break;
		case 'h':
			cout<<"d: segDecimate\n"
				"1: debug output\n"
				"2: info output\n"<<endl; break;
		}
	}

};//end of struct AprilTagprocessor

void usage(const int argc, const char **argv ) {
	cout<< "[usage] " <<argv[0]<<" [url] [tagfamiliesID]"<<endl;
	cout<< "Supported TagFamily ID List:\n";
	for(int i=0; i<(int)TagFamilyFactory::TAGTOTAL; ++i) {
		cout<<"\t"<<april::tag::TagFamilyFactory_SUPPORT_NAME[i]<<" id="<<i<<endl;
	}
	cout<<"Combination of TagFamily ID: 014 (use tagFamily 0, 1 and 4)"<<endl;
	cout<<"default tagfamiliesID=0"<<endl;
	cout<<"default url=camera://0"<<endl;
	cout<<"Example ImageSource url:\n";
	cout<<"url=photo:///home/simbaforrest/Videos/Webcam/seq_UMshort/*\n";
	cout<<"url=camera://0?w=640?h=480?f=60\n";
	cout<<"url=video:///home/simbaforrest/Videos/Webcam/keg_april.ogv"<<endl;
#ifdef USE_FLYCAP
	cout<<"url=pgr://0?v=5?f=4"<<endl;
#endif
}

int main(const int argc, const char **argv )
{
	LogHelper::GetOrSetLogLevel(LogHelper::LOG_INFO);

	if (argc > 1) {
		std::string arg1(argv[1]);
		if (arg1 == "-h" || arg1 == "/?" || arg1 == "--help") {
			usage(argc, argv);
			return -1;
		}
	}

	ConfigHelper::Config& cfg = GConfig::Instance();
	if(!cfg.autoLoad("AprilTagFinderLCM.cfg",DirHelper::getFileDir(argv[0]))) {
		logli("[main] no AprilTagFinderLCM.cfg file loaded");
	}
	if(argc>1) {
		cfg.reset(argc-1, argv+1);
	}
	logli("[main] final Config:");
	cfg->print(std::cout);
	logli("");

	cv::Ptr<ImageSource> is = helper::createImageSource(cfg.get("url","camera://0"));
	if(is.empty()) {
		tagle("createImageSource failed!");
		return -1;
	}
	is->reportInfo();

	//// create tagFamily
	string tagid = cfg.get("tagfamiliesID","0"); //defaul Tag16h5
	TagFamilyFactory::create(tagid, gTagFamilies);
	if(gTagFamilies.size()<=0) {
		tagle("create TagFamily failed all! exit...");
		return -1;
	}

	gDetector = new TagDetector(gTagFamilies);
	if(gDetector.empty()) {
		tagle("create TagDetector fail!");
		return -1;
	}

	AprilTagprocessor processor;
	processor.isPhoto = is->isClass<helper::ImageSource_Photo>();
	is->run(processor,-1, false,
		cfg.get<bool>("ImageSource:pause", is->getPause()),
		cfg.get<bool>("ImageSource:loop", is->getLoop()),
		cfg.get<bool>("ImageSource:showImage", is->getShowImage()) );

	cout<<"[main] DONE...exit!"<<endl;
	return 0;
}
