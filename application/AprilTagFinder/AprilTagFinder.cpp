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
#include <string>
#include <fstream>
#include <sstream>

#include "OpenCVHelper.h"
#define TAG_DEBUG_PERFORMANCE 0
#define TAG_DEBUG_DRAW 0
#include "apriltag/apriltag.hpp"
#include "apriltag/TagFamilyFactory.hpp"
#include "config.hpp"

using namespace std;
using namespace cv;
using april::tag::UINT64;
using april::tag::TagFamily;
using april::tag::TagFamilyFactory;
using april::tag::TagDetector;
using april::tag::TagDetection;
using helper::ImageSource;
using helper::GConfig;

std::vector< cv::Ptr<TagFamily> > gTagFamilies;
cv::Ptr<TagDetector> gDetector;

struct AprilTagprocessor : public ImageHelper::ImageSource::Processor {
	double tagTextScale;
	int tagTextThickness;
	bool doLog,doRecord;
	bool isPhoto; //whether image source is photo/list or others
	bool useEachValidPhoto; //whether do log for each frame
	std::string outputDir;

	bool undistortImage;
	int hammingThresh;

	cv::Mat K, distCoeffs;
	bool no_distortion;

	AprilTagprocessor() : isPhoto(false), doLog(false), doRecord(false), no_distortion(true) {
		tagTextScale = GConfig::Instance().get<double>("AprilTagprocessor::tagTextScale",1.0f);
		tagTextThickness = GConfig::Instance().get<int>("AprilTagprocessor::tagTextThickness",1);
		useEachValidPhoto = GConfig::Instance().get<bool>("AprilTagprocessor::useEachValidPhoto",false);
		hammingThresh = GConfig::Instance().get<int>("AprilTagprocessor::hammingThresh",0);
		undistortImage = GConfig::Instance().get<int>("AprilTagprocessor::undistortImage",false);
		gDetector->segDecimate = GConfig::Instance().get<bool>("AprilTag::segDecimate",false);
	}

	void loadIntrinsics() {
		{
			double K_[9];
			if(9!=GConfig::Instance().get<double, double[9]>("K",9,K_)) {
				logli("[loadIntrinsics warn] calibration matrix K"
					" not correctly specified in config!");
				this->undistortImage=false;
				this->no_distortion=true;
				return;
			}
			cv::Mat(3,3,CV_64FC1,K_).copyTo(K);
			K/=K_[8];
		}

		{
			double distCoeffs_[5]={0,0,0,0,0};
			if(5!=GConfig::Instance().get<double,double[5]>("distCoeffs",5,distCoeffs_)) {
				logli("[loadIntrinsics warn] distortion coefficients "
					"distCoeffs not correctly specified in config! Assume all zero!");
				for(int i=0; i<5; ++i) distCoeffs_[i]=0;
			}
			double sum=distCoeffs_[0]+distCoeffs_[1]
			+distCoeffs_[2]+distCoeffs_[3]+distCoeffs_[4];
			this->no_distortion=(sum==0);
			if(this->no_distortion) this->undistortImage=false;
			cv::Mat(5,1,CV_64FC1,distCoeffs_).copyTo(distCoeffs);
		}

		logli("[loadIntrinsics] K="<<K);
		logli("[loadIntrinsics] distCoeffs="<<distCoeffs);
	}

	/**
	write the detection in matlab script format:
	tag.id <1x1>, tag.H <3x3>, tag.p <2x4>, tag.c <2x1> [tag.up <2x4>]
	*/
	void writeMatlab(std::ostream& os, TagDetection &dd,
		std::string varname="tag", bool ud=false)
	{
		const cv::Mat H(3,3,CV_64FC1,(void*)dd.homography[0]);
		const cv::Mat p(4,2,CV_64FC1,(void*)dd.p[0]);
		const cv::Mat c(2,1,CV_64FC1,(void*)dd.cxy);
		os<<varname<<".id="<<dd.id<<";\n"
			<<varname<<".hammingDistance="<<dd.hammingDistance<<";\n"
			<<varname<<".familyName='"<<dd.familyName<<"';\n"
			<<varname<<".H="<<H<<";\n"
			<<varname<<".p="<<p<<"';\n"
			<<varname<<".c="<<c<<";"<<std::endl;

		if(!ud) return;

		dd.undistort<double>(this->K, this->distCoeffs);
		os<<varname<<".uH="<<H<<"';"<<std::endl;
		os<<varname<<".up="<<p<<"';"<<std::endl;
		os<<varname<<".uc="<<c<<";"<<std::endl;
	}
/////// Override
	void operator()(cv::Mat& frame) {
		static helper::PerformanceMeasurer PM;
		vector<TagDetection> detections;
		cv::Mat orgFrame;
		frame.copyTo(orgFrame);
		PM.tic();
		if(this->undistortImage) cv::undistort(orgFrame,frame,K,distCoeffs);
		gDetector->process(frame, detections);
		logld("[TagDetector] process time = "<<PM.toc()<<" sec.");

		//visualization
		int nValidDetections=0;
		logld(">>> find: ");
		for(int i=0; i<(int)detections.size(); ++i) {
			TagDetection &dd = detections[i];
			if(dd.hammingDistance>this->hammingThresh) continue;
			++nValidDetections;

			logld("id="<<dd.id<<", hdist="<<dd.hammingDistance<<", rotation="<<dd.rotation);
			cv::putText( frame, dd.toString(), cv::Point(dd.cxy[0],dd.cxy[1]),
				         CV_FONT_NORMAL, tagTextScale, helper::CV_BLUE, tagTextThickness );
			cv::Mat Homo = cv::Mat(3,3,CV_64FC1,dd.homography[0]);
			helper::drawHomography(frame, Homo);
			cv::circle(frame, cv::Point2d(dd.p[0][0],dd.p[0][1]), 3, helper::CV_GREEN, 2);
		}

		//logging results
		if(nValidDetections>0 && (doLog || (isPhoto && useEachValidPhoto) || (!isPhoto && doRecord))) {
			doLog=false;
			static int cnt=0;
			std::string fileid = helper::num2str(cnt, 5);

			//log images
			cv::imwrite(outputDir+"/AprilTagFinder_frame_"+fileid+".png", frame);
			if(!isPhoto) cv::imwrite(outputDir+"/AprilTagFinder_orgframe_"+fileid+".png", orgFrame);

			//log detections
			std::ofstream fs((outputDir+"/AprilTagFinder_log_"+fileid+".m").c_str());
			fs<<"% AprilTagFinder log "<<cnt<<std::endl;
			fs<<"% @ "<<LogHelper::getCurrentTimeString()<<std::endl;
			fs<<"K="<<K<<";"<<std::endl;
			fs<<"distCoeffs="<<distCoeffs<<";"<<std::endl;
			fs<<"tags={};\n"<<std::endl;
			for(int i=0,j=0; i<(int)detections.size(); ++i) {
				TagDetection &dd = detections[i];
				if(dd.hammingDistance>this->hammingThresh) continue;
				++j;//note matlab uses 1-based index

				writeMatlab(fs, dd, "tag", !this->undistortImage && !this->no_distortion);
				fs<<"tags{"<<j<<"}=tag;\n"<<std::endl;
			}

			++cnt;
		}//if doLog

		if(frame.cols>640) {
			cv::resize(frame, frame, cv::Size(640,480));
		}
	}

	void handle(char key) {
		switch (key) {
		case 'd':
			gDetector->segDecimate = !(gDetector->segDecimate);
			logli("[ProcessVideo] gDetector.segDecimate="<<gDetector->segDecimate); break;
		case 'l':
			doLog=true; break;
		case 'r':
			doRecord=true; break;
		case '1':
			LogHelper::GLogControl::Instance().level = LogHelper::LOG_DEBUG; break;
		case '2':
			LogHelper::GLogControl::Instance().level = LogHelper::LOG_INFO; break;
		case 'h':
			cout<<"d: segDecimate\n"
				"l: do log\n"
				"r: do record\n"
				"1: debug output\n"
				"2: info output\n"<<endl; break;
		}
	}

};//end of struct AprilTagprocessor

void usage(const int argc, const char **argv ) {
	cout<< "[usage] " <<argv[0]<<" <url> [TagFamilies ID]"<<endl;
	cout<< "Supported TagFamily ID List:\n";
	for(int i=0; i<(int)TagFamilyFactory::TAGTOTAL; ++i) {
		cout<<"\t"<<april::tag::TagFamilyFactory_SUPPORT_NAME[i]<<" id="<<i<<endl;
	}
	cout<<"Combination of TagFamily ID: 014 (use tagFamily 0, 1 and 4)"<<endl;
	cout<<"default ID: 0"<<endl;
	cout<<"Example ImageSource url:\n";
	cout<<"photo:///home/simbaforrest/Videos/Webcam/seq_UMshort/*\n";
	cout<<"camera://0?w=640?h=480?f=60\n";
	cout<<"video:///home/simbaforrest/Videos/Webcam/keg_april.ogv"<<endl;
#ifdef USE_FLYCAP
	cout<<"pgr://0?v=5?f=4"<<endl;
#endif
}

int main(const int argc, const char **argv )
{
	LogHelper::GLogControl::Instance().level = LogHelper::LOG_INFO;

	const int MIN_ARGS=2, CFG_ARGS_START=3;
	const int URL_POS=1, TAG_FAMILY_ID_POS=2;
	if(argc<MIN_ARGS) {
		usage(argc,argv);
		return -1;
	}

	ConfigHelper::Config& cfg = GConfig::Instance();
	std::string exeDir=helper::getFileDir(argv[0]);
	if(!cfg.load(exeDir+"AprilTagFinder.cfg")) {
		logli("[main] no "<<exeDir<<"AprilTagFinder.cfg file");
	} else {
		logli("[main] loaded "<<exeDir<<"AprilTagFinder.cfg");
	}
	if(argc>CFG_ARGS_START) {
		logli("[main] add/reset config info from command line arguments.");
		cfg.set(argc-CFG_ARGS_START, argv+CFG_ARGS_START);
	}

	cv::Ptr<ImageSource> is = helper::createImageSource(argv[URL_POS]);
	if(is.empty()) {
		tagle("createImageSource failed!");
		return -1;
	}
	is->reportInfo();

	//// create tagFamily
	string tagid("0"); //default tag16h5
	if(argc>TAG_FAMILY_ID_POS) tagid = string(argv[TAG_FAMILY_ID_POS]);
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
	processor.loadIntrinsics();
	processor.isPhoto = is->isClass<helper::ImageSource_Photo>();
	processor.outputDir = cfg.getValAsString("AprilTagprocessor::outputDir",
		is->getSourceDir());
	logli("[main] detection will be logged to outputDir="<<processor.outputDir);
	is->run(processor,-1, false,
		cfg.get<bool>("ImageSource::pause", is->getPause()),
		cfg.get<bool>("ImageSource::loop", is->getLoop()) );

	cout<<"[main] DONE...exit!"<<endl;
	return 0;
}
