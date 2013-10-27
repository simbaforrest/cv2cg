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
#include "Log.h"
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

std::vector< cv::Ptr<TagFamily> > tagFamilies;
cv::Ptr<TagDetector> detector;

static void write(std::ofstream& fs, const TagDetection &dd) {
	fs<<dd.id<<std::endl;
	fs<<helper::PrintMat<>(3,3,dd.homography[0])<<std::endl;
	fs<<helper::PrintMat<>(1,2,dd.p[3]);
	fs<<helper::PrintMat<>(1,2,dd.p[2]);
	fs<<helper::PrintMat<>(1,2,dd.p[1]);
	fs<<helper::PrintMat<>(1,2,dd.p[0])<<std::endl;
	fs<<helper::PrintMat<>(1,2,dd.cxy)<<std::endl;
}

struct AprilTagprocessor : public ImageHelper::ImageSource::Processor {
	bool takePhoto;
	int photoCnt;
	bool grabImage;
	bool takePhotoContinue;
	bool grabImageContinue;
	bool doNotWritePhoto;
	double tagTextScale;
	int tagTextThickness;
	AprilTagprocessor() : takePhoto(false), photoCnt(0), grabImage(false) {
		takePhotoContinue = GConfig::Instance().get<bool>("AprilTagprocessor::takePhotoContinue",false);
		grabImageContinue = GConfig::Instance().get<bool>("AprilTagprocessor::grabImageContinue",false);
		doNotWritePhoto = GConfig::Instance().get<bool>("AprilTagprocessor::doNotWritePhoto",false);
		tagTextScale = GConfig::Instance().get<double>("AprilTagprocessor::tagTextScale",1.0f);
		tagTextThickness = GConfig::Instance().get<int>("AprilTagprocessor::tagTextThickness",1);
	}
/////// Override
	void operator()(cv::Mat& frame) {
		static helper::PerformanceMeasurer PM;
		vector<TagDetection> detections;
		PM.tic();
		detector->process(frame, detections);
		logld<<"[TagDetector] process time = "<<PM.toc()<<" sec.";

		std::ofstream fs;
		if((takePhoto && detections.size()!=0)||takePhotoContinue) {
			std::string id = helper::num2str(photoCnt, 5);
			fs.open((id+".txt").c_str());
			if(!fs.is_open()) {
				logle<<"can not open: "<<id<<".txt";
				exit(-1);
			} else {
				logli<<"wrote detection: "<<id<<".txt";
			}
			if(!doNotWritePhoto) {
				if(cv::imwrite(id+".png", frame)) {
					logli<<"took photo: "<<id<<".png";
					takePhoto=false;
					++photoCnt;
				} else {
					logle<<"can not save photo: "<<id<<".png";
					exit(-1);
				}
			} else {
				++photoCnt;
			}
		}

		if(grabImage || grabImageContinue) {
			static int grabCnt=0;
			std::string id = helper::num2str(grabCnt, 2);
			if(cv::imwrite(id+".png", frame)) {
				logli<<"grab image: "<<id<<".png";
				++grabCnt;
				grabImage=false;
			} else {
				logle<<"can not grab image...exit";
				exit(-1);
			}
		}

		if(fs.is_open()) {
			fs<<(int)detections.size()<<std::endl;
		}
		logld<<">>> find: ";
		for(int id=0; id<(int)detections.size(); ++id) {
			TagDetection &dd = detections[id];
			//if(dd.hammingDistance>0) continue; //very strict!
			logld<<"id="<<dd.id<<", hdist="<<dd.hammingDistance<<", rotation="<<dd.rotation;

			if(fs.is_open()) {
				write(fs, dd);
			}

			cv::putText( frame, dd.toString(), cv::Point(dd.cxy[0],dd.cxy[1]),
				         CV_FONT_NORMAL, tagTextScale, helper::CV_BLUE, tagTextThickness );

			cv::Mat Homo = cv::Mat(3,3,CV_64FC1,dd.homography[0]);
			static double crns[4][2]={
				{-1, -1},
				{ 1, -1},
				{ 1,  1},
				{-1,  1}
			};
			helper::drawHomography(frame, Homo, crns);
			cv::circle(frame, cv::Point2d(dd.p[0][0],dd.p[0][1]), 3, helper::CV_GREEN, 2);
		}
		logld;
		if(fs.is_open()) {
			fs.close();
		}

		if(frame.cols>640) {
			cv::resize(frame, frame, cv::Size(640,480));
		}
	}

	void handle(char key) {
		switch (key) {
		case 'd':
			detector->segDecimate = !(detector->segDecimate);
			logli<<"[ProcessVideo] detector.segDecimate="<<detector->segDecimate; break;
		case 't':
			takePhoto=true; break;
		case 'g':
			grabImage=true; break;
		case '1':
			LogHelper::GLogControl::Instance().level = LogHelper::LOG_DEBUG; break;
		case '2':
			LogHelper::GLogControl::Instance().level = LogHelper::LOG_INFO; break;
		case 'h':
			cout<<"d: segDecimate\n"
				"t: takePhoto\n"
				"g: grabImage\n"
				"1: debug output\n"
				"2: info output\n"<<endl; break;
		}
	}

};//end of struct AprilTagprocessor

void usage( int argc, char **argv ) {
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

int main( int argc, char **argv )
{
	LogHelper::GLogControl::Instance().level = LogHelper::LOG_INFO;

	if(argc<2) {
		usage(argc,argv);
		return -1;
	}

	ConfigHelper::Config& cfg = GConfig::Instance();
	std::string exeDir=helper::getFileDir(argv[0]);
	if(!cfg.load(exeDir+"AprilTagFinder.cfg")) {
		flogli("[main] no "<<exeDir<<"AprilTagFinder.cfg file");
	} else {
		flogli("[main] loaded "<<exeDir<<"AprilTagFinder.cfg");
	}

	cv::Ptr<ImageSource> is = helper::createImageSource(argv[1]);
	if(is.empty()) {
		tagle<<"createImageSource failed!";
		return -1;
	}
	is->reportInfo();

	//// create tagFamily
	string tagid("0"); //default tag16h5
	if(argc>2) tagid = string(argv[2]);
	for(int i=0; i<(int)tagid.size(); ++i) {
		const char curchar[] = {tagid[i],'\0'};
		unsigned int curid = atoi(curchar);//atoi works on an array of char, not on a single char!!
		cv::Ptr<TagFamily> tagFamily = TagFamilyFactory::create(curid);
		if(tagFamily.empty()) {
			tagle<<"create TagFamily "<<curid<<" fail, skip!";
			continue;
		}
		tagFamilies.push_back(tagFamily);
	}
	if(tagFamilies.size()<=0) {
		tagle<<"create TagFamily failed all! exit...";
		return -1;
	}

	detector = new TagDetector(tagFamilies);
	if(detector.empty()) {
		tagle<<"create TagDetector fail!";
		return -1;
	}

	AprilTagprocessor processor;
	is->run(processor,-1, false,
		cfg.get<bool>("ImageSource::pause", is->getPause()),
		cfg.get<bool>("ImageSource::loop", is->getLoop()) );

	cout<<"[main] DONE...exit!"<<endl;
	return 0;
}
